//! SiFli SF32LB52 true-random seed source and PRNG output engine.
//!
//! The block conditions an internal oscillator into a 256-bit seed and then
//! produces random output in eight-word batches. Polling is deliberately
//! bounded so a failed oscillator or locked PRNG cannot hang firmware.

const Mmio = @import("../mmio.zig");
const Entropy = @import("../device/entropy.zig");

pub const Sf32lb52Trng = struct {
    mmio: Mmio,
    poll_limit: usize,

    pub const base_address: u64 = 0x5000f000;
    pub const default_poll_limit: usize = 100_000;

    const CTRL = 0x00;
    const STAT = 0x04;
    const CFG = 0x08;
    const IRQ = 0x0c;
    const RAND_NUM0 = 0x30;

    const ctrl_seed_start: u32 = 1 << 0;
    const ctrl_random_start: u32 = 1 << 1;
    const ctrl_seed_stop: u32 = 1 << 2;
    const ctrl_random_stop: u32 = 1 << 3;
    const stat_seed_valid: u32 = 1 << 1;
    const stat_random_valid: u32 = 1 << 3;
    const irq_prng_lockup: u32 = 1 << 2;

    fn checkHealth(self: Sf32lb52Trng) Entropy.Error!void {
        if ((self.mmio.read(u32, IRQ) & irq_prng_lockup) != 0)
            return error.HealthTestFailed;
    }

    fn waitFor(self: Sf32lb52Trng, mask: u32) Entropy.Error!void {
        var remaining = self.poll_limit;
        while (remaining != 0) : (remaining -= 1) {
            try self.checkHealth();
            if ((self.mmio.read(u32, STAT) & mask) != 0) return;
        }
        return error.Timeout;
    }

    fn updateControl(self: Sf32lb52Trng, start_bit: u32) void {
        var control = self.mmio.read(u32, CTRL);
        control &= ~(ctrl_seed_stop | ctrl_random_stop);
        control |= start_bit;
        self.mmio.write(u32, CTRL, control);
    }

    /// Enable automatic clock gating and start generation of an internal seed.
    pub fn initialize(self: Sf32lb52Trng) void {
        self.mmio.write(u32, CFG, self.mmio.read(u32, CFG) | 1);
        self.updateControl(ctrl_seed_start);
    }

    fn ensureSeed(self: Sf32lb52Trng) Entropy.Error!void {
        try self.checkHealth();
        if ((self.mmio.read(u32, STAT) & stat_seed_valid) == 0) {
            self.initialize();
            try self.waitFor(stat_seed_valid);
        }
    }

    fn generateBatch(self: Sf32lb52Trng, out: []u8) Entropy.Error!usize {
        self.updateControl(ctrl_random_start);
        try self.waitFor(stat_random_valid);

        const count = @min(out.len, 32);
        var written: usize = 0;
        var word_index: usize = 0;
        while (written < count) : (word_index += 1) {
            var word = self.mmio.read(u32, RAND_NUM0 + word_index * 4);
            var byte_index: usize = 0;
            while (byte_index < 4 and written < count) : (byte_index += 1) {
                out[written] = @truncate(word);
                word >>= 8;
                written += 1;
            }
        }
        return written;
    }

    pub fn fill(self: Sf32lb52Trng, out: []u8) Entropy.Error!void {
        if (out.len == 0) return;
        try self.ensureSeed();
        var offset: usize = 0;
        while (offset < out.len) offset += try self.generateBatch(out[offset..]);
    }

    pub fn entropy(self: *Sf32lb52Trng) Entropy {
        return Entropy.from(self);
    }
};

pub fn bind(mmio: Mmio, poll_limit: usize) Sf32lb52Trng {
    return .{ .mmio = mmio, .poll_limit = poll_limit };
}

pub fn fromBase() Sf32lb52Trng {
    return bind(Mmio.direct(Sf32lb52Trng.base_address), Sf32lb52Trng.default_poll_limit);
}
