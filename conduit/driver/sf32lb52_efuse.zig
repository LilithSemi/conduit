//! SF32LB52 one-time-programmable fuse controller.
//!
//! Pebble firmware consumes identification and calibration data, so this API is
//! deliberately read-only. Fuse programming is irreversible and additionally
//! needs board-level voltage sequencing that does not belong in a routine data
//! access interface.

const Mmio = @import("../mmio.zig");
const pac = @import("sf32lb52_regs");
const regs = pac.efusec;
const pmuc_regs = pac.pmuc;

pub const bank_count = 4;
pub const bank_size = 32;
pub const capacity = bank_count * bank_size;

pub const Error = error{ InvalidClock, OutOfRange, Timeout };

pub const Delay = struct {
    ctx: ?*anyopaque,
    wait_fn: *const fn (ctx: ?*anyopaque, microseconds: u32) void,

    pub fn wait(self: Delay, microseconds: u32) void {
        self.wait_fn(self.ctx, microseconds);
    }

    pub fn from(impl: anytype) Delay {
        const P = @TypeOf(impl);
        const Adapter = struct {
            fn wait(ctx: ?*anyopaque, microseconds: u32) void {
                const self: P = @ptrCast(@alignCast(ctx));
                self.waitMicroseconds(microseconds);
            }
        };
        return .{ .ctx = impl, .wait_fn = Adapter.wait };
    }
};

pub const Options = struct {
    pclk_hz: u32,
    poll_limit: u32 = 12_288_000,
};

pub const Sf32lb52Efuse = struct {
    mmio: Mmio,
    pmuc_mmio: Mmio,
    delay: Delay,
    poll_limit: u32,

    /// Read any byte range in the four 256-bit banks. A range may cross bank
    /// boundaries; each touched bank is latched with its own read transaction.
    pub fn read(self: *Sf32lb52Efuse, byte_offset: usize, output: []u8) Error!void {
        if (byte_offset > capacity or output.len > capacity - byte_offset) return error.OutOfRange;
        if (output.len == 0) return;

        const original_vout = self.pmuc_mmio.read(u32, pmuc_regs.hpsys_vout.offset);
        const old_level = original_vout & pmuc_regs.hpsys_vout.vout_mask;
        const raised_level = @max(@as(u32, 0xe), @min(@as(u32, 0xf), old_level + 3));
        self.pmuc_mmio.write(u32, pmuc_regs.hpsys_vout.offset, raised_level);
        self.delay.wait(20);
        defer self.pmuc_mmio.write(u32, pmuc_regs.hpsys_vout.offset, original_vout);

        var source = byte_offset;
        var destination: usize = 0;
        while (destination < output.len) {
            const bank: u2 = @intCast(source / bank_size);
            try self.latchBank(bank);

            const in_bank = source % bank_size;
            const count = @min(output.len - destination, bank_size - in_bank);
            var i: usize = 0;
            while (i < count) : (i += 1) {
                const byte_index = in_bank + i;
                const word = self.mmio.read(u32, regs.bank0_data0.offset + @as(usize, bank) * bank_size + (byte_index / 4) * 4);
                const shift: u5 = @intCast((byte_index % 4) * 8);
                output[destination + i] = @truncate(word >> shift);
            }
            source += count;
            destination += count;
        }
    }

    pub fn interruptPending(self: *const Sf32lb52Efuse) bool {
        return self.mmio.read(u32, regs.sr.offset) & regs.sr.done_mask != 0;
    }

    pub fn enableInterrupt(self: *Sf32lb52Efuse, enabled: bool) void {
        var control = self.mmio.read(u32, regs.cr.offset);
        if (enabled) control |= regs.cr.ie_mask else control &= ~regs.cr.ie_mask;
        self.mmio.write(u32, regs.cr.offset, control);
    }

    pub fn acknowledgeInterrupt(self: *Sf32lb52Efuse) void {
        self.mmio.write(u32, regs.sr.offset, regs.sr.done_mask);
    }

    fn latchBank(self: *Sf32lb52Efuse, bank: u2) Error!void {
        // DONE is write-one-to-clear. Clear a stale completion before starting
        // so it cannot make a new transaction appear to finish immediately.
        self.mmio.write(u32, regs.sr.offset, regs.sr.done_mask);
        const interrupt_enable = self.mmio.read(u32, regs.cr.offset) & regs.cr.ie_mask;
        const select = interrupt_enable | (@as(u32, bank) << @intCast(regs.cr.banksel_shift));
        self.mmio.write(u32, regs.cr.offset, select);
        self.mmio.write(u32, regs.cr.offset, select | regs.cr.en_mask);

        var remaining = self.poll_limit;
        while (remaining != 0) : (remaining -= 1) {
            if (self.mmio.read(u32, regs.sr.offset) & regs.sr.done_mask != 0) {
                self.acknowledgeInterrupt();
                return;
            }
        }
        return error.Timeout;
    }
};

/// Derive the controller timing fields from the HPSYS peripheral clock using
/// the limits and formulas in SiFli's SF32LB52 HAL.
pub fn timingFor(pclk_hz: u32) Error!u32 {
    if (pclk_hz == 0 or pclk_hz > 120_000_000) return error.InvalidClock;

    const pclk: u64 = pclk_hz;
    const thpck = pclk * 20 / 1_000_000_000 + 1;
    const thrck = pclk * 500 / 1_000_000_000 + 1;
    const tckhp = @max(@as(u64, 1), (pclk * 10 + 500_000) / 1_000_000);
    if (thpck > 0x7 or thrck > 0x7f or tckhp > 0x7ff) return error.InvalidClock;

    return @as(u32, @intCast(thrck)) |
        (@as(u32, @intCast(thpck)) << @intCast(regs.timr.thpck_shift)) |
        (@as(u32, @intCast(tckhp)) << @intCast(regs.timr.tckhp_shift));
}

pub fn bind(mmio: Mmio, pmuc_mmio: Mmio, delay: Delay, options: Options) Error!Sf32lb52Efuse {
    var controller = Sf32lb52Efuse{
        .mmio = mmio,
        .pmuc_mmio = pmuc_mmio,
        .delay = delay,
        .poll_limit = options.poll_limit,
    };
    controller.mmio.write(u32, regs.timr.offset, try timingFor(options.pclk_hz));
    return controller;
}

pub fn fromBase(delay: Delay, options: Options) Error!Sf32lb52Efuse {
    return bind(Mmio.direct(regs.base_address), Mmio.direct(pmuc_regs.base_address), delay, options);
}

test "timing matches the SF32LB52 HAL at 48 MHz" {
    const std = @import("std");
    try std.testing.expectEqual(@as(u32, 25 | (1 << 7) | (480 << 10)), try timingFor(48_000_000));
    try std.testing.expectError(error.InvalidClock, timingFor(120_000_001));
}
