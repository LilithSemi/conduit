//! SF32LB52 SPI1/SPI2 polling master driver.

const Mmio = @import("../mmio.zig");
const Spi = @import("../device/spi.zig");
const pac = @import("sf32lb52_regs");
const regs = pac.spi1;

pub const Instance = enum(usize) {
    spi1 = pac.spi1.base_address,
    spi2 = pac.spi2.base_address,
};

pub const Options = struct {
    /// Hardware CLK_DIV value. One selects the undivided clock path.
    prescaler: u7 = 2,
    mode: u2 = 0,
    idle_byte: u8 = 0xff,
    poll_limit: u32 = 100_000,
};

pub const ConfigError = error{ InvalidPrescaler, InvalidPollLimit };
pub const Error = error{ Timeout, Overrun, Underrun };

pub const Sf32lb52Spi = struct {
    mmio: Mmio,
    idle_byte: u8,
    poll_limit: u32,
    last_error: ?Error = null,

    pub fn transferChecked(self: *Sf32lb52Spi, tx: []const u8, rx: []u8) Error!void {
        self.last_error = null;
        const len = @max(tx.len, rx.len);
        if (len == 0) return;
        self.enable(true);
        defer self.enable(false);
        for (0..len) |index| {
            self.waitFor(regs.status.tnf_mask) catch |err| return self.fail(err);
            self.mmio.write(u8, regs.data.offset, if (index < tx.len) tx[index] else self.idle_byte);
            self.waitFor(regs.status.rne_mask) catch |err| return self.fail(err);
            const byte = self.mmio.read(u8, regs.data.offset);
            if (index < rx.len) rx[index] = byte;
        }
        self.waitIdle() catch |err| return self.fail(err);
    }

    pub fn transfer(self: *Sf32lb52Spi, tx: []const u8, rx: []u8) void {
        self.transferChecked(tx, rx) catch {};
    }

    pub fn spi(self: *Sf32lb52Spi) Spi {
        return Spi.from(self);
    }

    pub fn lastError(self: *const Sf32lb52Spi) ?Error {
        return self.last_error;
    }

    pub fn configureDma(self: *Sf32lb52Spi, tx: bool, rx: bool) void {
        self.mmio.write(u32, regs.fifo_ctrl.offset, (if (tx) regs.fifo_ctrl.tsre_mask else 0) |
            (if (rx) regs.fifo_ctrl.rsre_mask else 0));
    }

    fn enable(self: *Sf32lb52Spi, enabled: bool) void {
        var value = self.mmio.read(u32, regs.top_ctrl.offset);
        if (enabled) value |= regs.top_ctrl.sse_mask else value &= ~regs.top_ctrl.sse_mask;
        self.mmio.write(u32, regs.top_ctrl.offset, value);
    }

    fn waitFor(self: *Sf32lb52Spi, flag: u32) Error!void {
        var remaining = self.poll_limit;
        while (remaining != 0) : (remaining -= 1) {
            const status = self.mmio.read(u32, regs.status.offset);
            if (status & regs.status.ror_mask != 0) return error.Overrun;
            if (status & regs.status.tur_mask != 0) return error.Underrun;
            if (status & flag != 0) return;
        }
        return error.Timeout;
    }

    fn waitIdle(self: *Sf32lb52Spi) Error!void {
        var remaining = self.poll_limit;
        while (remaining != 0) : (remaining -= 1) {
            if (self.mmio.read(u32, regs.status.offset) & regs.status.bsy_mask == 0) return;
        }
        return error.Timeout;
    }

    fn fail(self: *Sf32lb52Spi, err: Error) Error {
        self.last_error = err;
        return err;
    }
};

pub fn bind(mmio: Mmio, options: Options) ConfigError!Sf32lb52Spi {
    if (options.prescaler == 0) return error.InvalidPrescaler;
    if (options.poll_limit == 0) return error.InvalidPollLimit;
    mmio.write(u32, regs.top_ctrl.offset, 0);
    mmio.write(u32, regs.inte.offset, 0);
    mmio.write(u32, regs.fifo_ctrl.offset, 0);
    const clock = regs.clk_ctrl.clk_en_mask |
        if (options.prescaler == 1) regs.clk_ctrl.clk_sel_mask else @as(u32, options.prescaler);
    mmio.write(u32, regs.clk_ctrl.offset, clock);
    const control = (@as(u32, 7) << regs.top_ctrl.dss_shift) |
        (if (options.mode & 1 != 0) regs.top_ctrl.sph_mask else 0) |
        (if (options.mode & 2 != 0) regs.top_ctrl.spo_mask else 0);
    mmio.write(u32, regs.top_ctrl.offset, control);
    mmio.write(u32, regs.triwire_ctrl.offset, 0);
    // Latch polarity before the first transfer, as required by the SiFli HAL.
    mmio.write(u32, regs.top_ctrl.offset, control | regs.top_ctrl.sse_mask);
    mmio.write(u32, regs.top_ctrl.offset, control);
    return .{ .mmio = mmio, .idle_byte = options.idle_byte, .poll_limit = options.poll_limit };
}

pub fn fromBase(instance: Instance, options: Options) ConfigError!Sf32lb52Spi {
    return bind(Mmio.direct(@backingInt(instance)), options);
}
