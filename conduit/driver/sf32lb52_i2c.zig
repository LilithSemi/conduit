//! SF32LB52 I2C1/2/3/4 master driver.
//!
//! Transfers are byte-stepped and polling, with bounded waits, repeated START
//! for combined write/read operations, 7- and 10-bit addresses, and explicit
//! hardware error reporting. DMA channel programming belongs to the DMAC1
//! driver; this controller remains fully usable without DMA.

const Mmio = @import("../mmio.zig");
const I2c = @import("../device/i2c.zig");
const pac = @import("sf32lb52_regs");
const regs = pac.i2_c1;

pub const Instance = enum(usize) {
    i2c1 = pac.i2_c1.base_address,
    i2c2 = pac.i2_c2.base_address,
    i2c3 = pac.i2_c3.base_address,
    i2c4 = pac.i2_c4.base_address,
};

pub const Options = struct {
    clock_hz: u32 = 48_000_000,
    bus_hz: u32 = 400_000,
    digital_filter_cycles: u3 = 0,
    poll_limit: u32 = 100_000,
};

pub const ConfigError = error{
    InvalidClock,
    InvalidBusFrequency,
    InvalidPollLimit,
    UnrepresentableTiming,
};

pub const Error = error{
    InvalidAddress,
    BusBusy,
    Timeout,
    Nack,
    ArbitrationLost,
    BusError,
    Overflow,
    Underflow,
};

pub const Sf32lb52I2c = struct {
    mmio: Mmio,
    poll_limit: u32,
    last_error: ?Error = null,

    pub fn init(self: *Sf32lb52I2c, options: Options) ConfigError!void {
        const timing = try calculateTiming(options);
        self.mmio.write(u32, regs.cr.offset, 0);
        var lcr = self.mmio.read(u32, regs.lcr.offset);
        lcr &= ~(regs.lcr.slv_mask | regs.lcr.flv_mask);
        lcr |= regs.lcr.slv_mask;
        lcr |= @as(u32, timing.flv) << flv_shift;
        self.mmio.write(u32, regs.lcr.offset, lcr);
        self.mmio.write(u32, regs.wcr.offset, timing.wait_count);
        self.mmio.write(u32, regs.ier.offset, 0);
        self.mmio.write(u32, regs.sar.offset, 0);

        // Fast/Fast+ timing, SCL output enabled, master mode. IUE is used for
        // slave operation and deliberately remains clear, matching SiFli HAL.
        const cr = @as(u32, 1) |
            regs.cr.scle_mask |
            (@as(u32, options.digital_filter_cycles) << dnf_shift);
        self.mmio.write(u32, regs.cr.offset, cr);
        self.poll_limit = options.poll_limit;
        self.last_error = null;
    }

    pub fn transferChecked(self: *Sf32lb52I2c, address: u16, write_bytes: []const u8, read_bytes: []u8) Error!void {
        self.last_error = null;
        if (address > 0x3ff) return self.fail(error.InvalidAddress);
        self.waitForIdle() catch |err| return self.fail(err);
        self.clearStatus(clearable_status);

        const ten_bit = address > 0x7f;
        if (ten_bit) {
            self.sendByte(tenBitHeader(address, false), regs.tcr.start_mask | regs.tcr.tb_mask) catch |err| return self.abort(err);
            const low: u8 = @truncate(address);
            const stop_after_low = write_bytes.len == 0 and read_bytes.len == 0;
            self.sendByte(low, regs.tcr.tb_mask | if (stop_after_low) regs.tcr.stop_mask else 0) catch |err| return self.abort(err);
        } else if (write_bytes.len != 0 or read_bytes.len == 0) {
            const stop_after_address = write_bytes.len == 0 and read_bytes.len == 0;
            self.sendByte(
                @as(u8, @intCast(address)) << 1,
                regs.tcr.start_mask | regs.tcr.tb_mask | if (stop_after_address) regs.tcr.stop_mask else 0,
            ) catch |err| return self.abort(err);
        }

        for (write_bytes, 0..) |byte, index| {
            const is_last_write = index + 1 == write_bytes.len;
            const control = regs.tcr.tb_mask |
                if (is_last_write and read_bytes.len == 0) regs.tcr.stop_mask else 0;
            self.sendByte(byte, control) catch |err| return self.abort(err);
        }

        if (read_bytes.len != 0) {
            const read_address = if (ten_bit)
                tenBitHeader(address, true)
            else
                (@as(u8, @intCast(address)) << 1) | 1;
            self.sendByte(read_address, regs.tcr.start_mask | regs.tcr.tb_mask) catch |err| return self.abort(err);

            for (read_bytes, 0..) |*byte, index| {
                const last = index + 1 == read_bytes.len;
                const control = regs.tcr.tb_mask |
                    if (last) regs.tcr.stop_mask | regs.tcr.nack_mask else 0;
                byte.* = self.receiveByte(control) catch |err| return self.abort(err);
            }
        }

        self.waitForLocalIdle() catch |err| return self.fail(err);
    }

    pub fn transfer(self: *Sf32lb52I2c, address: u16, write_bytes: []const u8, read_bytes: []u8) bool {
        self.transferChecked(address, write_bytes, read_bytes) catch return false;
        return true;
    }

    pub fn i2c(self: *Sf32lb52I2c) I2c {
        return I2c.from(self);
    }

    pub fn lastError(self: *const Sf32lb52I2c) ?Error {
        return self.last_error;
    }

    pub fn busLevels(self: *const Sf32lb52I2c) struct { scl: bool, sda: bool } {
        const value = self.mmio.read(u32, regs.bmr.offset);
        return .{ .scl = value & regs.bmr.scl_mask != 0, .sda = value & regs.bmr.sda_mask != 0 };
    }

    pub fn recoverBus(self: *Sf32lb52I2c, cycles: u4) Error!void {
        self.mmio.write(u32, regs.rccr.offset, cycles);
        const old = self.mmio.read(u32, regs.cr.offset);
        self.mmio.write(u32, regs.cr.offset, old | regs.cr.rstreq_mask);
        var remaining = self.poll_limit;
        while (remaining != 0) : (remaining -= 1) {
            if (self.mmio.read(u32, regs.cr.offset) & regs.cr.rstreq_mask == 0) return;
        }
        return self.fail(error.Timeout);
    }

    pub fn resetBusState(self: *Sf32lb52I2c) Error!void {
        const old = self.mmio.read(u32, regs.cr.offset);
        self.mmio.write(u32, regs.cr.offset, old | regs.cr.brgrst_mask);
        var remaining = self.poll_limit;
        while (remaining != 0) : (remaining -= 1) {
            if (self.mmio.read(u32, regs.cr.offset) & regs.cr.brgrst_mask == 0) return;
        }
        return self.fail(error.Timeout);
    }

    fn sendByte(self: *Sf32lb52I2c, byte: u8, control: u32) Error!void {
        self.clearStatus(regs.sr.te_mask | transfer_error_flags);
        self.mmio.write(u32, regs.dbr.offset, byte);
        self.mmio.write(u32, regs.tcr.offset, control);
        try self.waitFor(regs.sr.te_mask);
    }

    fn receiveByte(self: *Sf32lb52I2c, control: u32) Error!u8 {
        self.clearStatus(regs.sr.rf_mask | transfer_error_flags);
        self.mmio.write(u32, regs.tcr.offset, control);
        try self.waitFor(regs.sr.rf_mask);
        return @truncate(self.mmio.read(u32, regs.dbr.offset));
    }

    fn waitFor(self: *Sf32lb52I2c, flag: u32) Error!void {
        var remaining = self.poll_limit;
        while (remaining != 0) : (remaining -= 1) {
            const status = self.mmio.read(u32, regs.sr.offset);
            if (decodeError(status)) |err| return err;
            if (status & flag != 0) return;
        }
        return error.Timeout;
    }

    fn waitForIdle(self: *Sf32lb52I2c) Error!void {
        var remaining = self.poll_limit;
        while (remaining != 0) : (remaining -= 1) {
            const status = self.mmio.read(u32, regs.sr.offset);
            if (status & (regs.sr.ub_mask | regs.sr.ibb_mask | regs.sr.ebb_mask) == 0) return;
        }
        return error.BusBusy;
    }

    fn waitForLocalIdle(self: *Sf32lb52I2c) Error!void {
        var remaining = self.poll_limit;
        while (remaining != 0) : (remaining -= 1) {
            if (self.mmio.read(u32, regs.sr.offset) & regs.sr.ub_mask == 0) return;
        }
        return error.Timeout;
    }

    fn abort(self: *Sf32lb52I2c, err: Error) Error {
        const status = self.mmio.read(u32, regs.sr.offset);
        if (status & regs.sr.ub_mask != 0) self.mmio.write(u32, regs.tcr.offset, regs.tcr.stop_mask | regs.tcr.nack_mask);
        self.clearStatus(clearable_status);
        return self.fail(err);
    }

    fn fail(self: *Sf32lb52I2c, err: Error) Error {
        self.last_error = err;
        return err;
    }

    fn clearStatus(self: *Sf32lb52I2c, mask: u32) void {
        self.mmio.write(u32, regs.sr.offset, mask);
    }
};

const Timing = struct { flv: u9, wait_count: u8 };
const flv_shift: u5 = @intCast(regs.lcr.flv_shift);
const dnf_shift: u5 = @intCast(regs.cr.dnf_shift);
const transfer_error_flags = regs.sr.ald_mask | regs.sr.bed_mask | regs.sr.ebb_mask | regs.sr.of_mask | regs.sr.uf_mask;
const clearable_status = regs.sr.te_mask | regs.sr.rf_mask | regs.sr.ald_mask | regs.sr.bed_mask | regs.sr.of_mask | regs.sr.uf_mask | regs.sr.msd_mask;

fn calculateTiming(options: Options) ConfigError!Timing {
    if (options.clock_hz == 0) return error.InvalidClock;
    if (options.bus_hz == 0) return error.InvalidBusFrequency;
    if (options.poll_limit == 0) return error.InvalidPollLimit;
    const rounded_period = (@as(u64, options.clock_hz) + options.bus_hz / 2) / options.bus_hz;
    const fixed = @as(u64, options.digital_filter_cycles) + 7;
    if (rounded_period <= fixed) return error.UnrepresentableTiming;
    const flv = (rounded_period - fixed + 1) / 2;
    if (flv == 0 or flv > 0x1ff) return error.UnrepresentableTiming;
    const half = flv / 2;
    const wait_count = if (half < 3) 0 else half - 3;
    if (wait_count > 0xff) return error.UnrepresentableTiming;
    return .{ .flv = @intCast(flv), .wait_count = @intCast(wait_count) };
}

fn tenBitHeader(address: u16, read: bool) u8 {
    return 0xf0 | @as(u8, @intCast((address >> 7) & 0x06)) | @intFromBool(read);
}

fn decodeError(status: u32) ?Error {
    if (status & regs.sr.ald_mask != 0) return error.ArbitrationLost;
    if (status & regs.sr.bed_mask != 0) return error.Nack;
    if (status & regs.sr.ebb_mask != 0) return error.BusError;
    if (status & regs.sr.of_mask != 0) return error.Overflow;
    if (status & regs.sr.uf_mask != 0) return error.Underflow;
    return null;
}

pub fn bind(mmio: Mmio, options: Options) ConfigError!Sf32lb52I2c {
    var controller = Sf32lb52I2c{ .mmio = mmio, .poll_limit = options.poll_limit };
    try controller.init(options);
    return controller;
}

pub fn fromBase(instance: Instance, options: Options) ConfigError!Sf32lb52I2c {
    return bind(Mmio.direct(@backingInt(instance)), options);
}

test "timing follows the SF32LB52 HAL formula" {
    const std = @import("std");
    try std.testing.expectEqual(Timing{ .flv = 57, .wait_count = 25 }, try calculateTiming(.{}));
    try std.testing.expectEqual(Timing{ .flv = 237, .wait_count = 115 }, try calculateTiming(.{ .bus_hz = 100_000 }));
    try std.testing.expectError(error.UnrepresentableTiming, calculateTiming(.{ .bus_hz = 20_000_000 }));
}
