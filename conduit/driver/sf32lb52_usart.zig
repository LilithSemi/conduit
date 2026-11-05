//! SF32LB52 USART1/2/3 polling driver.
//!
//! Clock gates, resets, pinmux, and HPSYS_CFG routing are intentionally owned
//! by their respective drivers. This module configures an already-clocked
//! USART and exposes Conduit's byte-stream `Serial` contract.

const Mmio = @import("../mmio.zig");
const Serial = @import("../device/serial.zig");
const pac = @import("sf32lb52_regs");

// All three instances share a register layout. Use USART1 as the generated
// constants authority and select only the direct-MMIO base at runtime.
const regs = pac.usart1;

pub const Instance = enum(usize) {
    usart1 = pac.usart1.base_address,
    usart2 = pac.usart2.base_address,
    usart3 = pac.usart3.base_address,
};

pub const DataBits = enum(u4) {
    six = 6,
    seven = 7,
    eight = 8,
    nine = 9,
};

pub const Parity = enum { none, even, odd };
pub const StopBits = enum { one, two };
pub const FlowControl = enum { none, rts, cts, rts_cts };

pub const Options = struct {
    clock_hz: u32,
    baud: u32,
    data_bits: DataBits = .eight,
    parity: Parity = .none,
    stop_bits: StopBits = .one,
    flow_control: FlowControl = .none,
};

pub const ConfigError = error{
    InvalidClock,
    InvalidBaud,
    UnsupportedBaud,
    UnsupportedFrame,
};

pub const RxErrors = packed struct(u4) {
    parity: bool = false,
    framing: bool = false,
    noise: bool = false,
    overrun: bool = false,

    pub fn any(self: RxErrors) bool {
        return @as(u4, @bitCast(self)) != 0;
    }
};

const BaudConfig = struct {
    brr: u16,
    over8: bool,
};

pub const Sf32lb52Usart = struct {
    mmio: Mmio,
    rx_errors: RxErrors = .{},

    pub fn init(self: *Sf32lb52Usart, options: Options) ConfigError!void {
        const baud = try baudConfig(options.clock_hz, options.baud);
        const data_count: u5 = @backingInt(options.data_bits);
        const parity_count: u5 = if (options.parity == .none) 0 else 1;
        const packet_bits = data_count + parity_count;
        if (packet_bits < 6 or packet_bits > 9) return error.UnsupportedFrame;

        // UE must be clear while the frame and baud registers are changed.
        self.mmio.write(u32, regs.cr1.offset, 0);

        const stop: u32 = if (options.stop_bits == .two) 2 else 0;
        self.mmio.write(u32, regs.cr2.offset, stop << regs.cr2.stop_shift);

        var cr3: u32 = 0;
        if (options.flow_control == .rts or options.flow_control == .rts_cts) cr3 |= regs.cr3.rtse_mask;
        if (options.flow_control == .cts or options.flow_control == .rts_cts) cr3 |= regs.cr3.ctse_mask;
        self.mmio.write(u32, regs.cr3.offset, cr3);
        self.mmio.write(u32, regs.brr.offset, baud.brr);

        // Clear stale receive errors, idle/complete, and CTS edge state before
        // enabling the receiver. These ICR bits are write-one-to-clear.
        self.mmio.write(u32, regs.icr.offset, clearable_flags);

        var cr1: u32 = regs.cr1.ue_mask | regs.cr1.re_mask | regs.cr1.te_mask;
        cr1 |= @as(u32, packet_bits - 6) << regs.cr1.m_shift;
        if (baud.over8) cr1 |= regs.cr1.over8_mask;
        if (options.parity != .none) cr1 |= regs.cr1.pce_mask;
        if (options.parity == .odd) cr1 |= regs.cr1.ps_mask;
        self.mmio.write(u32, regs.cr1.offset, cr1);
        self.rx_errors = .{};
    }

    pub fn disable(self: *Sf32lb52Usart) void {
        const cr1 = self.mmio.read(u32, regs.cr1.offset);
        self.mmio.write(u32, regs.cr1.offset, cr1 & ~regs.cr1.ue_mask);
    }

    pub fn txReady(self: *const Sf32lb52Usart) bool {
        return self.mmio.read(u32, regs.isr.offset) & regs.isr.txe_mask != 0;
    }

    pub fn rxReady(self: *const Sf32lb52Usart) bool {
        return self.mmio.read(u32, regs.isr.offset) & regs.isr.rxne_mask != 0;
    }

    pub fn putc(self: *Sf32lb52Usart, byte: u8) void {
        while (!self.txReady()) {}
        self.mmio.write(u32, regs.tdr.offset, byte);
    }

    /// Write bytes exactly as supplied. Firmware transports must not silently
    /// turn LF into CRLF; console policy belongs above the UART driver.
    pub fn write(self: *Sf32lb52Usart, bytes: []const u8) void {
        for (bytes) |byte| self.putc(byte);
    }

    /// Read all bytes currently available. Receive errors are accumulated and
    /// can be retrieved with `takeRxErrors`; valid RDR contents are still
    /// returned for diagnostics and protocols that can recover above us.
    pub fn read(self: *Sf32lb52Usart, out: []u8) usize {
        var count: usize = 0;
        while (count < out.len) {
            const status = self.mmio.read(u32, regs.isr.offset);
            if (status & regs.isr.rxne_mask == 0) break;
            self.captureErrors(status);
            out[count] = @truncate(self.mmio.read(u32, regs.rdr.offset));
            count += 1;
        }
        return count;
    }

    pub fn takeRxErrors(self: *Sf32lb52Usart) RxErrors {
        const result = self.rx_errors;
        self.rx_errors = .{};
        return result;
    }

    pub fn flush(self: *const Sf32lb52Usart) void {
        while (self.mmio.read(u32, regs.isr.offset) & regs.isr.tc_mask == 0) {}
    }

    pub fn enableRxInterrupt(self: *Sf32lb52Usart, enabled: bool) void {
        self.updateCr1(regs.cr1.rxneie_mask, enabled);
    }

    pub fn enableTxEmptyInterrupt(self: *Sf32lb52Usart, enabled: bool) void {
        self.updateCr1(regs.cr1.txeie_mask, enabled);
    }

    pub fn configureDma(self: *Sf32lb52Usart, rx: bool, tx: bool) void {
        var value = self.mmio.read(u32, regs.cr3.offset);
        value = if (rx) value | regs.cr3.dmar_mask else value & ~regs.cr3.dmar_mask;
        value = if (tx) value | regs.cr3.dmat_mask else value & ~regs.cr3.dmat_mask;
        self.mmio.write(u32, regs.cr3.offset, value);
    }

    pub fn serial(self: *Sf32lb52Usart) Serial {
        return Serial.from(self);
    }

    fn updateCr1(self: *Sf32lb52Usart, mask: u32, enabled: bool) void {
        const old = self.mmio.read(u32, regs.cr1.offset);
        self.mmio.write(u32, regs.cr1.offset, if (enabled) old | mask else old & ~mask);
    }

    fn captureErrors(self: *Sf32lb52Usart, status: u32) void {
        const errors = status & receive_error_flags;
        if (errors == 0) return;
        self.rx_errors.parity = self.rx_errors.parity or errors & regs.isr.pe_mask != 0;
        self.rx_errors.framing = self.rx_errors.framing or errors & regs.isr.fe_mask != 0;
        self.rx_errors.noise = self.rx_errors.noise or errors & regs.isr.nf_mask != 0;
        self.rx_errors.overrun = self.rx_errors.overrun or errors & regs.isr.ore_mask != 0;
        self.mmio.write(u32, regs.icr.offset, errors);
    }
};

const receive_error_flags = regs.isr.pe_mask | regs.isr.fe_mask | regs.isr.nf_mask | regs.isr.ore_mask;
const clearable_flags = receive_error_flags | regs.icr.idlecf_mask | regs.icr.tccf_mask | regs.icr.ctscf_mask;

fn baudConfig(clock_hz: u32, baud: u32) ConfigError!BaudConfig {
    if (clock_hz == 0) return error.InvalidClock;
    if (baud == 0) return error.InvalidBaud;

    // In x16 mode BRR is round(clock / baud). In x8 mode the generated SVD's
    // documented INT.FRAC/16 encoding makes it round(2 * clock / baud).
    const div16 = roundedDiv(clock_hz, baud);
    if (validDivisor(div16)) return .{ .brr = @intCast(div16), .over8 = false };

    const div8 = roundedDiv(@as(u64, clock_hz) * 2, baud);
    if (validDivisor(div8)) return .{ .brr = @intCast(div8), .over8 = true };
    return error.UnsupportedBaud;
}

fn roundedDiv(numerator: u64, denominator: u32) u64 {
    return (numerator + denominator / 2) / denominator;
}

fn validDivisor(divisor: u64) bool {
    // INT is twelve bits and must be non-zero. FRAC occupies the low nibble.
    return divisor >= 16 and divisor <= 0xffff;
}

pub fn bind(mmio: Mmio, options: Options) ConfigError!Sf32lb52Usart {
    var usart = Sf32lb52Usart{ .mmio = mmio };
    try usart.init(options);
    return usart;
}

pub fn fromBase(instance: Instance, options: Options) ConfigError!Sf32lb52Usart {
    return bind(Mmio.direct(@backingInt(instance)), options);
}

test "baud divisor selects x16 and x8 modes" {
    const std = @import("std");
    try std.testing.expectEqual(BaudConfig{ .brr = 48, .over8 = false }, try baudConfig(48_000_000, 1_000_000));
    try std.testing.expectEqual(BaudConfig{ .brr = 16, .over8 = true }, try baudConfig(48_000_000, 6_000_000));
    try std.testing.expectError(error.InvalidBaud, baudConfig(48_000_000, 0));
    try std.testing.expectError(error.UnsupportedBaud, baudConfig(48_000_000, 7_000_000));
}
