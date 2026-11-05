//! SiFli SF32LB52 HPSYS peripheral signal routing.
//!
//! Selecting `.uart_i2c` in the pad mux only selects the shared pad function.
//! These registers connect a particular UART or I2C signal to that pad.

const std = @import("std");
const Mmio = @import("../mmio.zig");

pub const Sf32lb52HpsysCfg = struct {
    mmio: Mmio,

    pub const base_address: u64 = 0x5000b000;
    pub const max_pin: u6 = 44;

    pub const Uart = enum(u2) { uart1 = 1, uart2 = 2, uart3 = 3 };
    pub const I2c = enum(u3) { i2c1 = 1, i2c2 = 2, i2c3 = 3, i2c4 = 4 };

    pub const UartPins = struct {
        tx: u6,
        rx: u6,
        /// Leave an optional flow-control route unchanged when it is absent.
        rts: ?u6 = null,
        cts: ?u6 = null,
    };

    pub const I2cPins = struct {
        scl: u6,
        sda: u6,
    };

    const I2C1_PINR = 0x48;
    const USART1_PINR = 0x58;

    fn validPin(pin: u6) bool {
        return pin <= max_pin;
    }

    pub fn routeUart(self: Sf32lb52HpsysCfg, uart: Uart, pins: UartPins) void {
        std.debug.assert(validPin(pins.tx));
        std.debug.assert(validPin(pins.rx));
        if (pins.rts) |pin| std.debug.assert(validPin(pin));
        if (pins.cts) |pin| std.debug.assert(validPin(pin));

        const index = @as(usize, @backingInt(uart)) - 1;
        const offset = USART1_PINR + index * 4;
        var value = self.mmio.read(u32, offset);
        value = (value & ~@as(u32, 0x00003f3f)) |
            @as(u32, pins.tx) |
            (@as(u32, pins.rx) << 8);
        if (pins.rts) |pin| value = (value & ~@as(u32, 0x003f0000)) | (@as(u32, pin) << 16);
        if (pins.cts) |pin| value = (value & ~@as(u32, 0x3f000000)) | (@as(u32, pin) << 24);
        self.mmio.write(u32, offset, value);
    }

    pub fn routeI2c(self: Sf32lb52HpsysCfg, i2c: I2c, pins: I2cPins) void {
        std.debug.assert(validPin(pins.scl));
        std.debug.assert(validPin(pins.sda));

        const value = @as(u32, pins.scl) | (@as(u32, pins.sda) << 8);
        const index = @as(usize, @backingInt(i2c)) - 1;
        self.mmio.write(u32, I2C1_PINR + index * 4, value);
    }
};

pub fn bind(mmio: Mmio) Sf32lb52HpsysCfg {
    return .{ .mmio = mmio };
}

pub fn fromBase() Sf32lb52HpsysCfg {
    return bind(Mmio.direct(Sf32lb52HpsysCfg.base_address));
}
