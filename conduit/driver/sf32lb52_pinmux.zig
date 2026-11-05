//! SiFli SF32LB52 HPSYS pin multiplexer for PA0..PA44.
//!
//! Peripheral routing in HPSYS_CFG is intentionally separate.  This driver
//! owns only each pad's electrical configuration and function selection.

const std = @import("std");
const Mmio = @import("../mmio.zig");

pub const Sf32lb52Pinmux = struct {
    mmio: Mmio,

    pub const base_address: u64 = 0x50003000;
    pub const max_pin: u32 = 44;

    pub const Function = enum(u4) {
        gpio = 0,
        /// Shared selection used by UART and I2C; HPSYS_CFG picks the signal.
        uart_i2c = 4,
        _,
    };

    pub const Pull = enum { none, down, up };

    pub const Config = struct {
        function: Function = .gpio,
        pull: Pull = .none,
        input_enable: bool = true,
        schmitt: bool = false,
        slow_slew: bool = false,
        /// Raw DS[1:0] value. Its electrical meaning is pad-dependent.
        drive_strength: u2 = 0,
    };

    const PAD_PA00 = 0x34;
    const FSEL_MASK: u32 = 0x0f;
    const PE: u32 = 1 << 4;
    const PS: u32 = 1 << 5;
    const IE: u32 = 1 << 6;
    const IS: u32 = 1 << 7;
    const SR: u32 = 1 << 8;
    const DS_SHIFT = 9;

    pub fn configure(self: Sf32lb52Pinmux, pin: u32, config: Config) void {
        std.debug.assert(pin <= max_pin);

        var value: u32 = @backingInt(config.function) & FSEL_MASK;
        value |= switch (config.pull) {
            .none => 0,
            .down => PE,
            .up => PE | PS,
        };
        if (config.input_enable) value |= IE;
        if (config.schmitt) value |= IS;
        if (config.slow_slew) value |= SR;
        value |= @as(u32, config.drive_strength) << DS_SHIFT;

        self.mmio.write(u32, PAD_PA00 + @as(usize, pin) * 4, value);
    }

    /// Select GPIO with the input buffer enabled and both pulls disabled.
    /// The GPIO controller must separately disable its output driver.
    pub fn floatingInput(self: Sf32lb52Pinmux, pin: u32) void {
        self.configure(pin, .{});
    }
};

pub fn bind(mmio: Mmio) Sf32lb52Pinmux {
    return .{ .mmio = mmio };
}

pub fn fromBase() Sf32lb52Pinmux {
    return bind(Mmio.direct(Sf32lb52Pinmux.base_address));
}
