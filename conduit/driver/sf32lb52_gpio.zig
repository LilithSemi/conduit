//! SiFli SF32LB52 HPSYS GPIO controller.
//!
//! GPIO1 exposes PA0..PA44 as two register banks.  State-changing operations
//! use the controller's write-only SET/CLR aliases so callers never race a
//! read/modify/write against another execution context.

const std = @import("std");
const Mmio = @import("../mmio.zig");
const Gpio = @import("../device/gpio.zig");

pub const Sf32lb52Gpio = struct {
    mmio: Mmio,

    pub const base_address: u64 = 0x500a0000;
    pub const max_pin: u32 = 44;

    pub const InterruptTrigger = enum {
        low_level,
        high_level,
        falling_edge,
        rising_edge,
        both_edges,
    };

    const BANK_STRIDE = 0x80;
    const DIR = 0x00;
    const DOSR = 0x08;
    const DOCR = 0x0c;
    const DOESR = 0x14;
    const DOECR = 0x18;
    const IESR = 0x20;
    const IECR = 0x24;
    const ITSR = 0x2c;
    const ITCR = 0x30;
    const IPHSR = 0x38;
    const IPHCR = 0x3c;
    const IPLSR = 0x44;
    const IPLCR = 0x48;
    const ISR = 0x4c;

    fn bankOffset(pin: u32) usize {
        std.debug.assert(pin <= max_pin);
        return if (pin < 32) 0 else BANK_STRIDE;
    }

    fn pinBit(pin: u32) u32 {
        std.debug.assert(pin <= max_pin);
        return @as(u32, 1) << @intCast(pin & 31);
    }

    fn writePin(self: Sf32lb52Gpio, pin: u32, register: usize) void {
        self.mmio.write(u32, bankOffset(pin) + register, pinBit(pin));
    }

    pub fn direction(self: Sf32lb52Gpio, pin: u32, dir: Gpio.Dir) void {
        self.writePin(pin, switch (dir) {
            .input => DOECR,
            .output => DOESR,
        });
    }

    pub fn set(self: Sf32lb52Gpio, pin: u32, value: bool) void {
        self.writePin(pin, if (value) DOSR else DOCR);
    }

    pub fn get(self: Sf32lb52Gpio, pin: u32) bool {
        return self.mmio.read(u32, bankOffset(pin) + DIR) & pinBit(pin) != 0;
    }

    /// Configure and enable one pin's interrupt source.
    pub fn configureInterrupt(self: Sf32lb52Gpio, pin: u32, trigger: InterruptTrigger) void {
        const edge = switch (trigger) {
            .falling_edge, .rising_edge, .both_edges => true,
            .low_level, .high_level => false,
        };
        const high = switch (trigger) {
            .high_level, .rising_edge, .both_edges => true,
            .low_level, .falling_edge => false,
        };
        const low = switch (trigger) {
            .low_level, .falling_edge, .both_edges => true,
            .high_level, .rising_edge => false,
        };

        self.writePin(pin, if (edge) ITSR else ITCR);
        self.writePin(pin, if (high) IPHSR else IPHCR);
        self.writePin(pin, if (low) IPLSR else IPLCR);
        self.writePin(pin, IESR);
    }

    pub fn disableInterrupt(self: Sf32lb52Gpio, pin: u32) void {
        self.writePin(pin, IECR);
    }

    pub fn interruptPending(self: Sf32lb52Gpio, pin: u32) bool {
        return self.mmio.read(u32, bankOffset(pin) + ISR) & pinBit(pin) != 0;
    }

    /// Acknowledge one pin. ISR is write-one-to-clear.
    pub fn clearInterrupt(self: Sf32lb52Gpio, pin: u32) void {
        self.writePin(pin, ISR);
    }

    pub fn gpio(self: *Sf32lb52Gpio) Gpio {
        return Gpio.from(self);
    }
};

pub fn bind(mmio: Mmio) Sf32lb52Gpio {
    return .{ .mmio = mmio };
}

pub fn fromBase() Sf32lb52Gpio {
    return bind(Mmio.direct(Sf32lb52Gpio.base_address));
}
