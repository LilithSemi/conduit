//! SF32LB52 always-on power-management controller.
const Mmio = @import("../mmio.zig");
const regs = @import("sf32lb52_regs").pmuc;

pub const Error = error{ InvalidPin, InvalidAonPin };
pub const LowPowerClock = enum { rc10, xtal32 };
pub const PinMode = enum(u3) { high, low, rising, falling, both_edges };
pub const WakeSource = enum(u4) {
    rtc = 0,
    wdt1 = 1,
    wdt2 = 2,
    pin0 = 3,
    pin1 = 4,
    low_battery = 7,
    charger = 8,
};
pub const BootMode = enum { cold, reboot, hibernate };
pub const BootReason = struct { mode: BootMode, wake_sources: u32 };

pub const Sf32lb52Pmuc = struct {
    mmio: Mmio,

    pub fn enableWake(self: *Sf32lb52Pmuc, source: WakeSource, enabled: bool) void {
        const mask = @as(u32, 1) << @backingInt(source);
        var value = self.mmio.read(u32, regs.wer.offset);
        if (enabled) value |= mask else value &= ~mask;
        self.mmio.write(u32, regs.wer.offset, value);
    }

    /// Route one of the two PMUC deep-wakeup inputs to an HPSYS_AON pin.
    pub fn configureWakePin(self: *Sf32lb52Pmuc, pin: u2, aon_pin: u5, mode: PinMode) Error!void {
        if (pin > 1) return error.InvalidPin;
        if (aon_pin > 20 or (aon_pin >= 4 and aon_pin <= 9)) return error.InvalidAonPin;
        const mode_shift: u5 = @intCast(4 + @as(u32, pin) * 3);
        const select_shift: u5 = @intCast(10 + @as(u32, pin) * 5);
        const mode_mask = @as(u32, 0x7) << mode_shift;
        const select_mask = @as(u32, 0x1f) << select_shift;
        var control = self.mmio.read(u32, regs.cr.offset);
        control = (control & ~(mode_mask | select_mask)) |
            (@as(u32, @backingInt(mode)) << mode_shift) |
            (@as(u32, aon_pin) << select_shift);
        self.mmio.write(u32, regs.cr.offset, control);
        const wake_mask = @as(u32, 1) << @intCast(3 + @as(u32, pin));
        self.mmio.write(u32, regs.wcr.offset, wake_mask);
        self.mmio.write(u32, regs.wer.offset, self.mmio.read(u32, regs.wer.offset) | wake_mask);
    }

    pub fn wakeStatus(self: *const Sf32lb52Pmuc) u32 {
        return self.mmio.read(u32, regs.wsr.offset);
    }

    pub fn clearWakeStatus(self: *Sf32lb52Pmuc, mask: u32) void {
        self.mmio.write(u32, regs.wcr.offset, mask);
    }

    pub fn setLowPowerClock(self: *Sf32lb52Pmuc, clock: LowPowerClock) void {
        var value = self.mmio.read(u32, regs.cr.offset);
        if (clock == .xtal32) value |= regs.cr.sel_lpclk_mask else value &= ~regs.cr.sel_lpclk_mask;
        self.mmio.write(u32, regs.cr.offset, value);
    }

    pub fn setPinRetention(self: *Sf32lb52Pmuc, retained: bool) void {
        var value = self.mmio.read(u32, regs.cr.offset);
        if (retained) value |= regs.cr.pin_ret_mask else value &= ~regs.cr.pin_ret_mask;
        self.mmio.write(u32, regs.cr.offset, value);
    }

    /// Assert or release the hibernate request. The caller owns system
    /// quiescing, the required delay, and non-returning execution policy.
    pub fn setHibernateRequest(self: *Sf32lb52Pmuc, asserted: bool) void {
        var value = self.mmio.read(u32, regs.cr.offset);
        if (asserted) value |= regs.cr.hiber_en_mask else value &= ~regs.cr.hiber_en_mask;
        self.mmio.write(u32, regs.cr.offset, value);
    }

    pub fn requestReboot(self: *Sf32lb52Pmuc) void {
        self.mmio.write(u32, regs.cr.offset, self.mmio.read(u32, regs.cr.offset) | regs.cr.reboot_mask);
    }

    pub fn bootReason(self: *const Sf32lb52Pmuc) BootReason {
        const control = self.mmio.read(u32, regs.cr.offset);
        const mode: BootMode = if (control & regs.cr.hiber_en_mask != 0)
            .hibernate
        else if (control & regs.cr.reboot_mask != 0)
            .reboot
        else
            .cold;
        return .{ .mode = mode, .wake_sources = self.wakeStatus() };
    }

    pub fn hpsysVout(self: *const Sf32lb52Pmuc) u4 {
        return @truncate(self.mmio.read(u32, regs.hpsys_vout.offset));
    }

    pub fn setHpsysVout(self: *Sf32lb52Pmuc, level: u4) void {
        const old = self.mmio.read(u32, regs.hpsys_vout.offset);
        self.mmio.write(u32, regs.hpsys_vout.offset, (old & ~regs.hpsys_vout.vout_mask) | level);
    }
};

pub fn bind(mmio: Mmio) Sf32lb52Pmuc {
    return .{ .mmio = mmio };
}

pub fn fromBase() Sf32lb52Pmuc {
    return bind(Mmio.direct(regs.base_address));
}
