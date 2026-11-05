//! SF32LB52 high-performance always-on power and wake controller.
const Mmio = @import("../mmio.zig");
const regs = @import("sf32lb52_regs").hpsys_aon;

pub const Error = error{ InvalidWakePin, Timeout };
pub const PowerMode = enum(u2) { active, light_sleep, deep_sleep, standby };
pub const PinMode = enum(u3) { high, low, rising, falling, both_edges };
pub const Oscillator = enum { hrc48, hxt48 };
pub const WakeSource = enum(u5) {
    rtc = 0,
    gpio1 = 1,
    lptim1 = 2,
    pmuc = 3,
    lp2hp_request = 6,
    lp2hp_interrupt = 7,
};

pub const SleepRequirements = struct {
    hrc48: bool = false,
    hxt48: bool = false,
    power: bool = false,
    external_power: bool = false,
    power_down_ram: [3]bool = .{ false, false, false },
};

pub const Sf32lb52HpsysAon = struct {
    mmio: Mmio,

    pub fn configureSleep(self: *Sf32lb52HpsysAon, mode: PowerMode, requirements: SleepRequirements) void {
        const requests = requestBits(requirements);
        switch (mode) {
            .active => {},
            .light_sleep => self.mmio.write(u32, regs.lscr.offset, requests),
            .deep_sleep => self.mmio.write(u32, regs.dscr.offset, requests),
            .standby => {
                var value = requests;
                for (requirements.power_down_ram, 0..) |down, index| {
                    if (down) value |= @as(u32, 1) << @intCast(6 + index);
                }
                self.mmio.write(u32, regs.sbcr.offset, value);
            },
        }
        const old = self.mmio.read(u32, regs.pmr.offset);
        self.mmio.write(u32, regs.pmr.offset, (old & ~regs.pmr.mode_mask) | @backingInt(mode));
    }

    pub fn powerMode(self: *const Sf32lb52HpsysAon) PowerMode {
        return @fromBackingInt(@intCast(self.mmio.read(u32, regs.pmr.offset) & regs.pmr.mode_mask));
    }

    pub fn enableWake(self: *Sf32lb52HpsysAon, source: WakeSource, enabled: bool) void {
        const mask = @as(u32, 1) << @backingInt(source);
        var value = self.mmio.read(u32, regs.wer.offset);
        if (enabled) value |= mask else value &= ~mask;
        self.mmio.write(u32, regs.wer.offset, value);
    }

    /// Configure PA24..PA44 as an always-on wake source. PA28..PA33 are not
    /// connected to the wake detector on SF32LB52 and are rejected.
    pub fn configureWakePin(self: *Sf32lb52HpsysAon, pin: u5, mode: PinMode) Error!void {
        if (!validWakePin(pin)) return error.InvalidWakePin;
        const enable_mask = @as(u32, 1) << @intCast(8 + pin);
        self.mmio.write(u32, regs.wcr.offset, enable_mask | regs.wcr.aon_mask);

        const register_offset: usize = if (pin < 8) regs.cr1.offset else if (pin < 16) regs.cr2.offset else regs.cr3.offset;
        const local: u5 = if (pin < 8) pin else if (pin < 16) pin - 8 else pin - 16;
        const shift: u5 = @intCast(@as(u32, local) * 3);
        const mask = @as(u32, 0x7) << shift;
        const old = self.mmio.read(u32, register_offset);
        self.mmio.write(u32, register_offset, (old & ~mask) | (@as(u32, @backingInt(mode)) << shift));
        self.mmio.write(u32, regs.wer.offset, self.mmio.read(u32, regs.wer.offset) | enable_mask);
    }

    pub fn disableWakePin(self: *Sf32lb52HpsysAon, pin: u5) Error!void {
        if (!validWakePin(pin)) return error.InvalidWakePin;
        const mask = @as(u32, 1) << @intCast(8 + pin);
        self.mmio.write(u32, regs.wer.offset, self.mmio.read(u32, regs.wer.offset) & ~mask);
    }

    pub fn wakeStatus(self: *const Sf32lb52HpsysAon) u32 {
        return self.mmio.read(u32, regs.wsr.offset);
    }

    pub fn clearWakeStatus(self: *Sf32lb52HpsysAon, mask: u32) void {
        self.mmio.write(u32, regs.wcr.offset, mask | regs.wcr.aon_mask);
    }

    pub fn requestOscillator(self: *Sf32lb52HpsysAon, oscillator: Oscillator, poll_limit: u32) Error!void {
        const request_mask = if (oscillator == .hrc48) regs.acr.hrc48_req_mask else regs.acr.hxt48_req_mask;
        const ready_mask = if (oscillator == .hrc48) regs.acr.hrc48_rdy_mask else regs.acr.hxt48_rdy_mask;
        self.mmio.write(u32, regs.acr.offset, self.mmio.read(u32, regs.acr.offset) | request_mask);
        var remaining = poll_limit;
        while (remaining != 0) : (remaining -= 1) {
            if (self.mmio.read(u32, regs.acr.offset) & ready_mask != 0) return;
        }
        return error.Timeout;
    }

    pub fn releaseOscillator(self: *Sf32lb52HpsysAon, oscillator: Oscillator) void {
        const mask = if (oscillator == .hrc48) regs.acr.hrc48_req_mask else regs.acr.hxt48_req_mask;
        self.mmio.write(u32, regs.acr.offset, self.mmio.read(u32, regs.acr.offset) & ~mask);
    }

    pub fn requestLcpu(self: *Sf32lb52HpsysAon, poll_limit: u32) Error!void {
        self.mmio.write(u32, regs.issr.offset, self.mmio.read(u32, regs.issr.offset) | regs.issr.hp2_lp_req_mask);
        var remaining = poll_limit;
        while (remaining != 0) : (remaining -= 1) {
            if (self.mmio.read(u32, regs.issr.offset) & regs.issr.lp_active_mask != 0) return;
        }
        return error.Timeout;
    }

    pub fn releaseLcpu(self: *Sf32lb52HpsysAon) void {
        self.mmio.write(u32, regs.issr.offset, self.mmio.read(u32, regs.issr.offset) & ~regs.issr.hp2_lp_req_mask);
    }

    pub fn setPadIsolation(self: *Sf32lb52HpsysAon, isolated: bool) void {
        self.setIsolation(regs.anacr.pa_iso_mask, isolated);
    }

    pub fn setHpsysIsolation(self: *Sf32lb52HpsysAon, isolated: bool) void {
        self.setIsolation(regs.anacr.vhp_iso_mask, isolated);
    }

    pub fn globalTimer(self: *const Sf32lb52HpsysAon) u32 {
        return self.mmio.read(u32, regs.gtimr.offset);
    }

    fn setIsolation(self: *Sf32lb52HpsysAon, mask: u32, isolated: bool) void {
        var value = self.mmio.read(u32, regs.anacr.offset);
        if (isolated) value |= mask else value &= ~mask;
        self.mmio.write(u32, regs.anacr.offset, value);
    }
};

fn validWakePin(pin: u5) bool {
    return pin <= 20 and !(pin >= 4 and pin <= 9);
}

fn requestBits(requirements: SleepRequirements) u32 {
    return @as(u32, @intFromBool(requirements.hrc48)) |
        (@as(u32, @intFromBool(requirements.hxt48)) << 1) |
        (@as(u32, @intFromBool(requirements.power)) << 2) |
        (@as(u32, @intFromBool(requirements.external_power)) << 3);
}

pub fn bind(mmio: Mmio) Sf32lb52HpsysAon {
    return .{ .mmio = mmio };
}

pub fn fromBase() Sf32lb52HpsysAon {
    return bind(Mmio.direct(regs.base_address));
}
