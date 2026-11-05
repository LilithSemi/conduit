//! SF32LB52 advanced/general PWM and basic timers.
const Mmio = @import("../mmio.zig");
const Pwm = @import("../device/pwm.zig");
const pac = @import("sf32lb52_regs");
pub const PwmInstance = enum { atim1, gptim1, gptim2 };
pub const BasicInstance = enum(usize) { btim1 = pac.btim1.base_address, btim2 = pac.btim2.base_address, btim3 = pac.btim3.base_address, btim4 = pac.btim4.base_address };

pub const PwmTimer = struct {
    mmio: Mmio,
    clock_hz: u32,
    max_count: u32,
    advanced: bool,
    pub fn configure(self: *PwmTimer, channel: u32, config: Pwm.Config) Pwm.Error!void {
        if (channel >= 4) return error.InvalidChannel;
        if (config.period_ns == 0) return error.InvalidPeriod;
        if (config.active_ns > config.period_ns) return error.InvalidPulse;
        const timing = calculate(self.clock_hz, config.period_ns, self.max_count) catch return error.InvalidPeriod;
        const active = (@as(u128, timing.counts) * config.active_ns + config.period_ns / 2) / config.period_ns;
        self.mmio.write(u32, 0x28, timing.prescaler - 1);
        self.mmio.write(u32, 0x2c, timing.counts - 1);
        self.mmio.write(u32, 0x34 + channel * 4, @intCast(active));
        const pair_off: usize = if (channel < 2) 0x18 else 0x1c;
        const within = channel & 1;
        const mode_shift: u5 = @intCast(20 + within * 8);
        const preload_shift: u5 = @intCast(19 + within * 8);
        const select_shift: u5 = @intCast(within * 8);
        var ccmr = self.mmio.read(u32, pair_off);
        ccmr &= ~((@as(u32, 0xf) << mode_shift) | (@as(u32, 3) << select_shift));
        ccmr |= (@as(u32, 6) << mode_shift) | (@as(u32, 1) << preload_shift);
        self.mmio.write(u32, pair_off, ccmr);
        const polarity_bit: u5 = @intCast(channel * 4 + 1);
        var ccer = self.mmio.read(u32, 0x20);
        if (config.polarity == .active_low) ccer |= @as(u32, 1) << polarity_bit else ccer &= ~(@as(u32, 1) << polarity_bit);
        self.mmio.write(u32, 0x20, ccer);
        self.mmio.write(u32, 0x00, self.mmio.read(u32, 0x00) | (1 << 7));
        self.mmio.write(u32, 0x14, 1);
    }
    pub fn enable(self: *PwmTimer, channel: u32, enabled: bool) Pwm.Error!void {
        if (channel >= 4) return error.InvalidChannel;
        const bit: u5 = @intCast(channel * 4);
        var ccer = self.mmio.read(u32, 0x20);
        if (enabled) ccer |= @as(u32, 1) << bit else ccer &= ~(@as(u32, 1) << bit);
        self.mmio.write(u32, 0x20, ccer);
        if (self.advanced and enabled) self.mmio.write(u32, 0x44, self.mmio.read(u32, 0x44) | (1 << 15));
        if (enabled) self.mmio.write(u32, 0x00, self.mmio.read(u32, 0x00) | 1);
    }
    pub fn stop(self: *PwmTimer) void {
        self.mmio.write(u32, 0x00, self.mmio.read(u32, 0x00) & ~@as(u32, 1));
    }
    pub fn counter(self: *const PwmTimer) u32 {
        return self.mmio.read(u32, 0x24) & self.max_count;
    }
    pub fn pwm(self: *PwmTimer) Pwm {
        return Pwm.from(self);
    }
};

pub const BasicTimer = struct {
    mmio: Mmio,
    clock_hz: u32,
    pub fn configure(self: *BasicTimer, period_ns: u64, one_shot: bool) Pwm.Error!void {
        if (period_ns == 0) return error.InvalidPeriod;
        const timing = calculate(self.clock_hz, period_ns, 0xffffffff) catch return error.InvalidPeriod;
        self.mmio.write(u32, 0x00, (if (one_shot) @as(u32, 1 << 3) else 0) | (1 << 7));
        self.mmio.write(u32, 0x28, timing.prescaler - 1);
        self.mmio.write(u32, 0x2c, timing.counts - 1);
        self.mmio.write(u32, 0x14, 1);
        self.mmio.write(u32, 0x10, 0);
    }
    pub fn start(self: *BasicTimer) void {
        self.mmio.write(u32, 0x00, self.mmio.read(u32, 0x00) | 1);
    }
    pub fn stop(self: *BasicTimer) void {
        self.mmio.write(u32, 0x00, self.mmio.read(u32, 0x00) & ~@as(u32, 1));
    }
    pub fn enableUpdateInterrupt(self: *BasicTimer, enabled: bool) void {
        self.setDier(1, enabled);
    }
    pub fn enableUpdateDma(self: *BasicTimer, enabled: bool) void {
        self.setDier(1 << 8, enabled);
    }
    pub fn pending(self: *const BasicTimer) bool {
        return self.mmio.read(u32, 0x10) & 1 != 0;
    }
    pub fn acknowledge(self: *BasicTimer) void {
        self.mmio.write(u32, 0x10, self.mmio.read(u32, 0x10) & ~@as(u32, 1));
    }
    pub fn counter(self: *const BasicTimer) u32 {
        return self.mmio.read(u32, 0x24);
    }
    fn setDier(self: *BasicTimer, mask: u32, enabled: bool) void {
        var value = self.mmio.read(u32, 0x0c);
        if (enabled) value |= mask else value &= ~mask;
        self.mmio.write(u32, 0x0c, value);
    }
};
const Timing = struct { prescaler: u32, counts: u32 };
fn calculate(clock_hz: u32, period_ns: u64, max_count: u32) error{Unrepresentable}!Timing {
    if (clock_hz == 0 or period_ns == 0) return error.Unrepresentable;
    const ticks = (@as(u128, clock_hz) * period_ns + 500_000_000) / 1_000_000_000;
    if (ticks == 0) return error.Unrepresentable;
    const capacity = @as(u128, max_count) + 1;
    const prescaler = (ticks + capacity - 1) / capacity;
    if (prescaler == 0 or prescaler > 0x10000) return error.Unrepresentable;
    const counts = (ticks + prescaler / 2) / prescaler;
    if (counts == 0 or counts > capacity) return error.Unrepresentable;
    return .{ .prescaler = @intCast(prescaler), .counts = @intCast(counts) };
}
pub fn bindPwm(mmio: Mmio, clock_hz: u32, max_count: u32, advanced: bool) PwmTimer {
    return .{ .mmio = mmio, .clock_hz = clock_hz, .max_count = max_count, .advanced = advanced };
}
pub fn pwmFromBase(instance: PwmInstance, clock_hz: u32) PwmTimer {
    return switch (instance) {
        .atim1 => bindPwm(Mmio.direct(pac.atim1.base_address), clock_hz, 0xffffffff, true),
        .gptim1 => bindPwm(Mmio.direct(pac.gptim1.base_address), clock_hz, 0xffff, false),
        .gptim2 => bindPwm(Mmio.direct(pac.gptim2.base_address), clock_hz, 0xffff, false),
    };
}
pub fn bindBasic(mmio: Mmio, clock_hz: u32) BasicTimer {
    return .{ .mmio = mmio, .clock_hz = clock_hz };
}
pub fn basicFromBase(instance: BasicInstance, clock_hz: u32) BasicTimer {
    return bindBasic(Mmio.direct(@backingInt(instance)), clock_hz);
}
