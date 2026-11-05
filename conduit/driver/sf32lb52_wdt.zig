//! SiFli SF32LB52 HCPU watchdog (WDT1).
//!
//! WDT1 has a 24-bit first-stage counter and an optional 24-bit second-stage
//! counter. In two-stage mode, expiry of counter 0 raises an interrupt and
//! counter 1 then runs to the reset. Register writes are protected by magic
//! unlock/lock values.

const Mmio = @import("../mmio.zig");
const Watchdog = @import("../device/watchdog.zig");

pub const Sf32lb52Wdt = struct {
    mmio: Mmio,
    clock_hz: u32,

    pub const base_address: u64 = 0x50094000;
    pub const default_clock_hz: u32 = 32_768;
    pub const max_ticks: u32 = 0x00ff_ffff;

    const CVR0 = 0x00;
    const CVR1 = 0x04;
    const CR = 0x08;
    const CCR = 0x0c;
    const ICR = 0x10;
    const SR = 0x14;
    const WP = 0x18;

    const response_mode_interrupt_then_reset: u32 = 1 << 4;
    const command_restart: u32 = 0x76;
    const command_stop: u32 = 0x34;
    const protect_magic: u32 = 0x58ab99fc;
    const release_magic: u32 = 0x51ff8621;

    fn unlock(self: Sf32lb52Wdt) void {
        self.mmio.write(u32, WP, release_magic);
    }

    fn lock(self: Sf32lb52Wdt) void {
        self.mmio.write(u32, WP, protect_magic);
    }

    fn durationTicks(self: Sf32lb52Wdt, duration_ns: u64) Watchdog.Error!u32 {
        if (duration_ns == 0 or self.clock_hz == 0) return error.InvalidTimeout;
        const scaled = @as(u128, duration_ns) * self.clock_hz;
        const ticks = (scaled + 999_999_999) / 1_000_000_000;
        if (ticks == 0 or ticks > max_ticks) return error.InvalidTimeout;
        return @intCast(ticks);
    }

    pub fn start(self: Sf32lb52Wdt, config: Watchdog.Config) Watchdog.Error!void {
        const first_ticks = try self.durationTicks(config.timeout_ns);
        const second_ticks = if (config.warning_ns) |duration|
            try self.durationTicks(duration)
        else
            0;

        self.unlock();
        self.mmio.write(u32, CCR, command_stop);
        self.mmio.write(u32, CVR0, first_ticks);
        self.mmio.write(u32, CVR1, second_ticks);

        var control = self.mmio.read(u32, CR);
        control &= ~response_mode_interrupt_then_reset;
        if (config.warning_ns != null) control |= response_mode_interrupt_then_reset;
        self.mmio.write(u32, CR, control);
        self.mmio.write(u32, CCR, command_restart);
        self.lock();
    }

    pub fn feed(self: Sf32lb52Wdt) void {
        self.unlock();
        self.mmio.write(u32, CCR, command_restart);
        self.lock();
    }

    pub fn stop(self: Sf32lb52Wdt) Watchdog.Error!void {
        self.unlock();
        self.mmio.write(u32, CCR, command_stop);
        self.lock();
    }

    /// Clear a first-stage interrupt and reload both counters.
    pub fn acknowledge(self: Sf32lb52Wdt) void {
        self.unlock();
        self.mmio.write(u32, ICR, 1);
        self.lock();
    }

    pub fn active(self: Sf32lb52Wdt) bool {
        return (self.mmio.read(u32, SR) & (1 << 1)) != 0;
    }

    pub fn warningPending(self: Sf32lb52Wdt) bool {
        return (self.mmio.read(u32, SR) & 1) != 0;
    }

    pub fn watchdog(self: *Sf32lb52Wdt) Watchdog {
        return Watchdog.from(self);
    }
};

pub fn bind(mmio: Mmio, clock_hz: u32) Sf32lb52Wdt {
    return .{ .mmio = mmio, .clock_hz = clock_hz };
}

pub fn fromBase() Sf32lb52Wdt {
    return bind(Mmio.direct(Sf32lb52Wdt.base_address), Sf32lb52Wdt.default_clock_hz);
}
