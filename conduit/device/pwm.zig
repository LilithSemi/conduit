//! The pulse-width-modulation controller device-class contract.

const Pwm = @This();

pub const Error = error{
    InvalidChannel,
    InvalidPeriod,
    InvalidPulse,
    Unsupported,
    HardwareFault,
};

pub const Polarity = enum { active_high, active_low };

pub const Config = struct {
    period_ns: u64,
    active_ns: u64,
    polarity: Polarity = .active_high,
};

ctx: ?*anyopaque,
configure_fn: *const fn (ctx: ?*anyopaque, channel: u32, config: Config) Error!void,
enable_fn: *const fn (ctx: ?*anyopaque, channel: u32, enabled: bool) Error!void,

pub fn configure(self: Pwm, channel: u32, config: Config) Error!void {
    if (config.period_ns == 0) return error.InvalidPeriod;
    if (config.active_ns > config.period_ns) return error.InvalidPulse;
    return self.configure_fn(self.ctx, channel, config);
}

pub fn enable(self: Pwm, channel: u32, enabled: bool) Error!void {
    return self.enable_fn(self.ctx, channel, enabled);
}

pub fn from(impl: anytype) Pwm {
    const P = @TypeOf(impl);
    const gen = struct {
        fn configure(ctx: ?*anyopaque, channel: u32, config: Config) Error!void {
            const self: P = @ptrCast(@alignCast(ctx));
            return self.configure(channel, config);
        }
        fn enable(ctx: ?*anyopaque, channel: u32, enabled: bool) Error!void {
            const self: P = @ptrCast(@alignCast(ctx));
            return self.enable(channel, enabled);
        }
    };
    return .{ .ctx = impl, .configure_fn = gen.configure, .enable_fn = gen.enable };
}

test "PWM contract rejects impossible timing before touching hardware" {
    const std = @import("std");
    const Fake = struct {
        configurations: usize = 0,

        fn configure(self: *@This(), _: u32, _: Config) Error!void {
            self.configurations += 1;
        }
        fn enable(_: *@This(), _: u32, _: bool) Error!void {}
    };
    var fake = Fake{};
    const pwm = from(&fake);
    try std.testing.expectError(error.InvalidPeriod, pwm.configure(0, .{ .period_ns = 0, .active_ns = 0 }));
    try std.testing.expectError(error.InvalidPulse, pwm.configure(0, .{ .period_ns = 10, .active_ns = 11 }));
    try std.testing.expectEqual(@as(usize, 0), fake.configurations);
    try pwm.configure(0, .{ .period_ns = 10, .active_ns = 4 });
    try std.testing.expectEqual(@as(usize, 1), fake.configurations);
}
