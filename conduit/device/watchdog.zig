//! The watchdog device-class contract.

const Watchdog = @This();

pub const Error = error{
    InvalidTimeout,
    Unsupported,
    HardwareFault,
};

/// `warning_ns` selects a two-stage watchdog: the first period raises a
/// warning/interrupt and the second period resets the system. `null` requests
/// a reset after `timeout_ns` with no warning stage.
pub const Config = struct {
    timeout_ns: u64,
    warning_ns: ?u64 = null,
};

ctx: ?*anyopaque,
start_fn: *const fn (ctx: ?*anyopaque, config: Config) Error!void,
feed_fn: *const fn (ctx: ?*anyopaque) void,
stop_fn: *const fn (ctx: ?*anyopaque) Error!void,
acknowledge_fn: ?*const fn (ctx: ?*anyopaque) void = null,

pub fn start(self: Watchdog, config: Config) Error!void {
    return self.start_fn(self.ctx, config);
}

pub fn feed(self: Watchdog) void {
    self.feed_fn(self.ctx);
}

pub fn stop(self: Watchdog) Error!void {
    return self.stop_fn(self.ctx);
}

/// Acknowledge a first-stage warning. Returns false for reset-only watchdogs.
pub fn acknowledge(self: Watchdog) bool {
    const f = self.acknowledge_fn orelse return false;
    f(self.ctx);
    return true;
}

pub fn from(impl: anytype) Watchdog {
    const P = @TypeOf(impl);
    const gen = struct {
        fn start(ctx: ?*anyopaque, config: Config) Error!void {
            const self: P = @ptrCast(@alignCast(ctx));
            return self.start(config);
        }
        fn feed(ctx: ?*anyopaque) void {
            const self: P = @ptrCast(@alignCast(ctx));
            self.feed();
        }
        fn stop(ctx: ?*anyopaque) Error!void {
            const self: P = @ptrCast(@alignCast(ctx));
            return self.stop();
        }
        fn acknowledge(ctx: ?*anyopaque) void {
            const self: P = @ptrCast(@alignCast(ctx));
            self.acknowledge();
        }
    };
    return .{
        .ctx = impl,
        .start_fn = gen.start,
        .feed_fn = gen.feed,
        .stop_fn = gen.stop,
        .acknowledge_fn = gen.acknowledge,
    };
}

test "watchdog adapter preserves configuration and operations" {
    const std = @import("std");
    const Fake = struct {
        config: ?Config = null,
        feeds: usize = 0,
        warnings: usize = 0,

        fn start(self: *@This(), config: Config) Error!void {
            if (config.timeout_ns == 0) return error.InvalidTimeout;
            self.config = config;
        }
        fn feed(self: *@This()) void {
            self.feeds += 1;
        }
        fn stop(self: *@This()) Error!void {
            self.config = null;
        }
        fn acknowledge(self: *@This()) void {
            self.warnings += 1;
        }
    };

    var fake = Fake{};
    const watchdog = from(&fake);
    try watchdog.start(.{ .timeout_ns = 2_000_000_000, .warning_ns = 500_000_000 });
    watchdog.feed();
    try std.testing.expect(watchdog.acknowledge());
    try std.testing.expectEqual(@as(usize, 1), fake.feeds);
    try std.testing.expectEqual(@as(?u64, 500_000_000), fake.config.?.warning_ns);
    try watchdog.stop();
    try std.testing.expect(fake.config == null);
}
