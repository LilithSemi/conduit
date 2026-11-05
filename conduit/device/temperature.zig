//! The temperature-sensor device-class contract.

const Temperature = @This();

pub const Error = error{
    NotReady,
    Timeout,
    OutOfRange,
    HardwareFault,
};

ctx: ?*anyopaque,
read_fn: *const fn (ctx: ?*anyopaque) Error!i32,

/// Read temperature in thousandths of a degree Celsius.
pub fn readMilliCelsius(self: Temperature) Error!i32 {
    return self.read_fn(self.ctx);
}

pub fn from(impl: anytype) Temperature {
    const P = @TypeOf(impl);
    const gen = struct {
        fn read(ctx: ?*anyopaque) Error!i32 {
            const self: P = @ptrCast(@alignCast(ctx));
            return self.readMilliCelsius();
        }
    };
    return .{ .ctx = impl, .read_fn = gen.read };
}

test "temperature adapter preserves signed fixed-point values" {
    const std = @import("std");
    const Fake = struct {
        fn readMilliCelsius(_: *@This()) Error!i32 {
            return -12_375;
        }
    };
    var fake = Fake{};
    try std.testing.expectEqual(@as(i32, -12_375), try from(&fake).readMilliCelsius());
}
