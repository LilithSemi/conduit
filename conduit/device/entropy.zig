//! The hardware entropy-source device-class contract.

const Entropy = @This();

pub const Error = error{
    NotReady,
    HealthTestFailed,
    Timeout,
    HardwareFault,
};

ctx: ?*anyopaque,
fill_fn: *const fn (ctx: ?*anyopaque, out: []u8) Error!void,

/// Fill every byte in `out` with conditioned hardware entropy.
pub fn fill(self: Entropy, out: []u8) Error!void {
    return self.fill_fn(self.ctx, out);
}

pub fn from(impl: anytype) Entropy {
    const P = @TypeOf(impl);
    const gen = struct {
        fn fill(ctx: ?*anyopaque, out: []u8) Error!void {
            const self: P = @ptrCast(@alignCast(ctx));
            return self.fill(out);
        }
    };
    return .{ .ctx = impl, .fill_fn = gen.fill };
}

test "entropy adapter fills the complete destination" {
    const std = @import("std");
    const Fake = struct {
        fn fill(_: *@This(), out: []u8) Error!void {
            @memset(out, 0xa5);
        }
    };
    var fake = Fake{};
    var bytes: [7]u8 = undefined;
    try from(&fake).fill(&bytes);
    const expected = [_]u8{ 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5 };
    try std.testing.expectEqualSlices(u8, &expected, &bytes);
}
