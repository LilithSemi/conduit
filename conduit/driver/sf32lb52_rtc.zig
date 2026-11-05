//! SF32LB52 RTC. Calendar reads use the coherent HPSYS_CFG shadow registers.
const Mmio = @import("../mmio.zig");
const Rtc = @import("../device/rtc.zig");

pub const base_address: usize = 0x500cb000;
const TR = 0x00;
const DR = 0x04;
const CR = 0x08;
const ISR = 0x0c;
const PSCLR = 0x10;
const WUTR = 0x14;
const ALRMTR = 0x18;
const ALRMDR = 0x1c;
const BKP0R = 0x30;
const cfg_rtc_tr = 0x14;
const cfg_rtc_dr = 0x18;
const cr_alrme: u32 = 1 << 8;
const cr_wute: u32 = 1 << 9;
const cr_alrmie: u32 = 1 << 11;
const cr_wutie: u32 = 1 << 12;
const isr_alrmwf: u32 = 1;
const isr_alrmf: u32 = 1 << 1;
const isr_wutwf: u32 = 1 << 2;
const isr_wutf: u32 = 1 << 3;
const isr_initf: u32 = 1 << 9;
const isr_init: u32 = 1 << 10;

pub const Error = error{ InvalidDateTime, Timeout };
pub const WakeClock = enum { seconds, subseconds };

pub const Sf32lb52Rtc = struct {
    mmio: Mmio,
    cfg_mmio: Mmio,
    poll_limit: u32,

    pub fn now(self: *const Sf32lb52Rtc) Rtc.DateTime {
        const tr = self.cfg_mmio.read(u32, cfg_rtc_tr);
        const dr = self.cfg_mmio.read(u32, cfg_rtc_dr);
        const yy = fromBcd(@truncate((dr >> 16) & 0xff));
        return .{
            .year = (if (dr & (1 << 24) != 0) @as(u16, 1900) else 2000) + yy,
            .month = fromBcd(@truncate((dr >> 8) & 0x1f)),
            .day = fromBcd(@truncate(dr & 0x3f)),
            .hour = fromBcd(@truncate((tr >> 25) & 0x3f)),
            .minute = fromBcd(@truncate((tr >> 18) & 0x7f)),
            .second = fromBcd(@truncate((tr >> 11) & 0x7f)),
        };
    }

    pub fn set(self: *Sf32lb52Rtc, value: Rtc.DateTime) void {
        self.setChecked(value) catch {};
    }
    pub fn setChecked(self: *Sf32lb52Rtc, value: Rtc.DateTime) Error!void {
        if (!valid(value)) return error.InvalidDateTime;
        try self.enterInit();
        self.mmio.write(u32, TR, encodeTime(value));
        self.mmio.write(u32, DR, encodeDate(value));
        self.mmio.write(u32, ISR, self.mmio.read(u32, ISR) & ~isr_init);
    }

    pub fn setAlarm(self: *Sf32lb52Rtc, value: Rtc.DateTime) Error!void {
        if (!valid(value)) return error.InvalidDateTime;
        var control = self.mmio.read(u32, CR) & ~(cr_alrme | cr_alrmie);
        self.mmio.write(u32, CR, control);
        try self.waitFlag(isr_alrmwf, true);
        self.mmio.write(u32, ALRMTR, encodeTime(value));
        self.mmio.write(u32, ALRMDR, encodeDate(value) & 0xffff);
        self.mmio.write(u32, ISR, self.mmio.read(u32, ISR) & ~isr_alrmf);
        control |= cr_alrme | cr_alrmie;
        self.mmio.write(u32, CR, control);
    }
    pub fn disableAlarm(self: *Sf32lb52Rtc) Error!void {
        self.mmio.write(u32, CR, self.mmio.read(u32, CR) & ~(cr_alrme | cr_alrmie));
        try self.waitFlag(isr_alrmwf, true);
    }
    pub fn setWakeup(self: *Sf32lb52Rtc, count: u18, clock: WakeClock) Error!void {
        var control = self.mmio.read(u32, CR) & ~(cr_wute | cr_wutie);
        self.mmio.write(u32, CR, control);
        try self.waitFlag(isr_wutwf, true);
        if (clock == .subseconds) control |= 1 << 1 else control &= ~@as(u32, 1 << 1);
        self.mmio.write(u32, WUTR, count);
        self.mmio.write(u32, ISR, self.mmio.read(u32, ISR) & ~isr_wutf);
        self.mmio.write(u32, CR, control | cr_wute | cr_wutie);
    }
    pub fn interruptStatus(self: *const Sf32lb52Rtc) struct { alarm: bool, wakeup: bool } {
        const value = self.mmio.read(u32, ISR);
        return .{ .alarm = value & isr_alrmf != 0, .wakeup = value & isr_wutf != 0 };
    }
    pub fn acknowledgeInterrupts(self: *Sf32lb52Rtc, alarm: bool, wakeup: bool) void {
        var value = self.mmio.read(u32, ISR);
        if (alarm) value &= ~isr_alrmf;
        if (wakeup) value &= ~isr_wutf;
        self.mmio.write(u32, ISR, value);
    }
    pub fn readBackup(self: *const Sf32lb52Rtc, index: u4) ?u32 {
        if (index >= 10) return null;
        return self.mmio.read(u32, BKP0R + @as(usize, index) * 4);
    }
    pub fn writeBackup(self: *Sf32lb52Rtc, index: u4, value: u32) bool {
        if (index >= 10) return false;
        self.mmio.write(u32, BKP0R + @as(usize, index) * 4, value);
        return true;
    }
    pub fn rtc(self: *Sf32lb52Rtc) Rtc {
        return Rtc.from(self);
    }
    fn enterInit(self: *Sf32lb52Rtc) Error!void {
        self.mmio.write(u32, ISR, self.mmio.read(u32, ISR) | isr_init);
        try self.waitFlag(isr_initf, true);
    }
    fn waitFlag(self: *const Sf32lb52Rtc, mask: u32, asserted: bool) Error!void {
        var remaining = self.poll_limit;
        while (remaining != 0) : (remaining -= 1) {
            if ((self.mmio.read(u32, ISR) & mask != 0) == asserted) return;
        }
        return error.Timeout;
    }
};

fn encodeTime(v: Rtc.DateTime) u32 {
    return @as(u32, toBcd(v.second)) << 11 | @as(u32, toBcd(v.minute)) << 18 | @as(u32, toBcd(v.hour)) << 25;
}
fn encodeDate(v: Rtc.DateTime) u32 {
    const century: u32 = if (v.year < 2000) 1 << 24 else 0;
    return century | @as(u32, toBcd(@intCast(v.year % 100))) << 16 | @as(u32, toBcd(v.month)) << 8 | toBcd(v.day);
}
fn toBcd(v: u8) u8 {
    return (v / 10) << 4 | v % 10;
}
fn fromBcd(v: u8) u8 {
    return (v >> 4) * 10 + (v & 0xf);
}
fn valid(v: Rtc.DateTime) bool {
    return v.year >= 1970 and v.year <= 2099 and v.month >= 1 and v.month <= 12 and v.day >= 1 and v.day <= 31 and v.hour < 24 and v.minute < 60 and v.second < 60;
}

pub fn bind(mmio: Mmio, cfg_mmio: Mmio, poll_limit: u32) Sf32lb52Rtc {
    return .{ .mmio = mmio, .cfg_mmio = cfg_mmio, .poll_limit = poll_limit };
}
pub fn fromBase(poll_limit: u32) Sf32lb52Rtc {
    return bind(Mmio.direct(base_address), Mmio.direct(0x5000b000), poll_limit);
}

test "BCD helpers round trip" {
    const std = @import("std");
    try std.testing.expectEqual(@as(u8, 59), fromBcd(toBcd(59)));
}
