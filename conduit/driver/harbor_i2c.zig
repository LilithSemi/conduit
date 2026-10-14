//! Harbor I2C master driver, over an `Mmio`. Polled byte transfers. Each
//! register sits in its own 8-byte slot, because the controller sits on a
//! byte-addressed fabric that decodes the low bits of the byte address. 4-byte
//! spacing aliases every register onto its neighbour.
//!   0x00 CTRL, 0x08 STATUS, 0x10 DATA, 0x18 ADDR, 0x20 PRESCALE, 0x28 CMD.
//!
//! The slave address rides the DATA register as the first byte of the frame
//! (address << 1 | read), which is what the controller's shift engine sends, so
//! the ADDR register is only for slave mode.
//!
//! The CMD bits run as one sequence, not as separate triggers. A START comes
//! first, then the WRITE or READ byte, then a STOP. So START with WRITE sends
//! an address byte in a single command. A CMD write while the controller is
//! busy, or before CTRL enable is set, does nothing and sets the cmd_rejected
//! status bit.

const Mmio = @import("../mmio.zig");
const I2c = @import("../device/i2c.zig");
const match = @import("../match.zig");

pub const matcher = match.Matcher{
    .class = .i2c,
    .dt_compatible = &.{ "midstall,harbor-i2c", "harbor,i2c" },
    .driver = "harbor_i2c",
};

pub const Options = struct {
    /// SCL timing. 0 leaves the reset default. `prescaleFor` derives it from
    /// the controller input clock and the wanted bus speed.
    prescale: u32 = 0,
};

/// The PRESCALE value for `bus_freq_hz` on a controller clocked at
/// `input_clock_hz`. SCL low time is 2 * (PRESCALE + 1) input clock cycles and
/// SCL high time is the low time plus 3, so this comes from the smallest low
/// time UM10204 Table 10 permits for the mode. A slower input clock only makes
/// the low time longer, which stays legal.
///
/// The hardware field holds 16 bits, and the largest value a u32 input clock
/// can ask for is about 10100, so the result always fits.
pub fn prescaleFor(input_clock_hz: u32, bus_freq_hz: u32) u32 {
    const t_low_min_ns: u64 = if (bus_freq_hz <= 100_000)
        4700
    else if (bus_freq_hz <= 400_000)
        1300
    else
        500;

    const half_ns_per_s: u64 = 2_000_000_000;
    const ticks = t_low_min_ns * @as(u64, input_clock_hz);
    const p = (ticks + half_ns_per_s - 1) / half_ns_per_s;
    if (p == 0) return 1;
    return @intCast(p);
}

pub const HarborI2c = struct {
    mmio: Mmio,

    const CTRL = 0x00;
    const STATUS = 0x08;
    const DATA = 0x10;
    const ADDR = 0x18; // own address, slave mode only
    const PRESCALE = 0x20;
    const CMD = 0x28;

    const CTRL_ENABLE: u32 = 1 << 0;
    const CTRL_IRQ_EN: u32 = 1 << 1;

    const ST_BUSY: u32 = 1 << 0;
    const ST_ACK: u32 = 1 << 1;
    const ST_ARB_LOST: u32 = 1 << 2;
    const ST_RX_READY: u32 = 1 << 4;
    const ST_CMD_DONE: u32 = 1 << 5;
    const ST_CMD_REJECTED: u32 = 1 << 6;

    const CMD_START: u32 = 1 << 0;
    const CMD_STOP: u32 = 1 << 1;
    const CMD_WRITE: u32 = 1 << 2;
    const CMD_READ: u32 = 1 << 3;
    const CMD_NACK: u32 = 1 << 4;

    // One command finishes within a bounded number of SCL periods. This cap is
    // far above that at any prescale, so a healthy transfer never reaches it. A
    // controller that never reports CMD_DONE (unclocked or wedged) hits the cap
    // and the transfer gives up, rather than hanging the machine on the poll.
    const DONE_POLL_MAX: u32 = 1_000_000;

    /// Run one command and wait for it to finish. False means the controller
    /// refused the command or never finished it.
    fn issue(self: HarborI2c, command: u32) bool {
        self.mmio.write(u32, CMD, command);
        var i: u32 = 0;
        while (i < DONE_POLL_MAX) : (i += 1) {
            const st = self.mmio.read(u32, STATUS);
            // A write of any value clears cmd_done and cmd_rejected together.
            if (st & ST_CMD_REJECTED != 0) {
                self.mmio.write(u32, STATUS, ST_CMD_REJECTED);
                return false;
            }
            if (st & ST_CMD_DONE != 0) {
                self.mmio.write(u32, STATUS, ST_CMD_DONE);
                return true;
            }
        }
        return false;
    }

    /// The slave must acknowledge every byte the master sends, the address byte
    /// included.
    fn wroteOk(self: HarborI2c) bool {
        const st = self.mmio.read(u32, STATUS);
        return st & ST_ARB_LOST == 0 and st & ST_ACK != 0;
    }

    /// On a read the acknowledge bit holds the master's own choice, so only a
    /// lost arbitration makes a read fail.
    fn readOk(self: HarborI2c) bool {
        return self.mmio.read(u32, STATUS) & ST_ARB_LOST == 0;
    }

    fn addressPhase(self: HarborI2c, addr: u7, read: bool) bool {
        const rw: u32 = if (read) 1 else 0;
        self.mmio.write(u32, DATA, (@as(u32, addr) << 1) | rw);
        if (!self.issue(CMD_START | CMD_WRITE)) return false;
        return self.wroteOk();
    }

    fn phases(self: HarborI2c, addr: u7, wr: []const u8, rd: []u8) bool {
        if (wr.len > 0) {
            if (!self.addressPhase(addr, false)) return false;
            for (wr) |b| {
                self.mmio.write(u32, DATA, b);
                if (!self.issue(CMD_WRITE)) return false;
                if (!self.wroteOk()) return false;
            }
        }
        if (rd.len > 0) {
            // A repeated START turns the bus around with no STOP between the
            // two phases, so a device that holds a register pointer keeps it.
            if (!self.addressPhase(addr, true)) return false;
            for (rd, 0..) |*b, i| {
                // The last byte of a read gets a NACK, which makes the slave
                // release SDA before the STOP.
                const last = i == rd.len - 1;
                const command: u32 = if (last) CMD_READ | CMD_NACK else CMD_READ;
                if (!self.issue(command)) return false;
                if (!self.readOk()) return false;
                b.* = @truncate(self.mmio.read(u32, DATA));
            }
        }
        return true;
    }

    /// Write `wr`, then read `rd`, for 7-bit `addr`. A repeated START joins the
    /// two phases into one transaction.
    pub fn transfer(self: HarborI2c, addr: u16, wr: []const u8, rd: []u8) bool {
        if (wr.len == 0 and rd.len == 0) return true;
        const ok = self.phases(@truncate(addr), wr, rd);
        // The STOP goes out on the failure path too, so a transfer that gives
        // up does not leave the bus held.
        const stopped = self.issue(CMD_STOP);
        return ok and stopped;
    }

    pub fn i2c(self: *HarborI2c) I2c {
        return I2c.from(self);
    }
};

pub fn bind(mmio: Mmio, opts: Options) HarborI2c {
    const dev = HarborI2c{ .mmio = mmio };
    if (opts.prescale != 0) mmio.write(u32, HarborI2c.PRESCALE, opts.prescale);
    mmio.write(u32, HarborI2c.CTRL, HarborI2c.CTRL_ENABLE);
    return dev;
}
