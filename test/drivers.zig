//! Round-trip tests for the GPIO/I2C/SPI/timer/block/intc drivers over fake
//! register files. The file driver.zig covers the serial drivers.

const std = @import("std");
const fake = @import("fake.zig");
const Mmio = @import("conduit").Mmio;
const harbor_gpio = @import("conduit").driver.harbor_gpio;
const harbor_i2c = @import("conduit").driver.harbor_i2c;
const harbor_spi = @import("conduit").driver.harbor_spi;
const clint = @import("conduit").driver.clint;
const harbor_sdio = @import("conduit").driver.harbor_sdio;
const plic = @import("conduit").driver.plic;
const gicv2 = @import("conduit").driver.gicv2;
const gicv3 = @import("conduit").driver.gicv3;
const goldfish_rtc = @import("conduit").driver.goldfish_rtc;
const pl031 = @import("conduit").driver.pl031;

test "harbor_gpio: direction, output, input" {
    // Registers sit 8 bytes apart. With 4-byte spacing DIR and OUTPUT aliased
    // each other, so the direction write landed on the output value.
    var regs = fake.Flat(64){};
    std.mem.writeInt(u32, regs.buf[0x00..][0..4], 1 << 5, .little); // INPUT: pin 5 high
    var dev = harbor_gpio.bind(regs.mmio());
    const g = dev.gpio();

    g.direction(3, .output);
    try std.testing.expectEqual(@as(u32, 1 << 3), std.mem.readInt(u32, regs.buf[0x10..][0..4], .little)); // DIR
    g.set(3, true);
    try std.testing.expectEqual(@as(u32, 1 << 3), std.mem.readInt(u32, regs.buf[0x08..][0..4], .little)); // OUTPUT
    try std.testing.expect(g.get(5));
    try std.testing.expect(!g.get(4));

    // The direction write did not disturb the output register, and vice versa.
    dev.interruptEnable(7, true, true);
    try std.testing.expectEqual(@as(u32, 1 << 7), std.mem.readInt(u32, regs.buf[0x18..][0..4], .little)); // IRQ_EN
    try std.testing.expectEqual(@as(u32, 1 << 7), std.mem.readInt(u32, regs.buf[0x28..][0..4], .little)); // IRQ_EDGE
    try std.testing.expectEqual(@as(u32, 1 << 3), std.mem.readInt(u32, regs.buf[0x10..][0..4], .little)); // DIR intact
}

test "harbor_i2c: write phase acks and the last byte reaches DATA" {
    // Registers sit 8 bytes apart. STATUS is forced to CMD_DONE|ACK.
    var regs = fake.Flat(64){ .force_off = 0x08, .force_val = 0x22 };
    var dev = harbor_i2c.bind(regs.mmio(), .{ .prescale = 9 });
    const bus = dev.i2c();

    try std.testing.expect(bus.transfer(0x50, &.{ 0xab, 0xcd }, &.{}));
    try std.testing.expectEqual(@as(u32, 0xcd), std.mem.readInt(u32, regs.buf[0x10..][0..4], .little)); // DATA
    try std.testing.expectEqual(@as(u32, 9), std.mem.readInt(u32, regs.buf[0x20..][0..4], .little)); // PRESCALE
    try std.testing.expectEqual(@as(u32, 1), std.mem.readInt(u32, regs.buf[0x00..][0..4], .little)); // CTRL enable
}

test "harbor_i2c: a controller that never reports CMD_DONE fails the transfer" {
    // STATUS reads 0 forever, so no command ever completes. The driver must
    // give up rather than spin.
    var regs = fake.Flat(64){};
    var dev = harbor_i2c.bind(regs.mmio(), .{ .prescale = 9 });
    try std.testing.expect(!dev.i2c().transfer(0x50, &.{0xab}, &.{}));
}

test "harbor_spi: full-duplex loopback" {
    var regs = fake.Flat(64){}; // STATUS busy bit clear
    var dev = harbor_spi.bind(regs.mmio(), .{ .divider = 4, .mode = 0 });
    const bus = dev.spi();

    var rx: [3]u8 = undefined;
    bus.transfer(&.{ 1, 2, 3 }, &rx);
    try std.testing.expectEqualSlices(u8, &.{ 1, 2, 3 }, &rx); // DATA reads back what was written
    try std.testing.expectEqual(@as(u32, 4), std.mem.readInt(u32, regs.buf[0x18..][0..4], .little)); // DIVIDER
    try std.testing.expectEqual(@as(u32, 0), std.mem.readInt(u32, regs.buf[0x20..][0..4], .little)); // CS released
}

test "harbor_spi: a short tx clocks the idle byte, and CS picks its own line" {
    var regs = fake.Flat(64){};
    var dev = harbor_spi.bind(regs.mmio(), .{ .mode = 3, .cs_line = 2 });

    // Mode 3 is CPOL + CPHA, and the enable bit stays set.
    try std.testing.expectEqual(@as(u32, 0b111), std.mem.readInt(u32, regs.buf[0x00..][0..4], .little));

    dev.chipSelect(true);
    try std.testing.expectEqual(@as(u32, 1 << 2), std.mem.readInt(u32, regs.buf[0x20..][0..4], .little));

    // Read four bytes while writing one: the exchange runs for four, and the
    // three unwritten slots clock 0xFF rather than zero.
    var rx: [4]u8 = undefined;
    dev.spi().transfer(&.{0xa5}, &rx);
    try std.testing.expectEqualSlices(u8, &.{ 0xa5, 0xff, 0xff, 0xff }, &rx);
}

test "clint: timer and IPI registers" {
    var regs = fake.Flat(0xc000){};
    std.mem.writeInt(u64, regs.buf[0xbff8..][0..8], 0x9999, .little); // MTIME
    const dev = clint.bind(regs.mmio());

    try std.testing.expectEqual(@as(u64, 0x9999), dev.time());
    dev.setTimecmp(0, 0x1234_5678_9abc);
    try std.testing.expectEqual(@as(u64, 0x1234_5678_9abc), std.mem.readInt(u64, regs.buf[0x4000..][0..8], .little));
    dev.sendIpi(1);
    try std.testing.expectEqual(@as(u32, 1), std.mem.readInt(u32, regs.buf[0x0004..][0..4], .little)); // MSIP[1]
}

test "harbor_sdio: a dead controller yields an empty block device" {
    // A flat register file never drives a response, so every command reports a
    // timeout and identification must give up rather than report a phantom card.
    var regs = fake.Flat(128){};
    var dev = harbor_sdio.bind(regs.mmio(), .{ .freq = 50_000_000 });
    try std.testing.expect(!dev.cardPresent());

    const blk = dev.block();
    try std.testing.expectEqual(@as(u64, 0), blk.num_blocks);
    var buf: [512]u8 = undefined;
    try std.testing.expect(!blk.readBlocks(0, 1, &buf));
    try std.testing.expect(!blk.writeBlocks(0, 1, &buf));
}

test "plic: enable programs priority + enable bit, claim reads the claim reg" {
    var regs = fake.Sparse{};
    const dev = plic.bind(regs.mmio(), .{ .context = 0 });

    dev.enable(10);
    try std.testing.expectEqual(@as(u64, 1), regs.lastWrite(10 * 4).?); // priority
    try std.testing.expectEqual(@as(u64, 1 << 10), regs.lastWrite(0x2000).?); // enable bits

    regs.setRead(0x200004, 7); // claim/complete register
    try std.testing.expectEqual(@as(u32, 7), dev.claim().?);
}

test "gicv2: enable sets ISENABLER, claim/complete hit the CPU interface" {
    var dist = fake.Sparse{};
    var cpu = fake.Sparse{};
    const g = gicv2.bind(dist.mmio(), cpu.mmio());

    g.enable(33); // SPI 33 -> ISENABLER bank 1, bit 1
    try std.testing.expectEqual(@as(u64, 1 << 1), dist.lastWrite(0x100 + 4).?);

    cpu.setRead(0x0c, 42); // GICC_IAR
    try std.testing.expectEqual(@as(u32, 42), g.claim().?);
    g.complete(42);
    try std.testing.expectEqual(@as(u64, 42), cpu.lastWrite(0x10).?); // GICC_EOIR
}

test "gicv3: SPI via distributor, PPI via redistributor" {
    // Construct directly, not via bind. The init(), claim(), and complete() calls
    // touch the EL1-only ICC_* system registers. Those registers trap from a
    // userspace test. The enable paths are pure MMIO. This test verifies them.
    var dist = fake.Sparse{};
    var redist = fake.Sparse{};
    const g = gicv3.Gicv3{ .dist = dist.mmio(), .redist = redist.mmio() };

    g.enable(40); // SPI -> GICD_ISENABLER bank 1, bit 8
    try std.testing.expectEqual(@as(u64, 1 << 8), dist.lastWrite(0x100 + 4).?);

    g.enable(5); // PPI -> GICR_ISENABLER0, bit 5
    try std.testing.expectEqual(@as(u64, 1 << 5), redist.lastWrite(0x10000 + 0x100).?);
}

test "goldfish_rtc: read nanoseconds and convert to a date" {
    var regs = fake.Flat(64){};
    const ns: u64 = 1609459200 * 1_000_000_000; // 2021-01-01 00:00:00 UTC
    std.mem.writeInt(u32, regs.buf[0x00..][0..4], @truncate(ns), .little);
    std.mem.writeInt(u32, regs.buf[0x04..][0..4], @truncate(ns >> 32), .little);

    var dev = goldfish_rtc.bind(regs.mmio());
    const dt = dev.rtc().now();
    try std.testing.expectEqual(@as(u16, 2021), dt.year);
    try std.testing.expectEqual(@as(u8, 1), dt.month);
    try std.testing.expectEqual(@as(u8, 1), dt.day);
}

test "pl031: read seconds, and set writes the load register" {
    var regs = fake.Flat(64){};
    std.mem.writeInt(u32, regs.buf[0x00..][0..4], 1609459200, .little); // DR
    var dev = pl031.bind(regs.mmio());

    const r = dev.rtc();
    try std.testing.expectEqual(@as(u16, 2021), r.now().year);

    _ = r.set(.{ .year = 2000, .month = 1, .day = 1, .hour = 0, .minute = 0, .second = 0 });
    try std.testing.expectEqual(@as(u32, 946684800), std.mem.readInt(u32, regs.buf[0x08..][0..4], .little)); // LR = Y2K epoch
}

// A Harbor I2C controller that records the CMD sequence a transfer issues and
// answers each command the way the hardware does: cmd_done plus an acknowledge,
// and a byte in DATA for a read.
const MockI2c = struct {
    const STATUS = 0x08;
    const DATA = 0x10;
    const CMD = 0x28;

    const ST_ACK: u32 = 1 << 1;
    const ST_ARB_LOST: u32 = 1 << 2;
    const ST_CMD_DONE: u32 = 1 << 5;
    const ST_CMD_REJECTED: u32 = 1 << 6;

    const CMD_START: u32 = 1 << 0;
    const CMD_STOP: u32 = 1 << 1;
    const CMD_WRITE: u32 = 1 << 2;
    const CMD_READ: u32 = 1 << 3;
    const CMD_NACK: u32 = 1 << 4;

    cmds: [32]u32 = @splat(0),
    ncmd: usize = 0,
    /// The byte DATA held at each START, which is the addressed slave plus the
    /// read/write bit.
    addrs: [8]u8 = @splat(0),
    naddr: usize = 0,
    data: u32 = 0,
    rx: []const u8 = &.{},
    rx_i: usize = 0,
    status: u32 = 0,
    /// Refuse every command, the way a controller does when CMD arrives while
    /// it is busy.
    reject: bool = false,
    /// Refuse only the first command, then behave.
    reject_once: bool = false,
    /// Withhold the acknowledge on the nth byte the master sends, counting the
    /// address byte as 0.
    nack_write: ?usize = null,
    nwrite: usize = 0,
    /// Report a lost arbitration on every command.
    arb_lost: bool = false,
    /// Report a lost arbitration on one command only, counting from 0.
    arb_lost_at: ?usize = null,
    status_reads: usize = 0,

    fn rd(ctx: ?*anyopaque, off: usize, width: Mmio.Width) u64 {
        _ = width;
        const self: *MockI2c = @ptrCast(@alignCast(ctx));
        return switch (off) {
            STATUS => blk: {
                self.status_reads += 1;
                break :blk self.status;
            },
            DATA => self.data,
            else => 0,
        };
    }

    fn wr(ctx: ?*anyopaque, off: usize, width: Mmio.Width, val: u64) void {
        _ = width;
        const self: *MockI2c = @ptrCast(@alignCast(ctx));
        switch (off) {
            // Any write clears the two sticky bits and leaves the rest.
            STATUS => self.status &= ~(ST_CMD_DONE | ST_CMD_REJECTED),
            DATA => self.data = @truncate(val),
            CMD => self.run(@truncate(val)),
            else => {},
        }
    }

    fn run(self: *MockI2c, command: u32) void {
        if (self.reject or self.reject_once) {
            self.reject_once = false;
            self.status |= ST_CMD_REJECTED;
            return;
        }
        if (self.ncmd < self.cmds.len) {
            self.cmds[self.ncmd] = command;
            self.ncmd += 1;
        }
        if (command & CMD_START != 0 and self.naddr < self.addrs.len) {
            self.addrs[self.naddr] = @truncate(self.data);
            self.naddr += 1;
        }

        self.status = ST_CMD_DONE;
        if (self.arb_lost or self.arb_lost_at == self.ncmd - 1) self.status |= ST_ARB_LOST;

        if (command & CMD_READ != 0) {
            self.data = if (self.rx_i < self.rx.len) self.rx[self.rx_i] else 0xFF;
            self.rx_i += 1;
            return;
        }
        if (command & CMD_WRITE != 0) {
            const n = self.nwrite;
            self.nwrite += 1;
            if (self.nack_write != n) self.status |= ST_ACK;
            return;
        }
        self.status |= ST_ACK;
    }

    fn issued(self: *const MockI2c) []const u32 {
        return self.cmds[0..self.ncmd];
    }

    fn mmio(self: *MockI2c) Mmio {
        return .{ .ctx = self, .base = 0, .read_fn = rd, .write_fn = wr };
    }
};

test "harbor_i2c: a read NACKs its last byte and ACKs the rest" {
    var regs = MockI2c{ .rx = &.{ 0x11, 0x22, 0x33 } };
    var dev = harbor_i2c.bind(regs.mmio(), .{});

    var rx: [3]u8 = undefined;
    try std.testing.expect(dev.i2c().transfer(0x50, &.{}, &rx));
    try std.testing.expectEqualSlices(u8, &.{ 0x11, 0x22, 0x33 }, &rx);

    const C = MockI2c;
    try std.testing.expectEqualSlices(u32, &.{
        C.CMD_START | C.CMD_WRITE, // address byte, read direction
        C.CMD_READ,
        C.CMD_READ,
        C.CMD_READ | C.CMD_NACK, // the slave releases SDA before the STOP
        C.CMD_STOP,
    }, regs.issued());
}

test "harbor_i2c: a one-byte read NACKs that single byte" {
    var regs = MockI2c{ .rx = &.{0xa5} };
    var dev = harbor_i2c.bind(regs.mmio(), .{});

    var rx: [1]u8 = undefined;
    try std.testing.expect(dev.i2c().transfer(0x50, &.{}, &rx));
    try std.testing.expectEqual(@as(u8, 0xa5), rx[0]);
    try std.testing.expectEqual(MockI2c.CMD_READ | MockI2c.CMD_NACK, regs.issued()[1]);
}

test "harbor_i2c: write then read joins the phases with a repeated START" {
    var regs = MockI2c{ .rx = &.{0x7e} };
    var dev = harbor_i2c.bind(regs.mmio(), .{});

    var rx: [1]u8 = undefined;
    try std.testing.expect(dev.i2c().transfer(0x50, &.{0x0c}, &rx));
    try std.testing.expectEqual(@as(u8, 0x7e), rx[0]);

    const C = MockI2c;
    // One STOP, at the end. The second START is a repeated one, so the device
    // keeps the register pointer the write phase set.
    try std.testing.expectEqualSlices(u32, &.{
        C.CMD_START | C.CMD_WRITE, // address, write direction
        C.CMD_WRITE, // register index 0x0c
        C.CMD_START | C.CMD_WRITE, // repeated START, address, read direction
        C.CMD_READ | C.CMD_NACK,
        C.CMD_STOP,
    }, regs.issued());
}

test "harbor_i2c: the address byte carries the read/write bit" {
    var regs = MockI2c{ .rx = &.{0} };
    var dev = harbor_i2c.bind(regs.mmio(), .{});

    var rx: [1]u8 = undefined;
    try std.testing.expect(dev.i2c().transfer(0x50, &.{0x01}, &rx));

    // The write phase addresses the slave with bit 0 clear and the read phase
    // with bit 0 set. Both name the same slave.
    try std.testing.expectEqualSlices(u8, &.{ 0x50 << 1, (0x50 << 1) | 1 }, regs.addrs[0..regs.naddr]);
}

test "harbor_i2c: an address NACK fails the transfer and still sends a STOP" {
    var regs = MockI2c{ .nack_write = 0 };
    var dev = harbor_i2c.bind(regs.mmio(), .{});

    try std.testing.expect(!dev.i2c().transfer(0x50, &.{0xab}, &.{}));
    // The bus is released even though the address went unanswered.
    try std.testing.expectEqual(MockI2c.CMD_STOP, regs.issued()[regs.ncmd - 1]);
}

test "harbor_i2c: a data NACK stops the write early" {
    var regs = MockI2c{ .nack_write = 2 }; // address, first byte, then a NACK
    var dev = harbor_i2c.bind(regs.mmio(), .{});

    try std.testing.expect(!dev.i2c().transfer(0x50, &.{ 0x01, 0x02, 0x03 }, &.{}));
    const C = MockI2c;
    try std.testing.expectEqualSlices(u32, &.{
        C.CMD_START | C.CMD_WRITE,
        C.CMD_WRITE,
        C.CMD_WRITE, // unacknowledged, so the third byte never goes out
        C.CMD_STOP,
    }, regs.issued());
}

test "harbor_i2c: a lost arbitration fails both a write and a read" {
    var wr_regs = MockI2c{ .arb_lost = true };
    var wr_dev = harbor_i2c.bind(wr_regs.mmio(), .{});
    try std.testing.expect(!wr_dev.i2c().transfer(0x50, &.{0xab}, &.{}));

    var rx: [1]u8 = undefined;
    var rd_regs = MockI2c{ .arb_lost = true, .rx = &.{0x5a} };
    var rd_dev = harbor_i2c.bind(rd_regs.mmio(), .{});
    try std.testing.expect(!rd_dev.i2c().transfer(0x50, &.{}, &rx));
}

test "harbor_i2c: an empty transfer touches no command" {
    var regs = MockI2c{};
    var dev = harbor_i2c.bind(regs.mmio(), .{});
    try std.testing.expect(dev.i2c().transfer(0x50, &.{}, &.{}));
    try std.testing.expectEqual(@as(usize, 0), regs.ncmd);
}

test "harbor_i2c: prescaleFor holds the minimum SCL low time of each mode" {
    // SCL low is 2 * (prescale + 1) input clock cycles. At 50 MHz one cycle is
    // 20 ns, so standard mode needs a low time of at least 4700 ns.
    const p = harbor_i2c.prescaleFor(50_000_000, 100_000);
    try std.testing.expect(2 * (@as(u64, p) + 1) * 20 >= 4700);

    // Fast mode asks for 1300 ns and fast mode plus for 500 ns, so each step up
    // in bus speed lowers the divider.
    const fast = harbor_i2c.prescaleFor(50_000_000, 400_000);
    const fast_plus = harbor_i2c.prescaleFor(50_000_000, 1_000_000);
    try std.testing.expect(fast < p);
    try std.testing.expect(fast_plus < fast);
    try std.testing.expect(2 * (@as(u64, fast) + 1) * 20 >= 1300);
    try std.testing.expect(2 * (@as(u64, fast_plus) + 1) * 20 >= 500);

    // A clock too slow to need any division still divides by one, and the
    // fastest clock a u32 can hold stays inside the 16-bit field.
    try std.testing.expectEqual(@as(u32, 1), harbor_i2c.prescaleFor(1000, 100_000));
    try std.testing.expect(harbor_i2c.prescaleFor(0xFFFF_FFFF, 100_000) <= 0xFFFF);
}

test "harbor_i2c: a rejected command fails the transfer and clears the sticky bit" {
    // A controller refuses a CMD that arrives while it is busy, or before CTRL
    // enable is set, and raises cmd_rejected instead of cmd_done. The driver
    // must notice the refusal and clear the bit, because a bit left set hides
    // the result of the next command.
    var regs = MockI2c{ .reject_once = true };
    var dev = harbor_i2c.bind(regs.mmio(), .{});

    try std.testing.expect(!dev.i2c().transfer(0x50, &.{0xab}, &.{}));
    try std.testing.expectEqual(@as(u32, 0), regs.status & MockI2c.ST_CMD_REJECTED);

    // With the bit cleared the controller is usable again.
    var next = MockI2c{};
    var next_dev = harbor_i2c.bind(next.mmio(), .{});
    try std.testing.expect(next_dev.i2c().transfer(0x50, &.{0xab}, &.{}));
}

test "harbor_i2c: a controller that refuses every command gives up at once" {
    // The refusal path must not fall through to the poll cap: a transfer that
    // can never start should cost a handful of register reads, not a million.
    var regs = MockI2c{ .reject = true };
    var dev = harbor_i2c.bind(regs.mmio(), .{});
    try std.testing.expect(!dev.i2c().transfer(0x50, &.{0xab}, &.{}));
    try std.testing.expect(regs.status_reads < 100);
}

test "harbor_i2c: a read that loses arbitration fails the transfer" {
    // The address phase goes through, then the data byte loses arbitration. The
    // read path has to check for that itself: on a read the acknowledge bit is
    // the master's own choice, so it cannot stand in for an error.
    var regs = MockI2c{ .arb_lost_at = 1, .rx = &.{0x5a} };
    var dev = harbor_i2c.bind(regs.mmio(), .{});

    var rx: [1]u8 = undefined;
    try std.testing.expect(!dev.i2c().transfer(0x50, &.{}, &rx));
}
