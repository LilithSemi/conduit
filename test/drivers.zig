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
const harbor_trng = @import("conduit").driver.harbor_trng;
const sf32lb52_audio = @import("conduit").driver.sf32lb52_audio;
const sf32lb52_dmac = @import("conduit").driver.sf32lb52_dmac;
const sf32lb52_efuse = @import("conduit").driver.sf32lb52_efuse;
const sf32lb52_gpio = @import("conduit").driver.sf32lb52_gpio;
const sf32lb52_i2c = @import("conduit").driver.sf32lb52_i2c;
const sf32lb52_lcdc = @import("conduit").driver.sf32lb52_lcdc;
const sf32lb52_mpi = @import("conduit").driver.sf32lb52_mpi;
const sf32lb52_pdm = @import("conduit").driver.sf32lb52_pdm;
const sf32lb52_pmuc = @import("conduit").driver.sf32lb52_pmuc;
const sf32lb52_rtc = @import("conduit").driver.sf32lb52_rtc;
const sf32lb52_hpsys_rcc = @import("conduit").driver.sf32lb52_hpsys_rcc;
const sf32lb52_hpsys_cfg = @import("conduit").driver.sf32lb52_hpsys_cfg;
const sf32lb52_hpsys_aon = @import("conduit").driver.sf32lb52_hpsys_aon;
const sf32lb52_pinmux = @import("conduit").driver.sf32lb52_pinmux;
const sf32lb52_spi = @import("conduit").driver.sf32lb52_spi;
const sf32lb52_trng = @import("conduit").driver.sf32lb52_trng;
const sf32lb52_tsen = @import("conduit").driver.sf32lb52_tsen;
const sf32lb52_timer = @import("conduit").driver.sf32lb52_timer;
const sf32lb52_usart = @import("conduit").driver.sf32lb52_usart;
const sf32lb52_wdt = @import("conduit").driver.sf32lb52_wdt;

const RecordingDelay = struct {
    calls: usize = 0,
    total_us: u32 = 0,

    pub fn waitMicroseconds(self: *RecordingDelay, microseconds: u32) void {
        self.calls += 1;
        self.total_us += microseconds;
    }
};

test "sf32lb52 audio: codec playback power, volume, FIFO, and DMA" {
    var codec_regs = fake.Sparse{};
    var dma_regs = fake.Sparse{};
    var delay = RecordingDelay{};
    var codec = try sf32lb52_audio.bindCodec(
        codec_regs.mmio(),
        sf32lb52_audio.Delay.from(&delay),
        .{ .sample_rate = .hz16000, .volume_db = -18 },
    );
    try std.testing.expectEqual(@as(u64, 0x114), codec_regs.lastWrite(0x10).?);
    try std.testing.expectEqual(@as(u64, 3), (codec_regs.lastWrite(0x30).? >> 8) & 0xf);

    codec.start();
    try std.testing.expectEqual(@as(u32, 2276), delay.total_us);
    try std.testing.expectEqual(@as(u64, 0x00e0_0001), codec_regs.lastWrite(0xc0).?);
    try codec.writeWord(0x1234_5678);
    try std.testing.expectEqual(@as(u64, 0x1234_5678), codec_regs.lastWrite(0x50).?);

    var dma = sf32lb52_dmac.bind(dma_regs.mmio(), 4);
    try codec.configureDma(&dma, .ch2, true);
    try codec.startDma(&dma, .ch2, 0x2000_4000, 1024);
    try std.testing.expectEqual(@as(u64, 41 << 8), dma_regs.lastWrite(0xa8).?);
    try std.testing.expectEqual(@as(u64, 0x5008_8050), dma_regs.lastWrite(0x24).?);
    try std.testing.expect(codec_regs.lastWrite(0x30).? & 0x80 != 0);

    codec.powerOff();
    try std.testing.expectEqual(@as(u64, 0), codec_regs.lastWrite(0x74).? & 0x81);
}

test "sf32lb52 audio: codec validation and AUDPRC straight-through DMA" {
    var codec_regs = fake.Sparse{};
    var prc_regs = fake.Sparse{};
    var dma_regs = fake.Sparse{};
    var delay = RecordingDelay{};
    var codec = try sf32lb52_audio.bindCodec(
        codec_regs.mmio(),
        sf32lb52_audio.Delay.from(&delay),
        .{ .volume_db = -36, .poll_limit = 2 },
    );
    try std.testing.expectError(error.InvalidVolume, codec.setVolume(55));
    codec_regs.setRead(0x18, 8);
    try std.testing.expectError(error.Timeout, codec.writeWord(0));

    var prc = sf32lb52_audio.bindAudprc(prc_regs.mmio(), .{ .dac_strobe_div = 1000 });
    try std.testing.expectEqual(@as(u64, 1000), prc_regs.lastWrite(0x08).?);
    try std.testing.expectEqual(@as(u64, 3 << 20), prc_regs.lastWrite(0x54).?);
    var dma = sf32lb52_dmac.bind(dma_regs.mmio(), 4);
    try prc.configureDma(&dma, .ch1, true);
    try prc.startDma(&dma, .ch1, 0x2000_8000, 512);
    try std.testing.expectEqual(@as(u64, 51), dma_regs.lastWrite(0xa8).? & 0x3f);
    try std.testing.expectEqual(@as(u64, 0x5000_5014), dma_regs.lastWrite(0x10).?);
    try std.testing.expect(prc_regs.lastWrite(0x04).? & 0x41 == 0x41);
}

test "sf32lb52 LCDC1: configures PT2 JDI timing and asynchronous scanout" {
    var registers = fake.Sparse{};
    var delay = RecordingDelay{};
    var lcdc = try sf32lb52_lcdc.bind(
        registers.mmio(),
        sf32lb52_lcdc.Delay.from(&delay),
        .{ .lcdc_clock_hz = 240_000_000 },
    );

    try std.testing.expectEqual(@as(u64, 0x01e7_0079), registers.lastWrite(0xd0).?);
    try std.testing.expectEqual(@as(u64, 0x00a1_00a1), registers.lastWrite(0xdc).?);
    try std.testing.expectEqual(@as(u64, 0x4cba_4cba), registers.lastWrite(0xe0).?);

    try lcdc.present(.{
        .address = 0x2001_0000,
        .stride_bytes = 960,
    }, .{ .first_row = 10, .last_row = 19 });
    try std.testing.expectEqual(@as(u64, 0x2001_2580), registers.lastWrite(0x2c).?);
    try std.testing.expectEqual(@as(u64, 0x001d_00ef), registers.lastWrite(0x14).?);
    try std.testing.expect(registers.lastWrite(0x1c).? & 0x1c00_0000 == 0x1c00_0000);
    try std.testing.expectEqual(@as(u32, 36), delay.total_us);

    registers.setRead(0x08, 0x0010_0000);
    try std.testing.expectEqual(sf32lb52_lcdc.Event.frame_consumed, lcdc.handleInterrupt());
    try std.testing.expect(!lcdc.available());

    registers.setRead(0x08, 0x0010_0000);
    try std.testing.expectEqual(sf32lb52_lcdc.Event.none, lcdc.handleInterrupt());
    registers.setRead(0x08, 1);
    try std.testing.expectEqual(sf32lb52_lcdc.Event.complete, lcdc.handleInterrupt());
    try std.testing.expect(lcdc.available());
}

test "sf32lb52 LCDC1: validates scanout and reports underrun" {
    var registers = fake.Sparse{};
    var delay = RecordingDelay{};
    var lcdc = try sf32lb52_lcdc.bind(
        registers.mmio(),
        sf32lb52_lcdc.Delay.from(&delay),
        .{ .lcdc_clock_hz = 96_000_000 },
    );
    try std.testing.expectError(error.InvalidFrame, lcdc.present(.{
        .address = 3,
        .stride_bytes = 960,
    }, .{}));
    try std.testing.expectError(error.InvalidRegion, lcdc.present(.{
        .address = 0x2000_0000,
        .stride_bytes = 960,
    }, .{ .first_row = 42, .last_row = 41 }));

    try lcdc.present(.{ .address = 0x2000_0000, .stride_bytes = 960 }, .{});
    registers.setRead(0x08, 0x0020_0000);
    try std.testing.expectEqual(sf32lb52_lcdc.Event.underrun, lcdc.handleInterrupt());
    try std.testing.expect(lcdc.available());
    try std.testing.expectEqual(@as(u32, 0x6200_0000), sf32lb52_lcdc.systemBusAddress(0x1200_0000));
    try std.testing.expectEqual(@as(u32, 0x2000_0000), sf32lb52_lcdc.systemBusAddress(0x2000_0000));
}

test "sf32lb52 EFUSE: reads an unaligned range across bank boundaries" {
    var efuse_regs = fake.Sparse{};
    var pmuc_regs = fake.Sparse{};
    var delay = RecordingDelay{};
    efuse_regs.setRead(0x08, 1);
    efuse_regs.setRead(0x4c, 0xbbaa_2211);
    efuse_regs.setRead(0x50, 0x6655_4433);
    pmuc_regs.setRead(0x94, 9);

    var efuse = try sf32lb52_efuse.bind(
        efuse_regs.mmio(),
        pmuc_regs.mmio(),
        sf32lb52_efuse.Delay.from(&delay),
        .{ .pclk_hz = 48_000_000, .poll_limit = 4 },
    );
    var data: [6]u8 = undefined;
    try efuse.read(30, &data);

    try std.testing.expectEqualSlices(u8, &.{ 0xaa, 0xbb, 0x33, 0x44, 0x55, 0x66 }, &data);
    try std.testing.expect(efuse_regs.sawWrite(0x00, 1));
    try std.testing.expect(efuse_regs.sawWrite(0x00, (1 << 2) | 1));
    try std.testing.expect(pmuc_regs.sawWrite(0x94, 0xe));
    try std.testing.expectEqual(@as(u64, 9), pmuc_regs.lastWrite(0x94).?);
    try std.testing.expectEqual(@as(u32, 20), delay.total_us);
}

test "sf32lb52 EFUSE: validates ranges and restores voltage after timeout" {
    var efuse_regs = fake.Sparse{};
    var pmuc_regs = fake.Sparse{};
    var delay = RecordingDelay{};
    efuse_regs.setRead(0x08, 0);
    pmuc_regs.setRead(0x94, 0xc);
    var efuse = try sf32lb52_efuse.bind(
        efuse_regs.mmio(),
        pmuc_regs.mmio(),
        sf32lb52_efuse.Delay.from(&delay),
        .{ .pclk_hz = 96_000_000, .poll_limit = 2 },
    );

    var one: [1]u8 = undefined;
    try std.testing.expectError(error.OutOfRange, efuse.read(128, &one));
    try std.testing.expectError(error.Timeout, efuse.read(0, &one));
    try std.testing.expect(pmuc_regs.sawWrite(0x94, 0xf));
    try std.testing.expectEqual(@as(u64, 0xc), pmuc_regs.lastWrite(0x94).?);
}

test "sf32lb52 HPSYS_AON: sleep policy and wake pin routing" {
    var registers = fake.Sparse{};
    var aon = sf32lb52_hpsys_aon.bind(registers.mmio());
    aon.configureSleep(.standby, .{
        .hxt48 = true,
        .power = true,
        .power_down_ram = .{ true, false, true },
    });
    try std.testing.expectEqual(@as(u64, 0x146), registers.lastWrite(0x1c).?);
    try std.testing.expectEqual(@as(u64, 3), registers.lastWrite(0x00).?);

    try aon.configureWakePin(0, .rising);
    try std.testing.expectEqual(@as(u64, 2), registers.lastWrite(0x04).?);
    try std.testing.expectEqual(@as(u64, 0x8000_0100), registers.lastWrite(0x28).?);
    try std.testing.expectEqual(@as(u64, 0x100), registers.lastWrite(0x20).?);
    try std.testing.expectError(error.InvalidWakePin, aon.configureWakePin(5, .low));

    aon.enableWake(.rtc, true);
    try std.testing.expectEqual(@as(u64, 0x101), registers.lastWrite(0x20).?);
    aon.clearWakeStatus(0x101);
    try std.testing.expectEqual(@as(u64, 0x8000_0101), registers.lastWrite(0x28).?);
}

test "sf32lb52 HPSYS_AON: oscillator and LCPU requests are bounded" {
    var ready_registers = fake.Sparse{};
    ready_registers.setRead(0x10, 0x8000_0000);
    ready_registers.setRead(0x2c, 0x20);
    var ready = sf32lb52_hpsys_aon.bind(ready_registers.mmio());
    try ready.requestOscillator(.hxt48, 2);
    try ready.requestLcpu(2);

    var stalled_registers = fake.Sparse{};
    stalled_registers.setRead(0x10, 0);
    stalled_registers.setRead(0x2c, 0);
    var stalled = sf32lb52_hpsys_aon.bind(stalled_registers.mmio());
    try std.testing.expectError(error.Timeout, stalled.requestOscillator(.hrc48, 2));
    try std.testing.expectError(error.Timeout, stalled.requestLcpu(2));
}

test "sf32lb52 PMUC: deep wake, boot state, and regulator control" {
    var registers = fake.Flat(0x100){};
    var pmuc = sf32lb52_pmuc.bind(registers.mmio());

    try pmuc.configureWakePin(1, 20, .falling);
    try std.testing.expectEqual(@as(u32, (3 << 7) | (20 << 15)), registers.mmio().read(u32, 0x00));
    try std.testing.expectEqual(@as(u32, 1 << 4), registers.mmio().read(u32, 0x04));
    try std.testing.expectEqual(@as(u32, 1 << 4), registers.mmio().read(u32, 0x0c));
    try std.testing.expectError(error.InvalidPin, pmuc.configureWakePin(2, 0, .high));
    try std.testing.expectError(error.InvalidAonPin, pmuc.configureWakePin(0, 5, .high));

    pmuc.enableWake(.rtc, true);
    pmuc.setLowPowerClock(.xtal32);
    pmuc.setPinRetention(true);
    pmuc.setHibernateRequest(true);
    try std.testing.expectEqual(sf32lb52_pmuc.BootMode.hibernate, pmuc.bootReason().mode);
    pmuc.setHpsysVout(0xd);
    try std.testing.expectEqual(@as(u4, 0xd), pmuc.hpsysVout());
    pmuc.requestReboot();
    try std.testing.expect(registers.mmio().read(u32, 0x00) & (1 << 2) != 0);
}

test "sf32lb52 RTC: coherent calendar, setting, alarms, and wakeup" {
    var rtc_regs = fake.Sparse{};
    var cfg_regs = fake.Sparse{};
    const flags: u32 = (1 << 9) | 1 | (1 << 2);
    rtc_regs.setRead(0x0c, flags);
    cfg_regs.setRead(0x14, (@as(u32, 0x12) << 25) | (@as(u32, 0x34) << 18) | (@as(u32, 0x56) << 11));
    cfg_regs.setRead(0x18, (@as(u32, 0x26) << 16) | (@as(u32, 0x11) << 8) | 0x03);
    var rtc = sf32lb52_rtc.bind(rtc_regs.mmio(), cfg_regs.mmio(), 4);
    const now = rtc.rtc().now();
    try std.testing.expectEqual(@as(u16, 2026), now.year);
    try std.testing.expectEqual(@as(u8, 11), now.month);
    try std.testing.expectEqual(@as(u8, 3), now.day);
    try std.testing.expectEqual(@as(u8, 12), now.hour);
    try std.testing.expectEqual(@as(u8, 34), now.minute);
    try std.testing.expectEqual(@as(u8, 56), now.second);

    const value = @import("conduit").device.Rtc.DateTime{ .year = 2027, .month = 2, .day = 4, .hour = 5, .minute = 6, .second = 7 };
    try rtc.setChecked(value);
    try std.testing.expectEqual(@as(u64, (@as(u32, 0x05) << 25) | (@as(u32, 0x06) << 18) | (@as(u32, 0x07) << 11)), rtc_regs.lastWrite(0x00).?);
    try std.testing.expectEqual(@as(u64, (@as(u32, 0x27) << 16) | (@as(u32, 0x02) << 8) | 0x04), rtc_regs.lastWrite(0x04).?);
    try rtc.setAlarm(value);
    try std.testing.expectEqual(@as(u64, (1 << 8) | (1 << 11)), rtc_regs.lastWrite(0x08).?);
    try rtc.setWakeup(100, .seconds);
    try std.testing.expectEqual(@as(u64, 100), rtc_regs.lastWrite(0x14).?);
    try std.testing.expect(rtc_regs.lastWrite(0x08).? & ((1 << 9) | (1 << 12)) != 0);
}

test "sf32lb52 RTC: backup registers and bounded initialization" {
    var rtc_regs = fake.Sparse{};
    var cfg_regs = fake.Sparse{};
    var rtc = sf32lb52_rtc.bind(rtc_regs.mmio(), cfg_regs.mmio(), 2);
    try std.testing.expect(rtc.writeBackup(9, 0xfeedbeef));
    try std.testing.expectEqual(@as(u32, 0xfeedbeef), rtc.readBackup(9).?);
    try std.testing.expect(!rtc.writeBackup(10, 0));
    try std.testing.expectError(error.Timeout, rtc.setChecked(.{ .year = 2025, .month = 1, .day = 1, .hour = 0, .minute = 0, .second = 0 }));
    try std.testing.expectError(error.InvalidDateTime, rtc.setChecked(.{ .year = 2025, .month = 13, .day = 1, .hour = 0, .minute = 0, .second = 0 }));
}

test "sf32lb52 timers: GPTIM PWM timing, polarity, and channels" {
    var regs = fake.Sparse{};
    var timer = sf32lb52_timer.bindPwm(regs.mmio(), 48_000_000, 0xffff, false);
    try timer.pwm().configure(2, .{ .period_ns = 1_000_000, .active_ns = 250_000, .polarity = .active_low });
    try std.testing.expectEqual(@as(u64, 0), regs.lastWrite(0x28).?);
    try std.testing.expectEqual(@as(u64, 47_999), regs.lastWrite(0x2c).?);
    try std.testing.expectEqual(@as(u64, 12_000), regs.lastWrite(0x3c).?);
    try std.testing.expectEqual(@as(u64, (6 << 20) | (1 << 19)), regs.lastWrite(0x1c).?);
    try std.testing.expectEqual(@as(u64, 1 << 9), regs.lastWrite(0x20).?);
    try timer.pwm().enable(2, true);
    try std.testing.expectEqual(@as(u64, (1 << 9) | (1 << 8)), regs.lastWrite(0x20).?);
    try std.testing.expectError(error.InvalidChannel, timer.pwm().enable(4, true));
}

test "sf32lb52 timers: ATIM main output and basic timer events" {
    var atim_regs = fake.Sparse{};
    var atim = sf32lb52_timer.bindPwm(atim_regs.mmio(), 96_000_000, 0xffffffff, true);
    try atim.pwm().configure(0, .{ .period_ns = 20_000, .active_ns = 10_000 });
    try atim.pwm().enable(0, true);
    try std.testing.expectEqual(@as(u64, 1 << 15), atim_regs.lastWrite(0x44).?);
    var basic_regs = fake.Sparse{};
    var basic = sf32lb52_timer.bindBasic(basic_regs.mmio(), 48_000_000);
    try basic.configure(1_000_000_000, true);
    try std.testing.expectEqual(@as(u64, 47_999_999), basic_regs.lastWrite(0x2c).?);
    basic.enableUpdateInterrupt(true);
    basic.enableUpdateDma(true);
    try std.testing.expectEqual(@as(u64, 0x101), basic_regs.lastWrite(0x0c).?);
    basic_regs.setRead(0x10, 1);
    try std.testing.expect(basic.pending());
    basic.acknowledge();
    try std.testing.expectEqual(@as(u64, 0), basic_regs.lastWrite(0x10).?);
}

test "sf32lb52 PDM1: stereo microphone setup and DMA capture" {
    var pdm_regs = fake.Sparse{};
    var dma_regs = fake.Sparse{};
    var pdm = sf32lb52_pdm.bind(pdm_regs.mmio(), .{ .sample_rate = .hz48000, .depth = .bits16 });
    try std.testing.expectEqual(@as(u64, (1 << 8) | (1 << 7) | (1 << 6) | (1 << 2) | (1 << 1)), pdm_regs.lastWrite(0x00).?);
    try std.testing.expectEqual(@as(u64, 32), pdm_regs.lastWrite(0x08).?);
    try std.testing.expectEqual(@as(u64, 3), pdm_regs.lastWrite(0x38).?);
    try std.testing.expectEqual(@as(u64, 0x1326), pdm_regs.lastWrite(0x18).?);

    var dma = sf32lb52_dmac.bind(dma_regs.mmio(), 10);
    try pdm.configureDma(&dma, .ch3, .pdm1_left, true);
    try pdm.startDma(&dma, .ch3, 0x20002000, 1024);
    try std.testing.expectEqual(@as(u64, 0x5009a03c), dma_regs.lastWrite(0x38).?);
    try std.testing.expectEqual(@as(u64, 0x20002000), dma_regs.lastWrite(0x3c).?);
    try std.testing.expectEqual(@as(u64, 256), dma_regs.lastWrite(0x34).?);
    try std.testing.expect(pdm_regs.lastWrite(0x00).? & 1 != 0);
}

test "sf32lb52 PDM1: polling data and overflow errors" {
    var regs = fake.Sparse{};
    var pdm = sf32lb52_pdm.bind(regs.mmio(), .{ .channels = .right, .poll_limit = 2 });
    regs.setRead(0x44, 0);
    regs.setRead(0x40, 0x12345678);
    try std.testing.expectEqual(@as(u32, 0x12345678), try pdm.readWord(.right));
    regs.setRead(0x48, 1);
    try std.testing.expectError(error.OverflowRight, pdm.readWord(.right));
    pdm.clearOverflow();
    try std.testing.expectEqual(@as(u64, 3), regs.lastWrite(0x50).?);
}

test "sf32lb52 MPI2: indirect commands and GD25Q256E mapped reads" {
    var regs = fake.Sparse{};
    var mpi = sf32lb52_mpi.bind(regs.mmio(), .{ .prescaler = 3 });
    try std.testing.expectEqual(@as(u64, 3), regs.lastWrite(0x0c).?);
    try std.testing.expectEqual(@as(u64, 0x1ffff), regs.lastWrite(0x88).?);
    regs.setRead(0x10, 1);
    regs.setRead(0x54, 0);
    regs.setRead(0x04, 0xc8);
    const id = try mpi.readJedecId();
    try std.testing.expectEqualSlices(u8, &.{ 0xc8, 0xc8, 0xc8 }, &id);
    try std.testing.expectEqual(@as(u64, 2), regs.lastWrite(0x24).?);
    try std.testing.expectEqual(@as(u64, (1 << 18) | 1), regs.lastWrite(0x28).?);
    try std.testing.expectEqual(@as(u64, 0x9f), regs.lastWrite(0x18).?);

    try mpi.execute(.{ .opcode = 0x34, .address = 0x12345678, .address_bytes = 4, .address_mode = .single, .data_mode = .quad, .direction = .write }, &.{ 0xaa, 0x55 }, &.{});
    try std.testing.expect(regs.sawWrite(0x04, 0xaa));
    try std.testing.expect(regs.sawWrite(0x04, 0x55));
    mpi.configureGd25q256eMappedRead();
    try std.testing.expectEqual(@as(u64, 0xec), regs.lastWrite(0x40).? & 0xff);
    const expected: u64 = (3 << 18) | (4 << 13) | (3 << 8) | (3 << 6) | (3 << 3) | 1;
    try std.testing.expectEqual(expected, regs.lastWrite(0x48).?);
    try std.testing.expect(regs.lastWrite(0x00).? & ((1 << 20) | 1) == (1 << 20) | 1);
}

test "sf32lb52 MPI2: bounded command timeout" {
    var regs = fake.Sparse{};
    var mpi = sf32lb52_mpi.bind(regs.mmio(), .{ .poll_limit = 2 });
    try std.testing.expectError(error.Timeout, mpi.enterFourByteAddressMode());
}

test "sf32lb52 DMAC1: configure, start, status, and completion" {
    var regs = fake.Sparse{};
    var dma = sf32lb52_dmac.bind(regs.mmio(), 4);
    try dma.configure(.ch2, .{
        .request = .spi1_tx,
        .direction = .memory_to_peripheral,
        .memory_width = .word,
        .peripheral_width = .byte,
        .priority = .high,
        .transfer_interrupt = true,
    });
    const ccr: u32 = (2 << 12) | (2 << 10) | (1 << 7) | (1 << 4) | (1 << 3) | (1 << 1);
    try std.testing.expectEqual(@as(u64, ccr), regs.lastWrite(0x1c).?);
    try std.testing.expectEqual(@as(u64, 28 << 8), regs.lastWrite(0xa8).?);

    try dma.start(.ch2, 0x50095010, 0x20001000, 64);
    try std.testing.expectEqual(@as(u64, 0xf0), regs.lastWrite(0x04).?);
    try std.testing.expectEqual(@as(u64, 64), regs.lastWrite(0x20).?);
    try std.testing.expectEqual(@as(u64, 0x50095010), regs.lastWrite(0x24).?);
    try std.testing.expectEqual(@as(u64, 0x20001000), regs.lastWrite(0x28).?);
    try std.testing.expectEqual(@as(u64, ccr | 1), regs.lastWrite(0x1c).?);

    regs.setRead(0x00, 1 << 5);
    try std.testing.expect(dma.status(.ch2).complete);
    try dma.poll(.ch2);
    try std.testing.expectEqual(@as(u64, 0xf0), regs.lastWrite(0x04).?);
}

test "sf32lb52 DMAC1: errors and upper request mux" {
    var regs = fake.Sparse{};
    var dma = sf32lb52_dmac.bind(regs.mmio(), 2);
    try dma.configure(.ch8, .{ .request = .sdmmc1, .direction = .peripheral_to_memory });
    try std.testing.expectEqual(@as(u64, 57 << 24), regs.lastWrite(0xac).?);
    try std.testing.expectError(error.InvalidCount, dma.start(.ch8, 0, 0, 0));
    regs.setRead(0x00, @as(u32, 1) << 31);
    try std.testing.expectError(error.TransferError, dma.poll(.ch8));
}

test "sf32lb52 SPI: mode, divider, full-duplex transfer, and DMA requests" {
    var regs = fake.Sparse{};
    var controller = try sf32lb52_spi.bind(regs.mmio(), .{ .prescaler = 4, .mode = 3 });
    try std.testing.expectEqual(@as(u64, 0x104), regs.lastWrite(0x3c).?);
    try std.testing.expectEqual(@as(u64, (7 << 5) | (1 << 10) | (1 << 11)), regs.lastWrite(0x00).?);

    regs.setRead(0x14, (1 << 6) | (1 << 14));
    regs.setRead(0x10, 0xa5);
    var rx: [3]u8 = undefined;
    try controller.transferChecked(&.{ 1, 2 }, &rx);
    try std.testing.expectEqualSlices(u8, &.{ 0xa5, 0xa5, 0xa5 }, &rx);
    try std.testing.expect(regs.sawWrite(0x10, 1));
    try std.testing.expect(regs.sawWrite(0x10, 2));
    try std.testing.expect(regs.sawWrite(0x10, 0xff));
    try std.testing.expect(controller.lastError() == null);
    controller.configureDma(true, true);
    try std.testing.expectEqual(@as(u64, (1 << 10) | (1 << 11)), regs.lastWrite(0x04).?);
}

test "sf32lb52 SPI: checked timeout is retained through generic contract" {
    var regs = fake.Sparse{};
    var controller = try sf32lb52_spi.bind(regs.mmio(), .{ .poll_limit = 2 });
    try std.testing.expectError(error.Timeout, controller.transferChecked(&.{1}, &.{}));
    controller.spi().transfer(&.{1}, &.{});
    try std.testing.expectEqual(error.Timeout, controller.lastError().?);
}

test "sf32lb52 I2C: timing and combined write/read transfer" {
    var regs = fake.Sparse{};
    var controller = try sf32lb52_i2c.bind(regs.mmio(), .{});

    try std.testing.expectEqual(@as(u64, 9), regs.lastWrite(0x00).?);
    try std.testing.expectEqual(@as(u64, 0x73ff), regs.lastWrite(0x18).?);
    try std.testing.expectEqual(@as(u64, 25), regs.lastWrite(0x1c).?);

    regs.setRead(0x0c, (1 << 6) | (1 << 7));
    regs.setRead(0x10, 0x5a);
    var received: [2]u8 = undefined;
    try controller.transferChecked(0x50, &.{ 0x12, 0x34 }, &received);
    try std.testing.expectEqualSlices(u8, &.{ 0x5a, 0x5a }, &received);
    try std.testing.expectEqual(@as(u64, 0xa1), regs.lastWrite(0x10).?);
    try std.testing.expectEqual(@as(u64, 0x0d), regs.lastWrite(0x04).?);
    try std.testing.expect(controller.i2c().transfer(0x50, &.{}, &.{}));
}

test "sf32lb52 I2C: 10-bit addressing and explicit errors" {
    var regs = fake.Sparse{};
    var controller = try sf32lb52_i2c.bind(regs.mmio(), .{ .poll_limit = 2 });
    regs.setRead(0x0c, (1 << 6) | (1 << 7));
    regs.setRead(0x10, 0x33);
    var received: [1]u8 = undefined;
    try controller.transferChecked(0x2aa, &.{}, &received);
    try std.testing.expect(regs.sawWrite(0x10, 0xf4));
    try std.testing.expect(regs.sawWrite(0x10, 0xaa));
    try std.testing.expect(regs.sawWrite(0x10, 0xf5));
    try std.testing.expectEqual(@as(u8, 0x33), received[0]);
    try std.testing.expectError(error.InvalidAddress, controller.transferChecked(0x400, &.{}, &.{}));
    try std.testing.expectEqual(error.InvalidAddress, controller.lastError().?);

    var nack_regs = fake.Sparse{};
    var nack = try sf32lb52_i2c.bind(nack_regs.mmio(), .{});
    nack_regs.setRead(0x0c, 1 << 10);
    try std.testing.expectError(error.Nack, nack.transferChecked(0x50, &.{1}, &.{}));
    try std.testing.expectEqual(error.Nack, nack.lastError().?);
    try std.testing.expect(!nack.i2c().transfer(0x50, &.{1}, &.{}));

    var busy_regs = fake.Sparse{};
    var busy = try sf32lb52_i2c.bind(busy_regs.mmio(), .{ .poll_limit = 2 });
    busy_regs.setRead(0x0c, 1 << 3);
    try std.testing.expectError(error.BusBusy, busy.transferChecked(0x50, &.{}, &.{}));
}

test "sf32lb52 I2C: hardware bus recovery" {
    var regs = fake.Sparse{};
    var controller = try sf32lb52_i2c.bind(regs.mmio(), .{});
    regs.setRead(0x00, 9);
    try controller.recoverBus(9);
    try std.testing.expectEqual(@as(u64, 9), regs.lastWrite(0x20).?);
    try std.testing.expectEqual(@as(u64, 0x40000009), regs.lastWrite(0x00).?);
}

test "harbor TRNG implements the checked entropy contract" {
    var regs = fake.Sparse{};
    var trng = harbor_trng.bind(regs.mmio());
    var bytes: [6]u8 = undefined;

    try std.testing.expectError(error.NotReady, trng.entropy().fill(&bytes));
    regs.setRead(0x08, 0x3);
    try std.testing.expectError(error.HealthTestFailed, trng.entropy().fill(&bytes));

    regs.setRead(0x08, 0x1);
    regs.setRead(0x00, 0x44332211);
    try trng.entropy().fill(&bytes);
    try std.testing.expectEqualSlices(u8, &.{ 0x11, 0x22, 0x33, 0x44, 0x11, 0x22 }, &bytes);
}

test "sf32lb52 RCC: atomic gates and asymmetric reset map" {
    var regs = fake.Sparse{};
    const rcc = sf32lb52_hpsys_rcc.bind(regs.mmio());

    try std.testing.expect(rcc.enable(.dmac1));
    try std.testing.expectEqual(@as(u64, 1), regs.lastWrite(0x10).?);
    try std.testing.expect(rcc.disable(.i2c4));
    try std.testing.expectEqual(@as(u64, 1 << 25), regs.lastWrite(0x1c).?);

    // USART1 is resettable but its clock is not gateable on SF32LB52.
    try std.testing.expect(!rcc.enable(.usart1));
    try std.testing.expect(rcc.assertReset(.usart1));
    try std.testing.expectEqual(@as(u64, 1 << 3), regs.lastWrite(0x00).?);
    try std.testing.expect(rcc.deassertReset(.usart1));
    try std.testing.expectEqual(@as(u64, 0), regs.lastWrite(0x00).?);
    try std.testing.expect(!rcc.assertReset(.secu1));

    regs.setRead(0x0c, 1 << 23);
    try std.testing.expect(rcc.isEnabled(.tsen));
    try std.testing.expect(rcc.isEnabled(.usart1));
}

test "sf32lb52 RCC: source selection and derived bus frequencies" {
    var regs = fake.Sparse{};
    const rcc = sf32lb52_hpsys_rcc.bind(regs.mmio());

    rcc.selectSystemSource(.dll1);
    rcc.selectPeripheralSource(.hxt48);
    rcc.selectMpiSource(.mpi2, .dll2);
    rcc.selectTickSource(.rtc);
    try std.testing.expectEqual(@as(u64, 3 | (2 << 6) | (1 << 12)), regs.lastWrite(0x20).?);

    rcc.configureBusDividers(3, .div2, .div8);
    try std.testing.expectEqual(@as(u64, 3 | (1 << 8) | (3 << 12)), regs.lastWrite(0x24).?);
    const clocks = sf32lb52_hpsys_rcc.Frequencies{ .dll1_hz = 288_000_000, .dll2_hz = 240_000_000 };
    try std.testing.expectEqual(@as(u32, 288_000_000), rcc.systemFrequency(clocks));
    try std.testing.expectEqual(@as(u32, 48_000_000), rcc.peripheralFrequency(clocks));
    try std.testing.expectEqual(@as(u32, 96_000_000), rcc.hclkFrequency(clocks));
    try std.testing.expectEqual(@as(u32, 48_000_000), rcc.pclkFrequency(.pclk1, clocks));
    try std.testing.expectEqual(@as(u32, 12_000_000), rcc.pclkFrequency(.pclk2, clocks));
}

test "sf32lb52 RCC: DLL bring-up validates, locks, and reports frequency" {
    var rcc_regs = fake.Sparse{};
    var cfg_regs = fake.Sparse{};
    var delay = RecordingDelay{};
    rcc_regs.setRead(0x30, 0x8000_0000);
    cfg_regs.setRead(0x88, 0xa0);
    const rcc = sf32lb52_hpsys_rcc.bindWithConfig(rcc_regs.mmio(), cfg_regs.mmio());

    try rcc.enableDll(.dll2, 240_000_000, sf32lb52_hpsys_rcc.Delay.from(&delay), 3);
    try std.testing.expect(rcc_regs.sawWrite(0x30, 0x1025));
    try std.testing.expectEqual(@as(u32, 10), delay.total_us);
    try std.testing.expectEqual(@as(u64, 0xa1), cfg_regs.lastWrite(0x88).?);
    try std.testing.expectError(
        error.InvalidFrequency,
        rcc.enableDll(.dll1, 250_000_000, sf32lb52_hpsys_rcc.Delay.from(&delay), 3),
    );

    var frequency_regs = fake.Sparse{};
    frequency_regs.setRead(0x2c, 1 | (11 << 2));
    const frequency_rcc = sf32lb52_hpsys_rcc.bind(frequency_regs.mmio());
    try std.testing.expectEqual(@as(u32, 288_000_000), frequency_rcc.dllFrequency(.dll1));
}

test "sf32lb52 RCC: a DLL that does not lock times out disabled" {
    var rcc_regs = fake.Sparse{};
    var delay = RecordingDelay{};
    rcc_regs.setRead(0x2c, 0);
    const rcc = sf32lb52_hpsys_rcc.bind(rcc_regs.mmio());
    try std.testing.expectError(
        error.Timeout,
        rcc.enableDll(.dll1, 192_000_000, sf32lb52_hpsys_rcc.Delay.from(&delay), 2),
    );
    try std.testing.expectEqual(@as(u64, 0x101c), rcc_regs.lastWrite(0x2c).?);
}

test "sf32lb52 TSEN: official power sequence and temperature conversion" {
    const FakeDelay = struct {
        waits: [8]u32 = undefined,
        count: usize = 0,

        pub fn waitMicroseconds(self: *@This(), us: u32) void {
            self.waits[self.count] = us;
            self.count += 1;
        }
    };

    var tsen_regs = fake.Sparse{};
    var cfg_regs = fake.Sparse{};
    var delay = FakeDelay{};
    cfg_regs.setRead(0x88, 0xa0);
    tsen_regs.setRead(0x08, 1 << 2);
    tsen_regs.setRead(0x04, 1_074);

    var sensor = try sf32lb52_tsen.bind(
        tsen_regs.mmio(),
        cfg_regs.mmio(),
        sf32lb52_tsen.Delay.from(&delay),
        .{ .pclk_hz = 48_000_000 },
    );
    try std.testing.expectEqual(@as(u64, 0xa1), cfg_regs.lastWrite(0x88).?);
    try std.testing.expectEqual(@as(i32, 25_000), try sensor.temperature().readMilliCelsius());
    try std.testing.expectEqualSlices(u32, &.{ 40, 20 }, delay.waits[0..delay.count]);
    try std.testing.expectEqual(@as(u64, 1), tsen_regs.lastWrite(0x08).?);
    try std.testing.expectEqual(@as(u64, 0x3f006), tsen_regs.lastWrite(0x00).?);

    sensor.deinit();
    try std.testing.expectEqual(@as(u64, 0xa0), cfg_regs.lastWrite(0x88).?);
}

test "sf32lb52 TSEN: timeout powers the analog block down" {
    const NoopDelay = struct {
        pub fn waitMicroseconds(_: *@This(), _: u32) void {}
    };
    var tsen_regs = fake.Sparse{};
    var cfg_regs = fake.Sparse{};
    var delay = NoopDelay{};
    var sensor = try sf32lb52_tsen.bind(
        tsen_regs.mmio(),
        cfg_regs.mmio(),
        sf32lb52_tsen.Delay.from(&delay),
        .{ .pclk_hz = 8_000_000, .poll_limit = 2, .manage_bandgap = false },
    );
    try std.testing.expectError(error.Timeout, sensor.temperature().readMilliCelsius());
    try std.testing.expectEqual(@as(u64, 0x10006), tsen_regs.lastWrite(0x00).?);
    try std.testing.expect(cfg_regs.lastWrite(0x88) == null);
}

test "sf32lb52 watchdog: reset-only and two-stage operation" {
    var regs = fake.Sparse{};
    var driver = sf32lb52_wdt.bind(regs.mmio(), 32_768);
    const watchdog = driver.watchdog();

    // One second maps exactly to 32768 counter ticks. CR reset-length bits are
    // preserved while response mode is selected.
    regs.setRead(0x08, 5);
    try watchdog.start(.{ .timeout_ns = 1_000_000_000 });
    try std.testing.expectEqual(@as(u64, 32_768), regs.lastWrite(0x00).?);
    try std.testing.expectEqual(@as(u64, 0), regs.lastWrite(0x04).?);
    try std.testing.expectEqual(@as(u64, 5), regs.lastWrite(0x08).?);
    try std.testing.expectEqual(@as(u64, 0x76), regs.lastWrite(0x0c).?);
    try std.testing.expectEqual(@as(u64, 0x58ab99fc), regs.lastWrite(0x18).?);

    try watchdog.start(.{ .timeout_ns = 250_000_000, .warning_ns = 500_000_000 });
    try std.testing.expectEqual(@as(u64, 8_192), regs.lastWrite(0x00).?);
    try std.testing.expectEqual(@as(u64, 16_384), regs.lastWrite(0x04).?);
    try std.testing.expectEqual(@as(u64, 5 | (1 << 4)), regs.lastWrite(0x08).?);

    watchdog.feed();
    try std.testing.expectEqual(@as(u64, 0x76), regs.lastWrite(0x0c).?);
    try std.testing.expect(watchdog.acknowledge());
    try std.testing.expectEqual(@as(u64, 1), regs.lastWrite(0x10).?);
    try watchdog.stop();
    try std.testing.expectEqual(@as(u64, 0x34), regs.lastWrite(0x0c).?);

    regs.setRead(0x14, 0x3);
    try std.testing.expect(driver.active());
    try std.testing.expect(driver.warningPending());
}

test "sf32lb52 watchdog rejects unrepresentable durations" {
    var regs = fake.Sparse{};
    var driver = sf32lb52_wdt.bind(regs.mmio(), 32_768);
    const watchdog = driver.watchdog();

    try std.testing.expectError(error.InvalidTimeout, watchdog.start(.{ .timeout_ns = 0 }));
    try std.testing.expectError(error.InvalidTimeout, watchdog.start(.{ .timeout_ns = 1_000_000_000_000 }));
    try std.testing.expect(regs.lastWrite(0x18) == null);
}

test "sf32lb52 TRNG: checked 256-bit batches implement Entropy" {
    var regs = fake.Sparse{};
    var driver = sf32lb52_trng.bind(regs.mmio(), 8);
    const entropy = driver.entropy();

    // Internal seed and random output are valid. Each output register is read
    // little-endian, including a partial final word.
    regs.setRead(0x04, (1 << 1) | (1 << 3));
    regs.setRead(0x30, 0x44332211);
    regs.setRead(0x34, 0x88776655);
    var bytes: [6]u8 = undefined;
    try entropy.fill(&bytes);
    try std.testing.expectEqualSlices(u8, &.{ 0x11, 0x22, 0x33, 0x44, 0x55, 0x66 }, &bytes);
    try std.testing.expectEqual(@as(u64, 1 << 1), regs.lastWrite(0x00).?);
}

test "sf32lb52 TRNG: initialization, timeout, and lockup are surfaced" {
    var regs = fake.Sparse{};
    var driver = sf32lb52_trng.bind(regs.mmio(), 2);
    var byte: [1]u8 = undefined;

    try std.testing.expectError(error.Timeout, driver.entropy().fill(&byte));
    try std.testing.expectEqual(@as(u64, 1), regs.lastWrite(0x08).?);
    try std.testing.expectEqual(@as(u64, 1), regs.lastWrite(0x00).?);

    regs.setRead(0x0c, 1 << 2);
    try std.testing.expectError(error.HealthTestFailed, driver.entropy().fill(&byte));
}

test "sf32lb52 USART: 1 Mbaud 8N1 and exact byte-stream I/O" {
    var regs = fake.Sparse{};
    var usart = try sf32lb52_usart.bind(regs.mmio(), .{
        .clock_hz = 48_000_000,
        .baud = 1_000_000,
    });

    try std.testing.expectEqual(@as(u64, 48), regs.lastWrite(0x0c).?);
    try std.testing.expectEqual(@as(u64, (2 << 27) | 0x0d), regs.lastWrite(0x00).?);
    try std.testing.expectEqual(@as(u64, 0), regs.lastWrite(0x04).?);
    try std.testing.expectEqual(@as(u64, 0), regs.lastWrite(0x08).?);
    try std.testing.expectEqual(@as(u64, 0x25f), regs.lastWrite(0x20).?);

    regs.setRead(0x1c, 0x80 | 0x20 | 0x0a); // TXE, RXNE, framing + overrun.
    regs.setRead(0x24, 0x41);
    usart.serial().write("A\n");
    try std.testing.expectEqual(@as(u64, '\n'), regs.lastWrite(0x28).?);

    var bytes: [2]u8 = undefined;
    try std.testing.expectEqual(@as(usize, 2), usart.serial().read(&bytes));
    try std.testing.expectEqualSlices(u8, "AA", &bytes);
    const errors = usart.takeRxErrors();
    try std.testing.expect(errors.framing and errors.overrun);
    try std.testing.expect(!errors.parity and !errors.noise);
    try std.testing.expectEqual(@as(u64, 0x0a), regs.lastWrite(0x20).?);
}

test "sf32lb52 USART: frame, flow control, interrupts, and DMA" {
    var regs = fake.Sparse{};
    var usart = try sf32lb52_usart.bind(regs.mmio(), .{
        .clock_hz = 48_000_000,
        .baud = 6_000_000,
        .data_bits = .seven,
        .parity = .odd,
        .stop_bits = .two,
        .flow_control = .rts_cts,
    });

    // Seven data bits plus parity selects an eight-bit packet (M=2).
    try std.testing.expectEqual(@as(u64, (2 << 27) | (1 << 14) | (1 << 10) | (1 << 9) | 0x0d), regs.lastWrite(0x00).?);
    try std.testing.expectEqual(@as(u64, 2 << 12), regs.lastWrite(0x04).?);
    try std.testing.expectEqual(@as(u64, (1 << 8) | (1 << 9)), regs.lastWrite(0x08).?);
    try std.testing.expectEqual(@as(u64, 16), regs.lastWrite(0x0c).?);

    usart.enableRxInterrupt(true);
    usart.enableTxEmptyInterrupt(true);
    try std.testing.expect(regs.lastWrite(0x00).? & 0xa0 == 0xa0);
    usart.configureDma(true, true);
    try std.testing.expectEqual(@as(u64, 0x3c0), regs.lastWrite(0x08).?);
}

test "sf32lb52 hpsys cfg: UART and I2C signal routing" {
    var regs = fake.Sparse{};
    const cfg = sf32lb52_hpsys_cfg.bind(regs.mmio());

    // TX/RX routing must not silently connect omitted RTS/CTS signals to PA0.
    regs.setRead(0x58, 7 << 16 | 8 << 24);
    cfg.routeUart(.uart1, .{ .tx = 19, .rx = 18 });
    try std.testing.expectEqual(@as(u64, 19 | (18 << 8) | (7 << 16) | (8 << 24)), regs.lastWrite(0x58).?);

    cfg.routeUart(.uart3, .{ .tx = 42, .rx = 41, .rts = 40, .cts = 39 });
    const uart3: u32 = 42 | (41 << 8) | (40 << 16) | (39 << 24);
    try std.testing.expectEqual(@as(u64, uart3), regs.lastWrite(0x60).?);

    cfg.routeI2c(.i2c1, .{ .scl = 10, .sda = 11 });
    try std.testing.expectEqual(@as(u64, 10 | (11 << 8)), regs.lastWrite(0x48).?);
    cfg.routeI2c(.i2c4, .{ .scl = 43, .sda = 44 });
    try std.testing.expectEqual(@as(u64, 43 | (44 << 8)), regs.lastWrite(0x54).?);
}

test "sf32lb52 gpio: atomic GPIO operations span both register banks" {
    var regs = fake.Sparse{};
    var dev = sf32lb52_gpio.bind(regs.mmio());
    const gpio = dev.gpio();

    gpio.direction(3, .output);
    try std.testing.expectEqual(@as(u64, 1 << 3), regs.lastWrite(0x14).?);
    gpio.set(3, true);
    try std.testing.expectEqual(@as(u64, 1 << 3), regs.lastWrite(0x08).?);
    gpio.set(3, false);
    try std.testing.expectEqual(@as(u64, 1 << 3), regs.lastWrite(0x0c).?);

    // PA37 is bank 1, bit 5. This is the Pebble Time 2 up button.
    gpio.direction(37, .input);
    try std.testing.expectEqual(@as(u64, 1 << 5), regs.lastWrite(0x98).?);
    regs.setRead(0x80, 1 << 5);
    try std.testing.expect(gpio.get(37));
}

test "sf32lb52 gpio: both-edge interrupt uses set/clear aliases and W1C" {
    var regs = fake.Sparse{};
    const dev = sf32lb52_gpio.bind(regs.mmio());

    dev.configureInterrupt(34, .both_edges); // bank 1, bit 2
    try std.testing.expectEqual(@as(u64, 1 << 2), regs.lastWrite(0xac).?); // edge
    try std.testing.expectEqual(@as(u64, 1 << 2), regs.lastWrite(0xb8).?); // rising
    try std.testing.expectEqual(@as(u64, 1 << 2), regs.lastWrite(0xc4).?); // falling
    try std.testing.expectEqual(@as(u64, 1 << 2), regs.lastWrite(0xa0).?); // enabled

    regs.setRead(0xcc, 1 << 2);
    try std.testing.expect(dev.interruptPending(34));
    dev.clearInterrupt(34);
    try std.testing.expectEqual(@as(u64, 1 << 2), regs.lastWrite(0xcc).?);
    dev.disableInterrupt(34);
    try std.testing.expectEqual(@as(u64, 1 << 2), regs.lastWrite(0xa4).?);
}

test "sf32lb52 pinmux: pad function and electrical controls" {
    var regs = fake.Sparse{};
    const mux = sf32lb52_pinmux.bind(regs.mmio());

    mux.configure(37, .{
        .function = .gpio,
        .pull = .up,
        .input_enable = true,
        .schmitt = true,
        .slow_slew = true,
        .drive_strength = 2,
    });
    const expected: u32 = (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7) |
        (1 << 8) | (2 << 9);
    try std.testing.expectEqual(@as(u64, expected), regs.lastWrite(0x34 + 37 * 4).?);

    mux.configure(18, .{ .function = .uart_i2c, .pull = .none });
    try std.testing.expectEqual(@as(u64, 4 | (1 << 6)), regs.lastWrite(0x34 + 18 * 4).?);
}

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
