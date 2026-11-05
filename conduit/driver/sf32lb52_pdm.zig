//! SF32LB52 PDM1 microphone interface.
const Mmio = @import("../mmio.zig");
const Dmac = @import("sf32lb52_dmac.zig");
const pac = @import("sf32lb52_regs");
const regs = pac.pdm1;

pub const Channels = enum { left, right, stereo, stereo_swapped };
pub const SampleRate = enum(u32) { hz8000 = 8000, hz16000 = 16000, hz32000 = 32000, hz48000 = 48000, hz96000 = 96000 };
pub const Depth = enum(u2) { bits24 = 0, bits16 = 1, bits8 = 2, bits32 = 3 };
pub const ClockSource = enum { xtal_9m6, audio_pll_3m072 };
pub const Options = struct {
    channels: Channels = .stereo,
    sample_rate: SampleRate = .hz16000,
    depth: Depth = .bits16,
    clock_source: ClockSource = .audio_pll_3m072,
    gain_left: u7 = 0x26,
    gain_right: u7 = 0x26,
    sample_delay_left: u3 = 0,
    sample_delay_right: u3 = 0,
    poll_limit: u32 = 100_000,
};
pub const Error = error{ Timeout, OverflowLeft, OverflowRight, InvalidCount };
const data_left_offset = 0x3c;
const data_right_offset = 0x40;

pub const Pdm1 = struct {
    mmio: Mmio,
    channels: Channels,
    poll_limit: u32,
    pub fn start(self: *Pdm1) void {
        self.mmio.write(u32, regs.int_clr.offset, 3);
        self.mmio.write(u32, regs.int_msk.offset, 0);
        self.mmio.write(u32, regs.cfg0.offset, self.mmio.read(u32, regs.cfg0.offset) | regs.cfg0.pdmcoreen_mask);
    }
    pub fn stop(self: *Pdm1) void {
        self.mmio.write(u32, regs.cfg0.offset, self.mmio.read(u32, regs.cfg0.offset) & ~regs.cfg0.pdmcoreen_mask);
        self.mmio.write(u32, regs.int_msk.offset, 3);
    }
    pub fn readWord(self: *Pdm1, channel: enum { left, right }) Error!u32 {
        var remaining = self.poll_limit;
        const empty = if (channel == .left) regs.fifo_st.empty_l_mask else regs.fifo_st.empty_r_mask;
        while (remaining != 0) : (remaining -= 1) {
            try self.checkOverflow();
            if (self.mmio.read(u32, regs.fifo_st.offset) & empty == 0)
                return self.mmio.read(u32, if (channel == .left) data_left_offset else data_right_offset);
        }
        return error.Timeout;
    }
    pub fn dmaPeripheralAddress(self: *const Pdm1) u32 {
        const offset: u32 = if (self.channels == .right) data_right_offset else data_left_offset;
        return @intCast(pac.pdm1.base_address + offset);
    }
    pub fn configureDma(self: *Pdm1, dma: *Dmac.Dmac1, channel: Dmac.Channel, request: Dmac.Request, circular: bool) Dmac.Error!void {
        _ = self;
        try dma.configure(channel, .{ .request = request, .direction = .peripheral_to_memory, .memory_width = .word, .peripheral_width = .word, .circular = circular, .half_interrupt = true, .transfer_interrupt = true });
    }
    pub fn startDma(self: *Pdm1, dma: *Dmac.Dmac1, channel: Dmac.Channel, buffer_address: u32, byte_count: u32) (Dmac.Error || Error)!void {
        if (byte_count == 0 or byte_count & 3 != 0 or byte_count / 4 > 0xffff) return error.InvalidCount;
        try dma.start(channel, self.dmaPeripheralAddress(), buffer_address, @intCast(byte_count / 4));
        self.start();
    }
    pub fn overflow(self: *const Pdm1) struct { left: bool, right: bool } {
        const value = self.mmio.read(u32, regs.int_st.offset);
        return .{ .left = value & regs.int_st.overflow_l_mask != 0, .right = value & regs.int_st.overflow_r_mask != 0 };
    }
    pub fn clearOverflow(self: *Pdm1) void {
        self.mmio.write(u32, regs.int_clr.offset, 3);
    }
    fn checkOverflow(self: *Pdm1) Error!void {
        const value = self.mmio.read(u32, regs.int_st.offset);
        if (value & regs.int_st.overflow_l_mask != 0) return error.OverflowLeft;
        if (value & regs.int_st.overflow_r_mask != 0) return error.OverflowRight;
    }
};

pub fn bind(mmio: Mmio, options: Options) Pdm1 {
    var cfg0: u32 = 0;
    var fifo: u32 = @as(u32, @backingInt(options.depth)) << regs.fifo_cfg.byte_trunc_shift;
    switch (options.channels) {
        .left => {
            cfg0 |= regs.cfg0.left_en_mask;
            fifo |= regs.fifo_cfg.rx_dma_msk_r_mask;
        },
        .right => {
            cfg0 |= regs.cfg0.right_en_mask;
            fifo |= regs.fifo_cfg.rx_dma_msk_l_mask;
        },
        .stereo, .stereo_swapped => {
            cfg0 |= regs.cfg0.left_en_mask | regs.cfg0.right_en_mask | regs.cfg0.stereo_en_mask;
            fifo |= regs.fifo_cfg.byte_con_mask;
        },
    }
    if (options.channels == .stereo_swapped) cfg0 |= regs.cfg0.swap_en_mask;
    const timing = sampleTiming(options.sample_rate, options.clock_source);
    cfg0 |= @as(u32, timing.divider) << regs.cfg0.clk_div_shift;
    if (options.clock_source == .audio_pll_3m072) cfg0 |= regs.cfg0.clk_sel_mask;
    mmio.write(u32, regs.cfg0.offset, cfg0);
    mmio.write(u32, regs.cfg1.offset, @as(u32, options.sample_delay_left) << 5 | @as(u32, options.sample_delay_right) << 8);
    mmio.write(u32, regs.sinc_cfg.offset, timing.sinc_rate | if (timing.order4) regs.sinc_cfg.sinc_order_sel_mask else 0);
    mmio.write(u32, regs.lpf_cfg6.offset, if (timing.downsample4) regs.lpf_cfg6.lpf_ds_mask else 0);
    mmio.write(u32, regs.hpf_cfg.offset, 7);
    mmio.write(u32, regs.pga_cfg.offset, @as(u32, options.gain_left) | @as(u32, options.gain_right) << 7);
    mmio.write(u32, regs.fifo_cfg.offset, fifo);
    mmio.write(u32, regs.int_msk.offset, 3);
    return .{ .mmio = mmio, .channels = options.channels, .poll_limit = options.poll_limit };
}
const Timing = struct { divider: u4, sinc_rate: u8, order4: bool = false, downsample4: bool = false };
fn sampleTiming(rate: SampleRate, source: ClockSource) Timing {
    return switch (source) {
        .xtal_9m6 => switch (rate) {
            .hz8000 => .{ .divider = 4, .sinc_rate = 75, .downsample4 = true },
            .hz16000 => .{ .divider = 6, .sinc_rate = 50 },
            .hz32000 => .{ .divider = 6, .sinc_rate = 25 },
            .hz48000 => .{ .divider = 4, .sinc_rate = 25, .order4 = true },
            .hz96000 => .{ .divider = 1, .sinc_rate = 8, .order4 = true, .downsample4 = true },
        },
        .audio_pll_3m072 => switch (rate) {
            .hz8000 => .{ .divider = 3, .sinc_rate = 64 },
            .hz16000 => .{ .divider = 1, .sinc_rate = 96 },
            .hz32000 => .{ .divider = 1, .sinc_rate = 48 },
            .hz48000 => .{ .divider = 1, .sinc_rate = 32 },
            .hz96000 => .{ .divider = 1, .sinc_rate = 8, .order4 = true, .downsample4 = true },
        },
    };
}
pub fn fromBase(options: Options) Pdm1 {
    return bind(Mmio.direct(pac.pdm1.base_address), options);
}
