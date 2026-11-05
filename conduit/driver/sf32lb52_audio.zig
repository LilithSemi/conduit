//! SF32LB52 speaker playback through AUDCODEC and optional AUDPRC mixing.
//!
//! Board-level power-amplifier GPIO control remains with the board firmware.

const Mmio = @import("../mmio.zig");
const Dmac = @import("sf32lb52_dmac.zig");
const pac = @import("sf32lb52_regs");
const codec_regs = pac.audcodec;
const prc_regs = pac.audprc;

pub const Delay = struct {
    ctx: ?*anyopaque,
    wait_fn: *const fn (ctx: ?*anyopaque, microseconds: u32) void,

    pub fn wait(self: Delay, microseconds: u32) void {
        self.wait_fn(self.ctx, microseconds);
    }

    pub fn from(impl: anytype) Delay {
        const P = @TypeOf(impl);
        const Adapter = struct {
            fn wait(ctx: ?*anyopaque, microseconds: u32) void {
                const self: P = @ptrCast(@alignCast(ctx));
                self.waitMicroseconds(microseconds);
            }
        };
        return .{ .ctx = impl, .wait_fn = Adapter.wait };
    }
};

pub const SampleRate = enum(u32) {
    hz8000 = 8_000,
    hz12000 = 12_000,
    hz16000 = 16_000,
    hz24000 = 24_000,
    hz32000 = 32_000,
    hz48000 = 48_000,
};

pub const Source = enum(u2) { audprc = 0, memory = 1 };
pub const Error = error{ InvalidVolume, InvalidCount, Timeout, Overflow, Underrun };

const DacClock = struct { osr: u4 };
fn dacClock(rate: SampleRate) DacClock {
    return .{ .osr = switch (rate) {
        .hz48000 => 0,
        .hz32000 => 1,
        .hz24000 => 5,
        .hz16000 => 4,
        .hz12000 => 7,
        .hz8000 => 8,
    } };
}

pub const CodecOptions = struct {
    sample_rate: SampleRate = .hz16000,
    source: Source = .memory,
    volume_db: i8 = -18,
    poll_limit: u32 = 100_000,
};

pub const Audcodec = struct {
    mmio: Mmio,
    delay: Delay,
    source: Source,
    poll_limit: u32,
    powered: bool = false,

    /// Bring up the shared analog references and DAC1 in the order required
    /// by the SF32LB52 codec. The supported 8--48 kHz family runs from HXT48,
    /// so no PLL frequency calibration is needed.
    pub fn powerOn(self: *Audcodec) void {
        if (self.powered) return;
        self.mmio.write(u32, codec_regs.bg_cfg0.offset, codec_regs.bg_cfg0.en_mask |
            (@as(u32, 0xc) << codec_regs.bg_cfg0.vref_sel_shift) |
            codec_regs.bg_cfg0.en_rcflt_mask |
            (@as(u32, 4) << codec_regs.bg_cfg0.mic_vref_sel_shift) |
            codec_regs.bg_cfg0.en_amp_mask);
        self.mmio.write(u32, codec_regs.bg_cfg1.offset, 0);
        self.mmio.write(u32, codec_regs.bg_cfg2.offset, 0);
        self.delay.wait(100);

        var pll0 = self.mmio.read(u32, codec_regs.pll_cfg0.offset);
        pll0 &= ~codec_regs.pll_cfg0.icp_sel_mask;
        pll0 |= codec_regs.pll_cfg0.en_iary_mask | codec_regs.pll_cfg0.en_vco_mask |
            codec_regs.pll_cfg0.en_ana_mask | (@as(u32, 8) << codec_regs.pll_cfg0.icp_sel_shift);
        self.mmio.write(u32, codec_regs.pll_cfg0.offset, pll0);
        self.mmio.write(u32, codec_regs.pll_cfg2.offset, self.mmio.read(u32, codec_regs.pll_cfg2.offset) | codec_regs.pll_cfg2.en_dig_mask);
        self.mmio.write(u32, codec_regs.pll_cfg3.offset, self.mmio.read(u32, codec_regs.pll_cfg3.offset) | codec_regs.pll_cfg3.en_sdm_mask);
        self.mmio.write(u32, codec_regs.pll_cfg4.offset, self.mmio.read(u32, codec_regs.pll_cfg4.offset) | codec_regs.pll_cfg4.en_clk_dig_mask);
        self.mmio.write(u32, codec_regs.pll_cfg1.offset, (@as(u32, 3) << codec_regs.pll_cfg1.r3_sel_shift) |
            (@as(u32, 1) << codec_regs.pll_cfg1.rz_sel_shift) |
            (@as(u32, 3) << codec_regs.pll_cfg1.c2_sel_shift) |
            (@as(u32, 6) << codec_regs.pll_cfg1.cz_sel_shift));
        self.delay.wait(50);

        self.mmio.write(u32, codec_regs.refgen_cfg.offset, (self.mmio.read(u32, codec_regs.refgen_cfg.offset) &
            ~(codec_regs.refgen_cfg.en_chop_mask | codec_regs.refgen_cfg.lv_mode_mask)) |
            codec_regs.refgen_cfg.en_mask);
        self.mmio.write(u32, codec_regs.pll_cfg5.offset, self.mmio.read(u32, codec_regs.pll_cfg5.offset) |
            codec_regs.pll_cfg5.en_clk_chop_bg_mask | codec_regs.pll_cfg5.en_clk_chop_refgen_mask);
        self.delay.wait(2_000);

        var pll4 = self.mmio.read(u32, codec_regs.pll_cfg4.offset);
        pll4 &= ~(codec_regs.pll_cfg4.sel_clk_dac_mask | codec_regs.pll_cfg4.sel_clk_dac_source_mask);
        pll4 |= codec_regs.pll_cfg4.en_clk_chop_dac_mask | codec_regs.pll_cfg4.en_clk_dac_mask | codec_regs.pll_cfg4.en_clk_dig_mask;
        self.mmio.write(u32, codec_regs.pll_cfg4.offset, pll4);
        self.mmio.write(u32, codec_regs.pll_cfg2.offset, self.mmio.read(u32, codec_regs.pll_cfg2.offset) & ~codec_regs.pll_cfg2.rstb_mask);
        self.delay.wait(100);
        self.mmio.write(u32, codec_regs.pll_cfg2.offset, self.mmio.read(u32, codec_regs.pll_cfg2.offset) | codec_regs.pll_cfg2.rstb_mask);

        var dac1 = self.mmio.read(u32, codec_regs.dac1_cfg.offset);
        dac1 &= ~(codec_regs.dac1_cfg.lp_mode_mask | codec_regs.dac1_cfg.en_os_dac_mask);
        dac1 |= codec_regs.dac1_cfg.en_vcm_mask;
        self.mmio.write(u32, codec_regs.dac1_cfg.offset, dac1);
        self.delay.wait(5);
        dac1 |= codec_regs.dac1_cfg.en_amp_mask;
        self.mmio.write(u32, codec_regs.dac1_cfg.offset, dac1);
        self.delay.wait(1);
        dac1 |= codec_regs.dac1_cfg.en_os_dac_mask;
        self.mmio.write(u32, codec_regs.dac1_cfg.offset, dac1);
        self.delay.wait(10);
        dac1 |= codec_regs.dac1_cfg.en_dac_mask;
        self.mmio.write(u32, codec_regs.dac1_cfg.offset, dac1);
        self.delay.wait(10);
        self.mmio.write(u32, codec_regs.dac1_cfg.offset, dac1 & ~codec_regs.dac1_cfg.sr_mask);
        self.powered = true;
    }

    pub fn powerOff(self: *Audcodec) void {
        if (!self.powered) return;
        self.mute(true);
        var dac1 = self.mmio.read(u32, codec_regs.dac1_cfg.offset) | codec_regs.dac1_cfg.sr_mask;
        self.mmio.write(u32, codec_regs.dac1_cfg.offset, dac1);
        self.delay.wait(10);
        dac1 &= ~codec_regs.dac1_cfg.en_dac_mask;
        self.mmio.write(u32, codec_regs.dac1_cfg.offset, dac1);
        self.delay.wait(10);
        dac1 &= ~codec_regs.dac1_cfg.en_vcm_mask;
        self.mmio.write(u32, codec_regs.dac1_cfg.offset, dac1);
        self.delay.wait(10);
        dac1 &= ~(codec_regs.dac1_cfg.en_amp_mask | codec_regs.dac1_cfg.en_os_dac_mask);
        self.mmio.write(u32, codec_regs.dac1_cfg.offset, dac1);
        self.mmio.write(u32, codec_regs.cfg.offset, self.mmio.read(u32, codec_regs.cfg.offset) & ~codec_regs.cfg.dac_enable_mask);
        self.mmio.write(u32, codec_regs.pll_cfg0.offset, self.mmio.read(u32, codec_regs.pll_cfg0.offset) &
            ~(codec_regs.pll_cfg0.en_iary_mask | codec_regs.pll_cfg0.en_vco_mask | codec_regs.pll_cfg0.en_ana_mask));
        self.mmio.write(u32, codec_regs.pll_cfg2.offset, self.mmio.read(u32, codec_regs.pll_cfg2.offset) & ~codec_regs.pll_cfg2.en_dig_mask);
        self.mmio.write(u32, codec_regs.pll_cfg3.offset, self.mmio.read(u32, codec_regs.pll_cfg3.offset) & ~codec_regs.pll_cfg3.en_sdm_mask);
        self.mmio.write(u32, codec_regs.pll_cfg4.offset, self.mmio.read(u32, codec_regs.pll_cfg4.offset) & ~codec_regs.pll_cfg4.en_clk_dig_mask);
        self.mmio.write(u32, codec_regs.refgen_cfg.offset, self.mmio.read(u32, codec_regs.refgen_cfg.offset) & ~codec_regs.refgen_cfg.en_mask);
        self.mmio.write(u32, codec_regs.bg_cfg0.offset, self.mmio.read(u32, codec_regs.bg_cfg0.offset) &
            ~(codec_regs.bg_cfg0.en_mask | codec_regs.bg_cfg0.en_smpl_mask));
        self.powered = false;
    }

    pub fn start(self: *Audcodec) void {
        self.powerOn();
        self.mmio.write(u32, codec_regs.cfg.offset, self.mmio.read(u32, codec_regs.cfg.offset) | codec_regs.cfg.dac_enable_mask);
        self.mute(false);
    }

    pub fn stop(self: *Audcodec) void {
        self.mute(true);
        self.mmio.write(u32, codec_regs.dac_ch0_cfg.offset, self.mmio.read(u32, codec_regs.dac_ch0_cfg.offset) &
            ~(codec_regs.dac_ch0_cfg.dma_en_mask | codec_regs.dac_ch0_cfg.enable_mask));
    }

    pub fn mute(self: *Audcodec, enabled: bool) void {
        var value = self.mmio.read(u32, codec_regs.dac_ch0_cfg.offset);
        if (enabled) value |= codec_regs.dac_ch0_cfg.dout_mute_mask else value &= ~codec_regs.dac_ch0_cfg.dout_mute_mask;
        self.mmio.write(u32, codec_regs.dac_ch0_cfg.offset, value);
        self.mmio.write(u32, codec_regs.dac_ch0_debug.offset, (if (enabled) codec_regs.dac_ch0_debug.bypass_mask else 0) | 0xff);
    }

    pub fn setVolume(self: *Audcodec, db: i8) Error!void {
        if (db < -36 or db > 54) return error.InvalidVolume;
        const gain: u8 = @intCast(db + 36);
        const rough: u32 = gain / 6;
        const fine: u32 = @as(u32, gain % 6) * 2;
        var value = self.mmio.read(u32, codec_regs.dac_ch0_cfg.offset);
        value &= ~(codec_regs.dac_ch0_cfg.rough_vol_mask | codec_regs.dac_ch0_cfg.fine_vol_mask);
        value |= rough << codec_regs.dac_ch0_cfg.rough_vol_shift | fine << codec_regs.dac_ch0_cfg.fine_vol_shift;
        self.mmio.write(u32, codec_regs.dac_ch0_cfg.offset, value);
    }

    pub fn writeWord(self: *Audcodec, samples: u32) Error!void {
        var remaining = self.poll_limit;
        while (remaining != 0) : (remaining -= 1) {
            try self.checkErrors();
            const count = (self.mmio.read(u32, codec_regs.apb_stat.offset) & codec_regs.apb_stat.dac_ch0_fifo_cnt_mask) >> codec_regs.apb_stat.dac_ch0_fifo_cnt_shift;
            if (count < 8) {
                self.mmio.write(u32, codec_regs.dac_ch0_entry.offset, samples);
                return;
            }
        }
        return error.Timeout;
    }

    pub fn configureDma(self: *Audcodec, dma: *Dmac.Dmac1, channel: Dmac.Channel, circular: bool) Dmac.Error!void {
        _ = self;
        try dma.configure(channel, .{
            .request = .audcodec_dac0,
            .direction = .memory_to_peripheral,
            .memory_width = .word,
            .peripheral_width = .word,
            .circular = circular,
            .priority = .high,
            .half_interrupt = circular,
            .transfer_interrupt = true,
        });
    }

    pub fn startDma(self: *Audcodec, dma: *Dmac.Dmac1, channel: Dmac.Channel, address: u32, byte_count: u32) (Dmac.Error || Error)!void {
        if (byte_count == 0 or byte_count & 3 != 0 or byte_count / 4 > 0xffff) return error.InvalidCount;
        try dma.start(channel, @intCast(codec_regs.base_address + codec_regs.dac_ch0_entry.offset), address, @intCast(byte_count / 4));
        self.mmio.write(u32, codec_regs.dac_ch0_cfg.offset, self.mmio.read(u32, codec_regs.dac_ch0_cfg.offset) | codec_regs.dac_ch0_cfg.dma_en_mask);
        self.start();
    }

    fn checkErrors(self: *Audcodec) Error!void {
        const irq = self.mmio.read(u32, codec_regs.irq.offset);
        if (irq & codec_regs.irq.dac_ch0_apb_of_mask != 0) return error.Overflow;
        if (irq & codec_regs.irq.dac_ch0_out_uf_mask != 0) return error.Underrun;
    }
};

pub fn bindCodec(mmio: Mmio, delay: Delay, options: CodecOptions) Error!Audcodec {
    const clock = dacClock(options.sample_rate);
    var codec = Audcodec{ .mmio = mmio, .delay = delay, .source = options.source, .poll_limit = options.poll_limit };
    mmio.write(u32, codec_regs.dac_cfg.offset, @as(u32, clock.osr) << codec_regs.dac_cfg.osr_sel_shift |
        @as(u32, @backingInt(options.source)) << codec_regs.dac_cfg.op_mode_shift |
        @as(u32, 1) << codec_regs.dac_cfg.clk_div_shift);
    mmio.write(u32, codec_regs.dac_ch0_cfg.offset, codec_regs.dac_ch0_cfg.enable_mask |
        (@as(u32, 2) << codec_regs.dac_ch0_cfg.dem_mode_shift) |
        (@as(u32, 6) << codec_regs.dac_ch0_cfg.rough_vol_shift) |
        codec_regs.dac_ch0_cfg.data_format_mask |
        (@as(u32, 0x14d) << codec_regs.dac_ch0_cfg.sinc_gain_shift));
    mmio.write(u32, codec_regs.dac_ch0_cfg_ext.offset, codec_regs.dac_ch0_cfg_ext.ramp_en_mask | codec_regs.dac_ch0_cfg_ext.ramp_mode_mask |
        codec_regs.dac_ch0_cfg_ext.zero_adjust_en_mask | (@as(u32, 2) << codec_regs.dac_ch0_cfg_ext.ramp_interval_shift));
    mmio.write(u32, codec_regs.dac_ch0_debug.offset, 0xff);
    try codec.setVolume(options.volume_db);
    return codec;
}

pub const AudprcOptions = struct {
    audclk_div: u4 = 0,
    dac_strobe_div: u16,
    stereo: bool = true,
};

pub const Audprc = struct {
    mmio: Mmio,

    pub fn start(self: *Audprc) void {
        var channel = self.mmio.read(u32, prc_regs.tx_ch0_cfg.offset);
        channel &= ~prc_regs.tx_ch0_cfg.dma_msk_mask;
        channel |= prc_regs.tx_ch0_cfg.enable_mask;
        self.mmio.write(u32, prc_regs.tx_ch0_cfg.offset, channel);
        self.mmio.write(u32, prc_regs.cfg.offset, self.mmio.read(u32, prc_regs.cfg.offset) |
            prc_regs.cfg.enable_mask | prc_regs.cfg.dac_path_en_mask);
    }

    pub fn stop(self: *Audprc) void {
        var channel = self.mmio.read(u32, prc_regs.tx_ch0_cfg.offset);
        channel = (channel | prc_regs.tx_ch0_cfg.dma_msk_mask) & ~prc_regs.tx_ch0_cfg.enable_mask;
        self.mmio.write(u32, prc_regs.tx_ch0_cfg.offset, channel);
        self.mmio.write(u32, prc_regs.cfg.offset, self.mmio.read(u32, prc_regs.cfg.offset) & ~prc_regs.cfg.dac_path_en_mask);
    }

    pub fn configureDma(self: *Audprc, dma: *Dmac.Dmac1, channel: Dmac.Channel, circular: bool) Dmac.Error!void {
        _ = self;
        try dma.configure(channel, .{
            .request = .audprc_tx0,
            .direction = .memory_to_peripheral,
            .memory_width = .word,
            .peripheral_width = .word,
            .circular = circular,
            .priority = .high,
            .half_interrupt = circular,
            .transfer_interrupt = true,
        });
    }

    pub fn startDma(self: *Audprc, dma: *Dmac.Dmac1, channel: Dmac.Channel, address: u32, byte_count: u32) (Dmac.Error || Error)!void {
        if (byte_count == 0 or byte_count & 3 != 0 or byte_count / 4 > 0xffff) return error.InvalidCount;
        try dma.start(channel, @intCast(prc_regs.base_address + prc_regs.tx_ch0_entry.offset), address, @intCast(byte_count / 4));
        self.start();
    }
};

pub fn bindAudprc(mmio: Mmio, options: AudprcOptions) Audprc {
    mmio.write(u32, prc_regs.cfg.offset, @as(u32, options.audclk_div) << prc_regs.cfg.audclk_div_shift |
        prc_regs.cfg.audclk_div_update_mask | prc_regs.cfg.auto_gate_en_mask);
    mmio.write(u32, prc_regs.stb.offset, options.dac_strobe_div);
    mmio.write(u32, prc_regs.tx_ch0_cfg.offset, prc_regs.tx_ch0_cfg.dma_msk_mask | if (options.stereo) prc_regs.tx_ch0_cfg.mode_mask else 0);
    // Destination 0 is AUDCODEC. Source/mixer zero selects TX channel 0, so
    // reset values provide the straight-through path without resampling.
    mmio.write(u32, prc_regs.dac_path_cfg0.offset, (@as(u32, 6) << prc_regs.dac_path_cfg0.rough_vol_l_shift) |
        (@as(u32, 6) << prc_regs.dac_path_cfg0.rough_vol_r_shift));
    mmio.write(u32, prc_regs.dac_path_cfg1.offset, @as(u32, 3) << prc_regs.dac_path_cfg1.src_ch_en_shift);
    mmio.write(u32, prc_regs.dac_path_cfg2.offset, 0);
    return .{ .mmio = mmio };
}

pub fn codecFromBase(delay: Delay, options: CodecOptions) Error!Audcodec {
    return bindCodec(Mmio.direct(codec_regs.base_address), delay, options);
}

pub fn audprcFromBase(options: AudprcOptions) Audprc {
    return bindAudprc(Mmio.direct(prc_regs.base_address), options);
}
