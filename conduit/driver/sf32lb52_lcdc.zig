//! SF32LB52 LCDC1 scanout support for the Pebble Time 2 JDI panel.
//!
//! This is deliberately a physical-framebuffer interface. A Lattice backend
//! can expose a Prism resource as a render target, then submit its system-bus
//! address here without making Conduit depend on either graphics layer.

const Mmio = @import("../mmio.zig");
const pac = @import("sf32lb52_regs");
const regs = pac.lcdc1;

pub const Error = error{
    InvalidClock,
    InvalidFrame,
    InvalidRegion,
    Busy,
};

pub const PixelFormat = enum(u3) {
    rgb565 = 0,
    rgb888 = 1,
    /// Byte layout expected by LCDC's ARGB8888 fetcher. The alpha byte is
    /// ignored because scanout uses a constant, opaque layer alpha.
    xrgb8888 = 2,
    argb8565 = 3,
    rgb332 = 4,

    pub fn bytesPerPixel(self: PixelFormat) u3 {
        return switch (self) {
            .rgb332 => 1,
            .rgb565 => 2,
            .rgb888, .argb8565 => 3,
            .xrgb8888 => 4,
        };
    }
};

/// A framebuffer address visible to LCDC on the system bus. `stride_bytes`
/// describes the complete source image, even for a partial-row update.
pub const Frame = struct {
    address: u32,
    width: u16 = 240,
    height: u16 = 240,
    stride_bytes: u16,
    format: PixelFormat = .xrgb8888,
};

pub const Region = struct {
    first_row: u16 = 0,
    last_row: u16 = 239,
};

/// Timing values used by SiFli's JDI387A board support for the PT2 panel.
pub const JdiConfig = struct {
    lcdc_clock_hz: u32,
    hck_hz: u32 = 746_268,
    valid_columns: u16 = 240,
    valid_rows: u16 = 240,
    bank_column_head: u16 = 0,
    bank_column_tail: u16 = 4,
    bank_row_head: u16 = 0,
    bank_row_tail: u16 = 4,
    enable_start_column: u16 = 32,
    enable_end_column: u16 = 95,
};

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

pub const Event = enum {
    none,
    /// LCDC has consumed the framebuffer; the memory may be rendered into
    /// again, but panel timing is still completing.
    frame_consumed,
    complete,
    underrun,
};

const Phase = enum { idle, sending, tail };

pub const Lcdc1 = struct {
    mmio: Mmio,
    delay: Delay,
    config: JdiConfig,
    max_column: u16,
    max_line: u16,
    phase: Phase = .idle,

    pub fn available(self: *const Lcdc1) bool {
        return self.phase == .idle and self.mmio.read(u32, regs.status.offset) & regs.status.jdi_par_run_mask == 0;
    }

    /// Begin an asynchronous scanout. JDI parallel mode can update a range of
    /// rows, but the PT2 panel requires every column in those rows.
    pub fn present(self: *Lcdc1, frame: Frame, region: Region) Error!void {
        if (!self.available()) return error.Busy;
        if (frame.width != self.config.valid_columns or frame.height != self.config.valid_rows)
            return error.InvalidFrame;
        const bpp: u32 = frame.format.bytesPerPixel();
        const minimum_stride = @as(u32, frame.width) * bpp;
        if (frame.address & 3 != 0 or frame.stride_bytes < minimum_stride or frame.stride_bytes > 0x1fff)
            return error.InvalidFrame;
        if (region.first_row > region.last_row or region.last_row >= self.config.valid_rows)
            return error.InvalidRegion;

        const rows: u32 = @as(u32, region.last_row) - region.first_row + 1;
        const source_result = @addWithOverflow(frame.address, @as(u32, region.first_row) * frame.stride_bytes);
        if (source_result[1] != 0) return error.InvalidFrame;
        const source = source_result[0];
        const top = @as(u32, region.first_row) << 16;
        const bottom = (@as(u32, region.first_row) + rows * 2 - 1) << 16 | (frame.width - 1);

        self.mmio.write(u32, regs.canvas_tl_pos.offset, top);
        self.mmio.write(u32, regs.canvas_br_pos.offset, bottom);
        self.mmio.write(u32, regs.canvas_bg.offset, regs.canvas_bg.all_blending_bypass_mask);
        self.mmio.write(u32, regs.layer0_tl_pos.offset, top);
        self.mmio.write(u32, regs.layer0_br_pos.offset, bottom);
        self.mmio.write(u32, regs.layer0_src.offset, source);
        self.mmio.write(u32, regs.layer0_fill.offset, 0);

        const layer_config = @as(u32, @backingInt(frame.format)) |
            (@as(u32, 255) << regs.layer0_config.alpha_shift) |
            (@as(u32, frame.stride_bytes) << regs.layer0_config.width_shift) |
            regs.layer0_config.prefetch_en_mask |
            regs.layer0_config.line_fetch_mode_mask |
            regs.layer0_config.active_mask;
        self.mmio.write(u32, regs.layer0_config.offset, layer_config);

        const start_line = (@as(u32, self.config.bank_row_head) + region.first_row) * 2 + 1;
        const end_line = (@as(u32, self.config.bank_row_head) + region.last_row + 1) * 2;
        const start_column = @as(u32, self.config.bank_column_head) / 2;
        const end_column = (@as(u32, self.config.bank_column_head) + self.config.valid_columns) / 2 - 1;
        self.mmio.write(u32, regs.jdi_par_conf2.offset, start_line << 16 | end_line);
        self.mmio.write(u32, regs.jdi_par_conf3.offset, start_column << 16 | end_column);
        self.mmio.write(u32, regs.jdi_par_conf9.offset, (start_line + 1) << 16 | (end_line + 1));
        self.mmio.write(u32, regs.jdi_par_conf10.offset, start_line << 16 | (end_line + 2));

        var control = self.mmio.read(u32, regs.jdi_par_ctrl.offset);
        control &= ~(regs.jdi_par_ctrl.int_line_num_mask | regs.jdi_par_ctrl.enable_mask);
        control |= (@as(u32, self.max_line) / 2 - 1) << regs.jdi_par_ctrl.int_line_num_shift;
        control |= regs.jdi_par_ctrl.xrst_mask;
        self.mmio.write(u32, regs.jdi_par_ctrl.offset, control);

        var setting = self.mmio.read(u32, regs.setting.offset);
        setting |= regs.setting.eof_mask_mask | regs.setting.jdi_parl_intr_mask_mask | regs.setting.jdi_par_udr_mask_mask;
        self.mmio.write(u32, regs.setting.offset, setting);
        self.delay.wait(35);
        self.mmio.write(u32, regs.jdi_par_ctrl.offset, control | regs.jdi_par_ctrl.enable_mask);
        self.delay.wait(1);
        self.phase = .sending;
    }

    /// Clear and service LCDC1's interrupt status. Firmware should call this
    /// from its vector/interrupt layer, which remains outside Conduit's scope.
    pub fn handleInterrupt(self: *Lcdc1) Event {
        const irq = self.mmio.read(u32, regs.irq.offset);
        if (irq == 0) return .none;
        self.mmio.write(u32, regs.irq.offset, irq);

        if (irq & regs.irq.jdi_par_udr_raw_stat_mask != 0) {
            self.stopTransfer();
            return .underrun;
        }

        if (irq & regs.irq.jdi_parl_intr_raw_stat_mask != 0) {
            var control = self.mmio.read(u32, regs.jdi_par_ctrl.offset);
            const interrupt_line = (control & regs.jdi_par_ctrl.int_line_num_mask) >> regs.jdi_par_ctrl.int_line_num_shift;
            if (interrupt_line != self.max_line - 1) {
                control &= ~(regs.jdi_par_ctrl.int_line_num_mask | regs.jdi_par_ctrl.enable_mask);
                control |= @as(u32, self.max_line - 1) << regs.jdi_par_ctrl.int_line_num_shift;
                self.mmio.write(u32, regs.jdi_par_ctrl.offset, control);
                self.phase = .tail;
                return .frame_consumed;
            }

            self.mmio.write(u32, regs.setting.offset, self.mmio.read(u32, regs.setting.offset) & ~regs.setting.jdi_parl_intr_mask_mask);
            self.mmio.write(u32, regs.jdi_par_ctrl.offset, control & ~regs.jdi_par_ctrl.xrst_mask);
            self.mmio.write(u32, regs.jdi_par_ex_ctrl.offset, self.mmio.read(u32, regs.jdi_par_ex_ctrl.offset) | regs.jdi_par_ex_ctrl.cnt_en_mask);
        }

        if (irq & regs.irq.eof_stat_mask != 0) {
            self.stopTransfer();
            return .complete;
        }
        return .none;
    }

    pub fn abort(self: *Lcdc1) void {
        self.stopTransfer();
    }

    fn stopTransfer(self: *Lcdc1) void {
        self.mmio.write(u32, regs.setting.offset, self.mmio.read(u32, regs.setting.offset) &
            ~(regs.setting.eof_mask_mask | regs.setting.jdi_parl_intr_mask_mask | regs.setting.jdi_par_udr_mask_mask));
        self.mmio.write(u32, regs.jdi_par_ctrl.offset, self.mmio.read(u32, regs.jdi_par_ctrl.offset) &
            ~(regs.jdi_par_ctrl.enable_mask | regs.jdi_par_ctrl.xrst_mask));
        self.phase = .idle;
    }
};

pub fn bind(mmio: Mmio, delay: Delay, config: JdiConfig) Error!Lcdc1 {
    if (config.lcdc_clock_hz == 0 or config.hck_hz == 0 or config.valid_columns == 0 or config.valid_rows == 0)
        return error.InvalidClock;
    const columns_with_banks = @as(u32, config.bank_column_head) + config.valid_columns + config.bank_column_tail;
    const rows_with_banks = @as(u32, config.bank_row_head) + config.valid_rows + config.bank_row_tail;
    if (columns_with_banks & 1 != 0) return error.InvalidClock;
    const max_column = columns_with_banks / 2;
    const max_line = rows_with_banks * 2;
    const hck_ticks = ((config.lcdc_clock_hz + config.hck_hz - 1) / config.hck_hz) >> 1;
    const vck_ticks = hck_ticks * max_column;
    if (hck_ticks == 0 or hck_ticks > 0xffff or vck_ticks > 0xffff or max_column > 0xffff or max_line > 0xffff)
        return error.InvalidClock;

    mmio.write(u32, regs.lcd_conf.offset, @as(u32, 5) << regs.lcd_conf.lcd_intf_sel_shift);
    mmio.write(u32, regs.jdi_par_conf1.offset, (max_line - 1) << 16 | (max_column - 1));
    mmio.write(u32, regs.jdi_par_conf4.offset, hck_ticks << 16 | hck_ticks);
    mmio.write(u32, regs.jdi_par_conf5.offset, vck_ticks << 16 | vck_ticks);
    mmio.write(u32, regs.jdi_par_conf6.offset, (vck_ticks / 2) << 16 | hck_ticks);
    mmio.write(u32, regs.jdi_par_conf7.offset, regs.jdi_par_conf7.dp_mode_mask | hck_ticks / 2);
    mmio.write(u32, regs.jdi_par_conf8.offset, @as(u32, config.enable_start_column) << 16 | config.enable_end_column);
    mmio.write(u32, regs.jdi_par_ctrl.offset, 0);
    mmio.write(u32, regs.layer0_config.offset, 0);

    return .{
        .mmio = mmio,
        .delay = delay,
        .config = config,
        .max_column = @intCast(max_column),
        .max_line = @intCast(max_line),
    };
}

/// Translate an HCPU cached MPI window address to the system-bus alias LCDC
/// needs. SRAM and already-translated addresses pass through unchanged.
pub fn systemBusAddress(cpu_address: u32) u32 {
    if (cpu_address >= 0x1000_0000 and cpu_address < 0x2000_0000)
        return cpu_address + 0x5000_0000;
    return cpu_address;
}

pub fn fromBase(delay: Delay, config: JdiConfig) Error!Lcdc1 {
    return bind(Mmio.direct(regs.base_address), delay, config);
}
