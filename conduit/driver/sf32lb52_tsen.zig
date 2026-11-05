//! SF32LB52 on-chip temperature sensor.
//!
//! The conversion sequence and fixed-point transfer function follow SiFli's
//! SF32LB52 HAL. Clock gating remains the RCC driver's responsibility.

const Mmio = @import("../mmio.zig");
const Temperature = @import("../device/temperature.zig");
const pac = @import("sf32lb52_regs");
const regs = pac.tsen;
const cfg_regs = pac.hpsys_cfg;

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

pub const Options = struct {
    pclk_hz: u32,
    poll_limit: u16 = 10,
    poll_interval_us: u32 = 1_000,
    /// Set and later release HPSYS_CFG.ANAU_CR.EN_BG. Turn this off when a
    /// board-level analog-power manager owns the shared bandgap reference.
    manage_bandgap: bool = true,
};

pub const Sf32lb52Tsen = struct {
    mmio: Mmio,
    cfg_mmio: Mmio,
    delay: Delay,
    options: Options,

    pub fn init(self: *Sf32lb52Tsen) Temperature.Error!void {
        if (self.options.pclk_hz == 0) return error.HardwareFault;
        if (self.options.manage_bandgap) {
            const old = self.cfg_mmio.read(u32, cfg_regs.anau_cr.offset);
            self.cfg_mmio.write(u32, cfg_regs.anau_cr.offset, old | cfg_regs.anau_cr.en_bg_mask);
        }
    }

    pub fn deinit(self: *Sf32lb52Tsen) void {
        self.powerDown();
        if (self.options.manage_bandgap) {
            const old = self.cfg_mmio.read(u32, cfg_regs.anau_cr.offset);
            self.cfg_mmio.write(u32, cfg_regs.anau_cr.offset, old & ~cfg_regs.anau_cr.en_bg_mask);
        }
    }

    pub fn readMilliCelsius(self: *Sf32lb52Tsen) Temperature.Error!i32 {
        if (self.options.pclk_hz == 0) return error.HardwareFault;

        var control = self.mmio.read(u32, regs.tsen_ctrl_reg.offset);
        const divider: u32 = @min(self.options.pclk_hz / 500_000, 0x3f);
        const clk_shift: u5 = @intCast(regs.tsen_ctrl_reg.anau_tsen_clk_div_shift);
        control = (control & ~regs.tsen_ctrl_reg.anau_tsen_clk_div_mask) |
            (divider << clk_shift);

        control &= ~regs.tsen_ctrl_reg.anau_tsen_rstb_mask;
        self.mmio.write(u32, regs.tsen_ctrl_reg.offset, control);
        control |= regs.tsen_ctrl_reg.anau_tsen_en_mask | regs.tsen_ctrl_reg.anau_tsen_pu_mask;
        self.mmio.write(u32, regs.tsen_ctrl_reg.offset, control);
        self.delay.wait(40);

        control |= regs.tsen_ctrl_reg.anau_tsen_rstb_mask;
        self.mmio.write(u32, regs.tsen_ctrl_reg.offset, control);
        self.delay.wait(20);
        control |= regs.tsen_ctrl_reg.anau_tsen_run_mask;
        self.mmio.write(u32, regs.tsen_ctrl_reg.offset, control);

        var polls: u16 = 0;
        while (self.mmio.read(u32, regs.tsen_irq.offset) & regs.tsen_irq.tsen_irsr_mask == 0) {
            if (polls >= self.options.poll_limit) {
                self.powerDown();
                return error.Timeout;
            }
            polls += 1;
            self.delay.wait(self.options.poll_interval_us);
        }

        self.mmio.write(u32, regs.tsen_irq.offset, regs.tsen_irq.tsen_icr_mask);
        const raw: u12 = @truncate(self.mmio.read(u32, regs.tsen_rdata.offset));
        self.powerDown();
        return rawToMilliCelsius(raw);
    }

    pub fn temperature(self: *Sf32lb52Tsen) Temperature {
        return Temperature.from(self);
    }

    fn powerDown(self: *Sf32lb52Tsen) void {
        const old = self.mmio.read(u32, regs.tsen_ctrl_reg.offset);
        self.mmio.write(
            u32,
            regs.tsen_ctrl_reg.offset,
            old & ~(regs.tsen_ctrl_reg.anau_tsen_en_mask | regs.tsen_ctrl_reg.anau_tsen_pu_mask),
        );
    }
};

/// SiFli's integer transfer function returns whole degrees Celsius. Convert it
/// to Conduit's milli-Celsius device contract without introducing floats.
pub fn rawToMilliCelsius(raw: u12) i32 {
    const biased: i32 = @as(i32, raw) + 3_000;
    const degrees = @divTrunc(biased * 749, 10_100) - 277;
    return degrees * 1_000;
}

pub fn bind(tsen_mmio: Mmio, cfg_mmio: Mmio, delay: Delay, options: Options) Temperature.Error!Sf32lb52Tsen {
    var sensor = Sf32lb52Tsen{
        .mmio = tsen_mmio,
        .cfg_mmio = cfg_mmio,
        .delay = delay,
        .options = options,
    };
    try sensor.init();
    return sensor;
}

pub fn fromBase(delay: Delay, options: Options) Temperature.Error!Sf32lb52Tsen {
    return bind(Mmio.direct(regs.base_address), Mmio.direct(cfg_regs.base_address), delay, options);
}

test "official SF32LB52 transfer function is signed milli-Celsius" {
    const std = @import("std");
    try std.testing.expectEqual(@as(i32, 25_000), rawToMilliCelsius(1_074));
    try std.testing.expectEqual(@as(i32, -55_000), rawToMilliCelsius(0));
}
