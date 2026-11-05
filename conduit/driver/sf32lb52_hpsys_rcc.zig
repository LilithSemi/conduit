//! SF32LB52 high-performance subsystem reset and clock controller.
//!
//! Gate changes use the hardware ESR/ECR atomic aliases. Reset bits have no
//! aliases, so callers must serialize reset operations if multiple execution
//! contexts can touch RSTR1/RSTR2 concurrently.

const Mmio = @import("../mmio.zig");
const regs = @import("sf32lb52_regs").hpsys_rcc;
const cfg_regs = @import("sf32lb52_regs").hpsys_cfg;

pub const Error = error{ InvalidFrequency, Timeout };
pub const Dll = enum { dll1, dll2 };
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

pub const Module = enum {
    dmac1,
    mailbox1,
    pinmux1,
    usart1,
    usart2,
    ezip1,
    epic,
    lcdc1,
    i2s1,
    syscfg1,
    efusec,
    aes,
    crc1,
    trng,
    gptim1,
    gptim2,
    btim1,
    btim2,
    spi1,
    spi2,
    extdma,
    secu1,
    pdm1,
    i2c1,
    i2c2,
    ptc1,
    gpio1,
    mpi1,
    mpi2,
    sdmmc1,
    usbc,
    i2c3,
    atim1,
    usart3,
    audcodec,
    audprc,
    gpadc,
    tsen,
    i2c4,
};

pub const SystemSource = enum(u2) { hrc48 = 0, hxt48 = 1, dll1 = 3 };
pub const PeripheralSource = enum(u1) { hrc48 = 0, hxt48 = 1 };
pub const MpiSource = enum(u2) { peripheral = 0, dll1 = 1, dll2 = 2 };
pub const TickSource = enum(u2) { rtc = 0, hrc48 = 2, hxt48 = 3 };
pub const PclkDivider = enum(u3) {
    div1 = 0,
    div2 = 1,
    div4 = 2,
    div8 = 3,
    div16 = 4,
    div32 = 5,
    div64 = 6,
    div128 = 7,
};

pub const Frequencies = struct {
    hrc48_hz: u32 = 48_000_000,
    hxt48_hz: u32 = 48_000_000,
    dll1_hz: u32 = 0,
    dll2_hz: u32 = 0,
    rtc_hz: u32 = 32_768,
};

pub const Sf32lb52HpsysRcc = struct {
    mmio: Mmio,
    cfg_mmio: ?Mmio,

    /// Start DLL1 or DLL2 at an exactly representable 24 MHz step. When this
    /// controller was bound with HPSYS_CFG, the shared analog bandgap is
    /// enabled first as required by SiFli's HAL.
    pub fn enableDll(self: Sf32lb52HpsysRcc, dll: Dll, frequency_hz: u32, delay: Delay, poll_limit: u32) Error!void {
        const stage = try dllStage(frequency_hz);
        if (self.cfg_mmio) |cfg| {
            cfg.write(u32, cfg_regs.anau_cr.offset, cfg.read(u32, cfg_regs.anau_cr.offset) | cfg_regs.anau_cr.en_bg_mask);
        }

        const register = dllRegister(dll);
        var control = self.mmio.read(u32, register.offset);
        control &= ~(register.en_mask | register.ready_mask | register.stg_mask | register.out_div2_mask);
        self.mmio.write(u32, register.offset, control);
        control |= register.in_div2_mask | (@as(u32, stage) << register.stg_shift) | register.en_mask;
        self.mmio.write(u32, register.offset, control);
        delay.wait(10);

        var remaining = poll_limit;
        while (remaining != 0) : (remaining -= 1) {
            if (self.mmio.read(u32, register.offset) & register.ready_mask != 0) return;
        }
        self.mmio.write(u32, register.offset, control & ~register.en_mask);
        return error.Timeout;
    }

    pub fn disableDll(self: Sf32lb52HpsysRcc, dll: Dll) void {
        const register = dllRegister(dll);
        self.mmio.write(u32, register.offset, self.mmio.read(u32, register.offset) & ~register.en_mask);
    }

    pub fn dllFrequency(self: Sf32lb52HpsysRcc, dll: Dll) u32 {
        const register = dllRegister(dll);
        const control = self.mmio.read(u32, register.offset);
        if (control & register.en_mask == 0) return 0;
        const stage = (control & register.stg_mask) >> register.stg_shift;
        return 24_000_000 + stage * 24_000_000;
    }

    /// Enable a module clock. Returns false for an always-clocked block such
    /// as USART1, which has a reset bit but no gate bit on SF32LB52.
    pub fn enable(self: Sf32lb52HpsysRcc, module: Module) bool {
        const bit = gateBit(module) orelse return false;
        self.mmio.write(u32, if (bit.bank == .one) regs.esr1.offset else regs.esr2.offset, bit.mask);
        return true;
    }

    /// Disable a module clock through the atomic clear alias.
    pub fn disable(self: Sf32lb52HpsysRcc, module: Module) bool {
        const bit = gateBit(module) orelse return false;
        self.mmio.write(u32, if (bit.bank == .one) regs.ecr1.offset else regs.ecr2.offset, bit.mask);
        return true;
    }

    pub fn isEnabled(self: Sf32lb52HpsysRcc, module: Module) bool {
        const bit = gateBit(module) orelse return true;
        const value = self.mmio.read(u32, if (bit.bank == .one) regs.enr1.offset else regs.enr2.offset);
        return value & bit.mask != 0;
    }

    pub fn assertReset(self: Sf32lb52HpsysRcc, module: Module) bool {
        return self.updateReset(module, true);
    }

    pub fn deassertReset(self: Sf32lb52HpsysRcc, module: Module) bool {
        return self.updateReset(module, false);
    }

    /// Generate the same set/clear reset pulse used by SiFli's official HAL.
    pub fn reset(self: Sf32lb52HpsysRcc, module: Module) bool {
        if (!self.assertReset(module)) return false;
        return self.deassertReset(module);
    }

    pub fn selectSystemSource(self: Sf32lb52HpsysRcc, source: SystemSource) void {
        self.updateField(regs.csr.offset, regs.csr.sel_sys_mask, regs.csr.sel_sys_shift, @backingInt(source));
    }

    pub fn selectPeripheralSource(self: Sf32lb52HpsysRcc, source: PeripheralSource) void {
        self.updateField(regs.csr.offset, regs.csr.sel_peri_mask, regs.csr.sel_peri_shift, @backingInt(source));
    }

    pub fn selectMpiSource(self: Sf32lb52HpsysRcc, mpi: enum { mpi1, mpi2 }, source: MpiSource) void {
        const mask = if (mpi == .mpi1) regs.csr.sel_mpi1_mask else regs.csr.sel_mpi2_mask;
        const shift = if (mpi == .mpi1) regs.csr.sel_mpi1_shift else regs.csr.sel_mpi2_shift;
        self.updateField(regs.csr.offset, mask, shift, @backingInt(source));
    }

    pub fn selectTickSource(self: Sf32lb52HpsysRcc, source: TickSource) void {
        self.updateField(regs.csr.offset, regs.csr.sel_tick_mask, regs.csr.sel_tick_shift, @backingInt(source));
    }

    /// Configure HCLK and the two peripheral buses. HDIV=0 and HDIV=1 both
    /// divide by one according to the SVD; larger values are literal divisors.
    pub fn configureBusDividers(
        self: Sf32lb52HpsysRcc,
        hdiv: u8,
        pdiv1: PclkDivider,
        pdiv2: PclkDivider,
    ) void {
        var value = self.mmio.read(u32, regs.cfgr.offset);
        value &= ~(regs.cfgr.hdiv_mask | regs.cfgr.pdiv1_mask | regs.cfgr.pdiv2_mask);
        value |= @as(u32, hdiv) << regs.cfgr.hdiv_shift;
        value |= @as(u32, @backingInt(pdiv1)) << regs.cfgr.pdiv1_shift;
        value |= @as(u32, @backingInt(pdiv2)) << regs.cfgr.pdiv2_shift;
        self.mmio.write(u32, regs.cfgr.offset, value);
    }

    pub fn configureTickDivider(self: Sf32lb52HpsysRcc, divider: u6) void {
        self.updateField(regs.cfgr.offset, regs.cfgr.tickdiv_mask, regs.cfgr.tickdiv_shift, divider);
    }

    pub fn systemFrequency(self: Sf32lb52HpsysRcc, clocks: Frequencies) u32 {
        const source: u2 = @truncate((self.mmio.read(u32, regs.csr.offset) & regs.csr.sel_sys_mask) >> regs.csr.sel_sys_shift);
        return switch (source) {
            0 => clocks.hrc48_hz,
            1 => clocks.hxt48_hz,
            3 => clocks.dll1_hz,
            else => 0,
        };
    }

    pub fn peripheralFrequency(self: Sf32lb52HpsysRcc, clocks: Frequencies) u32 {
        return if (self.mmio.read(u32, regs.csr.offset) & regs.csr.sel_peri_mask == 0)
            clocks.hrc48_hz
        else
            clocks.hxt48_hz;
    }

    pub fn hclkFrequency(self: Sf32lb52HpsysRcc, clocks: Frequencies) u32 {
        const divider: u8 = @truncate((self.mmio.read(u32, regs.cfgr.offset) & regs.cfgr.hdiv_mask) >> regs.cfgr.hdiv_shift);
        return self.systemFrequency(clocks) / @max(divider, 1);
    }

    pub fn pclkFrequency(self: Sf32lb52HpsysRcc, bus: enum { pclk1, pclk2 }, clocks: Frequencies) u32 {
        const value = self.mmio.read(u32, regs.cfgr.offset);
        const exponent: u3 = @truncate(if (bus == .pclk1)
            (value & regs.cfgr.pdiv1_mask) >> regs.cfgr.pdiv1_shift
        else
            (value & regs.cfgr.pdiv2_mask) >> regs.cfgr.pdiv2_shift);
        return self.hclkFrequency(clocks) / (@as(u32, 1) << exponent);
    }

    fn updateReset(self: Sf32lb52HpsysRcc, module: Module, asserted: bool) bool {
        const bit = resetBit(module) orelse return false;
        const offset: usize = if (bit.bank == .one) regs.rstr1.offset else regs.rstr2.offset;
        const old = self.mmio.read(u32, offset);
        self.mmio.write(u32, offset, if (asserted) old | bit.mask else old & ~bit.mask);
        return true;
    }

    fn updateField(self: Sf32lb52HpsysRcc, offset: usize, mask: u32, shift: u32, raw: anytype) void {
        const old = self.mmio.read(u32, offset);
        const bit_shift: u5 = @intCast(shift);
        const value = (old & ~mask) | ((@as(u32, @intCast(raw)) << bit_shift) & mask);
        self.mmio.write(u32, offset, value);
    }
};

const Bank = enum { one, two };
const Bit = struct { bank: Bank, mask: u32 };
const DllRegister = struct {
    offset: usize,
    en_mask: u32,
    ready_mask: u32,
    stg_mask: u32,
    stg_shift: u5,
    in_div2_mask: u32,
    out_div2_mask: u32,
};

fn dllRegister(dll: Dll) DllRegister {
    return switch (dll) {
        .dll1 => makeDllRegister(regs.dll1_cr),
        .dll2 => makeDllRegister(regs.dll2_cr),
    };
}

fn makeDllRegister(comptime r: type) DllRegister {
    return .{
        .offset = r.offset,
        .en_mask = r.en_mask,
        .ready_mask = r.ready_mask,
        .stg_mask = r.stg_mask,
        .stg_shift = @intCast(r.stg_shift),
        .in_div2_mask = r.in_div2_en_mask,
        .out_div2_mask = r.out_div2_en_mask,
    };
}

fn dllStage(frequency_hz: u32) Error!u4 {
    if (frequency_hz < 24_000_000 or frequency_hz > 384_000_000) return error.InvalidFrequency;
    const delta = frequency_hz - 24_000_000;
    if (delta % 24_000_000 != 0) return error.InvalidFrequency;
    return @intCast(delta / 24_000_000);
}

fn gateBit(module: Module) ?Bit {
    return switch (module) {
        .usart1 => null,
        .dmac1 => .{ .bank = .one, .mask = regs.enr1.dmac1_mask },
        .mailbox1 => .{ .bank = .one, .mask = regs.enr1.mailbox1_mask },
        .pinmux1 => .{ .bank = .one, .mask = regs.enr1.pinmux1_mask },
        .usart2 => .{ .bank = .one, .mask = regs.enr1.usart2_mask },
        .ezip1 => .{ .bank = .one, .mask = regs.enr1.ezip1_mask },
        .epic => .{ .bank = .one, .mask = regs.enr1.epic_mask },
        .lcdc1 => .{ .bank = .one, .mask = regs.enr1.lcdc1_mask },
        .i2s1 => .{ .bank = .one, .mask = regs.enr1.i2_s1_mask },
        .syscfg1 => .{ .bank = .one, .mask = regs.enr1.syscfg1_mask },
        .efusec => .{ .bank = .one, .mask = regs.enr1.efusec_mask },
        .aes => .{ .bank = .one, .mask = regs.enr1.aes_mask },
        .crc1 => .{ .bank = .one, .mask = regs.enr1.crc1_mask },
        .trng => .{ .bank = .one, .mask = regs.enr1.trng_mask },
        .gptim1 => .{ .bank = .one, .mask = regs.enr1.gptim1_mask },
        .gptim2 => .{ .bank = .one, .mask = regs.enr1.gptim2_mask },
        .btim1 => .{ .bank = .one, .mask = regs.enr1.btim1_mask },
        .btim2 => .{ .bank = .one, .mask = regs.enr1.btim2_mask },
        .spi1 => .{ .bank = .one, .mask = regs.enr1.spi1_mask },
        .spi2 => .{ .bank = .one, .mask = regs.enr1.spi2_mask },
        .extdma => .{ .bank = .one, .mask = regs.enr1.extdma_mask },
        .secu1 => .{ .bank = .one, .mask = regs.enr1.secu1_mask },
        .pdm1 => .{ .bank = .one, .mask = regs.enr1.pdm1_mask },
        .i2c1 => .{ .bank = .one, .mask = regs.enr1.i2_c1_mask },
        .i2c2 => .{ .bank = .one, .mask = regs.enr1.i2_c2_mask },
        .ptc1 => .{ .bank = .one, .mask = regs.enr1.ptc1_mask },
        .gpio1 => .{ .bank = .two, .mask = regs.enr2.gpio1_mask },
        .mpi1 => .{ .bank = .two, .mask = regs.enr2.mpi1_mask },
        .mpi2 => .{ .bank = .two, .mask = regs.enr2.mpi2_mask },
        .sdmmc1 => .{ .bank = .two, .mask = regs.enr2.sdmmc1_mask },
        .usbc => .{ .bank = .two, .mask = regs.enr2.usbc_mask },
        .i2c3 => .{ .bank = .two, .mask = regs.enr2.i2_c3_mask },
        .atim1 => .{ .bank = .two, .mask = regs.enr2.atim1_mask },
        .usart3 => .{ .bank = .two, .mask = regs.enr2.usart3_mask },
        .audcodec => .{ .bank = .two, .mask = regs.enr2.audcodec_mask },
        .audprc => .{ .bank = .two, .mask = regs.enr2.audprc_mask },
        .gpadc => .{ .bank = .two, .mask = regs.enr2.gpadc_mask },
        .tsen => .{ .bank = .two, .mask = regs.enr2.tsen_mask },
        .i2c4 => .{ .bank = .two, .mask = regs.enr2.i2_c4_mask },
    };
}

fn resetBit(module: Module) ?Bit {
    if (module == .secu1) return null;
    if (module == .usart1) return .{ .bank = .one, .mask = regs.rstr1.usart1_mask };
    const gate = gateBit(module) orelse return null;
    // All remaining SF32LB52 reset positions match their corresponding enable
    // positions. The PAC exposes both maps; this equality is hardware-defined.
    return gate;
}

pub fn bind(mmio: Mmio) Sf32lb52HpsysRcc {
    return .{ .mmio = mmio, .cfg_mmio = null };
}

pub fn bindWithConfig(mmio: Mmio, cfg_mmio: Mmio) Sf32lb52HpsysRcc {
    return .{ .mmio = mmio, .cfg_mmio = cfg_mmio };
}

pub fn fromBase() Sf32lb52HpsysRcc {
    return bindWithConfig(Mmio.direct(regs.base_address), Mmio.direct(cfg_regs.base_address));
}
