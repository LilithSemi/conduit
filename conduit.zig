//! conduit: a hardware abstraction layer for Zig.
//!
//! Weir (RISC-V firmware) and Ferrite (microkernel) share it. `Mmio`,
//! `Backend`, and `Resource` bridge two decoupled halves:
//!
//!   * Discovery: a pluggable `Backend` (device tree, ACPI, or a host's own)
//!     feeds a match-rule `Registry` (runtime) or `Builder` (comptime). Each
//!     one iterates every device of a class.
//!   * Driving: shipped drivers (`driver.*`) implement device-class contracts
//!     (`device.*`). Each driver binds over an `Mmio`.
//!
//! See docs/superpowers/specs/2026-06-09-conduit-hal-design.md for the design.

/// Register constants generated from the SF32LB52 SVD pinned by this package.
pub const sf32lb52 = @import("sf32lb52_regs");

pub const config = @import("conduit/config.zig");

pub const dtree = if (config.have_dtree) @import("dtree") else struct {};
pub const almanac = if (config.have_almanac) @import("almanac") else struct {};

pub const Mmio = @import("conduit/mmio.zig");

pub const resource = @import("conduit/resource.zig");
pub const Resource = resource.Resource;
pub const ResourceList = resource.List;

pub const match = @import("conduit/match.zig");
pub const Class = match.Class;
pub const Matcher = match.Matcher;
pub const IdList = match.IdList;

const builtin = @import("builtin");

const backend_iface = @import("conduit/backend.zig");
pub const Backend = backend_iface.Backend;

/// The PCI backend builds only for the two OS targets it has an impl for (Linux
/// /sys and UEFI EFI_PCI_IO). On any other target (e.g. the freestanding
/// `conduit_bare` module) it stays an empty struct so the host-OS code never
/// compiles, even with `-Dpci` on.
const pci_supported = builtin.os.tag == .linux or builtin.os.tag == .uefi;

pub const backend = struct {
    pub const Backend = backend_iface.Backend;
    pub const Node = backend_iface.Node;
    pub const PciInfo = backend_iface.PciInfo;
    pub const Error = backend_iface.Error;
    pub const fromImpl = backend_iface.fromImpl;
    pub const dtree = if (config.have_dtree) @import("conduit/backend/dtree.zig") else struct {};
    pub const almanac = if (config.have_almanac) @import("conduit/backend/almanac.zig") else struct {};
    pub const pci = if (config.have_pci and pci_supported) @import("conduit/backend/pci.zig") else struct {};
};

pub const discover = @import("conduit/discover.zig");
pub const Registry = discover.Registry;
pub const Builder = discover.Builder;
pub const Match = discover.Match;
pub const ofClass = discover.ofClass;

/// Device-class contracts (vtables).
pub const device = struct {
    pub const Serial = @import("conduit/device/serial.zig");
    pub const Block = @import("conduit/device/block.zig");
    pub const Gpio = @import("conduit/device/gpio.zig");
    pub const I2c = @import("conduit/device/i2c.zig");
    pub const Spi = @import("conduit/device/spi.zig");
    pub const Rtc = @import("conduit/device/rtc.zig");
    pub const Intc = @import("conduit/device/intc.zig");
    pub const Sd = @import("conduit/device/sd.zig");
    pub const Watchdog = @import("conduit/device/watchdog.zig");
    pub const Entropy = @import("conduit/device/entropy.zig");
    pub const Temperature = @import("conduit/device/temperature.zig");
    pub const Pwm = @import("conduit/device/pwm.zig");
};

/// Shipped driver implementations. Each exports `bind(Mmio, ...)`, and the
/// DT/ACPI-discoverable ones also export a `matcher`. Statically-mapped devices
/// (bound directly over an `Mmio`, e.g. the Albion secure-element blocks) have
/// no matcher.
pub const driver = struct {
    pub const ns16550a = @import("conduit/driver/ns16550a.zig");
    pub const pl011 = @import("conduit/driver/pl011.zig");
    pub const harbor_gpio = @import("conduit/driver/harbor_gpio.zig");
    pub const harbor_i2c = @import("conduit/driver/harbor_i2c.zig");
    pub const harbor_spi = @import("conduit/driver/harbor_spi.zig");
    pub const plic = @import("conduit/driver/plic.zig");
    pub const clint = @import("conduit/driver/clint.zig");
    pub const gicv2 = @import("conduit/driver/gicv2.zig");
    pub const gicv3 = @import("conduit/driver/gicv3.zig");
    pub const harbor_sdio = @import("conduit/driver/harbor_sdio.zig");
    /// Deprecated spelling of [harbor_sdio]. The controller is Harbor's own
    /// register map, not the SD Host Controller Standard one.
    pub const sdhci = harbor_sdio;
    pub const sd_spi = @import("conduit/driver/sd_spi.zig");
    pub const virtio_blk = @import("conduit/driver/virtio_blk.zig");
    pub const virtio_gpu = @import("conduit/driver/virtio_gpu.zig");
    pub const goldfish_rtc = @import("conduit/driver/goldfish_rtc.zig");
    pub const pl031 = @import("conduit/driver/pl031.zig");
    pub const albion_ftpm = @import("conduit/driver/albion_ftpm.zig");
    pub const albion_pcr = @import("conduit/driver/albion_pcr.zig");
    pub const albion_ed25519 = @import("conduit/driver/albion_ed25519.zig");
    pub const albion_integrity = @import("conduit/driver/albion_integrity.zig");
    pub const albion_gpc = @import("conduit/driver/albion_gpc.zig");
    pub const albion_rmm = @import("conduit/driver/albion_rmm.zig");
    pub const albion_measure = @import("conduit/driver/albion_measure.zig");
    pub const albion_sealver = @import("conduit/driver/albion_sealver.zig");
    pub const albion_covi = @import("conduit/driver/albion_covi.zig");
    pub const albion_covi_inject = @import("conduit/driver/albion_covi_inject.zig");
    pub const albion_ed25519_sign = @import("conduit/driver/albion_ed25519_sign.zig");
    pub const albion_rotkdf = @import("conduit/driver/albion_rotkdf.zig");
    pub const harbor_trng = @import("conduit/driver/harbor_trng.zig");
    pub const sf32lb52_audio = @import("conduit/driver/sf32lb52_audio.zig");
    pub const sf32lb52_dmac = @import("conduit/driver/sf32lb52_dmac.zig");
    pub const sf32lb52_efuse = @import("conduit/driver/sf32lb52_efuse.zig");
    pub const sf32lb52_gpio = @import("conduit/driver/sf32lb52_gpio.zig");
    pub const sf32lb52_i2c = @import("conduit/driver/sf32lb52_i2c.zig");
    pub const sf32lb52_lcdc = @import("conduit/driver/sf32lb52_lcdc.zig");
    pub const sf32lb52_mpi = @import("conduit/driver/sf32lb52_mpi.zig");
    pub const sf32lb52_pdm = @import("conduit/driver/sf32lb52_pdm.zig");
    pub const sf32lb52_pmuc = @import("conduit/driver/sf32lb52_pmuc.zig");
    pub const sf32lb52_rtc = @import("conduit/driver/sf32lb52_rtc.zig");
    pub const sf32lb52_hpsys_rcc = @import("conduit/driver/sf32lb52_hpsys_rcc.zig");
    pub const sf32lb52_hpsys_cfg = @import("conduit/driver/sf32lb52_hpsys_cfg.zig");
    pub const sf32lb52_hpsys_aon = @import("conduit/driver/sf32lb52_hpsys_aon.zig");
    pub const sf32lb52_pinmux = @import("conduit/driver/sf32lb52_pinmux.zig");
    pub const sf32lb52_spi = @import("conduit/driver/sf32lb52_spi.zig");
    pub const sf32lb52_trng = @import("conduit/driver/sf32lb52_trng.zig");
    pub const sf32lb52_tsen = @import("conduit/driver/sf32lb52_tsen.zig");
    pub const sf32lb52_timer = @import("conduit/driver/sf32lb52_timer.zig");
    pub const sf32lb52_usart = @import("conduit/driver/sf32lb52_usart.zig");
    pub const sf32lb52_wdt = @import("conduit/driver/sf32lb52_wdt.zig");
};

/// A matcher table covering every shipped driver. A consumer can pass this
/// straight to a Registry/Builder, or assemble a narrower table by hand.
pub const all_matchers = [_]Matcher{
    driver.ns16550a.matcher,
    driver.pl011.matcher,
    driver.harbor_gpio.matcher,
    driver.harbor_i2c.matcher,
    driver.harbor_spi.matcher,
    driver.plic.matcher,
    driver.clint.matcher,
    driver.gicv2.matcher,
    driver.gicv3.matcher,
    driver.harbor_sdio.matcher,
    driver.sd_spi.matcher,
    driver.virtio_blk.matcher,
    driver.goldfish_rtc.matcher,
    driver.pl031.matcher,
};

// Unit tests: the inline `test` blocks live in the modules below (match, the
// device/driver register logic, the dtree decode/ranges helpers, etc).
// Integration tests live under test/ and run through the public module.
test {
    _ = match;
    _ = resource;
    _ = Mmio;
    _ = discover;
    inline for (.{
        device.Serial,   device.Block,   device.Gpio,        device.I2c,
        device.Spi,      device.Rtc,     device.Intc,        device.Sd,
        device.Watchdog, device.Entropy, device.Temperature, device.Pwm,
    }) |d| _ = d;
    inline for (.{
        driver.ns16550a,           driver.pl011,              driver.harbor_gpio,         driver.harbor_i2c,
        driver.harbor_spi,         driver.plic,               driver.clint,               driver.gicv2,
        driver.gicv3,              driver.harbor_sdio,        driver.sd_spi,              driver.virtio_blk,
        driver.virtio_gpu,         driver.goldfish_rtc,       driver.pl031,               driver.albion_ftpm,
        driver.albion_pcr,         driver.harbor_trng,        driver.albion_ed25519,      driver.albion_integrity,
        driver.albion_gpc,         driver.albion_rmm,         driver.albion_measure,      driver.albion_sealver,
        driver.albion_covi,        driver.albion_covi_inject, driver.albion_ed25519_sign, driver.albion_rotkdf,
        driver.sf32lb52_audio,     driver.sf32lb52_dmac,      driver.sf32lb52_efuse,      driver.sf32lb52_gpio,
        driver.sf32lb52_i2c,       driver.sf32lb52_lcdc,      driver.sf32lb52_mpi,        driver.sf32lb52_hpsys_rcc,
        driver.sf32lb52_hpsys_cfg, driver.sf32lb52_hpsys_aon, driver.sf32lb52_pdm,        driver.sf32lb52_pmuc,
        driver.sf32lb52_rtc,       driver.sf32lb52_pinmux,    driver.sf32lb52_spi,        driver.sf32lb52_trng,
        driver.sf32lb52_tsen,      driver.sf32lb52_timer,     driver.sf32lb52_usart,      driver.sf32lb52_wdt,
    }) |d| _ = d;
    if (config.have_dtree) _ = backend.dtree;
    if (config.have_almanac) _ = backend.almanac;
    if (config.have_pci and pci_supported) _ = backend.pci;
}

/// A matcher that claims every PCI device (any vendor or class). The string
/// `Registry`/`Builder` path then enumerates PCI nodes. A consumer refines on
/// `Match.pci` (the numeric vendor, device, and class). Pair it with `Class.pci`.
pub const pci_matcher = Matcher{ .class = .pci, .dt_compatible = &.{"pci"} };
