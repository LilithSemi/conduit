//! SF32LB52 DMAC1 eight-channel DMA driver.

const Mmio = @import("../mmio.zig");
const pac = @import("sf32lb52_regs");
const regs = pac.dmac1;

pub const Channel = enum(u3) { ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8 };
pub const Direction = enum { peripheral_to_memory, memory_to_peripheral, memory_to_memory };
pub const Width = enum(u2) { byte, half, word };
pub const Priority = enum(u2) { low, medium, high, very_high };

pub const Request = enum(u6) {
    mpi1 = 0,
    mpi2 = 1,
    i2c4 = 3,
    usart1_tx = 4,
    usart1_rx = 5,
    usart2_tx = 6,
    usart2_rx = 7,
    gptim1_update = 8,
    gptim1_trigger = 9,
    gptim1_cc1 = 10,
    gptim1_cc2 = 11,
    gptim1_cc3 = 12,
    gptim1_cc4 = 13,
    btim1 = 14,
    btim2 = 15,
    atim1_update = 16,
    atim1_trigger = 17,
    atim1_cc1 = 18,
    atim1_cc2 = 19,
    atim1_cc3 = 20,
    atim1_cc4 = 21,
    i2c1 = 22,
    i2c2 = 23,
    i2c3 = 24,
    atim1_com = 25,
    usart3_tx = 26,
    usart3_rx = 27,
    spi1_tx = 28,
    spi1_rx = 29,
    spi2_tx = 30,
    spi2_rx = 31,
    i2s1_tx = 32,
    i2s1_rx = 33,
    pdm1_left = 36,
    pdm1_right = 37,
    gpadc = 38,
    audcodec_adc0 = 39,
    audcodec_adc1 = 40,
    audcodec_dac0 = 41,
    audcodec_dac1 = 42,
    gptim2_update = 43,
    gptim2_trigger = 44,
    gptim2_cc1 = 45,
    audprc_tx_out1 = 46,
    audprc_tx_out0 = 47,
    audprc_tx3 = 48,
    audprc_tx2 = 49,
    audprc_tx1 = 50,
    audprc_tx0 = 51,
    audprc_rx1 = 52,
    audprc_rx0 = 53,
    gptim2_cc2 = 54,
    gptim2_cc3 = 55,
    gptim2_cc4 = 56,
    sdmmc1 = 57,
};

pub const Config = struct {
    request: Request,
    direction: Direction,
    memory_width: Width = .byte,
    peripheral_width: Width = .byte,
    memory_increment: bool = true,
    peripheral_increment: bool = false,
    circular: bool = false,
    priority: Priority = .low,
    burst_size: u8 = 0,
    transfer_interrupt: bool = false,
    half_interrupt: bool = false,
    error_interrupt: bool = true,
};

pub const Error = error{ Busy, InvalidCount, Timeout, TransferError };
pub const Status = struct { enabled: bool, complete: bool, half: bool, transfer_error: bool };

pub const Dmac1 = struct {
    mmio: Mmio,
    poll_limit: u32,

    pub fn configure(self: *Dmac1, channel: Channel, config: Config) Error!void {
        const off = channelOffset(channel);
        if (self.mmio.read(u32, off) & 1 != 0) return error.Busy;
        var control = @as(u32, @backingInt(config.priority)) << 12 |
            @as(u32, @backingInt(config.memory_width)) << 10 |
            @as(u32, @backingInt(config.peripheral_width)) << 8;
        if (config.memory_increment) control |= 1 << 7;
        if (config.peripheral_increment) control |= 1 << 6;
        if (config.circular) control |= 1 << 5;
        if (config.direction == .memory_to_peripheral) control |= 1 << 4;
        if (config.direction == .memory_to_memory) control |= 1 << 14;
        if (config.error_interrupt) control |= 1 << 3;
        if (config.half_interrupt) control |= 1 << 2;
        if (config.transfer_interrupt) control |= 1 << 1;
        self.mmio.write(u32, off, control);
        self.mmio.write(u32, off + 0x10, config.burst_size);
        self.selectRequest(channel, config.request);
    }

    /// Start a configured channel. `peripheral` is CPAR and `memory` is CM0AR;
    /// for memory-to-memory, peripheral is the destination and memory the source.
    pub fn start(self: *Dmac1, channel: Channel, peripheral: u32, memory: u32, count: u16) Error!void {
        if (count == 0) return error.InvalidCount;
        const off = channelOffset(channel);
        const control = self.mmio.read(u32, off);
        if (control & 1 != 0) return error.Busy;
        self.clearFlags(channel);
        self.mmio.write(u32, off + 4, count);
        self.mmio.write(u32, off + 8, peripheral);
        self.mmio.write(u32, off + 12, memory);
        self.mmio.write(u32, off, control | 1);
    }

    pub fn stop(self: *Dmac1, channel: Channel) void {
        const off = channelOffset(channel);
        self.mmio.write(u32, off, self.mmio.read(u32, off) & ~@as(u32, 1));
        self.clearFlags(channel);
    }

    pub fn poll(self: *Dmac1, channel: Channel) Error!void {
        var remaining = self.poll_limit;
        while (remaining != 0) : (remaining -= 1) {
            const flags = self.status(channel);
            if (flags.transfer_error) {
                self.clearFlags(channel);
                return error.TransferError;
            }
            if (flags.complete) {
                self.clearFlags(channel);
                return;
            }
        }
        return error.Timeout;
    }

    pub fn status(self: *const Dmac1, channel: Channel) Status {
        const shift = flagShift(channel);
        const flags = self.mmio.read(u32, regs.isr.offset) >> shift;
        return .{
            .enabled = self.mmio.read(u32, channelOffset(channel)) & 1 != 0,
            .complete = flags & 2 != 0,
            .half = flags & 4 != 0,
            .transfer_error = flags & 8 != 0,
        };
    }

    pub fn clearFlags(self: *const Dmac1, channel: Channel) void {
        self.mmio.write(u32, regs.ifcr.offset, @as(u32, 0xf) << flagShift(channel));
    }

    fn selectRequest(self: *Dmac1, channel: Channel, request: Request) void {
        const index: u3 = @backingInt(channel);
        const off: usize = if (index < 4) regs.cselr1.offset else regs.cselr2.offset;
        const shift: u5 = @as(u5, index & 3) * 8;
        var value = self.mmio.read(u32, off);
        value = (value & ~(@as(u32, 0x3f) << shift)) | (@as(u32, @backingInt(request)) << shift);
        self.mmio.write(u32, off, value);
    }
};

fn channelOffset(channel: Channel) usize {
    return regs.ccr1.offset + @as(usize, @backingInt(channel)) * 0x14;
}

fn flagShift(channel: Channel) u5 {
    return @intCast(@as(u5, @backingInt(channel)) * 4);
}

pub fn bind(mmio: Mmio, poll_limit: u32) Dmac1 {
    return .{ .mmio = mmio, .poll_limit = poll_limit };
}

pub fn fromBase(poll_limit: u32) Dmac1 {
    return bind(Mmio.direct(pac.dmac1.base_address), poll_limit);
}
