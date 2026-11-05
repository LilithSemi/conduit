//! SF32LB52 MPI2 serial-flash controller.
const Mmio = @import("../mmio.zig");
const pac = @import("sf32lb52_regs");
const regs = pac.mpi2;

pub const LineMode = enum(u3) { none = 0, single = 1, dual = 2, quad = 3, quad_dtr = 7 };
pub const Direction = enum { read, write };
pub const Command = struct {
    opcode: u8,
    address: u32 = 0,
    address_bytes: u3 = 0,
    instruction_mode: LineMode = .single,
    address_mode: LineMode = .none,
    data_mode: LineMode = .none,
    alternate_mode: LineMode = .none,
    alternate_bytes: u2 = 0,
    alternate: u32 = 0,
    dummy_cycles: u5 = 0,
    direction: Direction = .read,
};
pub const Options = struct { prescaler: u8 = 2, poll_limit: u32 = 1_000_000, timeout_cycles: u16 = 0xffff };
pub const Error = error{ InvalidLength, InvalidAddressSize, Timeout, WatchdogTimeout };

pub const Mpi2 = struct {
    mmio: Mmio,
    poll_limit: u32,

    pub fn execute(self: *Mpi2, command: Command, write_data: []const u8, read_data: []u8) Error!void {
        if (write_data.len != 0 and read_data.len != 0) return error.InvalidLength;
        if (command.address_bytes > 4) return error.InvalidAddressSize;
        const len = @max(write_data.len, read_data.len);
        if (len > 0x100000 or (len != 0 and command.data_mode == .none)) return error.InvalidLength;
        try self.waitNotBusy();
        self.mmio.write(u32, regs.scr.offset, regs.scr.tcfc_mask | regs.scr.smfc_mask);
        self.mmio.write(u32, regs.fifocr.offset, regs.fifocr.rxclr_mask | regs.fifocr.txclr_mask);
        if (len != 0) self.mmio.write(u32, regs.dlr1.offset, @intCast(len - 1));
        self.mmio.write(u32, regs.abr1.offset, command.alternate);
        self.mmio.write(u32, regs.ccr1.offset, encode(command));
        self.mmio.write(u32, regs.ar1.offset, command.address);
        var written: usize = 0;
        while (written < write_data.len and !self.txFull()) : (written += 1)
            self.mmio.write(u8, regs.dr.offset, write_data[written]);
        self.mmio.write(u32, regs.cmdr1.offset, command.opcode);
        while (written < write_data.len) : (written += 1) {
            try self.waitTxSpace();
            self.mmio.write(u8, regs.dr.offset, write_data[written]);
        }
        for (read_data) |*byte| {
            try self.waitRxData();
            byte.* = self.mmio.read(u8, regs.dr.offset);
        }
        try self.waitComplete();
        self.mmio.write(u32, regs.scr.offset, regs.scr.tcfc_mask);
    }

    pub fn readJedecId(self: *Mpi2) Error![3]u8 {
        var id: [3]u8 = undefined;
        try self.execute(.{ .opcode = 0x9f, .data_mode = .single }, &.{}, &id);
        return id;
    }
    pub fn enterFourByteAddressMode(self: *Mpi2) Error!void {
        try self.execute(.{ .opcode = 0xb7 }, &.{}, &.{});
    }
    /// The flash QE bit must already be set.
    pub fn configureGd25q256eMappedRead(self: *Mpi2) void {
        const command = Command{ .opcode = 0xec, .address_bytes = 4, .address_mode = .quad, .data_mode = .quad, .alternate_mode = .quad, .alternate_bytes = 1, .dummy_cycles = 4 };
        var hcmd = self.mmio.read(u32, regs.hcmdr.offset);
        hcmd = (hcmd & ~regs.hcmdr.rcmd_mask) | command.opcode;
        self.mmio.write(u32, regs.hcmdr.offset, hcmd);
        self.mmio.write(u32, regs.hrabr.offset, command.alternate);
        self.mmio.write(u32, regs.hrccr.offset, encode(command) & ~regs.ccr1.fmode_mask);
        var control = self.mmio.read(u32, regs.cr.offset);
        control = (control | regs.cr.en_mask | regs.cr.hwife_mask) & ~regs.cr.ahbdis_mask;
        self.mmio.write(u32, regs.cr.offset, control);
    }
    pub fn enableDma(self: *Mpi2, enabled: bool) void {
        var control = self.mmio.read(u32, regs.cr.offset);
        if (enabled) control |= regs.cr.dmae_mask else control &= ~regs.cr.dmae_mask;
        self.mmio.write(u32, regs.cr.offset, control);
    }
    pub fn abort(self: *Mpi2) Error!void {
        self.mmio.write(u32, regs.cr.offset, self.mmio.read(u32, regs.cr.offset) | regs.cr.abort_mask);
        try self.waitNotBusy();
        self.mmio.write(u32, regs.fifocr.offset, regs.fifocr.rxclr_mask | regs.fifocr.txclr_mask);
    }
    fn waitComplete(self: *Mpi2) Error!void {
        var remaining = self.poll_limit;
        while (remaining != 0) : (remaining -= 1) {
            if (self.mmio.read(u32, regs.wdtr.offset) & regs.wdtr.tof_mask != 0) return error.WatchdogTimeout;
            if (self.mmio.read(u32, regs.sr.offset) & regs.sr.tcf_mask != 0) return;
        }
        return error.Timeout;
    }
    fn waitNotBusy(self: *Mpi2) Error!void {
        var remaining = self.poll_limit;
        while (remaining != 0) : (remaining -= 1)
            if (self.mmio.read(u32, regs.sr.offset) & regs.sr.busy_mask == 0) return;
        return error.Timeout;
    }
    fn waitTxSpace(self: *Mpi2) Error!void {
        var remaining = self.poll_limit;
        while (remaining != 0) : (remaining -= 1) if (!self.txFull()) return;
        return error.Timeout;
    }
    fn waitRxData(self: *Mpi2) Error!void {
        var remaining = self.poll_limit;
        while (remaining != 0) : (remaining -= 1)
            if (self.mmio.read(u32, regs.fifocr.offset) & regs.fifocr.rxe_mask == 0) return;
        return error.Timeout;
    }
    fn txFull(self: *Mpi2) bool {
        return self.mmio.read(u32, regs.fifocr.offset) & regs.fifocr.txf_mask != 0;
    }
};

fn encode(c: Command) u32 {
    return (@as(u32, @backingInt(c.direction)) << regs.ccr1.fmode_shift) |
        (@as(u32, @backingInt(c.data_mode)) << regs.ccr1.dmode_shift) |
        (@as(u32, c.dummy_cycles) << regs.ccr1.dcyc_shift) |
        (@as(u32, if (c.alternate_bytes == 0) 0 else c.alternate_bytes - 1) << regs.ccr1.absize_shift) |
        (@as(u32, @backingInt(c.alternate_mode)) << regs.ccr1.abmode_shift) |
        (@as(u32, if (c.address_bytes == 0) 0 else c.address_bytes - 1) << regs.ccr1.adsize_shift) |
        (@as(u32, @backingInt(c.address_mode)) << regs.ccr1.admode_shift) |
        @as(u32, @backingInt(c.instruction_mode));
}
pub fn bind(mmio: Mmio, options: Options) Mpi2 {
    mmio.write(u32, regs.timr.offset, 0xff);
    mmio.write(u32, regs.cir.offset, 0x50005000);
    mmio.write(u32, regs.abr1.offset, 0xff);
    mmio.write(u32, regs.hrabr.offset, 0xff);
    mmio.write(u32, regs.psclr.offset, options.prescaler);
    mmio.write(u32, regs.wdtr.offset, regs.wdtr.en_mask | options.timeout_cycles);
    mmio.write(u32, regs.cr.offset, regs.cr.en_mask);
    return .{ .mmio = mmio, .poll_limit = options.poll_limit };
}
pub fn fromBase(options: Options) Mpi2 {
    return bind(Mmio.direct(pac.mpi2.base_address), options);
}
