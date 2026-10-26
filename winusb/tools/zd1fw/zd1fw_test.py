# -*- coding: utf-8 -*-
# SPDX-License-Identifier: GPL-2.0-only
#
# zd1fw_gen.py のプログラムを、模擬ペリフェラル上で実行して手順を確かめる。
#   python zd1fw_test.py

import unittest
import zd1fw_gen as g
from fr30asm import Cpu

I2C = 0xFD8FE0


class FakeI2c:
    """Status/Control/Data を持つ I2C マスターと、スレーブのレジスタを模擬する。

    スレーブ 0x12 は 2 バイトのレジスタ番号、0xC0 は 1 バイトのレジスタ番号 (どちらも値は 2 バイト)。
    """

    REGLEN = {0x12: 2, 0xC0: 1}

    def __init__(self):
        self.regs = {}
        self.log = []
        self.ctrl = 0x40
        self.data = 0
        self.done = False
        self.nack = False
        self.expect_addr = True
        self.slave = None
        self.wb = []
        self.reading = False
        self.rx = []

    def status_r(self):
        return 0x10 if self.nack else 0

    def ctrl_r(self):
        return 1 if self.done else self.ctrl

    def start_read(self):
        n = self.REGLEN[self.slave]
        reg = int.from_bytes(bytes(self.wb[:n]), 'big')
        self.rx = list(self.regs.get((self.slave, reg), 0).to_bytes(2, 'big'))
        self.reading = True

    def ctrl_w(self, v):
        self.ctrl = v
        self.done = False
        if v == 0x40:
            n = self.REGLEN.get(self.slave, 1)
            if not self.reading and len(self.wb) == n + 2:
                reg = int.from_bytes(bytes(self.wb[:n]), 'big')
                self.regs[(self.slave, reg)] = int.from_bytes(bytes(self.wb[n:]), 'big')
            self.log.append(('stop',))
            self.expect_addr = True
            self.wb = []
            self.reading = False
            self.nack = False
            return
        self.done = True
        self.nack = False
        if v == 0x70 or (self.expect_addr and v == 0x50):
            addr = self.data
            self.expect_addr = False
            self.slave = addr & 0xFE
            self.nack = self.slave not in self.REGLEN
            self.log.append(('addr', addr))
            if (addr & 1) and not self.nack:
                self.start_read()
        elif self.reading:
            self.log.append(('rd', v))
        else:
            self.wb.append(self.data)
            self.log.append(('data', self.data))

    def data_r(self):
        return self.rx.pop(0) if self.rx else 0xFF

    def data_w(self, v):
        self.data = v & 0xFF

    def mmio(self):
        return {
            I2C + 0x00: (self.status_r, None),
            I2C + 0x04: (self.ctrl_r, self.ctrl_w),
            I2C + 0x10: (self.data_r, self.data_w),
        }


def mb(off):
    return g.VB + off


def run_mailbox(cpu, seq, addr, wdata, rlen):
    for i, b in enumerate(wdata):
        cpu.w8(mb(g.V_MWD) + i, b)
    cpu.w32(mb(g.V_MREQ), seq << 24 | addr << 16 | len(wdata) << 8 | rlen)
    ok = cpu.run(cpu.steps + 3000000, until=lambda c: (c.mem.get(mb(g.V_MANS), 0) >> 24) == seq)
    assert ok, 'core1 did not answer'
    return cpu.mem[mb(g.V_MANS)] & 0xFF


class Core1Test(unittest.TestCase):
    def setup(self):
        self.i2c = FakeI2c()
        self.ep3 = []
        self.kicks = 0
        mmio = self.i2c.mmio()
        mmio[g.EP3_FIFO] = (None, lambda v: self.ep3.append(v))

        def kick(v):
            self.kicks += 1
        mmio[g.EP3_KICK] = (None, kick)
        cpu = Cpu(code_base=0xA0000, mmio=mmio)
        cpu.load_words(0xA0000, g.build_core1().words())
        cpu.w32(mb(g.V_SLAVE), 0x12C00000)
        cpu.w32(mb(g.V_SLOT), g.CMD_WINDOW)
        self.seed = 0x1234ABCD
        cpu.w32(mb(g.V_EXPECT), self.seed)
        return cpu

    # ---- mailbox ----
    def test_mailbox_write_read(self):
        cpu = self.setup()
        self.i2c.regs[(0x12, 0x0507)] = 0x1234
        err = run_mailbox(cpu, 1, 0x12, [0x05, 0x07], 2)
        self.assertEqual(err, 0)
        self.assertEqual(cpu.r8(mb(g.V_MRD)), 0x12)
        self.assertEqual(cpu.r8(mb(g.V_MRD) + 1), 0x34)
        self.assertGreater(cpu.mem[mb(g.V_LIVE)], 0)

    def test_mailbox_write_only(self):
        cpu = self.setup()
        self.assertEqual(run_mailbox(cpu, 7, 0x12, [5, 7, 0, 3], 0), 0)
        self.assertEqual(self.i2c.regs[(0x12, 0x0507)], 3)

    def test_mailbox_nack(self):
        cpu = self.setup()
        self.assertEqual(run_mailbox(cpu, 3, 0x40, [1], 0), 2)
        self.assertEqual(self.i2c.log[-1], ('stop',))

    # ---- コマンドパケット ----
    def put_packet(self, cpu, slot, seq, words, reply):
        base = g.CMD_WINDOW + slot * 512
        cpu.w32(base + 4, (0x80000000 if reply else 0) | len(words))
        for i, w in enumerate(words):
            cpu.w32(base + 8 + 4 * i, w)
        cpu.w32(base + 508, seq)
        cpu.w32(base, seq)

    def run_until_reply(self, cpu, kicks):
        ok = cpu.run(cpu.steps + 6000000, until=lambda c: self.kicks >= kicks)
        self.assertTrue(ok, 'no reply')
        cpu.run(cpu.steps + 3000)       # 送出の後始末 (最初の失敗の消去、次のスロットへの前進) まで進める

    def test_cmd_write_read(self):
        cpu = self.setup()
        words = [
            (0x0507 << 16) | 0x0003,                  # write 2 バイト番号
            0x40000000 | (0x0507 << 16),              # read
            0x20000000 | (0x24 << 16) | 0x1234,       # write 1 バイト番号 (スレーブ B)
            0x60000000 | (0x24 << 16),                # read (B)
        ]
        self.put_packet(cpu, 0, self.seed, words, reply=True)
        self.run_until_reply(cpu, 1)
        self.assertEqual(self.i2c.regs[(0x12, 0x0507)], 3)
        self.assertEqual(self.i2c.regs[(0xC0, 0x24)], 0x1234)
        self.assertEqual(self.ep3, [self.seed, 2, 3, 0x1234])
        self.assertEqual(cpu.mem[mb(g.V_EXPECT)], self.seed + 1)
        self.assertEqual(cpu.mem[mb(g.V_SLOT)], g.CMD_WINDOW + 512)

    def test_cmd_modify(self):
        cpu = self.setup()
        self.i2c.regs[(0x12, 0x0123)] = 0xABCD
        words = [
            0x10000000 | (0x0123 << 16) | 0x0034,     # modify: (v & mask) | 0x0034
            0x18000000 | 0xFF00,
            0x40000000 | (0x0123 << 16),
        ]
        self.put_packet(cpu, 0, self.seed, words, reply=True)
        self.run_until_reply(cpu, 1)
        self.assertEqual(self.i2c.regs[(0x12, 0x0123)], 0xAB34)
        self.assertEqual(self.ep3, [self.seed, 1, 0xAB34])

    def test_cmd_no_reply_accumulates_and_errors(self):
        cpu = self.setup()
        # 1 つ目は答えなし。2 つ目はメモリ・アクセス (未対応) で失敗コード 5
        self.put_packet(cpu, 0, self.seed, [(0x0001 << 16) | 0x0002], reply=False)
        self.put_packet(cpu, 1, self.seed + 1, [0x08000000, 0x40000000 | (0x0001 << 16)], reply=True)
        self.run_until_reply(cpu, 1)
        self.assertEqual(self.i2c.regs[(0x12, 0x0001)], 2)
        self.assertEqual(self.ep3[0], self.seed + 1)
        self.assertEqual(self.ep3[1] >> 16, 5)
        self.assertEqual(cpu.mem[mb(g.V_FERR)], 0)       # 答えを返したら 0 に戻る

    def test_cmd_waits_for_tail(self):
        cpu = self.setup()
        base = g.CMD_WINDOW
        cpu.w32(base, self.seed)                         # 先頭だけ揃っても実行されない
        cpu.w32(base + 4, 0x80000000)
        cpu.run(cpu.steps + 20000)
        self.assertEqual(self.kicks, 0)
        cpu.w32(base + 508, self.seed)
        self.run_until_reply(cpu, 1)

    def test_cmd_wraps_after_16_slots(self):
        cpu = self.setup()
        for i in range(16):
            self.put_packet(cpu, i, self.seed + i, [(0x0001 << 16) | i], reply=(i == 15))
        self.run_until_reply(cpu, 1)
        self.assertEqual(cpu.mem[mb(g.V_SLOT)], g.CMD_WINDOW)
        self.assertEqual(cpu.mem[mb(g.V_EXPECT)], self.seed + 16)
        self.assertEqual(self.i2c.regs[(0x12, 0x0001)], 15)

    def test_cmd_answer_limit_is_14(self):
        cpu = self.setup()
        self.put_packet(cpu, 0, self.seed, [0x40000000 | (0x0001 << 16)] * 20, reply=True)
        self.run_until_reply(cpu, 1)
        self.assertEqual(self.ep3[1] & 0xFF, 14)
        self.assertEqual(len(self.ep3), 2 + 14)

    def test_mailbox_still_works_between_packets(self):
        cpu = self.setup()
        self.put_packet(cpu, 0, self.seed, [(0x0001 << 16) | 9], reply=True)
        self.run_until_reply(cpu, 1)
        self.assertEqual(run_mailbox(cpu, 1, 0x12, [0, 1], 2), 0)
        self.assertEqual(cpu.r8(mb(g.V_MRD) + 1), 9)


class AddrTest(unittest.TestCase):
    def run_path(self, write, h, addr):
        cpu = Cpu(code_base=0xE1000)
        cpu.load_words(0xE1000, g.build_addr(write).words())
        cpu.w8(g.STATE_H, h)
        cpu.r[4] = addr
        ret = g.RET_WRITE if write else g.RET_READ
        cpu.run(100000, until=lambda c: c.pc == ret)
        self.assertEqual(cpu.pc, ret)
        return cpu

    def test_widen_read(self):
        cpu = self.run_path(False, 0x01, 0x0A3F7C)
        self.assertEqual(cpu.r[4], 0x010A3F7C)
        self.assertEqual(cpu.mem[g.STATE_ADDR], 0x010A3F7C)
        self.assertEqual(cpu.r8(g.STATE_H), 1)

    def test_widen_write_clears_h(self):
        cpu = self.run_path(True, 0x17, 0x00A3F00)
        self.assertEqual(cpu.mem[g.STATE_ADDR], 0x170A3F00)
        self.assertEqual(cpu.r8(g.STATE_H), 0)

    def test_alias(self):
        cpu = self.run_path(False, 0, 0x90018)
        self.assertEqual(cpu.mem[g.STATE_ADDR], 0xF10018)

    def test_passthrough(self):
        cpu = self.run_path(False, 0, 0x00E9AC0)
        self.assertEqual(cpu.r[4], 0x00E9AC0)
        self.assertNotIn(g.STATE_ADDR, cpu.mem)
        cpu = self.run_path(True, 0, 0xA0000)      # 上限は含まない
        self.assertNotIn(g.STATE_ADDR, cpu.mem)

    def test_ring(self):
        cpu = Cpu(code_base=0xE1000)
        cpu.load_words(0xE1000, g.build_addr(False).words())
        cpu.w8(g.STATE_H, 0xFE)
        slot_byte = 0x32                    # 周回 1、short、スロット 2
        cpu.r[4] = slot_byte << 16 | 0x123

        def ring_w(v):
            cpu.mem[g.RING_ANSWER] = (slot_byte << 24) | 0x1234
        cpu.mmio[g.RING_BASE + 2 * 4] = (None, ring_w)
        cpu.run(100000, until=lambda c: c.pc == g.RET_READ)
        self.assertEqual(cpu.pc, g.RET_READ)
        self.assertEqual(cpu.mem[g.RING_BASE + 8], 0x40000000 | 0x123 << 16 | 0x80000000 | 0x20000000)
        self.assertEqual(cpu.mem[g.STATE_ADDR], g.RING_ANSWER)


class Core0Test(unittest.TestCase):
    def test_assembles_and_runs_init(self):
        words = g.build_core0().words()
        self.assertLess(len(words) * 4, 0x3000)
        cpu = Cpu(code_base=0xA0000)
        # DMA 停止待ちを抜けるため 0x1068 は 0 を返す。TS エンジンの書き込み位置は固定
        cpu.mmio[0x1068] = (lambda: 0, None)
        cpu.mmio[0xD02038] = (lambda: 0x1000, None)
        cpu.load_words(0xA0000, words)
        cpu.run(20000)
        self.assertGreater(cpu.mem.get(0xA3F00, 0), 5)      # ループ回数が増える
        self.assertEqual(cpu.mem[0x1018], 0x00AE4000)
        self.assertEqual(cpu.mem[0x1048], 0x1000)


if __name__ == '__main__':
    unittest.main()
