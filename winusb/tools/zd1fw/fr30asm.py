# -*- coding: utf-8 -*-
# SPDX-License-Identifier: GPL-2.0-only
#
# 最小限の FR30 アセンブラとエミュレーター。
# CECH-ZD1 J のブリッジ内蔵 CPU (FR30 系、ビッグエンディアン) へ書き込むプログラムを
# 生成・検証するための道具で、本ドライバーが使う命令だけを持つ。
# 命令エンコードは GNU binutils の cpu/fr30.cpu (CGEN 記述) に基づく。

CC = {'ra': 0, 'no': 1, 'eq': 2, 'ne': 3, 'c': 4, 'nc': 5, 'n': 6, 'p': 7,
      'v': 8, 'nv': 9, 'lt': 10, 'ge': 11, 'le': 12, 'gt': 13, 'ls': 14, 'hi': 15}

RP = 1  # 専用レジスタ番号 (MOV Rs,Ri / MOV Ri,Rs)


class Asm:
    def __init__(self):
        self.h = []          # 16 ビットの半語列
        self.labels = {}
        self.fix = []        # (index, kind, label, cc)

    # ---- 基本 ----
    def pc(self):
        return len(self.h)

    def label(self, name):
        assert name not in self.labels, name
        self.labels[name] = len(self.h)

    def raw(self, *hw):
        self.h.extend(x & 0xFFFF for x in hw)

    # ---- ロード/ストア (Rj = アドレス, Ri = データ) ----
    def ld(self, rj, ri):    self.raw(0x0400 | rj << 4 | ri)
    def lduh(self, rj, ri):  self.raw(0x0500 | rj << 4 | ri)
    def ldub(self, rj, ri):  self.raw(0x0600 | rj << 4 | ri)
    def st(self, ri, rj):    self.raw(0x1400 | rj << 4 | ri)
    def sth(self, ri, rj):   self.raw(0x1500 | rj << 4 | ri)
    def stb(self, ri, rj):   self.raw(0x1600 | rj << 4 | ri)
    # R13 + Rj
    def ld13(self, rj, ri):  self.raw(0x0000 | rj << 4 | ri)
    def st13(self, ri, rj):  self.raw(0x1000 | rj << 4 | ri)
    # R14 + disp10 (4 の倍数)
    def ld14(self, disp, ri):
        assert disp % 4 == 0 and 0 <= disp < 1024
        self.raw(0x2000 | (disp // 4) << 4 | ri)
    def st14(self, ri, disp):
        assert disp % 4 == 0 and 0 <= disp < 1024
        self.raw(0x3000 | (disp // 4) << 4 | ri)
    def push(self, ri):      self.raw(0x1700 | ri)
    def pop(self, ri):       self.raw(0x0700 | ri)

    # ---- 即値 ----
    def ldi8(self, i, ri):
        assert 0 <= i < 256
        self.raw(0xC000 | i << 4 | ri)

    def ldi20(self, i, ri):
        assert 0 <= i < (1 << 20)
        self.raw(0x9B00 | ((i >> 16) & 0xF) << 4 | ri, i & 0xFFFF)

    def ldi32(self, i, ri):
        i &= 0xFFFFFFFF
        self.raw(0x9F80 | ri, i >> 16, i & 0xFFFF)

    def ldi(self, i, ri):
        """最短の LDI を選ぶ。"""
        if 0 <= i < 256:
            self.ldi8(i, ri)
        elif 0 <= i < (1 << 20):
            self.ldi20(i, ri)
        else:
            self.ldi32(i, ri)

    # ---- ALU ----
    def mov(self, rj, ri):   self.raw(0x8B00 | rj << 4 | ri)
    def add(self, rj, ri):   self.raw(0xA600 | rj << 4 | ri)
    def sub(self, rj, ri):   self.raw(0xAC00 | rj << 4 | ri)
    def addi(self, u4, ri):
        assert 0 <= u4 < 16
        self.raw(0xA400 | u4 << 4 | ri)
    def subi(self, n, ri):
        """ri -= n (1..16)  (ADD2 #-n)"""
        assert 1 <= n <= 16
        self.raw(0xA500 | ((-n) & 0xF) << 4 | ri)
    def cmp(self, rj, ri):   self.raw(0xAA00 | rj << 4 | ri)   # ri - rj
    def cmpi(self, u4, ri):
        assert 0 <= u4 < 16
        self.raw(0xA800 | u4 << 4 | ri)
    def and_(self, rj, ri):  self.raw(0x8200 | rj << 4 | ri)
    def or_(self, rj, ri):   self.raw(0x9200 | rj << 4 | ri)
    def eor(self, rj, ri):   self.raw(0x9A00 | rj << 4 | ri)
    def lsl(self, n, ri):
        assert 0 <= n < 32
        if n < 16: self.raw(0xB400 | n << 4 | ri)
        else:      self.raw(0xB500 | (n - 16) << 4 | ri)
    def lsr(self, n, ri):
        assert 0 <= n < 32
        if n < 16: self.raw(0xB000 | n << 4 | ri)
        else:      self.raw(0xB100 | (n - 16) << 4 | ri)
    def asr(self, n, ri):
        assert 0 <= n < 32
        if n < 16: self.raw(0xB800 | n << 4 | ri)
        else:      self.raw(0xB900 | (n - 16) << 4 | ri)
    def extub(self, ri):     self.raw(0x9790 | ri)
    def nop(self):           self.raw(0x9FA0)

    # ---- 専用レジスタ ----
    def mov_from_rs(self, rs, ri): self.raw(0xB700 | rs << 4 | ri)
    def mov_to_rs(self, ri, rs):   self.raw(0xB300 | rs << 4 | ri)

    # ---- 分岐 (遅延スロットなし) ----
    def b(self, cc, label):
        self.fix.append((len(self.h), 'b', label))
        self.raw(0xE000 | CC[cc] << 8)

    def bra(self, label): self.b('ra', label)
    def beq(self, label): self.b('eq', label)
    def bne(self, label): self.b('ne', label)
    def bc(self, label):  self.b('c', label)
    def bnc(self, label): self.b('nc', label)
    def bls(self, label): self.b('ls', label)
    def bhi(self, label): self.b('hi', label)
    def blt(self, label): self.b('lt', label)
    def bge(self, label): self.b('ge', label)

    def call(self, label):
        self.fix.append((len(self.h), 'call', label))
        self.raw(0xD000)

    def ret(self):        self.raw(0x9720)
    def jmp(self, ri):    self.raw(0x9700 | ri)

    # ---- 解決 ----
    def link(self):
        for idx, kind, name in self.fix:
            tgt = self.labels[name]
            d = tgt - (idx + 1)       # 次の命令からの半語数
            if kind == 'b':
                assert -128 <= d <= 127, (name, d)
                self.h[idx] = (self.h[idx] & 0xFF00) | (d & 0xFF)
            else:
                assert -1024 <= d <= 1023, (name, d)
                self.h[idx] = 0xD000 | (d & 0x7FF)
        self.fix = []
        return self

    def words(self):
        """32 ビット (ビッグエンディアン) ワード列。奇数半語なら NOP で埋める。"""
        h = list(self.h)
        if len(h) % 2:
            h.append(0x9FA0)
        return [h[i] << 16 | h[i + 1] for i in range(0, len(h), 2)]


class Cpu:
    """テスト用の FR30 サブセット・エミュレーター。メモリは dict (ワード単位、ビッグエンディアン)。

    mmio: {アドレス(ワード境界): (read_fn or None, write_fn or None)}
    """

    def __init__(self, code_base=0, mmio=None):
        self.r = [0] * 16
        self.rp = 0
        self.z = self.n = self.c = self.v = 0
        self.mem = {}
        self.mmio = mmio or {}
        self.pc = code_base
        self.steps = 0

    # ---- メモリ ----
    def load_words(self, base, words):
        for i, w in enumerate(words):
            self.mem[base + 4 * i] = w

    def r32(self, a):
        a &= 0xFFFFFFFF
        if a in self.mmio and self.mmio[a][0]:
            return self.mmio[a][0]() & 0xFFFFFFFF
        return self.mem.get(a, 0)

    def w32(self, a, v):
        a &= 0xFFFFFFFF
        v &= 0xFFFFFFFF
        if a in self.mmio and self.mmio[a][1]:
            self.mmio[a][1](v)
        self.mem[a] = v

    def r16(self, a):
        w = self.r32(a & ~3)
        return (w >> 16) & 0xFFFF if (a & 2) == 0 else w & 0xFFFF

    def r8(self, a):
        w = self.r32(a & ~3)
        return (w >> (24 - 8 * (a & 3))) & 0xFF

    def w16(self, a, v):
        base = a & ~3
        w = self.mem.get(base, 0)
        if (a & 2) == 0:
            w = (w & 0x0000FFFF) | (v & 0xFFFF) << 16
        else:
            w = (w & 0xFFFF0000) | (v & 0xFFFF)
        self.w32(base, w)

    def w8(self, a, v):
        base = a & ~3
        sh = 24 - 8 * (a & 3)
        w = self.mem.get(base, 0)
        w = (w & ~(0xFF << sh)) | (v & 0xFF) << sh
        self.w32(base, w)

    def fetch(self):
        v = self.r16(self.pc)
        self.pc += 2
        return v

    # ---- フラグ ----
    def _nz(self, v):
        v &= 0xFFFFFFFF
        self.n = v >> 31
        self.z = int(v == 0)
        return v

    def _addc(self, a, b):
        res = a + b
        self.c = int(res > 0xFFFFFFFF)
        r = res & 0xFFFFFFFF
        self.v = int(((a ^ r) & (b ^ r)) >> 31 & 1)
        return self._nz(r)

    def _sub(self, a, b):     # a - b
        res = a - b
        self.c = int(res < 0)
        r = res & 0xFFFFFFFF
        self.v = int(((a ^ b) & (a ^ r)) >> 31 & 1)
        return self._nz(r)

    def cond(self, cc):
        n, z, v, c = self.n, self.z, self.v, self.c
        return [1, 0, z, 1 - z, c, 1 - c, n, 1 - n, v, 1 - v,
                int(n != v), int(n == v), int(z or (n != v)), int(not z and n == v),
                int(c or z), int(not c and not z)][cc]

    def step(self):
        self.steps += 1
        op = self.fetch()
        r = self.r
        n0, n1, n2, n3 = op >> 12, (op >> 8) & 15, (op >> 4) & 15, op & 15
        hi = op >> 8
        if hi == 0x04: r[n3] = self.r32(r[n2])
        elif hi == 0x05: r[n3] = self.r16(r[n2])
        elif hi == 0x06: r[n3] = self.r8(r[n2])
        elif hi == 0x00: r[n3] = self.r32(r[13] + r[n2])
        elif hi == 0x14: self.w32(r[n2], r[n3])
        elif hi == 0x15: self.w16(r[n2], r[n3])
        elif hi == 0x16: self.w8(r[n2], r[n3])
        elif hi == 0x10: self.w32(r[13] + r[n2], r[n3])
        elif n0 == 2: r[n3] = self.r32(r[14] + ((op >> 4) & 0xFF) * 4)
        elif n0 == 3: self.w32(r[14] + ((op >> 4) & 0xFF) * 4, r[n3])
        elif hi == 0x17 and n2 == 0: r[15] -= 4; self.w32(r[15], r[n3])
        elif hi == 0x07 and n2 == 0: r[n3] = self.r32(r[15]); r[15] += 4
        elif n0 == 0xC: r[n3] = (op >> 4) & 0xFF
        elif hi == 0x9B:
            r[n3] = n2 << 16 | self.fetch()
        elif (op & 0xFFF0) == 0x9F80:
            a = self.fetch(); b = self.fetch(); r[n3] = a << 16 | b
        elif hi == 0x8B: r[n3] = r[n2]
        elif hi == 0xA6: r[n3] = self._addc(r[n3], r[n2])
        elif hi == 0xAC: r[n3] = self._sub(r[n3], r[n2])
        elif hi == 0xA4: r[n3] = self._addc(r[n3], n2)
        elif hi == 0xA5: r[n3] = self._addc(r[n3], (n2 | 0xFFFFFFF0) & 0xFFFFFFFF)
        elif hi == 0xAA: self._sub(r[n3], r[n2])
        elif hi == 0xA8: self._sub(r[n3], n2)
        elif hi == 0x82: r[n3] = self._nz(r[n3] & r[n2])
        elif hi == 0x92: r[n3] = self._nz(r[n3] | r[n2])
        elif hi == 0x9A: r[n3] = self._nz(r[n3] ^ r[n2])
        elif hi in (0xB4, 0xB5, 0xB0, 0xB1, 0xB8, 0xB9):
            sh = n2 + (16 if hi in (0xB5, 0xB1, 0xB9) else 0)
            x = r[n3]
            if hi in (0xB4, 0xB5): y = (x << sh) & 0xFFFFFFFF
            elif hi in (0xB0, 0xB1): y = x >> sh
            else: y = (((x ^ 0x80000000) - 0x80000000) >> sh) & 0xFFFFFFFF
            r[n3] = self._nz(y)
        elif hi == 0x97 and n2 == 9: r[n3] &= 0xFF
        elif hi == 0xB7 and n2 == RP: r[n3] = self.rp
        elif hi == 0xB3 and n2 == RP: self.rp = r[n3]
        elif n0 == 0xE:
            if self.cond(n1):
                d = op & 0xFF
                if d & 0x80: d -= 256
                self.pc += d * 2
        elif n0 == 0xD:
            if op & 0x0800:
                raise RuntimeError('call:d unsupported')
            d = op & 0x7FF
            if d & 0x400: d -= 0x800
            self.rp = self.pc
            self.pc += d * 2
        elif op == 0x9720: self.pc = self.rp
        elif hi == 0x97 and n2 == 0: self.pc = r[n3]
        elif op == 0x9FA0: pass
        else:
            raise RuntimeError('unsupported opcode %04X at %X' % (op, self.pc - 2))

    def run(self, max_steps=1000000, until=None):
        while self.steps < max_steps:
            if until and until(self):
                return True
            self.step()
        return False
