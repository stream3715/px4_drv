# -*- coding: utf-8 -*-
# SPDX-License-Identifier: GPL-2.0-only
#
# CECH-ZD1 J のブリッジ (MB86H47A) へホストが書き込む FR30 プログラムを生成し、
# C++ ヘッダー zd1_firmware.h を出力する。
#   python zd1fw_gen.py <出力ヘッダー>
#
# プログラムの入出力は spec の 02 章 (要求ハンドラのパッチ、codec core 1 / core 0)
# に従う。core 1 はコマンドパケット (バルク転送) と mailbox の I2C 転送を持つ。

import sys
from fr30asm import Asm

R0, R1, R2, R3, R4, R5, R6, R7, R8, R9, R10, R11, R12, R13, R14, R15 = range(16)

STATE_H = 0xC4640       # 要求状態 +0x40 (wValue 下位バイト)
STATE_ADDR = 0xC4644    # 要求状態 +0x44 (転送が使うアドレス)
RING_BASE = 0x17A3E00   # core 1 データ RAM のリング (ホスト座標)
RING_ANSWER = 0x17A3E40
RET_READ = 0xE9AE6
RET_WRITE = 0xE9E2E


def build_addr(write):
    a = Asm()
    a.ldi20(STATE_H, R0)
    a.ldub(R0, R1)
    a.cmpi(0, R1)
    a.beq('h0')
    if not write:
        a.ldi8(0xFE, R12)
        a.cmp(R12, R1)
        a.beq('ring')
    a.lsl(24, R1)
    a.or_(R1, R4)
    if write:
        a.ldi8(0, R12)
        a.stb(R12, R0)          # 以後の転送で下位バイトを再適用しない
    a.addi(4, R0)
    a.st(R4, R0)
    a.bra('ret')
    a.label('h0')
    a.ldi20(0x80000, R1)
    a.cmp(R1, R4)
    a.bc('ret')                 # R4 < 0x80000
    a.ldi20(0xA0000, R1)
    a.cmp(R1, R4)
    a.bnc('ret')                # R4 >= 0xA0000
    a.ldi20(0x3FFFF, R1)
    a.and_(R1, R4)
    a.ldi32(0xF00000, R1)
    a.add(R1, R4)
    a.ldi20(STATE_ADDR, R0)
    a.st(R4, R0)
    a.label('ret')
    a.ldi20(RET_WRITE if write else RET_READ, R12)
    a.jmp(R12)
    if not write:
        # リング読み出し (H == 0xFE)。作業レジスタは R0, R1, R4, R12 のみ
        a.label('ring')
        a.mov(R4, R0)
        a.lsr(16, R0)
        a.ldi8(0xFF, R1)
        a.and_(R1, R0)              # R0 = slot_byte
        a.ldi20(0x7FF, R1)
        a.and_(R1, R4)
        a.lsl(16, R4)
        a.ldi32(0x40000000, R1)     # ringRead
        a.or_(R1, R4)
        a.mov(R0, R1)
        a.ldi8(0x10, R12)
        a.and_(R12, R1)
        a.beq('nolap')
        a.ldi32(0x80000000, R12)    # ringLap
        a.or_(R12, R4)
        a.label('nolap')
        a.mov(R0, R1)
        a.ldi8(0x20, R12)
        a.and_(R12, R1)
        a.beq('noshort')
        a.ldi32(0x20000000, R12)    # ringShort
        a.or_(R12, R4)
        a.label('noshort')
        a.mov(R0, R1)
        a.ldi8(0x0F, R12)
        a.and_(R12, R1)
        a.lsl(2, R1)
        a.ldi32(RING_BASE, R12)
        a.add(R12, R1)
        a.st(R4, R1)                # リングのスロットへ置く
        a.ldi8(0x1F, R1)
        a.and_(R1, R0)              # R0 = slot_byte & 0x1F
        a.ldi20(0x40000, R12)
        a.ldi32(RING_ANSWER, R4)
        a.label('poll')
        a.ld(R4, R1)
        a.lsl(3, R1)
        a.lsr(27, R1)               # (answer >> 24) & 0x1F
        a.cmp(R0, R1)
        a.beq('done')
        a.subi(1, R12)
        a.bne('poll')
        a.label('done')
        a.ldi20(STATE_ADDR, R1)
        a.st(R4, R1)                # 応答はリング応答 (+ 失敗記録) を読ませる
        a.bra('ret')
    return a.link()


# ---- core 1 (I2C マスター制御: コマンドパケットと mailbox) ----
# コア内アドレス: コード 0xA0000、スタック 0xA37FC、変数 0xA3C00。
VB = 0xA3C00
I2C_BASE = 0xFD8FE0
CMD_WINDOW = 0x82100000          # コマンドチャネルの受信スロット (512 バイト × 16)
EP3_FIFO = 0xFDBD8C
EP3_KICK = 0xFDBD1C

# 変数 (VB からのオフセット)
V_EXPECT = 0x00      # 次に実行するパケットのシーケンス
V_SLOT = 0x04        # 次に実行する受信スロットの先頭
V_REPLY = 0x08
V_FERR = 0x0C        # 前回の答え以降に起きた最初の失敗コード
V_NANS = 0x10
V_W = 0x14
V_MASK = 0x18
V_ANS = 0x20         # 答えのワード (最大 14)
V_VAL = 0x58
V_SLAVE = 0x60       # A (2 バイト番号) << 24 | B (1 バイト番号) << 16
V_LIVE = 0x7C
V_MANS = 0x80        # mailbox の答え
V_MRD = 0x84         # mailbox の読み出しデータ (32 バイト)
V_MREQ = 0xC0        # mailbox の要求
V_MWD = 0xC4         # mailbox の書くデータ (32 バイト)
V_WBUF = 0xE8        # レジスタ・アクセスの I2C 書き込みバッファ


def build_core1():
    a = Asm()

    def ldv(off, rd):
        a.ldi8(off, R0)
        a.ld13(R0, rd)

    def stv(rs, off):
        assert rs != R0
        a.ldi8(off, R0)
        a.st13(rs, R0)

    # R15 = スタック、R14 = I2C 基底、R13 = 変数基底、R12 = 最後に処理した mailbox seq
    a.ldi20(0xA37FC, R15)
    a.ldi32(I2C_BASE, R14)
    a.ldi20(VB, R13)
    a.ldi8(0, R12)

    a.label('main')
    ldv(V_LIVE, R1)
    a.addi(1, R1)
    stv(R1, V_LIVE)

    # ---- コマンドパケット (先頭と末尾のワードがどちらも期待シーケンスなら揃っている) ----
    ldv(V_SLOT, R1)
    a.ld(R1, R2)
    ldv(V_EXPECT, R3)
    a.cmp(R3, R2)
    a.bne('mbox')
    a.ldi20(508, R4)
    a.add(R1, R4)
    a.ld(R4, R2)
    a.cmp(R3, R2)
    a.bne('mbox')
    a.call('exec_cmd')
    a.bra('main')

    # ---- mailbox (任意の I2C 転送。コマンドチャネルが使えない間の経路) ----
    a.label('mbox')
    ldv(V_MREQ, R1)
    a.mov(R1, R2)
    a.lsr(24, R2)
    a.cmp(R12, R2)
    a.beq('main')
    a.mov(R2, R12)
    a.mov(R1, R7)
    a.lsr(16, R7)
    a.ldi8(0xFF, R5)
    a.and_(R5, R7)
    a.mov(R1, R8)
    a.lsr(8, R8)
    a.and_(R5, R8)
    a.mov(R1, R9)
    a.and_(R5, R9)
    a.ldi20(VB + V_MWD, R10)
    a.call('transfer')
    a.mov(R12, R1)
    a.lsl(24, R1)
    a.or_(R0, R1)
    stv(R1, V_MANS)
    a.bra('main')

    # ---- バス解放待ち ----
    a.label('wait_idle')
    a.ldi20(0x40000, R4)
    a.ldi8(0x80, R5)
    a.label('wi_l')
    a.ld14(0, R0)
    a.and_(R5, R0)
    a.beq('wi_ok')
    a.subi(1, R4)
    a.bne('wi_l')
    a.ldi8(4, R0)
    a.ret()
    a.label('wi_ok')
    a.ldi8(0, R0)
    a.ret()

    # ---- バイト送出 (R1 = データ, R2 = Control 値, R3 = 無応答コード) ----
    a.label('send_byte')
    a.st14(R1, 0x10)
    a.label('ctl_byte')
    a.st14(R2, 0x04)
    a.ldi20(0x40000, R4)
    a.label('sb_poll')
    a.ld14(0x04, R0)
    a.ldi8(0x80, R5)
    a.mov(R0, R6)
    a.and_(R5, R6)
    a.bne('sb_bus')
    a.ldi8(1, R5)
    a.and_(R5, R0)
    a.bne('sb_done')
    a.subi(1, R4)
    a.bne('sb_poll')
    a.ldi8(4, R0)
    a.ret()
    a.label('sb_bus')
    a.ldi8(1, R0)
    a.ret()
    a.label('sb_done')
    a.ld14(0, R0)
    a.ldi8(0x10, R5)
    a.and_(R5, R0)
    a.beq('sb_ok')
    a.mov(R3, R0)
    a.ret()
    a.label('sb_ok')
    a.ldi8(0, R0)
    a.ret()

    # ---- 転送 (R7 = スレーブ 8 ビット, R8 = 書く長さ, R9 = 読む長さ, R10 = 書くデータ) ----
    a.label('transfer')
    a.mov_from_rs(1, R11)
    a.cmpi(0, R8)
    a.beq('tr_nowrite')
    a.call('wait_idle')
    a.cmpi(0, R0)
    a.bne('tr_fail')
    a.mov(R7, R1)
    a.ldi8(0xFE, R5)
    a.and_(R5, R1)
    a.ldi8(0x50, R2)
    a.ldi8(2, R3)
    a.call('send_byte')
    a.cmpi(0, R0)
    a.bne('tr_fail')
    a.label('tr_wloop')
    a.cmpi(0, R8)
    a.beq('tr_wdone')
    a.ldub(R10, R1)
    a.ldi8(0x50, R2)
    a.ldi8(3, R3)
    a.call('send_byte')
    a.cmpi(0, R0)
    a.bne('tr_fail')
    a.addi(1, R10)
    a.subi(1, R8)
    a.bra('tr_wloop')
    a.label('tr_wdone')
    a.cmpi(0, R9)
    a.bne('tr_rstart')
    a.bra('tr_stop')
    a.label('tr_rstart')
    a.mov(R7, R1)
    a.ldi8(1, R5)
    a.or_(R5, R1)
    a.ldi8(0x70, R2)
    a.ldi8(2, R3)
    a.call('send_byte')
    a.cmpi(0, R0)
    a.bne('tr_fail')
    a.bra('tr_read')
    a.label('tr_nowrite')
    a.cmpi(0, R9)
    a.beq('tr_ok')
    a.call('wait_idle')
    a.cmpi(0, R0)
    a.bne('tr_fail')
    a.mov(R7, R1)
    a.ldi8(1, R5)
    a.or_(R5, R1)
    a.ldi8(0x50, R2)
    a.ldi8(2, R3)
    a.call('send_byte')
    a.cmpi(0, R0)
    a.bne('tr_fail')
    a.label('tr_read')
    a.ldi20(VB + V_MRD, R10)
    a.label('tr_rloop')
    a.ldi8(0x58, R2)
    a.cmpi(1, R9)
    a.bne('tr_rctl')
    a.ldi8(0x50, R2)
    a.label('tr_rctl')
    a.ldi8(0, R3)
    a.call('ctl_byte')
    a.cmpi(0, R0)
    a.bne('tr_fail')
    a.ld14(0x10, R1)
    a.stb(R1, R10)
    a.addi(1, R10)
    a.subi(1, R9)
    a.bne('tr_rloop')
    a.label('tr_stop')
    a.ldi8(0x40, R1)
    a.st14(R1, 0x04)
    a.call('wait_idle')
    a.mov_to_rs(R11, 1)
    a.ret()
    a.label('tr_ok')
    a.ldi8(0, R0)
    a.mov_to_rs(R11, 1)
    a.ret()
    a.label('tr_fail')
    a.ldi8(0x40, R1)
    a.st14(R1, 0x04)
    a.mov_to_rs(R11, 1)
    a.ret()

    # ---- 1 つのレジスタ・アクセス (V_W = アクセス・ワード, V_MASK = 修正の mask) ----
    # 結果: R0 = 失敗コード、読み出した値は V_VAL。R1, R2 は保存する。
    a.label('do_access')
    a.mov_from_rs(1, R0)
    a.push(R0)
    a.push(R1)
    a.push(R2)
    ldv(V_W, R5)
    a.mov(R5, R1)
    a.lsr(16, R1)
    a.ldi20(0x7FF, R0)
    a.and_(R0, R1)                      # R1 = レジスタ番号
    a.ldi32(0x20000000, R0)
    a.mov(R5, R2)
    a.and_(R0, R2)                      # short か
    ldv(V_SLAVE, R7)
    a.ldi20(VB + V_WBUF, R10)
    a.cmpi(0, R2)
    a.bne('da_short')
    a.lsr(24, R7)                       # A
    a.mov(R1, R0)
    a.lsr(8, R0)
    a.stb(R0, R10)
    a.mov(R10, R3)
    a.addi(1, R3)
    a.stb(R1, R3)
    a.ldi8(2, R8)
    a.bra('da_have')
    a.label('da_short')
    a.lsr(16, R7)
    a.ldi8(0xFF, R0)
    a.and_(R0, R7)                      # B
    a.stb(R1, R10)
    a.ldi8(1, R8)
    a.label('da_have')
    a.ldi32(0x50000000, R0)             # read | modify
    a.mov(R5, R2)
    a.and_(R0, R2)
    a.beq('da_write')
    a.push(R7)
    a.push(R8)
    a.ldi8(2, R9)
    a.call('transfer')                  # [reg] を書いて 2 バイト読む
    a.pop(R8)
    a.pop(R7)
    a.cmpi(0, R0)
    a.bne('da_ret')
    a.ldi20(VB + V_MRD, R1)
    a.ldub(R1, R2)
    a.lsl(8, R2)
    a.addi(1, R1)
    a.ldub(R1, R3)
    a.or_(R3, R2)                       # R2 = 読んだ値
    stv(R2, V_VAL)
    ldv(V_W, R5)
    a.ldi32(0x10000000, R0)
    a.and_(R0, R5)
    a.beq('da_ok')
    ldv(V_MASK, R6)
    a.and_(R6, R2)
    ldv(V_W, R5)
    a.ldi20(0xFFFF, R0)
    a.and_(R0, R5)
    a.or_(R5, R2)                       # (読んだ値 & mask) | value
    a.bra('da_dowrite')
    a.label('da_write')
    ldv(V_W, R2)
    a.ldi20(0xFFFF, R0)
    a.and_(R0, R2)
    a.label('da_dowrite')
    a.ldi20(VB + V_WBUF, R10)
    a.mov(R10, R3)
    a.add(R8, R3)
    a.mov(R2, R0)
    a.lsr(8, R0)
    a.stb(R0, R3)
    a.addi(1, R3)
    a.stb(R2, R3)
    a.addi(2, R8)
    a.ldi8(0, R9)
    a.call('transfer')
    a.bra('da_ret')
    a.label('da_ok')
    a.ldi8(0, R0)
    a.label('da_ret')
    a.pop(R2)
    a.pop(R1)
    a.pop(R4)
    a.mov_to_rs(R4, 1)
    a.ret()

    # ---- コマンドパケットの実行 (R1 = スロット先頭, R3 = シーケンス) ----
    a.label('exec_cmd')
    a.mov_from_rs(1, R0)
    a.push(R0)
    a.push(R3)
    a.push(R1)
    a.mov(R1, R0)
    a.addi(4, R0)
    a.ld(R0, R2)                        # ヘッダー: bit31 = 答えを要求, bit7-0 = ワード数
    a.mov(R2, R6)
    a.lsr(31, R6)
    stv(R6, V_REPLY)
    a.ldi8(0xFF, R0)
    a.and_(R0, R2)                      # R2 = 残りのワード数
    a.addi(8, R1)                       # R1 = 次のアクセス・ワード
    a.label('ec_loop')
    a.cmpi(0, R2)
    a.beq('ec_done')
    a.ld(R1, R5)
    a.addi(4, R1)
    a.subi(1, R2)
    a.ldi32(0x08000000, R6)
    a.mov(R5, R0)
    a.and_(R6, R0)
    a.beq('ec_reg')
    a.ldi8(5, R6)                       # メモリ・アクセスは未対応
    ldv(V_FERR, R4)
    a.cmpi(0, R4)
    a.bne('ec_abort')
    stv(R6, V_FERR)
    a.label('ec_abort')
    a.ldi8(0, R2)
    a.bra('ec_loop')
    a.label('ec_reg')
    a.ldi32(0x10000000, R6)
    a.mov(R5, R0)
    a.and_(R6, R0)
    a.beq('ec_nomod')
    a.ld(R1, R6)                        # 修正の mask ワード
    a.addi(4, R1)
    a.subi(1, R2)
    a.ldi20(0xFFFF, R0)
    a.and_(R0, R6)
    a.bra('ec_do')
    a.label('ec_nomod')
    a.ldi8(0, R6)
    a.label('ec_do')
    stv(R5, V_W)
    stv(R6, V_MASK)
    a.call('do_access')
    a.cmpi(0, R0)
    a.beq('ec_ok')
    a.mov(R0, R6)
    ldv(V_FERR, R4)
    a.cmpi(0, R4)
    a.bne('ec_loop')
    stv(R6, V_FERR)
    a.bra('ec_loop')
    a.label('ec_ok')
    ldv(V_W, R5)
    a.ldi32(0x40000000, R6)
    a.and_(R6, R5)
    a.beq('ec_loop')
    ldv(V_NANS, R4)
    a.cmpi(14, R4)
    a.bnc('ec_loop')                    # 答えは 14 ワードまで
    ldv(V_VAL, R5)
    a.mov(R4, R6)
    a.lsl(2, R6)
    a.ldi20(VB + V_ANS, R0)
    a.add(R6, R0)
    a.st(R5, R0)
    a.addi(1, R4)
    stv(R4, V_NANS)
    a.bra('ec_loop')

    a.label('ec_done')
    a.pop(R1)
    a.pop(R3)
    ldv(V_REPLY, R0)
    a.cmpi(0, R0)
    a.beq('ec_adv')
    ldv(V_NANS, R4)
    a.ldi32(EP3_FIFO, R5)
    a.st(R3, R5)                        # 答えのワード 0: このパケットのシーケンス
    ldv(V_FERR, R7)
    a.lsl(16, R7)
    a.or_(R4, R7)
    a.st(R7, R5)                        # ワード 1: 最初の失敗 << 16 | 答えの数
    a.mov(R4, R8)
    a.ldi20(VB + V_ANS, R6)
    a.label('ra_l')
    a.cmpi(0, R8)
    a.beq('ra_done')
    a.ld(R6, R9)
    a.st(R9, R5)
    a.addi(4, R6)
    a.subi(1, R8)
    a.bra('ra_l')
    a.label('ra_done')
    a.ldi32(EP3_KICK, R6)
    a.ldi8(4, R9)
    a.st(R9, R6)                        # 送出
    a.ldi8(0, R9)
    stv(R9, V_FERR)
    stv(R9, V_NANS)
    a.label('ec_adv')
    a.addi(1, R3)
    stv(R3, V_EXPECT)
    a.ldi20(512, R0)
    a.add(R0, R1)
    a.ldi32(CMD_WINDOW + 16 * 512, R0)
    a.cmp(R0, R1)
    a.bne('ec_slot')
    a.ldi32(CMD_WINDOW, R1)
    a.label('ec_slot')
    stv(R1, V_SLOT)
    a.pop(R4)
    a.mov_to_rs(R4, 1)
    a.ret()
    return a.link()


# ---- core 0 (SDRAM リング → エンドポイント 0x81 のパケタイザ) ----
def build_core0():
    a = Asm()
    a.ldi20(0xA37FC, R15)

    def w32(addr, val):
        a.ldi(addr, R0)
        a.ldi(val, R1)
        a.st(R1, R0)

    def w16(addr, val):
        a.ldi(addr, R0)
        a.ldi(val, R1)
        a.sth(R1, R0)

    for addr, val in [
        (0x0000, 0x77), (0x0000, 0), (0x0004, 0xFFFFFFFF), (0x0030, 0x004041FF),
        (0x0050, 0x004041FF), (0x0090, 0xFFFFFFFF), (0x00B8, 0xFFFFFFFF),
        (0x00C8, 0xFFFFFFFF), (0x0010, 0xFFFFFFFF), (0x0020, 0xFFFFFFFF),
        (0x0040, 0), (0x1060, 0x300), (0x1064, 0x300), (0x1068, 0x300),
        (0x106C, 0x300), (0x10B0, 0x300), (0x10B4, 0x300)]:
        w32(addr, val)

    a.label('dmastop')          # DMA の停止待ち
    a.ldi(0x1068, R0)
    a.ld(R0, R1)
    a.ldi(0x300, R2)
    a.and_(R2, R1)
    a.bne('dmastop')

    w32(0x1018, 0x00AE4000)
    w32(0x1028, 0x600000)
    a.ldi32(0xD02038, R0)
    a.ld(R0, R2)                # at
    a.ldi(0x1048, R0)
    a.st(R2, R0)
    a.ldi(0x1038, R0)
    a.st(R2, R0)
    w32(0x1000, 0xFFF00000)

    for addr, val in [
        (0x10010, 0), (0x10030, 0), (0x10034, 0x800), (0x10038, 0x1000),
        (0x1003C, 0x1840), (0x10040, 0x1840), (0x10080, 0xB8), (0x10084, 0xB8),
        (0x10088, 0xC0), (0x1008C, 0xB8), (0x100B0, 0x1880), (0x100B4, 0x1900)]:
        w16(addr, val)

    for addr, val in [
        (0x2020, 0), (0x2024, 0), (0x202C, 0x03000400), (0x2030, 0x05010101),
        (0x3008, 0), (0x5000, 0x411080C0), (0x5008, 0x10000000),
        (0x5004, 0x01000000), (0x200C, 0x01000000)]:
        w32(addr, val)

    # デバッグカウンタ (0xA3F00: ループ, +4: 送出ユニット, +8: 補充, +C: 直近イベント)
    a.ldi20(0xA3F00, R13)
    a.ldi8(0, R0)
    for off in (0, 4, 8, 12):
        a.ldi8(off, R1)
        a.st13(R0, R1)

    w32(0x1078, 0x460)
    w32(0x1068, 0x01000000)
    a.ldi8(1, R8)               # 補充中
    a.ldi8(0, R9)               # ユニット数

    def inc(off):
        a.ldi8(off, R0)
        a.ld13(R0, R1)
        a.addi(1, R1)
        a.st13(R1, R0)

    a.label('loop')
    inc(0)
    a.ldi(0x20, R0)
    a.ld(R0, R2)                # event
    a.ldi8(12, R0)
    a.st13(R2, R0)
    a.ldi32(0x20000000, R3)
    a.mov(R2, R1)
    a.and_(R3, R1)
    a.beq('nofill')
    a.ldi(0x20, R0)
    a.st(R3, R0)                # 補充完了を確認
    a.ldi8(0, R8)
    a.label('nofill')
    a.ldi32(0xD02038, R0)
    a.ld(R0, R1)
    a.ldi(0x1038, R0)
    a.st(R1, R0)                # DMA に、エンジンが書いたところまでを読ませる
    a.cmpi(0, R8)
    a.bne('noref')
    a.ldi20(0x10068, R0)
    a.lduh(R0, R1)
    a.ldi20(0x10058, R0)
    a.lduh(R0, R3)
    a.sub(R3, R1)               # free = u16[0x10068] - u16[0x10058]
    a.bne('fnz')
    a.ldi20(0x840, R1)
    a.bra('fchk')
    a.label('fnz')
    a.bge('fchk')
    a.ldi20(0x840, R3)
    a.add(R3, R1)
    a.label('fchk')
    a.ldi20(0x470, R3)
    a.cmp(R3, R1)
    a.bls('noref')              # free <= 0x470 (符号なし)
    w32(0x1078, 0x460)
    w32(0x1068, 0x01000000)
    a.ldi8(1, R8)
    inc(8)
    a.label('noref')
    a.ldi8(0x20, R3)
    a.and_(R3, R2)
    a.beq('loop')               # 送出するユニットが来ていない
    a.ldi(0x2000, R0)
    a.ld(R0, R1)
    a.ldi8(7, R3)
    a.and_(R3, R1)
    a.bne('loop')               # パケタイザが忙しい
    w32(0x20, 8)
    w32(0x200C, 0x01000000)
    w32(0x2004, 0xBF)
    w32(0x2000, 0x0D000041)
    inc(4)
    a.addi(1, R9)
    a.cmpi(10, R9)
    a.bne('loop')
    a.ldi8(0, R9)
    a.ldi(0x1048, R0)
    a.ld(R0, R1)                # read
    a.ldi8(8, R3)
    a.cmp(R3, R1)
    a.bnc('rdsub')
    a.ldi32(0x600000 - 8, R3)
    a.add(R3, R1)
    a.bra('rdset')
    a.label('rdsub')
    a.subi(8, R1)
    a.label('rdset')
    a.ldi32(0xD0202C, R0)
    a.st(R1, R0)
    a.bra('loop')
    return a.link()


def emit_array(out, name, words):
    out.write('static const std::uint32_t %s[] = {\n' % name)
    for i in range(0, len(words), 6):
        out.write('\t' + ', '.join('0x%08X' % w for w in words[i:i + 6]) + ',\n')
    out.write('};\n\n')


def main():
    progs = [('kZd1FwAddrRead', build_addr(False)),
             ('kZd1FwAddrWrite', build_addr(True)),
             ('kZd1FwCore1', build_core1()),
             ('kZd1FwCore0', build_core0())]
    with open(sys.argv[1], 'w', newline='\n') as out:
        out.write('// zd1_firmware.h\n// zd1fw_gen.py が生成する。手で編集しない。\n\n')
        out.write('#pragma once\n\n#include <cstdint>\n\nnamespace px4 {\nnamespace zd1fw {\n\n')
        for name, asm in progs:
            emit_array(out, name, asm.words())
        out.write('} // namespace zd1fw\n} // namespace px4\n')
    for name, asm in progs:
        print('%-18s %4d words' % (name, len(asm.words())))


if __name__ == '__main__':
    main()
