#!/usr/bin/env python3
"""VW MLB（Macan 纵向代发）ACC_05 / TSK_04 校验信号名 —— 防回归断言。

背景（2026-09-29 复核，结论见 superproject 的
DAY_LOG_2026-09-29-MLB_CHECKSUM_NAME_GUARD.md）：
上游 `mouxan/master` 线把 `vw_mlb.dbc` 里两条报文的首字节校验信号改了名——
  BO_ 269 ACC_05: CHECKSUM -> ACC_05_CHK
  BO_ 270 TSK_04: CHECKSUM -> TSK_04_CHK
（我们追随的 C3 线 `mouxan/master-c3` / `mouxan/tn-c3`、以及本 fork 的
`macanlong-test` 都仍然是 `CHECKSUM`，所以当前**不需要**跟进这个改名。）

而 opendbc 这条链路上名字是**字面硬编码**的：
  1. `opendbc/can/dbc.py::set_signal_type()`：只有 `sig.name == "CHECKSUM"`
     才会把 `SignalType.VOLKSWAGEN_MLB_CHECKSUM` + `volkswagen_mlb_checksum`
     挂到该信号上；
  2. `opendbc/can/packer.py::make_can_msg()`：只按 `sig.type > SignalType.COUNTER`
     自动回填校验字节。

Macan 纵向是 **OP 代发** ACC_05(0x10D)/TSK_04(0x10E)——`mlbcan.create_acc_accel_control`
组出来的 `acc_05_values` **不手填** CHECKSUM，完全依赖上面第 2 步回填。因此一旦把
改名版 DBC 换进来、而 `can/dbc.py` 没同步放宽，OP 发出去的 ACC_05/TSK_04 校验字节
会**恒为 0**，原厂 ACC/网关 CRC 校验过不去 —— 正是最怕的"原厂不认 OP 帧"故障。
（已实测：用改名版 DBC 打包 -> 两条报文校验字节都 0x00。）
上游 commaai 线不主动 TX 这两条报文，所以这个坑在上游不会暴露。

本测试把该不变量钉死，被动红（不靠记忆）：
  * DBC 侧改名            -> test_checksum_signal_name_and_type 失败并给出修复指引
  * dbc.py 丢名字处理     -> 同上（无信号被挂上校验类型）
  * packer 不再自动回填   -> test_packer_autofills_checksum_bytes 失败
"""
import unittest

from opendbc.can import CANPacker
from opendbc.can.dbc import DBC, SignalType

# 校验信号在本 fork 内必须是这个字面名（can/dbc.py 就是按它匹配的）
CANONICAL_CHECKSUM_NAME = "CHECKSUM"

# Macan 纵向由 OP 代发、且校验靠自动回填的两条报文：(报文名, 11bit 地址, 打包用物理量)
CASES = (
  ("ACC_05", 0x10D, {"ACC_Momentenanforderung": 500.0}),
  ("TSK_04", 0x10E, {"TSK_Wunsch_Uebersetz": 10.0}),
)

FIX_HINT = "".join((
  "若确认要吸收把校验信号改名为 ACC_05_CHK / TSK_04_CHK 的 DBC，必须在**同一个提交**里把 ",
  "opendbc/can/dbc.py::set_signal_type() 放宽（例：sig.name == 'CHECKSUM' or sig.name.endswith('_CHK')），",
  "否则 CANPacker 不会回填校验字节 -> 原厂会拒收 OP 代发的 ACC_05/TSK_04。",
))

RENAMED_MSG = "".join(("{}(0x{}) 的校验信号必须仍叫 'CHECKSUM'，当前信号名={}\n", FIX_HINT))
UNWIRED_MSG = "".join(("{} 必须恰好有一个信号被挂上校验类型（type > SignalType.COUNTER），当前={}\n", FIX_HINT))
MISSING_MSG = "".join(("{} 没有可用的校验信号，无法回填校验字节\n", FIX_HINT))
ZERO_MSG = "".join(("{} 校验字节恒为 0 —— 校验信号没被识别/未回填，原厂会拒收\n", FIX_HINT))


class TestMacanMlbChecksumWiring(unittest.TestCase):

  def test_checksum_signal_name_and_type(self):
    """DBC + can/dbc.py 的接线不变量：信号仍叫 CHECKSUM、且被挂上校验类型。"""
    dbc = DBC("vw_mlb")
    for msg_name, address, _ in CASES:
      with self.subTest(msg=msg_name):
        msg = dbc.name_to_msg[msg_name]
        self.assertEqual(msg.address, address)

        names = [s.name for s in msg.sigs.values()]
        self.assertIn(CANONICAL_CHECKSUM_NAME, names, RENAMED_MSG.format(msg_name, f"{address:X}", names))

        wired = [s for s in msg.sigs.values() if s.type > SignalType.COUNTER]
        self.assertEqual([s.name for s in wired], [CANONICAL_CHECKSUM_NAME], UNWIRED_MSG.format(msg_name, wired))

        sig = wired[0]
        self.assertEqual((sig.start_bit, sig.size), (0, 8), f"{msg_name} 校验字节应在首字节")
        self.assertEqual(sig.type, SignalType.VOLKSWAGEN_MLB_CHECKSUM)
        self.assertIsNotNone(sig.calc_checksum, f"{msg_name} 校验算法未挂载 (calc_checksum is None)")

  def test_packer_autofills_checksum_bytes(self):
    """功能面：OP 代发的两帧必须带**非 0**且与算法一致的校验字节。

    覆盖"名字对但 packer 不再回填"的旁路（例如有人改坏 CANPacker 的
    `sig.type > SignalType.COUNTER` 判定）。
    """
    packer = CANPacker("vw_mlb")
    dbc = DBC("vw_mlb")
    for msg_name, address, values in CASES:
      with self.subTest(msg=msg_name):
        addr, dat, _ = packer.make_can_msg(msg_name, 0, values)
        self.assertEqual(addr, address)

        wired = [s for s in dbc.name_to_msg[msg_name].sigs.values() if s.type > SignalType.COUNTER]
        self.assertEqual(len(wired), 1, MISSING_MSG.format(msg_name))

        sig = wired[0]
        payload = bytearray(dat)
        payload[0] = 0  # 只对 payload 求校验（xor 本身会跳过校验字节，这里显式清零）
        expected = sig.calc_checksum(address, sig, payload)

        self.assertNotEqual(dat[0], 0, ZERO_MSG.format(msg_name))
        self.assertEqual(dat[0], expected, f"{msg_name} 校验字节与 volkswagen_mlb_checksum 算出的不一致")


if __name__ == "__main__":
  unittest.main()
