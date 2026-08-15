"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import IntFlag


class VolkswagenFlagsSP(IntFlag):
  STOP_AND_GO = 1  # Macan 起步跟停：原厂停车保持态时由视觉模型决定起步，OP 代发 RESUME 按键帧解除
