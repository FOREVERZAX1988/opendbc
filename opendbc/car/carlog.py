import os
import logging

# set up logging
LOGPRINT = os.environ.get('LOGPRINT', 'INFO').upper()
carlog = logging.getLogger('carlog')
carlog.setLevel(LOGPRINT)
carlog.propagate = False

handler = logging.StreamHandler()
handler.setFormatter(logging.Formatter('%(message)s'))
carlog.addHandler(handler)

# 已知持续性告警的去重表：同 key 的 error 进程内只报一次。
# 典型场景：EPS-patched（横向收假 MAC）+ 无 SecOC 密钥时，每次
# SECOC_SYNCHRONIZATION 计数器跳变都必然失配，属已知无害告警，不应刷屏。
_reported_once: set = set()


def error_once(key: str, msg: str) -> None:
  if key in _reported_once:
    return
  _reported_once.add(key)
  carlog.error(msg)
