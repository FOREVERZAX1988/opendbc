"""Typed access to the carrot/sunnypilot Params store from opendbc code.

The HKG/GM ports in this fork read tunables through ``get_int`` / ``get_float`` /
``get_bool`` / ``put_bool``. ``openpilot.common.params.Params`` only exposes ``get`` /
``get_bool`` / ``put`` / ``put_bool``, so those call sites raised AttributeError at
runtime (e.g. ``CarState.__init__`` for every Hyundai/Kia CAN-FD car).

sunnypilot's carrot ``UnifiedParams`` provides the typed API (plus the carrot
defaults), so this module hands it out. When opendbc is imported without openpilot
(standalone tests, other consumers) it degrades to ``None`` and callers fall back to
their own defaults.
"""
import importlib

_UnifiedParams = None
_loaded = False


def _unified_params_class():
  global _UnifiedParams, _loaded
  if not _loaded:
    _loaded = True
    try:
      _UnifiedParams = importlib.import_module("openpilot.sunnypilot.carrot.config").UnifiedParams
    except Exception:
      _UnifiedParams = None
  return _UnifiedParams


def get_params():
  """Return the carrot Params accessor, or None when openpilot is unavailable."""
  cls = _unified_params_class()
  if cls is None:
    return None
  try:
    return cls()
  except Exception:
    return None


def get_int(key: str, default: int = 0) -> int:
  p = get_params()
  if p is None:
    return default
  try:
    return int(p.get_int(key, default))
  except Exception:
    return default


def get_float(key: str, default: float = 0.0) -> float:
  p = get_params()
  if p is None:
    return default
  try:
    return float(p.get_float(key, default))
  except Exception:
    return default


def get_bool(key: str, default: bool = False) -> bool:
  p = get_params()
  if p is None:
    return default
  try:
    return bool(p.get_bool(key, default))
  except Exception:
    return default


def put_bool(key: str, value: bool) -> None:
  p = get_params()
  if p is None:
    return
  try:
    p.put_bool(key, bool(value))
  except Exception:
    pass
