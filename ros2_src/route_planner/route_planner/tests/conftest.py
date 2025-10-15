"""pytest用の共通設定。"""

from __future__ import annotations

import os
import sys

# pytest実行時のみスタブモジュールを優先利用する
_STUB_PATH = os.path.join(os.path.dirname(__file__), "stubs")
if _STUB_PATH not in sys.path:
    sys.path.insert(0, _STUB_PATH)
