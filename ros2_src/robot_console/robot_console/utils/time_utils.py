"""時間関連のユーティリティ。"""

from datetime import datetime, timezone


def now() -> datetime:
    """UTC タイムゾーンの現在時刻を返す。"""

    return datetime.now(tz=timezone.utc)
