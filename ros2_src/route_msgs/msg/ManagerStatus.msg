# route_manager が外部へ公開する状態情報（Phase2仕様）

std_msgs/Header header
string state        # "idle", "running", "updating_route", "holding"
string decision     # "none", "update", "replan_first", "shift", "skip", "failed"
string last_cause   # HOLDING時のみ滞留理由（"front_blocked", "no_hint", など）
uint32 route_version
