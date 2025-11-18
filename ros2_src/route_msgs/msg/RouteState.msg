# route_manager 観点のルート状態の概要（監視・UI向け）

std_msgs/Header header
int32 route_version
int32 total_waypoints
int32 current_index
string current_label

uint8 STATUS_UNKNOWN=0
uint8 STATUS_IDLE=1
uint8 STATUS_RUNNING=2
uint8 STATUS_UPDATING_ROUTE=3
uint8 STATUS_HOLDING=4
uint8 STATUS_COMPLETED=5
uint8 STATUS_ERROR=6

uint8 status
string message



