## 桥接节点统一输出

std_msgs/Header header
VisionDetection2D detection_2d
VisionTarget3D target_3d
GraspCandidate[] candidates
int32 selected_index            # -1 表示无有效候选
string reject_reason            # 无候选时填写原因