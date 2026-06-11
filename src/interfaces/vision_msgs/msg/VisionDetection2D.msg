## 保留原始检测信息

std_msgs/Header header          # stamp=图像时间, frame_id=相机光学坐标系
string camera_name
uint32 image_width
uint32 image_height

int32 track_id                  # 跟踪ID, 无则-1
string class_name
int32 class_id
float32 confidence              # [0,1]

# bbox (xywh, 像素)
float32 bbox_cx
float32 bbox_cy
float32 bbox_w
float32 bbox_h

# 可选关键点, 像素坐标扁平数组 [x1,y1,x2,y2,...]
float32[] keypoints_xy
float32[] keypoints_score