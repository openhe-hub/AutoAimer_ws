uint8 aim_id
uint16 car_id          # 正在击打的目标
uint16 detection_info  # 识别到的序列(0-8装甲板, 9-10能量机关)
float32 yaw 
float32 pitch 
float32 yaw_v 
float32 pitch_v 
float32 dist 
uint8 shoot             # ShootMode
uint16 period       # 2021 版反陀螺的延迟发射要求
uint8 lrc
bool lock_yaw
