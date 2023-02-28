int8 aim_id=0
uint16 car_id          # 正在击打的目标
uint16 detection_info  # 识别到的序列(0-8装甲板, 9-10能量机关)
float32 yaw = 0
float32 pitch = 0
float32 yaw_v = 0
float32 pitch_v = 0
float32 dist = 0
uint8 shoot             # ShootMode
uint16 period = 0       # 2021 版反陀螺的延迟发射要求
uint8 lrc=0
bool lock_yaw=0
