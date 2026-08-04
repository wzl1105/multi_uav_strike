std_msgs/Header header

# === 状态字 0 (8 个 bit,LSB=D0) ===
bool device_ok           # D0  设备是否工作正常
bool self_test_done      # D1  是否自检完成
bool mode_qb_reserved    # D2  QB 模式标志(原文 BAOZHA,预留)
bool stage1_released     # D3  第一级保险解除
bool stage2_released     # D4  第二级保险解除
bool in_attack_state     # D5  进入攻击状态(12V 供电 + 三级解保后自动)
bool qb_done             # D6  QB 完成
bool self_destruct       # D7  自毁

uint8  raw_status_byte   # 原始状态字 0 字节(便于调试核对)
bool   frame_valid       # 上行帧累加校验是否通过