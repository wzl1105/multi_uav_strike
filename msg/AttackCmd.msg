# MAV_CMD_ATTACK(51001) 的 typed 版本
# 目标锁定时有效：SEARCH_TRACK 支持全部操作；SEARCH_STRIKE
# 自动锁定后支持 action=1/2，用于取消本次自动制导。
# action:
#   0 = 打击(解保,触发打击流程)
#   1 = 忽略(取消跟踪,目标不上报)
#   2 = 暂存(取消跟踪,目标按 type=2 上报)
uint8    action
float32  strike_speed
