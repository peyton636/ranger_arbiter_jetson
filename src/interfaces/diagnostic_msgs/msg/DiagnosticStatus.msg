# 单个组件的诊断状态

# 运行等级
byte OK=0                           # 正常
byte WARN=1                         # 警告
byte ERROR=2                        # 错误
byte STALE=3                        # 数据过期

byte level                          # 当前等级（见上）
string name                         # 组件/测试名称
string message                      # 状态描述
string hardware_id                  # 硬件唯一标识
KeyValue[] values                   # 扩展键值对
