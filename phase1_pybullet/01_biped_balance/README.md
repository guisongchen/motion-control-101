# 项目1.1: PyBullet 2D双足站稳

## 目标
让双足机器人在仿真中稳定站立5秒以上。

## 当前状态
基础PD控制已实现，但机器人在 ~2.7秒 摔倒。

## 问题分析
当前简单PD控制的问题：
1. **没有考虑质心位置** - 只控制关节角度，不控制整体平衡
2. **没有考虑支撑面** - 需要ZMP在支撑多边形内
3. **控制策略过于简单** - 需要更复杂的平衡策略

## 改进方向

### 1. 加入质心高度控制
- 测量 torso 高度
- 控制髋关节保持目标高度

### 2. 加入姿态控制
- 根据 torso 倾斜角度调整踝关节
- 类似倒立摆的平衡策略

### 3. 加入脚接触力反馈
- 检测脚是否着地
- 根据接触力调整姿态

## 运行

```bash
uv run python biped_balance.py
```

## 验收标准
- [ ] 机器人能在原地站立5秒不倒
- [ ] 能抵抗小扰动（用手推不倒）- 可用 `p.applyExternalForce` 测试
- [ ] 理解PD参数对稳定性的影响

## 参考
- PyBullet Humanoid例子: `examples/pybullet/examples/humanoid.py`
- 文档: `doc/motion_control_practice_guide.md`
