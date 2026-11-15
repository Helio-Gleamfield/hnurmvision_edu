# 开源方案调研与本作业采用的方法

本次阅读了两所高校的源码，并额外核对 rm_auto_aim 上游与 OpenCV 官方接口。以下比较以所链接版本为准，不把本作业的匀速模型称为完整比赛级旋转目标模型。

| 项目 | 位姿解算与坐标 | 跟踪与选板 | 本作业采用 |
|---|---|---|---|
| 西安理工大学 NEXT E 2024 | 已知大小板尺寸，米制物点，四角点对应，IPPE | 九维目标状态 EKF；预测位置与观测关联，结合编号与朝向门限；DETECTING/TRACKING/TEMP_LOST/LOST | 采用同一坐标系内的预测关联、时间间隔和状态机思想。OpenCV 不识别编号，因此不伪造编号门控；采用六维线性匀速 KF，直接跟踪板中心 |
| 南京理工大学 Alliance rmcs_auto_aim_v2 | RobustPnpSolution 用 IPPE 多解，正深度、姿态物理先验与重投影筛选，并有 yaw 重投影优化 | 模块化 Robot/Outpost/Rune 模型、噪声参数和超时机制 | 采用平面多解筛选、LM 精化，以及可选的已知 15°装甲板倾角约束；在倾角约束下搜索 yaw、最小二乘求平移，保留重投影门限与回退 |

## 阅读的固定版本

- NEXT E 官方 README 明确学校与项目归属：[项目说明](https://github.com/NEXT-E-Vision/autoaim_nexte_2024/blob/96a3fb41f20f886b6978bb149cf2e7688caa5a73/readme.md)。
- NEXT E：[PnP](https://github.com/NEXT-E-Vision/autoaim_nexte_2024/blob/96a3fb41f20f886b6978bb149cf2e7688caa5a73/rm_auto_aim/armor_detector/src/pnp_solver.cpp)、[跟踪器](https://github.com/NEXT-E-Vision/autoaim_nexte_2024/blob/96a3fb41f20f886b6978bb149cf2e7688caa5a73/rm_auto_aim/armor_tracker/src/tracker.cpp)。
- Alliance 官方 README 明确南京理工大学归属：[项目说明](https://github.com/Alliance-Algorithm/rmcs_auto_aim_v2/blob/4f4340956a989b0d3ecd4fbd94b1e72a301293cd/README.md)。
- Alliance：[位姿解算](https://github.com/Alliance-Algorithm/rmcs_auto_aim_v2/blob/4f4340956a989b0d3ecd4fbd94b1e72a301293cd/src/utility/math/solve_pnp/pnp_solution.cpp)、[跟踪组织与超时](https://github.com/Alliance-Algorithm/rmcs_auto_aim_v2/blob/4f4340956a989b0d3ecd4fbd94b1e72a301293cd/src/kernel/tracker.cpp)。
- [rm_auto_aim 上游](https://github.com/chenjunnn/rm_auto_aim)与 [OpenCV PnP 官方说明](https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html)。

## 实际实现与边界

本作业使用 [p,v] 六维线性状态，按实际图像时间间隔更新 F 与白噪声加速度 Q，使用 Joseph 形式更新协方差。选板仅使用检测结果、几何评分、重投影残差和预测创新距离；仿真真值只进入独立验收器，不进入 PnP 或滤波器。

默认基础接口可关闭倾角先验，用于一般平面目标；课程演示针对已知倾斜装甲板开启 15°先验。此先验来源于固定机械结构，不能用于倾角未知的任意平面。匀速预测在正弦运动拐点仍会存在模型误差，未实现车体旋转、换板半径或弹道模型。
