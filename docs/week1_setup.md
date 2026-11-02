# 第一周环境与仓库记录

- GitHub 用户名：`Helio-Gleamfield`
- 个人 Fork：<https://github.com/Helio-Gleamfield/hnurmvision_edu>
- 开发分支：`feature/Helio-Gleamfield-intro`
- 本地系统：Ubuntu 22.04.5 LTS，x86_64。
- 已检查工具：Git 2.34.1、VS Code 1.142.0、G++ 11.4.0、CMake 3.22.1。

## 部署记录

已 Clone 个人 Fork，并确认 `origin` 指向个人仓库。`upstream` 用于获取原仓库更新，
已禁用其推送地址，避免误推送。

当前预检尚未找到 ROS2 Humble、colcon、Rustup/Rust 1.95.0，以及 OpenCV、Eigen3、
yaml-cpp 开发包。主程序与仿真器尚未构建，运行结果和截图待依赖配置完成后核验。

按仓库 README，先执行 `./scripts/build_training.sh`，再执行
`./scripts/build_simulator.sh`。运行时分别使用 `./scripts/run_simulator.sh` 和
`./scripts/run_training.sh`。

保留原始 `configs/simulation.yaml`，其中 `allow_fire: false`。
本次仅新增环境记录，没有修改算法骨架、相机标定或训练评分逻辑。
