# 26matchF4

浙江省机器人竞赛旅游车探险赛道代码。

## 目录结构

```text
26matchF4/
├─ Core/                  STM32CubeMX 生成的启动与外设代码
│  ├─ Inc/
│  └─ Src/
├─ Drivers/               STM32 HAL 与 CMSIS
├─ User/                  项目业务代码
│  ├─ App/                底盘行为、任务和路线
│  │  ├─ Inc/
│  │  └─ Src/
│  ├─ Device/             电机、传感器、IMU、OLED、机械臂等设备驱动
│  │  ├─ Inc/
│  │  └─ Src/
│  ├─ Bsp/                CAN、DWT 等板级支持
│  │  ├─ Inc/
│  │  └─ Src/
│  └─ Algorithm/          PID 与通用算法
│     ├─ Inc/
│     └─ Src/
├─ MDK-ARM/               Keil 工程、启动文件、RTE 与编译输出
└─ 26matchF4.ioc    STM32CubeMX 工程
```

## 文件归属约定

- `MDK-ARM` 中不再放业务 `.c/.h` 文件，只保留 Keil 自身需要的文件。
- 模块头文件放在对应的 `Inc`，实现文件放在对应的 `Src`。
- 新增模块后，需要同时加入 `MDK-ARM/26matchF4.uvprojx` 的对应 `User/*` 分组。
- CubeMX 生成的文件继续留在 `Core`，避免与手写业务代码混放。
