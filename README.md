# 车载平衡滚球控制系统

本仓库保存可直接修改和回退的正式源码，不再使用历史 `ROUND-*` 差异目录。

## 目录

- `mspm0/`：立创地猛星 MSPM0G3507 CCS 工程与电机闭环控制。
- `k230/`：K230 视觉识别与UART位置输出脚本。

## 当前控制版本

当前MSPM0入口为 `mspm0/empty.c`，构建标识：

```text
ROUND-036_BALANCE_MEASUREMENT_V1
```

当前为视觉标定与手动角度实验平台：上电不自动运动，VOFA FireWater每20ms显示球位置、球速度、电机目标角和实际角。通过UART0发送CAL、ANGLE、STOP等命令。

通过 JOG,+1 / JOG,-1 微调当前角度，用 CAL,BALANCE 采集实测平衡零点。

当前步骤见 [平衡位置测量](mspm0/BALANCE_MEASUREMENT.md)，VOFA接线见 [实验说明](mspm0/EXPERIMENT_STEP1.md)。

## CCS使用

在CCS中导入 mspm0 工程。生成文件留在 Debug/，不会提交到Git。确保每个模块仅编译一份，新增的 Control/experiment_protocol.c 和 Hardware/experiment_console.c 必须参与编译。旧视觉轨迹及远程模块不由当前入口调用。

现在只交付源码，不默认生成固件。每轮刷新工作区根目录的“待复制文件”；在CCS中自行编译烧录。

## Git约定

后续每轮修改直接在本仓库中进行，并用提交消息标注轮次，例如：

```text
ROUND-033: describe the tested control adjustment
```

这样可使用 `git log`、`git show` 与 `git revert` 安全回退。
