# 电子设计协会软件培训第三题

**项目状态 · 全部功能已通过实板测试。** 本项目面向《2026 创新基地班软件考核》第三题，在 **KF56-10 冠军开发板**上实现双通道触摸屏示波器。固件基于 **STM32F407ZGT6**、STM32CubeMX 和 HAL，使用板载 **ILI9341** 屏幕显示波形，并通过板载 USB 串口向 VOFA+ 发送数据。

## 功能概览

上电默认进入 **AUTO** 时域页，时基为 **200 µs/格**，垂直档位为 **500 mV/格**。如果出现黄色校准十字，依次点准并松开四个十字，校准结果会保存在板载 EEPROM。

| 功能 | 操作与显示 |
| --- | --- |
| **ADC 采集与上位机波形** | TIM2 以 **200 kHz** 触发 ADC，DMA 搬运样点。VOFA+ 可查看 **100 Hz～20 kHz** 的输入波形。 |
| **时域显示与自动测量** | TFT 按触发位置对齐波形，显示网格、时间和电压刻度、频率、峰峰值及直流分量。自动识别正弦波、方波和三角波。 |
| **触发模式与设置** | `MODE` 切换自动、正常和单次。自动档无触发时继续刷新，正常档保持上一帧，单次档捕获后冻结。点 `SINGLE` 重新武装，点 `EDGE` 切换边沿，点亮 `LEVEL` 后上下拖动调整触发电平。 |
| **时基与垂直档位** | `TIME` 按 **25 µs/格～5 ms/格** 的八档时基切换，`V/DIV` 按 **50 mV/格～1 V/格** 的六档垂直档位切换。 |
| **冻结与双游标** | 点 `CURSORS` 或 `RUN/STOP` 冻结当前帧，再拖动两条竖线读取时间差 `dT` 和换算频率 `1/dT`。 |
| **串口控制与查询** | 串口和触摸共用同一份配置。支持运行、冻结、单次捕获、触发设置和测量查询，非法、越界或超长指令返回 `ERR`。 |
| **双通道与相位差** | `CH1/2` 切换单路或双路显示。ADC1 和 ADC2 同步采样，两路分别显示测量值。同频时显示 **0°～360°** 的相位差，CH2 相对 CH1 超前为正。 |
| **谐波与 THD** | `FFT` 切换频谱页，显示 CH1 的 **1～5 次谐波**幅值柱图和总谐波失真。 |
| **自动量程** | `RANGE` 切换自动与手动量程。输入在 **0.1～3.0 Vpp** 范围内变化时自动调整垂直档位，使波形约占绘图区高度的 **60%～80%**。 |

## 硬件接口

| 功能 | 开发板接口 | 对应外设 |
| --- | --- | --- |
| CH1 模拟输入 | **P8-29 → PA5** | ADC1_IN5 |
| CH2 模拟输入 | **P8-32 → PA6** | ADC2_IN6 |
| 信号地 | **J7-1 → GND** | 信号发生器和开发板共地 |
| 上位机串口 | **USB3**，保留 P12 的 **1–2、3–4** 跳帽 | CH340C、USART1（PA9 TX、PA10 RX），1,200,000 baud、8N1 |
| 屏幕和触摸 | **LCD1** | ILI9341，触摸线为 PB0、PB1、PB2、PC13、PF11 |
| 烧录与调试 | **JTAG1** | ST-LINK / SWD（PA13、PA14） |

接外部信号前，确认每路输入都在 **0～3.3 V** 内。接 CH1 时，断开 J4 上与 `STM_ADC` 相连的跳帽。接 CH2 时，先从 P2 拔掉摄像头。首次联调可用 **1 kHz 正弦波、1 Vpp、1.5 V 直流偏置**。完整接线和逐项验收见 [引脚与功能测试](docs/引脚与功能测试.md)。

## 构建与烧录

用 VS Code 打开工程根目录，在 STM32Cube 扩展或 CMake Tools 中选择 `Release` 预设构建。命令行构建需要将 CMake、Ninja 和 Arm GNU Toolchain 加入 `PATH`。

```sh
cmake --preset Release
cmake --build --preset Release
```

用 STM32CubeProgrammer 通过 **ST-LINK / SWD** 下载并校验 `build/Release/Third_Task.elf`。需要源码调试时可改用 `Debug` 预设。正式演示前先打开串口桥接并连好 VOFA+，演示期间保持串口连接。

## Mac 上的 VOFA+

工程自带 Python 桥接，将 CH340C 串口发来的样点转换为 JustFloat。在工程根目录执行以下命令。

```sh
python3 -m venv build/host-venv
build/host-venv/bin/python -m pip install -r tools/requirements.txt
build/host-venv/bin/python tools/vofa_bridge.py --wave-file build/capture.csv
```

1. 在 VOFA+ 选择 **TCP 客户端**和 **JustFloat**，连接 `127.0.0.1:9000`，将 CH1 绑定到波形控件。Mac 上有多个串口设备时，启动桥接可用 `--port` 指定实际设备。
2. 双路显示在屏幕上点 `CH1/2` 或发送 `CHAN 2`，双路串流另发 `STREAM 2`。单路横轴默认按 **5 µs/点**（含插值），加 `--raw-points` 后按实收 **10 µs/点**，双路按 **20 µs/点**。
3. 在桥接终端或 VOFA+ 发送区以 ASCII 加换行发送命令。`MEAS?` 查询读数，`WAVE?` 将当前捕获的 240 点波形保存到 `build/capture.csv`，`CONF?` 和 `STAT?` 分别查询配置和运行状态。

常用控制命令包括 `RUN`、`STOP`、`SINGLE`、`TRIG 1.5`、`EDGE RISE`、`TB 500us` 和 `VDIV 500mV`。`STOP` 冻结屏幕捕获，`STREAM OFF` 停止向上位机连续发送波形。需要重新校准触摸时点屏幕上的 `CAL` 或发送 `CAL`。

## 代码结构

| 路径 | 职责 |
| --- | --- |
| [App/app.c](App/app.c) / [App/acquisition.c](App/acquisition.c) | 应用初始化、主循环调度、读取 ADC DMA 半块并保存同步样点 |
| [App/scope.c](App/scope.c) | 触发判定、时基、量程、发布和冻结捕获帧 |
| [App/measurement.c](App/measurement.c) / [App/spectrum.c](App/spectrum.c) | 时域参数、波形识别、相位差、FFT、谐波和 THD |
| [App/ui.c](App/ui.c) / [Screen/](Screen/) | 波形与频谱绘制、触摸交互、LCD 和触摸校准驱动 |
| [App/commands.c](App/commands.c) / [App/serial_link.c](App/serial_link.c) / [App/stream_encode.c](App/stream_encode.c) | 串口命令、DMA 收发、波形打包和串流编码 |
| [tools/](tools/) | VOFA+ 桥接、DSP 回归、实板串口和桥接测试 |
| [Core/](Core/) / [Third_Task.ioc](Third_Task.ioc) | CubeMX 外设配置、初始化代码和中断入口 |

调整采样率时，同步修改 `Third_Task.ioc` 的 TIM2 周期和 [App/scope_config.h](App/scope_config.h) 的 `SCOPE_SAMPLE_HZ`。时基和垂直档位表在 [App/scope.c](App/scope.c)。各功能的源码入口和实现步骤见 [功能实现导读](docs/功能实现导读.md)，采集、显示和串口的数据流见 [架构说明](docs/架构.md)。
