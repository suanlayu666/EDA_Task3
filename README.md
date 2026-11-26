# 电子设计协会软件培训第三题

**项目状态 · 题目功能已实现。** 本仓库包含双通道触摸屏示波器的基本功能，以及双通道测量、相位差、谐波与 THD、自动量程四项拓展功能。项目基于 KF56-10 冠军开发板、STM32F407ZGT6 和板载 ILI9341 触摸屏，使用 STM32CubeMX、HAL、VS Code、CMake 和 Arm GNU Toolchain。界面由板载 LCD 驱动绘制，上位机通过 Python 桥接连接 VOFA+。

固件已进行算法回归和实板串口联调。模拟输入精度、双路相位精度和实际显示效果的逐项验收方法见 [引脚与功能测试](docs/引脚与功能测试.md)。

## 功能与操作

上电进入 `AUTO` 时域页，默认单通道、上升沿触发，触发电平 **1.50 V**，时基 **200 µs/格**，垂直档位 **500 mV/格**。如果屏幕显示四点校准提示，按顺序点准并松开四个十字，完成后进入波形页。校准结果保存在板载 EEPROM，需要重新校准时发送串口命令 `CAL`。

| 功能 | 操作与显示 |
| --- | --- |
| **ADC 采集与上位机波形** | TIM2 每 **5 µs** 触发 ADC1、ADC2 同步转换，DMA 循环搬运样点，每路原始采样率 **200 kS/s**。上位机单路串流 **100 kS/s**，双路串流每路 **50 kS/s**，用于观察 **100 Hz～20 kHz** 输入波形。 |
| **时域显示与自动测量** | 屏幕显示网格、时间和电压刻度、频率、峰峰值及直流分量，识别正弦波、方波和三角波。波形按触发位置对齐，触发参考为 CH1。 |
| **运行、冻结与单次捕获** | 点 `RUN/STOP` 切换运行和冻结。点 `SINGLE` 武装一次捕获，满足触发条件后冻结，再点可重新武装。冻结屏幕时，ADC 采集和上位机连续串流仍继续。 |
| **触发模式与设置** | `MODE` 切换自动、正常和单次。自动档无触发时超时刷新，正常档等到有效触发才更新，单次档捕获后保持。`EDGE` 切换上升沿与下降沿，点亮 `LEVEL` 后在波形区上下拖动调整触发电平。 |
| **时基与垂直档位** | `TIME` 切换 **25、50、100、200、500 µs/格、1、2、5 ms/格**。`V/DIV` 切换 **50、100、200、250、500 mV/格、1 V/格**。时基改变显示时间窗口，垂直档位改变电压到像素的缩放。 |
| **双游标测量** | 点 `CURSORS` 冻结波形，拖动两条竖线读取时间差 `dT` 和换算频率 `1/dT`。将两条线放在相邻周期的相同位置，可测量周期和频率。 |
| **串口控制与查询** | 触摸与串口共用同一份配置。支持运行、冻结、触发设置、测量和波形查询。指令以回车或换行结束，非法、越界或超长指令返回 `ERR`，未结束的指令超时后清除。 |
| **双通道与相位差** | `CH1/2` 切换单路或双路显示，两路用不同颜色绘制并分别显示测量值。同频时显示 **0°～360°** 相位差，CH2 相对 CH1 超前为正。 |
| **谐波与 THD** | `FFT` 切换时域和频谱页，通过 **4096 点 FFT** 显示 CH1 的 **1～5 次谐波**幅值及 THD。THD 按第 2～5 次谐波计算，定量测试使用 **1～5 kHz** 基波。 |
| **自动量程** | `RANGE` 切换自动和手动量程。输入在 **0.1～3.0 Vpp** 范围内变化时自动调整每格电压，使波形约占绘图区高度的 **60%～80%**。 |

## 硬件接口

| 功能 | 开发板接口 | 外设 |
| --- | --- | --- |
| CH1 模拟输入 | **P8-29 → PA5** | ADC1_IN5，网名 `STM_ADC` |
| CH2 模拟输入 | **P8-32 → PA6** | ADC2_IN6，网名 `DCMI_PCLK` |
| 信号地 | **J7-1 → GND** | 两路信号源和开发板共地 |
| 上位机串口 | **USB3**，P12 保留 **1–2、3–4** 跳帽 | CH340C、USART1，PA9 TX / PA10 RX，**1,200,000 baud、8N1** |
| 屏幕与触摸 | **LCD1** | ILI9341 / FSMC，触摸使用 PB0、PB1、PB2、PC13、PF11 |
| 触摸校准保存 | 板载 EEPROM | PB8 SCL / PB9 SDA |
| 烧录与调试 | **JTAG1** | ST-LINK / SWD，PA13 SWDIO / PA14 SWCLK |

接外部信号前，断开 J4 上与 `STM_ADC` 相连的跳帽，使用 CH2 时先拔下 P2 摄像头。每路输入保持在 **0～3.3 V**，首次测试可设为 **1 kHz 正弦波、1 Vpp、1.5 V 直流偏置**。核对信号发生器的负载设置和实际输出电压。完整板内接线、各功能测试步骤及通过标准见 [引脚与功能测试](docs/引脚与功能测试.md)。

## 构建与烧录

在 VS Code 中打开项目根目录，通过 STM32Cube 扩展或 CMake Tools 选择 `Release` 预设并构建。命令行环境中，确保 CMake、Ninja 和 `arm-none-eabi-gcc` 已加入 `PATH`，然后执行：

```sh
cmake --preset Release
cmake --build --preset Release
```

构建产物为 `build/Release/Third_Task.elf`。通过 STM32CubeProgrammer 使用 ST-LINK / SWD 烧录并校验，需要源码调试时改用 `Debug` 预设，产物为 `build/Debug/Third_Task.elf`。正式演示前接好信号源和上位机，在同一次上电中依次操作各项功能。

## Mac 上的 VOFA+

工程自带 Python 桥接，将串口数据校验、解包后转换为 VOFA+ 的 JustFloat。首次使用时，在工程根目录创建虚拟环境并安装依赖：

```sh
python3 -m venv build/host-venv
build/host-venv/bin/python -m pip install -r tools/requirements.txt
```

每次使用时，接好 USB3，启动桥接并保持终端运行：

```sh
build/host-venv/bin/python tools/vofa_bridge.py --wave-file build/capture.csv
```

1. 桥接自动选择唯一的 CH340 串口。存在多个设备时，先用 `ls /dev/cu.usbserial*` 查看设备名，再加 `--port /dev/cu.usbserial-实际编号` 指定。
2. 打开 VOFA+，选择 **TCP 客户端**和 **JustFloat**，连接 **127.0.0.1:9000**，为波形控件绑定 CH1。
3. 单路默认横轴按 **5 µs/点**，包含插值。启动桥接时加 `--raw-points` 可只显示实收样点，此时按 **10 µs/点**。双路均按 **20 µs/点**。
4. 板上点 `CH1/2` 或发送 `CHAN 2` 开启双路显示，另发 `STREAM 2` 开启双路串流，在 VOFA+ 中绑定 CH1、CH2。也可用 `--channels 2` 启动桥接。回到单路时分别发送 `CHAN 1` 和 `STREAM 1`。

在桥接终端或 VOFA+ 发送区输入 ASCII 命令，以回车或换行结束。常用命令如下：

| 命令 | 用途 |
| --- | --- |
| `RUN` / `STOP` / `SINGLE` | 运行、冻结屏幕、武装单次捕获 |
| `MODE AUTO` / `MODE NORMAL` / `MODE SINGLE` | 设置触发模式 |
| `TRIG 1.5` / `EDGE RISE` / `EDGE FALL` | 设置触发电平和边沿 |
| `TB 500us` / `VDIV 500mV` | 设置时基和垂直档位 |
| `MEAS?` / `THD?` | 查询时域测量值或谐波与 THD |
| `CONF?` / `STAT?` | 查询配置、采样率、FPS 和错误计数 |
| `WAVE?` | 导出当前捕获的 240 点波形至 `build/capture.csv` |
| `STREAM OFF` | 停止向上位机连续发送波形 |
| `CAL` | 重新进行四点触摸校准 |

CH340C 重新打开串口时可能触发板上自动复位电路，演示期间保持桥接和串口连接。

## 代码结构

| 路径 | 职责 |
| --- | --- |
| [App/app.c](App/app.c) / [App/acquisition.c](App/acquisition.c) | 启动应用、主循环调度、消费 ADC DMA 半块、保存双路同步样点 |
| [App/scope.c](App/scope.c) | 触发判定、时基和量程、发布捕获帧、冻结及单次捕获 |
| [App/measurement.c](App/measurement.c) / [App/spectrum.c](App/spectrum.c) | 频率、峰峰值、直流分量、波形识别、相位差、FFT、谐波与 THD |
| [App/ui.c](App/ui.c) / [Screen/](Screen/) | 波形和频谱绘制、触摸交互、LCD 驱动及触摸校准 |
| [App/commands.c](App/commands.c) / [App/serial_link.c](App/serial_link.c) / [App/stream_encode.c](App/stream_encode.c) | 指令校验和分发、串口 DMA 收发、实时样点打包和串流编码 |
| [tools/vofa_bridge.py](tools/vofa_bridge.py) / [tools/](tools/) | VOFA+ 桥接、DSP 回归、实板串口和桥接测试 |
| [Core/](Core/) / [Third_Task.ioc](Third_Task.ioc) | CubeMX 生成的外设初始化、中断入口和配置 |

现场修改采样率时，同步调整 [Third_Task.ioc](Third_Task.ioc) 的 TIM2 周期和 [App/scope_config.h](App/scope_config.h) 的 `SCOPE_SAMPLE_HZ`，重新生成、构建并用 `STAT?` 核对。时基和手动垂直档位表位于 [App/scope.c](App/scope.c)，默认参数位于 `Scope_Init()`。

各功能所在文件、实现步骤和数据流见 [功能实现导读](docs/功能实现导读.md) 与 [架构说明](docs/架构.md)。
