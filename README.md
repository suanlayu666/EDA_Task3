# KF56-10 触摸屏示波器

`Third_Task.ioc` 为 STM32CubeMX 工程。在 VS Code 的 STM32 扩展或 Cube 工具链环境中，选择 `Release` 后编译：

```sh
cmake --preset Release
cmake --build --preset Release
```

烧录 `build/Release/Third_Task.elf`，使用 STM32CubeProgrammer 的 ST-LINK / SWD、下载并校验。代码结构参见 [架构.md](docs/架构.md)，各功能的源码入口和运行逻辑见 [功能实现导读.md](docs/功能实现导读.md)。

## 接线与首次联调

- 单通道接 `P8-29 PA5`，CH2 接 **`P8-32 PA6`**，发生器与开发板共地。外部信号接入 PA5 前断开 J4 上连往 `STM_ADC` 的跳帽，PA6 做采集时拔掉摄像头。改回电位器前先拔掉外部信号，再接回 `R_ADC ↔ STM_ADC` 跳帽。[各功能引脚和逐项验收](docs/引脚与功能测试.md)
- 接线前实际检查信号在 **0～3.3 V**。首次建议用 **1 kHz、1 Vpp、1.5 V 偏置**的正弦波。
- USB 线连板上 CH340C 串口，P12 的 USART1 跳帽维持连通。开串口可能因板上的自动复位电路短暂复位 MCU，正式一次上电演示前把桥接和 VOFA+ 都连好，演示过程保持串口连接。

## Mac 上的 VOFA+

工程自带桥接，STM32 通过板载 CH340C 串口以 1.2 Mbaud 把紧凑二进制发到 Mac，桥接在本地转成 VOFA+ 的 JustFloat。桥接仅监听本机，VOFA+ 使用官方自带协议。

```sh
python3 -m venv build/host-venv
build/host-venv/bin/python -m pip install -r tools/requirements.txt
build/host-venv/bin/python tools/vofa_bridge.py --wave-file build/capture.csv
```

在 VOFA+ 选择 **TCP 客户端**、`127.0.0.1`、端口 `9000`、协议 **JustFloat**，绑定第一路到波形控件。单路串口实收样本间隔是 **10 µs**。桥接默认在相邻点间补一个插值点，VOFA+ 单路横轴按 **5 µs**，可用 `--raw-points` 查看 **10 µs** 的实收点。双路串流按 **20 µs**。若串口设备不止一个，用 `--port /dev/cu.usbserial-1130` 指定实际设备名。`--channels 2` 打开双路实时串流。

命令可以在桥接终端输入，也可从 VOFA+ 的发送区以 ASCII 加换行发送。VOFA+ 的字节接收区和桥接终端都能收到 `OK`、`ERR` 和测量应答。在字节接收区设置中可隐藏采样帧以便查看应答。`WAVE?` 回显帧号和点数，桥接可将冻结帧或当前帧写成 CSV。波形部分快速持续发送，LCD 的 STOP 和 SINGLE 控制捕获帧，`STREAM OFF` 独立停止持续串流。

| 命令 | 作用 |
|---|---|
| `RUN`、`STOP`、`SINGLE` | 恢复、冻结或重新武装单次捕获。单次取触发后的样本测量，再冻结当前画面和读数。 |
| `TRIG 1.5`、`EDGE RISE`、`EDGE FALL` | 触发电平与穿越方向 |
| `TB 500us`、`VDIV 500mV` | 设置每格时间、每格电压。Time 切档在运行或武装态执行 |
| `MEAS?`、`WAVE?`、`THD?` | 读屏幕捕获的测量结果、时域样本和频谱 |
| `MODE AUTO`、`MODE NORMAL`、`MODE SINGLE` | 触发模式 |
| `CHAN 2`、`STREAM 2` | 分别打开 LCD 双路和串口双路 |
| `RANGE AUTO`、`RANGE MANUAL` | 自动和手动量程 |
| `PAGE FFT`、`PAGE SCOPE` | 频谱和时域页 |
| `CONF?`、`STAT?` | 当前配置、采样和丢块计数、屏幕型号和帧率。`LCDSYNC` 表示已读到连续扫描，`LCDMISS` 统计错过写屏窗口的帧数。 |
| `VREF 3.27`、`CAL` | 设置实测 ADC 参考电压或重新做触摸校准 |

屏幕 `LEVEL` 点亮后，在绘图区上下拖动设触发电平。再点 `LEVEL` 退出调整。停止以后拖动最近的竖线，读两线间的 `dT` 和 `1/dT`。测频时移到相邻周期的相同位置，先选能在画面上拉开两线的时基。电阻屏首次弹出四点校准时，依次点亮并松开十字。配置保存在板载 EEPROM。

## 回归和现场检查

```sh
cc -std=c11 -O2 -Wall -Wextra -Werror -I App tools/verify_dsp.c \
   App/measurement.c App/spectrum.c App/scope.c App/stream_encode.c -lm -o build/verify_dsp
build/verify_dsp
build/host-venv/bin/python tools/verify_board.py --seconds 30
build/host-venv/bin/python tools/verify_bridge.py --seconds 15
```

运行实板脚本前关闭桥接，确保串口由一个进程打开。DSP 回归使用量化合成输入；实板脚本校验串口命令、冻结快照、正常档无触发保持、持续吞吐和固件状态。

现场再用发生器和触摸笔做下列检查：

1. 单路检查 100 Hz 与 20 kHz 连续波形。频率 1–5 kHz 逐点测（误差 ≤3%），峰峰值和直流分量测幅值两端（分别 ≤5%）。读数应在信号变化后 2 秒内更新。
2. 打乱输入正弦、方波和三角波，分别在 1、2、5 kHz 看识别结果。
3. 低频方波检查对齐、可见撕裂、刷新、改频响应，验证 Auto/Normal/Single、触摸切档、阈值和双游标（换算频率误差 ≤5%）。改频后 1 秒内重新稳定。
4. 双路接同频信号，检查同屏与各路测量，再逐点测试相位差。CH2 超前 CH1 显示正角；跨零按圆周误差计算，目标 ≤5°。
5. 用已知谐波含量的信号验证 CH1 频谱和 THD，分别在 1–5 kHz 检查。改变幅度到 0.1–3.0 Vpp，检查波形占高 60%–80% 和换挡稳定性。

## 本机验证记录

- Debug 和 Release 编译通过。Release 已通过 ST-LINK 烧录校验。[CubeMX 重生成日志](build/ioc-verify.log)记录 ADC、DMA、TIM2 和 USART1 配置检查。
- 160 kHz 改参演练通过 CubeMX 生成、CMake 编译、ST-LINK 校验和实板单双路串流检查。测试脚本用 `STAT?` 的采样数增量核对 ADC 频率。[改参演练记录](build/rate-drill.log)
- [实板串口记录](build/board-verify.log)覆盖八组指令、参数不变性、超长输入、冻结快照、单双路持续传输，以及时域和频谱页面切换时的 LCD 同步状态。
- [最长时基双路记录](build/long-window-verify.log)检查 5 ms/格下的 50 ms 双路捕获、连续串流和 240 点 CSV。冻结帧采用双路 12 位无损打包，普通 RAM 链接用量为 116,992 B（89.26%）。
- [实板主栈读回](build/stack-verify.log)覆盖双路 50 ms 连续串流、21 次快照和成批查询。在该负载下，4,096 B 预留栈中最深写到 1,672 B，仍有 2,424 B 保留启动时写入的标记。测试后恢复并校验正式 Release。
- [同一次上电的联调记录](build/one-boot-verify.log)覆盖冻结、超长指令、正常档保持、单次武装、单路→双路→单路切换，以及 32 批共 256 条非法指令。全程约 142 秒，MCU 运行时间连续增长，桥接没有留下样点缺口，发送丢包计数保持为零。[ST-LINK 在线读回](build/live-registers-verify.log)核实了 TIM2 触发、ADC 双规则同时转换和 32 位循环 DMA。
- [LCD 像素读回记录](build/lcd-readback.log)中的四个 `0x18` 各代表一种颜色的 24 个像素通过读回校验。
- [桥接记录](build/bridge-verify.log)覆盖实际 CH340C 到本地 JustFloat 的数据和指令往返，包括无换行的超长输入。[恢复算法记录](build/recovery-verify.log)覆盖连续 1–12 包缺失和本机测试中实际出现的非连续缺包组合。
- [实板合成输入记录](build/target-fixture-verify.log)覆盖 CH340 串流的 100 Hz、20 kHz 两端，双路 1–5 kHz 测量、波形识别、相位、触发、THD 和自动量程。临时固件从 DMA 取块后注入已知样本。两张图是当时从板载 LCD 显存读回的完整画面。[双路时域](build/lcd-scope.png) [FFT](build/lcd-fft.png)
- [算法回归](build/dsp-verify.log)、[波形传输仿真](build/waveform-verify.log)和[触摸逻辑仿真](build/ui-sim-verify.log)通过。现场接入发生器后的精度、实屏触摸和可见撕裂仍按上方清单验收。
