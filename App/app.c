#include "app.h"
#include "acquisition.h"
#include "commands.h"
#include "scope.h"
#include "serial_link.h"
#include "ui.h"
#include "main.h"

/* 初始化应用，LCD 完成启动后再开始连续采样。 */
void App_Init(void)
{
    Scope_Init();
    Ui_Init();
    Serial_Init();
    Acquisition_Start();
    Serial_SendText("OK KF56-10 SCOPE READY");
}

/* 优先消费 DMA，再服务串口、测量和一小块屏幕。 */
void App_Poll(void)
{
    uint32_t now = HAL_GetTick();
    SampleBlock block;
    if (Acquisition_Poll(&block)) {
        Scope_Consume(&block);
        Serial_StreamBlock(&block);
    }
    Serial_Poll(now, Commands_Execute);
    Scope_Poll(now);
    Ui_Poll(now);
}
