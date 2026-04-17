#include "stm32f10x.h"
#include "FSV9563.h"
#include "RFID_Compat.h"

void System_Init(void);
void delay_init(u8 SYSCLK);

/* 环形缓冲区：中断写入，主循环读取，彻底避免处理命令期间丢字节 */
#define UART_RX_BUF_SIZE  128
volatile u8  g_rxBuf[UART_RX_BUF_SIZE];
volatile u8  g_rxHead = 0;   /* 中断写入位置 */
volatile u8  g_rxTail = 0;   /* 主循环读取位置 */

int main()
{
    // 第一步：初始化 UART
    System_Init();

    // 第二步：初始化 RFID 兼容层
    RFID_Compat_Init();

    // 第三步：使能全局中断
    __enable_irq();

    while(1)
    {
        /* 逐字节取出环形缓冲区中的数据并送入帧解析器
         * 即使 RFID_RespondAnalysis 内部耗时（SPI 操作），
         * 中断仍会持续向 g_rxBuf 写入，不会丢失任何字节 */
        while(g_rxHead != g_rxTail)
        {
            u8 data = g_rxBuf[g_rxTail];
            g_rxTail = (g_rxTail + 1) % UART_RX_BUF_SIZE;
            RFID_RespondAnalysis(data);
        }
    }
}

void System_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    // 初始化系统时钟
    SystemInit();
    
    // 初始化延时函数 (72MHz)
    delay_init(72);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO, ENABLE);

    // UART1 TX/RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // UART - 115200 @ 72MHz
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    // NVIC
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 使能 UART 中断
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    // 使能 USART
    USART_Cmd(USART1, ENABLE);

    // 清除标志位
    USART_ClearFlag(USART1, USART_FLAG_TC | USART_FLAG_RXNE);
}

void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        u8 data = (u8)USART_ReceiveData(USART1);
        u8 nextHead = (g_rxHead + 1) % UART_RX_BUF_SIZE;
        if(nextHead != g_rxTail)   /* 缓冲区未满才写入，防止覆盖未读数据 */
        {
            g_rxBuf[g_rxHead] = data;
            g_rxHead = nextHead;
        }
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}
