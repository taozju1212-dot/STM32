/**
 * @file    RFID_Compat.c
 * @brief   RFID 模块指令兼容层
 */

#include "RFID_Compat.h"
#include "FSV9563.h"
#include "nfc.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#ifndef NULL
#define NULL    ((void*)0)
#endif

/*==================== 全局变量 ====================*/
static u8 S_RFID_ModuleAdr = 0x01;
static u8 DEFAULT_RFID_PWS[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
u8 u8_RFIDBusy = 0;
u8 u8_RFIDCurCmd = 0;
u8 s_RFID_Signel_Ch = 1;

/*==================== 通道切换表 ====================*/
#define PIN_PA5    GPIO_Pin_5
#define PIN_PA1    GPIO_Pin_1
#define PIN_PA15   GPIO_Pin_15
#define PIN_PB3    GPIO_Pin_3

const u8 Channel_Table[6][4] = {
    {1,   0,   0,   0},
    {1,   0,   1,   0},
    {1,   0,   0,   1},
    {0,   1,   0,   0},
    {0,   1,   1,   0},
    {0,   1,   0,   1}
};

/*==================== UART 接收缓冲区 ====================*/
static u8 UART_RxBuf[64];
static u8 UART_RxLen = 0;
static u8 UART_RxState = 0;
static u8 UART_FrameLen = 0;

/*==================== 辅助函数 ====================*/
u8 Get_RFIDCheck(u8* ptr, u16 len)
{
    u32 sum = 0;
    u16 i;
    if(ptr == NULL) return 0;
    for(i = 0; i < len; i++) sum += ptr[i];
    return (~sum) & 0xFF;
}

char Get_Module_Ch(u8 idx)
{
    switch(idx)
    {
        case 1:  return 0;
        case 3:  return 1;
        case 5:  return 2;
        case 7:  return 3;
        case 9:  return 4;
        case 11: return 5;
        default: return 100;
    }
}

void RFID_Switch_Channel(u8 internal_ch)
{
    if(internal_ch >= 6) return;
    GPIO_WriteBit(GPIOA, PIN_PA5,  (BitAction)Channel_Table[internal_ch][0]);
    GPIO_WriteBit(GPIOA, PIN_PA1,  (BitAction)Channel_Table[internal_ch][1]);
    GPIO_WriteBit(GPIOA, PIN_PA15, (BitAction)Channel_Table[internal_ch][2]);
    GPIO_WriteBit(GPIOB, PIN_PB3,  (BitAction)Channel_Table[internal_ch][3]);
}

/*==================== 发送响应 ====================*/
void RFID_SendResponse(u8 addr, u8 cmd, u8 sw, u8* data, u8 dataLen)
{
    u8 txBuf[64];
    u8 txLen, i;
    u8 frameLen = dataLen + 5;

    txBuf[0] = frameLen;
    txBuf[1] = addr;
    txBuf[2] = cmd;
    txBuf[3] = sw;

    if(data != NULL && dataLen > 0 && dataLen <= 55)
    {
        for(i = 0; i < dataLen; i++)
            txBuf[4 + i] = data[i];
    }

    txBuf[4 + dataLen] = Get_RFIDCheck(&txBuf[0], 4 + dataLen);
    txLen = 5 + dataLen;

    for(i = 0; i < txLen; i++)
    {
        USART_SendData(USART1, txBuf[i]);
        while((USART1->SR & USART_FLAG_TXE) == RESET);
    }
    while((USART1->SR & USART_FLAG_TC) == RESET);
}

/*==================== 命令处理 ====================*/

/**
 * @brief  获取模块信息 0x15
 */
void RFID_Cmd_GetInfo(void)
{
    const u8 data[29] = {
        0x43, 0x55, 0x35, 0x30, 0x30, 0x2D, 0x31, 0x32,
        0x41, 0x4E, 0x54, 0x20, 0x56, 0x31, 0x2E, 0x32,
        0x33, 0x20, 0x32, 0x30, 0x32, 0x34, 0x2D, 0x30,
        0x37, 0x2D, 0x31, 0x32, 0x00
    };
    RFID_SendResponse(S_RFID_ModuleAdr, 0x15, 0x00, (u8*)data, 29);
}

/**
 * @brief  选择通道 0x01
 */
void RFID_Cmd_SelectChannel(u8* rxBuf, u8 rxLen)
{
    u8 ch, resp[1];
    char internal_ch;

    if(rxLen < 1) {
        RFID_SendResponse(S_RFID_ModuleAdr, 0x01, 0x01, NULL, 0);
        return;
    }

    ch = rxBuf[3];
    internal_ch = Get_Module_Ch(ch);

    if(internal_ch == 100) {
        RFID_SendResponse(S_RFID_ModuleAdr, 0x01, 0x01, NULL, 0);
        return;
    }

    RFID_Switch_Channel(internal_ch);
    s_RFID_Signel_Ch = ch;
    
    // 短暂延时等待芯片稳定
    {
        volatile int i;
        for(i = 0; i < 10000; i++);
    }

    resp[0] = ch;
    RFID_SendResponse(S_RFID_ModuleAdr, 0x01, 0x00, resp, 1);
}

/**
 * @brief  获取地址 0x11
 */
void RFID_Cmd_GetAddr(void)
{
    u8 resp[1];
    resp[0] = S_RFID_ModuleAdr;
    /* 控制板在响应帧中查找 S_RFID_ModuleAdr (0x01) 字节作为帧头标识，
     * 必须用模块地址而不是 0x00，否则控制板无法定位响应帧 */
    RFID_SendResponse(S_RFID_ModuleAdr, 0x11, 0x00, resp, 1);
}

/**
 * @brief  设置地址 0x12
 */
void RFID_Cmd_SetAddr(u8* rxBuf, u8 rxLen)
{
    u8 newAdr;
    if(rxLen < 1) {
        RFID_SendResponse(S_RFID_ModuleAdr, 0x12, 0x01, NULL, 0);
        return;
    }
    newAdr = rxBuf[3];
    /* 控制板用旧地址 (0x01) 匹配响应帧头，必须先用旧地址发出响应，
     * 控制板收到成功响应后才会在本地更新自己记录的模块地址，
     * 之后再修改本机地址，双方保持同步 */
    RFID_SendResponse(S_RFID_ModuleAdr, 0x12, 0x00, NULL, 0);
    S_RFID_ModuleAdr = newAdr;
}

/**
 * @brief  自动扫描 0x02
 *         响应格式：LEN ID FC SW DATA(n) CHECK
 *         DATA: 总卡数 (1) + N*[UID(4) + 通道号 (1)]
 *         无卡时返回：05 01 02 03 F4 (SW=0x03 表示无卡)
 */
void RFID_Cmd_AutoScan(u8* rxBuf, u8 rxLen)
{
    u8 cardInfo[60];
    memset(cardInfo, 0, sizeof(cardInfo));

    (void)rxLen;

    /* 控制板发送 2 字节通道掩码，bit(外部通道号-1) = 1 表示扫描该通道：
     *   mask1 (rxBuf[3]) bit0,2,4,6 → 外部通道 1,3,5,7  → 内部通道 0,1,2,3
     *   mask2 (rxBuf[4]) bit0,2     → 外部通道 9,11      → 内部通道 4,5
     * 例：0x55 0x15 → 全部 6 个外部通道（1/3/5/7/9/11）均选中 */
    u8 mask1 = rxBuf[3];
    u8 mask2 = rxBuf[4];

    /* 外部通道号表：内部通道 i → 外部通道号 channelMap[i] */
    const u8 channelMap[6] = {1, 3, 5, 7, 9, 11};

    u8 cardCount = 0;
    u8 i;

    for(i = 0; i < 6; i++)
    {
        /* 取出该内部通道对应的掩码位（偶数位映射） */
        u8 mByte = (i < 4) ? mask1 : mask2;
        u8 mBit  = (i < 4) ? (u8)(i * 2) : (u8)((i - 4) * 2);

        if((mByte & (1 << mBit)) == 0)
            continue;   /* 该通道未选中，跳过 */

        /* FSV9563_SingleChannel_Read 内部已完整处理：
         * 通道切换(delay_ms) → PDOWN复位(delay_ms) → ConfigISOType(delay_ms) → 激活卡
         * 无需在此重复做，直接调用即可 */
        {
            u8 uid[12], uid_len, card_type = 0, card_data[16];

            FSV9563_SingleChannel_Read(i, &card_type, uid, &uid_len, card_data);

            /* AutoScan 只需判断卡片是否存在（激活成功即可）
             * 不要求数据读取/鉴权成功，card_type==1 表示 TypeA 卡已被激活 */
            if(card_type == 1)
            {
                u8 extChannel = channelMap[i];

                /* 填入响应数据：UID(4字节) + 通道号(1字节) */
                cardInfo[1 + cardCount * 5] = uid[0];
                cardInfo[2 + cardCount * 5] = uid[1];
                cardInfo[3 + cardCount * 5] = uid[2];
                cardInfo[4 + cardCount * 5] = uid[3];
                cardInfo[5 + cardCount * 5] = extChannel;

                cardCount++;
            }
        }
    }
    
    // 发送响应
    cardInfo[0] = cardCount;
    
    if(cardCount > 0)
    {
        // 有卡：SW=0x00
        RFID_SendResponse(S_RFID_ModuleAdr, 0x02, 0x00, cardInfo, 1 + cardCount * 5);
    }
    else
    {
        // 无卡：SW=0x03
        RFID_SendResponse(S_RFID_ModuleAdr, 0x02, 0x03, NULL, 0);
    }
}

/**
 * @brief  本地辅助：切换到当前选定通道，复位芯片，激活卡并获取 UID
 * @param  uid: 输出，至少 12 字节
 * @retval MI_OK=卡已激活，其他=无卡或错误
 */
static s8 RFID_ActivateCard(u8 *uid)
{
    u8 atqa[2], uid_len, sak;
    char internal_ch = Get_Module_Ch(s_RFID_Signel_Ch);
    if(internal_ch == 100) return (s8)MI_NOTAGERR;

    RFID_Switch_Channel((u8)internal_ch);
    delay_ms(5);

    PDOWN_1;
    delay_ms(1);
    PDOWN_0;
    delay_ms(5);

    FSV9563_PcdConfigISOType('A');
    delay_ms(10);

    return FSV9563_PcdActivateA(atqa, uid, &uid_len, &sak);
}

/**
 * @brief  读块 0x21
 *         控制板帧：0C 01 21 [sector] [block] [key×6] [BCC]
 *         成功响应：15 01 21 00 [data×16] [BCC]
 */
void RFID_Cmd_ReadBlock(u8* rxBuf, u8 rxLen)
{
    u8 sectorIdx, blockIdx, key[6], uid[12], card_data[16];
    u8 blockAddr;
    s8 status;

    if(rxLen < 8) {
        RFID_SendResponse(S_RFID_ModuleAdr, 0x21, 0x05, NULL, 0);
        return;
    }

    sectorIdx = rxBuf[3];
    blockIdx  = rxBuf[4];
    memcpy(key, &rxBuf[5], 6);

    if(sectorIdx > 15 || blockIdx > 3) {
        RFID_SendResponse(S_RFID_ModuleAdr, 0x21, 0x05, NULL, 0);
        return;
    }

    blockAddr = (u8)(sectorIdx * 4 + blockIdx);

    /* 激活卡（含通道切换 + 芯片复位） */
    status = RFID_ActivateCard(uid);
    if(status != MI_OK) {
        RFID_SendResponse(S_RFID_ModuleAdr, 0x21, 0x03, NULL, 0);
        return;
    }

    /* 用控制板传来的密钥鉴权该扇区（FSV9563 须先 LoadKey 再 MFAuthent） */
    status = FSV9563_CMD_LoadKey(key);
    if(status != MI_OK) {
        RFID_SendResponse(S_RFID_ModuleAdr, 0x21, 0x04, NULL, 0);
        return;
    }
    status = FSV9563_CMD_MfcAuthenticate(0x60, (u8)(sectorIdx * 4), uid);
    if(status != MI_OK) {
        RFID_SendResponse(S_RFID_ModuleAdr, 0x21, 0x04, NULL, 0);
        return;
    }

    /* 读目标块 */
    status = FSV9563_PcdMfcRead(blockAddr, card_data);
    if(status == MI_OK) {
        RFID_SendResponse(S_RFID_ModuleAdr, 0x21, 0x00, card_data, 16);
    } else {
        RFID_SendResponse(S_RFID_ModuleAdr, 0x21, 0x05, NULL, 0);
    }
}

/**
 * @brief  写块 0x22
 *         控制板帧：1C 01 22 [sector] [block] [key×6] [data×16] [BCC]
 *         成功响应：05 01 22 00 [BCC]
 */
void RFID_Cmd_WriteBlock(u8* rxBuf, u8 rxLen)
{
    u8 sectorIdx, blockIdx, key[6], uid[12];
    u8 blockAddr;
    s8 status;

    if(rxLen < 24) {
        RFID_SendResponse(S_RFID_ModuleAdr, 0x22, 0x06, NULL, 0);
        return;
    }

    sectorIdx = rxBuf[3];
    blockIdx  = rxBuf[4];
    memcpy(key, &rxBuf[5], 6);

    /* 禁止写扇区0块0（制造商数据）和扇区尾块（块3） */
    if((sectorIdx == 0 && blockIdx == 0) || blockIdx == 3 || sectorIdx > 15) {
        RFID_SendResponse(S_RFID_ModuleAdr, 0x22, 0x06, NULL, 0);
        return;
    }

    blockAddr = (u8)(sectorIdx * 4 + blockIdx);

    /* 激活卡 */
    status = RFID_ActivateCard(uid);
    if(status != MI_OK) {
        RFID_SendResponse(S_RFID_ModuleAdr, 0x22, 0x03, NULL, 0);
        return;
    }

    /* 用控制板传来的密钥鉴权（FSV9563 须先 LoadKey 再 MFAuthent） */
    status = FSV9563_CMD_LoadKey(key);
    if(status != MI_OK) {
        RFID_SendResponse(S_RFID_ModuleAdr, 0x22, 0x04, NULL, 0);
        return;
    }
    status = FSV9563_CMD_MfcAuthenticate(0x60, (u8)(sectorIdx * 4), uid);
    if(status != MI_OK) {
        RFID_SendResponse(S_RFID_ModuleAdr, 0x22, 0x04, NULL, 0);
        return;
    }

    /* 写目标块，数据在 rxBuf[11..26] */
    status = FSV9563_PcdMfcWrite(blockAddr, &rxBuf[11]);
    if(status == MI_OK) {
        RFID_SendResponse(S_RFID_ModuleAdr, 0x22, 0x00, NULL, 0);
    } else {
        RFID_SendResponse(S_RFID_ModuleAdr, 0x22, 0x05, NULL, 0);
    }
}

/**
 * @brief  读扇区 0x25
 *         控制板帧：0B 01 25 [sector] [key×6] [BCC]
 *         成功响应：35 01 25 00 [data×48] [BCC]（block 0/1/2，跳过扇区尾块）
 */
void RFID_Cmd_ReadSector(u8* rxBuf, u8 rxLen)
{
    u8 sectorIdx, key[6], uid[12], card_data[48];
    u8 b;
    s8 status;

    if(rxLen < 7) {
        RFID_SendResponse(S_RFID_ModuleAdr, 0x25, 0x05, NULL, 0);
        return;
    }

    sectorIdx = rxBuf[3];
    memcpy(key, &rxBuf[4], 6);

    if(sectorIdx > 15) {
        RFID_SendResponse(S_RFID_ModuleAdr, 0x25, 0x05, NULL, 0);
        return;
    }

    /* 激活卡 */
    status = RFID_ActivateCard(uid);
    if(status != MI_OK) {
        RFID_SendResponse(S_RFID_ModuleAdr, 0x25, 0x03, NULL, 0);
        return;
    }

    /* 用控制板传来的密钥鉴权（FSV9563 须先 LoadKey 再 MFAuthent） */
    status = FSV9563_CMD_LoadKey(key);
    if(status != MI_OK) {
        RFID_SendResponse(S_RFID_ModuleAdr, 0x25, 0x04, NULL, 0);
        return;
    }
    status = FSV9563_CMD_MfcAuthenticate(0x60, (u8)(sectorIdx * 4), uid);
    if(status != MI_OK) {
        RFID_SendResponse(S_RFID_ModuleAdr, 0x25, 0x04, NULL, 0);
        return;
    }

    /* 依次读扇区3个数据块（block 0/1/2），每块16字节，共48字节 */
    for(b = 0; b < 3; b++) {
        status = FSV9563_PcdMfcRead((u8)(sectorIdx * 4 + b), &card_data[b * 16]);
        if(status != MI_OK) {
            RFID_SendResponse(S_RFID_ModuleAdr, 0x25, 0x05, NULL, 0);
            return;
        }
    }

    RFID_SendResponse(S_RFID_ModuleAdr, 0x25, 0x00, card_data, 48);
}

/**
 * @brief  写扇区 0x27
 *         控制板帧：3D 01 27 [sector] FF 0A [key×6] [data×48] [BCC]
 *         成功响应：05 01 27 00 [BCC]（写 block 0/1/2，跳过扇区尾块）
 */
void RFID_Cmd_WriteSector(u8* rxBuf, u8 rxLen)
{
    u8 sectorIdx, key[6], uid[12];
    u8 b;
    s8 status;

    /* rxBuf[4]=0xFF, rxBuf[5]=0x0A 为控制板固定字段，key 从 rxBuf[6] 开始 */
    if(rxLen < 57) {
        RFID_SendResponse(S_RFID_ModuleAdr, 0x27, 0x06, NULL, 0);
        return;
    }

    sectorIdx = rxBuf[3];
    memcpy(key, &rxBuf[6], 6);

    /* 禁止写扇区0（含制造商块） */
    if(sectorIdx == 0 || sectorIdx > 15) {
        RFID_SendResponse(S_RFID_ModuleAdr, 0x27, 0x06, NULL, 0);
        return;
    }

    /* 激活卡 */
    status = RFID_ActivateCard(uid);
    if(status != MI_OK) {
        RFID_SendResponse(S_RFID_ModuleAdr, 0x27, 0x03, NULL, 0);
        return;
    }

    /* 用控制板传来的密钥鉴权（FSV9563 须先 LoadKey 再 MFAuthent） */
    status = FSV9563_CMD_LoadKey(key);
    if(status != MI_OK) {
        RFID_SendResponse(S_RFID_ModuleAdr, 0x27, 0x04, NULL, 0);
        return;
    }
    status = FSV9563_CMD_MfcAuthenticate(0x60, (u8)(sectorIdx * 4), uid);
    if(status != MI_OK) {
        RFID_SendResponse(S_RFID_ModuleAdr, 0x27, 0x04, NULL, 0);
        return;
    }

    /* 依次写扇区3个数据块（block 0/1/2），数据从 rxBuf[12] 起，每块16字节 */
    for(b = 0; b < 3; b++) {
        status = FSV9563_PcdMfcWrite((u8)(sectorIdx * 4 + b), &rxBuf[12 + b * 16]);
        if(status != MI_OK) {
            RFID_SendResponse(S_RFID_ModuleAdr, 0x27, 0x05, NULL, 0);
            return;
        }
    }

    RFID_SendResponse(S_RFID_ModuleAdr, 0x27, 0x00, NULL, 0);
}

/**
 * @brief  串口调试命令 0xD1 — 写卡+读回验证
 *
 * 发送帧（27字节）：
 *   1B 01 D1 [sector] [block] [key×6] [data×16] [BCC]
 *
 * 响应帧（38字节）：
 *   26 01 D1 [step_code] [written×16] [readback×16] [BCC]
 *
 * step_code:
 *   0x00 成功且数据匹配
 *   0x01 写阶段激活失败（无卡）
 *   0x02 写阶段鉴权失败
 *   0x03 写操作失败
 *   0x04 读阶段重新激活失败
 *   0x05 读阶段鉴权失败
 *   0x06 读操作失败
 *   0x07 写读均成功但数据不一致
 *   0x08 参数非法（保护块）
 */
static void RFID_Cmd_WriteReadTest(u8* rxBuf, u8 rxLen)
{
    u8 sectorIdx, blockIdx, key[6];
    u8 uid[12], blockAddr;
    u8 written[16], readback[16];
    u8 resp[33];   /* step_code(1) + written(16) + readback(16) */
    s8 status;
    u8 b;

    memset(written,  0, sizeof(written));
    memset(readback, 0, sizeof(readback));

    /* 帧长检查：LEN(1)+ADDR(1)+CMD(1)+sector(1)+block(1)+key(6)+data(16)+BCC(1) = 28
     * RFID_ProcessCommand 传入 rxLen = frame_len-4 = 28-4 = 24 */
    if(rxLen < 24) {
        resp[0] = 0x08;
        memcpy(&resp[1], written, 16);
        memcpy(&resp[17], readback, 16);
        RFID_SendResponse(S_RFID_ModuleAdr, 0xD1, resp[0], &resp[1], 32);
        return;
    }

    sectorIdx = rxBuf[3];
    blockIdx  = rxBuf[4];
    memcpy(key,     &rxBuf[5],  6);
    memcpy(written, &rxBuf[11], 16);

    /* 保护块检查 */
    if((sectorIdx == 0 && blockIdx == 0) || blockIdx == 3 || sectorIdx > 15) {
        resp[0] = 0x08;
        memcpy(&resp[1], written, 16);
        memcpy(&resp[17], readback, 16);
        RFID_SendResponse(S_RFID_ModuleAdr, 0xD1, resp[0], &resp[1], 32);
        return;
    }

    blockAddr = (u8)(sectorIdx * 4 + blockIdx);

    /* ── 写阶段 ── */
    status = RFID_ActivateCard(uid);
    if(status != MI_OK) {
        resp[0] = 0x01;
        memcpy(&resp[1], written, 16); memcpy(&resp[17], readback, 16);
        RFID_SendResponse(S_RFID_ModuleAdr, 0xD1, resp[0], &resp[1], 32);
        return;
    }

    status = FSV9563_CMD_LoadKey(key);
    if(status == MI_OK)
        status = FSV9563_CMD_MfcAuthenticate(0x60, (u8)(sectorIdx * 4), uid);
    if(status != MI_OK) {
        resp[0] = 0x02;
        memcpy(&resp[1], written, 16); memcpy(&resp[17], readback, 16);
        RFID_SendResponse(S_RFID_ModuleAdr, 0xD1, resp[0], &resp[1], 32);
        return;
    }

    status = FSV9563_PcdMfcWrite(blockAddr, written);
    if(status != MI_OK) {
        resp[0] = 0x03;
        memcpy(&resp[1], written, 16); memcpy(&resp[17], readback, 16);
        RFID_SendResponse(S_RFID_ModuleAdr, 0xD1, resp[0], &resp[1], 32);
        return;
    }

    /* ── 读回阶段（完整重新激活，确保不依赖上次会话状态） ── */
    status = RFID_ActivateCard(uid);
    if(status != MI_OK) {
        resp[0] = 0x04;
        memcpy(&resp[1], written, 16); memcpy(&resp[17], readback, 16);
        RFID_SendResponse(S_RFID_ModuleAdr, 0xD1, resp[0], &resp[1], 32);
        return;
    }

    status = FSV9563_CMD_LoadKey(key);
    if(status == MI_OK)
        status = FSV9563_CMD_MfcAuthenticate(0x60, (u8)(sectorIdx * 4), uid);
    if(status != MI_OK) {
        resp[0] = 0x05;
        memcpy(&resp[1], written, 16); memcpy(&resp[17], readback, 16);
        RFID_SendResponse(S_RFID_ModuleAdr, 0xD1, resp[0], &resp[1], 32);
        return;
    }

    status = FSV9563_PcdMfcRead(blockAddr, readback);
    if(status != MI_OK) {
        resp[0] = 0x06;
        memcpy(&resp[1], written, 16); memcpy(&resp[17], readback, 16);
        RFID_SendResponse(S_RFID_ModuleAdr, 0xD1, resp[0], &resp[1], 32);
        return;
    }

    /* ── 对比 ── */
    resp[0] = 0x00;
    for(b = 0; b < 16; b++) {
        if(written[b] != readback[b]) { resp[0] = 0x07; break; }
    }
    memcpy(&resp[1], written, 16);
    memcpy(&resp[17], readback, 16);
    RFID_SendResponse(S_RFID_ModuleAdr, 0xD1, resp[0], &resp[1], 32);
}

/**
 * @brief  命令处理入口
 */
void RFID_ProcessCommand(u8* rxBuf, u8 rxLen)
{
    u8 cmd = rxBuf[2];

    u8_RFIDCurCmd = cmd;
    u8_RFIDBusy = 1;

    switch(cmd)
    {
        case 0x15:  RFID_Cmd_GetInfo(); break;
        case 0x01:  RFID_Cmd_SelectChannel(rxBuf, rxLen - 4); break;
        case 0x11:  RFID_Cmd_GetAddr(); break;
        case 0x12:  RFID_Cmd_SetAddr(rxBuf, rxLen - 4); break;
        case 0x02:  RFID_Cmd_AutoScan(rxBuf, rxLen - 4); break;
        case 0x21:  RFID_Cmd_ReadBlock(rxBuf, rxLen - 4); break;
        case 0x22:  RFID_Cmd_WriteBlock(rxBuf, rxLen - 4); break;
        case 0x25:  RFID_Cmd_ReadSector(rxBuf, rxLen - 4); break;
        case 0x27:  RFID_Cmd_WriteSector(rxBuf, rxLen - 4); break;
        case 0xD1:  RFID_Cmd_WriteReadTest(rxBuf, rxLen - 4); break;
        default:
            RFID_SendResponse(S_RFID_ModuleAdr, cmd, 0xFF, NULL, 0);
            break;
    }

    u8_RFIDBusy = 0;
}

/*==================== UART 接收分析 ====================*/
BOOL RFID_RespondAnalysis(u8 data)
{
    switch(UART_RxState)
    {
        case 0:
            UART_RxLen = 0;
            /* 上限从 60 调整为 63：控制板 WriteSector 帧长 = 61 (0x3D)，
             * 超出旧上限 60 会被丢弃导致写扇区永远失败；
             * 接收缓冲区 UART_RxBuf[64] 可容纳最大 63 字节帧（索引 0-62）*/
            if(data >= 4 && data <= 63)
            {
                UART_RxBuf[0] = data;
                UART_FrameLen = data;
                UART_RxLen = 1;
                UART_RxState = 1;
            }
            return FALSE;

        case 1:
            if(UART_RxLen < 64)
            {
                UART_RxBuf[UART_RxLen++] = data;
            }
            if(UART_RxLen >= UART_FrameLen)
            {
                u8 receivedCheck = UART_RxBuf[UART_RxLen - 1];
                u8 calcCheck = Get_RFIDCheck(&UART_RxBuf[0], UART_RxLen - 1);

                UART_RxState = 0;
                UART_RxLen = 0;
                UART_FrameLen = 0;

                if(receivedCheck == calcCheck)
                {
                    RFID_ProcessCommand(UART_RxBuf, UART_FrameLen);
                    return TRUE;
                }
                return FALSE;
            }
            return FALSE;

        default:
            UART_RxState = 0;
            UART_RxLen = 0;
            UART_FrameLen = 0;
            return FALSE;
    }
}

void RFID_Var_Init(void)
{
    S_RFID_ModuleAdr = 0x01;
    memset(DEFAULT_RFID_PWS, 0xFF, 6);
    UART_RxState = 0;
    UART_RxLen = 0;
    UART_FrameLen = 0;
    u8_RFIDBusy = 0;
    u8_RFIDCurCmd = 0;
}

void RFID_Compat_Init(void)
{
    RFID_Var_Init();
    FSV9563_Init();  // 恢复 FSV9563 初始化
   // RFID_Switch_Channel(0);
}
