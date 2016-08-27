/*===========================================================================
* 网址 ：http://yhmcu.taobao.com/   http://www.cdebyte.com/                 *
* 作者 ：李勇  原 亿和电子工作室  现 亿佰特电子科技有限公司                 * 
* 邮件 ：yihe_liyong@126.com                                                *
* 电话 ：18615799380                                                        *
============================================================================*/

#ifndef _nRF24NRF24L01_H_
#define _nRF24NRF24L01_H_

#include "sys.h"
#include "NRF24L01_Reg.h"
// 常量定义
#define TX              1       // 发送模式
#define RX              0       // 接收模式
#define REPEAT_CNT      15  // 0-15, repeat transmit count




#define NRF24L01_CSN_LOW()   (PGout(7)=0)
#define NRF24L01_CSN_HIGH()  (PGout(7)=1)

#define NRF24L01_CE_LOW()    (PGout(6)=0)
#define NRF24L01_CE_HIGH()   (PGout(6)=1)

#define NRF24L01_IRQ_READ()  PGin(8)

// nRF24NRF24L01P相关函数接口
// 初始化NRF24L01
void NRF24L01_Init(uint8_t mode);

// 复位TX FIFO指针      
void NRF24L01_FlushTX(void);

// 复位RX FIFO指针     
void NRF24L01_FlushRX(void);     

// 读取中断
uint8_t NRF24L01_ReadIRQSource(void);          

// 清除中断
#define IRQ_ALL  ((1<<RX_DR) | (1<<TX_DS) | (1<<MAX_RT))
void NRF24L01_ClearIRQ(uint8_t IRQ_Source); 
   
// 读取FIFO数据宽度
uint8_t NRF24L01_ReadTopFIFOWidth(void);

// 读取接收到的数据       
uint8_t NRF24L01_ReadRXPayload(uint8_t *pBuff);  

// 设置NRF24L01模式 
typedef enum{ TX_MODE, RX_MODE } NRF24L01MD; 
void NRF24L01_SetTRMode(NRF24L01MD mode);

// 设置NRF24L01空速                 
typedef enum{ SPD_250K, SPD_1M, SPD_2M } NRF24L01SPD;
void NRF24L01_SetSpeed(NRF24L01SPD speed);

// 设置NRF24L01功率                 
typedef enum{ P_F18DBM, P_F12DBM, P_F6DBM, P_0DBM } NRF24L01PWR;
void NRF24L01_SetPower(NRF24L01PWR power);

// 设置NRF24L01频率                
void NRF24L01_WriteHoppingPoint(uint8_t FreqPoint);    

uint8_t NRF24L01_ReadStatusReg(void);

// 写数据到一个寄存器
void NRF24L01_WriteSingleReg(uint8_t Addr, uint8_t Value);

// 读取一个寄存器的值   
uint8_t NRF24L01_ReadSingleReg(uint8_t Addr);

// 读取多个寄存器的值                 
void NRF24L01_ReadMultiReg(uint8_t StartAddr, uint8_t nBytes, uint8_t *pBuff);

// 写数据到多个寄存器
void NRF24L01_WriteMultiReg(uint8_t StartAddr, uint8_t *pBuff, uint8_t Length);

// 写数据到TXFIFO(带ACK返回)
void NRF24L01_WriteTXPayload_Ack(uint8_t *pBuff, uint8_t nBytes);

// 写数据到TXFIFO(不带ACK返回)
void NRF24L01_WriteTXPayload_NoAck(uint8_t *Data, uint8_t Data_Length);

// 设置发送物理地址
void NRF24L01_SetTXAddr(uint8_t *pAddr, uint8_t Addr_Length);

// 设置接收物理地址
void NRF24L01_SetRXAddr(uint8_t PipeNum, uint8_t *pAddr, uint8_t Addr_Length);
uint8_t NRF24L01_SendPacket(uint8_t *Sendbuffer, uint8_t length);
void NRF24L01_RecvPacket(uint8_t *Recv_buffer,uint8_t length);
#endif//_nRF24NRF24L01_H_

/*===========================================================================
-----------------------------------文件结束----------------------------------
===========================================================================*/