/*===========================================================================
* ��ַ ��http://yhmcu.taobao.com/   http://www.cdebyte.com/                 *
* ���� ������  ԭ �ں͵��ӹ�����  �� �ڰ��ص��ӿƼ����޹�˾                 * 
* �ʼ� ��yihe_liyong@126.com                                                *
* �绰 ��18615799380                                                        *
============================================================================*/

#ifndef _nRF24NRF24L01_H_
#define _nRF24NRF24L01_H_

#include "sys.h"
#include "NRF24L01_Reg.h"
// ��������
#define TX              1       // ����ģʽ
#define RX              0       // ����ģʽ
#define REPEAT_CNT      15  // 0-15, repeat transmit count




#define NRF24L01_CSN_LOW()   (PGout(7)=0)
#define NRF24L01_CSN_HIGH()  (PGout(7)=1)

#define NRF24L01_CE_LOW()    (PGout(6)=0)
#define NRF24L01_CE_HIGH()   (PGout(6)=1)

#define NRF24L01_IRQ_READ()  PGin(8)

// nRF24NRF24L01P��غ����ӿ�
// ��ʼ��NRF24L01
void NRF24L01_Init(uint8_t mode);

// ��λTX FIFOָ��      
void NRF24L01_FlushTX(void);

// ��λRX FIFOָ��     
void NRF24L01_FlushRX(void);     

// ��ȡ�ж�
uint8_t NRF24L01_ReadIRQSource(void);          

// ����ж�
#define IRQ_ALL  ((1<<RX_DR) | (1<<TX_DS) | (1<<MAX_RT))
void NRF24L01_ClearIRQ(uint8_t IRQ_Source); 
   
// ��ȡFIFO���ݿ��
uint8_t NRF24L01_ReadTopFIFOWidth(void);

// ��ȡ���յ�������       
uint8_t NRF24L01_ReadRXPayload(uint8_t *pBuff);  

// ����NRF24L01ģʽ 
typedef enum{ TX_MODE, RX_MODE } NRF24L01MD; 
void NRF24L01_SetTRMode(NRF24L01MD mode);

// ����NRF24L01����                 
typedef enum{ SPD_250K, SPD_1M, SPD_2M } NRF24L01SPD;
void NRF24L01_SetSpeed(NRF24L01SPD speed);

// ����NRF24L01����                 
typedef enum{ P_F18DBM, P_F12DBM, P_F6DBM, P_0DBM } NRF24L01PWR;
void NRF24L01_SetPower(NRF24L01PWR power);

// ����NRF24L01Ƶ��                
void NRF24L01_WriteHoppingPoint(uint8_t FreqPoint);    

uint8_t NRF24L01_ReadStatusReg(void);

// д���ݵ�һ���Ĵ���
void NRF24L01_WriteSingleReg(uint8_t Addr, uint8_t Value);

// ��ȡһ���Ĵ�����ֵ   
uint8_t NRF24L01_ReadSingleReg(uint8_t Addr);

// ��ȡ����Ĵ�����ֵ                 
void NRF24L01_ReadMultiReg(uint8_t StartAddr, uint8_t nBytes, uint8_t *pBuff);

// д���ݵ�����Ĵ���
void NRF24L01_WriteMultiReg(uint8_t StartAddr, uint8_t *pBuff, uint8_t Length);

// д���ݵ�TXFIFO(��ACK����)
void NRF24L01_WriteTXPayload_Ack(uint8_t *pBuff, uint8_t nBytes);

// д���ݵ�TXFIFO(����ACK����)
void NRF24L01_WriteTXPayload_NoAck(uint8_t *Data, uint8_t Data_Length);

// ���÷��������ַ
void NRF24L01_SetTXAddr(uint8_t *pAddr, uint8_t Addr_Length);

// ���ý��������ַ
void NRF24L01_SetRXAddr(uint8_t PipeNum, uint8_t *pAddr, uint8_t Addr_Length);
uint8_t NRF24L01_SendPacket(uint8_t *Sendbuffer, uint8_t length);
void NRF24L01_RecvPacket(uint8_t *Recv_buffer,uint8_t length);
#endif//_nRF24NRF24L01_H_

/*===========================================================================
-----------------------------------�ļ�����----------------------------------
===========================================================================*/