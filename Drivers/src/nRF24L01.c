/*===========================================================================
* ��ַ ��http://yhmcu.taobao.com/   http://www.cdebyte.com/                 *
* ���� ������  ԭ �ں͵��ӹ�����  �� �ڰ��ص��ӿƼ����޹�˾                 * 
* �ʼ� ��yihe_liyong@126.com                                                *
* �绰 ��18615799380                                                        *
============================================================================*/

#include "NRF24L01.h"
uint8_t RX_ADD[5]={5,4,3,2,1},TX_ADD[5]={1,2,3,4,5};
uint8_t SPI_ExchangeByte(uint8_t TxData) // ͨ��SPI�������ݽ���
{
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_FLAG_TXE) == RESET){}//�ȴ���������  
  
  SPI_I2S_SendData(SPI2, TxData); //ͨ������SPIx����һ��byte  ����
  
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_FLAG_RXNE) == RESET){} //�ȴ�������һ��byte  
  
  return SPI_I2S_ReceiveData(SPI2); //����ͨ��SPIx������յ�����	
  
}

/*===========================================================================
* ���� ��NRF24L01_ReadSingleReg() => ��ȡһ���Ĵ�����ֵ                          * 
* ���� ��Addr����ȡ�ļĴ�����ַ                                             * 
* ��� ��������ֵ                                                           * 
============================================================================*/
uint8_t NRF24L01_ReadSingleReg(uint8_t Addr)
{
  uint8_t btmp;
  
  NRF24L01_CSN_LOW();
  SPI_ExchangeByte(R_REGISTER | Addr);
  btmp = SPI_ExchangeByte(0xFF);
  NRF24L01_CSN_HIGH();
  
  return (btmp);
}

/*===========================================================================
* ���� ��NRF24L01_WriteSingleReg() => д���ݵ�һ���Ĵ���                         * 
* ���� ��Addr��д��Ĵ����ĵ�ַ��Value����д���ֵ                          * 
============================================================================*/
void NRF24L01_WriteSingleReg(uint8_t Addr, uint8_t Value)
{
  NRF24L01_CSN_LOW();
  SPI_ExchangeByte(W_REGISTER | Addr);
  SPI_ExchangeByte(Value);
  NRF24L01_CSN_HIGH();
}

/*===========================================================================
* ���� ��NRF24L01_WriteMultiReg() => д���ݵ�����Ĵ���                          * 
* ���� ��StartAddr,д��Ĵ������׵�ַ��pBuff,ָ���д���ֵ��Length,����    * 
============================================================================*/
void NRF24L01_WriteMultiReg(uint8_t StartAddr, uint8_t *pBuff, uint8_t Length)
{
  uint8_t i;
  
  NRF24L01_CSN_LOW();
  SPI_ExchangeByte(W_REGISTER | StartAddr);
  for (i=0; i<Length; i++)    { SPI_ExchangeByte(*(pBuff+i)); }
  NRF24L01_CSN_HIGH();
}

/*===========================================================================
* ���� ��NRF24L01_FlushTX() => ��λTX FIFOָ��                                   * 
============================================================================*/
void NRF24L01_FlushTX(void)
{
  NRF24L01_CSN_LOW();
  SPI_ExchangeByte(FLUSH_TX);
  NRF24L01_CSN_HIGH();
}

/*===========================================================================
* ���� ��NRF24L01_FlushRX() => ��λRX FIFOָ��                                   *
============================================================================*/
void NRF24L01_FlushRX(void)
{
  NRF24L01_CSN_LOW();
  SPI_ExchangeByte(FLUSH_RX);
  NRF24L01_CSN_HIGH();
}

uint8_t NRF24L01_ReadStatusReg(void)
{
  uint8_t Status;
  NRF24L01_CSN_LOW();
  Status = SPI_ExchangeByte(R_REGISTER + NRF24L01REG_STATUS);
  NRF24L01_CSN_HIGH();
  return (Status);
}

/*===========================================================================
* ���� ��NRF24L01_ClearIRQ() => ����ж�                                         * 
* ���� ��IRQ_Source����Ҫ������ж�Դ                                       * 
============================================================================*/
void NRF24L01_ClearIRQ(uint8_t IRQ_Source)
{
  uint8_t btmp = 0;
  
  IRQ_Source &= (1<<RX_DR) | (1<<TX_DS) | (1<<MAX_RT);
  btmp = NRF24L01_ReadStatusReg();
  
  NRF24L01_CSN_LOW();
  SPI_ExchangeByte(W_REGISTER + NRF24L01REG_STATUS);
  SPI_ExchangeByte(IRQ_Source | btmp);
  NRF24L01_CSN_HIGH();
  
  NRF24L01_ReadStatusReg();
}

/*===========================================================================
* ���� ��NRF24L01_ReadIRQSource() => ��ȡ�ж�                                    *         
* ��� ���������ж�Դ                                                       * 
============================================================================*/
uint8_t NRF24L01_ReadIRQSource(void)
{
  return (NRF24L01_ReadStatusReg() & ((1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT)));
}

/*===========================================================================
* ���� ��NRF24L01_ReadTopFIFOWidth() => ��ȡFIFO���ݿ��                         * 
============================================================================*/
uint8_t NRF24L01_ReadTopFIFOWidth(void)
{
  uint8_t btmp;
  
  NRF24L01_CSN_LOW();
  SPI_ExchangeByte(R_RX_PL_WID);
  btmp = SPI_ExchangeByte(0xFF);
  NRF24L01_CSN_HIGH();
  
  return (btmp);
}

/*===========================================================================
* ���� ��NRF24L01_ReadRXPayload() => ��ȡ���յ�������                            * 
* ���� ��pBuff��ָ���յ�������                                              * 
* ��� �����ݳ���                                                           * 
============================================================================*/
uint8_t NRF24L01_ReadRXPayload(uint8_t *pBuff)
{
  uint8_t width, PipeNum;
  PipeNum = (NRF24L01_ReadSingleReg(NRF24L01REG_STATUS)>>1) & 0x07;
  width = NRF24L01_ReadTopFIFOWidth();
  
  NRF24L01_CSN_LOW();
  SPI_ExchangeByte(R_RX_PAYLOAD);
  for (PipeNum=0; PipeNum<width; PipeNum++)
  {
    *(pBuff+PipeNum) = SPI_ExchangeByte(0xFF);
  }
  NRF24L01_CSN_HIGH();
  NRF24L01_FlushRX();
  return (width);
}

/*===========================================================================
* ���� ��NRF24L01_WriteTXPayload_Ack() => д���ݵ�TXFIFO(��ACK����)              * 
* ���� ��pBuff��ָ���д������ݣ�nBytes��д�����ݵĳ���                    * 
============================================================================*/
void NRF24L01_WriteTXPayload_Ack(uint8_t *pBuff, uint8_t nBytes)
{
  uint8_t btmp;
  uint8_t length = (nBytes>32) ? 32 : nBytes;
  
  NRF24L01_FlushTX();
  NRF24L01_CSN_LOW();
  SPI_ExchangeByte(W_TX_PAYLOAD);
  for (btmp=0; btmp<length; btmp++)   { SPI_ExchangeByte(*(pBuff+btmp)); }
  NRF24L01_CSN_HIGH();
}

/*===========================================================================
* ���� ��NRF24L01_WriteTXPayload_Ack() => д���ݵ�TXFIFO(����ACK����)            * 
* ���� ��Data��ָ���д������ݣ�Data_Length��д�����ݵĳ���                * 
============================================================================*/
void NRF24L01_WriteTXPayload_NoAck(uint8_t *Data, uint8_t Data_Length)
{
  if ((Data_Length>32) || (Data_Length==0))   { return; }
  
  NRF24L01_CSN_LOW();
  SPI_ExchangeByte(W_TX_PAYLOAD_NOACK);
  while (Data_Length--)                       { SPI_ExchangeByte(*Data++); }
  NRF24L01_CSN_HIGH();
}

/*===========================================================================
* ���� ��NRF24L01_SetTXAddr() => ���÷��������ַ                                * 
* ���� ��pAddrָ����Ҫ���õĵ�ַ���ݣ�Addr_Length����ַ����                 * 
============================================================================*/
void NRF24L01_SetTXAddr(uint8_t *pAddr, uint8_t Addr_Length)
{
  uint8_t Length = (Addr_Length>5) ? 5 : Addr_Length;
  NRF24L01_WriteMultiReg(NRF24L01REG_TX_ADDR, pAddr, Length);
}

/*===========================================================================
* ���� ��NRF24L01_SetRXAddr() => ���ý��������ַ                                * 
* ���� ��PipeNum���ܵ��ţ�pAddrָ����Ҫ���õ�ַ���ݣ�Addr_Length����ַ����  * 
============================================================================*/
void NRF24L01_SetRXAddr(uint8_t PipeNum, uint8_t *pAddr, uint8_t Addr_Length)
{
  uint8_t Length = (Addr_Length>5) ? 5 : Addr_Length;
  uint8_t pipe = (PipeNum>5) ? 5 : PipeNum;
  
  NRF24L01_WriteMultiReg(NRF24L01REG_RX_ADDR_P0 + pipe, pAddr, Length);
}

/*===========================================================================
* ���� ��NRF24L01_SetSpeed() => ����NRF24L01����                                      * 
* ���� ��speed��=SPD_250K(250K), =SPD_1M(1M), =SPD_2M(2M)                   * 
============================================================================*/
void NRF24L01_SetSpeed(NRF24L01SPD speed)
{
  uint8_t btmp = NRF24L01_ReadSingleReg(NRF24L01REG_RF_SETUP);
  
  btmp &= ~((1<<5) | (1<<3));
  
  switch (speed)
  {
  case SPD_250K:  btmp |= (1<<5);             break;  // 250K
  case SPD_1M:    btmp &= ~((1<<5) | (1<<3)); break;  // 1M
  case SPD_2M:    btmp |= (1<<3);             break;  // 2M
  default:        break;                                     
  
  }
  NRF24L01_WriteSingleReg(NRF24L01REG_RF_SETUP, btmp);
}

/*===========================================================================
* ���� ��NRF24L01_SetPower() => ����NRF24L01����                                      * 
* ���� ��power, =P_F18DBM(18DB),=P_F12DBM(12DB),=P_F6DBM(6DB),=P_0DBM(0DB)  *
============================================================================*/
void NRF24L01_SetPower(NRF24L01PWR power)
{
  uint8_t btmp = NRF24L01_ReadSingleReg(NRF24L01REG_RF_SETUP) & ~0x07;
  
  switch(power)
  {
  case P_F18DBM:  btmp |= PWR_18DB; break;    // 18DBM
  case P_F12DBM:  btmp |= PWR_12DB; break;    // 12DBM
  case P_F6DBM:   btmp |= PWR_6DB;  break;    // 6DBM
  case P_0DBM:    btmp |= PWR_0DB;  break;    // 0DBM
  default:        break;
  }
  NRF24L01_WriteSingleReg(NRF24L01REG_RF_SETUP, btmp);
}

/*===========================================================================
* ���� ��NRF24L01_WriteHoppingPoint() => ����NRF24L01Ƶ��                             * 
* ���� ��FreqPoint�������õ�Ƶ��                                            * 
============================================================================*/
void NRF24L01_WriteHoppingPoint(uint8_t FreqPoint)
{
  NRF24L01_WriteSingleReg(NRF24L01REG_RF_CH, FreqPoint & 0x7F);
}

/*===========================================================================
* ���� ��NRF24L01_SetTRMode() => ����NRF24L01ģʽ                                     * 
* ���� ��mode��=TX_MODE, TX mode; =RX_MODE, RX mode                         * 
============================================================================*/
void NRF24L01_SetTRMode(NRF24L01MD mode)
{
  uint8_t controlreg = NRF24L01_ReadSingleReg(NRF24L01REG_CONFIG);
  if      (mode == TX_MODE)       { controlreg &= ~(1<<PRIM_RX); }
  else if (mode == RX_MODE)       { controlreg |= (1<<PRIM_RX); }
  
  NRF24L01_WriteSingleReg(NRF24L01REG_CONFIG, controlreg);
}

/*===========================================================================
* ���� ��NRF24L01_Init() => ��ʼ��NRF24L01                                             * 
============================================================================*/
void NRF24L01_Init(uint8_t mode)
{
 
  GPIO_InitTypeDef  GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOG, ENABLE);//ʹ��GPIOB,Gʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);//ʹ��SPI2ʱ��
  
  
  //GPIOG6,7�������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOG, &GPIO_InitStructure);//��ʼ��PG6,7
  
  //GPIOG.8��������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//����
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOG, &GPIO_InitStructure);//��ʼ��PG8
  
  //GPIOFB3,4,5��ʼ������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;//PB3~5���ù������	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
  
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI2); //PB3����Ϊ SPI2
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_SPI2); //PB4����Ϊ SPI2
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_SPI2); //PB5����Ϊ SPI2
  
  
  SPI_Cmd(SPI2, DISABLE); //ʧ��SPI����
  
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//����SPI����ģʽ:����Ϊ��SPI
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//����ͬ��ʱ�ӵĿ���״̬Ϊ�͵�ƽ
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	//����ͬ��ʱ�ӵĵ�1�������أ��������½������ݱ�����
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
  SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRCֵ����Ķ���ʽ
  SPI_Init(SPI2, &SPI_InitStructure);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
  
  SPI_Cmd(SPI2, ENABLE); //ʹ��SPI����
  NRF24L01_CE_LOW();    
  NRF24L01_ClearIRQ(IRQ_ALL);
  
  // ʹ�ܹܵ�0��̬������
  NRF24L01_WriteSingleReg(NRF24L01REG_DYNPD, (1<<0));
  NRF24L01_WriteSingleReg(NRF24L01REG_FEATRUE, 0x06);
  NRF24L01_ReadSingleReg(NRF24L01REG_DYNPD);
  NRF24L01_ReadSingleReg(NRF24L01REG_FEATRUE);
  
  NRF24L01_WriteSingleReg(NRF24L01REG_CONFIG, (1<<EN_CRC)|(1<<PWR_UP));
  NRF24L01_WriteSingleReg(NRF24L01REG_EN_AA, 0x00);     // �Զ�Ӧ�𣨹ܵ�0��
  NRF24L01_WriteSingleReg(NRF24L01REG_EN_RXADDR, 0x01);  // ʹ�ܽ��գ��ܵ�0��
  NRF24L01_WriteSingleReg(NRF24L01REG_SETUP_AW, AW_5BYTES);     // ��ַ��� 5byte
  NRF24L01_WriteSingleReg(NRF24L01REG_RETR, ARD_4000US|(REPEAT_CNT&0x0F));
  
  NRF24L01_WriteSingleReg(NRF24L01REG_RF_CH, 60);               // ��ʼ��Ƶ��
  NRF24L01_WriteSingleReg(NRF24L01REG_RF_SETUP, 0x26); 
  NRF24L01_SetTXAddr(&TX_ADD[0], 5);                         // ���õ�ַ�����ͣ�
  NRF24L01_SetRXAddr(0,&RX_ADD[0], 5);                      // ���õ�ַ�����գ�
  if (RX == mode)     { NRF24L01_SetTRMode(RX_MODE); } // ����ģʽ      
  NRF24L01_WriteHoppingPoint(60);                      // ����Ƶ��            
  NRF24L01_SetSpeed(SPD_2M);                           // ���ÿ���Ϊ1M        
  
  NRF24L01_FlushRX();                                  // ��λ����FIFOָ��    
  NRF24L01_FlushTX();                                  // ��λ����FIFOָ��
  NRF24L01_ClearIRQ(IRQ_ALL);                          // ��������ж�
  if (RX == mode)     { NRF24L01_CE_HIGH(); }          // CE = 1, ��������          
  
}
uint8_t NRF24L01_SendPacket(uint8_t *Sendbuffer, uint8_t length)
{
  uint8_t tmp = 0;
  
  NRF24L01_CE_LOW();               // CE = 0, �رշ���    
  
  NRF24L01_SetTRMode(TX_MODE);     // ����Ϊ����ģʽ      	
  NRF24L01_WriteTXPayload_NoAck(Sendbuffer, length);  
  
  NRF24L01_CE_HIGH();              // CE = 1, ��������   
  
  // �ȴ������жϲ���
  while (0 != NRF24L01_IRQ_READ());
  while (0 == (tmp=NRF24L01_ReadIRQSource()));
  
  NRF24L01_FlushTX();              // ��λ����FIFOָ��    
  NRF24L01_ClearIRQ(IRQ_ALL);      // ����ж� 
  
  NRF24L01_CE_LOW();               // CE = 0, �رշ���    
  
  return (tmp & (1<<TX_DS));  // ���ط����Ƿ�ɹ�   
}
void NRF24L01_RecvPacket(uint8_t *Recv_buffer,uint8_t length)
{    
  length=0;
  if (0 == NRF24L01_IRQ_READ())                    // �������ģ���Ƿ���������ж� 
  {
    if (NRF24L01_ReadIRQSource() & (1<<RX_DR))   // �������ģ���Ƿ���յ�����
    {
      
      length = NRF24L01_ReadRXPayload(Recv_buffer);
      
      for(uint8_t i=0;i<length;i++)
      {
	printf("%d ",Recv_buffer[i]);
      }
    }    
    
    NRF24L01_FlushRX();                          // ��λ����FIFOָ��    
    NRF24L01_ClearIRQ(IRQ_ALL);                  // ����ж�            
  }
}
/*===========================================================================
-----------------------------------�ļ�����----------------------------------
===========================================================================*/
