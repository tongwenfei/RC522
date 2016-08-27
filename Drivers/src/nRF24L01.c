/*===========================================================================
* 网址 ：http://yhmcu.taobao.com/   http://www.cdebyte.com/                 *
* 作者 ：李勇  原 亿和电子工作室  现 亿佰特电子科技有限公司                 * 
* 邮件 ：yihe_liyong@126.com                                                *
* 电话 ：18615799380                                                        *
============================================================================*/

#include "NRF24L01.h"
uint8_t RX_ADD[5]={5,4,3,2,1},TX_ADD[5]={1,2,3,4,5};
uint8_t SPI_ExchangeByte(uint8_t TxData) // 通过SPI进行数据交换
{
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_FLAG_TXE) == RESET){}//等待发送区空  
  
  SPI_I2S_SendData(SPI2, TxData); //通过外设SPIx发送一个byte  数据
  
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_FLAG_RXNE) == RESET){} //等待接收完一个byte  
  
  return SPI_I2S_ReceiveData(SPI2); //返回通过SPIx最近接收的数据	
  
}

/*===========================================================================
* 函数 ：NRF24L01_ReadSingleReg() => 读取一个寄存器的值                          * 
* 输入 ：Addr，读取的寄存器地址                                             * 
* 输出 ：读出的值                                                           * 
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
* 函数 ：NRF24L01_WriteSingleReg() => 写数据到一个寄存器                         * 
* 输入 ：Addr，写入寄存器的地址，Value，待写入的值                          * 
============================================================================*/
void NRF24L01_WriteSingleReg(uint8_t Addr, uint8_t Value)
{
  NRF24L01_CSN_LOW();
  SPI_ExchangeByte(W_REGISTER | Addr);
  SPI_ExchangeByte(Value);
  NRF24L01_CSN_HIGH();
}

/*===========================================================================
* 函数 ：NRF24L01_WriteMultiReg() => 写数据到多个寄存器                          * 
* 输入 ：StartAddr,写入寄存器的首地址，pBuff,指向待写入的值，Length,长度    * 
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
* 函数 ：NRF24L01_FlushTX() => 复位TX FIFO指针                                   * 
============================================================================*/
void NRF24L01_FlushTX(void)
{
  NRF24L01_CSN_LOW();
  SPI_ExchangeByte(FLUSH_TX);
  NRF24L01_CSN_HIGH();
}

/*===========================================================================
* 函数 ：NRF24L01_FlushRX() => 复位RX FIFO指针                                   *
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
* 函数 ：NRF24L01_ClearIRQ() => 清除中断                                         * 
* 输入 ：IRQ_Source，需要清除的中断源                                       * 
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
* 函数 ：NRF24L01_ReadIRQSource() => 读取中断                                    *         
* 输出 ：读出的中断源                                                       * 
============================================================================*/
uint8_t NRF24L01_ReadIRQSource(void)
{
  return (NRF24L01_ReadStatusReg() & ((1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT)));
}

/*===========================================================================
* 函数 ：NRF24L01_ReadTopFIFOWidth() => 读取FIFO数据宽度                         * 
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
* 函数 ：NRF24L01_ReadRXPayload() => 读取接收到的数据                            * 
* 输入 ：pBuff，指向收到的数据                                              * 
* 输出 ：数据长度                                                           * 
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
* 函数 ：NRF24L01_WriteTXPayload_Ack() => 写数据到TXFIFO(带ACK返回)              * 
* 输入 ：pBuff，指向待写入的数据，nBytes，写入数据的长度                    * 
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
* 函数 ：NRF24L01_WriteTXPayload_Ack() => 写数据到TXFIFO(不带ACK返回)            * 
* 输入 ：Data，指向待写入的数据，Data_Length，写入数据的长度                * 
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
* 函数 ：NRF24L01_SetTXAddr() => 设置发送物理地址                                * 
* 输入 ：pAddr指向需要设置的地址数据，Addr_Length，地址长度                 * 
============================================================================*/
void NRF24L01_SetTXAddr(uint8_t *pAddr, uint8_t Addr_Length)
{
  uint8_t Length = (Addr_Length>5) ? 5 : Addr_Length;
  NRF24L01_WriteMultiReg(NRF24L01REG_TX_ADDR, pAddr, Length);
}

/*===========================================================================
* 函数 ：NRF24L01_SetRXAddr() => 设置接收物理地址                                * 
* 输入 ：PipeNum，管道号，pAddr指向需要设置地址数据，Addr_Length，地址长度  * 
============================================================================*/
void NRF24L01_SetRXAddr(uint8_t PipeNum, uint8_t *pAddr, uint8_t Addr_Length)
{
  uint8_t Length = (Addr_Length>5) ? 5 : Addr_Length;
  uint8_t pipe = (PipeNum>5) ? 5 : PipeNum;
  
  NRF24L01_WriteMultiReg(NRF24L01REG_RX_ADDR_P0 + pipe, pAddr, Length);
}

/*===========================================================================
* 函数 ：NRF24L01_SetSpeed() => 设置NRF24L01空速                                      * 
* 输入 ：speed，=SPD_250K(250K), =SPD_1M(1M), =SPD_2M(2M)                   * 
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
* 函数 ：NRF24L01_SetPower() => 设置NRF24L01功率                                      * 
* 输入 ：power, =P_F18DBM(18DB),=P_F12DBM(12DB),=P_F6DBM(6DB),=P_0DBM(0DB)  *
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
* 函数 ：NRF24L01_WriteHoppingPoint() => 设置NRF24L01频率                             * 
* 输入 ：FreqPoint，待设置的频率                                            * 
============================================================================*/
void NRF24L01_WriteHoppingPoint(uint8_t FreqPoint)
{
  NRF24L01_WriteSingleReg(NRF24L01REG_RF_CH, FreqPoint & 0x7F);
}

/*===========================================================================
* 函数 ：NRF24L01_SetTRMode() => 设置NRF24L01模式                                     * 
* 输入 ：mode，=TX_MODE, TX mode; =RX_MODE, RX mode                         * 
============================================================================*/
void NRF24L01_SetTRMode(NRF24L01MD mode)
{
  uint8_t controlreg = NRF24L01_ReadSingleReg(NRF24L01REG_CONFIG);
  if      (mode == TX_MODE)       { controlreg &= ~(1<<PRIM_RX); }
  else if (mode == RX_MODE)       { controlreg |= (1<<PRIM_RX); }
  
  NRF24L01_WriteSingleReg(NRF24L01REG_CONFIG, controlreg);
}

/*===========================================================================
* 函数 ：NRF24L01_Init() => 初始化NRF24L01                                             * 
============================================================================*/
void NRF24L01_Init(uint8_t mode)
{
 
  GPIO_InitTypeDef  GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOG, ENABLE);//使能GPIOB,G时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);//使能SPI2时钟
  
  
  //GPIOG6,7推挽输出
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOG, &GPIO_InitStructure);//初始化PG6,7
  
  //GPIOG.8上拉输入
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//输入
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOG, &GPIO_InitStructure);//初始化PG8
  
  //GPIOFB3,4,5初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;//PB3~5复用功能输出	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
  
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI2); //PB3复用为 SPI2
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_SPI2); //PB4复用为 SPI2
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_SPI2); //PB5复用为 SPI2
  
  
  SPI_Cmd(SPI2, DISABLE); //失能SPI外设
  
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收8位帧结构
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//串行同步时钟的空闲状态为低电平
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	//串行同步时钟的第1个跳变沿（上升或下降）数据被采样
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;		//定义波特率预分频的值:波特率预分频值为256
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
  SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
  SPI_Init(SPI2, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
  
  SPI_Cmd(SPI2, ENABLE); //使能SPI外设
  NRF24L01_CE_LOW();    
  NRF24L01_ClearIRQ(IRQ_ALL);
  
  // 使能管道0动态包长度
  NRF24L01_WriteSingleReg(NRF24L01REG_DYNPD, (1<<0));
  NRF24L01_WriteSingleReg(NRF24L01REG_FEATRUE, 0x06);
  NRF24L01_ReadSingleReg(NRF24L01REG_DYNPD);
  NRF24L01_ReadSingleReg(NRF24L01REG_FEATRUE);
  
  NRF24L01_WriteSingleReg(NRF24L01REG_CONFIG, (1<<EN_CRC)|(1<<PWR_UP));
  NRF24L01_WriteSingleReg(NRF24L01REG_EN_AA, 0x00);     // 自动应答（管道0）
  NRF24L01_WriteSingleReg(NRF24L01REG_EN_RXADDR, 0x01);  // 使能接收（管道0）
  NRF24L01_WriteSingleReg(NRF24L01REG_SETUP_AW, AW_5BYTES);     // 地址宽度 5byte
  NRF24L01_WriteSingleReg(NRF24L01REG_RETR, ARD_4000US|(REPEAT_CNT&0x0F));
  
  NRF24L01_WriteSingleReg(NRF24L01REG_RF_CH, 60);               // 初始化频率
  NRF24L01_WriteSingleReg(NRF24L01REG_RF_SETUP, 0x26); 
  NRF24L01_SetTXAddr(&TX_ADD[0], 5);                         // 设置地址（发送）
  NRF24L01_SetRXAddr(0,&RX_ADD[0], 5);                      // 设置地址（接收）
  if (RX == mode)     { NRF24L01_SetTRMode(RX_MODE); } // 接收模式      
  NRF24L01_WriteHoppingPoint(60);                      // 设置频率            
  NRF24L01_SetSpeed(SPD_2M);                           // 设置空速为1M        
  
  NRF24L01_FlushRX();                                  // 复位接收FIFO指针    
  NRF24L01_FlushTX();                                  // 复位发送FIFO指针
  NRF24L01_ClearIRQ(IRQ_ALL);                          // 清除所有中断
  if (RX == mode)     { NRF24L01_CE_HIGH(); }          // CE = 1, 启动接收          
  
}
uint8_t NRF24L01_SendPacket(uint8_t *Sendbuffer, uint8_t length)
{
  uint8_t tmp = 0;
  
  NRF24L01_CE_LOW();               // CE = 0, 关闭发送    
  
  NRF24L01_SetTRMode(TX_MODE);     // 设置为发送模式      	
  NRF24L01_WriteTXPayload_NoAck(Sendbuffer, length);  
  
  NRF24L01_CE_HIGH();              // CE = 1, 启动发射   
  
  // 等待发射中断产生
  while (0 != NRF24L01_IRQ_READ());
  while (0 == (tmp=NRF24L01_ReadIRQSource()));
  
  NRF24L01_FlushTX();              // 复位发送FIFO指针    
  NRF24L01_ClearIRQ(IRQ_ALL);      // 清除中断 
  
  NRF24L01_CE_LOW();               // CE = 0, 关闭发送    
  
  return (tmp & (1<<TX_DS));  // 返回发送是否成功   
}
void NRF24L01_RecvPacket(uint8_t *Recv_buffer,uint8_t length)
{    
  length=0;
  if (0 == NRF24L01_IRQ_READ())                    // 检测无线模块是否产生接收中断 
  {
    if (NRF24L01_ReadIRQSource() & (1<<RX_DR))   // 检测无线模块是否接收到数据
    {
      
      length = NRF24L01_ReadRXPayload(Recv_buffer);
      
      for(uint8_t i=0;i<length;i++)
      {
	printf("%d ",Recv_buffer[i]);
      }
    }    
    
    NRF24L01_FlushRX();                          // 复位接收FIFO指针    
    NRF24L01_ClearIRQ(IRQ_ALL);                  // 清除中断            
  }
}
/*===========================================================================
-----------------------------------文件结束----------------------------------
===========================================================================*/
