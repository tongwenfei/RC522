#include "AD7705.h"
void AD7750_Init(void)
{
   uint8_t temp=0;
  GPIO_InitTypeDef GPIO_InitStructure;
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOH, ENABLE);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		/* ��Ϊ����� */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		/* ��Ϊ����ģʽ */
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	/* ���������費ʹ�� */
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	/* IO������ٶ� */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_15;//clk rst
  GPIO_Init(GPIOH, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		/* ��Ϊ����� */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		/* ��Ϊ����ģʽ */
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	/* ���������費ʹ�� */
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	/* IO������ٶ� */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//clk rst
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		/* ��Ϊ����� */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		/* ��Ϊ����ģʽ */
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	/* ���������費ʹ�� */
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	/* IO������ٶ� */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_4;//dout
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;		/* ��Ϊ����� */
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	/* ���������費ʹ�� */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_5;//DIN DRDY
  GPIO_Init(GPIOC, &GPIO_InitStructure);
 
  AD7705_Reset();
  SPI_WriteByte(0x20); ////дͨ�żĴ�������һ��дͨ��CH1ʱ�ӼĴ���
  // Delay(1); 
  SPI_WriteByte(0x03);////ʱ�ӼĴ����ⲿ����2.4576MHzCLKΪ1(����2M)CLKDIV,FS1,FS0=011ʱ���Ƶ��Ϊ25HZ
   SPI_WriteByte(0x28);//��ʱ�ӼĴ���������      
  temp=SPI_ReadByte();   
  // Delay(1); 
  SPI_WriteByte(0x10);////0x0010 дͨ�żĴ�������һ��дͨ��CHN1���üĴ���
  // Delay(1); 
  SPI_WriteByte(0x44); ///����Ϊ1����У׼ģʽ�������ԣ�
  delay_ms(50);
   SPI_WriteByte(0x18);//��ʱ�ӼĴ���������      
  temp=SPI_ReadByte();  
   SPI_WriteByte(0x08);//��ʱ�ӼĴ���������      
  temp=SPI_ReadByte();  
  //SPI_WriteByte(0x4c); ///����Ϊ2����У׼ģʽ�������ԣ�
  
  SPI_WriteByte(0x48);//�����ԼĴ���     
  temp=SPI_ReadByte(); 
  while(AD7705_DRDY)
  {
  }
}

void AD7705_Reset(void) 
{
  u8 i;
  //reset the ad7705
  AD7705_RST=0;//reset ADRSTC;
  delay_us(50);//delay 10ms
  AD7705_RST=1;//set ADRSTC;
  delay_us(50);//delay 10ms
  AD7705_MOSI=1;//set mosi;
  AD7705_SCLK=1;
  delay_us(50);//delay 10ms
  for(i=0;i<35;i++)  // ��������32�� DIN=1 ʹ���ڸ�λ 
  {  
    AD7705_SCLK=0; 
    delay_us(30); 
    AD7705_MOSI=1;
    delay_us(30);
    AD7705_SCLK=1;   
    delay_us(30); 
  } 
}
uint8_t SPI_ReadByte(void)
{
 uint8_t buff=0; 
  u8  j=0; 
  AD7705_SCLK=1;
  delay_us(30);  
  for(j=0;j<8;j++) 
  { 
    AD7705_SCLK=0; 
    delay_us(10);  
    buff<<=1; 
    if(AD7705_MISO == 0x01) buff|=0x01; 
    else; 
    AD7705_SCLK=1; 
    delay_us(10); 
  } 
  return(buff); 
}
uint16_t SPI_ReadHalfWord(void)
{
 uint16_t buff=0; 
  u8  j=0; 
  AD7705_SCLK=1;
  delay_us(30);  
  for(j=0;j<16;j++) 
  { 
    AD7705_SCLK=0; 
    delay_us(10);  
    buff<<=1; 
    if(AD7705_MISO == 0x01) buff|=0x01; 
    else; 
    AD7705_SCLK=1; 
    delay_us(10); 
  } 
  return(buff); 
}
void SPI_WriteByte(uint8_t Data)
{
 u8 i; 
  AD7705_SCLK=1; 
  delay_us(30); 
  for(i=0;i<8;i++) 
  { 
    AD7705_SCLK=0;  
    delay_us(30); 
    if(Data&0x80) AD7705_MOSI=1; 
    else AD7705_MOSI=0; //����һλ  
    Data<<=1;
    delay_us(30);
    AD7705_SCLK=1;    
    delay_us(30); //AD7705�����λ 
  } 
  // AD7705_SCLK=0;
  AD7705_MOSI=0;
  delay_us(50);
}
uint16_t Voltage=0;
void Get_Voltage(void)
{
  uint8_t temp=0;
  SPI_WriteByte(0x38);
   while(AD7705_DRDY)
  {
    SPI_WriteByte(0x08);//��ʱ�ӼĴ���������      
  temp=SPI_ReadByte(); 
  SPI_WriteByte(0x38);
  }
  Voltage=SPI_ReadHalfWord();
    
}