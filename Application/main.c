/**
******************************************************************************
* @file    Project/STM32F4xx_StdPeriph_Templates/main.c 
* @author  MCD Application Team
* @version V1.5.0
* @date    06-March-2015
* @brief   Main program body
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
*
* Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
* You may not use this file except in compliance with the License.
* You may obtain a copy of the License at:
*
*        http://www.st.com/software_license_agreement_liberty_v2
*
* Unless required by applicable law or agreed to in writing, software 
* distributed under the License is distributed on an "AS IS" BASIS, 
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup Template
* @{
*/ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
* @brief   Main program
* @param  None
* @retval None
*/



int main(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
  uart_init(115200);	 	//串口初始化为115200	
  delay_init(168);	    	 //延时函数初始化
MFRC522_Init();
  while(1) 
  {
    if ( ! PICC_IsNewCardPresent())
    {
     delay_ms(500);
      continue;
    }

    // Select one of the cards
    if ( ! PICC_ReadCardSerial())
    {
      delay_ms(500);
      continue;
    }

    
    

    // Print Card UID
    printf("Card UID: ");
    for (uint8_t i = 0; i < uid.size; i++)
    {
      printf(" %X02", uid.uiduint8_t[i]);
    }
    printf("\n\r");

    // Print Card type
    PICC_Type piccType = PICC_GetType(uid.sak);
    printf("PICC Type: %s \n\r", PICC_GetTypeName(piccType));	
    
  }
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
