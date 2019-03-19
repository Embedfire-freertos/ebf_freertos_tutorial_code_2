/**
  ******************************************************************************
  * @file    bsp_debug_usart.c
  * @author  fire
  * @version V1.0
  * @date    2016-xx-xx
  * @brief   使用串口1，重定向c库printf函数到usart端口，中断接收模式
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火  STM32 F746 开发板  
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */ 
  
#include "./usart/bsp_debug_usart.h"
/* FreeRTOS头文件 */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

UART_HandleTypeDef UartHandle;
DMA_HandleTypeDef DMA_Handle;

__attribute__ ((at(0x30000000))) uint8_t RX_BUFF[USART_RBUFF_SIZE]={0};
 /**
  * @brief  DEBUG_USART GPIO 配置,工作模式配置。115200 8-N-1
  * @param  无
  * @retval 无
  */  
void DEBUG_USART_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit;
        
    DEBUG_USART_RX_GPIO_CLK_ENABLE();
    DEBUG_USART_TX_GPIO_CLK_ENABLE();
    
    /* 配置串口1时钟源*/
		RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
		RCC_PeriphClkInit.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
		HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);
    /* 使能串口1时钟 */
    DEBUG_USART_CLK_ENABLE();

    /* 配置Tx引脚为复用功能  */
    GPIO_InitStruct.Pin = DEBUG_USART_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = DEBUG_USART_TX_AF;
    HAL_GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStruct);
    
    /* 配置Rx引脚为复用功能 */
    GPIO_InitStruct.Pin = DEBUG_USART_RX_PIN;
    GPIO_InitStruct.Alternate = DEBUG_USART_RX_AF;
    HAL_GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStruct); 
    
    /* 配置串DEBUG_USART 模式 */
    UartHandle.Instance = DEBUG_USART;
    UartHandle.Init.BaudRate = 115200;
    UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits = UART_STOPBITS_1;
    UartHandle.Init.Parity = UART_PARITY_NONE;
    UartHandle.Init.Mode = UART_MODE_TX_RX;
    UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
    UartHandle.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED;
    UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    HAL_UART_Init(&UartHandle);

    /*串口1中断初始化 */
    HAL_NVIC_SetPriority(DEBUG_USART_IRQ, 6, 0);
    HAL_NVIC_EnableIRQ(DEBUG_USART_IRQ);
		
}

void USART_DMA_Config(void)
{
  /*开启DMA时钟*/
  DEBUG_USART_DMA_CLK_ENABLE();

  DMA_Handle.Instance = DEBUG_USART_DMA_STREAM;
  /*usart1 tx对应dma2，通道4，数据流7*/	
  DMA_Handle.Init.Request = DMA_REQUEST_USART1_RX; 
  /*方向：从内存到外设*/		
  DMA_Handle.Init.Direction= DMA_PERIPH_TO_MEMORY;	
  /*外设地址不增*/	    
  DMA_Handle.Init.PeriphInc = DMA_PINC_DISABLE; 
  /*内存地址自增*/
  DMA_Handle.Init.MemInc = DMA_MINC_ENABLE;	
  /*外设数据单位*/	
  DMA_Handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  /*内存数据单位 8bit*/
  DMA_Handle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;	
  /*DMA模式：不断循环*/
  DMA_Handle.Init.Mode = DMA_CIRCULAR;	 
  /*优先级：中*/	
  DMA_Handle.Init.Priority = DMA_PRIORITY_MEDIUM;      
  /*禁用FIFO*/
  DMA_Handle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;        
  DMA_Handle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;    
  /*存储器突发传输 1个节拍*/
  DMA_Handle.Init.MemBurst = DMA_MBURST_SINGLE;    
  /*外设突发传输 1个节拍*/
  DMA_Handle.Init.PeriphBurst = DMA_PBURST_SINGLE;    
  /*配置DMA2的数据流7*/		   
//  /* Deinitialize the stream for new transfer */
  HAL_DMA_DeInit(&DMA_Handle);
  /* Configure the DMA stream */
  HAL_DMA_Init(&DMA_Handle); 
  
//	__HAL_DMA_ENABLE(&UartHandle);  
	
   /* Associate the DMA handle */
  __HAL_LINKDMA(&UartHandle, hdmarx, DMA_Handle);
	
	HAL_UART_Receive_DMA(&UartHandle, RX_BUFF, USART_RBUFF_SIZE);
  
	/*配置串口接收中断，在串口初始化调用的话会进入一次中断导致错误，在外部使能即可 */
	__HAL_UART_CLEAR_IT(&UartHandle, UART_CLEAR_IDLEF);
	__HAL_UART_ENABLE_IT(&UartHandle,UART_IT_IDLE);  
  
}

extern SemaphoreHandle_t BinarySem_Handle;

void Uart_DMA_Rx_Data(void)
{
  BaseType_t pxHigherPriorityTaskWoken;
  
  // 关闭DMA ，防止干扰
  __HAL_DMA_DISABLE(&DMA_Handle);      
  // 清DMA标志位
  __HAL_DMA_CLEAR_FLAG(&DMA_Handle,DMA_FLAG_TCIF3_7);     

  //  重新赋值计数值，必须大于等于最大可能接收到的数据帧数目   
  WRITE_REG(((DMA_Stream_TypeDef   *)DMA_Handle.Instance)->NDTR , USART_RBUFF_SIZE);

  __HAL_DMA_ENABLE(&DMA_Handle);  

  //给出二值信号量 ，发送接收到新数据标志，供前台程序查询
  xSemaphoreGiveFromISR(BinarySem_Handle,&pxHigherPriorityTaskWoken);	//释放二值信号量
  //如果需要的话进行一次任务切换，系统会判断是否需要进行切换
  portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
	
}


/*****************  发送字符串 **********************/
void Usart_SendString(uint8_t *str)
{
	unsigned int k=0;
  do 
  {
      HAL_UART_Transmit( &UartHandle,(uint8_t *)(str + k) ,1,1000);
      k++;
  } while(*(str + k)!='\0');
  
}
///重定向c库函数printf到串口DEBUG_USART，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
	/* 发送一个字节数据到串口DEBUG_USART */
	HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 1000);	
	
	return (ch);
}

///重定向c库函数scanf到串口DEBUG_USART，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{
		
	int ch;
	HAL_UART_Receive(&UartHandle, (uint8_t *)&ch, 1, 1000);	
	return (ch);
}


void UART_IdelCallback(void)
{
  Uart_DMA_Rx_Data();       /* 释放一个信号量，表示数据已接收 */
}

/*********************************************END OF FILE**********************/
