/**
  ******************************************************************************
  * @file    periph_serial_exuart.c
  * @author  zhy
  * @brief   extend uart wk2124 config.
  *          This is the common part of the extend  uart periph initialization
  *
  @verbatim
  ==============================================================================
                     ##### How to use this driver #####
  ==============================================================================
    [..]
    The common HAL driver contains a set of generic and common APIs that can be
    used by the PPP peripheral drivers and the user to start using the HAL.
    [..]
    The HAL contains two APIs' categories:
         (+) Common HAL APIs
         (+) Services HAL APIs

  @endverbatim
  ******************************************************************************
  * @attention None
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "periph_serial_exuart.h"  
#include <stdlib.h>
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/** @defgroup extend uart Types
* @{
*/  

/**
* @brief  Struct def of wk2124 SPI read
*/   
typedef struct 
{
  
  SpiClient_t *client;
  volatile uint32_t spiFlag;
  volatile uint64_t lastUpdate;
  uint8_t error;
  uint8_t enabled;
  EXPORT_T *export_t[EXUARTMAX_ID];
  
} Exuart_Struct_T;  

/**
* @}
*/  
/* Private define ------------------------------------------------------------*/
/* --------------------------WK2124 system frequency define ------------------*/
/**
* @note  unit hz (11.0592MHZ)
*/ 
#define WK_SYS_FREQUENCY    (11059200) 

/**
* @note  max fifo size 128 byte
*/ 
#define WK_FIFO_SIZE        256

/* --------------------------WK2124 register define --------------------------*/
#define  WK_SET_GREG(greg)          (0X00|greg)
#define  WK_GET_GREG(greg)          (0X40|greg)
/**
* @note  uartx 0-3
*/ 
#define  WK_SET_SREG(uartx,sreg)     (0X00|(uartx<<4)|sreg)
#define  WK_GET_SREG(uartx,sreg)     (0X40|(uartx<<4)|sreg)

#define  WK_SET_SFIFO(uartx)         (0X80|(uartx<<4))
#define  WK_GET_SFIFO(uartx)         (0XC0|(uartx<<4))

/* --------------------------Global register define --------------------------*/
#define  WK_GREG_GENA     (0X00)
#define  WK_GREG_GRST     (0X01)
#define  WK_GREG_GIER     (0X10)
#define  WK_GREG_GIFR     (0X11)

/* --------------Global register receive interrupt bitdefines ---------------*/
#define  WK_UT1INT	(0x01)
#define  WK_UT2INT	(0x02)
#define  WK_UT3INT	(0x04)
#define  WK_UT4INT	(0x08)

/* --------------------------Sub-serial control register defines --------------*/
#define  WK_SREG_SPAGE    (0X03)
//PAGE0
#define  WK_SREG_SCR      (0X04)
#define  WK_SREG_LCR      (0X05)
#define  WK_SREG_FCR      (0X06)
#define  WK_SREG_SIER     (0X07)
#define  WK_SREG_SIFR     (0X08)
#define  WK_SREG_TFCNT    (0X09)
#define  WK_SREG_RFCNT    (0X0A)
#define  WK_SREG_FSR      (0X0B)
#define  WK_SREG_LSR      (0X0C)
#define  WK_SREG_FDAT     (0X0D)
//PAGE1
#define  WK_SREG_BAUD1    (0X04)
#define  WK_SREG_BAUD0    (0X05)
#define  WK_SREG_PRES     (0X06)
#define  WK_SREG_RFTL     (0X07)
#define  WK_SREG_TFTL     (0X08)

/* ---------Sub-serial control register receive interrupt bitdefines ---------*/
#define  WK_RXOVT_INT     (0x02)
#define  WK_RFTRIG_INT    (0x01)
#define  WK_RXEMPTY       (0x08)

/* --------------------------Connect gpio define -----------------------------*/
#define EXUART_M1_CS_CLK_ENABLE()          __HAL_RCC_GPIOB_CLK_ENABLE()
#define EXUART_M1_CS_PORT                   GPIOB
#define EXUART_M1_CS_PIN                    GPIO_PIN_12
#define EXUART_M1_RST_CLK_ENABLE()          __HAL_RCC_GPIOA_CLK_ENABLE()
#define EXUART_M1_RST_PORT                  GPIOA
#define EXUART_M1_RST_PIN                   GPIO_PIN_3 
#define EXUART_M1_INT_CLK_ENABLE()          __HAL_RCC_GPIOB_CLK_ENABLE()
#define EXUART_M1_INT_PORT                  GPIOB
#define EXUART_M1_INT_PIN                   GPIO_PIN_5
#define EXUART_M1_INT_IRQHandler            EXTI9_5_IRQHandler
#define EXUART_M1_INT_IRQn                  EXTI9_5_IRQn

#define EXUART_M2_CS_CLK_ENABLE()          __HAL_RCC_GPIOG_CLK_ENABLE()
#define EXUART_M2_CS_PORT                   GPIOG
#define EXUART_M2_CS_PIN                    GPIO_PIN_11
#define EXUART_M2_RST_CLK_ENABLE()          __HAL_RCC_GPIOG_CLK_ENABLE()
#define EXUART_M2_RST_PORT                  GPIOB
#define EXUART_M2_RST_PIN                   GPIO_PIN_4 
#define EXUART_M2_INT_CLK_ENABLE()          __HAL_RCC_GPIOG_CLK_ENABLE()
#define EXUART_M2_INT_PORT                  GPIOG
#define EXUART_M2_INT_PIN                   GPIO_PIN_15
#define EXUART_M2_INT_IRQHandler            EXTI15_10_IRQHandler
#define EXUART_M2_INT_IRQn                  EXTI15_10_IRQn

/* Private variables ---------------------------------------------------------*/
static Exuart_Struct_T exuart[EXUART_MAX] = {0};
#pragma location = 0x24003AF0
static ALIGN_32BYTES (uint8_t wk_conftxbuf[4]) = {0};
#pragma location = 0x24003B30
static ALIGN_32BYTES (uint8_t wk_confrxbuf[4]) = {0};
#pragma location = 0x24003B70
static ALIGN_32BYTES (uint8_t wk_rxbuf[WK_FIFO_SIZE + 4]) = {0};
#pragma location = 0x24003c90
static ALIGN_32BYTES (uint8_t wk_txbuf[WK_FIFO_SIZE + 4]) = {0};

/* Private function prototypes -----------------------------------------------*/
static void periph_exuart_gpio_init(uint8_t exuart_module);
static void periph_exuart_wk_rst(uint8_t exuart_module);
static void periph_exuart_wkstart(Exuart_Struct_T *exuart_t);
static void periph_exuart_wksetgreg(Exuart_Struct_T *exuart_t, uint8_t greg, \
                                                                   uint8_t value);
static uint8_t periph_exuart_wkgetgreg(Exuart_Struct_T *exuart_t, uint8_t reg);
static bool periph_exuart_verify_wksetgreg(Exuart_Struct_T *exuart_t, uint8_t greg, \
                                                                   uint8_t value);
static bool periph_exuart_wkconfig(Exuart_Struct_T *exuart_t, uint32_t exuart1_bdrate,\
                                   uint32_t exuart2_bdrate,uint32_t exuart3_bdrate,\
                                                        uint32_t exuart4_bdrate);
static void periph_exuart_wksetsreg(Exuart_Struct_T *exuart_t, uint8_t uartx, \
                                                    uint8_t sreg, uint8_t value);
static uint8_t periph_exuart_wkgetsreg(Exuart_Struct_T *exuart_t,uint8_t uartx, \
                                                                    uint8_t sreg);
static bool periph_exuart_verify_wksetsreg(Exuart_Struct_T *exuart_t, uint8_t uartx, \
                                                     uint8_t sreg, uint8_t value);
static void periph_exuart_wksetsfifo(Exuart_Struct_T *exuart_t, uint8_t uartx, \
                                                   uint8_t *txbuff,uint16_t len);
static void periph_exuart_wkgetsfifo(Exuart_Struct_T *exuart_t, uint8_t uartx,uint16_t len);
static uint16_t periph_exuart_wkgetrxlen(Exuart_Struct_T *exuart_t, uint8_t uartx);
static void periph_exuart_wkrxbuf(Exuart_Struct_T *exuart_t, uint8_t uartx);
static void periph_exuart_rx_buff(Exuart_Struct_T *exuart_t, uint8_t uartx);
static bool periph_exuart_wksetbdrate(Exuart_Struct_T *exuart_t, uint8_t uartx,
                                                               uint32_t bd_rate);

/* Private functions ---------------------------------------------------------*/
/**
* @brief  Exuart init.  
* @param  exuart_module EXUART_MODULE
          bd_rate Baud rate 
* @retval None    
*/
void Periph_Exuart_Serial_Open(uint8_t exuart_module, uint32_t exuart1_bdrate,\
                               uint32_t exuart2_bdrate, uint32_t exuart3_bdrate,\
                                                        uint32_t exuart4_bdrate)
{
  switch (exuart_module){
    case EXUART_M1:
    EXUART_M1_CS_CLK_ENABLE();
    periph_exuart_gpio_init(exuart_module);
    exuart[exuart_module].client = Bsp_Spi_Serial_Open(SPI2_ID, SPI_BAUDRATEPRESCALER_8,\
                                   SPI_MO, EXUART_M1_CS_PORT, EXUART_M1_CS_PIN, \
                                          &exuart[exuart_module].spiFlag, NULL);
    periph_exuart_wk_rst(exuart_module);
    periph_exuart_wkstart(&exuart[exuart_module]);
    if(!periph_exuart_wkconfig(&exuart[exuart_module],exuart1_bdrate, exuart2_bdrate,\
                                                exuart3_bdrate, exuart4_bdrate)){
      exuart[exuart_module].error = 1; 
    }
    else{
      exuart[exuart_module].export_t[EXUART1_ID] = NULL;
      exuart[exuart_module].export_t[EXUART2_ID] = NULL;
      exuart[exuart_module].export_t[EXUART3_ID] = NULL;
      exuart[exuart_module].export_t[EXUART4_ID] = NULL;
      exuart[exuart_module].enabled = 1;
    }
    
    break;
    case EXUART_M2:
    EXUART_M2_CS_CLK_ENABLE();
    periph_exuart_gpio_init(exuart_module);
    exuart[exuart_module].client = Bsp_Spi_Serial_Open(SPI6_ID, SPI_BAUDRATEPRESCALER_8,\
                                   SPI_MO, EXUART_M2_CS_PORT, EXUART_M2_CS_PIN, \
                                          &exuart[exuart_module].spiFlag, NULL);
    periph_exuart_wk_rst(exuart_module);
    periph_exuart_wkstart(&exuart[exuart_module]);
    if(!periph_exuart_wkconfig(&exuart[exuart_module],exuart1_bdrate, exuart2_bdrate,\
                                                exuart3_bdrate, exuart4_bdrate)){
      exuart[exuart_module].error = 1; 
    }
    else{
      exuart[exuart_module].export_t[EXUART1_ID] = NULL;
      exuart[exuart_module].export_t[EXUART2_ID] = NULL;
      exuart[exuart_module].export_t[EXUART3_ID] = NULL;
      exuart[exuart_module].export_t[EXUART4_ID] = NULL;
      exuart[exuart_module].enabled = 1;  
    }   
    break; 
    default:
    break;
  }
}

/**
* @brief   Register a serial uart.  
* @param   exuart_module EXUART_MODULE
           exuart_id EXUART_ID
* @retval None   
* @note  rx_buff_size < 256 
*/
void Periph_Exuart_Serial_Register(uint8_t exuart_module, uint8_t exuart_id,\
                                    EXPORT_T *export_t,  uint8_t *rx_buff, \
                                                           uint16_t rx_buff_size)
{
  
  export_t->exuart_module = exuart_module;
  export_t->exuart_id = exuart_id;
  export_t->pRxBuf = rx_buff;
  export_t->usRxBufSize = rx_buff_size;
  export_t->usRxHead           = 0;     // rx write index
  export_t->usRxTail           = 0;     // rx read index
  export_t->usRxCount          = 0;     // counter of data received
  export_t->rxOverflow         = false;
  export_t->rxComplete         = false;
  exuart[exuart_module].export_t[exuart_id] = export_t;
  
}

/**
* @brief  Exuart tranfer buff.  
* @param  exuart_t  EXUART_T addr
          buf send buff addr
          len byte of send         
* @retval  None
* @note    len < 256  
*/
void Periph_Exuart_Send_Buff(EXPORT_T *export_t, uint8_t *buf, uint16_t len)
{
   periph_exuart_wksetsfifo(&exuart[export_t->exuart_module],export_t->exuart_id,buf,len);
}

/**
* @brief  Exuart tranfer buff.  
* @param  exuart_t  EXUART_T addr       
* @retval  None
* @note   None
*/
static void periph_exuart_rx_buff(Exuart_Struct_T *exuart_t,uint8_t uartx)
{
   periph_exuart_wkrxbuf(exuart_t,uartx); 
}

/**
* @brief  Exuart gpio init.  
* @retval exuart_module EXUART_MODULE  
*/
static void periph_exuart_gpio_init(uint8_t exuart_module)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  switch (exuart_module){
    case EXUART_M1:
    EXUART_M1_RST_CLK_ENABLE();
    EXUART_M1_INT_CLK_ENABLE();
    
    GPIO_InitStruct.Pin = EXUART_M1_RST_PIN;       
    HAL_GPIO_Init(EXUART_M1_RST_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(EXUART_M1_RST_PORT, EXUART_M1_RST_PIN, GPIO_PIN_SET);
    
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin  = EXUART_M1_INT_PIN;
    HAL_GPIO_Init(EXUART_M1_INT_PORT, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXUART_M1_INT_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXUART_M1_INT_IRQn);   
    break;
    
    case EXUART_M2:
    EXUART_M2_RST_CLK_ENABLE();
    EXUART_M2_INT_CLK_ENABLE();
    GPIO_InitStruct.Pin = EXUART_M2_RST_PIN;       
    HAL_GPIO_Init(EXUART_M2_RST_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(EXUART_M2_RST_PORT, EXUART_M2_RST_PIN, GPIO_PIN_SET);
    
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin  = EXUART_M2_INT_PIN;
    HAL_GPIO_Init(EXUART_M2_INT_PORT, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXUART_M2_INT_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXUART_M2_INT_IRQn);  
    break; 
    default:
    break;
  }
}

/**
* @brief  periph  wk2124 rst.  
* @retval exuart_module EXUART_MODULE  
*/
static void periph_exuart_wk_rst(uint8_t exuart_module)
{
  switch (exuart_module){
    case EXUART_M1:
    HAL_GPIO_WritePin(EXUART_M1_RST_PORT, EXUART_M1_RST_PIN, GPIO_PIN_SET);					
    HAL_Delay(50);
    HAL_GPIO_WritePin(EXUART_M1_RST_PORT, EXUART_M1_RST_PIN, GPIO_PIN_RESET);						
    HAL_Delay(10);
    HAL_GPIO_WritePin(EXUART_M1_RST_PORT, EXUART_M1_RST_PIN, GPIO_PIN_SET);						 
    HAL_Delay(10);
    break;
    case EXUART_M2:
    HAL_GPIO_WritePin(EXUART_M2_RST_PORT, EXUART_M2_RST_PIN, GPIO_PIN_SET);					
    HAL_Delay(50);
    HAL_GPIO_WritePin(EXUART_M2_RST_PORT, EXUART_M2_RST_PIN, GPIO_PIN_RESET);						
    HAL_Delay(10);
    HAL_GPIO_WritePin(EXUART_M2_RST_PORT, EXUART_M2_RST_PIN, GPIO_PIN_SET);						 
    HAL_Delay(10);
    break; 
    default:
    break;
  }
}

/**
* @brief  WK2124  start transmission.  
* @param  exuart_t   Exuart_Struct_T pointer
* @retval None    
*/
static void periph_exuart_wkstart(Exuart_Struct_T *exuart_t)
{
  uint16_t timeout = 0xfff;
  wk_conftxbuf[0] = 0xff;
  Bsp_Spi_Transaction(exuart_t->client,&wk_confrxbuf[0],&wk_conftxbuf[0],1);
  do{
    timeout--;
  }while((exuart_t->spiFlag == 0) && (timeout != 0));
}

/**
* @brief  WK2124  write gobal register.  
* @param  exuart_t   Exuart_Struct_T pointer
          greg       gobal register address
          value      set value        
* @retval None    
*/
static void periph_exuart_wksetgreg(Exuart_Struct_T *exuart_t, uint8_t greg, \
                                                                   uint8_t value)
{ 
  uint16_t timeout = 0xfff;
  wk_conftxbuf[0] = WK_SET_GREG(greg);
  wk_conftxbuf[1] = value;
  Bsp_Spi_Transaction(exuart_t->client,&wk_confrxbuf[0],&wk_conftxbuf[0],2);
  do{
    timeout--;
  }while((exuart_t->spiFlag == 0) && (timeout != 0));
}

/**
* @brief  WK2124  read gobal register.  
* @param  exuart_t   Exuart_Struct_T pointer
          greg       gobal register address       
* @retval None    
*/  
static uint8_t periph_exuart_wkgetgreg(Exuart_Struct_T *exuart_t, uint8_t greg)
{ 
  uint16_t timeout = 0xfff;
  wk_conftxbuf[0] = WK_GET_GREG(greg);
  Bsp_Spi_Transaction(exuart_t->client,&wk_confrxbuf[0],&wk_conftxbuf[0],2);
  do{
    timeout--;
  }while((exuart_t->spiFlag == 0) && (timeout != 0));
  
  return wk_confrxbuf[1];
}

/**
* @brief  WK2124     verify write gobal register.  
* @param  exuart_t   Exuart_Struct_T pointer
          greg       gobal register address
          value      set value        
* @retval None    
*/
static bool periph_exuart_verify_wksetgreg(Exuart_Struct_T *exuart_t, uint8_t greg, \
                                                                  uint8_t value)
{  
  uint8_t dec = 10;
  bool ret = true;
  do {
    periph_exuart_wksetgreg(exuart_t, greg, value);
    dec--;
  } while (periph_exuart_wkgetgreg(exuart_t,greg) != value && (dec != 0));
  
  if (dec == 0) {
    ret = false;
  }
  return ret;
}

/**
* @brief  WK2124  write Sub-serial register.  
* @param  exuart_t   Exuart_Struct_T pointer
          sreg       Sub-serial register address
          value      set value        
* @retval None    
*/
static void periph_exuart_wksetsreg(Exuart_Struct_T *exuart_t, uint8_t uartx, \
                                                    uint8_t sreg, uint8_t value)
{ 
  uint16_t timeout = 0xfff;
  wk_conftxbuf[0] = WK_SET_SREG(uartx,sreg);
  wk_conftxbuf[1] = value;
  Bsp_Spi_Transaction(exuart_t->client, &wk_confrxbuf[0], &wk_conftxbuf[0], 2);
  do{
    timeout--;
  }while((exuart_t->spiFlag == 0) && (timeout != 0));
}
/**
* @brief  WK2124  read Sub-serial register.  
* @param  exuart_t   Exuart_Struct_T pointer
          sreg       Sub-serial register address       
* @retval None    
*/  
static uint8_t periph_exuart_wkgetsreg(Exuart_Struct_T *exuart_t,uint8_t uartx, \
                                                                    uint8_t sreg)
{ 
  uint16_t timeout = 0xfff;
  wk_conftxbuf[0] = WK_GET_SREG(uartx,sreg);
  Bsp_Spi_Transaction(exuart_t->client, &wk_confrxbuf[0], &wk_conftxbuf[0], 2);
  do{
    timeout--;
  }while((exuart_t->spiFlag == 0) && (timeout != 0));
  
  return wk_confrxbuf[1];
}

/**
* @brief  WK2124     verify write Sub-serial register.  
* @param  exuart_t   Exuart_Struct_T pointer
          sreg       Sub-serial register address
          value      set value        
* @retval None    
*/
static bool periph_exuart_verify_wksetsreg(Exuart_Struct_T *exuart_t, uint8_t uartx, \
                                                     uint8_t sreg, uint8_t value)
{  
  uint8_t dec = 10;
  bool ret = true;
  do {
    periph_exuart_wksetsreg(exuart_t, uartx, sreg, value);
    dec--;
  } while (periph_exuart_wkgetsreg(exuart_t, uartx, sreg) != value && (dec != 0));
  
  if (dec == 0) {
    ret = false;
  }
  return ret;
}
/**
* @brief  WK2124     write Sub-serial fifo.  
* @param  exuart_t   Exuart_Struct_T pointer
          uartx      uart id
          txbuff     tranfer  buff addr
          len        send len       
* @retval None  
* @note   len < 256  
*/
static void periph_exuart_wksetsfifo(Exuart_Struct_T *exuart_t, uint8_t uartx, \
                                                   uint8_t *txbuff,uint16_t len)
{
 
  uint16_t timeout = 0xffff;
  wk_txbuf[0] = WK_SET_SFIFO(uartx);
  memcpy(&wk_txbuf[1], txbuff, len);
  Bsp_Spi_Transaction(exuart_t->client, wk_rxbuf, wk_txbuf, len+1);
  do{
    timeout--;
  }while((exuart_t->spiFlag == 0) && (timeout != 0)); 
  
}

/**
* @brief  WK2124     read Sub-serial fifo.  
* @param  exuart_t   Exuart_Struct_T pointer
          uartx      uart id
          rxbuff     receive  buff addr
          len        receive len      
* @note   len < 256   
*/

static void periph_exuart_wkgetsfifo(Exuart_Struct_T *exuart_t, uint8_t uartx,uint16_t len)
{

  uint16_t timeout = 0xffff,i = 0;
  wk_txbuf[0] = WK_GET_SFIFO(uartx);
  Bsp_Spi_Transaction(exuart_t->client, wk_rxbuf, wk_txbuf, len+1);
  
  do{
    timeout--;
  }while((exuart_t->spiFlag == 0) && (timeout != 0));
  for(i = 0; i < len; i++){
    
    exuart_t->export_t[uartx]->pRxBuf[exuart_t->export_t[uartx]->usRxHead] = wk_rxbuf[i+1];
    exuart_t->export_t[uartx]->usRxHead = (exuart_t->export_t[uartx]->usRxHead + 1) % exuart_t->export_t[uartx]->usRxBufSize;
    if(exuart_t->export_t[uartx]->usRxCount < exuart_t->export_t[uartx]->usRxBufSize){
      exuart_t->export_t[uartx]->usRxCount++;
    }
    else{
      exuart_t->export_t[uartx]->rxOverflow = true;
    }
  }
  exuart_t->export_t[uartx]->rxComplete  = true;
  exuart_t->export_t[uartx]->rxlastupdatems = HAL_GetTick();
}
/**
* @brief  WK2124    Receive len.  
* @param  exuart_t   Exuart_Struct_T pointer
          uartx      uart id 
* @retval Receive len
          
*/
static uint16_t periph_exuart_wkgetrxlen(Exuart_Struct_T *exuart_t, uint8_t uartx)
{
    uint8_t wk_sifr = 0,wk_rxfifo_emp = 0,wk_rxcnt = 0;
    uint16_t len = 0;
    wk_sifr = periph_exuart_wkgetsreg(exuart_t,uartx,WK_SREG_SIFR);
    if((wk_sifr & WK_RXOVT_INT) || (wk_sifr & WK_RFTRIG_INT)){
       wk_rxfifo_emp = periph_exuart_wkgetsreg(exuart_t,uartx,WK_SREG_FSR);
       if(wk_rxfifo_emp & WK_RXEMPTY){
         wk_rxcnt = periph_exuart_wkgetsreg(exuart_t,uartx,WK_SREG_RFCNT);
         len = (wk_rxcnt==0)?256:wk_rxcnt;
       }
    }
    return len;
}

/**
* @brief  WK2124    receive buff.  
* @param  exuart_t   Exuart_Struct_T pointer
          uartx      uart id
          rxbuff     receive  buff addr   
*/
static void periph_exuart_wkrxbuf(Exuart_Struct_T *exuart_t, uint8_t uartx)
{
  uint16_t len = 0;
  len = periph_exuart_wkgetrxlen(exuart_t,uartx);
  if(len > 0){
    periph_exuart_wkgetsfifo(exuart_t,uartx,len);
  } 
}

/**
* @brief  WK2124 uart set bd_rate and  all uart receive and tranfer enble 
* @param  exuart_t   Exuart_Struct_T pointer
          uartx      exuart_id
          bd_rate    Baud rate   
* @retval 0-failed   1-success   
*/
static bool periph_exuart_wksetbdrate(Exuart_Struct_T *exuart_t, uint8_t uartx,
                                                               uint32_t bd_rate)
{ 
  uint8_t buad_high = 0,buad_low = 0,buad_pre = 0;
  uint16_t reg_integer = 0;
  float reg_float = 0,reg_decimal = 0;
  reg_float = (float)WK_SYS_FREQUENCY/bd_rate/16;
  reg_integer = WK_SYS_FREQUENCY/bd_rate/16;
  reg_decimal = reg_float - reg_integer;
  buad_pre = (uint8_t)(reg_decimal * 16 + 0.5);
  buad_high = (reg_integer & 0xff00)>>8;
  buad_low = (reg_integer & 0x00ff) - 1;
  //switch to page0
  if(!periph_exuart_verify_wksetsreg(exuart_t,uartx,WK_SREG_SPAGE,0x00)){
    goto wk_bderror;
  }
  //disable all uart receive and tranfer 
  if(!periph_exuart_verify_wksetsreg(exuart_t,uartx,WK_SREG_SCR,0x00)){
     goto wk_bderror;
  }   
  //switch to page1 
  if(!periph_exuart_verify_wksetsreg(exuart_t,uartx,WK_SREG_SPAGE,0x01)){
     goto wk_bderror;
  }   
  //set bd_rate
  if(!periph_exuart_verify_wksetsreg(exuart_t,uartx,WK_SREG_BAUD1,buad_high)){
     goto wk_bderror;
  }
  if(!periph_exuart_verify_wksetsreg(exuart_t,uartx,WK_SREG_BAUD0,buad_low)){
     goto wk_bderror;
  }
  if(!periph_exuart_verify_wksetsreg(exuart_t,uartx,WK_SREG_PRES,buad_pre)){
     goto wk_bderror;
  }
  //switch to page0
  if(!periph_exuart_verify_wksetsreg(exuart_t,uartx,WK_SREG_SPAGE,0x00)){
     goto wk_bderror;
  }
  //enable all uart receive and tranfer 
  if(!periph_exuart_verify_wksetsreg(exuart_t,uartx,WK_SREG_SCR,0x03)){
     goto wk_bderror;
  } 
  return true;
wk_bderror:
  return false;
  
}

/**
* @brief  WK2124 uart config.  
* @param  exuart_t   Exuart_Struct_T pointer
          exuart1_bdrate    uart1 bdrate
          exuart2_bdrate    uart2 bdrate
          exuart3_bdrate    uart3 bdrate
          exuart4_bdrate    uart4 bdrate
* @retval 0-failed   1-success   
*/
static bool periph_exuart_wkconfig(Exuart_Struct_T *exuart_t, uint32_t exuart1_bdrate,\
                                   uint32_t exuart2_bdrate, uint32_t exuart3_bdrate, \
                                                         uint32_t exuart4_bdrate)
{
  //set all uart clock enable
  if(!periph_exuart_verify_wksetgreg(exuart_t, WK_GREG_GENA, 0x3f)){
    goto wkeror;
  }
  // reset the all uart 
  periph_exuart_wksetgreg(exuart_t,WK_GREG_GRST,0x0f);
  //set all uart global interrupt enable
  if(!periph_exuart_verify_wksetgreg(exuart_t,WK_GREG_GIER,0x0f)){
    goto wkeror;
  }
  //set all uart receive timeout and fifo tiger interrupted enable
  if(!periph_exuart_verify_wksetsreg(exuart_t,EXUART1_ID,WK_SREG_SIER,0x03)){
    goto wkeror;
  }
  if(!periph_exuart_verify_wksetsreg(exuart_t,EXUART2_ID,WK_SREG_SIER,0x03)){
    goto wkeror;
  }
  if(!periph_exuart_verify_wksetsreg(exuart_t,EXUART3_ID,WK_SREG_SIER,0x03)){
    goto wkeror;
  }
  if(!periph_exuart_verify_wksetsreg(exuart_t,EXUART4_ID,WK_SREG_SIER,0x03)){
    goto wkeror;
  }
  //set all uart receive tranfer fifo enable and reset
  periph_exuart_wksetsreg(exuart_t, EXUART1_ID, WK_SREG_FCR, 0x0f);
  periph_exuart_wksetsreg(exuart_t, EXUART2_ID, WK_SREG_FCR, 0x0f);
  periph_exuart_wksetsreg(exuart_t, EXUART3_ID, WK_SREG_FCR, 0x0f);
  periph_exuart_wksetsreg(exuart_t, EXUART4_ID, WK_SREG_FCR, 0x0f);
  //switch to page1 
  if(!periph_exuart_verify_wksetsreg(exuart_t,EXUART1_ID,WK_SREG_SPAGE,0x01)){
    goto wkeror;
  }
  if(!periph_exuart_verify_wksetsreg(exuart_t,EXUART2_ID,WK_SREG_SPAGE,0x01)){
    goto wkeror;
  }
  if(!periph_exuart_verify_wksetsreg(exuart_t,EXUART3_ID,WK_SREG_SPAGE,0x01)){
    goto wkeror;
  }
  if(!periph_exuart_verify_wksetsreg(exuart_t,EXUART4_ID,WK_SREG_SPAGE,0x01)){
    goto wkeror;
  }
  //set receive and tranfer trigr control
  if(!periph_exuart_verify_wksetsreg(exuart_t,EXUART1_ID,WK_SREG_RFTL,0xff)){
    goto wkeror;
  }
  if(!periph_exuart_verify_wksetsreg(exuart_t,EXUART1_ID,WK_SREG_TFTL,0x04)){
    goto wkeror;
  }
  if(!periph_exuart_verify_wksetsreg(exuart_t,EXUART2_ID,WK_SREG_RFTL,0xff)){
    goto wkeror;
  }
  if(!periph_exuart_verify_wksetsreg(exuart_t,EXUART2_ID,WK_SREG_TFTL,0x04)){
    goto wkeror;
  }
  if(!periph_exuart_verify_wksetsreg(exuart_t,EXUART3_ID,WK_SREG_RFTL,0xff)){
    goto wkeror;
  }
  if(!periph_exuart_verify_wksetsreg(exuart_t,EXUART3_ID,WK_SREG_TFTL,0x04)){
    goto wkeror;
  }
  if(!periph_exuart_verify_wksetsreg(exuart_t,EXUART4_ID,WK_SREG_RFTL,0xff)){
    goto wkeror;
  }
  if(!periph_exuart_verify_wksetsreg(exuart_t,EXUART4_ID,WK_SREG_TFTL,0x04)){
    goto wkeror;
  }
  //set bd_rate and  all uart receive and tranfer enble
  if(!periph_exuart_wksetbdrate(exuart_t, EXUART1_ID, exuart1_bdrate)){
    goto wkeror;
  }
  if(!periph_exuart_wksetbdrate(exuart_t, EXUART2_ID, exuart2_bdrate)){
    goto wkeror;
  } 
  if(!periph_exuart_wksetbdrate(exuart_t, EXUART3_ID, exuart3_bdrate)){
    goto wkeror;
  }
  if(!periph_exuart_wksetbdrate(exuart_t, EXUART4_ID, exuart4_bdrate)){
    goto wkeror;
  }
  return true;
wkeror:
  return false;  
}

/* -----------------------------------INT IQN---------------------------------*/

/**
* @brief  M1 int gpio interpurt handler.  
* @param  
*/
void EXUART_M1_INT_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(EXUART_M1_INT_PIN);
}

/**
* @brief  M2 int gpio interpurt handler.  
* @param  
*/
void EXUART_M2_INT_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(EXUART_M2_INT_PIN);
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint8_t wk_gifr = 0,dec = 0;
  if (GPIO_Pin == EXUART_M1_INT_PIN){
    wk_gifr = periph_exuart_wkgetgreg(&exuart[EXUART_M1], WK_GREG_GIFR);
    dec = 5;
    do{
      dec--;
      if(exuart[EXUART_M1].export_t[EXUART1_ID] != NULL && (wk_gifr&WK_UT1INT)){
        periph_exuart_rx_buff(&exuart[EXUART_M1], EXUART1_ID); 
      }
      if(exuart[EXUART_M1].export_t[EXUART2_ID] != NULL && (wk_gifr&WK_UT2INT)){
        periph_exuart_rx_buff(&exuart[EXUART_M1], EXUART2_ID); 
      }
      if(exuart[EXUART_M1].export_t[EXUART3_ID] != NULL && (wk_gifr&WK_UT3INT)){
        periph_exuart_rx_buff(&exuart[EXUART_M1], EXUART3_ID); 
      }
      if(exuart[EXUART_M1].export_t[EXUART4_ID] != NULL && (wk_gifr&WK_UT4INT)){
        periph_exuart_rx_buff(&exuart[EXUART_M1], EXUART4_ID); 
      }
      wk_gifr = periph_exuart_wkgetgreg(&exuart[EXUART_M1], WK_GREG_GIFR);
    }while((wk_gifr&0x0f) && (dec != 0));
  }
  if (GPIO_Pin == EXUART_M2_INT_PIN){
    wk_gifr = periph_exuart_wkgetgreg(&exuart[EXUART_M2], WK_GREG_GIFR);
    dec = 5;
    do{
      dec--;
      if(exuart[EXUART_M2].export_t[EXUART1_ID] != NULL && (wk_gifr&WK_UT1INT)){
        periph_exuart_rx_buff(&exuart[EXUART_M2], EXUART1_ID); 
      }
      if(exuart[EXUART_M2].export_t[EXUART2_ID] != NULL && (wk_gifr&WK_UT2INT)){
        periph_exuart_rx_buff(&exuart[EXUART_M2], EXUART2_ID); 
      }
      if(exuart[EXUART_M2].export_t[EXUART3_ID] != NULL && (wk_gifr&WK_UT3INT)){
        periph_exuart_rx_buff(&exuart[EXUART_M2], EXUART3_ID); 
      }
      if(exuart[EXUART_M2].export_t[EXUART4_ID] != NULL && (wk_gifr&WK_UT4INT)){
        periph_exuart_rx_buff(&exuart[EXUART_M2], EXUART4_ID); 
      }
      wk_gifr = periph_exuart_wkgetgreg(&exuart[EXUART_M2], WK_GREG_GIFR);
    }while((wk_gifr&0x0f) && (dec != 0));
  }
}

/********************************END OF FILE***********************************/
