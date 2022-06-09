/**
  ******************************************************************************
  * @file    periph_serial_mxuart.c
  * @author  zhy
  * @brief   extend uart max14830 config.
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
  * @attention  1)Since receive triggers can only be multiples of 8 bytes, 
                polling receive is used
               2) SPI CLOCK MAX 26MHZ
               3) rend 128 byte time < 1ms
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "periph_serial_mxuart.h"  

/* Private typedef -----------------------------------------------------------*/
/** @defgroup extend uart Types
* @{
*/  

/**
* @brief  Struct def of max14380 SPI read
*/   
typedef struct 
{
  
  SpiClient_t *client;
  volatile uint32_t spiFlag;
  volatile uint64_t lastUpdate;
  uint8_t error;
  uint8_t enabled;
  MXPORT_T *mxport_t[MXUARTMAX_ID];
  
} Mxuart_Struct_T;  

/**
* @}
*/  

/* Private define ------------------------------------------------------------*/
/* --------------------------MAX14380 system frequency define ----------------*/
/**
* @note  unit hz (3.6864MHZ)
*/ 
#define  MX_SYS_FREQUENCY           (3686400) 
#define  MX_SET_GREG(greg)          (0X80|greg)
#define  MX_GET_GREG(greg)          (0X00|greg)

/**
* @note  uartx 0-3
*/ 
#define  MX_SET_SREG(uartx,sreg)     (0X80|(uartx<<5)|sreg)
#define  MX_GET_SREG(uartx,sreg)     (0X00|(uartx<<5)|sreg)

/**
* @note  max fifo size 128 byte
*/ 
#define Mx_FIFO_SIZE        128

/* --------------------------MAX14380 register define ------------------------*/
/* --------------------------Global register define --------------------------*/
#define MX_GREG_IRQ               (0X1F)
#define MX_GREG_PLL               (0X1A)
#define MX_GREG_CLK               (0X1E) 

/* --------------Global register receive interrupt bitdefines ----------------*/
#define MX_GREG_IRQ0_BIT          (0X01 << 0)
#define MX_GREG_IRQ1_BIT          (0X01 << 1)
#define MX_GREG_IRQ2_BIT          (0X01 << 2) 
#define MX_GREG_IRQ3_BIT          (0X01 << 3)

/* --------------------------Sub-serial control register defines -------------*/
#define MX_SREG_RHR               (0X00)
#define MX_SREG_IRQ               (0X01)
#define MX_SREG_ISR               (0X02)
#define MX_SREG_MODE1             (0X09)
#define MX_SREG_MODE2             (0X0A)
#define MX_SREG_LCR               (0X0B)
#define MX_SREG_FIFOTRGLV         (0X10)
#define MX_SREG_RXFIFOLV          (0X12)
#define MX_SREG_GPIOCONFIG        (0X18)
#define MX_SREG_GPIODATA          (0X19)

#define MX_SREG_BRG               (0X1B) 
#define MX_SREG_DIVLSB            (0X1C) 
#define MX_SREG_DIVMSB            (0X1D) 

/* ---------Sub-serial control register receive interrupt bitdefines ---------*/
#define MX_RFTRIG_INT             (0x01 << 4)
#define MX_RXEMPTY                (0x01 << 6)

/* --------------------------Connect gpio define -----------------------------*/
#define MXUART_M1_CS_CLK_ENABLE()          __HAL_RCC_GPIOB_CLK_ENABLE()
#define MXUART_M1_CS_PORT                   GPIOB
#define MXUART_M1_CS_PIN                    GPIO_PIN_12
#define MXUART_M1_RST_CLK_ENABLE()          __HAL_RCC_GPIOA_CLK_ENABLE()
#define MXUART_M1_RST_PORT                  GPIOA
#define MXUART_M1_RST_PIN                   GPIO_PIN_3 

#define MXUART_M2_CS_CLK_ENABLE()          __HAL_RCC_GPIOG_CLK_ENABLE()
#define MXUART_M2_CS_PORT                   GPIOG
#define MXUART_M2_CS_PIN                    GPIO_PIN_11
#define MXUART_M2_RST_CLK_ENABLE()          __HAL_RCC_GPIOG_CLK_ENABLE()
#define MXUART_M2_RST_PORT                  GPIOB
#define MXUART_M2_RST_PIN                   GPIO_PIN_4 

/* Private variables ---------------------------------------------------------*/
static Mxuart_Struct_T mxuart[MXUART_MAX] = {0};
#pragma location = 0x24003990
static ALIGN_32BYTES (uint8_t mx_conftxbuf[4]) = {0};
#pragma location = 0x240039D0
static ALIGN_32BYTES (uint8_t mx_confrxbuf[4]) = {0};
#pragma location = 0x24003A20
static ALIGN_32BYTES (uint8_t mx_rxbuf[Mx_FIFO_SIZE + 4]) = {0};
#pragma location = 0x24003AB0
static ALIGN_32BYTES (uint8_t mx_txbuf[Mx_FIFO_SIZE + 4]) = {0};

/* Private function prototypes -----------------------------------------------*/
static void periph_mxuart_gpio_init(uint8_t mxuart_module);
static void periph_mxuart_rst(uint8_t mxuart_module);
static bool periph_mxuart_config(Mxuart_Struct_T *mxuart_t, uint32_t mxuart1_bdrate, \
                                   uint32_t mxuart2_bdrate, uint32_t mxuart3_bdrate, \
                                                          uint32_t mxuart4_bdrate);
static void periph_mxuart_setgreg(Mxuart_Struct_T *mxuart_t, uint8_t greg, \
                                                                   uint8_t value);
static uint8_t periph_mxuart_getgreg(Mxuart_Struct_T *mxuart_t, uint8_t greg);
static bool periph_mxuart_verify_setgreg(Mxuart_Struct_T *mxuart_t, uint8_t greg, \
                                                                  uint8_t value);
static void periph_mxuart_setsreg(Mxuart_Struct_T *mxuart_t, uint8_t uartx, \
                                                    uint8_t sreg, uint8_t value);
static uint8_t periph_mxuart_getsreg(Mxuart_Struct_T *mxuart_t,uint8_t uartx, \
                                                                    uint8_t sreg);
static bool periph_mxuart_verify_setsreg(Mxuart_Struct_T *mxuart_t, uint8_t uartx, \
                                                     uint8_t sreg, uint8_t value);
static bool periph_mxuart_setbdrate(Mxuart_Struct_T *mxuart_t, uint8_t uartx,
                                                               uint32_t bd_rate);
static void periph_mxuart_getsfifo(Mxuart_Struct_T *mxuart_t, uint8_t uartx,uint16_t len);

/* Private functions ---------------------------------------------------------*/
/**
* @brief  mxuart init.  
* @param  mxuart_module MXUART_MODULE
          bd_rate Baud rate 
* @retval None    
*/
void Periph_Mxuart_Serial_Open(uint8_t mxuart_module, uint32_t mxuart1_bdrate, \
                               uint32_t mxuart2_bdrate, uint32_t mxuart3_bdrate, \
                                                        uint32_t mxuart4_bdrate)
{
  
  switch (mxuart_module){
    case MXUART_M1:
         MXUART_M1_CS_CLK_ENABLE();
         periph_mxuart_gpio_init(mxuart_module);
         periph_mxuart_rst(mxuart_module);
         mxuart[mxuart_module].client = Bsp_Spi_Serial_Open(SPI2_ID, SPI_BAUDRATEPRESCALER_4, \
                                        SPI_MO, MXUART_M1_CS_PORT, MXUART_M1_CS_PIN, \
                                        &mxuart[mxuart_module].spiFlag, NULL);
         
         
         if(!periph_mxuart_config(&mxuart[mxuart_module],mxuart1_bdrate, mxuart2_bdrate, \
                                                mxuart3_bdrate, mxuart4_bdrate)){
             mxuart[mxuart_module].error = 1; 
         }
         else{
           mxuart[mxuart_module].mxport_t[MXUART1_ID] = NULL;
           mxuart[mxuart_module].mxport_t[MXUART2_ID] = NULL;
           mxuart[mxuart_module].mxport_t[MXUART3_ID] = NULL;
           mxuart[mxuart_module].mxport_t[MXUART4_ID] = NULL;
           mxuart[mxuart_module].enabled = 1; 
         }
    break;
    case MXUART_M2:
         MXUART_M2_CS_CLK_ENABLE();
         periph_mxuart_gpio_init(mxuart_module);
         periph_mxuart_rst(mxuart_module);
         mxuart[mxuart_module].client = Bsp_Spi_Serial_Open(SPI6_ID, SPI_BAUDRATEPRESCALER_4, \
                                        SPI_MO, MXUART_M2_CS_PORT, MXUART_M2_CS_PIN, \
                                        &mxuart[mxuart_module].spiFlag, NULL);
         
         
         if(!periph_mxuart_config(&mxuart[mxuart_module],mxuart1_bdrate, mxuart2_bdrate, \
                                                mxuart3_bdrate, mxuart4_bdrate)){
             mxuart[mxuart_module].error = 1; 
         }
         else{
           mxuart[mxuart_module].mxport_t[MXUART1_ID] = NULL;
           mxuart[mxuart_module].mxport_t[MXUART2_ID] = NULL;
           mxuart[mxuart_module].mxport_t[MXUART3_ID] = NULL;
           mxuart[mxuart_module].mxport_t[MXUART4_ID] = NULL;
           mxuart[mxuart_module].enabled = 1; 
         } 
    break; 
    default:
    break;
  }
}

/* @brief   Register a serial uart.  
* @param   mxuart_module MXUART_MODULE
           mxuart_id MXUART_ID
* @retval None   
* @note   rx_buff_size <= 128  
*/
void Periph_Mxuart_Serial_Register(uint8_t mxuart_module, uint8_t mxuart_id, \
                                   MXPORT_T *mxport_t,  uint8_t *rx_buff, \
                                                           uint16_t rx_buff_size)
{
  
  mxport_t->mxuart_module      = mxuart_module;
  mxport_t->mxuart_id          = mxuart_id;
  mxport_t->pRxBuf             = rx_buff;
  mxport_t->usRxBufSize        = rx_buff_size;
  mxport_t->usRxHead           = 0;     // rx write index
  mxport_t->usRxTail           = 0;     // rx read index
  mxport_t->usRxCount          = 0;     // counter of data received
  mxport_t->rxOverflow         = false;
  mxport_t->rxComplete         = false;
  mxuart[mxuart_module].mxport_t[mxuart_id] = mxport_t;
  
}

/**
* @brief  mxuart receive   
  @param  mxuart_module  MXUART_MODULE  
* @retval  None 
* @note   put it in thread
*/
void Periph_Mxuart_Serial_Receive(uint8_t  mxuart_module)
{
  uint8_t rx_len = 0;
  if(mxuart_module < MXUART_MAX){
    rx_len = periph_mxuart_getsreg(&mxuart[mxuart_module],MXUART1_ID,MX_SREG_RXFIFOLV);
    if(rx_len > 0 && mxuart[mxuart_module].mxport_t[MXUART1_ID]->pRxBuf != NULL){    
      periph_mxuart_getsfifo(&mxuart[mxuart_module],MXUART1_ID,rx_len);
    }
    rx_len = periph_mxuart_getsreg(&mxuart[mxuart_module],MXUART2_ID,MX_SREG_RXFIFOLV);
    if(rx_len > 0 && mxuart[mxuart_module].mxport_t[MXUART2_ID]->pRxBuf != NULL){
      periph_mxuart_getsfifo(&mxuart[mxuart_module],MXUART2_ID,rx_len);
    }       
    rx_len = periph_mxuart_getsreg(&mxuart[mxuart_module],MXUART3_ID,MX_SREG_RXFIFOLV);
    if(rx_len > 0 && mxuart[mxuart_module].mxport_t[MXUART3_ID]->pRxBuf != NULL){
      periph_mxuart_getsfifo(&mxuart[mxuart_module],MXUART3_ID,rx_len);
    }        
    rx_len = periph_mxuart_getsreg(&mxuart[mxuart_module],MXUART4_ID,MX_SREG_RXFIFOLV);
    if(rx_len > 0 && mxuart[mxuart_module].mxport_t[MXUART4_ID]->pRxBuf != NULL){
      periph_mxuart_getsfifo(&mxuart[mxuart_module],MXUART4_ID,rx_len);
    } 
  }
}

/**
* @brief  mxuart gpio init.  
* @retval mxuart_module         MXUART_MODULE  
*/

static void periph_mxuart_gpio_init(uint8_t mxuart_module)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  switch (mxuart_module){
    case MXUART_M1:
    MXUART_M1_RST_CLK_ENABLE();
    
    GPIO_InitStruct.Pin = MXUART_M1_RST_PIN;  
    HAL_GPIO_Init(MXUART_M1_RST_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(MXUART_M1_RST_PORT, MXUART_M1_RST_PIN, GPIO_PIN_SET);
    
    break;
    
    case MXUART_M2:
    MXUART_M2_RST_CLK_ENABLE();
    
    GPIO_InitStruct.Pin = MXUART_M2_RST_PIN;  
    HAL_GPIO_Init(MXUART_M2_RST_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(MXUART_M2_RST_PORT, MXUART_M2_RST_PIN, GPIO_PIN_SET);
    break; 
    default:
    break;
  }
}



/**
* @brief  periph  max14380 rst.  
* @retval exuart_module EXUART_MODULE  
*/
static void periph_mxuart_rst(uint8_t mxuart_module)
{

  switch (mxuart_module){
    case MXUART_M1:
    HAL_GPIO_WritePin(MXUART_M1_RST_PORT, MXUART_M1_RST_PIN, GPIO_PIN_SET);					
    HAL_Delay(50);
    HAL_GPIO_WritePin(MXUART_M1_RST_PORT, MXUART_M1_RST_PIN, GPIO_PIN_RESET);	
    HAL_Delay(50);
    HAL_GPIO_WritePin(MXUART_M1_RST_PORT, MXUART_M1_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(50);
    
    break;
    case MXUART_M2:
    HAL_GPIO_WritePin(MXUART_M2_RST_PORT, MXUART_M2_RST_PIN, GPIO_PIN_SET);					
    HAL_Delay(50);
    HAL_GPIO_WritePin(MXUART_M2_RST_PORT, MXUART_M2_RST_PIN, GPIO_PIN_RESET);	
    HAL_Delay(50);
    HAL_GPIO_WritePin(MXUART_M2_RST_PORT, MXUART_M2_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(50);     
      
    default:
    break;
  }
}

/**
* @brief  MAX14380     write gobal register.  
* @param  mxuart_t   Mxuart_Struct_T pointer
          greg       gobal register address
          value      set value        
* @retval None    
*/
static void periph_mxuart_setgreg(Mxuart_Struct_T *mxuart_t, uint8_t greg, \
                                                                   uint8_t value)
{ 
  uint16_t timeout = 0xfff;
  mx_conftxbuf[0] = MX_SET_GREG(greg);
  mx_conftxbuf[1] = value;
  Bsp_Spi_Transaction(mxuart_t->client,&mx_confrxbuf[0],&mx_conftxbuf[0],2);
  do{
    timeout--;
  }while((mxuart_t->spiFlag == 0) && (timeout != 0));
}

/**
* @brief  MAX14380  read gobal register.  
* @param  mxuart_t   Mxuart_Struct_T pointer
          greg       gobal register address       
* @retval None    
*/  
static uint8_t periph_mxuart_getgreg(Mxuart_Struct_T *mxuart_t, uint8_t greg)
{ 

  uint16_t timeout = 0xfff;
  mx_conftxbuf[0] = MX_GET_GREG(greg);
  Bsp_Spi_Transaction(mxuart_t->client,&mx_confrxbuf[0],&mx_conftxbuf[0],2);
  do{
    timeout--;
  }while((mxuart_t->spiFlag == 0) && (timeout != 0));
  
  return mx_confrxbuf[1];
}

/**
* @brief  MAX14380     verify write gobal register.  
* @param  mxuart_t    Mxuart_Struct_T pointer
          greg       gobal register address
          value      set value        
* @retval None    
*/
static bool periph_mxuart_verify_setgreg(Mxuart_Struct_T *mxuart_t, uint8_t greg, \
                                                                  uint8_t value)
{  
  uint8_t dec = 10;
  bool ret = true;
  do {
    periph_mxuart_setgreg(mxuart_t, greg, value);
    dec--;
  } while (periph_mxuart_getgreg(mxuart_t,greg) != value && (dec != 0));
  
  if (dec == 0) {
    ret = false;
  }
  return ret;
}

/**
* @brief  MAX14380  write Sub-serial register.  
* @param  mxuart_t   Mxuart_Struct_T pointer
          sreg       Sub-serial register address
          value      set value        
* @retval None    
*/
static void periph_mxuart_setsreg(Mxuart_Struct_T *mxuart_t, uint8_t uartx, \
                                                    uint8_t sreg, uint8_t value)
{ 
 
  uint16_t timeout = 0xfff;
  mx_conftxbuf[0] = MX_SET_SREG(uartx,sreg);
  mx_conftxbuf[1] = value;
  Bsp_Spi_Transaction(mxuart_t->client, &mx_confrxbuf[0], &mx_conftxbuf[0], 2);
  do{
    timeout--;
  }while((mxuart_t->spiFlag == 0) && (timeout != 0));
}
/**
* @brief  MAX14380   read Sub-serial register.  
* @param  mxuart_t   Mxuart_Struct_T pointer
          sreg       Sub-serial register address       
* @retval None    
*/  
static uint8_t periph_mxuart_getsreg(Mxuart_Struct_T *mxuart_t,uint8_t uartx, \
                                                                    uint8_t sreg)
{ 

  uint16_t timeout = 0xfff;
  mx_conftxbuf[0] = MX_GET_SREG(uartx,sreg);
  Bsp_Spi_Transaction(mxuart_t->client, &mx_confrxbuf[0], &mx_conftxbuf[0], 2);
  do{
    timeout--;
  }while((mxuart_t->spiFlag == 0) && (timeout != 0));
  
  return mx_confrxbuf[1];
}

/**
* @brief  MAX14380   verify write Sub-serial register.  
* @param  mxuart_t   Mxuart_Struct_T pointer
          sreg       Sub-serial register address
          value      set value        
* @retval None    
*/
static bool periph_mxuart_verify_setsreg(Mxuart_Struct_T *mxuart_t, uint8_t uartx, \
                                                     uint8_t sreg, uint8_t value)
{  
  uint8_t dec = 10;
  bool ret = true;
  do {
    periph_mxuart_setsreg(mxuart_t, uartx, sreg, value);
    dec--;
  } while (periph_mxuart_getsreg(mxuart_t, uartx, sreg) != value && (dec != 0));
  
  if (dec == 0) {
    ret = false;
  }
  return ret;
}

/**
* @brief  MAX14380   uart set bd_rate 
* @param  exuart_t   Mxuart_Struct_T pointer
          uartx      mxuart_id
          bd_rate    Baud rate   
* @retval 0-failed   1-success   
*/
static bool periph_mxuart_setbdrate(Mxuart_Struct_T *mxuart_t, uint8_t uartx,
                                                               uint32_t bd_rate)
{ 
  uint8_t buad_high = 0,buad_low = 0,buad_pre = 0;
  uint16_t reg_integer = 0;
  float reg_float = 0,reg_decimal = 0;
  reg_float = (float)MX_SYS_FREQUENCY/bd_rate/16;
  reg_integer = MX_SYS_FREQUENCY/bd_rate/16;
  reg_decimal = reg_float - reg_integer;
  buad_pre = (uint8_t)(reg_decimal * 16 + 0.5) & 0x0f;
  buad_high = (reg_integer & 0xff00)>>8;
  buad_low = (reg_integer & 0x00ff);
  //set bd_rate
  if(!periph_mxuart_verify_setsreg(mxuart_t,uartx,MX_SREG_DIVMSB,buad_high)){
     goto wk_bderror;
  }
  if(!periph_mxuart_verify_setsreg(mxuart_t,uartx,MX_SREG_DIVLSB,buad_low)){
     goto wk_bderror;
  }
  if(!periph_mxuart_verify_setsreg(mxuart_t,uartx,MX_SREG_BRG,buad_pre)){
     goto wk_bderror;
  }
  return true;
wk_bderror:
  return false;

}
/**
* @brief  MAX14380 uart config.  
* @param  mxuart_t   Exuart_Struct_T pointer
          mxuart1_bdrate    uart1 bdrate
          mxuart2_bdrate    uart2 bdrate
          mxuart3_bdrate    uart3 bdrate
          mxuart4_bdrate    uart4 bdrate
* @retval 0-failed   1-success   
*/
static bool periph_mxuart_config(Mxuart_Struct_T *mxuart_t, uint32_t mxuart1_bdrate, \
                                   uint32_t mxuart2_bdrate, uint32_t mxuart3_bdrate, \
                                                          uint32_t mxuart4_bdrate)
{
// disable clock 
//  if(!periph_mxuart_verify_setsreg(mxuart_t,MXUART1_ID,MX_SREG_BRG,0x40)){
//    goto mxeror;
//  }
//  if(!periph_mxuart_verify_setsreg(mxuart_t,MXUART2_ID,MX_SREG_BRG,0x40)){
//    goto mxeror;
//  }
//  if(!periph_mxuart_verify_setsreg(mxuart_t,MXUART3_ID,MX_SREG_BRG,0x40)){
//    goto mxeror;
//  }
//  if(!periph_mxuart_verify_setsreg(mxuart_t,MXUART4_ID,MX_SREG_BRG,0x40)){
//    goto mxeror;
//  }
  //config clock source CrystalEn
  if(!periph_mxuart_verify_setgreg(mxuart_t,MX_GREG_CLK,0x0A)){
    goto mxeror;
  }
  //config clock baud rate
  if(!periph_mxuart_setbdrate(mxuart_t,MXUART1_ID,mxuart1_bdrate)){
    goto mxeror;
  }
  if(!periph_mxuart_setbdrate(mxuart_t,MXUART2_ID,mxuart2_bdrate)){
    goto mxeror;
  } 
  if(!periph_mxuart_setbdrate(mxuart_t,MXUART3_ID,mxuart3_bdrate)){
    goto mxeror;
  }
  if(!periph_mxuart_setbdrate(mxuart_t,MXUART4_ID,mxuart4_bdrate)){
    goto mxeror;
  }
  //config uart mode
  //1) config irqsel high   
//  if(!periph_mxuart_verify_setsreg(mxuart_t,MXUART1_ID,MX_SREG_MODE1,0x90)){
//    goto mxeror;
//  }
//  if(!periph_mxuart_verify_setsreg(mxuart_t,MXUART2_ID,MX_SREG_MODE1,0x90)){
//    goto mxeror;
//  }
//  if(!periph_mxuart_verify_setsreg(mxuart_t,MXUART3_ID,MX_SREG_MODE1,0x90)){
//    goto mxeror;
//  }
//  if(!periph_mxuart_verify_setsreg(mxuart_t,MXUART4_ID,MX_SREG_MODE1,0x90)){
//    goto mxeror;
//  }
  //2) config rxemty inv  
//  if(!periph_mxuart_verify_setsreg(mxuart_t,MXUART1_ID,MX_SREG_MODE2,0x08)){
//    goto mxeror;
//  }
//  if(!periph_mxuart_verify_setsreg(mxuart_t,MXUART2_ID,MX_SREG_MODE2,0x08)){
//    goto mxeror;
//  }
//  if(!periph_mxuart_verify_setsreg(mxuart_t,MXUART3_ID,MX_SREG_MODE2,0x08)){
//    goto mxeror;
//  }
//  if(!periph_mxuart_verify_setsreg(mxuart_t,MXUART4_ID,MX_SREG_MODE2,0x08)){
//    goto mxeror;
//  }
  //3) config uart 1 stop bit 8 word length
  if(!periph_mxuart_verify_setsreg(mxuart_t,MXUART1_ID,MX_SREG_LCR,0x03)){
    goto mxeror;
  }
  if(!periph_mxuart_verify_setsreg(mxuart_t,MXUART2_ID,MX_SREG_LCR,0x03)){
    goto mxeror;
  }
  if(!periph_mxuart_verify_setsreg(mxuart_t,MXUART3_ID,MX_SREG_LCR,0x03)){
    goto mxeror;
  }
  if(!periph_mxuart_verify_setsreg(mxuart_t,MXUART4_ID,MX_SREG_LCR,0x03)){
    goto mxeror;
  }
  //config irq rx not emty and rx fifotrg
//  if(!periph_mxuart_verify_setsreg(mxuart_t,MXUART1_ID,MX_SREG_FIFOTRGLV,0x1F)){
//    goto mxeror;
//  }
//  if(!periph_mxuart_verify_setsreg(mxuart_t,MXUART2_ID,MX_SREG_FIFOTRGLV,0x1F)){
//    goto mxeror;
//  }
//  if(!periph_mxuart_verify_setsreg(mxuart_t,MXUART3_ID,MX_SREG_FIFOTRGLV,0x1F)){
//    goto mxeror;
//  }
//  if(!periph_mxuart_verify_setsreg(mxuart_t,MXUART4_ID,MX_SREG_FIFOTRGLV,0x1F)){
//    goto mxeror;
//  }
//  if(!periph_mxuart_verify_setsreg(mxuart_t,MXUART1_ID,MX_SREG_IRQ,0x08)){
//    goto mxeror;
//  }
//  if(!periph_mxuart_verify_setsreg(mxuart_t,MXUART2_ID,MX_SREG_IRQ,0x08)){
//    goto mxeror;
//  }
//  if(!periph_mxuart_verify_setsreg(mxuart_t,MXUART3_ID,MX_SREG_IRQ,0x08)){
//    goto mxeror;
//  }
//  if(!periph_mxuart_verify_setsreg(mxuart_t,MXUART4_ID,MX_SREG_IRQ,0x08)){
//    goto mxeror;
//  }
  //gpio config
  if(!periph_mxuart_verify_setsreg(mxuart_t,MXUART1_ID,MX_SREG_GPIOCONFIG,0x0F)){
    goto mxeror;
  }
  if(!periph_mxuart_verify_setsreg(mxuart_t,MXUART2_ID,MX_SREG_GPIOCONFIG,0x0F)){
    goto mxeror;
  }
  if(!periph_mxuart_verify_setsreg(mxuart_t,MXUART3_ID,MX_SREG_GPIOCONFIG,0x0F)){
    goto mxeror;
  }
  if(!periph_mxuart_verify_setsreg(mxuart_t,MXUART4_ID,MX_SREG_GPIOCONFIG,0x0F)){
    goto mxeror;
  }
  if(!periph_mxuart_verify_setsreg(mxuart_t,MXUART1_ID,MX_SREG_GPIODATA,0xFF)){
    goto mxeror;
  }
  if(!periph_mxuart_verify_setsreg(mxuart_t,MXUART2_ID,MX_SREG_GPIODATA,0xFF)){
    goto mxeror;
  }
  if(!periph_mxuart_verify_setsreg(mxuart_t,MXUART3_ID,MX_SREG_GPIODATA,0xFF)){
    goto mxeror;
  }
  if(!periph_mxuart_verify_setsreg(mxuart_t,MXUART4_ID,MX_SREG_GPIODATA,0xFF)){
    goto mxeror;
  }
//   enable clock 
//  if(!periph_mxuart_verify_setsreg(mxuart_t,MXUART1_ID,MX_SREG_BRG,0x00)){
//    goto mxeror;
//  }
//  if(!periph_mxuart_verify_setsreg(mxuart_t,MXUART2_ID,MX_SREG_BRG,0x00)){
//    goto mxeror;
//  }
//  if(!periph_mxuart_verify_setsreg(mxuart_t,MXUART3_ID,MX_SREG_BRG,0x00)){
//    goto mxeror;
//  }
//  if(!periph_mxuart_verify_setsreg(mxuart_t,MXUART4_ID,MX_SREG_BRG,0x00)){
//    goto mxeror;
//  }
  
  return true;
mxeror:
  return false;  
  
}

/**
* @brief  Max14380     read Sub-serial fifo.  
* @param  mxuart_t   Mxuart_Struct_T pointer
          uartx      uart id
          rxbuff     receive  buff addr
          len        receive len      
* @note   len =< 128  
*/
static void periph_mxuart_getsfifo(Mxuart_Struct_T *mxuart_t, uint8_t uartx,uint16_t len)
{
 
  uint16_t timeout = 0xffff,i = 0;
  mx_txbuf[0] = MX_GET_SREG(uartx,MX_SREG_RHR);
  Bsp_Spi_Transaction(mxuart_t->client, &mx_rxbuf[0], &mx_txbuf[0], len+1);
  do{
    timeout--;
  }while((mxuart_t->spiFlag == 0) && (timeout != 0));
  for(i = 0; i < len; i++){
    mxuart_t->mxport_t[uartx]->pRxBuf[mxuart_t->mxport_t[uartx]->usRxHead] = mx_rxbuf[i+1];
    mxuart_t->mxport_t[uartx]->usRxHead = (mxuart_t->mxport_t[uartx]->usRxHead + 1) % mxuart_t->mxport_t[uartx]->usRxBufSize;
    if(mxuart_t->mxport_t[uartx]->usRxCount < mxuart_t->mxport_t[uartx]->usRxBufSize){
      mxuart_t->mxport_t[uartx]->usRxCount++;
    }
    else{
      mxuart_t->mxport_t[uartx]->rxOverflow = true;
    }
  }
  mxuart_t->mxport_t[uartx]->rxComplete  = true;
  mxuart_t->mxport_t[uartx]->rxlastupdatems = HAL_GetTick();
  
}

/********************************END OF FILE***********************************/
