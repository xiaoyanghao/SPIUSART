/**
  ******************************************************************************
  * @file    periph_serial_exuart.h
  * @author  zhy
  * @brief   This file contains all the functions prototypes for the extend uart
  *          config driver.
  ******************************************************************************
  * @attention   None
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PERIPH_SERIAL_EXUART_H
#define PERIPH_SERIAL_EXUART_H

#ifdef __cplusplus
 extern "C" {
#endif
   
/* Includes ------------------------------------------------------------------*/
#include "bsp_spi_drv.h" 
#include <stdbool.h>  

/* Exported types ------------------------------------------------------------*/
/** @defgroup extend uart acquisition Types
* @{
*/  
typedef enum
{
   
   EXUART_M1,           
   EXUART_M2,   
   EXUART_MAX, 
   
}EXUART_MODULE;
 
typedef enum
{
   
   EXUART1_ID,           
   EXUART2_ID,         
   EXUART3_ID, 
   EXUART4_ID,           
   EXUARTMAX_ID, 
   
}EXUART_ID;
 
/**
  * @brief  Uart sturct
  */ 
typedef struct
{
  uint8_t            exuart_module;
  uint8_t            exuart_id;
  /* FIFO */
  uint8_t 	     *pRxBuf;		/* Data receive buffer */
  uint16_t 	     usRxBufSize;       /* length of receive buff */

  __IO uint16_t      usRxHead;		/*!<write index of receive buff */
  __IO uint16_t      usRxTail;		/*!< read index of receive buff */
  __IO uint16_t      usRxCount;       /*!< count of data to be read */
  bool               rxComplete;
  bool               rxOverflow;
  __IO uint32_t      rxlastupdatems; 
}EXPORT_T;

/**
* @}
*/

/* Peripheral Control functions  ************************************************/
void Periph_Exuart_Serial_Open(uint8_t exuart_module,uint32_t exuart1_bdrate,\
                               uint32_t exuart2_bdrate,uint32_t exuart3_bdrate,\
                                                        uint32_t exuart4_bdrate);
void Periph_Exuart_Serial_Register(uint8_t exuart_module, uint8_t exuart_id,\
                                         EXPORT_T *export_t,  uint8_t *rx_buff, \
                                                           uint16_t rx_buff_size);
void Periph_Exuart_Send_Buff(EXPORT_T *export_t, uint8_t *buf, uint16_t len);


#ifdef __cplusplus
}
#endif



#endif 

/********************************END OF FILE***********************************/
