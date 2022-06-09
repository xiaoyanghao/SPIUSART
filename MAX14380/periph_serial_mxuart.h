/**
  ******************************************************************************
  * @file    periph_serial_mxuart.h
  * @author  zhy
  * @brief   This file contains all the functions prototypes for the extend uart
  *          config driver.
  ******************************************************************************
  * @attention   None
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PERIPH_SERIAL_MXUART_H
#define PERIPH_SERIAL_MXUART_H

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
   
   MXUART_M1,           
   MXUART_M2,   
   MXUART_MAX, 
   
}MXUART_MODULE;
 
typedef enum
{
   
   MXUART1_ID,           
   MXUART2_ID,         
   MXUART3_ID, 
   MXUART4_ID,           
   MXUARTMAX_ID, 
   
}MXUART_ID;
 
/**
  * @brief  Uart sturct
  */ 
typedef struct
{
  
  uint8_t            mxuart_module;
  uint8_t            mxuart_id;
  /* FIFO */
  uint8_t 	     *pRxBuf;		/* Data receive buffer */
  uint16_t 	     usRxBufSize;       /* length of receive buff */

  __IO uint16_t      usRxHead;		/*!<write index of receive buff */
  __IO uint16_t      usRxTail;		/*!< read index of receive buff */
  __IO uint16_t      usRxCount;       /*!< count of data to be read */
  bool               rxComplete;
  bool               rxOverflow;
  __IO uint32_t      rxlastupdatems; 
  
}MXPORT_T;


/**
* @}
*/

/* Peripheral Control functions  ************************************************/

void Periph_Mxuart_Serial_Open(uint8_t mxuart_module, uint32_t mxuart1_bdrate, \
                               uint32_t mxuart2_bdrate, uint32_t mxuart3_bdrate, \
                                                        uint32_t mxuart4_bdrate);
void Periph_Mxuart_Serial_Register(uint8_t mxuart_module, uint8_t mxuart_id, \
                                   MXPORT_T *mxport_t,  uint8_t *rx_buff, \
                                                           uint16_t rx_buff_size);
void Periph_Mxuart_Serial_Receive(uint8_t  mxuart_module);

#ifdef __cplusplus
}
#endif



#endif 

/********************************END OF FILE***********************************/
