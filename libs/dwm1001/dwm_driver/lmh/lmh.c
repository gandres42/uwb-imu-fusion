/*! ------------------------------------------------------------------------------------------------------------------
 * @file    lmh.c
 * @brief   low-level module handshake (LMH) utilities to handle DWM1001 data 
 *          transmission and receiving.
 *          Use LMH_Init() before using to initialize the utilities. 
 *          Use LMH_Tx() to send request message 
 *          Use LMH_WaitForRx() to wait for response message
 *
 * @attention
 *
 * Copyright 2017 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */
#include <stdint.h>
#include "dwm_api.h"
#include "lmh.h"
#include "hal.h"
#include "hal_log.h"

/** 
 * @brief initializes the LMH utilities over defined interface
 *
 * @param none
 *
 * @return none
 */
void LMH_Init(void)
{
#if INTERFACE_NUMBER == 0
   HAL_Log("lmh:     LMH_UARTRX_Init()...\n");  
   LMH_UARTRX_Init();
#elif INTERFACE_NUMBER == 1
   HAL_Log("lmh:     LMH_SPIRX_Init(): SPI dev%d...\n", HAL_SPI_Which());  
   LMH_SPIRX_Init();
#elif INTERFACE_NUMBER == 2
   HAL_Log("lmh:     LMH_SPIRX_DRDY_Init(): SPI dev%d...\n", HAL_SPI_Which());  
   LMH_SPIRX_DRDY_Init();
#endif
}

/** 
 * @brief de-initializes the LMH utilities over defined interface
 *
 * @param none
 *
 * @return none
 */
void LMH_DeInit(void)
{
#if INTERFACE_NUMBER == 0
   HAL_Log("lmh:     LMH_UARTRX_DeInit()...\n");  
   LMH_UARTRX_DeInit();
#elif INTERFACE_NUMBER == 1
   HAL_Log("lmh:     LMH_SPIRX_DeInit(): SPI dev%d...\n", HAL_SPI_Which());  
   LMH_SPIRX_DeInit();
#elif INTERFACE_NUMBER == 2
   HAL_Log("lmh:     LMH_SPIRX_DRDY_DeInit(): SPI dev%d...\n", HAL_SPI_Which());  
   LMH_SPIRX_DRDY_DeInit();
#endif
}

/** 
 * @brief transmit data over defined interface
 *
 * @param [in] data: pointer to the Tx data buffer
 * @param [in] length: length of data to be received
 *
 * @return Error code
 */
int LMH_Tx(uint8_t* data, uint8_t* length)
{
   return HAL_IF_Tx(data, length);
}

/** 
 * @brief wait for response data over defined interface
 *       note: this function is blocking 
 *
 * @param [out] data: pointer to the RX data buffer
 * @param [out] length: length of data to be received
 * @param [in] exp_length: expected length of response data. 
 *             Note - If the user doesn't know how long the response from DWM1001 to the host 
 *                   is, then this parameter should be set to DWM1001_TLV_MAX_SIZE as defined
 *                   in dwm1001.h. In this case,
 *                   for SPI, length check won't report error no matter how long the received
 *                   data is;
 *                   for UART, this function will not return until the timeout period expires. 
 *
 * @return Error code
 */
int LMH_WaitForRx(uint8_t* data, uint16_t* length, uint16_t exp_length)
{
#if INTERFACE_NUMBER == 0
   return LMH_UARTRX_WaitForRx(data, length, exp_length);
#elif INTERFACE_NUMBER == 1
   return LMH_SPIRX_WaitForRx(data, length, exp_length);
#elif INTERFACE_NUMBER == 2
   return LMH_SPIRX_DRDY_WaitForRx(data, length, exp_length);
#endif
}

/**
 * @brief check if GPIO pin: index is among available pins
 *
 * @param[in] idx, GPIO pin index
 *
 * @return Error code
 */
int LMH_CheckRetVal(uint8_t* ret_val)
{
   if(ret_val[0] != DWM1001_TLV_TYPE_RET_VAL)
   {
      HAL_Log("lmh: *** ERROR *** %s: RET_VAL type wrong: %d\n", HAL_IF_STR, ret_val[0]);    
      return LMH_ERR;
   }
   if(ret_val[1] != 1)
   {
      HAL_Log("lmh: *** ERROR *** %s: RET_VAL length wrong: %d\n", HAL_IF_STR, ret_val[1]);    
      return LMH_ERR;
   }
   
   if(ret_val[2] == RV_OK)
   { 
      return LMH_OK;
   }   
   else
   {
      HAL_Log("lmh: *** ERROR *** %s: DWM1001_RV_ERR: %d\n", HAL_IF_STR, ret_val[2]);    
      return LMH_ERR;
   }
}
   
/** 
 * @brief wait for response data over defined interface
 *       note: this function is blocking 
 *
 * @param [in] ret_val: pointer to the response data buffer, where the first three bytes must 
 *       be TLV values 0x40, 0x01, 0x00 meaning a RV_OK, to indicating that the request is 
 *       properly parsed. Otherwise the previous communication between the host and DWM1001 
 *       was not acting correctly.
 *
 * @return Error code
 */
int LMH_CheckGPIOIdx(dwm_gpio_idx_t idx)
{
   if((idx == 2 ) || (idx == 8 ) || (idx == 9 ) || (idx == 10) || (idx == 12)  \
   || (idx == 13) || (idx == 14) || (idx == 15) || (idx == 22) || (idx == 23)  \
   || (idx == 27) || (idx == 30) || (idx == 31))
   {
      return LMH_OK; //good
   } 
   else
   {
      return LMH_ERR; //error
   }
}





