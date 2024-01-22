/*! ------------------------------------------------------------------------------------------------------------------
 * @file    lmh_spirx.c
 * @brief   low-level module handshake (LMH) utilities to handle DWM1001 data 
 *          transmission and receiving over SPI interface
 *          Use LMH_SPIRX_Init() before using to initialize the utilities. 
 *          Use LMH_SPIRX_WaitForRx() to wait for response message
 *
 *          In Makefile, interface configuration needs to be defined as:
 *          INTERFACE_NUMBER = 1
 *
 *          This file describes the RX setup example. 
 *
 * @attention
 *
 * Copyright 2017 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "hal.h"
#include "hal_log.h"
#include "lmh.h"

#define LMH_SPIRX_HEADER_LENGTH           1
#define LMH_SPIRX_SIZE_OFFSET             0
#if LMH_SPIRX_HEADER_LENGTH == 2
#define LMH_SPIRX_NUM_OFFSET              1
#endif   

#define LMH_SPIRX_TIMEOUT_DEFAULT         1000

static bool lmh_spirx_initialized[2]={false, false};
static int  lmh_spirx_timeout = LMH_SPIRX_TIMEOUT_DEFAULT;
static int  lmh_spirx_wait = HAL_SPI_WAIT_PERIOD;

/**
 * @brief : initialises the SPIRX functions. 
 */
void LMH_SPIRX_Init(void)
{  
   int dev = HAL_SPI_Which();
   if(dev > 1)
   {
      HAL_Log("lmh: *** ERROR *** SPI: Cannot find SPI dev%d.\n", dev);      
      return;
   }
   if(lmh_spirx_initialized[dev])
   {
      HAL_Log("lmh:     SPI%d: LMH_SPIRX already initialized.\n", dev);      
      return;
   }
   HAL_SPI_Init();
   
   LMH_SPIRX_SetTimeout(LMH_SPIRX_TIMEOUT_DEFAULT);
   LMH_SPIRX_SetWait(HAL_SPI_WAIT_PERIOD);
   LMH_SPIRX_SetToIdle();
   
   lmh_spirx_initialized[dev] = true;
   
   HAL_Log("lmh:     SPI: LMH_SPIRX_Init for SPI dev%d done.\n", dev);     
}

/**
 * @brief : de-initialises the SPIRX functions. 
 */
void LMH_SPIRX_DeInit(void)
{
   lmh_spirx_initialized[HAL_SPI_Which()] = false;
}

/**
 * @brief : sets the DWM1001 module SPIRX functions into idle mode. 
 */
void LMH_SPIRX_SetToIdle(void)
{
   uint8_t dummy = 0xff, length = 1;
   int i = 3;
   HAL_Log("lmh:     SPI%d: Reseting DWM1001 to SPI:IDLE\n", HAL_SPI_Which()); 
   while(i-- > 0){
      HAL_SPI_Tx(&dummy, &length);
      HAL_Delay(lmh_spirx_wait);
      HAL_Log("lmh:     SPI%d: Wait %d ms...\n", HAL_SPI_Which(), lmh_spirx_wait); 
   }
}

/**
 * @brief : Set the SPIRX time out period. 
 *
 * @param [in] timeout, SPIRX time out period in ms
 */
void LMH_SPIRX_SetTimeout(int timeout)
{
   lmh_spirx_timeout = timeout;
}

/**
 * @brief : Set the SPIRX wait period between each poll of SIZE. 
 *
 * @param [in] wait, SPIRX wait period in ms
 */
void LMH_SPIRX_SetWait(int wait)
{
   lmh_spirx_wait = wait;
}

/**
 * @brief : wait length=exp_length for max time=lmh_spirx_wait
 *          needs LMH_SPIRX_Init() at initialization 
 *
 * @param [out] data,       pointer to received data 
 * @param [out] length,     pointer to received data length 
 * @param [in] exp_length,  expected data length
 *
 * @return Error code
 */
int LMH_SPIRX_WaitForRx(uint8_t* data, uint16_t* length, uint16_t exp_length)
{
   uint8_t len_header, sizenum[LMH_SPIRX_HEADER_LENGTH];
   int timeout = lmh_spirx_timeout;

   int dev = HAL_SPI_Which();
   if(!lmh_spirx_initialized[dev])
   {
      HAL_Log("lmh: *** ERROR *** SPI%d: LMH_SPIRX not initialized.\n", dev);
      return LMH_ERR;
   }
   if(exp_length < DWM1001_TLV_RET_VAL_MIN_SIZE)
   {
      HAL_Log("lmh: *** ERROR *** SPI%d: exp_length must be >= 3.\n", dev);    
      return LMH_ERR;
   }
   
   HAL_Log("lmh:     SPI%d: Rx step 1:\n", dev);
   memset(sizenum, 0, LMH_SPIRX_HEADER_LENGTH);
   while((sizenum[LMH_SPIRX_SIZE_OFFSET] == 0) && (timeout>0))
   {
      HAL_Delay(lmh_spirx_wait);
      timeout-=lmh_spirx_wait;
      HAL_Log("lmh:     SPI%d: Wait %d ms...\n", dev, lmh_spirx_wait); 
      
      len_header = LMH_SPIRX_HEADER_LENGTH;
      HAL_SPI_Rx(sizenum, &len_header);
   }
   
   if((timeout < 0) && (sizenum[LMH_SPIRX_SIZE_OFFSET] == 0)){
      HAL_Log("lmh: *** ERROR *** SPI%d: Read SIZE timed out after %d ms...\n", dev, lmh_spirx_timeout);  
        
      return LMH_ERR;
   }
   
   *length = 0;
   HAL_Log("lmh:     SPISPI%d: Rx step 2:\n", dev);
   
#if LMH_SPIRX_HEADER_LENGTH == 2
   uint8_t i;
   for(i = 0; i < sizenum[LMH_SPIRX_NUM_OFFSET]; i++)
#endif
   {
      HAL_Delay(lmh_spirx_wait);
      timeout-=lmh_spirx_wait;
      HAL_Log("lmh:     SPI%d: Wait %d ms...\n", dev, lmh_spirx_wait); 
      HAL_SPI_Rx(data, sizenum+LMH_SPIRX_SIZE_OFFSET);      
      *length += sizenum[LMH_SPIRX_SIZE_OFFSET];
   }
     
   HAL_Delay(lmh_spirx_wait);
   HAL_Log("lmh:     SPI%d: Wait %d ms...\n", dev, lmh_spirx_wait); 
   
   if(LMH_CheckRetVal(data) != LMH_OK)
   {
      return LMH_ERR;
   }
   
   if((*length != exp_length) && (exp_length != DWM1001_TLV_MAX_SIZE))
   {
      HAL_Log("lmh: *** ERROR *** SPI%d: Expecting %d bytes, received %d bytes, in %d ms\n", \
      dev, exp_length, *length, lmh_spirx_timeout-timeout);
      return LMH_ERR;
   }
   
   HAL_Log("lmh:     SPI%d: Received %d bytes, in %d ms \t OK\n", dev, *length, lmh_spirx_timeout-timeout);
   return LMH_OK;      
}










