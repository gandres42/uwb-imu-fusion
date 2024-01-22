/*! ------------------------------------------------------------------------------------------------------------------
 * @file    lmh.h
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
#ifndef _LMH_H_
#define _LMH_H_

#include <stdint.h>
#include "dwm_api.h"
#include "hal.h"
#include "hal_interface.h"

#define LMH_OK    HAL_OK
#define LMH_ERR   HAL_ERR

#if INTERFACE_NUMBER == 0
#include "lmh_uartrx.h"
#elif INTERFACE_NUMBER == 1
#include "lmh_spirx.h"
#elif INTERFACE_NUMBER == 2
#include "lmh_spirx.h"
#include "lmh_spirx_drdy.h"
#endif  


/** 
 * @brief initializes the LMH utilities over defined interface
 *
 * @param none
 *
 * @return none
 */
void LMH_Init(void);


/** 
 * @brief de-initializes the LMH utilities over defined interface
 *
 * @param none
 *
 * @return none
 */
void LMH_DeInit(void);

/** 
 * @brief transmit data over defined interface
 *
 * @param [in] data: pointer to the Tx data buffer
 * @param [in] length: length of data to be received
 *
 * @return Error code
 */
int  LMH_Tx(uint8_t* data, uint8_t* length);

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
int  LMH_WaitForRx(uint8_t* data, uint16_t* length, uint16_t exp_length);

/**
 * @brief check if GPIO pin: index is among available pins
 *
 * @param[in] idx, GPIO pin index
 *
 * @return Error code
 */
int LMH_CheckGPIOIdx(dwm_gpio_idx_t idx);

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
int LMH_CheckRetVal(uint8_t* ret_val);


#endif //_LMH_H_

