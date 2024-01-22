/*! ------------------------------------------------------------------------------------------------------------------
 * @file    lmh_spirx.h
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
#ifndef _LMH_SPIRX_H_
#define _LMH_SPIRX_H_

#include <stdint.h>

/**
 * @brief : initialises the SPIRX functions. 
 */
void LMH_SPIRX_Init(void);

/**
 * @brief : de-initialises the SPIRX functions. 
 */
void LMH_SPIRX_DeInit(void);

/**
 * @brief : sets the DWM1001 module SPIRX functions into idle mode. 
 */
void LMH_SPIRX_SetToIdle(void);

/**
 * @brief : Set the SPIRX time out period. 
 *
 * @param [in] timeout, SPIRX time out period in ms
 */
void LMH_SPIRX_SetTimeout(int timeout);

/**
 * @brief : Set the SPIRX wait period between each poll of SIZE. 
 *
 * @param [in] wait, SPIRX wait period in ms
 */
void LMH_SPIRX_SetWait(int wait);

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
int  LMH_SPIRX_WaitForRx(uint8_t* data, uint16_t* length, uint16_t exp_length);


#endif //_LMH_SPIRX_H_

