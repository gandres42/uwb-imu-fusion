/*! ------------------------------------------------------------------------------------------------------------------
 * @file    lmh_uartrx.c
 * @brief   low-level module handshake (LMH) utilities to handle DWM1001 SPI
 *          rx functionalities with DataReady pin enabled (DRDY). 
 *
 *          In Makefile, interface configuration needs to be defined as:
 *          INTERFACE_NUMBER = 2
 *
 *          This file describes the RX setup example. 
 
 * @attention
 *
 * Copyright 2017 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */
#ifndef _LMH_SPIRX_DRDY_H_
#define _LMH_SPIRX_DRDY_H_

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief : initialises the UARTRX functions. 
 */
void LMH_SPIRX_DRDY_Init(void);

/**
 * @brief : de-initialises the UARTRX functions. 
 */
void LMH_SPIRX_DRDY_DeInit(void);

/**
 * @brief : sets the DWM1001 module SPIRX functions into idle mode. 
 */
void LMH_SPIRX_DRDY_SetToIdle(void);

/**
 * @brief : update lmh_spirx_drdy_drdy_flag status according to GPIO_DRDY status
 *
 * @return none
 */
void LMH_SPIRX_DRDY_PinCheck(void);

/**
 * @brief : get lmh_spirx_drdy_drdy_flag 
 *
 * @return lmh_spirx_drdy_drdy_flag
 */
bool LMH_SPIRX_DRDY_PinGet(void);

/**
 * @brief : Set the SPIRX_DRDY time out period. 
 *
 * @param [in] timeout, SPIRX_DRDY time out period in ms
 */
void LMH_SPIRX_DRDY_SetTimeout(int timeout);

/**
 * @brief : Set the SPIRX_DRDY wait period between each poll of SIZE. 
 *
 * @param [in] wait, SPIRX_DRDY wait period in ms
 */
void LMH_SPIRX_DRDY_SetWait(int wait);

/**
 * @brief : wait length=exp_length for max time=lmh_spirx_drdy_wait
 *          needs LMH_SPIRX_DRDY_Init() at initialization 
 *
 * @param [out] data,       pointer to received data 
 * @param [out] length,     pointer to received data length 
 * @param [in] exp_length,  expected data length
 *
 * @return Error code
 */
int  LMH_SPIRX_DRDY_WaitForRx(uint8_t* data, uint16_t* length, uint16_t exp_length);


#endif //_LMH_SPIRX_DRDY_H_

