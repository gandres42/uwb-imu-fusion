/*! ------------------------------------------------------------------------------------------------------------------
 * @file    lmh_uartrx.h
 * @brief   low-level module handshake (LMH) utilities to handle DWM1001 UART
 *          rx functionalities. 
 *
 *          In Makefile, interface configuration needs to be defined as:
 *          INTERFACE_NUMBER = 0
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
#ifndef _LMH_UARTRX_H_
#define _LMH_UARTRX_H_

#include <stdint.h>

void LMH_UARTRX_Init(void);

/**
 * @brief : de-initialises the UARTRX functions. 
 */
void LMH_UARTRX_DeInit(void);


/**
 * @brief : Sets the lmh_uartrx_flag 
 */
void LMH_UARTRX_Set(void);

/**
 * @brief : Clears the lmh_uartrx_flag 
 *          Needs to run before Tx the TLV request or after Rx from the UART buffer
 *          
 */
void LMH_UARTRX_Clear(void);

/**
 * @brief : gets the lmh_uartrx_flag flag
 *
 * @return lmh_uartrx_flag
 */
bool LMH_UARTRX_IsSet(void);

/**
 * @brief : Set the UARTRX time out period. 
 *
 * @param [in] timeout, UARTRX time out period in ms
 */
void LMH_UARTRX_SetTimeout(int timeout);

/**
 * @brief : Used to wait for TLV response after Tx the TLV request
 *          wait length=exp_length for max time=lmh_uartrx_timeout        
 *          Needs LMH_UARTRX_Init() at initialization 
 *          Needs LMH_UARTRX_Reset() before Tx the TLV request
 *
 * @param [out] data,       pointer to received data 
 * @param [out] length,     pointer to received data length 
 * @param [in] exp_length,  expected data length
 *
 * @return Error code
 */
int  LMH_UARTRX_WaitForRx(uint8_t* data, uint16_t* length, uint16_t exp_length);


#endif //_LMH_UARTRX_H_

