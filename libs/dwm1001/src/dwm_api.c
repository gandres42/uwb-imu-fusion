/*! ------------------------------------------------------------------------------------------------------------------
 * @file    dwm_api.h
 * @brief   DWM1001 host API header 
 *
 * @attention
 *
 * Copyright 2017 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */ 

#include "dwm_api.h"
#include <string.h>
#include <stdio.h>   /* Standard input/output definitions */ 
#include <unistd.h>  /* Linuxstandard function definitions*/ 
#include <fcntl.h>   /* File control definitions*/ 
#include <errno.h>   /* Error number definitions*/ 
#include <termios.h> /* POSIX terminal control definitions*/  

#define RESP_ERRNO_LEN           3
#define RESP_DAT_TYPE_OFFSET     RESP_ERRNO_LEN
#define RESP_DAT_LEN_OFFSET      RESP_DAT_TYPE_OFFSET+1
#define RESP_DAT_VALUE_OFFSET    RESP_DAT_LEN_OFFSET+1

dwm1001 * dwm_init(char * serial_port)
{
   int fd;
   fd = open(serial_port, O_RDWR | O_NOCTTY | O_NDELAY); 
   if (fd == -1)
   {
      perror("open_port: Unable to open /dev/ttyS0 - ");
   }
   else
   {
      fcntl(fd, F_SETFL, 0);
   }
   return (fd);   
}

void dwm_deinit(dwm1001 * dwm)
{
   LMH_DeInit();
}

int dwm_pos_set(dwm1001 * dwm, dwm_pos_t* pos)
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_POS_SET;
   tx_data[tx_len++] = 13;
   *(uint32_t*)(tx_data+tx_len) = pos->x;
   tx_len+=4;
   *(uint32_t*)(tx_data+tx_len) = pos->y;
   tx_len+=4;
   *(uint32_t*)(tx_data+tx_len) = pos->z;
   tx_len+=4;
   tx_data[tx_len++] = pos->qf;
   LMH_Tx(tx_data, &tx_len);      
   return LMH_WaitForRx(rx_data, &rx_len, 3);
}

int dwm_pos_get(dwm1001 * dwm, dwm_pos_t* p_pos)
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   uint8_t data_cnt;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_POS_GET;
   tx_data[tx_len++] = 0;   
   LMH_Tx(tx_data, &tx_len);   
   if(LMH_WaitForRx(rx_data, &rx_len, 18) == RV_OK)
   {
      data_cnt = RESP_DAT_VALUE_OFFSET;
      p_pos->x = rx_data[data_cnt] 
             + (rx_data[data_cnt+1]<<8) 
             + (rx_data[data_cnt+2]<<16) 
             + (rx_data[data_cnt+3]<<24); 
      data_cnt += 4;
      p_pos->y = rx_data[data_cnt] 
             + (rx_data[data_cnt+1]<<8) 
             + (rx_data[data_cnt+2]<<16) 
             + (rx_data[data_cnt+3]<<24); 
      data_cnt += 4;
      p_pos->z = rx_data[data_cnt] 
             + (rx_data[data_cnt+1]<<8) 
             + (rx_data[data_cnt+2]<<16) 
             + (rx_data[data_cnt+3]<<24); 
      data_cnt += 4;
      p_pos->qf = rx_data[data_cnt];
      return RV_OK;
   }   
   return RV_ERR;
}

int dwm_upd_rate_set(dwm1001 * dwm, uint16_t ur, uint16_t ur_static)
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_UR_SET;
   tx_data[tx_len++] = 4;
   tx_data[tx_len++] = ur & 0xff;
   tx_data[tx_len++] = (ur>>8) & 0xff;   
   tx_data[tx_len++] = ur_static & 0xff;
   tx_data[tx_len++] = (ur_static>>8) & 0xff;   
   LMH_Tx(tx_data, &tx_len);    
   return LMH_WaitForRx(rx_data, &rx_len, 3);   
}

int dwm_upd_rate_get(dwm1001 * dwm, uint16_t *ur, uint16_t *ur_static)
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   uint8_t data_cnt;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_UR_GET;
   tx_data[tx_len++] = 0;   
   LMH_Tx(tx_data, &tx_len);   
   if(LMH_WaitForRx(rx_data, &rx_len, 9) == RV_OK)
   {
      data_cnt = RESP_DAT_VALUE_OFFSET;
      *ur = rx_data[data_cnt] + (rx_data[data_cnt+1]<<8);
      data_cnt += 2;
      *ur_static  = rx_data[data_cnt] + (rx_data[data_cnt+1]<<8);
      data_cnt += 2;
      return RV_OK;
   }   
   return RV_ERR;
}

int dwm_cfg_tag_set(dwm1001 * dwm, dwm_cfg_tag_t* cfg) 
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_CFG_TN_SET;
   tx_data[tx_len++] = 2;
   tx_data[tx_len++] = (cfg->low_power_en ?        (1<<7):0)
                     + (cfg->loc_engine_en ?       (1<<6):0)
                     + (cfg->common.enc_en ?       (1<<5):0)
                     + (cfg->common.led_en ?       (1<<4):0)
                     + (cfg->common.ble_en ?       (1<<3):0)
                     + (cfg->common.fw_update_en ? (1<<2):0)
                     + (((cfg->common.uwb_mode) &  0x03)<<0);
   tx_data[tx_len++] = (cfg->stnry_en ?            (1<<2):0)
                     + (((cfg->meas_mode)       &  0x03)<<0);
   LMH_Tx(tx_data, &tx_len);    
   return LMH_WaitForRx(rx_data, &rx_len, 3);   
}

int dwm_cfg_anchor_set(dwm1001 * dwm, dwm_cfg_anchor_t* cfg)
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_CFG_AN_SET;
   tx_data[tx_len++] = 2;
   tx_data[tx_len++] = (cfg->initiator ?           (1<<7):0)
                     + (cfg->bridge ?              (1<<6):0)
                     + (cfg->common.enc_en ?       (1<<5):0)
                     + (cfg->common.led_en ?       (1<<4):0)
                     + (cfg->common.ble_en ?       (1<<3):0)
                     + (cfg->common.fw_update_en ? (1<<2):0)
                     + (((cfg->common.uwb_mode)  & 0x03)<<0);
   tx_data[tx_len++] = ((cfg->uwb_bh_routing      & 0x03)<<0);
   LMH_Tx(tx_data, &tx_len);    
   return LMH_WaitForRx(rx_data, &rx_len, 3);   
}

int dwm_cfg_get(dwm1001 * dwm, dwm_cfg_t* cfg)
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_CFG_GET;
   tx_data[tx_len++] = 0;   
   LMH_Tx(tx_data, &tx_len);   
   if(LMH_WaitForRx(rx_data, &rx_len, 7) == RV_OK)
   {
      cfg->uwb_bh_routing        = (rx_data[6]>>6) & 0x03;
      cfg->mode                  = (rx_data[6]>>5) & 0x01;
      cfg->initiator             = (rx_data[6]>>4) & 0x01;
      cfg->bridge                = (rx_data[6]>>3) & 0x01;
      cfg->stnry_en              = (rx_data[6]>>2) & 0x01;
      cfg->meas_mode             = (rx_data[6]>>0) & 0x03;
      
      cfg->low_power_en          = (rx_data[5]>>7) & 0x01;
      cfg->loc_engine_en         = (rx_data[5]>>6) & 0x01;
      cfg->common.enc_en         = (rx_data[5]>>5) & 0x01;
      cfg->common.led_en         = (rx_data[5]>>4) & 0x01;
      cfg->common.ble_en         = (rx_data[5]>>3) & 0x01;
      cfg->common.fw_update_en   = (rx_data[5]>>2) & 0x01;
      cfg->common.uwb_mode       = (rx_data[5]>>0) & 0x03;
      return RV_OK;
   }   
   return RV_ERR;
}

int dwm_sleep(dwm1001 * dwm)
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_SLEEP;
   tx_data[tx_len++] = 0;  
   LMH_Tx(tx_data, &tx_len);      
   return LMH_WaitForRx(rx_data, &rx_len, 3);   
}

#define RESP_DATA_ANLIST_CNT_MAX     14
int dwm_anchor_list_get(dwm1001 * dwm, dwm_anchor_list_t *p_list)
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   uint8_t data_cnt, i;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_AN_LIST_GET;
   tx_data[tx_len++] = 0;   
   LMH_Tx(tx_data, &tx_len);   
   if(LMH_WaitForRx(rx_data, &rx_len, DWM1001_TLV_MAX_SIZE) == RV_OK)
   {
      if(rx_len<RESP_DAT_VALUE_OFFSET)// ok + an_list
      {
         return RV_ERR;
      }
         
      if(rx_data[RESP_DAT_TYPE_OFFSET]==DWM1001_TLV_TYPE_AN_LIST)
      {
         if(rx_data[RESP_DAT_LEN_OFFSET]==0)
         {
            p_list->cnt = 0;
            return RV_OK;
         }            
         data_cnt = RESP_DAT_VALUE_OFFSET;   
         p_list->cnt = rx_data[data_cnt++];
         if(p_list->cnt > RESP_DATA_ANLIST_CNT_MAX)
         {
            return RV_ERR;
         }        
         for (i = 0; i < p_list->cnt; i++)
         {
            p_list->v[i].node_id = rx_data[data_cnt]
                                 + (rx_data[data_cnt+1]<<8);
            data_cnt += 2;
            p_list->v[i].x = rx_data[data_cnt]
                           + (rx_data[data_cnt+1]<<8)
                           + (rx_data[data_cnt+2]<<16)
                           + (rx_data[data_cnt+3]<<24);
            data_cnt += 4;
            p_list->v[i].y = rx_data[data_cnt]
                           + (rx_data[data_cnt+1]<<8)
                           + (rx_data[data_cnt+2]<<16)
                           + (rx_data[data_cnt+3]<<24);
            data_cnt += 4;
            p_list->v[i].z = rx_data[data_cnt]
                           + (rx_data[data_cnt+1]<<8)
                           + (rx_data[data_cnt+2]<<16)
                           + (rx_data[data_cnt+3]<<24);
            data_cnt += 4;
            p_list->v[i].rssi = rx_data[data_cnt++];
            p_list->v[i].seat = rx_data[data_cnt] & 0x0f;
            p_list->v[i].neighbor_network = (rx_data[data_cnt] & 0x10) >> 4;
            data_cnt++;
         }
         
         return RV_OK;
      }
   }   
   return RV_ERR;
}

#define RESP_DATA_LOC_LOC_SIZE     15
#define RESP_DATA_LOC_DIST_OFFSET  RESP_DAT_TYPE_OFFSET + RESP_DATA_LOC_LOC_SIZE
#define RESP_DATA_LOC_DIST_LEN_MIN 3
int dwm_loc_get(dwm1001 * dwm, dwm_loc_data_t* loc)
{ 
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   uint8_t data_cnt, i, j;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_LOC_GET;
   tx_data[tx_len++] = 0;   
   LMH_Tx(tx_data, &tx_len);   
   if(LMH_WaitForRx(rx_data, &rx_len, DWM1001_TLV_MAX_SIZE) == RV_OK)
   {
      if(rx_len<RESP_ERRNO_LEN+RESP_DATA_LOC_LOC_SIZE + RESP_DATA_LOC_DIST_LEN_MIN)// ok + pos + distance/range
      {
         return RV_ERR;
      }
      
      if(rx_data[RESP_DAT_TYPE_OFFSET]==DWM1001_TLV_TYPE_POS_XYZ)//0x41
      {
         // node self position.
         data_cnt = RESP_DAT_VALUE_OFFSET;// jump Type and Length, goto data
         loc->p_pos->x = rx_data[data_cnt] 
                      + (rx_data[data_cnt+1]<<8) 
                      + (rx_data[data_cnt+2]<<16) 
                      + (rx_data[data_cnt+3]<<24); 
         data_cnt += 4;
         loc->p_pos->y = rx_data[data_cnt] 
                      + (rx_data[data_cnt+1]<<8) 
                      + (rx_data[data_cnt+2]<<16) 
                      + (rx_data[data_cnt+3]<<24); 
         data_cnt += 4;
         loc->p_pos->z = rx_data[data_cnt] 
                      + (rx_data[data_cnt+1]<<8) 
                      + (rx_data[data_cnt+2]<<16) 
                      + (rx_data[data_cnt+3]<<24); 
         data_cnt += 4;
         loc->p_pos->qf = rx_data[data_cnt++];
      }
      
      if(rx_data[RESP_DATA_LOC_DIST_OFFSET]==DWM1001_TLV_TYPE_RNG_AN_DIST)//0x48
      {
         // node is Anchor, recording Tag ID, distances and qf
         loc->anchors.dist.cnt = rx_data[RESP_DATA_LOC_DIST_OFFSET+2];
         loc->anchors.an_pos.cnt = 0;
         data_cnt = RESP_DATA_LOC_DIST_OFFSET + 3; // jump Type, Length and cnt, goto data
         for (i = 0; i < loc->anchors.dist.cnt; i++)
         {
            // Tag ID
            loc->anchors.dist.addr[i] = 0;
            for (j = 0; j < 8; j++)
            {
               loc->anchors.dist.addr[i] += rx_data[data_cnt++]<<(j*8);
            }
            // Tag distance
            loc->anchors.dist.dist[i] = 0;
            for (j = 0; j < 4; j++)
            {
               loc->anchors.dist.dist[i] += rx_data[data_cnt++]<<(j*8);
            }
            // Tag qf
            loc->anchors.dist.qf[i] = rx_data[data_cnt++];
         }
      }
      else if (rx_data[RESP_DATA_LOC_DIST_OFFSET]==DWM1001_TLV_TYPE_RNG_AN_POS_DIST)//0x49
      {
         // node is Tag, recording Anchor ID, distances, qf and positions
         loc->anchors.dist.cnt = rx_data[RESP_DATA_LOC_DIST_OFFSET+2];
         loc->anchors.an_pos.cnt = rx_data[RESP_DATA_LOC_DIST_OFFSET+2];
         data_cnt = RESP_DATA_LOC_DIST_OFFSET + 3; // jump Type, Length and cnt, goto data
         for (i = 0; i < loc->anchors.dist.cnt; i++)
         {
            // anchor ID
            loc->anchors.dist.addr[i] = 0;
            for (j = 0; j < 2; j++)
            {
               loc->anchors.dist.addr[i] += ((uint64_t)rx_data[data_cnt++])<<(j*8);
            }
            // anchor distance
            loc->anchors.dist.dist[i] = 0;
            for (j = 0; j < 4; j++)
            {
               loc->anchors.dist.dist[i] += ((uint32_t)rx_data[data_cnt++])<<(j*8);
            }
            // anchor qf
            loc->anchors.dist.qf[i] = rx_data[data_cnt++];
            // anchor position
            loc->anchors.an_pos.pos[i].x  = rx_data[data_cnt] 
                                         + (rx_data[data_cnt+1]<<8) 
                                         + (rx_data[data_cnt+2]<<16) 
                                         + (rx_data[data_cnt+3]<<24); 
            data_cnt += 4;
            loc->anchors.an_pos.pos[i].y = rx_data[data_cnt] 
                                         + (rx_data[data_cnt+1]<<8) 
                                         + (rx_data[data_cnt+2]<<16) 
                                         + (rx_data[data_cnt+3]<<24); 
            data_cnt += 4;
            loc->anchors.an_pos.pos[i].z = rx_data[data_cnt] 
                                         + (rx_data[data_cnt+1]<<8) 
                                         + (rx_data[data_cnt+2]<<16) 
                                         + (rx_data[data_cnt+3]<<24); 
            data_cnt += 4;
            loc->anchors.an_pos.pos[i].qf = rx_data[data_cnt++];
         }
      }
      else
      {
         return RV_ERR;   
      }
   }
   else
   {
      return RV_ERR;   
   }
   return RV_OK;
}

int dwm_baddr_set(dwm1001 * dwm, dwm_baddr_t* p_baddr)
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_BLE_ADDR_SET;
   tx_data[tx_len++] = 6;  
   tx_data[tx_len++] = p_baddr->byte[0];  
   tx_data[tx_len++] = p_baddr->byte[1];  
   tx_data[tx_len++] = p_baddr->byte[2];  
   tx_data[tx_len++] = p_baddr->byte[3];  
   tx_data[tx_len++] = p_baddr->byte[4];  
   tx_data[tx_len++] = p_baddr->byte[5];  
   LMH_Tx(tx_data, &tx_len);      
   return LMH_WaitForRx(rx_data, &rx_len, 3);   
}

int dwm_baddr_get(dwm1001 * dwm, dwm_baddr_t* p_baddr)
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   uint8_t data_cnt;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_BLE_ADDR_GET;
   tx_data[tx_len++] = 0;   
   LMH_Tx(tx_data, &tx_len);   
   if(LMH_WaitForRx(rx_data, &rx_len, 11) == RV_OK)
   {
      data_cnt = RESP_DAT_VALUE_OFFSET;
      p_baddr->byte[0] = rx_data[data_cnt++];
      p_baddr->byte[1] = rx_data[data_cnt++];
      p_baddr->byte[2] = rx_data[data_cnt++];
      p_baddr->byte[3] = rx_data[data_cnt++];
      p_baddr->byte[4] = rx_data[data_cnt++];
      p_baddr->byte[5] = rx_data[data_cnt++];
      return RV_OK;
   }   
   return RV_ERR;
}

int dwm_stnry_cfg_set(dwm1001 * dwm, dwm_stnry_sensitivity_t sensitivity)
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   if(sensitivity > DWM_STNRY_SENSITIVITY_HIGH)
   {
      return RV_ERR_PARAM;
   }
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_STNRY_CFG_SET;
   tx_data[tx_len++] = 1;  
   tx_data[tx_len++] = sensitivity;  
   LMH_Tx(tx_data, &tx_len);      
   return LMH_WaitForRx(rx_data, &rx_len, 3);   
}

int dwm_stnry_cfg_get(dwm1001 * dwm, dwm_stnry_sensitivity_t* p_sensitivity)
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_STNRY_CFG_GET;
   tx_data[tx_len++] = 0;   
   LMH_Tx(tx_data, &tx_len);   
   if(LMH_WaitForRx(rx_data, &rx_len, 6) == RV_OK)
   {
      *p_sensitivity = rx_data[RESP_DAT_VALUE_OFFSET];
      return RV_OK;
   }   
   return RV_ERR;
}

int dwm_factory_reset(dwm1001 * dwm)
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_FAC_RESET;
   tx_data[tx_len++] = 0;  
   LMH_Tx(tx_data, &tx_len);      
   return LMH_WaitForRx(rx_data, &rx_len, 3);   
}  

int dwm_reset(dwm1001 * dwm)
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_RESET;
   tx_data[tx_len++] = 0;  
   LMH_Tx(tx_data, &tx_len);      
   return LMH_WaitForRx(rx_data, &rx_len, 3);   
}

int dwm_ver_get(dwm1001 * dwm, dwm_ver_t* ver)
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   uint8_t data_cnt;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_VER_GET;
   tx_data[tx_len++] = 0;   
   LMH_Tx(tx_data, &tx_len);   
   if(LMH_WaitForRx(rx_data, &rx_len, 21) == RV_OK)
   {    
      // read fw_version
      if(rx_data[RESP_DAT_TYPE_OFFSET] != DWM1001_TLV_TYPE_FW_VER)
      {
         return RV_ERR;
      }
      if(rx_data[RESP_DAT_LEN_OFFSET] != 4)
      {
         return RV_ERR;
      }
      data_cnt = RESP_DAT_VALUE_OFFSET;  
      ver->fw.res = (rx_data[data_cnt]>>4) & 0x0f;
      ver->fw.var = rx_data[data_cnt] & 0x0f;
      data_cnt++;
      ver->fw.patch = rx_data[data_cnt++];
      ver->fw.min = rx_data[data_cnt++];
      ver->fw.maj = rx_data[data_cnt++];
      // read cfg_version
      if(rx_data[data_cnt++] != DWM1001_TLV_TYPE_CFG_VER)
      {
         return RV_ERR;
      }
      if(rx_data[data_cnt++] != 4)
      {
         return RV_ERR;
      }
      ver->cfg = rx_data[data_cnt] 
              + (rx_data[data_cnt+1]<<8) 
              + (rx_data[data_cnt+2]<<16) 
              + (rx_data[data_cnt+3]<<24); 
      data_cnt += 4;
      // read hw_version
      if(rx_data[data_cnt++] != DWM1001_TLV_TYPE_HW_VER)
      {
         return RV_ERR;
      }
      if(rx_data[data_cnt++] != 4)
      {
         return RV_ERR;
      }
      ver->hw =  rx_data[data_cnt] 
              + (rx_data[data_cnt+1]<<8) 
              + (rx_data[data_cnt+2]<<16) 
              + (rx_data[data_cnt+3]<<24); 
      data_cnt += 4;
      return RV_OK;
   }   
   return RV_ERR;
}

int dwm_uwb_cfg_set(dwm1001 * dwm, dwm_uwb_cfg_t* p_cfg) 
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_UWB_CFG_SET;
   tx_data[tx_len++] = 5;
   tx_data[tx_len++] = p_cfg->pg_delay;
   tx_data[tx_len++] = (p_cfg->tx_power >> 0) & 0xff;
   tx_data[tx_len++] = (p_cfg->tx_power >> 8) & 0xff;
   tx_data[tx_len++] = (p_cfg->tx_power >> 16) & 0xff;
   tx_data[tx_len++] = (p_cfg->tx_power >> 24) & 0xff;
   LMH_Tx(tx_data, &tx_len);    
   return LMH_WaitForRx(rx_data, &rx_len, 3);   
}

int dwm_uwb_cfg_get(dwm1001 * dwm, dwm_uwb_cfg_t* p_cfg) 
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   uint8_t data_cnt;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_UWB_CFG_GET;
   tx_data[tx_len++] = 0;   
   LMH_Tx(tx_data, &tx_len);   
   if(LMH_WaitForRx(rx_data, &rx_len, 15) == RV_OK)
   {
      data_cnt = RESP_DAT_VALUE_OFFSET;
      p_cfg->pg_delay = rx_data[data_cnt++];
      p_cfg->tx_power = (uint64_t)rx_data[data_cnt] 
                     + ((uint64_t)rx_data[data_cnt+1]<<8) 
                     + ((uint64_t)rx_data[data_cnt+2]<<16) 
                     + ((uint64_t)rx_data[data_cnt+3]<<24);
      data_cnt += 4;
      p_cfg->compensated.pg_delay = rx_data[data_cnt++];
      p_cfg->compensated.tx_power = (uint64_t)rx_data[data_cnt] 
                                 + ((uint64_t)rx_data[data_cnt+1]<<8) 
                                 + ((uint64_t)rx_data[data_cnt+2]<<16) 
                                 + ((uint64_t)rx_data[data_cnt+3]<<24);
      return RV_OK;
   }   
   return RV_ERR;
}

int dwm_usr_data_read(dwm1001 * dwm, uint8_t* p_data, uint8_t* p_len)
{     
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_USR_DATA_READ;
   tx_data[tx_len++] = 0; 
   LMH_Tx(tx_data, &tx_len);   
   if(LMH_WaitForRx(rx_data, &rx_len, DWM1001_TLV_MAX_SIZE) == RV_OK)
   {
      if((rx_len<3+2) || (rx_len > 3+2+DWM_API_USR_DATA_LEN_MAX))// ok + pos + distance/range
      {
         return RV_ERR;
      }      
      memset(p_data, 0, DWM_API_USR_DATA_LEN_MAX);
      *p_len = rx_data[4];
      if((rx_len == 5) && (*p_len == 0))
      {
         return RV_OK;
      }
      if(*p_len <= DWM_API_USR_DATA_LEN_MAX)
      {
         memcpy(p_data, rx_data+5, *p_len);
         return RV_OK;
      }
   }   
   return RV_ERR;   
}

int dwm_usr_data_write(dwm1001 * dwm, uint8_t* p_data, uint8_t len, bool overwrite)
{        
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   if(len > DWM_API_USR_DATA_LEN_MAX)
   {
      return RV_ERR;   
   }
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_USR_DATA_WRITE;
   tx_data[tx_len++] = len+1;
   tx_data[tx_len++] = overwrite;
   memcpy(tx_data+tx_len, p_data, len);
   tx_len += len;
   LMH_Tx(tx_data, &tx_len);   
   return LMH_WaitForRx(rx_data, &rx_len, 3);
}

int dwm_label_read(dwm1001 * dwm, uint8_t* p_label, uint8_t* p_len)
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_LABEL_READ;
   tx_data[tx_len++] = 0;
   LMH_Tx(tx_data, &tx_len);   
   if(LMH_WaitForRx(rx_data, &rx_len, DWM1001_TLV_MAX_SIZE) == RV_OK)
   {
      if((rx_len<3+2) || (rx_len > 3+2+DWM_LABEL_LEN_MAX))// ok + data
      {
         return RV_ERR;
      }      
      memset(p_label, 0, DWM_LABEL_LEN_MAX);
      *p_len = rx_data[4];
      if((rx_len == 5) && (*p_len == 0))
      {
         return RV_OK;
      }
      if(*p_len <= DWM_LABEL_LEN_MAX)
      {
         memcpy(p_label, rx_data+5, *p_len);
         return RV_OK;
      }
   }   
   return RV_ERR;   
}

int dwm_label_write(dwm1001 * dwm, uint8_t* p_label, uint8_t len)
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   if(len > DWM_LABEL_LEN_MAX)
   {
      return RV_ERR;   
   }
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_LABEL_WRITE;
   tx_data[tx_len++] = len;
   memcpy(tx_data+tx_len, p_label, len);
   tx_len += len;
   LMH_Tx(tx_data, &tx_len);   
   return LMH_WaitForRx(rx_data, &rx_len, 3);
}

int dwm_gpio_cfg_output(dwm1001 * dwm, dwm_gpio_idx_t idx, bool value)
{      
   if(LMH_CheckGPIOIdx(idx)!= RV_OK)
   {
      return RV_ERR;
   }
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_GPIO_CFG_OUTPUT;
   tx_data[tx_len++] = 2;
   tx_data[tx_len++] = idx & 0xff;
   tx_data[tx_len++] = (uint8_t)value;       
   LMH_Tx(tx_data, &tx_len);      
   return LMH_WaitForRx(rx_data, &rx_len, 3);
}

int dwm_gpio_cfg_input(dwm1001 * dwm, dwm_gpio_idx_t idx, dwm_gpio_pin_pull_t pull_mode)
{      
   if(LMH_CheckGPIOIdx(idx)!= RV_OK)
   {
      return RV_ERR;
   }
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_GPIO_CFG_INPUT;
   tx_data[tx_len++] = 2;
   tx_data[tx_len++] = idx & 0xff;
   tx_data[tx_len++] = (uint8_t)pull_mode;      
   LMH_Tx(tx_data, &tx_len);    
   return LMH_WaitForRx(rx_data, &rx_len, 3);
}

int dwm_gpio_value_set(dwm1001 * dwm, dwm_gpio_idx_t idx, bool value)
{
   if(LMH_CheckGPIOIdx(idx)!= RV_OK)
   {
      return RV_ERR;
   }
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_GPIO_VAL_SET;
   tx_data[tx_len++] = 2;
   tx_data[tx_len++] = idx & 0xff;
   tx_data[tx_len++] = (uint8_t)value;      
   LMH_Tx(tx_data, &tx_len);   
   return LMH_WaitForRx(rx_data, &rx_len, 3);
}

int dwm_gpio_value_get(dwm1001 * dwm, dwm_gpio_idx_t idx, bool* p_value)
{
   if(LMH_CheckGPIOIdx(idx)!= RV_OK)
   {
      return RV_ERR;
   }
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_GPIO_VAL_GET;
   tx_data[tx_len++] = 1;
   tx_data[tx_len++] = idx & 0xff;  
   LMH_Tx(tx_data, &tx_len);   
   if(LMH_WaitForRx(rx_data, &rx_len, 6) == RV_OK)
   {
      *p_value = (bool)rx_data[5];
      return RV_OK;
   }   
   return RV_ERR;
}

int dwm_gpio_value_toggle(dwm1001 * dwm, dwm_gpio_idx_t idx)
{
   if(LMH_CheckGPIOIdx(idx)!= RV_OK)
   {
      return RV_ERR;
   }
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_GPIO_VAL_TOGGLE;
   tx_data[tx_len++] = 1;
   tx_data[tx_len++] = idx & 0xff;  
   LMH_Tx(tx_data, &tx_len);   
   return LMH_WaitForRx(rx_data, &rx_len, 3);
}

int dwm_panid_set(dwm1001 * dwm, uint16_t value)
{        
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_PANID_SET;
   tx_data[tx_len++] = 2;
   tx_data[tx_len++] = value & 0xff;
   tx_data[tx_len++] = (value & 0xff00)>>8;    
   LMH_Tx(tx_data, &tx_len);   
   return LMH_WaitForRx(rx_data, &rx_len, 3);
}

int dwm_panid_get(dwm1001 * dwm, uint16_t* p_value)
{        
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_PANID_GET;
   tx_data[tx_len++] = 0; 
   LMH_Tx(tx_data, &tx_len);   
   if(LMH_WaitForRx(rx_data, &rx_len, 7) == RV_OK)
   {
      *p_value = rx_data[5] + ((uint16_t)rx_data[6]<<8);
      return RV_OK;
   }   
   return RV_ERR;
}

int dwm_node_id_get(dwm1001 * dwm, uint64_t *p_node_id)
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   uint8_t data_cnt;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_NODE_ID_GET;
   tx_data[tx_len++] = 0; 
   LMH_Tx(tx_data, &tx_len);  
   if(LMH_WaitForRx(rx_data, &rx_len, 13) == RV_OK)
   {
      data_cnt = RESP_DAT_VALUE_OFFSET;
      *p_node_id = (uint64_t)rx_data[data_cnt] 
               + ((uint64_t)rx_data[data_cnt+1]<<8) 
               + ((uint64_t)rx_data[data_cnt+2]<<16) 
               + ((uint64_t)rx_data[data_cnt+3]<<24) 
               + ((uint64_t)rx_data[data_cnt+4]<<32) 
               + ((uint64_t)rx_data[data_cnt+5]<<40) 
               + ((uint64_t)rx_data[data_cnt+6]<<48) 
               + ((uint64_t)rx_data[data_cnt+7]<<56); 
      return RV_OK;
   }   
   return RV_ERR; 
}

int dwm_status_get(dwm1001 * dwm, dwm_status_t* p_status)
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   uint16_t flags;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_STATUS_GET;
   tx_data[tx_len++] = 0;   
   LMH_Tx(tx_data, &tx_len);   
   if(LMH_WaitForRx(rx_data, &rx_len, 7) == RV_OK)
   {
      flags = (uint16_t)rx_data[5] + (((uint16_t)rx_data[6])<<8);
      p_status->loc_data         = (flags & API_STATUS_FLAG_LOC_READY)? 1:0;
      p_status->uwbmac_joined    = (flags & API_STATUS_FLAG_UWBMAC_JOINED)? 1:0;
      p_status->bh_data_ready    = (flags & API_STATUS_FLAG_BH_STATUS_CHANGED)? 1:0;
      p_status->bh_status_changed = (flags & API_STATUS_FLAG_BH_DATA_READY)? 1:0;
      p_status->bh_initialized   = (flags & API_STATUS_FLAG_BH_INITIALIZED)? 1:0;
      p_status->uwb_scan_ready   = (flags & API_STATUS_FLAG_UWB_SCAN_READY)? 1:0;
      p_status->usr_data_ready   = (flags & API_STATUS_FLAG_USR_DATA_READY)? 1:0;
      p_status->usr_data_sent    = (flags & API_STATUS_FLAG_USR_DATA_SENT)? 1:0;
      p_status->fwup_in_progress = (flags & API_STATUS_FLAG_FWUP_IN_PROGRESS)? 1:0;
      return RV_OK;
   }   
   return RV_ERR;
}

int dwm_int_cfg_set(dwm1001 * dwm, uint16_t value)
{        
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_INT_CFG_SET;
   tx_data[tx_len++] = 2;
   tx_data[tx_len++] = value & 0xff;    
   tx_data[tx_len++] = (value>>8) & 0xff;   
   LMH_Tx(tx_data, &tx_len);   
   return LMH_WaitForRx(rx_data, &rx_len, 3);
}

int dwm_int_cfg_get(dwm1001 * dwm, uint16_t *p_value)
{        
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_INT_CFG_GET;
   tx_data[tx_len++] = 0;
   LMH_Tx(tx_data, &tx_len);   
   if(LMH_WaitForRx(rx_data, &rx_len, 7) == RV_OK)
   {
      *p_value = (uint16_t)rx_data[5] + (((uint16_t)rx_data[6])<<8);
      return RV_OK;
   }   
   return RV_ERR;
}

int dwm_bh_status_get(dwm1001 * dwm, bh_status_t * p_bh_status)
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   uint8_t data_cnt, i;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_BH_STATUS_GET;
   tx_data[tx_len++] = 0;
   LMH_Tx(tx_data, &tx_len);   
   if(LMH_WaitForRx(rx_data, &rx_len, DWM1001_TLV_MAX_SIZE) == RV_OK)
   {
      if((rx_len<3+9) || (rx_len > 3+9+(DWM_API_BH_ORIGIN_CNT_MAX*4)))
      {
         return RV_ERR;
      }      
      memset(p_bh_status, 0, sizeof(bh_status_t));
      data_cnt = RESP_DAT_VALUE_OFFSET;
      p_bh_status->sf_number = rx_data[data_cnt] 
                              + (rx_data[data_cnt+1]<<8);
      data_cnt+=2;
      p_bh_status->bh_seat_map = rx_data[data_cnt] 
                              + (rx_data[data_cnt+1]<<8) 
                              + (rx_data[data_cnt+2]<<16) 
                              + (rx_data[data_cnt+3]<<24); 
      data_cnt+=4;
      p_bh_status->origin_cnt = rx_data[data_cnt++];
      
      if(p_bh_status->origin_cnt > DWM_API_BH_ORIGIN_CNT_MAX)
      {
         return RV_ERR;
      }
      
      for(i = 0; i < p_bh_status->origin_cnt; i++)
      {
         p_bh_status->origin_info[i].node_id = rx_data[data_cnt] 
                              + (rx_data[data_cnt+1]<<8) ;
         data_cnt += 2;
         p_bh_status->origin_info[i].bh_seat = rx_data[data_cnt++]; 
         p_bh_status->origin_info[i].hop_lvl = rx_data[data_cnt++]; 
      }
      return RV_OK;
   }   
   return RV_ERR;
}
         
int dwm_enc_key_set(dwm1001 * dwm, dwm_enc_key_t* p_key)
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint8_t i;
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_ENC_KEY_SET;
   tx_data[tx_len++] = DWM_ENC_KEY_LEN;
   for(i = 0; i < DWM_ENC_KEY_LEN; i++)
   {
      tx_data[tx_len++] = p_key->byte[i];      
   }   
   LMH_Tx(tx_data, &tx_len);   
   return LMH_WaitForRx(rx_data, &rx_len, 3);
}
   
int dwm_enc_key_clear(dwm1001 * dwm)
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_ENC_KEY_CLEAR;
   tx_data[tx_len++] = 0;  
   LMH_Tx(tx_data, &tx_len);      
   return LMH_WaitForRx(rx_data, &rx_len, 3);   
}  

// =======================================================================================
// =======================================================================================
// =======================================================================================
// ================================ internal api for test ================================
// =======================================================================================
// =======================================================================================
// =======================================================================================

int dwm_uwb_preamble_code_set(dwm1001 * dwm, dwm_uwb_preamble_code_t code)
{        
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_UWB_PREAMBLE_SET;
   tx_data[tx_len++] = 1;
   tx_data[tx_len++] = code;  
   LMH_Tx(tx_data, &tx_len);   
   return LMH_WaitForRx(rx_data, &rx_len, 3);
}

int dwm_uwb_preamble_code_get(dwm1001 * dwm, dwm_uwb_preamble_code_t *p_code)
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_UWB_PREAMBLE_GET;
   tx_data[tx_len++] = 0;   
   LMH_Tx(tx_data, &tx_len);   
   if(LMH_WaitForRx(rx_data, &rx_len, 6) == RV_OK)
   {
      *p_code = rx_data[5];
      return RV_OK;
   }   
   return RV_ERR;
}

int dwm_uwb_scan_start(dwm1001 * dwm)
{        
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_UWB_SCAN_START;
   tx_data[tx_len++] = 0;
   LMH_Tx(tx_data, &tx_len);   
   return LMH_WaitForRx(rx_data, &rx_len, 3);
}

int dwm_uwb_scan_result_get(dwm1001 * dwm, dwm_uwb_scan_result_t *p_result)
{        
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint8_t data_cnt, i;
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_UWB_SCAN_RES_GET;
   tx_data[tx_len++] = 0;
   LMH_Tx(tx_data, &tx_len);   
   if(LMH_WaitForRx(rx_data, &rx_len, DWM1001_TLV_MAX_SIZE) == RV_OK)
   {
      if((rx_len<RESP_DAT_VALUE_OFFSET) || (rx_len > RESP_DAT_VALUE_OFFSET+DWM_UWB_SCAN_RESULT_CNT_MAX*2))
      {
         return RV_ERR;
      }      
      if(rx_data[RESP_DAT_LEN_OFFSET] & 1)// if not even number
      {
         return RV_ERR;
      }
      p_result->cnt = rx_data[RESP_DAT_LEN_OFFSET]>>1;
      data_cnt = RESP_DAT_VALUE_OFFSET;  
      for(i = 0; i < p_result->cnt; i++)
      {
         p_result->mode[i] = rx_data[data_cnt++];
         p_result->rssi[i] = rx_data[data_cnt++];
      }
      return RV_OK;
   }   
   return RV_ERR;
}


