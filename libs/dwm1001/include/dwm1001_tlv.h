#ifndef _DWM1001_TLV_H_
#define _DWM1001_TLV_H_

#include <stdbool.h>
#include <stdint.h>


#define API_RV_OK                0   /* command ret value OK */
#define API_RV_ERR_TLV           1   /* command ret value TLV ERROR, unknown cmd or broken frame */
#define API_RV_ERR_INTERNAL      2   /* command ret value internal error */
#define API_RV_ERR_INVAL_PARAM   3   /* command ret value invalid input parameter*/
#define API_RV_ERR_BUSY          4   /* command ret value busy indication*/
#define API_RV_BACKHAUL          127 /* command ret value fixme USER DATA */


#define DWM1001_TLV_MAX_SIZE           255
#define DWM1001_TLV_RET_VAL_MIN_SIZE   3
/****************************************************************************//**
 * @brief definitions for TLV Type byte
**/

#define DWM1001_TLV_TYPE_CMD_POS_SET               0x01  /* command set position coordinates XYZ */
#define DWM1001_TLV_TYPE_CMD_POS_GET               0x02  /* command get position coordinates XYZ */
#define DWM1001_TLV_TYPE_CMD_UR_SET                0x03  /* command set position update rate*/
#define DWM1001_TLV_TYPE_CMD_UR_GET                0x04  /* command get position update rate*/
#define DWM1001_TLV_TYPE_CMD_CFG_TN_SET            0x05  /* command set configuration for the tag */
#define DWM1001_TLV_TYPE_CMD_CFG_AN_SET            0x07  /* command set configuration for the anchor */
#define DWM1001_TLV_TYPE_CMD_CFG_GET               0x08  /* command get configuration data */
#define DWM1001_TLV_TYPE_CMD_SLEEP                 0x0a  /* command sleep */
#define DWM1001_TLV_TYPE_CMD_AN_LIST_GET		      0x0b	/* command anchor list get */
#define DWM1001_TLV_TYPE_CMD_LOC_GET               0x0c  /* command location get */
#define DWM1001_TLV_TYPE_CMD_BLE_ADDR_SET          0x0f  /* command BLE address set */
#define DWM1001_TLV_TYPE_CMD_BLE_ADDR_GET          0x10  /* command BLE address get */
#define DWM1001_TLV_TYPE_CMD_STNRY_CFG_SET		   0x11	/* command stationary configuration set */
#define DWM1001_TLV_TYPE_CMD_STNRY_CFG_GET		   0x12	/* command stationary configuration get */
#define DWM1001_TLV_TYPE_CMD_FAC_RESET			      0x13	/* command factory reset */
#define DWM1001_TLV_TYPE_CMD_RESET                 0x14  /* command reset */
#define DWM1001_TLV_TYPE_CMD_VER_GET               0x15  /* command FW version get */
#define DWM1001_TLV_TYPE_CMD_UWB_CFG_SET		      0x17	/* command UWB configuration set */
#define DWM1001_TLV_TYPE_CMD_UWB_CFG_GET		      0x18	/* command UWB configuration get */
#define DWM1001_TLV_TYPE_CMD_USR_DATA_READ         0x19  /* command user data read */
#define DWM1001_TLV_TYPE_CMD_USR_DATA_WRITE        0x1a  /* command user data write */
#define DWM1001_TLV_TYPE_CMD_LABEL_READ            0x1c  /* command lebel read */
#define DWM1001_TLV_TYPE_CMD_LABEL_WRITE           0x1d  /* command label write */
#define DWM1001_TLV_TYPE_CMD_UWB_PREAMBLE_SET	   0x1e	/* command set uwb preamble code */
#define DWM1001_TLV_TYPE_CMD_UWB_PREAMBLE_GET	   0x1f	/* command get uwb preamble code */
#define DWM1001_TLV_TYPE_CMD_UWB_SCAN_START		   0x23	/* command UWB scan start */
#define DWM1001_TLV_TYPE_CMD_UWB_SCAN_RES_GET	   0x24	/* command UWB scan results get */
#define DWM1001_TLV_TYPE_CMD_GPIO_CFG_OUTPUT       0x28  /* command configure output pin and set */
#define DWM1001_TLV_TYPE_CMD_GPIO_CFG_INPUT        0x29  /* command configure input pin */
#define DWM1001_TLV_TYPE_CMD_GPIO_VAL_SET          0x2a  /* command set pin value */
#define DWM1001_TLV_TYPE_CMD_GPIO_VAL_GET          0x2b  /* command get pin value */
#define DWM1001_TLV_TYPE_CMD_GPIO_VAL_TOGGLE       0x2c  /* command toggle pin value */
#define DWM1001_TLV_TYPE_CMD_PANID_SET             0x2e  /* command panid set */
#define DWM1001_TLV_TYPE_CMD_PANID_GET             0x2f  /* command panid get */
#define DWM1001_TLV_TYPE_CMD_NODE_ID_GET           0x30   /* command node id get */
#define DWM1001_TLV_TYPE_CMD_STATUS_GET            0x32  /* command status get */
#define DWM1001_TLV_TYPE_CMD_INT_CFG_SET           0x34  /* command configure interrupts */
#define DWM1001_TLV_TYPE_CMD_INT_CFG_GET           0x35  /* command get interrupt configuration */
#define DWM1001_TLV_TYPE_CMD_BACKHAUL_XFER         0x37  /* command BACKHAUL data transfer */
#define DWM1001_TLV_TYPE_CMD_BH_STATUS_GET         0x3a  /* command to get UWBMAC status*/
#define DWM1001_TLV_TYPE_CMD_ENC_KEY_SET           0x3c  /* command to set security key */
#define DWM1001_TLV_TYPE_CMD_ENC_KEY_CLEAR         0x3d  /* command to set security key */
#define DWM1001_TLV_TYPE_CMD_VA_ARG_POS_SET        0x80  /* VA ARG command set position */
//#define DWM1001_TLV_TYPE_CMD_N_LOC_GET         130   /* nested request location get */
#define DWM1001_TLV_TYPE_RET_VAL                   0x40  /* request return value (as the response) */
#define DWM1001_TLV_TYPE_POS_XYZ                   0x41  /* position coordinates x,y,z*/
#define DWM1001_TLV_TYPE_POS_X                     0x42  /* position coordinate x */
#define DWM1001_TLV_TYPE_POS_Y                     0x43  /* position coordinate y */
#define DWM1001_TLV_TYPE_POS_Z                     0x44  /* position coordinate z */
#define DWM1001_TLV_TYPE_UR                        0x45  /* update rate */
#define DWM1001_TLV_TYPE_CFG                       0x46  /* configuration data */
#define DWM1001_TLV_TYPE_INT_CFG                   0x47  /* interrupt configuration */
#define DWM1001_TLV_TYPE_RNG_AN_DIST               0x48  /* ranging anchor distances*/
#define DWM1001_TLV_TYPE_RNG_AN_POS_DIST           0x49  /* ranging anchor distances and positions*/
#define DWM1001_TLV_TYPE_STNRY_SENSITIVITY	      0x4a	/* stationary sensitivity */
#define DWM1001_TLV_TYPE_USR_DATA                  0x4b  /* user data */
#define DWM1001_TLV_TYPE_LABEL                     0x4c  /* label */
#define DWM1001_TLV_TYPE_PANID                     0x4d  /* PANID */
#define DWM1001_TLV_TYPE_NODE_ID                   0x4e  /* node ID */
#define DWM1001_TLV_TYPE_UWB_CFG			            0x4f	/* UWB configuration */
#define DWM1001_TLV_TYPE_FW_VER                    0x50  /* fw_version */
#define DWM1001_TLV_TYPE_CFG_VER                   0x51  /* cfg_version */
#define DWM1001_TLV_TYPE_HW_VER                    0x52  /* hw_version */
#define DWM1001_TLV_TYPE_PIN_VAL                   0x55  /* pin value */
#define DWM1001_TLV_TYPE_AN_LIST                   0x56  /* anchor list */
#define DWM1001_TLV_TYPE_STATUS                    0x5a  /* status */
#define DWM1001_TLV_TYPE_UWB_SCAN_RESULT           0x5c  /* UWB scan result */
#define DWM1001_TLV_TYPE_UWBMAC_STATUS             0x5d  /* UWBMAC status */
#define DWM1001_TLV_TYPE_BLE_ADDR                  0x5f  /* BLE address */
#define DWM1001_TLV_TYPE_DOWNLINK_CHUNK_0          0x64  /* downlink data chunk nr.1 */
#define DWM1001_TLV_TYPE_BUF_IDX                   0x69  /* index of the buffer in the container */
#define DWM1001_TLV_TYPE_BUF_SIZE                  0x6a  /* data size in the buffer of the container */
#define DWM1001_TLV_TYPE_UPLINK_CHUNK_0            0x6e  /* uplink data chunk nr.1 */
#define DWM1001_TLV_TYPE_UPLINK_LOC_DATA           0x78  /* FIXME: set proper define name and value */
#define DWM1001_TLV_TYPE_UPLINK_IOT_DATA           0x79  /* FIXME: set proper define name and value */
#define DWM1001_TLV_TYPE_VAR_LEN_PARAM             0x7f  /* parameter with variable length */

#define DWM_API_PARAMS_CNT_MAX                     3
#define DWM1001_TLV_TYPE_IDLE                      0xff /* Dummy byte, or type: idle */


#endif //_DWM1001_TLV_H_
