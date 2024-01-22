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

#ifndef _DWM_API_H_
#define _DWM_API_H_

#include "dwm1001_tlv.h"

/**
 * @brief DWM Error codes, returned from DWM1001 module
 */
#define DWM_OK				   (0)
#define DWM_ERR_INTERNAL	(-1)
#define DWM_ERR_BUSY		   (-2)
#define DWM_ERR_INVAL_ADDR	(-3)
#define DWM_ERR_INVAL_PARAM   (-4)
#define DWM_ERR_OVERRUN		(-5)
#define DWM_ERR_I2C_ANACK	(-10)
#define DWM_ERR_I2C_DNACK	(-11)

/**
 * @brief Return value Error codes, returned from DWM API functions
 */
#define RV_OK           (0)      /*  ret value OK */
#define RV_ERR          (1)      /*  ret value ERROR: unknown command or broken tlv frame */
#define RV_ERR_INTERNAL (2)      /*  ret value ERROR: internal error */
#define RV_ERR_PARAM    (3)      /*  ret value ERROR: invalid parameter */
#define RV_ERR_BUSY     (4)      /*  ret value ERROR: busy  */
#define RV_ERR_PERMIT   (5)      /*  ret value ERROR: operation not permitted */

#define DWM_API_USR_DATA_LEN_MAX 		34
#define DWM_API_LABEL_LEN_MAX 			16
#define DWM_API_UWB_SCAN_RESULT_CNT_MAX 16
#define DWM_API_BH_ORIGIN_CNT_MAX 		9
#define DWM_RANGING_ANCHOR_CNT_MAX	14
#define DWM_API_USR_DATA_LEN_MAX	34

#define DWM_LABEL_LEN_MAX	16

/* BLE address length */
#define DWM_BLE_ADDR_LEN 6
/********************************************************************************************************************/
/*                                                     API LIST                                                     */
/********************************************************************************************************************/
/**
 * @brief Position coordinates in millimeters + quality factor
 */
typedef struct {
	int32_t x;
	int32_t y;
	int32_t z;
	uint8_t qf;
} dwm_pos_t;

/**
 * @brief maximum and minimum update rate in multiple of 100 ms
 */
enum dwm_upd_rate{
	DWM_UPD_RATE_MAX = 600,	/* 1 minute */
	DWM_UPD_RATE_MIN = 1	/* 100 ms */
};

/**
 * @brief Position measurement modes
 */
typedef enum {
	DWM_MEAS_MODE_TWR = 0,//!< DWM_MEAS_MODE_TWR
	DWM_MEAS_MODE_TDOA = 1//!< DWM_MEAS_MODE_TDOA
} dwm_meas_mode_t;

/**
 * @brief Device modes
 */
typedef enum {
	DWM_MODE_TAG = 0,  //!< DWM_MODE_TAG
	DWM_MODE_ANCHOR = 1//!< DWM_MODE_ANCHOR
} dwm_mode_t;

typedef enum {
	DWM_UWB_MODE_OFF = 0,
	DWM_UWB_MODE_PASSIVE = 1,
	DWM_UWB_MODE_ACTIVE = 2
}dwm_uwb_mode_t;

typedef enum {
	DWM_UWB_BH_ROUTING_OFF = 0,
	DWM_UWB_BH_ROUTING_ON = 1,
	DWM_UWB_BH_ROUTING_AUTO = 2,
} dwm_uwb_bh_routing_t;

typedef struct dwm_cfg_common {
	dwm_uwb_mode_t uwb_mode;
	bool fw_update_en;
	bool ble_en;
	bool led_en;
	bool enc_en;
} dwm_cfg_common_t;

typedef struct dwm_cfg_anchor {
	dwm_cfg_common_t common;
	bool bridge;
	bool initiator;
	dwm_uwb_bh_routing_t uwb_bh_routing;
} dwm_cfg_anchor_t;

typedef struct dwm_cfg_tag {
	dwm_cfg_common_t common;
	bool loc_engine_en;
	bool low_power_en;
	bool stnry_en;
	dwm_meas_mode_t meas_mode;
} dwm_cfg_tag_t;

typedef struct dwm_cfg {
	dwm_cfg_common_t common;
	bool loc_engine_en;
	bool low_power_en;
	bool stnry_en;
	dwm_meas_mode_t meas_mode;
	dwm_uwb_bh_routing_t uwb_bh_routing;
	bool bridge;
	bool initiator;
	dwm_mode_t mode;
} dwm_cfg_t;

/**
 * @brief List of anchors
 */
typedef struct {
	uint8_t cnt;
	dwm_anchor_list_elt_t v[DWM_RANGING_ANCHOR_CNT_MAX];
} dwm_anchor_list_t;

/**
 * @brief Element of anchor list
 */
typedef struct {
	uint16_t node_id;
	int32_t x,y,z;
	int8_t rssi;
	uint8_t seat;
	bool neighbor_network;
} dwm_anchor_list_elt_t;


/**
 * @brief Distances of ranging anchors
 */
typedef struct {
	uint8_t cnt;
	uint16_t addr[DWM_RANGING_ANCHOR_CNT_MAX];
	uint32_t dist[DWM_RANGING_ANCHOR_CNT_MAX];
	uint8_t qf[DWM_RANGING_ANCHOR_CNT_MAX];
}dwm_distance_t;

/**
 * @brief Position of ranging anchors
 */
typedef struct {
	uint8_t cnt;
	dwm_pos_t pos[DWM_RANGING_ANCHOR_CNT_MAX];
}dwm_anchor_pos_t;

/**
 * @brief Distances and position of ranging anchors
 */
typedef struct {
	dwm_distance_t dist;
	dwm_anchor_pos_t an_pos;
}dwm_ranging_anchors_t;

/**
 * @brief Location data (position of current node and list of positions
 * and distances of ranging anchors)
 */
typedef struct dwm_loc_data_t {
	dwm_pos_t* p_pos;
	dwm_ranging_anchors_t anchors;
} dwm_loc_data_t;

/**
 * @brief BLE address
 */
typedef struct {
	uint8_t byte[DWM_BLE_ADDR_LEN];
}dwm_baddr_t;

typedef enum {
	DWM_STNRY_SENSITIVITY_LOW = 0,
	DWM_STNRY_SENSITIVITY_NORMAL = 1,
	DWM_STNRY_SENSITIVITY_HIGH = 2
} dwm_stnry_sensitivity_t;

/**
 * @brief Firmware version data
 */
typedef struct dwm_fw_ver_t {
	uint8_t maj;
	uint8_t min;
	uint8_t patch;
	uint8_t res;
	uint8_t var;
}dwm_fw_ver_t;

/**
 * @brief Version data
 */
typedef struct dwm_ver_t {
	dwm_fw_ver_t fw;
	uint32_t cfg;
	uint32_t hw;
} dwm_ver_t;

/**
 * @brief UWB configuration parameters
 */
typedef struct {
	uint8_t pg_delay;
	uint32_t tx_power;
	struct {
		uint8_t pg_delay;
		uint32_t tx_power;
	} compensated;
} dwm_uwb_cfg_t;

/**
 * @brief GPIO pins available for the user application
 */
typedef enum {
	DWM_GPIO_IDX_2 = 2,  //!< DWM_GPIO_IDX_2
	DWM_GPIO_IDX_8 = 8,  //!< DWM_GPIO_IDX_8
	DWM_GPIO_IDX_9 = 9,  //!< DWM_GPIO_IDX_9
	DWM_GPIO_IDX_10 = 10,  //!< DWM_GPIO_IDX_10
	DWM_GPIO_IDX_12 = 12,  //!< DWM_GPIO_IDX_12
	DWM_GPIO_IDX_13 = 13,  //!< DWM_GPIO_IDX_13
	DWM_GPIO_IDX_14 = 14,  //!< DWM_GPIO_IDX_14
	DWM_GPIO_IDX_15 = 15,  //!< DWM_GPIO_IDX_15
	DWM_GPIO_IDX_22 = 22,  //!< DWM_GPIO_IDX_22
	DWM_GPIO_IDX_23 = 23,  //!< DWM_GPIO_IDX_23
	DWM_GPIO_IDX_27 = 27,  //!< DWM_GPIO_IDX_27
	DWM_GPIO_IDX_30 = 30,  //!< DWM_GPIO_IDX_30
	DWM_GPIO_IDX_31 = 31 //!< DWM_GPIO_IDX_31
} dwm_gpio_idx_t;



class dwm1001
{
public:
	dwm1001();
	~dwm1001();
	int dwm_pos_set(dwm_pos_t* pos);
	int dwm_pos_set_xyz(int32_t* p_x, int32_t* p_y, int32_t* p_z);
	int dwm_pos_get(dwm_pos_t* pos);
	int dwm_upd_rate_set(uint16_t ur, uint16_t urs);
	int dwm_upd_rate_get(uint16_t *ur, uint16_t *urs);
	int dwm_cfg_tag_set(dwm_cfg_tag_t* cfg);
	int dwm_cfg_anchor_set(dwm_cfg_anchor_t* cfg);
	int dwm_cfg_get(dwm_cfg_t* cfg);
	int dwm_sleep(void);
	int dwm_anchor_list_get(dwm_anchor_list_t* p_list);
	int dwm_loc_get(dwm_loc_data_t* loc);
	int dwm_baddr_set(dwm_baddr_t* baddr);
	int dwm_baddr_get(dwm_baddr_t* baddr);
	int dwm_stnry_cfg_set(dwm_stnry_sensitivity_t sensitivity);
	int dwm_stnry_cfg_get(dwm_stnry_sensitivity_t* p_sensitivity);
	int dwm_reset(void);
	int dwm_factory_reset(void);
	int dwm_ver_get(dwm_ver_t* ver);

	/**
	 * @brief Sets UWB configuration parameters
	 *
	 * @param[in] p_cfg Configuration parameters
	 *
	 * @return Error code
	 * @retval DWM_OK Success
	 * @retval DWM_ERR_INTERNAL
	 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
	 */
	int dwm_uwb_cfg_set(dwm_uwb_cfg_t *p_cfg);

	/**
	 * @brief Gets UWB configuration parameters
	 *
	 * @param[out] p_cfg Configuration parameters
	 *
	 * @return Error code
	 * @retval DWM_OK Success
	 * @retval DWM_ERR_INTERNAL Can't get configuration
	 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
	 */
	int dwm_uwb_cfg_get(dwm_uwb_cfg_t *p_cfg);

	/**
	 * @brief Read user data received from the network.
	 *
	 * @param[out] p_data Pointer to a receive buffer
	 * @param[in, out] p_len Pointer to number of bytes to be received, if provided
	 * length is not enough(DWM_ERR_OVERRUN), this parameter contains expected lenght
	 *
	 * @return Error code
	 * @retval > 0 Success
	 * @retval 0 If no user data are available.
	 * @retval DWM_ERR_INTERNAL Internal error
	 * @retval DWM_ERR_OVERRUN If length provided is not enough.
	 * @retval DWM_ERR_INVAL_ADDR Null pointer supplied
	 */
	int dwm_usr_data_read(uint8_t* p_data, uint8_t* p_len);

	/**
	 * @brief Nonblocking write of user data to the bridge node.
	 *
	 * @param[in] p_data Pointer to the user data
	 * @param[in] len Number of bytes to be written
	 * @param[in] overwrite If true, might overwrite recent data
	 *
	 * @return Error code
	 * @retval len Success
	 * @retval DWM_ERR_BUSY If overwrite == false and the device is busy
	 * handling last request.
	 * @retval DWM_ERR_INTERNAL No backhaul available
	 * @retval DWM_ERR_INVAL_PARAM If length exceeds DWM_USR_DATA_LEN_MAX.
	 * @retval DWM_ERR_INVAL_ADDR Null pointer supplied
	 */
	int dwm_usr_data_write(uint8_t* p_data, uint8_t len, bool overwrite);

	/**
	 * @brief Reads label from the node
	 *
	 * @param[out] p_label Pointer to label buffer
	 * @param[in, out] p_len Pointer to number of bytes, if provided
	 * length is not enough(DWM_ERR_OVERRUN) then this parameter contains expected
	 * length DWM_LABEL_LEN_MAX
	 *
	 * @return Error code
	 * @retval 0 Success
	 * @retval DWM_ERR_INTERNAL Internal error
	 * @retval DWM_ERR_OVERRUN If length provided is not enough.
	 * @retval DWM_ERR_INVAL_ADDR Null pointer supplied
	 */
	int dwm_label_read(uint8_t* p_label, uint8_t* p_len);

	/**
	 * @brief Writes label to the node, nonblocking.
	 *
	 * @param[in] p_label Pointer to label bytes
	 * @param[in] len Number of bytes to be written
	 *
	 * @return Error code
	 * @retval len Success
	 * @retval DWM_ERR_INTERNAL Internal error
	 * @retval DWM_ERR_INVAL_PARAM If length exceeds DWM_LABEL_LEN_MAX.
	 * @retval DWM_ERR_INVAL_ADDR Null pointer supplied
	 */
	int dwm_label_write(uint8_t* p_label, uint8_t len);

};











/**
 * @brief Enumerator used for selecting the pin to be pulled down
 * or up at the time of pin configuration
 */
typedef enum {
	DWM_GPIO_PIN_NOPULL = 0,
	DWM_GPIO_PIN_PULLDOWN = 1,
	DWM_GPIO_PIN_PULLUP = 3
} dwm_gpio_pin_pull_t;

/**
 * @brief Configures pin as output and sets the pin value
 *
 * @param[in] idx, Pin index (see dwm_gpio_idx_t)
 *
 * @param[in] value, Initial value of the output pin
 *
 * @return Error code
 */
int dwm_gpio_cfg_output(dwm_gpio_idx_t idx, bool value);

/**
 * @brief Configures pin as input and sets pull mode
 *
 * @param[in] idx, Pin index (see dwm_gpio_idx_t)
 *
 * @param[in] pull_mode, Pull mode of the input pin (see dwm_gpio_pin_pull_t)
 *
 * @return Error code
 */
int dwm_gpio_cfg_input(dwm_gpio_idx_t idx, dwm_gpio_pin_pull_t pull_mode);

/**
 * @brief Sets value of the output pin
 *
 * @param[in] idx, Pin index (see dwm_gpio_idx_t)
 *
 * @param[in] value, Pin value (0, 1)
 *
 * @return Error code
 */
int dwm_gpio_value_set(dwm_gpio_idx_t idx, bool value);

/**
 * @brief Gets value of the input pin
 *
 * @param[in] idx, Pin index (see dwm_gpio_idx_t)
 *
 * @param[out] value, Pointer to the pin value
 *
 * @return Error code
 */
int dwm_gpio_value_get(dwm_gpio_idx_t idx, bool* value);

/**
 * @brief Toggles value of the output pin
 *
 * @param[in] idx, Pin index (see dwm_gpio_idx_t)
 *
 * @return Error code
 */
int dwm_gpio_value_toggle(dwm_gpio_idx_t idx);

/**
 * @brief Sets PAN ID
 *
 * @param[in] panid The id
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INTERNAL Can't set PAN ID
 */
int dwm_panid_set(uint16_t panid);

/**
 * @brief Gets PAN ID
 *
 * @param[out] p_panid Pointer to the PAN ID
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INTERNAL Can't read PAN ID
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 */
int dwm_panid_get(uint16_t *p_panid);


/**
 * @brief Gets UWB address/ID of the node
 *
 * @param[out] p_node_id Pointer to the node ID
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INTERNAL Can't read node ID
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 */
int dwm_node_id_get(uint64_t *p_node_id);


/**
 * @brief DWM status bits mask
 */ 
typedef enum {
	API_STATUS_FLAG_LOC_READY = 1<<0,
	API_STATUS_FLAG_UWBMAC_JOINED = 1<<1,
	API_STATUS_FLAG_BH_STATUS_CHANGED = 1<<2,
	API_STATUS_FLAG_BH_DATA_READY = 1<<3,
	API_STATUS_FLAG_BH_INITIALIZED = 1<<4,
	API_STATUS_FLAG_UWB_SCAN_READY = 1<<5,
	API_STATUS_FLAG_USR_DATA_READY = 1<<6,
	API_STATUS_FLAG_USR_DATA_SENT  = 1<<7,
	API_STATUS_FLAG_FWUP_IN_PROGRESS = 1<<8
} api_status_flags_t;

/**
 * @brief DWM status
 */
typedef struct dwm_status_t {
	bool loc_data;
	bool uwbmac_joined;
	bool bh_data_ready;
	bool bh_status_changed;
	bool bh_initialized;
	bool uwb_scan_ready;
	bool usr_data_ready;
	bool usr_data_sent;
	bool fwup_in_progress;
} dwm_status_t;

/**
 * @brief Get system status: Location Data ready
 *
 * @param[out] status
 *
 * @return Error code
 */
int dwm_status_get(dwm_status_t* p_status);


/*
 * @brief bit definitions for TLV Type request: DWM1001_TLV_TYPE_CMD_INT_CFG
**/
/* Interrupts */
#define DWM1001_INTR_NONE                    0
#define DWM1001_INTR_LOC_READY				   (1 << 0)	/* location data ready */
#define DWM1001_INTR_SPI_DATA_READY			   (1 << 1)	/* SPI data ready */
#define DWM1001_INTR_UWBMAC_STATUS_CHANGED	(1 << 2)	/* UWBMAC status changed */
#define DWM1001_INTR_UWBMAC_BH_DATA_READY	   (1 << 3)	/* UWBMAC backhaul data ready*/
#define DWM1001_INTR_UWBMAC_BH_AVAILABLE		(1 << 4)	/* backhaul available */
#define DWM1001_INTR_UWBMAC_SCAN_RESULT		(1 << 5)	/* UWBMAC scan result */
#define DWM1001_INTR_UWBMAC_USR_DATA_READY	(1 << 6)	/* new user data ready*/
#define DWM1001_INTR_UWBMAC_JOINED_CHANGED	(1 << 7)	/* UWBMAC joined */
#define DWM1001_INTR_UWBMAC_USR_DATA_SENT		(1 << 8)	/* user data TX completed over UWBMAC */

/**
 * @brief Enables interrupt generation for various events
 *
 * @param[in] value, interrupt values among: 
 * DWM1001_INTR_NONE                   0
 * DWM1001_INTR_LOC_READY		         (1 << 0)
 * DWM1001_INTR_SPI_DATA_READY	      (1 << 1)
 * DWM1001_INTR_BH_STATUS_CHANGED      (1 << 2)
 * DWM1001_INTR_BH_DATA_READY	         (1 << 3)
 * DWM1001_INTR_BH_AVAILABLE_CHANGED   (1 << 4)
 * DWM1001_INTR_UWB_SCAN_READY	      (1 << 5)
 * DWM1001_INTR_USR_DATA_READY	      (1 << 6)
 * DWM1001_INTR_UWBMAC_JOINED_CHANGED  (1 << 7)
 * DWM1001_INTR_UWBMAC_USR_DATA_SENT	(1 << 8)
 *
 * @return Error code
 */
int dwm_int_cfg_set(uint16_t value);

/**
 * @brief Get status of interrupt generation events
 *
 * @param[in] value, interrupt values among: 
 * DWM1001_INTR_NONE                   0
 * DWM1001_INTR_LOC_READY		         (1 << 0)
 * DWM1001_INTR_SPI_DATA_READY	      (1 << 1)
 * DWM1001_INTR_BH_STATUS_CHANGED      (1 << 2)
 * DWM1001_INTR_BH_DATA_READY	         (1 << 3)
 * DWM1001_INTR_BH_AVAILABLE_CHANGED   (1 << 4)
 * DWM1001_INTR_UWB_SCAN_READY	      (1 << 5)
 * DWM1001_INTR_USR_DATA_READY	      (1 << 6)
 * DWM1001_INTR_UWBMAC_JOINED_CHANGED  (1 << 7)
 * DWM1001_INTR_UWBMAC_USR_DATA_SENT	(1 << 8)
 *
 * @return Error code
 */
int dwm_int_cfg_get(uint16_t *p_value);
   
typedef struct {
	uint16_t node_id;
	uint8_t bh_seat;
	uint8_t hop_lvl;
}origin_info_t;

typedef struct {
	uint16_t sf_number;
	uint32_t bh_seat_map;
	uint8_t origin_cnt;
	origin_info_t origin_info[DWM_API_BH_ORIGIN_CNT_MAX];
}bh_status_t;
/**
 * @brief Reads UWBMAC status
 *
 * @param[out] p_params->v[0].p_value Pointer to UWBMAC status
 *
 * @return 0 if success, > 0 if error
 */
int dwm_bh_status_get(bh_status_t * p_bh_status);


/* Encryption key length */
#define DWM_ENC_KEY_LEN 16
/**
 * @brief encryption key
 */
typedef struct {
	uint8_t byte[DWM_ENC_KEY_LEN];
}dwm_enc_key_t;

/**
 * @brief Sets encryption key. The key is stored in nonvolatile memory.
 * he key that consists of just zeros is considered as invalid. If key is set,
 * the node can enable encryption automatically. Automatic enabling of the
 * encryption is triggered via UWB network when the node detects encrypted
 * message and is capable of decrypting the messages with the key. BLE option
 * is disabled when encryption is enabled automatically. The encryption can
 * be disabled by clearing the key (dwm_enc_key_clear).
 * This call writes to internal flash in case of new value being set, hence
 * should not be used frequently and can take in worst case hundreds of
 * milliseconds! Requires reset for new configuration to take effect.
 *
 * @param[in] p_key Pointer to encryption key structure
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INVAL_INTERNAL key can't be set or key is just all zeros
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 */
int dwm_enc_key_set(dwm_enc_key_t* p_key);

/**
 * @brief Clears the encryption key and disables encryption option if
 * enabled. Does nothing if the key is not set. This call writes to
 * internal flash in case of new value being set, hence should not be used
 * frequently and can take in worst case hundreds of milliseconds! Requires
 * reset for new configuration to take effect.
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INVAL_INTERNAL can't be cleared
 */
int dwm_enc_key_clear(void);

// =======================================================================================
// =======================================================================================
// =======================================================================================
// ================================ internal api for test ================================
// =======================================================================================
// =======================================================================================
// =======================================================================================

/* valid UWB preamble codes */
typedef enum {
	DWM_UWB_PRAMBLE_CODE_9 = 9,
	DWM_UWB_PRAMBLE_CODE_10 = 10,
	DWM_UWB_PRAMBLE_CODE_11 = 11,
	DWM_UWB_PRAMBLE_CODE_12 = 12
} dwm_uwb_preamble_code_t;

/**
 * @brief Sets UWB preamble code
 *
 * @param[in] code One of the preamble codes in dwm_uwb_preamble_code_t
 *
 * @return Error code
 * @retval DWM_OK Success
 *
 * @retval DWM_ERR_INTERNAL DWM internal error
 * @retval DWM_ERR_INVAL_PARAM Invalid preamble code
 */
int dwm_uwb_preamble_code_set(dwm_uwb_preamble_code_t code); // todo

/**
 * @brief Gets UWB preamble code
 *
 * @param[out] p_code Pointer to the preamble code
 *
 * @return Error code
 * @retval DWM_OK Success
 *
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 * @retval DWM_ERR_INTERNAL DWM internal error
 */
int dwm_uwb_preamble_code_get(dwm_uwb_preamble_code_t *p_code); // todo


#define DWM_UWB_SCAN_RESULT_CNT_MAX 16

/**
 * @brief UWB scan result data
 */
typedef struct {
	uint8_t cnt;
	uint8_t mode[DWM_UWB_SCAN_RESULT_CNT_MAX];
	int8_t rssi[DWM_UWB_SCAN_RESULT_CNT_MAX];
} dwm_uwb_scan_result_t;

/**
 * @brief Starts UWB scan
 *
 * @return Error code
 * @retval DWM_OK Success
 *
 * @retval DWM_ERR_INTERNAL DWM internal error
 */
int dwm_uwb_scan_start(void);

/**
 * @brief Read latest UWB scan result
 *
 * @param[out] p_result The scan result data pointer 
 * output = err_code, mode, rssi, [mode, rssi]
 * mode = 1 byte, unsigned integer
   mode 0: 
      channel=5 
      PRF=64 
      PREAMBLE_LENGTH=128 
      PAC_SIZE=8 
      TXCODE=9 
      RXCODE=9 
      STANDARD_FRAME_DELIM=0 
      BAUD_RATE=6800 
      PHR_EXTENDED 
      SFD_TIMEOUT=129 
      SMARTPOWER=1 
   mode 1: 
      channel=5 
      PRF=16 
      PREAMBLE_LENGTH=128 
      PAC_SIZE=8 
      TXCODE=9 
      RXCODE=9 
      STANDARD_FRAME_DELIM=0 
      BAUD_RATE=6800 
      PHR_EXTENDED 
      SFD_TIMEOUT=129 
      SMARTPOWER=1      
 * rssi = 1 byte, signed integer (* UWB signal strength indication in dB*)
 * @return Error code
 * @retval DWM_OK Success
 *
 * @retval DWM_ERR_INTERNAL DWM internal error
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 */
int dwm_uwb_scan_result_get(dwm_uwb_scan_result_t *p_result);

#endif //_DWM_API_H_

