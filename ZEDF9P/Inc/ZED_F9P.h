/*
 * ZED_F9P.h
 *
 *  Created on: Aug 21, 2024
 *      Author: brian
 */

#ifndef INC_ZED_F9P_H_
#define INC_ZED_F9P_H_

/*
 * INCLUDES
 */
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"

/*
 * DEFINES
 */
#define F9P_UBX_SYNCH_1         0xB5
#define F9P_UBX_SYNCH_2         0x62

// UBX message classes
static const uint8_t UBX_NAV_CLASS = 0x01;
static const uint8_t UBX_CFG_CLASS = 0x06;

// UBX message IDs
static const uint8_t UBX_NAV_PVT_ID = 0x07;
static const uint8_t UBX_CFG_VALSET = 0x8a;

// UBX message lengths
static const uint8_t UBX_NAV_PVT_LEN = 92;

/*
 * Structs & Enums
 */

// 3.2 UBX frame structure, Interface Description
typedef struct {
  uint8_t cls;
  uint8_t id;
  uint16_t length; // length of payload
  uint8_t* payload;
  uint8_t checksum_a;
  uint8_t checksum_b;
} UbxPacket;

// 3.2 UBX frame structure, Interface Description
typedef enum {
  UBX_MSG_SYNC_1,
  UBX_MSG_SYNC_2,
  UBX_MSG_CLASS,
  UBX_MSG_ID,
  UBX_MSG_LEN_LSB,
  UBX_MSG_LEN_MSB,
  UBX_MSG_PAYLOAD,
  UBX_MSG_CHK_A,
  UBX_MSG_CHK_B,
  UBX_MSG_DONE, // Message is complete & checksum passes
  UBX_MSG_ERROR
} UbxPacketState;

typedef struct {
  UbxPacket message;
  UbxPacketState state;
  uint32_t index;
} UbxMessageParser;

typedef struct {
  UART_HandleTypeDef *uartHandle;
  UbxMessageParser ubx_uart_parser;
  float longitude;
  float latitude;
} ZedF9P;

/*
 * Functions
 */
void F9P_update(ZedF9P *gps);
void F9P_init(ZedF9P *gps, UART_HandleTypeDef *huart);
void F9P_parse_byte_uart(UbxMessageParser *parser, uint8_t byte);
uint32_t extract_int32(uint8_t *buf, uint16_t index);
void generate_ubx_message_checksum(UbxPacket *message);
void F9P_apply_configurations(ZedF9P *gps);
void F9P_enable_periodic_updates(ZedF9P *gps);

#endif /* INC_ZED_F9P_H_ */
