/*
 * ZED_F9P.cpp
 *
 *  Created on: Aug 21, 2024
 *      Author: brian
 *
 *  All references to documentation refers to:
 *  u-blox F9 high precision GNSS receiver - Interface Description
 */

#include "ZED_F9P.h"
#include "debug.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>



// Sync (2) + body(4) + Checksum(2)
const uint32_t UBX_FRAME_DEFAULT_SIZE = 8;

// extern UART_HandleTypeDef huart4;

void generate_ubx_buffer_checksum(uint8_t *buffer, uint32_t size) {
  // Taken from pseudo code found in 3.4 UBX checksum
  // 8-bit Fletcher algorithm
  uint8_t ck_a = 0;
  uint8_t ck_b = 0;

  for (int i = 2; i < size - 2; i++) {
    ck_a = (ck_a + buffer[i]);
    ck_b = (ck_b + ck_a);
  }

  buffer[size - 2] = ck_a;
  buffer[size - 1] = ck_b;
}


/*
 * Sets the checksum values of message. Assumes rest of message is correctly set.
 */
void generate_ubx_message_checksum(IN OUT UbxPacket *message) {
  // 3.2 UBX frame structure, Interface Description
  const int MSG_HEAD_BUFFER_LENGTH = 4;

  uint8_t ck_a = 0;
  uint8_t ck_b = 0;

  uint8_t head_buffer[MSG_HEAD_BUFFER_LENGTH];
  head_buffer[0] = message->cls;
  head_buffer[1] = message->id;
  head_buffer[2] = (uint8_t)(message->length);
  head_buffer[3] = (uint8_t)(message->length >> 8);

  // run checksum over head (class, id, length)
  for (int i = 0; i < MSG_HEAD_BUFFER_LENGTH; i++) {
    ck_a = (ck_a + head_buffer[i]);
    ck_b = (ck_b + ck_a);
  }

  // run checksum over payload
  for (int i = 0; i < message->length; i++) {
    ck_a = (ck_a + message->payload[i]);
    ck_b = (ck_b + ck_a);
  }

  message->checksum_a = ck_a;
  message->checksum_b = ck_b;
}

void F9P_send_uart_ubx_message(ZedF9P *gps, UbxPacket *message) {
  uint32_t buffer_size = UBX_FRAME_DEFAULT_SIZE + (uint32_t)message->length;
  uint8_t buf[buffer_size];
  // Taken from 3.2 UBX frame structure
  buf[0] = F9P_UBX_SYNCH_1;
  buf[1] = F9P_UBX_SYNCH_2;
  buf[2] = message->cls;
  buf[3] = message->id;
  buf[4] = (uint8_t)(message->length >> 8);
  buf[5] = (uint8_t)(message->length);

  for (int i = 0; i < message->length; i++) {
    buf[6 + i] = message->payload[i];
  }

  generate_ubx_buffer_checksum(buf, buffer_size);

  //  for (uint32_t i = 0; i < buffer_size; i++) DEBUG_LOG("%02x ", buf[i]);
  HAL_UART_Transmit(gps->uartHandle, buf, buffer_size, HAL_MAX_DELAY);
}

void F9P_init(IN OUT ZedF9P *gps, IN UART_HandleTypeDef *huart) {
  DEBUG_LOG("ZED INIT! \r\n");
  gps->uartHandle = huart;
}

void F9P_parse_incoming_uart(ZedF9P *gps) {
  uint8_t buffer[100];
  uint32_t timeout = 1000;
  HAL_StatusTypeDef status = HAL_UART_Receive(gps->uartHandle,
                                              buffer,
                                              sizeof(buffer)-1,
                                              timeout);
  if (status == HAL_OK) {
//    DEBUG_LOG("%s\r\n", buffer);
    DEBUG_LOG("NEW MESSAGE");
  }
}

void F9P_apply_configurations(ZedF9P *gps) {
  // disable NMEA output on UART1 to reduce noise in UART lines
  UbxPacket disable_nmea_cmd;
  disable_nmea_cmd.cls = UBX_CFG_CLASS;
  disable_nmea_cmd.id = UBX_CFG_VALSET;

  uint8_t cfg_uart1output_payload[] = {
    // valset header info (4 bytes)
    0x0, // version
    0x7, // layers configuration should be saved (7 = all)
    0x0, 0x0, // reserved

    // Disable NMEA on UART1 command (5 bytes)
    0x10, 0x74, 0x00, 0x02, // CFG-UART1OUTPROT-NMEA key id
    0, // value (false)
  };

  disable_nmea_cmd.payload = cfg_uart1output_payload;

  disable_nmea_cmd.length = 4 + 5;

  F9P_send_uart_ubx_message(gps, &disable_nmea_cmd);
}

void F9P_parse_byte_uart(UbxMessageParser *parser, uint8_t byte) {
  switch (parser->state) {
    case UBX_MSG_SYNC_1:
      if (byte == F9P_UBX_SYNCH_1) {
        parser->state = UBX_MSG_SYNC_2;
      }
      break;
    case UBX_MSG_SYNC_2:
      if (byte == F9P_UBX_SYNCH_2) {
        parser->state = UBX_MSG_CLASS;
      } else {
        parser->state = UBX_MSG_ERROR;
      }
      break;
    case UBX_MSG_CLASS:
      parser->message.cls = byte;
      parser->state = UBX_MSG_ID;
      break;
    case UBX_MSG_ID:
      parser->message.id = byte;
      parser->state = UBX_MSG_LEN_LSB;
      break;
    case UBX_MSG_LEN_LSB:
      parser->message.length = byte;
      parser->state = UBX_MSG_LEN_MSB;
      break;
    case UBX_MSG_LEN_MSB:
      parser->message.length |= (uint16_t)byte << 8;
      parser->state = UBX_MSG_PAYLOAD;
      parser->index = 0;
      
      // TODO: check if MSG_LEN matches expected for CLASS + ID
      
      if (parser->message.payload != NULL) free(parser->message.payload); // maybe return with error code instead?
      parser->message.payload = (uint8_t *)malloc(parser->message.length * sizeof(uint8_t));
      break;
    case UBX_MSG_PAYLOAD:
      parser->message.payload[parser->index++] = byte;
      if (parser->index >= parser->message.length) {
        parser->state = UBX_MSG_CHK_A;
      }
      break;
    case UBX_MSG_CHK_A:
      parser->message.checksum_a = byte;
      parser->state = UBX_MSG_CHK_B;
      break;
    case UBX_MSG_CHK_B:
      parser->message.checksum_b = byte;
      parser->state = UBX_MSG_DONE;
      break;
    default:
      break;
  }

  if (parser->state == UBX_MSG_DONE) {
    // Process the message
    // TODO: implement checksum and state machine to handle message
    printf("NEW UBX MESSAGE\r\n");
    int32_t raw_lon_val = extract_int32(parser->message.payload, 24);
    int32_t raw_lat_val = extract_int32(parser->message.payload, 28);

    printf("Raw Longitude: %ld \r\n", raw_lon_val);
    printf("Raw Latitude: %ld \r\n", raw_lat_val);

    printf("Longitude: %.2f \r\n", (float)raw_lon_val * 1e-7);
    printf("Latitude: %.2f \r\n", (float)raw_lat_val * 1e-7);

    // NOTE: Unable to print float values!
    // TODO: write reset function for message to prevent memory leak
    parser->state = UBX_MSG_SYNC_1;
  } else if (parser->state == UBX_MSG_ERROR) {
    parser->state = UBX_MSG_SYNC_1;
  }
}

void F9P_enable_periodic_updates(ZedF9P *gps) {
    // disable NMEA output on UART1 to reduce noise in UART lines
  UbxPacket set_pvt_out_rate_cmd;
  set_pvt_out_rate_cmd.cls = UBX_CFG_CLASS;
  set_pvt_out_rate_cmd.id = UBX_CFG_VALSET;

  uint8_t config_payload[] = {
    // valset header info (4 bytes)
    0x0, // version
    0x7, // layers configuration should be saved (7 = all)
    0x0, 0x0, // reserved

    // Set output rate of NAV PVT on UART1 (5 bytes)
    0x20, 0x91, 0x00, 0x07, // CFG-MSGOUT-UBX_NAV_PVT_UART1 key id
    20, // value (20 times a second)
  };

  set_pvt_out_rate_cmd.payload = config_payload;

  set_pvt_out_rate_cmd.length = 4 + 5;

  F9P_send_uart_ubx_message(gps, &set_pvt_out_rate_cmd);
}

void reset_ubx_parser(UbxMessageParser *parser) {
  // parser->message.payload;
}

void reset_ubx_message(UbxPacketState *message) {
}

/*
* assumes little endian byte order
*/
uint32_t extract_int32(uint8_t *buf, uint16_t index) {
  return (uint32_t)buf[index] | (uint32_t)buf[index + 1] << 8 | (uint32_t)buf[index + 2] << 16 | (uint32_t)buf[index + 3] << 24;
}

void F9P_update(ZedF9P *gps) {
  UbxPacket message;
  message.cls = UBX_NAV_CLASS;
  message.id = UBX_NAV_PVT_ID;
  message.length = 0;

  F9P_send_uart_ubx_message(gps, &message);
//  F9P_parse_incoming_uart(gps);
}
