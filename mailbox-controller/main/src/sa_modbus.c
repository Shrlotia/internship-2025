#include "sa_modbus.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_intr_alloc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "hal/gpio_types.h"
#include "portmacro.h"
#include "soc/uart_reg.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "sa_utils.h"

static uint16_t crc16(const uint8_t *buffer, size_t buffer_length);
static void timerExpired(void *param);
static bool rxFsm(void);

static uint8_t s_serverAddr = 1;
static uart_port_t s_portNum = UART_NUM_0;
static int s_txIoNum = UART_PIN_NO_CHANGE;
static int s_rxIoNum = UART_PIN_NO_CHANGE;
static int s_rtsIoNum = UART_PIN_NO_CHANGE;

static const char *TAG = "sa_modbus";

static sa_modbus_readCoils_cb s_readCoilsCb = NULL;
static sa_modbus_readHoldingRegisters_cb s_readHoldingRegistersCb = NULL;
static sa_modbus_writeSingleCoil_cb s_writeSingleCoilCb = NULL;
static sa_modbus_writeSingleRegister_cb s_writeSingleRegisterCb = NULL;
static sa_modbus_writeMultipleRegisters_cb s_writeMultipleRegistersCb = NULL;

static QueueHandle_t uartQueue;
static TaskHandle_t uartTaskHandle;
static TaskHandle_t modbusSlaveTaskHandle;
static esp_timer_handle_t timerHandle;

typedef enum {
  STATE_RX_INIT,
  STATE_RX_IDLE,
  STATE_RX_RCV,
  STATE_RX_ERROR,
} sa_modbus_rxState;

typedef enum {
  EV_READY = 0x01,
  EV_FRAME_RECEIVED = 0x02,
  EV_EXECUTE = 0x04,
  EV_FRAME_SENT = 0x08,
  EV_FRAME_TRANSMIT = 0x10,
} sa_modbus_event_type;

static volatile sa_modbus_rxState rxState;

static uint8_t rxBuf[256];
static uint16_t rxLen = 0;

static uint8_t txBuf[256];
static uint16_t txLen = 0;

static void prepareResponse(uint8_t fc) {
  txBuf[0] = s_serverAddr;
  txBuf[1] = fc;
  txLen = 2;
}

static void sendResponse() {
  uint16_t crc = crc16(txBuf, txLen);
  writeUInt16LE(txBuf, crc, txLen);
  txLen += 2;
  ESP_LOGD(TAG, "Sending response...");
  ESP_LOG_BUFFER_HEXDUMP(TAG, txBuf, txLen, ESP_LOG_VERBOSE);
  uart_write_bytes(s_portNum, txBuf, txLen);
  if (uart_wait_tx_done(s_portNum, pdMS_TO_TICKS(2200)) != ESP_OK) {
    ESP_LOGE(TAG, "mb serial sent buffer failure.");
  }
}

static void sendExceptionResponse(uint8_t exceptionCode) {
  txBuf[1] |= (1 << 7);
  txBuf[2] = exceptionCode;
  txLen = 3;
  sendResponse();
}

static void handleReadCoils(uint16_t startingAddress, uint16_t numberOfCoils, bool isBroadcast) {
  if (isBroadcast) return;
  if (numberOfCoils < 1 || numberOfCoils > 0x07D0) {
    sendExceptionResponse(SA_MODBUS_ILLEGAL_DATA_VALUE);
    return;
  }
  if ((uint32_t)startingAddress + (uint32_t)numberOfCoils > 0xFFFF) {
    sendExceptionResponse(SA_MODBUS_ILLEGAL_DATA_ADDRESS);
    return;
  }
  uint8_t ret = s_readCoilsCb(startingAddress, numberOfCoils, &txBuf[3], &txBuf[2]);
  if (ret == 0) {
    txLen += txBuf[2] + 1;
    sendResponse();
  } else {
    sendExceptionResponse(ret);
  }
}

static void handleReadHoldingRegisters(uint16_t startingAddress, uint16_t numberOfRegisters, bool isBroadcast) {
  if (isBroadcast) return;
  if (numberOfRegisters < 1 || numberOfRegisters > 0x007D) {
    sendExceptionResponse(SA_MODBUS_ILLEGAL_DATA_VALUE);
    return;
  }
  if ((uint32_t)startingAddress + (uint32_t)numberOfRegisters > 0xFFFF) {
    sendExceptionResponse(SA_MODBUS_ILLEGAL_DATA_ADDRESS);
    return;
  }
  uint8_t ret = s_readHoldingRegistersCb(startingAddress, numberOfRegisters, &txBuf[3], &txBuf[2]);
  if (ret == 0) {
    txLen += txBuf[2] + 1;
    sendResponse();
  } else {
    sendExceptionResponse(ret);
  }
}

static void handleWriteSingleCoil(uint16_t outputAddress, uint16_t outputValue, bool isBroadcast) {
  if (outputValue != 0x0000 && outputValue != 0xFF00) {
    if (!isBroadcast) sendExceptionResponse(SA_MODBUS_ILLEGAL_DATA_VALUE);
    return;
  }
  uint8_t ret = s_writeSingleCoilCb(outputAddress, outputValue);
  if (!isBroadcast) {
    if (ret == 0) {
      writeUInt16BE(txBuf, outputAddress, 2);
      writeUInt16BE(txBuf, outputValue, 4);
      txLen += 4;
      sendResponse();
    } else {
      sendExceptionResponse(ret);
    }
  }
}

static void handleWriteSingleRegister(uint16_t registerAddress, uint16_t registerValue, bool isBroadcast) {
  uint8_t ret = s_writeSingleRegisterCb(registerAddress, registerValue);
  if (!isBroadcast) {
    if (ret == 0) {
      writeUInt16BE(txBuf, registerAddress, 2);
      writeUInt16BE(txBuf, registerValue, 4);
      txLen += 4;
      sendResponse();
    } else {
      sendExceptionResponse(ret);
    }
  }
}

static void handleWriteMultipleRegisters(
  uint16_t startingAddress,
  uint16_t quantityOfRegisters,
  const uint8_t *registersValue,
  uint8_t byteCount,
  bool isBroadcast
) {
  // TODO: validation
  uint8_t ret = s_writeMultipleRegistersCb(startingAddress, quantityOfRegisters, registersValue, byteCount);
  if (!isBroadcast) {
    if (ret == 0) {
      writeUInt16BE(txBuf, startingAddress, 2);
      writeUInt16BE(txBuf, quantityOfRegisters, 2);
      txLen += 4;
      sendResponse();
    } else {
      sendExceptionResponse(ret);
    }
  }
}

static void processFrame(const uint8_t *frame, size_t size) {
  ESP_LOGV(TAG, "Received:");
  ESP_LOG_BUFFER_HEXDUMP(TAG, frame, size, ESP_LOG_VERBOSE);

  uint8_t addr = frame[0];
  if (addr != 0 && addr != s_serverAddr) {
    ESP_LOGD(TAG, "Received a frame for %d instead", addr);
    return;
  }
  bool isBroadcast = addr == 0;

  uint8_t fc = frame[1];
  ESP_LOG_BUFFER_HEXDUMP(TAG, frame, size, ESP_LOG_INFO);
  uint16_t crc = readUInt16LE(frame, size - 2);
  if (crc != crc16(frame, size - 2)) {
    ESP_LOGW(TAG, "CRC mismatch (crc: 0x%04x)", crc);
    return;
  }
  ESP_LOGI(TAG, "fc: %u", fc);
  switch (fc) {
    case 0x01:
      prepareResponse(fc);
      handleReadCoils(readUInt16BE(frame, 2), readUInt16BE(frame, 4), isBroadcast);
      break;
    case 0x03:
      prepareResponse(fc);
      handleReadHoldingRegisters(readUInt16BE(frame, 2), readUInt16BE(frame, 4), isBroadcast);
      break;
    case 0x05:
      prepareResponse(fc);
      handleWriteSingleCoil(readUInt16BE(frame, 2), readUInt16BE(frame, 4), isBroadcast);
      break;
    case 0x06:
      prepareResponse(fc);
      handleWriteSingleRegister(readUInt16BE(frame, 2), readUInt16BE(frame, 4), isBroadcast);
      break;
    case 0x10:
      prepareResponse(fc);
      handleWriteMultipleRegisters(readUInt16BE(frame, 2), readUInt16BE(frame, 4), &frame[7], frame[6], isBroadcast);
      break;
    default: {
      if (!isBroadcast) {
        prepareResponse(fc);
        sendExceptionResponse(SA_MODBUS_ILLEGAL_FUNCTION);
      }
      break;
    }
  }
}

static void modbus_slave_task(void *pvParameters) {
  uint32_t ulNotifiedValue;
  BaseType_t xResult;
  for (;;) {
    xResult = xTaskNotifyWait(pdFALSE, UINT32_MAX, &ulNotifiedValue, portMAX_DELAY);
    if (xResult == pdPASS) {
      if ((ulNotifiedValue & EV_READY) != 0) {
        ESP_LOGD(TAG, "EV_READY");
      }
      if ((ulNotifiedValue & EV_FRAME_RECEIVED) != 0) {
        processFrame(rxBuf, rxLen);
      }
    }
  }
}

static void uart_task(void *pvParameters) {
  uart_event_t event;
  for (;;) {
    if (xQueueReceive(uartQueue, (void *)&event, portMAX_DELAY)) {
      switch(event.type) {
        case UART_DATA:
          ESP_LOGV(TAG, "Data event, length: %zu", event.size);
          if (event.timeout_flag) {
            ESP_ERROR_CHECK(uart_get_buffered_data_len(s_portNum, &event.size));
            bool read = true;
            uint16_t count = 0;
            while (read && (count++ <= event.size)) {
              read = rxFsm();
            }
            uart_flush_input(s_portNum);
          }
          break;
        case UART_FIFO_OVF:
          ESP_LOGV(TAG, "hw fifo overflow");
          xQueueReset(uartQueue);
          break;
        case UART_BUFFER_FULL:
          ESP_LOGV(TAG, "ring buffer full");
          uart_flush_input(s_portNum);
          xQueueReset(uartQueue);
          break;
        case UART_BREAK:
          ESP_LOGV(TAG, "uart rx break");
          break;
        case UART_PARITY_ERR:
          ESP_LOGV(TAG, "uart parity error");
          break;
        case UART_FRAME_ERR:
          ESP_LOGV(TAG, "uart frame error");
          break;
        default:
          ESP_LOGV(TAG, "uart event type: %u", event.type);
          break;
      }
    }
  }
  vTaskDelete(NULL);
}

void sa_modbus_init(
  int portNum,
  int txIoNum,
  int rxIoNum,
  int rtsIoNum,
  uint8_t serverAddr,
  sa_modbus_readCoils_cb readCoilsCb,
  sa_modbus_readHoldingRegisters_cb readHoldingRegistersCb,
  sa_modbus_writeSingleCoil_cb writeSingleCoilCb,
  sa_modbus_writeSingleRegister_cb writeSingleRegisterCb,
  sa_modbus_writeMultipleRegisters_cb writeMultipleRegistersCb
) {
  s_portNum = portNum;
  if (txIoNum != UART_PIN_NO_CHANGE) s_txIoNum = txIoNum;
  if (rxIoNum != UART_PIN_NO_CHANGE) s_rxIoNum = rxIoNum;
  if (rtsIoNum != UART_PIN_NO_CHANGE) s_rtsIoNum = rtsIoNum;
  s_serverAddr = serverAddr;
  s_readCoilsCb = readCoilsCb;
  s_readHoldingRegistersCb = readHoldingRegistersCb;
  s_writeSingleCoilCb = writeSingleCoilCb;
  s_writeSingleRegisterCb = writeSingleRegisterCb;
  s_writeMultipleRegistersCb = writeMultipleRegistersCb;

  uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_EVEN,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 2,
    .source_clk = UART_SCLK_DEFAULT,
  };

  ESP_ERROR_CHECK(uart_param_config(s_portNum, &uart_config));
  ESP_ERROR_CHECK(uart_driver_install(s_portNum, 256, 256, 20, &uartQueue, 0));
  ESP_LOGI(TAG, "Setting uart pin (portNum: %d, txIoNum: %d, rxIoNum: %d, rtsIoNum: %d)", s_portNum, s_txIoNum, s_rxIoNum, s_rtsIoNum);
  ESP_ERROR_CHECK(uart_set_pin(s_portNum, s_txIoNum, s_rxIoNum,
                               s_rtsIoNum, UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(uart_set_mode(s_portNum, UART_MODE_RS485_HALF_DUPLEX));
  uart_set_always_rx_timeout(s_portNum, true);

  esp_timer_create_args_t timerConf = {
    .callback = timerExpired,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "t32Timer",
  };
  ESP_ERROR_CHECK(esp_timer_create(&timerConf, &timerHandle));

  BaseType_t status = xTaskCreate(uart_task, "uart_queue_task", 4096, NULL, 10, &uartTaskHandle);
  if (status != pdPASS) {
    vTaskDelete(uartTaskHandle);
    ESP_LOGE(TAG, "mb stack task creation error (0x%x)", status);
    return;
  } else {
    vTaskSuspend(uartTaskHandle);
  }

  xTaskCreate(modbus_slave_task, "modbus_slave_task", 4096, NULL, 9, &modbusSlaveTaskHandle);
}

static void startTimer() {
  esp_timer_stop(timerHandle);
  esp_timer_start_once(timerHandle, 1750);
}

static void stopTimer() {
  esp_timer_stop(timerHandle);
}

void sa_modbus_start() {
  vTaskResume(uartTaskHandle);
  rxState = STATE_RX_INIT;
  startTimer();
}

void sa_modbus_stop() {
  vTaskSuspend(uartTaskHandle);
}

static bool rxFsm(void) {
  uint8_t byte;
  bool read = uart_read_bytes(s_portNum, &byte, 1, pdMS_TO_TICKS(1)) == 1;
  // ESP_LOGI(TAG, "byte received: 0x%0x", byte);
  switch (rxState) {
    case STATE_RX_INIT:
      // ESP_LOGI(TAG, "Current state: rx init");
      rxLen = 0;
      rxState = STATE_RX_IDLE;
      startTimer();
      break;
    case STATE_RX_ERROR:
      // ESP_LOGI(TAG, "Current state: rx error");
      startTimer();
      break;
    case STATE_RX_IDLE:
      // ESP_LOGI(TAG, "Current state: rx idle");
      rxLen = 0;
      rxBuf[rxLen++] = byte;
      rxState = STATE_RX_RCV;
      startTimer();
      break;
    case STATE_RX_RCV:
      // ESP_LOGI(TAG, "Current state: rx rcv");
      if (rxLen >= 256) {
        rxState = STATE_RX_ERROR;
      } else {
        if (read) {
          rxBuf[rxLen++] = byte;
          // ESP_LOG_BUFFER_HEXDUMP(TAG, rxBuf, 256, ESP_LOG_INFO);
        }
      }
      startTimer();
      break;
  }
  return read;
}

static void timerExpired(void *param) {
  switch (rxState) {
    case STATE_RX_INIT:
      xTaskNotify(modbusSlaveTaskHandle, EV_READY, eSetBits);
      break;
    case STATE_RX_RCV:
      xTaskNotify(modbusSlaveTaskHandle, EV_FRAME_RECEIVED, eSetBits);
      break;
    case STATE_RX_ERROR:
      break;
    default:
      break;
  }
  stopTimer();
  rxState = STATE_RX_IDLE;
}

static const uint8_t table_crc_hi[] = {
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
  0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
  0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
  0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
  0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
  0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
  0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
  0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
  0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

/* Table of CRC values for low-order byte */
static const uint8_t table_crc_lo[] = {
  0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
  0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
  0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
  0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
  0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
  0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
  0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
  0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
  0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
  0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
  0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
  0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
  0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
  0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
  0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
  0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
  0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
  0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
  0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
  0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
  0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
  0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
  0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
  0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
  0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
  0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

static uint16_t crc16(const uint8_t *buffer, size_t buffer_length) {
  uint8_t crc_hi = 0xFF; /* high CRC byte initialized */
  uint8_t crc_lo = 0xFF; /* low CRC byte initialized */
  unsigned int i; /* will index into CRC lookup */

  while (buffer_length--) {
    i = crc_lo ^ *buffer++; /* calculate the CRC  */
    crc_lo = crc_hi ^ table_crc_hi[i];
    crc_hi = table_crc_lo[i];
  }
  return (crc_hi << 8 | crc_lo);
}