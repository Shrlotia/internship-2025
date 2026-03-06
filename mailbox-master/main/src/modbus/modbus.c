#include "modbus.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "esp_log.h"

#include "utils.h"

#define UART_PORT_NUM 1
#define MODBUS_RX_IO_PIN 40
#define MODBUS_TX_IO_PIN 38
#define MODBUS_RTS_IO_PIN 39

#define MODBUS_TIMEOUT pdMS_TO_TICKS(500)

static const char *TAG = "MODBUS";

static uint16_t crc16(const uint8_t *buffer, size_t buffer_length);
static bool rxFsm(void);

static uart_port_t s_portNum = UART_NUM_0;
static int s_txIoNum = UART_PIN_NO_CHANGE;
static int s_rxIoNum = UART_PIN_NO_CHANGE;
static int s_rtsIoNum = UART_PIN_NO_CHANGE;

static QueueHandle_t uartQueue;
static TaskHandle_t uartTaskHandle;
static TaskHandle_t uartQueueTaskHandle = NULL;

typedef enum {
  STATE_RX_STILL,
  STATE_RX_WAITING,
  STATE_RX_RCV,
} modbus_rxState_t;

static FHCallback callback = NULL;

void registerCallback(FHCallback _callback) {
  callback = _callback;
}

static volatile modbus_rxState_t rxState = STATE_RX_STILL;

static uint8_t rxBuf[256];
static uint16_t rxLen = 0;

static uint8_t txBuf[256];
static uint16_t txLen = 0;

static void prepareRequest(uint8_t serverAddr, uint8_t fc) {
  txBuf[0] = serverAddr;
  txBuf[1] = fc;
  txLen = 2;
}

static void sendRequest() {
  uint16_t crc = crc16(txBuf, txLen);
  writeUInt16LE(txBuf, crc, txLen);
  txLen += 2;
  ESP_LOGI(TAG, "Sending Modbus request...");
  uart_write_bytes(s_portNum, txBuf, txLen);
  if (uart_wait_tx_done(s_portNum, pdMS_TO_TICKS(1000)) == ESP_OK) {
    ESP_LOGI(TAG, "Request sent successfully.");
    ESP_LOG_BUFFER_HEXDUMP(TAG, txBuf, 10, ESP_LOG_INFO);
  } else {
    ESP_LOGE(TAG, "Failed to send request.");
  }
  rxState = STATE_RX_WAITING;
}

// Function 01 Request
static void readCoilRegistersReq(uint16_t startingAddress, uint16_t quantityOfOutputs) {
  writeUInt16BE(txBuf, startingAddress, 2);
  writeUInt16BE(txBuf, quantityOfOutputs, 4);
  txLen += 4;
  sendRequest();
}

// Function 03 request
static void readHoldingRegReq(uint16_t startingAddress, uint16_t quantityOfRegisters) {
  writeUInt16BE(txBuf, startingAddress, 2);
  writeUInt16BE(txBuf, quantityOfRegisters, 4);
  txLen += 4;
  sendRequest();
}

// Function 05 request
static void writeSingleCoilReq(uint16_t coilAddress, uint16_t coilValue) {
  writeUInt16BE(txBuf, coilAddress, 2);
  writeUInt16BE(txBuf, coilValue, 4);
  txLen += 4;
  sendRequest();
}

// Function 06 request
static void writeSingleRegisterReq(uint16_t registerAddress, uint16_t registerValue) {
  writeUInt16BE(txBuf, registerAddress, 2);
  writeUInt16BE(txBuf, registerValue, 4);
  txLen += 4;
  sendRequest();
}

// // Function 16 request
// static void writeMultipleRegistersReq(uint16_t startingAddress, uint16_t quantityOfRegisters, const uint8_t *registersValue, uint8_t byteCount) {
//   writeUInt16BE(txBuf, startingAddress, 2);
//   writeUInt16BE(txBuf, quantityOfRegisters, 4);
//   txBuf[6] = byteCount;
//   for (int i = 0; i < quantityOfRegisters; i++) {
//     writeUInt16BE(txBuf, registersValue[i*2], 7 + i*2);
//   }
//   txLen += 5;
//   txLen += byteCount;
//   sendRequest();
// }

// Function 01 respone 
static modbus_ret_t readCoilRegistersRes(uint8_t serverAddr, uint16_t startingAddress, uint16_t quantityOfOutputs, uint8_t *buf) {
  uint8_t addr = rxBuf[0];
  ESP_LOG_BUFFER_HEXDUMP(TAG, rxBuf, 10, ESP_LOG_INFO);
  if (addr != serverAddr) {
    ESP_LOGW(TAG, "Wrong server addr in read coil register response: %d", addr);
    return MODBUS_FAILED;
  }
  uint8_t fc = rxBuf[1];
  if (fc == 0x83) {
    ESP_LOGW(TAG, "Read coil register return error code");
    return MODBUS_FAILED;
  } else if (fc != 0x01) {
    ESP_LOGW(TAG, "Read coil register return wrong function code (0x%x)", fc);
    return MODBUS_FAILED;
  }
  uint16_t crc = readUInt16LE(rxBuf, rxLen - 2);
  if (crc != crc16(rxBuf, rxLen - 2)) {
    ESP_LOGW(TAG, "CRC mismatch");
    return MODBUS_FAILED;
  }
  uint8_t byteCount = rxBuf[2];
  if (byteCount + 5 != rxLen) {
    ESP_LOGW(TAG, "Invalid byte count or response length");
    return MODBUS_FAILED;
  }
  int len = 0;
  if (buf != NULL) {
    for (int i = 3; i < rxLen - 2; i++) {
      buf[len++] = rxBuf[i];
    }
  }
  return MODBUS_OK;
}

// Function 03 respone
static modbus_ret_t readHoldingRegistersRes(uint8_t serverAddr, uint16_t startingAddress, uint16_t numberOfRegisters, uint8_t *buf) {
  uint8_t addr = rxBuf[0];
  ESP_LOG_BUFFER_HEXDUMP(TAG, rxBuf, 10, ESP_LOG_INFO);
  if (addr != serverAddr) {
    ESP_LOGW(TAG, "Wrong server addr in read holding register response: %d", addr);
    ESP_LOG_BUFFER_HEXDUMP(TAG, rxBuf, 256, ESP_LOG_INFO);
    return MODBUS_FAILED;
  }
  uint8_t fc = rxBuf[1];
  if (fc == 0x83) {
    ESP_LOGW(TAG, "read holding register return error code");
    return MODBUS_FAILED;
  } else if (fc != 0x03) {
    ESP_LOGW(TAG, "read holding register return wrong function code (0x%x)", fc);
    return MODBUS_FAILED;
  }
  uint16_t crc = readUInt16LE(rxBuf, rxLen - 2);
  if (crc != crc16(rxBuf, rxLen - 2)) {
    ESP_LOGW(TAG, "CRC mismatch");
    return MODBUS_FAILED;
  }
  uint8_t byteCount = rxBuf[2];
  if (byteCount != numberOfRegisters * 2) {
    ESP_LOGW(TAG, "byte count and number of registers mismatch");
    return MODBUS_FAILED;
  }
  int len = 0;
  if (buf != NULL) {
    for (int i = 3; i < rxLen - 2; i++) {
      buf[len++] = rxBuf[i];
    }
  }
  return MODBUS_OK;
}

// Function 05 respone
static modbus_ret_t writeSingleCoilRes(uint8_t serverAddr, uint16_t coilAddress, uint16_t coilValue) {
  if (rxLen != 8) {
    ESP_LOGW(TAG, "Invalid response length for write single coil");
    return MODBUS_FAILED;
  }
  uint8_t addr = rxBuf[0];
    ESP_LOG_BUFFER_HEXDUMP(TAG, rxBuf, 10, ESP_LOG_INFO);
  if (addr != serverAddr) {
    ESP_LOGW(TAG, "Wrong server addr in write single coil response");
    ESP_LOG_BUFFER_HEXDUMP(TAG, rxBuf, 256, ESP_LOG_INFO);
    return MODBUS_FAILED;
  }
  uint8_t fc = rxBuf[1];
  if (fc == 0x85) {
    ESP_LOGW(TAG, "Write single coil return error code");
    return MODBUS_FAILED;
  } else if (fc != 0x05) {
    ESP_LOGW(TAG, "Write single coil return wrong function code (0x%x)", fc);
    return MODBUS_FAILED;
  }
  uint16_t crc = readUInt16LE(rxBuf, rxLen - 2);
  if (crc != crc16(rxBuf, rxLen - 2)) {
    ESP_LOGW(TAG, "CRC mismatch");
    return MODBUS_FAILED;
  }
  uint16_t _coilAddress = readUInt16BE(rxBuf, 2);
  if (coilAddress != _coilAddress) {
    ESP_LOGW(TAG, "Coil address mismatch: expected %d, got %d", coilAddress, _coilAddress);
    return MODBUS_FAILED;
  }
  uint16_t _coilValue = readUInt16BE(rxBuf, 4);
  if (coilValue != _coilValue) {
    ESP_LOGW(TAG, "Coil value mismatch: expected %d, got %d", coilValue, _coilValue);
    return MODBUS_FAILED;
  }
  return MODBUS_OK;
}

// Function 06 respone
static modbus_ret_t writeSingleRegisterRes(uint8_t serverAddr, uint16_t registerAddress, uint16_t registerValue) {
  if (rxLen != 8) {
    ESP_LOGW(TAG, "Invalid response length for write single register");
    return MODBUS_FAILED;
  }
  uint8_t addr = rxBuf[0];
  if (addr != serverAddr) {
    ESP_LOGW(TAG, "Wrong server addr in read write single register response");
    return MODBUS_FAILED;
  }
  uint8_t fc = rxBuf[1];
  if (fc == 0x86) {
    ESP_LOGW(TAG, "Write single register return error code");
    return MODBUS_FAILED;
  } else if (fc != 0x06) {
    ESP_LOGW(TAG, "Write single register return wrong function code (0x%x)", fc);
    return MODBUS_FAILED;
  }
  uint16_t crc = readUInt16LE(rxBuf, rxLen - 2);
  if (crc != crc16(rxBuf, rxLen - 2)) {
    ESP_LOGW(TAG, "CRC mismatch");
    return MODBUS_FAILED;
  }
  uint16_t _registerAddress = readUInt16BE(rxBuf, 2);
  if (registerAddress != _registerAddress) {
    ESP_LOGW(TAG, "Register address mismatch: expected %d, got %d", registerAddress, _registerAddress);
    return MODBUS_FAILED;
  }
  uint16_t _registerValue = readUInt16BE(rxBuf, 4);
  if (registerValue != _registerValue) {
    ESP_LOGW(TAG, "Register value mismatch: expected %d, got %d", registerValue, _registerValue);
    return MODBUS_FAILED;
  }
  return MODBUS_OK;
}

// function 01 send
modbus_ret_t sendReadCoilRegister (uint8_t slaveAddress, uint16_t startingAddress, uint16_t quantityOfOutputs, uint8_t *buf) {
  prepareRequest(slaveAddress, 0x01);
  readCoilRegistersReq(startingAddress, quantityOfOutputs);
  ESP_LOGI(TAG, "Requesting coil for slave %d", slaveAddress);
  uartQueueTaskHandle = xTaskGetCurrentTaskHandle();
  if (xTaskNotifyWait(pdFALSE, pdFALSE, NULL, MODBUS_TIMEOUT) == pdFALSE) {
    ESP_LOGW(TAG, "request read coils timed out.");
    uartQueueTaskHandle = NULL;
    return MODBUS_FAILED;
  }
  uartQueueTaskHandle = NULL;
  return readCoilRegistersRes(slaveAddress, startingAddress, quantityOfOutputs, buf);
}

// function 03 send
modbus_ret_t sendReadHoldingRegister (uint8_t slaveAddress, uint16_t startingAddress, uint16_t quantityOfRegisters, uint8_t *buf) {
  prepareRequest(slaveAddress, 0x03);
  readHoldingRegReq(startingAddress, quantityOfRegisters);
  ESP_LOGI(TAG, "Requesting register for slave %d", slaveAddress);
  uartQueueTaskHandle = xTaskGetCurrentTaskHandle();
  if (xTaskNotifyWait(pdFALSE, pdFALSE, NULL, MODBUS_TIMEOUT) == pdFALSE) {
    ESP_LOGW(TAG, "request read holding register timed out.");
    uartQueueTaskHandle = NULL;
  return MODBUS_FAILED;
  }
  uartQueueTaskHandle = NULL;
  return readHoldingRegistersRes(slaveAddress, startingAddress, quantityOfRegisters, buf);
}

// function 05 send
modbus_ret_t sendWriteSingleCoil (uint8_t slaveAddress, uint16_t coilAddress, uint16_t coilValue) {
  prepareRequest(slaveAddress, 0x05);
  writeSingleCoilReq(coilAddress, coilValue);
  ESP_LOGI(TAG, "Requesting coil for slave %d", slaveAddress);
  uartQueueTaskHandle = xTaskGetCurrentTaskHandle();
  if (xTaskNotifyWait(pdFALSE, pdFALSE, NULL, MODBUS_TIMEOUT) == pdFALSE) {
    ESP_LOGW(TAG, "request write single coil timed out for %d", coilAddress);
    uartQueueTaskHandle = NULL;
  return MODBUS_FAILED;
  }
  uartQueueTaskHandle = NULL;
  return writeSingleCoilRes(slaveAddress, coilAddress, coilValue);
}

// function 06 send
modbus_ret_t sendWriteSingleRegister (uint8_t slaveAddress, uint16_t registerAddress, uint16_t registerValue) {
  prepareRequest(slaveAddress, 0x06);
  writeSingleRegisterReq(registerAddress, registerValue);
  ESP_LOGI(TAG, "Requesting register for slave %d", slaveAddress);
  uartQueueTaskHandle = xTaskGetCurrentTaskHandle();
  if (xTaskNotifyWait(pdFALSE, pdFALSE, NULL, MODBUS_TIMEOUT) != pdTRUE) {
    ESP_LOGW(TAG, "request write single register timed out for %d", registerAddress);
    uartQueueTaskHandle = NULL;
    return MODBUS_FAILED;
  }
  uartQueueTaskHandle = NULL;
  return writeSingleRegisterRes(slaveAddress, registerAddress, registerValue);
}

static void uart_task(void *pvParameters) {
  uart_event_t event;
  for (;;) {
    if (xQueueReceive(uartQueue, (void *)&event, portMAX_DELAY)) {
      ESP_LOGI(TAG, "UART received something");
      switch(event.type) {
        case UART_DATA:
          ESP_LOGV(TAG, "Data event, length: %zu", event.size);
          ESP_ERROR_CHECK(uart_get_buffered_data_len(s_portNum, &event.size));
          bool read = true;
          uint16_t count = 0;
          while (read && (count++ < event.size)) {
            read = rxFsm();
          }
          if (rxState != STATE_RX_STILL) {
            xTaskNotify(uartQueueTaskHandle, 0, eNoAction);
          }
          uart_flush_input(s_portNum);
          break;
        case UART_FIFO_OVF:
          ESP_LOGW(TAG, "Hardware FIFO overflow");
          xQueueReset(uartQueue);
          uart_flush_input(s_portNum);
          break;
        case UART_BUFFER_FULL:
          ESP_LOGW(TAG, "Ring buffer full");
          uart_flush_input(s_portNum);
          xQueueReset(uartQueue);
          break;
        case UART_BREAK:
          ESP_LOGW(TAG, "UART RX break detected");
          break;
        case UART_PARITY_ERR:
          ESP_LOGW(TAG, "UART parity error detected");
          break;
        case UART_FRAME_ERR:
          ESP_LOGW(TAG, "UART frame error detected");
          break;
        default:
          ESP_LOGV(TAG, "Unknown UART event type: %u", event.type);
          break;
      }
    }
  }
  vTaskDelete(NULL);
}

static bool rxFsm(void) {
  uint8_t byte;
  bool read = uart_read_bytes(s_portNum, &byte, 1, pdMS_TO_TICKS(1)) == 1;
  if (read) {
    switch (rxState) {
      case STATE_RX_STILL:
        ESP_LOGW(TAG, "Received something while not sending request");
        rxState = STATE_RX_WAITING;
        break;
      case STATE_RX_WAITING:
        rxLen = 0;
        rxBuf[rxLen++] = byte;
        rxState = STATE_RX_RCV;
        break;
      case STATE_RX_RCV:
        if (rxLen < sizeof(rxBuf)) {
          rxBuf[rxLen++] = byte;
        } else {
          ESP_LOGE(TAG, "Buffer overflow");
          rxState = STATE_RX_STILL;
        }
      break;
    }
  }
  return read;
}

void modbus_init() {
  s_portNum = UART_PORT_NUM;
  if (MODBUS_TX_IO_PIN != UART_PIN_NO_CHANGE) s_txIoNum = MODBUS_TX_IO_PIN;
  if (MODBUS_RX_IO_PIN != UART_PIN_NO_CHANGE) s_rxIoNum = MODBUS_RX_IO_PIN;
  if (MODBUS_RTS_IO_PIN != UART_PIN_NO_CHANGE) s_rtsIoNum = MODBUS_RTS_IO_PIN;

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
  ESP_ERROR_CHECK(uart_set_pin(s_portNum, s_txIoNum, s_rxIoNum, s_rtsIoNum, UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(uart_set_mode(s_portNum, UART_MODE_RS485_HALF_DUPLEX));
  uart_set_always_rx_timeout(s_portNum, true);

  BaseType_t status = xTaskCreate(uart_task, "uart_queue_task", 4096, NULL, 10, &uartTaskHandle);
  if (status != pdPASS) {
    vTaskDelete(uartTaskHandle);
    ESP_LOGE(TAG, "mb stack task creation error (0x%x)", status);
    return;
  } else {
    vTaskSuspend(uartTaskHandle);
  }
}

void modbus_start() {
  vTaskResume(uartTaskHandle);
}

void modbus_stop() {
  vTaskSuspend(uartTaskHandle);
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