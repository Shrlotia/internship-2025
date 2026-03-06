#include "modbus/modbus_data_api.h"

#include "driver/gpio.h"
#include "hal/gpio_types.h"

#include "esp_log.h"

#include "event/modbus_return.h"
#include "modbus/modbus_struct.h"
#include "modbus/modbus_function.h"
#include "utils.h"

// Write Coil Values
#define COIL_ON   0xFF00
#define COIL_OFF  0x0000

static const char *TAG = "MODBUS_DATA";

static const uint8_t s_dsIoNums[8] = {20, 19, 21, 33, 34, 35, 36, 37};

static uint8_t slave_ids[MAX_SLAVE_ID];
static uint8_t num_of_slaves = 0;

static modbus_stage_t slaveStage[MAX_SLAVE_ID];
static modbus_data_t currentSettings[MAX_SLAVE_ID];
static modbus_data_t recievedSettings;
static modbus_data_t desiredSettings;

// Getters for coil values
static ret_t getCoil_IR_Trio(modbus_data_t *settings, uint8_t *index, bool *values, size_t *values_size) {
  if (settings == NULL || values == NULL) return DATA_FAIL;
  if (index == NULL && values_size != NULL) {
    if (*values_size < COIL_IR_TRIO_ENABLED_COUNT) return DATA_FAIL;
    memcpy(values, settings->Coil_IR_Trio, COIL_IR_TRIO_ENABLED_COUNT * sizeof(bool));
    return DATA_OK;
  }
  if (index != NULL && values_size == NULL) {
    if (*index >= COIL_IR_TRIO_ENABLED_COUNT) return DATA_FAIL;
    *values = settings->Coil_IR_Trio[*index];
    return DATA_OK;
  }
  return DATA_FAIL;
}

static ret_t getCoil_IR_Mono(modbus_data_t *settings, uint8_t *index, bool *values, size_t *values_size) {
  if (settings == NULL || values == NULL) return DATA_FAIL;
  if (index == NULL && values_size != NULL) {
    if (*values_size < COIL_IR_MONO_ENABLED_COUNT) return DATA_FAIL;
    memcpy(values, settings->Coil_IR_Mono, COIL_IR_MONO_ENABLED_COUNT * sizeof(bool));
    return DATA_OK;
  }
  if (index != NULL && values_size == NULL) {
    if (*index >= COIL_IR_MONO_ENABLED_COUNT) return DATA_FAIL;
    *values = settings->Coil_IR_Mono[*index];
    return DATA_OK;
  }
  return DATA_FAIL;
}

static ret_t getCoil_Lock(modbus_data_t *settings, uint8_t *index, bool *values, size_t *values_size) {
  if (settings == NULL || values == NULL) return DATA_FAIL;
  if (index == NULL && values_size != NULL) {
    if (*values_size < COIL_LOCK_ENABLED_COUNT) return DATA_FAIL;
    memcpy(values, settings->Coil_Lock, COIL_LOCK_ENABLED_COUNT * sizeof(bool));
    return DATA_OK;
  }
  if (index != NULL && values_size == NULL) {
    if (*index >= COIL_LOCK_ENABLED_COUNT) return DATA_FAIL;
    *values = settings->Coil_Lock[*index];
    return DATA_OK;
  }
  return DATA_FAIL;
}

static ret_t getCoil_LED(modbus_data_t *settings, uint8_t *index, bool *values, size_t *values_size) {
  if (settings == NULL || values == NULL) return DATA_FAIL;
  if (index == NULL && values_size != NULL) {
    if (*values_size < COIL_LED_ENABLED_COUNT) return DATA_FAIL;
    memcpy(values, settings->Coil_LED, COIL_LED_ENABLED_COUNT * sizeof(bool));
    return DATA_OK;
  }
  if (index != NULL && values_size == NULL) {
    if (*index >= COIL_LED_ENABLED_COUNT) return DATA_FAIL;
    *values = settings->Coil_LED[*index];
    return DATA_OK;
  }
  return DATA_FAIL;
}

static ret_t getCoil_IR_Trio_State(modbus_data_t *settings, uint8_t *index, bool *values, size_t *values_size) {
  if (settings == NULL || values == NULL) return DATA_FAIL;
  if (index == NULL && values_size != NULL) {
    if (*values_size < COIL_IR_TRIO_STATE_COUNT) return DATA_FAIL;
    memcpy(values, settings->Coil_IR_Trio_State, COIL_IR_TRIO_STATE_COUNT * sizeof(bool));
    return DATA_OK;
  }
  if (index != NULL && values_size == NULL) {
    if (*index >= COIL_IR_TRIO_STATE_COUNT) return DATA_FAIL;
    *values = settings->Coil_IR_Trio_State[*index];
    return DATA_OK;
  }
  return DATA_FAIL;
}

static ret_t getCoil_IR_Mono_State(modbus_data_t *settings, uint8_t *index, bool *values, size_t *values_size) {
  if (settings == NULL || values == NULL) return DATA_FAIL;
  if (index == NULL && values_size != NULL) {
    if (*values_size < COIL_IR_MONO_STATE_COUNT) return DATA_FAIL;
    memcpy(values, settings->Coil_IR_Mono_State, COIL_IR_MONO_STATE_COUNT * sizeof(bool));
    return DATA_OK;
  }
  if (index != NULL && values_size == NULL) {
    if (*index >= COIL_IR_MONO_STATE_COUNT) return DATA_FAIL;
    *values = settings->Coil_IR_Mono_State[*index];
    return DATA_OK;
  }
  return DATA_FAIL;
}

static ret_t getCoil_IR_Latched_State(modbus_data_t *settings, uint8_t *index, bool *values, size_t *values_size) {
  if (settings == NULL || values == NULL) return DATA_FAIL;
  if (index == NULL && values_size != NULL) {
    if (*values_size < COIL_IR_LATCHED_STATE_COUNT) return DATA_FAIL;
    memcpy(values, settings->Coil_IR_Latched_State, COIL_IR_LATCHED_STATE_COUNT * sizeof(bool));
    return DATA_OK;
  }
  if (index != NULL && values_size == NULL) {
    if (*index >= COIL_IR_LATCHED_STATE_COUNT) return DATA_FAIL;
    *values = settings->Coil_IR_Latched_State[*index];
    return DATA_OK;
  }
  return DATA_FAIL;
}

static ret_t getCoil_Blink_LEDS(modbus_data_t *settings, uint8_t *index, bool *values, size_t *values_size) {
  if (settings == NULL || values == NULL) return DATA_FAIL;
  if (index == NULL && values_size != NULL) {
    if (*values_size < COIL_BLINK_LEDS_COUNT) return DATA_FAIL;
    memcpy(values, settings->Coil_Blink_LEDS, COIL_BLINK_LEDS_COUNT * sizeof(bool));
    return DATA_OK;
  }
  if (index != NULL && values_size == NULL) {
    if (*index >= COIL_BLINK_LEDS_COUNT) return DATA_FAIL;
    *values = settings->Coil_Blink_LEDS[*index];
    return DATA_OK;
  }
  return DATA_FAIL;
}

static ret_t getCoil_Lock_State(modbus_data_t *settings, uint8_t *index, bool *values, size_t *values_size) {
  if (settings == NULL || values == NULL) return DATA_FAIL;
  if (index == NULL && values_size != NULL) {
    if (*values_size < COIL_LOCK_STATE_COUNT) return DATA_FAIL;
    memcpy(values, settings->Coil_Lock_State, COIL_LOCK_STATE_COUNT * sizeof(bool));
    return DATA_OK;
  }
  if (index != NULL && values_size == NULL) {
    if (*index >= COIL_LOCK_STATE_COUNT) return DATA_FAIL;
    *values = settings->Coil_Lock_State[*index];
    return DATA_OK;
  }
  return DATA_FAIL;
}

static ret_t getCoil_OTA_Update_Start_Command(modbus_data_t *settings, uint8_t *index, bool *values, size_t *values_size) {
  if (settings == NULL || values == NULL) return DATA_FAIL;
  if (index == NULL && values_size != NULL) {
    if (*values_size < COIL_OTA_UPDATE_START_COMMAND_COUNT) return DATA_FAIL;
    memcpy(values, settings->Coil_OTA_Update_Start_Command, COIL_OTA_UPDATE_START_COMMAND_COUNT * sizeof(bool));
    return DATA_OK;
  }
  if (index != NULL && values_size == NULL) {
    if (*index >= COIL_OTA_UPDATE_START_COMMAND_COUNT) return DATA_FAIL;
    *values = settings->Coil_OTA_Update_Start_Command[*index];
    return DATA_OK;
  }
  return DATA_FAIL;
}

static ret_t getCoil_OTA_Update_Finish_Command(modbus_data_t *settings, uint8_t *index, bool *values, size_t *values_size) {
  if (settings == NULL || values == NULL) return DATA_FAIL;
  if (index == NULL && values_size != NULL) {
    if (*values_size < COIL_OTA_UPDATE_FINISH_COMMAND_COUNT) return DATA_FAIL;
    memcpy(values, settings->Coil_OTA_Update_Finish_Command, COIL_OTA_UPDATE_FINISH_COMMAND_COUNT * sizeof(bool));
    return DATA_OK;
  }
  if (index != NULL && values_size == NULL) {
    if (*index >= COIL_OTA_UPDATE_FINISH_COMMAND_COUNT) return DATA_FAIL;
    *values = settings->Coil_OTA_Update_Finish_Command[*index];
    return DATA_OK;
  }
  return DATA_FAIL;
}

static ret_t getCoil_OTA_Update_Abort_Command(modbus_data_t *settings, uint8_t *index, bool *values, size_t *values_size) {
  if (settings == NULL || values == NULL) return DATA_FAIL;
  if (index == NULL && values_size != NULL) {
    if (*values_size < COIL_OTA_UPDATE_ABORT_COMMAND_COUNT) return DATA_FAIL;
    memcpy(values, settings->Coil_OTA_Update_Abort_Command, COIL_OTA_UPDATE_ABORT_COMMAND_COUNT * sizeof(bool));
    return DATA_OK;
  }
  if (index != NULL && values_size == NULL) {
    if (*index >= COIL_OTA_UPDATE_ABORT_COMMAND_COUNT) return DATA_FAIL;
    *values = settings->Coil_OTA_Update_Abort_Command[*index];
    return DATA_OK;
  }
  return DATA_FAIL;
}

// Getters for register values
static ret_t getRegister_Lock_Duration(modbus_data_t *settings, uint16_t *value) {
  if(settings == NULL || value == NULL) return DATA_FAIL;
  *value = settings->Register_Lock_Duration;
  return DATA_OK;
}

static ret_t getRegister_Current_Version(modbus_data_t *settings, uint16_t *value) {
  if(settings == NULL || value == NULL) return DATA_FAIL;
  *value = settings->Register_Current_Version;
  return DATA_OK;
}

static ret_t getRegister_OTA_Update_Data(modbus_data_t *settings, uint16_t *value) {
  if(settings == NULL || value == NULL) return DATA_FAIL;
  *value = settings->Register_OTA_Update_Data;
  return DATA_OK;
}

// Setters for coil values
static ret_t setCoil_IR_Trio(modbus_data_t *settings, uint8_t *index, bool *values, size_t values_size) {
  if (settings == NULL || values == NULL) return DATA_FAIL;
  if (index == NULL) {
    if (values_size < COIL_IR_TRIO_ENABLED_COUNT) return DATA_FAIL;
    memcpy(settings->Coil_IR_Trio, values, sizeof(bool) * COIL_IR_TRIO_ENABLED_COUNT);
  } else {
    if (*index >= COIL_IR_TRIO_ENABLED_COUNT) return DATA_FAIL;
    settings->Coil_IR_Trio[*index] = *values;
  }
  return DATA_OK;
}

static ret_t setCoil_IR_Mono(modbus_data_t *settings, uint8_t *index, bool *values, size_t values_size) {
  if (settings == NULL || values == NULL) return DATA_FAIL;
  if (index == NULL) {
    if (values_size < COIL_IR_MONO_ENABLED_COUNT) return DATA_FAIL;
    memcpy(settings->Coil_IR_Mono, values, sizeof(bool) * COIL_IR_MONO_ENABLED_COUNT);
  } else {
    if (*index >= COIL_IR_MONO_ENABLED_COUNT) return DATA_FAIL;
    settings->Coil_IR_Mono[*index] = *values;
  }
  return DATA_OK;
}

static ret_t setCoil_Lock(modbus_data_t *settings, uint8_t *index, bool *values, size_t values_size) {
  if (settings == NULL || values == NULL) return DATA_FAIL;
  if (index == NULL) {
    if (values_size < COIL_LOCK_ENABLED_COUNT) return DATA_FAIL;
    memcpy(settings->Coil_Lock, values, sizeof(bool) * COIL_LOCK_ENABLED_COUNT);
  } else {
    if (*index >= COIL_LOCK_ENABLED_COUNT) return DATA_FAIL;
    settings->Coil_Lock[*index] = *values;
  }
  return DATA_OK;
}

static ret_t setCoil_LED(modbus_data_t *settings, uint8_t *index, bool *values, size_t values_size) {
  if (settings == NULL || values == NULL) return DATA_FAIL;
  if (index == NULL) {
    if (values_size < COIL_LED_ENABLED_COUNT) return DATA_FAIL;
    memcpy(settings->Coil_LED, values, sizeof(bool) * COIL_LED_ENABLED_COUNT);
  } else {
    if (*index >= COIL_LED_ENABLED_COUNT) return DATA_FAIL;
    settings->Coil_LED[*index] = *values;
  }
  return DATA_OK;
}

static ret_t setCoil_IR_Trio_State(modbus_data_t *settings, uint8_t *index, bool *values, size_t values_size) {
  if (settings == NULL || values == NULL) return DATA_FAIL;
  if (index == NULL) {
    if (values_size < COIL_IR_TRIO_STATE_COUNT) return DATA_FAIL;
    memcpy(settings->Coil_IR_Trio_State, values, sizeof(bool) * COIL_IR_TRIO_STATE_COUNT);
  } else {
    if (*index >= COIL_IR_TRIO_STATE_COUNT) return DATA_FAIL;
    settings->Coil_IR_Trio_State[*index] = *values;
  }
  return DATA_OK;
}

static ret_t setCoil_IR_Mono_State(modbus_data_t *settings, uint8_t *index, bool *values, size_t values_size) {
  if (settings == NULL || values == NULL) return DATA_FAIL;
  if (index == NULL) {
    if (values_size < COIL_IR_MONO_STATE_COUNT) return DATA_FAIL;
    memcpy(settings->Coil_IR_Mono_State, values, sizeof(bool) * COIL_IR_MONO_STATE_COUNT);
  } else {
    if (*index >= COIL_IR_MONO_STATE_COUNT) return DATA_FAIL;
    settings->Coil_IR_Mono_State[*index] = *values;
  }
  return DATA_OK;
}

static ret_t setCoil_IR_Latched_State(modbus_data_t *settings, uint8_t *index, bool *values, size_t values_size) {
  if (settings == NULL || values == NULL) return DATA_FAIL;
  if (index == NULL) {
    if (values_size < COIL_IR_LATCHED_STATE_COUNT) return DATA_FAIL;
    memcpy(settings->Coil_IR_Latched_State, values, sizeof(bool) * COIL_IR_LATCHED_STATE_COUNT);
  } else {
    if (*index >= COIL_IR_LATCHED_STATE_COUNT) return DATA_FAIL;
    settings->Coil_IR_Latched_State[*index] = *values;
  }
  return DATA_OK;
}

static ret_t setCoil_Blink_LEDS(modbus_data_t *settings, uint8_t *index, bool *values, size_t values_size) {
  if (settings == NULL || values == NULL) return DATA_FAIL;
  if (index == NULL) {
    if (values_size < COIL_BLINK_LEDS_COUNT) return DATA_FAIL;
    memcpy(settings->Coil_Blink_LEDS, values, sizeof(bool) * COIL_BLINK_LEDS_COUNT);
  } else {
    if (*index >= COIL_BLINK_LEDS_COUNT) return DATA_FAIL;
    settings->Coil_Blink_LEDS[*index] = *values;
  }
  return DATA_OK;
}

static ret_t setCoil_Lock_State(modbus_data_t *settings, uint8_t *index, bool *values, size_t values_size) {
  if (settings == NULL || values == NULL) return DATA_FAIL;
  if (index == NULL) {
    if (values_size < COIL_LOCK_STATE_COUNT) return DATA_FAIL;
    memcpy(settings->Coil_Lock_State, values, sizeof(bool) * COIL_LOCK_STATE_COUNT);
  } else {
    if (*index >= COIL_LOCK_STATE_COUNT) return DATA_FAIL;
    settings->Coil_Lock_State[*index] = *values;
  }
  return DATA_OK;
}

static ret_t setCoil_OTA_Update_Start_Command(modbus_data_t *settings, uint8_t *index, bool *values, size_t values_size) {
  if (settings == NULL || values == NULL) return DATA_FAIL;
  if (index == NULL) {
    if (values_size < COIL_OTA_UPDATE_START_COMMAND_COUNT) return DATA_FAIL;
    memcpy(settings->Coil_OTA_Update_Start_Command, values, sizeof(bool) * COIL_OTA_UPDATE_START_COMMAND_COUNT);
  } else {
    if (*index >= COIL_OTA_UPDATE_START_COMMAND_COUNT) return DATA_FAIL;
    settings->Coil_OTA_Update_Start_Command[*index] = *values;
  }
  return DATA_OK;
}

static ret_t setCoil_OTA_Update_Finish_Command(modbus_data_t *settings, uint8_t *index, bool *values, size_t values_size) {
  if (settings == NULL || values == NULL) return DATA_FAIL;
  if (index == NULL) {
    if (values_size < COIL_OTA_UPDATE_FINISH_COMMAND_COUNT) return DATA_FAIL;
    memcpy(settings->Coil_OTA_Update_Finish_Command, values, sizeof(bool) * COIL_OTA_UPDATE_FINISH_COMMAND_COUNT);
  } else {
    if (*index >= COIL_OTA_UPDATE_FINISH_COMMAND_COUNT) return DATA_FAIL;
    settings->Coil_OTA_Update_Finish_Command[*index] = *values;
  }
  return DATA_OK;
}

static ret_t setCoil_OTA_Update_Abort_Command(modbus_data_t *settings, uint8_t *index, bool *values, size_t values_size) {
  if (settings == NULL || values == NULL) return DATA_FAIL;
  if (index == NULL) {
    if (values_size < COIL_OTA_UPDATE_ABORT_COMMAND_COUNT) return DATA_FAIL;
    memcpy(settings->Coil_OTA_Update_Abort_Command, values, sizeof(bool) * COIL_OTA_UPDATE_ABORT_COMMAND_COUNT);
  } else {
    if (*index >= COIL_OTA_UPDATE_ABORT_COMMAND_COUNT) return DATA_FAIL;
    settings->Coil_OTA_Update_Abort_Command[*index] = *values;
  }
  return DATA_OK;
}

// Setters for register values
static ret_t setRegister_Lock_Duration(modbus_data_t *settings, uint16_t value) {
  if (settings == NULL) return DATA_FAIL;
  settings->Register_Lock_Duration = value;
  return DATA_OK;
}

static ret_t setRegister_Current_Version(modbus_data_t *settings, uint16_t value) {
  if (settings == NULL) return DATA_FAIL;
  settings->Register_Current_Version = value;
  return DATA_OK;
}

static ret_t setRegister_OTA_Update_Data(modbus_data_t *settings, uint16_t value) {
  if (settings == NULL) return DATA_FAIL;
  settings->Register_OTA_Update_Data = value;
  return DATA_OK;
}

static const uint8_t allCoilUseStartAddress = 0;
static const uint8_t allCoilUseEndAddress = 11;
static const uint8_t allRegisterUseStartAddress = 12;
static const uint8_t allRegisterUseEndAddress = 14;

typedef struct {
  const char* name;
	modbus_function_t event;
  uint16_t start;
	uint16_t end;
  uint16_t count;
	ret_t (*getCoilArray)(modbus_data_t *settings, uint8_t *index, bool *values, size_t *values_size);
	ret_t (*setCoilArray)(modbus_data_t *settings, uint8_t *index, bool *values, size_t values_size);
	ret_t (*getRegisterArray)(modbus_data_t *settings, uint16_t *value);
	ret_t (*setRegisterArray)(modbus_data_t *settings, uint16_t value);
} Range; //range_t

static const Range slaveRanges[] = {
	{ "COIL_IR_TRIO_ENABLED(0)",      COIL_IR_TRIO_ENABLED,           COIL_IR_TRIO_ENABLED_START,     COIL_IR_TRIO_ENABLED_END,       COIL_IR_TRIO_ENABLED_COUNT,           getCoil_IR_Trio,                    setCoil_IR_Trio,                    NULL,                         NULL                        },
	{ "COIL_IR_MONO_ENABLED(1)",      COIL_IR_MONO_ENABLED,           COIL_IR_MONO_ENABLED_START,     COIL_IR_MONO_ENABLED_END,       COIL_IR_MONO_ENABLED_COUNT,           getCoil_IR_Mono,                    setCoil_IR_Mono,                    NULL,                         NULL                        },
	{ "COIL_LOCK_ENABLED(2)",         COIL_LOCK_ENABLED,              COIL_LOCK_ENABLED_START,        COIL_LOCK_ENABLED_END,          COIL_LOCK_ENABLED_COUNT,              getCoil_Lock,                       setCoil_Lock,                       NULL,                         NULL                        },
	{ "COIL_LED_ENABLED(3)",          COIL_LED_ENABLED,               COIL_LED_ENABLED_START,         COIL_LED_ENABLED_END,           COIL_LED_ENABLED_COUNT,               getCoil_LED,                        setCoil_LED,                        NULL,                         NULL                        },
	{ "COIL_IR_TRIO_STATE(4)",        COIL_IR_TRIO_STATE,             COIL_IR_TRIO_STATE_START,       COIL_IR_TRIO_STATE_END,         COIL_IR_TRIO_STATE_COUNT,             getCoil_IR_Trio_State,              setCoil_IR_Trio_State,              NULL,                         NULL                        },
	{ "COIL_IR_MONO_STATE(5)",        COIL_IR_MONO_STATE,             COIL_IR_MONO_STATE_START,       COIL_IR_MONO_STATE_END,         COIL_IR_MONO_STATE_COUNT,             getCoil_IR_Mono_State,              setCoil_IR_Mono_State,              NULL,                         NULL                        },
	{ "COIL_IR_LATCHED_STATE(6)",     COIL_IR_LATCHED_STATE,          COIL_IR_LATCHED_STATE_START,    COIL_IR_LATCHED_STATE_END,      COIL_IR_LATCHED_STATE_COUNT,          getCoil_IR_Latched_State,           setCoil_IR_Latched_State,           NULL,                         NULL                        },
	{ "COIL_BLINK_LEDS(7)",           COIL_BLINK_LEDS_STATE,          COIL_BLINK_LEDS,                COIL_BLINK_LEDS,                COIL_BLINK_LEDS_COUNT,                getCoil_Blink_LEDS,                 setCoil_Blink_LEDS,                 NULL,                         NULL                        },
	{ "COIL_LOCK_STATE(8)",           COIL_LOCK_STATE,                COIL_LOCK_STATE_START,          COIL_LOCK_STATE_END,            COIL_LOCK_STATE_COUNT,                getCoil_Lock_State,                 setCoil_Lock_State,                 NULL,                         NULL                        },
	{ "COIL_OTA_START(9)",            COIL_OTA_UPDATE_START,          COIL_OTA_UPDATE_START_COMMAND,  COIL_OTA_UPDATE_START_COMMAND,  COIL_OTA_UPDATE_START_COMMAND_COUNT,  getCoil_OTA_Update_Start_Command,   setCoil_OTA_Update_Start_Command,   NULL,                         NULL                        },
	{ "COIL_OTA_FINISH(10)",          COIL_OTA_UPDATE_FINISH,         COIL_OTA_UPDATE_FINISH_COMMAND, COIL_OTA_UPDATE_FINISH_COMMAND, COIL_OTA_UPDATE_FINISH_COMMAND_COUNT, getCoil_OTA_Update_Finish_Command,  setCoil_OTA_Update_Finish_Command,  NULL,                         NULL                        },
	{ "COIL_OTA_ABORT(11)",           COIL_OTA_UPDATE_ABORT,          COIL_OTA_UPDATE_ABORT_COMMAND,  COIL_OTA_UPDATE_ABORT_COMMAND,  COIL_OTA_UPDATE_ABORT_COMMAND_COUNT,  getCoil_OTA_Update_Abort_Command,   setCoil_OTA_Update_Abort_Command,   NULL,                         NULL                        },
	{ "REGISTER_LOCK_DURATION(12)",   REGISTER_LOCK_DURATION_STATE,   REGISTER_LOCK_DURATION,         REGISTER_LOCK_DURATION,         REGISTER_LOCK_DURATION_COUNT,         NULL,                               NULL,                               getRegister_Lock_Duration,    setRegister_Lock_Duration   },
	{ "REGISTER_CURRENT_VERSION(13)", REGISTER_CURRENT_VERSION_STATE, REGISTER_CURRENT_VERSION_START, REGISTER_CURRENT_VERSION_END,   REGISTER_CURRENT_VERSION_COUNT,       NULL,                               NULL,                               getRegister_Current_Version,  setRegister_Current_Version },
	{ "REGISTER_OTA_UPDATE_DATA(14)", REGISTER_OTA_UPDATE_DATA_STATE, REGISTER_OTA_UPDATE_DATA_START, REGISTER_OTA_UPDATE_DATA_START, REGISTER_OTA_UPDATE_DATA_COUNT,       NULL,                               NULL,                               getRegister_OTA_Update_Data,  setRegister_OTA_Update_Data }
};
static const uint8_t rangeMapCount = sizeof(slaveRanges) / sizeof(slaveRanges[0]);

// Getter and Setter for currentSetting
ret_t getCurrentSettingCoilValue(uint8_t rangeId, uint8_t *index, bool *value, size_t *value_size) {
  if (rangeId > rangeMapCount || value == NULL ) return DATA_FAIL;
  return slaveRanges[rangeId].getCoilArray(currentSettings, index, value, value_size);
}

ret_t getCurrentSettingRegisterValue(uint8_t rangeId, uint16_t *value) {
  if (rangeId > rangeMapCount || value == NULL ) return DATA_FAIL;
  return slaveRanges[rangeId].getRegisterArray(currentSettings, value);
}

ret_t setCurrentSettingCoilValue(uint8_t rangeId, uint8_t *index, bool *values, size_t values_size) {
  if (rangeId > rangeMapCount || value == NULL ) return DATA_FAIL;
  return slaveRanges[rangeId].setCoilArray(currentSettings, index, values, values_size);
}

ret_t setCurrentSettingRegisterValue(uint8_t rangeId, uint16_t value) {
  if (rangeId > rangeMapCount || value == NULL ) return DATA_FAIL;
  return slaveRanges[rangeId].setRegisterArray(currentSettings, value);
}

// Getter for recievedSettings
ret_t getRecievedSettingCoilValue(uint8_t rangeId, uint8_t *index, bool *value, size_t *value_size) {
  if (rangeId > rangeMapCount || value == NULL ) return DATA_FAIL;
  return slaveRanges[rangeId].getCoilArray(recievedSettings, index, value, value_size);
}

ret_t getRecievedSettingRegisterValue(uint8_t rangeId, uint16_t *value) {
  if (rangeId > rangeMapCount || value == NULL ) return DATA_FAIL;
  return slaveRanges[rangeId].getRegisterArray(recievedSettings, value);
}

// Setter for desiredSettings
ret_t setDesiredSettingCoilValue(uint8_t rangeId, uint8_t *index, bool *values, size_t values_size) {
  if (rangeId > rangeMapCount || value == NULL ) return DATA_FAIL;
  return slaveRanges[rangeId].setCoilArray(desiredSettings, index, values, values_size);
}

ret_t setDesiredSettingRegisterValue(uint8_t rangeId, uint16_t value) {
  if (rangeId > rangeMapCount || value == NULL ) return DATA_FAIL;
  return slaveRanges[rangeId].setRegisterArray(desiredSettings, value);
}

// getter for slaveRange
ret_t getSlaveRangeStart(uint8_t rangeId, uint16_t *value) {
  if (rangeId > rangeMapCount || value == NULL ) return DATA_FAIL;
  value = slaveRanges[rangeId].start;
  return DATA_OK;
}

ret_t getSlaveRangeEnd(uint8_t rangeId, uint16_t *value) {
  if (rangeId > rangeMapCount || value == NULL ) return DATA_FAIL;
  value = slaveRanges[rangeId].end;
  return DATA_OK;
}

ret_t getSlaveRangeCount(uint8_t rangeId, uint16_t *value) {
  if (rangeId > rangeMapCount || value == NULL ) return DATA_FAIL;
  value = slaveRanges[rangeId].count;
  return DATA_OK;
}

// Function 01 set data
ret_t setReadCoilRegistersRes(uint16_t serverAddr, uint8_t rangeId, uint8_t *rxBuf) {
  uint8_t startingAddress = slaveRanges[rangeId].start;
  uint8_t quantityOfOutputs = slaveRanges[rangeId].count;
  for (uint8_t i = 0; i < quantityOfOutputs; i++) {
    uint8_t byteIndex = i / 8;
    uint8_t bitIndex = i % 8;
    bool coilState = (rxBuf[byteIndex] >> bitIndex) & 0x01;
    uint8_t index = startingAddress - slaveRanges[rangeId].start + i;
    if (slaveRanges[rangeId].setCoilArray != NULL) {
      slaveRanges[rangeId].setCoilArray(&recievedSettings, &index, &coilState);
      ESP_LOGI(TAG, "Coil_IR_Trio: %d", coilState);
    } else {
      ESP_LOGW(TAG, "No setter function for coil range %s", slaveRanges[rangeId].name);
      return DATA_FAIL;
    }
  }
  return DATA_OK;
}

// Function 03 set data
ret_t setReadHoldingRegistersRes(uint8_t serverAddr, uint8_t rangeId, uint8_t *rxBuf) {
  uint8_t numberOfRegisters = slaveRanges[rangeId].count;
  for (int i = 0; i < numberOfRegisters; ++i) {
    uint16_t registerValue = readUInt16BE(rxBuf, i * 2);
    if (slaveRanges[rangeId].setRegisterArray != NULL) {
      slaveRanges[rangeId].setRegisterArray(&recievedSettings, registerValue);
      ESP_LOGI(TAG, "Register value: %d", registerValue);
    } else {
      ESP_LOGW(TAG, "No setter function for register range %s", slaveRanges[rangeId].name);
      return DATA_FAIL;
    }
	}
  return DATA_OK;
}

// Help modbus find the correct range using
uint8_t getRangeIdForModbus(uint16_t startingAddress, uint16_t quantityOfOutputs, uint8_t rangeSize) {
  uint8_t indexOfRange = 255;
  for (int i = 0; i < rangeSize; i++) {
    if (slaveRanges[i].start <= startingAddress && slaveRanges[i].start + slaveRanges[i].count > startingAddress) {
      indexOfRange = i;
      break;
    }
  }
  return indexOfRange;
}

// Function prototypes
uint16_t getCoilValue(bool coilState) {
  return coilState ? COIL_ON : COIL_OFF;
}

uint8_t getNumOfSlaves() {
  return num_of_slaves;
}

uint8_t getSlaveAddressBySlaveId(uint8_t slaveId) {
  if (slaveId >= num_of_slaves) return 255;
  return slave_ids[slaveId];
}

uint8_t getSlaveID(uint8_t slaveAddress) {
  for (int i = 0; i < num_of_slaves; i++) {
    if (slave_ids[i] == slaveAddress) return i;
  } return 255;
}

uint8_t getSlaveAddressBySetting(modbus_data_t *setting) {
  if (setting == NULL) return 255;
  return setting->slaveAddress;
}

uint8_t getRangeIdByFunctionType(modbus_function_t setting) {
  for(uint8_t i = 0; i < rangeMapCount; i++) {
    if(slaveRanges[i].event == setting) return i;
  } else return 255;
}

modbus_function_t getFunctionTypeByRangeId(uint8_t rangeId) { 
  return slaveRanges[rangeId].event;
}

void setSlaveAddress(modbus_data_t *setting, uint16_t slaveAddress) {
  if (setting == NULL) return;
  setting->slaveAddress = slaveAddress;
}

bool areSettingsEqual() {
  return memcmp(currentSettings, recievedSettings, sizeof(modbus_data_t)) == 0;
}

bool areArrayEqual(uint8_t rangeId, uint8_t slaveAddress) {
  uint8_t slaveId = getSlaveID(slaveAddress);
  if (rangeId >= allCoilUseStartAddress && rangeId <= allCoilUseEndAddress) { 
    uint16_t count;
    getSlaveRangeCount(rangeId, &count);
    bool currentCoilValue;
    bool recievedCoilValue;
    for (uint8_t i = 0; i < count; i++) {
      slaveRanges[rangeId].getCoilArray(currentSettings[slaveId], i, &currentCoilValue, NULL);
      slaveRanges[rangeId].getCoilArray(recievedSettings, i, &recievedCoilValue, NULL);
      if (currentCoilValue != recievedCoilValue) return false;
    }
    return true;
  }
  if (rangeId >= allRegisterUseStartAddress && rangeId <= allRegisterUseEndAddress) {
    uint16_t currentRegisterValue;
    uint16_t recievedRegisterValue;
    slaveRanges[rangeId].getRegisterArray(currentSettings[slaveId], &currentRegisterValue);
    slaveRanges[rangeId].getRegisterArray(recievedSettings, &recievedRegisterValue);
    return currentRegisterValue == recievedRegisterValue;
  }
  return false;
}

// bool areCoilValueEqual(uint8_t slaveAddress, uint8_t rangeId, uint8_t coilNumber, bool newValue) {
//   uint8_t slaveId = getSlaveID(slaveAddress);
//   bool *value = slaveRanges[rangeId].getCoilArray(&currentSettings[slaveId]);
//   return value[coilNumber--] != newValue;
// }

// bool areRegisterValueEqual(uint8_t slaveAddress, uint8_t rangeId, uint16_t newValue) {
//   uint8_t slaveId = getSlaveID(slaveAddress);
//   uint16_t value = slaveRanges[rangeId].getRegisterArray(&currentSettings[slaveId]);
//   return value != newValue;
// }

void numOfSlavesReq() {
  gpio_config_t ioConf = {
    .intr_type = GPIO_INTR_DISABLE,
    .mode = GPIO_MODE_INPUT,
    .pin_bit_mask = (1ULL << s_dsIoNums[0] | 1ULL << s_dsIoNums[1] | 1ULL << s_dsIoNums[2] | 1ULL << s_dsIoNums[3] | 1ULL << s_dsIoNums[4] | 1ULL << s_dsIoNums[5] | 1ULL << s_dsIoNums[6] | 1ULL << s_dsIoNums[7]),
    .pull_down_en = 1,
    .pull_up_en = 0,
  };
  memset(slave_ids, 0, sizeof(slave_ids));
  num_of_slaves = 0;
  gpio_config(&ioConf);
  uint8_t gpio_num = 0;
  for (int i = 0; i < 8; ++i) {
    gpio_num |= gpio_get_level(s_dsIoNums[i]) << i;
  }
  if (gpio_num > 0) {
    for (int j = 0; j < gpio_num; j++) {
    slave_ids[j] = j;
    }
    num_of_slaves = gpio_num;
  }
  if (gpio_num == 0) {
    for (int i = 0; i < MAX_SLAVE_ID; i++) {
      if (sendReadCoilRegister(i, slaveRanges[0].start, slaveRanges[0].count, slaveRanges[0].name, NULL) == MODBUS_OK) {
        ESP_LOGW(TAG, "Slave %d found", i);
        slave_ids[num_of_slaves] = i;
        num_of_slaves++;
      } else {
        ESP_LOGW(TAG, "Slave %d not found", i);
      }
    }
  }
  ESP_LOGI(TAG, "Number of slaves set to %d", num_of_slaves);
}

void init_slaves(uint8_t num_of_slaves) {
  ESP_LOGI(TAG, "Initializing %d slaves", num_of_slaves);
  // Current settings initialization
  for (int i = 0; i < num_of_slaves; i++) {
    uint8_t slaveAddress = slave_ids[i];
    currentSettings[i].slaveAddress = slaveAddress;
    ESP_LOGI(TAG, "Current setting %d initialized", currentSettings[i].slaveAddress);
  }
  // Slave stage initialization
  for (int i = 0; i < num_of_slaves; i++) {
    uint8_t slaveAddress = slave_ids[i];
    slaveStage[i].slaveAddress = slaveAddress;
    slaveStage[i].Online = true;
    slaveStage[i].Changed = false;
    // memset(slaveStage[i].Modbus_OK, 0, sizeof(slaveStage[i].Modbus_OK));
    // memset(slaveStage[i].Modbus_Failed, 0, sizeof(slaveStage[i].Modbus_Failed));
    ESP_LOGI(TAG, "Slave stage %d initialized", slaveStage[i].slaveAddress);
  }
}

void init_slave_malloc(uint8_t num_of_slaves) {
  // Current settings initialization
  if (currentSettings != NULL) {
    free(currentSettings);
    currentSettings = NULL;
  }
  currentSettings = malloc(sizeof(modbus_data_t) * num_of_slaves);
  if (currentSettings == NULL) {
    ESP_LOGE(TAG, "Failed to allocate memory for slaves");
    return;
  }
  for (int i = 0; i < num_of_slaves; i++) {
    uint8_t slaveAddress = slave_ids[i];
    currentSettings[i].slaveAddress = slaveAddress;
    ESP_LOGI(TAG, "Current setting %d initialized", currentSettings[i].slaveAddress);
  }
  // Slave stage initialization
  if (slaveStage != NULL) {
    free(slaveStage);
    slaveStage = NULL;
  }
  slaveStage = malloc(sizeof(modbus_data_t) * num_of_slaves);
  if (slaveStage == NULL) {
    ESP_LOGE(TAG, "Failed to allocate memory for slaves");
    return;
  }
  for (int i = 0; i < num_of_slaves; i++) {
    uint8_t slaveAddress = slave_ids[i];
    slaveStage[i].slaveAddress = slaveAddress;
    slaveStage[i].Online = true;
    slaveStage[i].Changed = false;
    // memset(slaveStage[i].Modbus_OK, 0, sizeof(slaveStage[i].Modbus_OK));
    // memset(slaveStage[i].Modbus_Failed, 0, sizeof(slaveStage[i].Modbus_Failed));
    ESP_LOGI(TAG, "Slave stage %d initialized", slaveStage[i].slaveAddress);
  }
}

// Modbus data initialization function
void modbus_data_init() {
  ESP_LOGI(TAG, "Initializing modbus data...");
  numOfSlavesReq();
  init_slaves(num_of_slaves);
}