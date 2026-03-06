#include <string.h>

#include "esp_log.h"
#include "esp_http_server.h"
#include "cJSON.h"

#include "mailbox/server_event_handler.h"
#include "event/modbus_data_event.h"

static const char *TAG = "JSON_HANDLER";

// Helper: Serialize modbus_data_all_data to JSON
static cJSON *modbus_data_all_data_to_json(const modbus_data_all_data *settings) {
  cJSON *root = cJSON_CreateObject();
  if (!settings) return root;
  // Each pointer is assumed to be valid or NULL
  cJSON_AddBoolToObject(root, "Coil_IR_Trio", settings->Coil_IR_Trio ? *settings->Coil_IR_Trio : false);
  cJSON_AddBoolToObject(root, "Coil_IR_Mono", settings->Coil_IR_Mono ? *settings->Coil_IR_Mono : false);
  cJSON_AddBoolToObject(root, "Coil_Lock", settings->Coil_Lock ? *settings->Coil_Lock : false);
  cJSON_AddBoolToObject(root, "Coil_LED", settings->Coil_LED ? *settings->Coil_LED : false);
  cJSON_AddBoolToObject(root, "Coil_IR_Trio_State", settings->Coil_IR_Trio_State ? *settings->Coil_IR_Trio_State : false);
  cJSON_AddBoolToObject(root, "Coil_IR_Mono_State", settings->Coil_IR_Mono_State ? *settings->Coil_IR_Mono_State : false);
  cJSON_AddBoolToObject(root, "Coil_IR_Latched_State", settings->Coil_IR_Latched_State ? *settings->Coil_IR_Latched_State : false);
  cJSON_AddBoolToObject(root, "Coil_Blink_LEDS", settings->Coil_Blink_LEDS ? *settings->Coil_Blink_LEDS : false);
  cJSON_AddBoolToObject(root, "Coil_Lock_State", settings->Coil_Lock_State ? *settings->Coil_Lock_State : false);
  cJSON_AddBoolToObject(root, "Coil_OTA_Update_Start_Command", settings->Coil_OTA_Update_Start_Command ? *settings->Coil_OTA_Update_Start_Command : false);
  cJSON_AddBoolToObject(root, "Coil_OTA_Update_Finish_Command", settings->Coil_OTA_Update_Finish_Command ? *settings->Coil_OTA_Update_Finish_Command : false);
  cJSON_AddBoolToObject(root, "Coil_OTA_Update_Abort_Command", settings->Coil_OTA_Update_Abort_Command ? *settings->Coil_OTA_Update_Abort_Command : false);

  cJSON_AddNumberToObject(root, "Register_Lock_Duration", settings->Register_Lock_Duration ? *settings->Register_Lock_Duration : 0);
  cJSON_AddNumberToObject(root, "Register_Current_Version", settings->Register_Current_Version ? *settings->Register_Current_Version : 0);
  cJSON_AddNumberToObject(root, "Register_OTA_Update_Data", settings->Register_OTA_Update_Data ? *settings->Register_OTA_Update_Data : 0);
  return root;
}

// Helper: Parse JSON to modbus_data_all_data
static modbus_data_all_data *json_to_modbus_data_all_data(const cJSON *root) {
  if (!root) return NULL;
  modbus_data_all_data *settings = calloc(1, sizeof(modbus_data_all_data));
  if (!settings) return NULL;

  // Allocate and assign each pointer
#define BOOL_FIELD(name) \
  do { \
    cJSON *item = cJSON_GetObjectItem(root, #name); \
    if (item) { \
      settings->name = malloc(sizeof(bool)); \
      *settings->name = cJSON_IsTrue(item); \
    } \
  } while(0)

#define UINT16_FIELD(name) \
  do { \
    cJSON *item = cJSON_GetObjectItem(root, #name); \
    if (item) { \
      settings->name = malloc(sizeof(uint16_t)); \
      *settings->name = (uint16_t)item->valueint; \
    } \
  } while(0)

  BOOL_FIELD(Coil_IR_Trio);
  BOOL_FIELD(Coil_IR_Mono);
  BOOL_FIELD(Coil_Lock);
  BOOL_FIELD(Coil_LED);
  BOOL_FIELD(Coil_IR_Trio_State);
  BOOL_FIELD(Coil_IR_Mono_State);
  BOOL_FIELD(Coil_IR_Latched_State);
  BOOL_FIELD(Coil_Blink_LEDS);
  BOOL_FIELD(Coil_Lock_State);
  BOOL_FIELD(Coil_OTA_Update_Start_Command);
  BOOL_FIELD(Coil_OTA_Update_Finish_Command);
  BOOL_FIELD(Coil_OTA_Update_Abort_Command);

  UINT16_FIELD(Register_Lock_Duration);
  UINT16_FIELD(Register_Current_Version);
  UINT16_FIELD(Register_OTA_Update_Data);

#undef BOOL_FIELD
#undef UINT16_FIELD

  return settings;
}

// Helper: Serialize modbus_data_param_t to JSON
static cJSON *modbus_data_param_to_json(const modbus_data_param_t *param) {
  cJSON *root = cJSON_CreateObject();
  if (!param) return root;
  cJSON_AddNumberToObject(root, "slaveAddress", param->slaveAddress);
  cJSON_AddNumberToObject(root, "function", param->function);

  // coilValue
  if (param->modbus_data.coilValue) {
    cJSON *coil = cJSON_CreateObject();
    cJSON_AddNumberToObject(coil, "coilAddress", param->modbus_data.coilValue->coilAddress);
    cJSON_AddBoolToObject(coil, "value", param->modbus_data.coilValue->value);
    cJSON_AddItemToObject(root, "coilValue", coil);
  }
  // registerValue
  if (param->modbus_data.registerValue) {
    cJSON *reg = cJSON_CreateObject();
    cJSON_AddNumberToObject(reg, "value", param->modbus_data.registerValue->value);
    cJSON_AddItemToObject(root, "registerValue", reg);
  }
  // settings
  if (param->modbus_data.settings) {
    cJSON *settings = modbus_data_all_data_to_json(param->modbus_data.settings);
    cJSON_AddItemToObject(root, "settings", settings);
  }

  return root;
}

// Helper: Parse JSON to modbus_data_param_t (now supports coilValue, registerValue, settings)
static modbus_data_param_t *json_to_modbus_data_param(const cJSON *root) {
  if (!root) return NULL;
  modbus_data_param_t *param = calloc(1, sizeof(modbus_data_param_t));
  if (!param) return NULL;
  cJSON *slaveAddress = cJSON_GetObjectItem(root, "slaveAddress");
  cJSON *function = cJSON_GetObjectItem(root, "function");
  if (slaveAddress) param->slaveAddress = slaveAddress->valueint;
  if (function) param->function = function->valueint;

  cJSON *coil = cJSON_GetObjectItem(root, "coilValue");
  if (coil) {
    param->modbus_data.coilValue = calloc(1, sizeof(modbus_data_coil_data));
    cJSON *coilAddress = cJSON_GetObjectItem(coil, "coilAddress");
    cJSON *value = cJSON_GetObjectItem(coil, "value");
    if (coilAddress) param->modbus_data.coilValue->coilAddress = coilAddress->valueint;
    if (value) param->modbus_data.coilValue->value = cJSON_IsTrue(value);
  }

  cJSON *reg = cJSON_GetObjectItem(root, "registerValue");
  if (reg) {
    param->modbus_data.registerValue = calloc(1, sizeof(modbus_data_register_data));
    cJSON *value = cJSON_GetObjectItem(reg, "value");
    if (value) param->modbus_data.registerValue->value = value->valueint;
  }

  cJSON *settings = cJSON_GetObjectItem(root, "settings");
  if (settings) {
    param->modbus_data.settings = json_to_modbus_data_all_data(settings);
  }

  return param;
}

// HTTP POST handler: receive JSON and trigger event
esp_err_t http_post_json_handler(httpd_req_t *req) {
  char buf[1024];
  int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
  if (ret <= 0) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No data");
    return ESP_FAIL;
  }
  buf[ret] = 0;
  cJSON *root = cJSON_Parse(buf);
  if (!root) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
    return ESP_FAIL;
  }
  modbus_data_param_t *param = json_to_modbus_data_param(root);
  cJSON_Delete(root);
  if (!param) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Parse error");
    return ESP_FAIL;
  }
  // Create and send event
  server_event_handler_t event;
  event.event = EV_MODBUS_DIFF_RES;
  event.params.settings = param;
  sendServerEvent(event);
  httpd_resp_sendstr(req, "OK");
  return ESP_OK;
}

// HTTP GET handler: send JSON of current state
esp_err_t http_get_json_handler(httpd_req_t *req) {
  // Example: get current modbus_data_param_t (replace with your getter)
  modbus_data_param_t *param = get_current_modbus_data_param();
  cJSON *root = modbus_data_param_to_json(param);
  char *json_str = cJSON_PrintUnformatted(root);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_sendstr(req, json_str);
  cJSON_Delete(root);
  free(json_str);
  return ESP_OK;
}

// Register handlers with HTTP server
void register_json_handlers(httpd_handle_t server) {
  httpd_uri_t post_uri = {
    .uri = "/api/modbus",
    .method = HTTP_POST,
    .handler = http_post_json_handler,
    .user_ctx = NULL
  };
  httpd_register_uri_handler(server, &post_uri);

  httpd_uri_t get_uri = {
    .uri = "/api/modbus",
    .method = HTTP_GET,
    .handler = http_get_json_handler,
    .user_ctx = NULL
  };
  httpd_register_uri_handler(server, &get_uri);
}