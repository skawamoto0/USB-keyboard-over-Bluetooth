/*
  Copyright (C) 2024 Suguru Kawamoto
  This software is released under the MIT License.
*/

/*
  This software includes the work that is distributed in the Apache License 2.0
  https://components.espressif.com/components/espressif/usb_host_hid/
  Copyright 2022-2023 Espressif Systems (Shanghai) CO LTD

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

  http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEHIDDevice.h>
#include <HIDTypes.h>
#include <HIDKeyboardTypes.h>
#include <esp_pm.h>
#include <nvs_flash.h>
#include <esp_mac.h>
#include "usb/usb_host.h"
#include "usb/hid_host.h"
#include "usb/hid_usage_keyboard.h"

class BLEServerCallbacks1 : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t* param);
  void onDisconnect(BLEServer* pServer, esp_ble_gatts_cb_param_t* param);
};

static const uint8_t REPORT_MAP[] = {
  USAGE_PAGE(1),
  0x01,
  USAGE(1),
  0x06,
  COLLECTION(1),
  0x01,
  REPORT_ID(1),
  0x01,
  USAGE_PAGE(1),
  0x07,
  USAGE_MINIMUM(1),
  0xe0,
  USAGE_MAXIMUM(1),
  0xe7,
  LOGICAL_MINIMUM(1),
  0x00,
  LOGICAL_MAXIMUM(1),
  0x01,
  REPORT_COUNT(1),
  0x08,
  REPORT_SIZE(1),
  0x01,
  HIDINPUT(1),
  0x02,
  REPORT_COUNT(1),
  0x01,
  REPORT_SIZE(1),
  0x08,
  HIDINPUT(1),
  0x01,
  REPORT_COUNT(1),
  0x06,
  REPORT_SIZE(1),
  0x08,
  LOGICAL_MINIMUM(1),
  0x00,
  LOGICAL_MAXIMUM(1),
  0xdf,
  USAGE_MINIMUM(1),
  0x00,
  USAGE_MAXIMUM(1),
  0xdf,
  HIDINPUT(1),
  0x00,
  REPORT_COUNT(1),
  0x05,
  REPORT_SIZE(1),
  0x01,
  USAGE_PAGE(1),
  0x08,
  USAGE_MINIMUM(1),
  0x01,
  USAGE_MAXIMUM(1),
  0x05,
  LOGICAL_MINIMUM(1),
  0x00,
  LOGICAL_MAXIMUM(1),
  0x01,
  HIDOUTPUT(1),
  0x02,
  REPORT_COUNT(1),
  0x01,
  REPORT_SIZE(1),
  0x03,
  HIDOUTPUT(1),
  0x01,
  END_COLLECTION(0)
};

usb_host_client_handle_t client;
usb_device_handle_t device;

BLEServer* server;
BLEServerCallbacks1 callback;
BLEHIDDevice* hid;
BLECharacteristic* input;
BLECharacteristic* output;
BLESecurity* security;
BLEAdvertising* advertising;

bool sleeping;
bool awakening;
int64_t timer;
uint8_t current;
uint8_t selector;
bool reconnect;
bool connected;
esp_bd_addr_t remote;
hid_keyboard_input_report_boot_t state;

void hid_host_interface_callback(hid_host_device_handle_t hid_device_handle, const hid_host_interface_event_t event, void* arg) {
  uint8_t data[64];
  size_t length;
  switch(event) {
  case HID_HOST_INTERFACE_EVENT_INPUT_REPORT:
    hid_host_device_get_raw_input_report_data(hid_device_handle, data, sizeof(data), &length);
    memcpy(&state, &data, sizeof(state));
    awakening = (state.modifier.val != 0 || state.key[0] != 0);
    if(state.modifier.val == (HID_LEFT_CONTROL | HID_LEFT_SHIFT | HID_LEFT_ALT)) {
      switch(state.key[0]) {
      case HID_KEY_1:
        selector = 0;
        break;
      case HID_KEY_2:
        selector = 1;
        break;
      case HID_KEY_3:
        selector = 2;
        break;
      case HID_KEY_4:
        selector = 3;
        break;
      case HID_KEY_5:
        selector = 4;
        break;
      case HID_KEY_6:
        selector = 5;
        break;
      case HID_KEY_7:
        selector = 6;
        break;
      case HID_KEY_8:
        selector = 7;
        break;
      case HID_KEY_0:
        esp_ble_remove_bond_device(remote);
        reconnect = true;
        break;
      }
      if(selector != current) {
        reconnect = true;
      }
    }
    if(reconnect) {
      memset(&state, 0, sizeof(state));
    }
    if(connected) {
      input->setValue((uint8_t*)&state, sizeof(state));
      input->notify();
    }
    break;
  case HID_HOST_INTERFACE_EVENT_DISCONNECTED:
    hid_host_device_close(hid_device_handle);
    memset(&state, 0, sizeof(state));
    awakening = false;
    if(connected) {
      input->setValue((uint8_t*)&state, sizeof(state));
      input->notify();
    }
    break;
  }
}

void hid_host_device_callback(hid_host_device_handle_t hid_device_handle, const hid_host_driver_event_t event, void* arg) {
  hid_host_dev_params_t params;
  hid_host_device_config_t config;
  switch(event) {
  case HID_HOST_DRIVER_EVENT_CONNECTED:
    hid_host_device_get_params(hid_device_handle, &params);
    if(params.sub_class == HID_SUBCLASS_BOOT_INTERFACE) {
      if(params.proto == HID_PROTOCOL_KEYBOARD) {
        config.callback = hid_host_interface_callback;
        config.callback_arg = NULL;
        hid_host_device_open(hid_device_handle, &config);
        hid_class_request_set_protocol(hid_device_handle, HID_REPORT_PROTOCOL_BOOT);
        hid_class_request_set_idle(hid_device_handle, 0, 0);
        hid_host_device_start(hid_device_handle);
      }
    }
    break;
  }
}

void taskUsb(void* pvParameters) {
  usb_host_config_t config;
  hid_host_driver_config_t host;
  uint32_t flags;
  while(true) {
    config.skip_phy_setup = false;
    config.intr_flags = ESP_INTR_FLAG_LEVEL1;
    config.enum_filter_cb = NULL;
    usb_host_install(&config);
    host.create_background_task = true;
    host.task_priority = 5;
    host.stack_size = CONFIG_ESP_MAIN_TASK_STACK_SIZE;
    host.core_id = xPortGetCoreID();
    host.callback = hid_host_device_callback;
    host.callback_arg = NULL;
    hid_host_install(&host);
    while(true) {
      usb_host_lib_handle_events(1000, &flags);
      if(flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
        break;
      }
    }
    hid_host_uninstall();
    usb_host_uninstall();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  vTaskDelete(NULL);
}

void BLEServerCallbacks1::onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t* param) {
  ((BLE2902*)input->getDescriptorByUUID(BLEUUID((uint16_t)0x2902)))->setNotifications(true);
  memcpy(&remote, &param->connect.remote_bda, sizeof(esp_bd_addr_t));
  connected = true;
}

void BLEServerCallbacks1::onDisconnect(BLEServer* pServer, esp_ble_gatts_cb_param_t* param) {
  connected = false;
  ((BLE2902*)input->getDescriptorByUUID(BLEUUID((uint16_t)0x2902)))->setNotifications(false);
}

void taskBluetooth(void* pvParameters) {
  nvs_handle_t nvs;
  uint8_t mac[6];
  sntp_set_system_time(0x40000000, 0);
  nvs_flash_init();
  while(true) {
    nvs_open("storage", NVS_READWRITE, &nvs);
    current = 0;
    nvs_get_u8(nvs, "select", &current);
    selector = current;
    esp_efuse_mac_get_default(mac);
    mac[5] += current * 4;
    esp_base_mac_addr_set(mac);
    BLEDevice::init("ESP32-S3");
    server = BLEDevice::createServer();
    server->setCallbacks(&callback);
    hid = new BLEHIDDevice(server);
    input = hid->inputReport(1);
    output = hid->outputReport(1);
    hid->manufacturer()->setValue("Espressif");
    hid->pnp(0x02, 0xe502, 0xa111, 0x0210);
    hid->hidInfo(0x00, 0x02);
    security = new BLESecurity();
    security->setAuthenticationMode(ESP_LE_AUTH_BOND);
    hid->reportMap((uint8_t*)&REPORT_MAP, sizeof(REPORT_MAP));
    hid->startServices();
    hid->setBatteryLevel(100);
    advertising = server->getAdvertising();
    advertising->setAppearance(HID_KEYBOARD);
    advertising->addServiceUUID(hid->hidService()->getUUID());
    advertising->start();
    sleeping = false;
    awakening = false;
    timer = esp_timer_get_time();
    while(true) {
      if(reconnect || sleeping) {
        break;
      }
      if(awakening) {
        timer = esp_timer_get_time();
      } else {
        if(esp_timer_get_time() - timer > 5 * 60 * 1000000ll) {
          sleeping = true;
        }
      }
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    if(selector != current) {
      nvs_set_u8(nvs, "select", selector);
      nvs_commit(nvs);
    }
    nvs_close(nvs);
    advertising->stop();
    delete security;
    delete hid;
    BLEDevice::deinit(false);
    while(true) {
      if(reconnect || awakening) {
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP.restart();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  vTaskDelete(NULL);
}

void setup() {
  // put your setup code here, to run once:
  esp_pm_config_esp32_t config;
  esp_pm_get_configuration(&config);
  config.min_freq_mhz = 10;
  config.light_sleep_enable = true;
  esp_pm_configure(&config);
  vTaskDelay(pdMS_TO_TICKS(100));
  xTaskCreatePinnedToCore(taskUsb, "Usb", CONFIG_ESP_MAIN_TASK_STACK_SIZE, NULL, 0, NULL, xPortGetCoreID());
  xTaskCreatePinnedToCore(taskBluetooth, "Bluetooth", CONFIG_ESP_MAIN_TASK_STACK_SIZE, NULL, 0, NULL, xPortGetCoreID());
}

void loop() {
  // put your main code here, to run repeatedly:
  vTaskDelay(pdMS_TO_TICKS(60000));
}
