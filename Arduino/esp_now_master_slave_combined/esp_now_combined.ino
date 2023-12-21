#include "esp_chip_info.h"
#include "esp_mac.h"
#include <WiFi.h>
#include <esp_wifi.h>
#include "device.h"
#define STARTUP_DELAY 2000


char current_board_mac_address[MAC_ADDRESS_STRING_LENGTH];

bool is_this_board_master = false;


void on_data_sent(const uint8_t *mac_address, esp_now_send_status_t status) {
  Serial.println(status);
}

void on_data_receive(const uint8_t *mac_address, const uint8_t *incoming_data, int length) {
  data_transfer_object dto;
  memcpy(&dto, incoming_data, length);
  Serial.println(dto.message);
  Serial.printf("Length: %d\n", length);
}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUD_RATE);
  WiFi.mode(WIFI_STA);

  delay(STARTUP_DELAY);

  strcpy(current_board_mac_address, WiFi.macAddress().c_str());
  is_this_board_master = is_current_board_master(current_board_mac_address);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }


  if (is_this_board_master) {
    esp_now_register_send_cb(on_data_sent);

    esp_now_peer_info_t peer = get_peer();
    esp_err_t peer_result = esp_now_add_peer(&peer);

    Serial.println("Connecting peer ... ");
    Serial.print(peer_result, HEX);
    Serial.println();
  }
   
    esp_now_register_recv_cb(on_data_receive);
  
}

void loop() {
  // put your main code here, to run repeatedly:

  if (is_this_board_master) {
    
    data_transfer_object dto;
    strcpy(dto.message, "TEST WORDS");
    esp_err_t result = esp_now_send(broadcast_address, (uint8_t *) &dto, sizeof(dto));

    if (result == ESP_OK) {
      Serial.println("Sent with success");
    } else {
      Serial.print("Failed sending message: ");
      Serial.print(result, HEX);
      Serial.println();
    }

    delay(2000);
  } else {
    Serial.println("Waiting to receive something .... ");
    delay(1000);
  }
}
