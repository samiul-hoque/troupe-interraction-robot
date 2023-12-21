#ifndef DEVICE_H
#define DEVICE_H
#include <map>
#include <cstring>
#include <esp_now.h>

const int BAUD_RATE = 115200;
const int MAC_ADDRESS_STRING_LENGTH = 18;
const int MAX_LENGTH = 150;


const char* esp_devices[2] = {
  "08:D1:F9:CD:1B:E0", // usb0
  "24:DC:C3:CF:15:F0" // usb1
};

uint8_t broadcast_address[] = {0x24, 0xDC, 0xC3, 0xCF, 0x15, 0xF0};

esp_now_peer_info_t get_peer(){
  esp_now_peer_info_t peer;
  peer.channel = 0;
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_STA;
  memcpy(peer.peer_addr, broadcast_address, ESP_NOW_ETH_ALEN);
  return peer;
}

struct data_transfer_object {
  char message[MAX_LENGTH];
};


bool is_current_board_master(const char *mac_address, const char *master_board_mac_address = esp_devices[0]){
  if (!strcmp(mac_address, master_board_mac_address)){
    return true;
  }

  return false;
}

#endif