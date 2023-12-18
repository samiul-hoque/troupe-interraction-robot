#include <esp_now.h>
#include <WiFi.h>

uint8_t robot1[] = { 0xAC, 0x67, 0xB2, 0x3C, 0x61, 0xE8 };  //02OP6R7Z
uint8_t robot2[] = { 0x84, 0xCC, 0xA8, 0x65, 0x55, 0x44 };  //02MOE3SY
uint8_t robot3[] = { 0xAC, 0x67, 0xB2, 0x3C, 0x60, 0x70 };  //02JJVOOJ

int id = 1;  //robotID

typedef struct struct_message {
  int id;
  int icounter;
} struct_message;

struct_message recData;

struct_message robot1data;
struct_message robot2data;
struct_message robot3data;


struct_message boardsStruct[3] = { robot1data, robot2data, robot3data };

esp_now_peer_info_t peerInfo;
