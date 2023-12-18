// ROBOT 1 //
//com16
#include <FastLED.h>
#define NUM_LEDS 24
#define DATA_PIN 26
CRGB leds[NUM_LEDS];


#include "bitarrays.h"
#include "neighbors.h"
#include "motors.h"

//pin definitions
#define BUTTON_PIN     19
#define DEBOUNCE_TIME  50

//tuning variables
int emotionalState = 0;
int interractionThreshhold = 5; //threshhold to reach happy
int interractionCounter = 0;
const int decayTimer = 1000;   //rate at which interractiond decays

unsigned long currentMillis = 0;
unsigned long lastDecayMillis = 0;


//int icounterarray[][] = {{0,0,0},{0,0,0};


//debounce
int lastSteadyState = LOW;
int lastFlickerableState = LOW;
int currentState;
bool buttonState = 0;
unsigned long lastDebounceTime = 0;


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  //Serial.print("Packet received from: ");
  // snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
  //          mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  //Serial.println(macStr);
  memcpy(&recData, incomingData, sizeof(recData));
  //Serial.printf("Board ID %u: %u bytes\n", recData.id, len);
  // Update the structures with the new incoming data
  boardsStruct[recData.id - 1].id = recData.id;
  boardsStruct[recData.id - 1].icounter = recData.icounter;
  //Serial.printf("id value: %d \n", boardsStruct[recData.id - 1].id);
  //Serial.printf("icounter value: %d \n", boardsStruct[recData.id - 1].icounter);
  //Serial.println();
}

//void populateData(){
//  icounterArray
//  }
//
//void compare(){
//  boardSruct
//  }

void addPeers() {
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  // memcpy(peerInfo.peer_addr, robot1, 6);
  // if (esp_now_add_peer(&peerInfo) != ESP_OK) {
  //   Serial.println("Failed to add peer");
  //   return;
  // }
  memcpy(peerInfo.peer_addr, robot2, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  memcpy(peerInfo.peer_addr, robot3, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

}


void debugNetwork() {
  Serial.print("Robot 1-> ");
  Serial.print("id: ");
  Serial.print(robot1data.id);
  Serial.print(",");
  Serial.print("icounter: ");
  Serial.print(robot1data.icounter);
  Serial.print("  ");
  Serial.print("Robot 2-> ");
  Serial.print("id: ");
  Serial.print(boardsStruct[1].id);
  Serial.print(",");
  Serial.print("icounter: ");
  Serial.print(boardsStruct[1].icounter);
  Serial.print("  ");
  Serial.print("Robot 3-> ");
  Serial.print("id: ");
  Serial.print(boardsStruct[2].id);
  Serial.print(",");
  Serial.print("icounter: ");
  Serial.println(boardsStruct[2].icounter);
}

void publishData() {
  // esp_err_t result1 = esp_now_send(robot1, (uint8_t *)&robot1data, sizeof(robot1data));
  // if (result1 == ESP_OK) {
  //   Serial.println("Sent with success");
  // } else {
  //   Serial.println("Error sending the data");
  // }
  esp_err_t result2 = esp_now_send(robot2, (uint8_t *)&robot1data, sizeof(robot1data));
  if (result2 == ESP_OK) {
    //Serial.println("Sent with success");
  } else {
    //Serial.println("Error sending the data");
  }
  esp_err_t result3 = esp_now_send(robot3, (uint8_t *)&robot1data, sizeof(robot1data));
  if (result3 == ESP_OK) {
    //Serial.println("Sent with success");
  } else {
    //Serial.println("Error sending the data");
  }
}

void showAnimation(const byte PROGMEM frame[][512], int frame_count) {
  for (int f = 0; f < frame_count; f++) {
    display.clearDisplay();
    display.drawBitmap(32, 0, frame[f], FRAME_WIDTH, FRAME_HEIGHT, 1);
    display.display();
  }
  emotionalState = 0;
}

void readButton() {
  currentState = !digitalRead(BUTTON_PIN);
  if (currentState != lastFlickerableState) {
    lastDebounceTime = millis();
    lastFlickerableState = currentState;
  }
  if ((millis() - lastDebounceTime) > DEBOUNCE_TIME) {
    if (lastSteadyState == HIGH && currentState == LOW)
      buttonState = true;
    //Serial.println("The button is pressed");
    else if (lastSteadyState == LOW && currentState == HIGH)
      buttonState = false;
    //Serial.println("The button is released");
    lastSteadyState = currentState;
  }
}

void decayInterraction() {
  if (currentMillis - lastDecayMillis >= decayTimer) {
    if (interractionCounter > 0) {
      interractionCounter--;
      lastDecayMillis = currentMillis;
    }
  }
}

void forward() {
  //Serial.println("forward");
  //motors.drive(-motorPWM, motorDelayRate);
  motors.drive(-motorPWM);
}
void reverse() {
  //Serial.println("reverse");
  //motors.drive(motorPWM, motorDelayRate);
  motors.drive(motorPWM);
}
void left() {
  //Serial.println("left");
  //motors.drive(motorPWM, -motorPWM, turnDuration);
  motors.drive(motorPWM, -motorPWM);
}
void right() {
  //Serial.println("right");
  //motors.drive(-motorPWM, motorPWM, turnDuration);
  motors.drive(-motorPWM, motorPWM);
}
void stopmotor(){
    motors.brake();
  }

void runMotion(int m) {
  if (currentMillis - lastMotorMillis > motorTimer)
  {
    if (m == 0) {
    right();
  } else if (m == 1) {
    left();
  } else if (m == 2) {
    stopmotor();
  } else if (m == 3) {
    forward();
  }else if (m == 4) {
    reverse();
  }
    lastMotorMillis = currentMillis;
   }
}

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT);
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_I2C_ADDR);
  FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(3);
  motors.begin();
  robot1data.id = id;
  robot1data.icounter = -1;

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  addPeers();
}

void loop() {
  currentMillis = millis();
  readButton();

  //interraction state + button pressed
  if (buttonState == true) {
    emotionalState = 1;
    interractionCounter++;
  }
  //happy state
  if (interractionCounter > interractionThreshhold) {
    emotionalState = 2;
    robot1data.icounter = 2;
  } else {
    robot1data.icounter = -1;
  }

  if (/*boardsStruct[0].icounter == 2 ||*/ boardsStruct[1].icounter == 2 || boardsStruct[2].icounter == 2)
  {
    emotionalState = 3;
  }


  ///animate
  if (emotionalState == 0) {
    showAnimation(idle, frame_counts[4]);
    runMotion(random(0,4));
    fill_solid(leds, NUM_LEDS, CRGB::White);
  }
  if (emotionalState == 1) {
    showAnimation(love, frame_counts[1]);
    runMotion(random(2,4));
    fill_solid(leds, NUM_LEDS, CRGB::Cyan);
  }
  if (emotionalState == 2) {
    showAnimation(happy, frame_counts[0]);
    runMotion(random(0,2));
    fill_solid(leds, NUM_LEDS, CRGB::Blue);
  }
  if (emotionalState == 3) {
    showAnimation(cry, frame_counts[3]);
    runMotion(2);
    fill_solid(leds, NUM_LEDS, CRGB::Red);
  }
  publishData();
  FastLED.show();
  decayInterraction();

  //debugNetwork();
  Serial.println(interractionCounter);
}
