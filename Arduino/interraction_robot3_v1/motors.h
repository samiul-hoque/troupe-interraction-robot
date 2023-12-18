#include <TB6612FNG.h>
const int motorTimer = 50;   //motor motion timer
unsigned long lastMotorMillis = 0;
int motionState = 2;

Tb6612fng motors(33, 12, 4, 0, 32, 25, 27);
float motorPWM = 0.5;
