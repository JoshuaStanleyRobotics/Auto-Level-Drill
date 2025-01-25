/*
   Description: This program is used to interpret accelerometer data from a MPU6050 and use an addressable
        LED ring to indicate which direction to tilt a drill so that it is pointed straight up and down

        Wiring: The required components are a MPU6050, an addressable ring LED, and a Seeed XIAO microcontroller.
        The GND, VCC, SCL, and SDA pins of the MPU6050 are wired to GND, 3.3V, A5, and A4 respectively,
        The GND, PWR, and IN pins of the ring LED are wired to GND, 3.3v, and D3 respectively.
*/

#include "Wire.h"
#include "Adafruit_NeoPixel.h"
Adafruit_NeoPixel led(12, 10, NEO_GRB + NEO_KHZ800);

//Variables
int16_t input[7];                                                   //X acc, Y acc, Z acc, temp, X gyro, Y gyro, Z gyro
double pitch, yaw;
int pitchError, yawError, angle, magnitude;                         //Variables to quantify how far the sensor is from level
unsigned long millisPrev = 0;

//Constants
const int pitchOffset = -2;                                         //Adjust pitch and yaw offset to calibrate sensor
const int yawOffset = -1;
const int closeEnough = 2;                                          //Dead-zone value dictating precision required to light up the ring green
const int brightness = 25;                                          //LED ring brightness value

void setup() {
  Serial.begin(9600);                                               //Begin Serial communication
  Serial.println("Serial Communication Initialized");

  Wire.begin();                                                     //Begin I2C communication with MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.println("IMU Initialized");

  led.begin();                                                      //Initialize LED ring control with NeoPixel library
  Serial.println("LED Ring Initialized");

  for (int i = 8; i < 12; i++) {                                    //Start-up Animation
    led.clear();
    led.setPixelColor(i, led.Color(0, brightness, 0));
    led.show();
    delay(150);
  }
  for (int i = 0; i < 8; i++) {
    led.clear();
    led.setPixelColor(i, led.Color(0, brightness, 0));
    led.show();
    delay(150);
  }
  led.clear();
  led.show();
  delay(250);

  for (int i = 0; i < 12; i++) {
    led.setPixelColor(i, led.Color(0, brightness, 0));
  }
  led.show();
  delay(500);

  led.clear();
  led.show();
  delay(250);
  Serial.println("Running");
}

void loop() {
  Wire.beginTransmission(0x68);                                                                             //Request and receive data from IMU
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);

  for (int i = 0; i < 7; i++) {
    input[i] = Wire.read() << 8 | Wire.read();                                                              //Assign accelerometer and gyroscope data to input array
  }

  pitch += (double(-input[5]) / 131) * (millis() - millisPrev) / 1000;                                      //Calculate new angles based on previous angle, angular speed, and elapsed time
  yaw += (double(-input[6]) / 131) * (millis() - millisPrev) / 1000;

  millisPrev = millis();

  pitch = 0.95 * pitch + 0.05 * ((double(atan2(input[0], input[2])) * 180 / PI) + pitchOffset);             //Complimentary filter combining accelerometer and gyroscope data
  yaw = 0.95 * yaw + 0.05 * ((double(-atan2(input[0], input[1])) * 180 / PI) + yawOffset);

  Serial.println("Pitch = " + String(pitch) + "   Yaw = " + String(yaw));
  led.clear();


  if (pitch > -45) {                                                                                    //If the drill is roughly horizontal
    if (pitch > closeEnough) {                                                                              //If the front of the drill is tilted up, light up the top LED with color scaled to error
      pitchError = constrain(pitch * 5, 0, 100);
      led.setPixelColor(8, led.Color((brightness * pitchError / 100), (brightness - (brightness * pitchError / 100)), 0));
    }
    else if (pitch < (-closeEnough)) {                                                                      //If the front of the drill is tilted down, light up the bottom LED with color scaled to error
      pitchError = constrain(-pitch * 5, 0, 100);
      led.setPixelColor(2, led.Color((brightness * pitchError / 100), (brightness - (brightness * pitchError / 100)), 0));
    }
    else {                                                                                                  //Otherwise, light up entire LED ring green
      for (int i = 0; i < 12; i++) {
        led.setPixelColor(i, led.Color(0, brightness, 0));
      }
    }
  }
  else {                                                                                                //If the drill is roughly vertical
    if (abs(yaw - 90) > closeEnough) yawError = constrain(5 * (yaw - 90), -100, 100);                       //Calcualte the error in the yaw direction
    else yawError = 0;

    if (abs(pitch + 90) > closeEnough) pitchError = constrain(5 * (pitch + 90), -100, 100);                 //Calculate the error in the pitch direction
    else pitchError = 0;

    magnitude = constrain(sqrt(sq(yawError) + sq(pitchError)), 0, 100);                                     //Calculate the magnitude of the error
    angle = map((atan2(pitchError, -yawError) * 100), -314, 314, 0, 11);                                    //Calculate the angle of the error

    if (magnitude == 0) {                                                                                   //If the magnitude is small enough, light up all LEDs green
      for (int i = 0; i < 12; i++) {
        led.setPixelColor(i, led.Color(0, brightness, 0));
      }
    }
    else led.setPixelColor(angle, led.Color(((brightness * magnitude) / 100), (brightness - (brightness * magnitude) / 100), 0));   //Otherwise light up the LED for the corresponding angle with color scaled to the magnitude of the error
  }
  led.show();
  delay(5);
}
