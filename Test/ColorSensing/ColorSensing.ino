#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include "ColorConverterLib.h"

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);

void setup()
{
  Serial.begin(9600);

  if (!tcs.begin())
  {
    Serial.println("Error al iniciar TCS34725");
    while (1) delay(1000);
  }
}

void loop()
{
  uint16_t clear, red, green, blue;

  tcs.setInterrupt(false);
  delay(60); // Takes 50ms to capture the color
  tcs.getRawData(&red, &green, &blue, &clear);
  tcs.setInterrupt(true);

  // Make rgb measurement relative
  uint32_t sum = clear;
  float r, g, b;
  r = red; r /= sum;
  g = green; g /= sum;
  b = blue; b /= sum;

  // Scale rgb to bytes
  r *= 256; g *= 256; b *= 256;

  // Convert to hue, saturation, value
  double hue, saturation, value;
  ColorConverter::RgbToHsv(static_cast<uint8_t>(r), static_cast<uint8_t>(g), static_cast<uint8_t>(b), hue, saturation, value);

  // Show color name
  printColorName(hue * 360);

  delay(1000);
}

void printColorName(double hue)
{
    if (hue > 30 && hue < 50) {
        Serial.println("Yellow");
    }
    else if (hue > 55 && hue < 65) {
        Serial.println("Black");
    }
    else if (hue > 222 && hue < 265) {
        Serial.println("Purple");
    }
    else if (hue > 195 && hue < 222) {
        Serial.println("Blue");
    }
    else if (hue > 300 && hue < 360) {
        Serial.println("Pink");
    }
    else if (hue > 0 && hue < 25) {
        Serial.println("Red");
    }
    else if (hue > 85 && hue < 110) {
        Serial.println("Green");
    }
    else {
        Serial.println("None of the above");
    }
    Serial.println(hue);
}

void showColor(){
  if(Serial.available() > 0){
    if(Serial.read("Yellow")){
      analogWrite(R,0);
      analogWrite(G,0);
      analogWrite(B,255);
    }else if(Serial.read("Black")){
      analogWrite(R,0);
      analogWrite(G,0);
      analogWrite(B,0);
    }else if(Serial.read("Purple")){
      analogWrite(R,184);
      analogWrite(G,37);
      analogWrite(B,174);
    }else if(Serial.read("Blue")){
      analogWrite(R,0);
      analogWrite(G,0);
      analogWrite(B,255);
    }else if(Serial.read("Pink")){
      analogWrite(R,255);
      analogWrite(G,23);
      analogWrite(B,185);
    }else if(Serial.read("Red")){
      analogWrite(R,255);
      analogWrite(G,0);
      analogWrite(B,0);
    }else if(Serial.read("Green")){
      analogWrite(R,0);
      analogWrite(G,0);
      analogWrite(B,255);
    }else{
      analogWrite(R,255);
      analogWrite(G,143);
      analogWrite(B,23);
    }
    }
  }