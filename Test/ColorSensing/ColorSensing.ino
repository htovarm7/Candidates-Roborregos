#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include "ColorConverterLib.h"

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);

const int R = 6;
const int G = 8;
const int B = 7;

String getColor(double hue)
{
    Serial.println(hue);
    if (hue > 30 && hue < 59.5) {
        return "Yellow";
    }
    else if (hue > 59.9 && hue < 65) {
        return "Black";
    }
    else if (hue > 222 && hue < 265) {
        return "Purple";
    }
    else if (hue > 195 && hue < 222) {
        return "Blue";
    }
    else if (hue > 300 && hue < 360) {
        return "Pink";
    }
    else if (hue > 0 && hue < 25) {
        return "Red";
    }
    else if (hue > 85 && hue < 125) {
        return "Green";
    }
    else {
        return "None of the above";
    }
}

void showColor(String color){
    if(color == "Yellow"){
      analogWrite(R,255);
      analogWrite(G,255);
      analogWrite(B,0);
    }else if(color == "Black"){
      analogWrite(R,0);
      analogWrite(G,0);
      analogWrite(B,0);
    }else if(color == "Purple"){
      analogWrite(R,100);
      analogWrite(G,0);
      analogWrite(B,100);
    }else if(color == "Blue"){
      analogWrite(R,0);
      analogWrite(G,0);
      analogWrite(B,255);
    }else if(color == "Pink"){
      analogWrite(R,255);
      analogWrite(G,23);
      analogWrite(B,192);
    }else if(color == "Red"){
      analogWrite(R,255);
      analogWrite(G,0);
      analogWrite(B,0);
    }else if(color == "Green"){
      analogWrite(R,0);
      analogWrite(G,255);
      analogWrite(B,0);
    }else{
      analogWrite(R,255);
      analogWrite(G,143);
      analogWrite(B,23);
    }
}

void setup()
{
  Serial.begin(9600);
  if (!tcs.begin()) {
    Serial.println("No se detectó el sensor TCS34725. Verifique la conexión.");
    while (1);
  }else{
    Serial.println("Se inicio correctamente");
  }
  pinMode(R, OUTPUT);
  pinMode(G, OUTPUT);
  pinMode(B, OUTPUT);
}


void loop()
{
  // Serial.println("Prueba de comunicación serial");
  // delay(1000);

  // Serial.println("¡Hola, mundo!");
  uint16_t clear, red, green, blue;

  tcs.setInterrupt(false);
  delay(60); // Takes 60ms to capture the color
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
  String detectedColor = getColor(hue * 360);
  Serial.println("Hue: " + String(hue * 360) + ", Color detected: " + detectedColor);  // Print hue and detected color
  showColor(detectedColor);
  // differentColor();

  delay(1000);
}