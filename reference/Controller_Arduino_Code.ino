/* 
Emre Kalem, 04.2025 Eskisehir,Turkiye

CE Pin: 9, 
CSN Pin: 10, 
SCK Pin: 13, 
MOSI Pin: 11 
MISO Pin: 12



Before uploading please install RF24 Library, 
--> Sketch / Include Library / Manage Libraries / Search RF24 by TMRh20 / Install

*/

#include <LiquidCrystal.h>
#include  <SPI.h> 
#include "nRF24L01.h"
#include "RF24.h" 

const int rs = 3, en = 4, d4 = 5, d5 = 6, d6 = 7, d7 = 8;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

int message[10];

int x;
int y;
int a;
int b;
int sw1;
int sw2;
int tg1;
int tg2;
int pt1;
int pt2;

const int x_pin = A0; // Joystick1 X Axis
const int y_pin = A1; // Joystick1 Y Axis
const int a_pin = A2; // Joystick2 X Axis
const int b_pin = A3; // Joystick2 Y Axis
const int pt1_pin = A4; // pot 1
const int pt2_pin = A5; // pot 2

const int sw1_pin = 4; // Joystick 1 Switch
const int sw2_pin = 5; // Joystick 2 Switch
const int tg1_pin = 2; // toggle switch 1
const int tg2_pin = 3; // toggle switch 2



RF24 transmitter(9,10);  //CE-CSN
const uint64_t kanal = 0xE8E8F0F0E1LL;


void setup() 
{
  pinMode(x_pin, INPUT);
  pinMode(y_pin, INPUT);
  pinMode(a_pin, INPUT);
  pinMode(b_pin, INPUT);
  pinMode(pt1_pin, INPUT);
  pinMode(pt2_pin, INPUT);

  pinMode(sw1_pin, INPUT_PULLUP);
  pinMode(sw2_pin, INPUT_PULLUP);
  pinMode(tg1_pin, INPUT_PULLUP);
  pinMode(tg2_pin, INPUT_PULLUP);
  
  Serial.begin(9600);
  transmitter.begin();
  transmitter.openWritingPipe(kanal); 
  lcd.begin(16, 2);
  lcd.print("Melih Mentes");
  

}

void loop() 
{
  x = analogRead(x_pin);
  y = analogRead(y_pin);
  a = analogRead(a_pin);
  b = analogRead(b_pin);
  pt1 = analogRead(pt1_pin);
  pt2 = analogRead(pt2_pin);

  sw1 = digitalRead(sw1_pin);
  sw2 = digitalRead(sw2_pin);
  tg1 = digitalRead(tg1_pin);
  tg2 = digitalRead(tg2_pin);
  message[0] = x;
  message[1] = y;
  message[2] = a;
  message[3] = b;
  message[4] = pt1;
  message[5] = pt2;
  message[6] = tg1;
  message[7] = tg2;
  message[8] = sw1;
  message[9] = sw2;
  
  
  transmitter.write(message, sizeof(message));

  lcd.setCursor(0, 1);
  lcd.print(millis() / 1000);

  Serial.print(x);
  Serial.print(" ");
  Serial.print(y);
  Serial.print(" ");
  Serial.print(a);
  Serial.print(" ");
  Serial.print(b);
  Serial.print(" ");
  Serial.print(pt1);
  Serial.print(" ");
  Serial.print(pt2);
  Serial.print(" ");
  Serial.print(tg1);
  Serial.print(" ");
  Serial.print(tg2);
  Serial.print(" ");
  Serial.print(sw1);
  Serial.print(" ");
  Serial.println(sw2);

}
