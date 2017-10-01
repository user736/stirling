#include <Arduino.h>
#include <TM1637Display.h>

#define CLK 13
#define DIO 12

TM1637Display display(CLK, DIO);

int t_freq=0;
int E1_PV;
int E2_PV;
const int df=5;
const int pin_B  = 0;
const int pin_E1 = 3;
const int pin_E2 = 4;
int v7=0;

void check_enc(){
  int e1,e2;
  e1=digitalRead(pin_E1);
  e2=digitalRead(pin_E2);
  if (e1!=E1_PV){
    E1_PV=e1;
    if (!e1){
      if (e2){
        t_freq+=df;
      }else{
        t_freq-=df;
      }
      if (t_freq<0){
        t_freq=0;
      }
    }
  }
}

void setup() {
  // put your setup code here, to run once:
    display.setBrightness(0x0f);
    pinMode(pin_E1, INPUT_PULLUP);
    pinMode(pin_E2, INPUT_PULLUP);
    pinMode(pin_B, INPUT_PULLUP);
    pinMode(7,OUTPUT);
    E1_PV=digitalRead(pin_E1);
    E2_PV=digitalRead(pin_E2);
}

void loop() {
  // put your main code here, to run repeatedly:
  check_enc();
  display.showNumberDec(t_freq, false, 4, 0);
  //delay(50);
  digitalWrite(7,v7);
  v7=1-v7;
}
