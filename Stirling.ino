#include <Arduino.h>
#include <TM1637Display.h>
#include <EEPROM.h>
#include "max6675.h"

int thermoDO = 46;
int thermoCS = 48;
int thermoCLK = 50;

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);
#define CLK_1 53
#define DIO_1 51
#define CLK_2 37
#define DIO_2 39
#define CLK_3 23
#define DIO_3 27

TM1637Display display_1(CLK_1, DIO_1);
TM1637Display display_2(CLK_2, DIO_2);
TM1637Display display_3(CLK_3, DIO_3);

boolean started=false;
boolean accelerated=false;
boolean flag=false;
int i_int=0;
int tiks=0;
int t_freq=200;
int E1_PV;
int E2_PV;
const int df=10;
const int pin_B  = 49;
const int pin_E1 = 47;
const int pin_E2 = 45;
const int pin_pwm=7;
const int pin_rele=5;
int P_tiks=0;
int intervals=0;
int OCR2A_v=127;
int temp=0;
int i_t=0;

float EEPROM_float_read(int addr) {    
  byte raw[4];
  for(byte i = 0; i < 4; i++) raw[i] = EEPROM.read(addr+i);
  float &num = (float&)raw;
  return num;
}

void EEPROM_float_write(int addr, float num) {
  byte raw[4];
  (float&)raw = num;
  for(byte i = 0; i < 4; i++) EEPROM.write(addr+i, raw[i]);
}

int check_enc(){
  int e1,e2;
  e1=digitalRead(pin_E1);
  e2=digitalRead(pin_E2);
  if (e1!=E1_PV){
    E1_PV=e1;
    if (!e1){
      if (e2){
        return 1;
      }else{
        return -1;
      }
    }
  }
  return 0;
}

boolean check_conf_timeout(int but, int timeout){
  for(int i=0; i<timeout; i++){
      if (digitalRead(but)){
        return false;
      }
      delay(100);
    }
    return true;
  }

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
    display_1.setBrightness(0x0f);
    display_2.setBrightness(0x0f);
    display_3.setBrightness(0x0f);
    pinMode(3, INPUT_PULLUP);
    pinMode(2, INPUT_PULLUP);
    pinMode(pin_E1, INPUT_PULLUP);
    pinMode(pin_E2, INPUT_PULLUP);
    pinMode(pin_B, INPUT_PULLUP);
    pinMode(pin_pwm,OUTPUT);
    pinMode(pin_rele,OUTPUT);
    E1_PV=digitalRead(pin_E1);
    E2_PV=digitalRead(pin_E2);

    // Timer/Counter 2 initialization
    // Clock source: System Clock
    // Clock value: 125,000 kHz
    // Mode: Normal top=0xFF
    // OC2A output: Disconnected
    // OC2B output: Disconnected
    // Timer Period: 2,048 ms
    ASSR=(0<<EXCLK) | (0<<AS2);
    TCCR2A=(0<<COM2A1) | (0<<COM2A0) | (0<<COM2B1) | (0<<COM2B0) | (0<<WGM21) | (0<<WGM20);
    TCCR2B=(0<<WGM22) | (1<<CS22) | (0<<CS21) | (1<<CS20);
    TCNT2=0x00;
    OCR2A=0x7f;
    OCR2B=0x00;

    // Timer/Counter 2 Interrupt(s) initialization
    TIMSK2=(0<<OCIE2B) | (1<<OCIE2A) | (1<<TOIE2);

    // External Interrupt(s) initialization
// INT0: Off
// INT1: Off
// INT2: Off
// INT3: Off
// INT4: On
// INT4 Mode: Rising Edge
// INT5: On
// INT5 Mode: Rising Edge
// INT6: Off
// INT7: Off
EICRA=(0<<ISC31) | (0<<ISC30) | (0<<ISC21) | (0<<ISC20) | (0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (0<<ISC00);
EICRB=(0<<ISC71) | (0<<ISC70) | (0<<ISC61) | (0<<ISC60) | (1<<ISC51) | (1<<ISC50) | (1<<ISC41) | (1<<ISC40);
EIMSK=(0<<INT7) | (0<<INT6) | (1<<INT5) | (1<<INT4) | (0<<INT3) | (0<<INT2) | (0<<INT1) | (0<<INT0);
EIFR=(0<<INTF7) | (0<<INTF6) | (1<<INTF5) | (1<<INTF4) | (0<<INTF3) | (0<<INTF2) | (0<<INTF1) | (0<<INTF0);
    PCICR=(0<<PCIE2) | (0<<PCIE1) | (0<<PCIE0);

    if(check_conf_timeout(pin_B,20)){
        display_1.showNumberDec(OCR2A_v, false, 4, 0);
        while(not(check_conf_timeout(pin_B,20))){
          display_1.showNumberDec(OCR2A_v, false, 4, 0);
          OCR2A_v+=check_enc();
        }
        EEPROM_float_write(0, OCR2A_v);
        
    }

    OCR2A_v=EEPROM_float_read(0);
    OCR2A=OCR2A_v;
}

ISR (INT4_vect){
  if (flag){
    tiks++;
  }
  flag=false;
}

ISR (INT5_vect){
  flag=true;
}

ISR (TIMER2_COMPA_vect) {
  digitalWrite(pin_pwm,0);
}

ISR(TIMER2_OVF_vect) {
  if (i_int==0){
    if (intervals==0){
      P_tiks=tiks;
      tiks=0;
    }
    intervals++;
    if (intervals==50){
      intervals=0;
    }
    digitalWrite(pin_pwm,1);
  }
  i_int++;
  if (i_int==5){
    i_int=0;
  }
  if (not(started)){
    t_freq+=check_enc()*df;
    if (t_freq<0){
      t_freq=0;
    }
  }else{
    
  }
}

void loop() {
  if (not(digitalRead(pin_B))){
    started=true;
    accelerated=false;
    OCR2A=OCR2A_v;
    digitalWrite(pin_rele,1);
  }
  if (not(started)){
    display_1.showNumberDec(t_freq, false, 4, 0);
  }else{
    display_1.showNumberDec(P_tiks*19.5, false, 4, 0);
    if (P_tiks*19.5>t_freq){
      accelerated=true;
    }else{
      if (OCR2A<0xF0){
        OCR2A+=30;
      }
    }
    
    if (accelerated){
      delay(3000);
      digitalWrite(pin_rele,0);
      OCR2A=OCR2A_v;
    }
  }
  display_2.showNumberDec(digitalRead(18), false, 4, 0);
  if (i_t==0){
    temp=thermocouple.readCelsius();
  }
  i_t++;
  if (i_t==5){
    i_t=0;
  }
  display_3.showNumberDec(temp, false, 4, 0);
  delay(100);
}
