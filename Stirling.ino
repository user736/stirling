#include <Arduino.h>
#include <TM1637Display.h>
#include <EEPROM.h>
#include "max6675.h"

#define i_temp_head 0
#define i_temp_cool_in 1
#define i_temp_cool_out 2
#define CLK_1 53
#define DIO_1 51
#define CLK_2 37
#define DIO_2 39
#define CLK_3 23
#define DIO_3 27
#define thermoDO 46
#define thermoCS 48
#define thermoCLK 50
#define clockHC 44
#define latchHC 42
#define dataHC 40
#define rpm_load1 450
#define rpm_load2 470
#define rpm_load3 550
#define rpm_load4 600
#define rpm_unload1 400
#define rpm_unload2 400
#define rpm_unload3 500
#define rpm_unload4 550
#define load1 36
#define load2 30
#define load3 34
#define load4 32
#define temp2start 120 //200
#define tiks_per_m 29296
#define averaging 50
#define temps_count 12

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

TM1637Display display_1(CLK_1, DIO_1);
TM1637Display display_2(CLK_2, DIO_2);
TM1637Display display_3(CLK_3, DIO_3);

int i_temp=0;
int temps[temps_count];
boolean started=false;
boolean accelerated=false;
boolean flag=false;
boolean ready2start=false;
boolean tiks_flag=false;
int rpms[averaging+1];
int i_int=0;
int tiks=0;
int tiks_int=0;
int tiks_int2d=1;
int tiks_h=0;
int t_freq=200;
int E1_PV;
int E2_PV;
const int df=10;
const int pin_B  = 49;
const int pin_E1 = 47;
const int pin_E2 = 45;
const int pin_pwm=7;
const int pin_rele=4;
int P_tiks=0;
int P_tiks_h=0;
int intervals=0;
int OCR2A_v=127;
int temp=0;
int i_t=0;
int tiks2rpm_clean=0;
int shift_data;

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

void set_rpms(int val){
  val=tiks_per_m/val;
  int s=0;
  int c=0;
  for (int i=1; i<averaging; i++){
    rpms[i-1]=rpms[i];
    if(rpms[i]==0){
      s=0;
      c=0;
    }else{
      s+=rpms[i];
      c++;
    }
  }
  s+=val;
  c++;
  rpms[averaging]=s/c;
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
    for (int i=1; i<averaging+1; i++){
      rpms[i]=0;
    }
    for(int i = 0; i < temps_count; i++){
      temps[i]=0;
    }
    display_1.setBrightness(0x0f);
    display_2.setBrightness(0x0f);
    display_3.setBrightness(0x0f);
    pinMode(latchHC, OUTPUT);
    pinMode(clockHC, OUTPUT);
    pinMode(dataHC, OUTPUT);
    pinMode(3, INPUT_PULLUP);
    pinMode(2, INPUT_PULLUP);
    pinMode(pin_E1, INPUT_PULLUP);
    pinMode(pin_E2, INPUT_PULLUP);
    pinMode(pin_B, INPUT_PULLUP);
    pinMode(load1, OUTPUT);
    pinMode(load2, OUTPUT);
    pinMode(load3, OUTPUT);
    pinMode(load4, OUTPUT);
    digitalWrite(load1, 1);
    digitalWrite(load2, 1);
    digitalWrite(load3, 1);
    digitalWrite(load4, 1);
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
// INT3: On
// INT3 Mode: Falling Edge
// INT4: On
// INT4 Mode: Rising Edge
// INT5: On
// INT5 Mode: Rising Edge
// INT6: Off
// INT7: Off
EICRA=(1<<ISC31) | (0<<ISC30) | (0<<ISC21) | (0<<ISC20) | (0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (0<<ISC00);
EICRB=(0<<ISC71) | (0<<ISC70) | (0<<ISC61) | (0<<ISC60) | (1<<ISC51) | (1<<ISC50) | (1<<ISC41) | (1<<ISC40);
EIMSK=(0<<INT7) | (0<<INT6) | (1<<INT5) | (1<<INT4) | (1<<INT3) | (0<<INT2) | (0<<INT1) | (0<<INT0);
EIFR=(0<<INTF7) | (0<<INTF6) | (1<<INTF5) | (1<<INTF4) | (1<<INTF3) | (0<<INTF2) | (0<<INTF1) | (0<<INTF0);
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

ISR (INT3_vect){
  tiks_h++;
  tiks_int2d=tiks_int;
  tiks_int=0;
  tiks_flag=true;
  tiks2rpm_clean=0;
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
      tiks2rpm_clean++;
      P_tiks=tiks;
      P_tiks_h=tiks_h;
      tiks=0;
      tiks_h=0;
    }
    intervals++;
    if (intervals==75){
      intervals=0;
    }
    digitalWrite(pin_pwm,1);
  }
  i_int++;
  tiks_int++;
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
  i_temp++;
  if (i_temp>=temps_count){
    i_temp=0;
  }
  digitalWrite(latchHC, LOW);
  shift_data = 1<<i_temp;
  shiftOut(dataHC, clockHC, MSBFIRST, (shift_data >> 8)); 
  shiftOut(dataHC, clockHC, MSBFIRST, shift_data);
  digitalWrite(latchHC, HIGH);
  delay(50);
  
  if(tiks2rpm_clean>9){
    rpms[averaging]=0;
    rpms[averaging-1]=0;
  }
  if(tiks_flag){
    tiks_flag=0;
    set_rpms(tiks_int2d);
  }

  if (ready2start&&temp>temp2start){
    if(not(accelerated)){
      digitalWrite(pin_rele,1);
    }
    started=true;
  }
 if (not(digitalRead(pin_B))){
      ready2start=true;
      accelerated=false;
      OCR2A=OCR2A_v;
      display_2.showNumberDec(temps[i_temp_cool_in], false, 4, 0);
      display_1.showNumberDec(temps[i_temp_cool_out], false, 4, 0);
  }else{
      display_2.showNumberDec(P_tiks_h*78, false, 4, 0);
      if (not(ready2start)){
          display_1.showNumberDec(t_freq, false, 4, 0);
      }else{
          display_1.showNumberDec(rpms[averaging], false, 4, 0);
          //display_1.showNumberDec(P_tiks*13, false, 4, 0);     
      }
  }
  if (started){
    
    if (rpms[averaging]>rpm_load1){
      digitalWrite(load1,0);
    }else{
      if (rpms[averaging]<rpm_unload1){
        digitalWrite(load1,1);
      }
    }
    if (rpms[averaging]>rpm_load2){
      digitalWrite(load2,0);
    }else{
      if (rpms[averaging]<rpm_unload2){
        digitalWrite(load2,1);
      }
    }
    if (rpms[averaging]>rpm_load3){
      digitalWrite(load3,0);
    }else{
      if (rpms[averaging]<rpm_unload3){
        digitalWrite(load3,1);
      }
    }
    if (rpms[averaging]>rpm_load4){
      digitalWrite(load4,0);
    }else{
      if (rpms[averaging]<rpm_unload4){
        digitalWrite(load4,1);
      }
    }

    if (rpms[averaging]>t_freq){
      accelerated=true;
    }else{
      if (OCR2A<0xF0&&not(accelerated)){
        OCR2A+=20;
      }
    }
    
    if (accelerated&&OCR2A>OCR2A_v){
      delay(3000);
      digitalWrite(pin_rele,0);
      OCR2A=OCR2A_v;
    }
  }
  delay(300);
   temps[i_temp]=thermocouple.readCelsius();
  Serial.println(i_temp);
  Serial.println(temps[i_temp]);
  //i_t++;
  //if (i_t==5){
  //  i_t=0;
  //}
  display_3.showNumberDec(temps[i_temp_head], false, 4, 0);
  delay(100);
}
