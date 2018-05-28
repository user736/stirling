#include <LiquidCrystal.h>
#include <max6675.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define LCD_RS 41
#define LCD_E 39
#define LCD_D4 37
#define LCD_D5 35
#define LCD_D6 33
#define LCD_D7 31

#define thermoDO 46
#define thermoCS 48
#define thermoCLK 50
#define clockHC 44
#define latchHC 42
#define dataHC 40
#define i_temp_head 0
#define i_temp_cool_in 1
#define i_temp_cool_out 2
#define temps_count 4

#define OCR2A_v 127
#define pin_B 49
#define temp2start 30
#define pin_pwm 29
#define averaging 8
#define tiks_per_m 29296
#define rele_pwm_val 1024
#define fan_pwm_val 1024
#define pwm_rele 0
#define pwm_fan 1
#define pwm_load 2
#define n_rpm 500
#define t_rpm 400

LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);


int temps[temps_count];
int i_temp = 0;
int shift_data;

int RPM=0;
int rpms[averaging+1];
int i_int=0;
int tiks_int2d=0;
int tiks_int=0;
int tiks2rpm_clean=0;
boolean started=false;
boolean accelerated=false;
boolean ready2start=false;
boolean tiks_flag=false;
boolean c_tiks_flag=false;
boolean accelerating=false;
boolean start_error=false;
int tiks2start_error=0;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
    Serial.begin(9600);
  
    lcd.begin(20, 4);
    lcd.setCursor(2, 0);
    lcd.write("** ROTOR SUMY **");
    lcd.setCursor(0, 1);
    lcd.write("P1-    P2-    P3-");
    lcd.setCursor(0, 2);
    lcd.write("T1-    T2-    T3-");
    lcd.setCursor(0, 3);
    lcd.write("RPM-      STATE-");

    for(int i = 0; i < temps_count; i++){
        temps[i]=0;
    }
    pinMode(latchHC, OUTPUT);
    pinMode(clockHC, OUTPUT);
    pinMode(dataHC, OUTPUT);

    for (int i=1; i<averaging+1; i++){
        rpms[i]=0;
    }
    pinMode(pin_B, INPUT_PULLUP);
    pinMode(pin_pwm,OUTPUT);

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
    OCR2A=OCR2A_v;
    OCR2B=0x00;
    // Timer/Counter 2 Interrupt(s) initialization
    TIMSK2=(0<<OCIE2B) | (1<<OCIE2A) | (1<<TOIE2);
    
    // External Interrupt(s) initialization
    // INT0: Off
    // INT1: Off
    // INT2: Off
    // INT3: On
    // INT3 Mode: Falling Edge
    // INT4: Off
    // INT5: Off
    // INT6: Off
    // INT7: Off
    EICRA=(1<<ISC31) | (0<<ISC30) | (0<<ISC21) | (0<<ISC20) | (0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (0<<ISC00);
    EICRB=(0<<ISC71) | (0<<ISC70) | (0<<ISC61) | (0<<ISC60) | (0<<ISC51) | (0<<ISC50) | (0<<ISC41) | (0<<ISC40);
    EIMSK=(0<<INT7) | (0<<INT6) | (0<<INT5) | (0<<INT4) | (1<<INT3) | (0<<INT2) | (0<<INT1) | (0<<INT0);
    EIFR=(0<<INTF7) | (0<<INTF6) | (0<<INTF5) | (0<<INTF4) | (1<<INTF3) | (0<<INTF2) | (0<<INTF1) | (0<<INTF0);

    pwm.begin();
    pwm.setPWMFreq(1000);
    Wire.setClock(200000);
    for (int i=0; i<16; i++){
      pwm.setPWM(i,0,4096); 
    }
}

ISR (TIMER2_COMPA_vect) {
  digitalWrite(pin_pwm,0);
}

ISR (INT3_vect){
  tiks_int2d=tiks_int;
  tiks_int=0;
  tiks_flag=true;
  c_tiks_flag=true;
}

ISR(TIMER2_OVF_vect) {  
    if (i_int==0){
        digitalWrite(pin_pwm,1);
    }
    i_int++;
    if (i_int==5){
        i_int=0;
    }
    if (c_tiks_flag){
        tiks_int++;
    }
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
    delay(550);

   if (not(digitalRead(pin_B))){
      started=true;
      accelerated=false;
      start_error=false;
      OCR2A=OCR2A_v;
   }
   
    if (temps[i_temp_head]>temp2start){
      ready2start=true;
    }else{
      ready2start=false;
    }
    if (tiks_int>200){
        c_tiks_flag=0;
        tiks_int=0;
        rpms[averaging]=0;
    }
    if(tiks_flag){
        tiks_flag=0;
        set_rpms(tiks_int2d);
    }

    if (accelerated){
      pwm.setPWM(pwm_fan, 0, fan_pwm_val);
      pwm.setPWM(pwm_load, 0, load_pwm_val);
    }

    if (started&&ready2start){
        if (not(accelerating)){
            accelerating=true;
            pwm.setPWM(pwm_rele, 0, rele_pwm_val);
            delay(300);
        }
        if (rpms[averaging]>t_rpm){
            accelerated=true;
            accelerating=false;
            started=false;
            delay(3000);
            pwm.setPWM(pwm_rele, 0, 4096);
            OCR2A=OCR2A_v;
        }else{
            if (OCR2A<0xF0&&not(accelerated)){
                OCR2A+=20;
            }
        }
    }

    if (accelerated||accelerating){
        if (rpms[averaging]==0){
            tiks2start_error++;
            if (tiks2start_error>5){
                pwm.setPWM(pwm_rele, 0, 4096);
                OCR2A=OCR2A_v;
                accelerating=false;
                accelerated=false;
                started=false;
                start_error=true;
                tiks2start_error=0;
            }
        }else{
            tiks2start_error=0;
        }
    }
    
    lcd.setCursor(3, 2);
    lcd.print("    ");
    lcd.setCursor(3, 2);
    lcd.print(temps[i_temp_head]);
    lcd.setCursor(10, 2);
    lcd.print("    ");
    lcd.setCursor(10, 2);
    lcd.print(temps[i_temp_cool_in]);
    lcd.setCursor(17, 2);
    lcd.print("    ");
    lcd.setCursor(17, 2);
    lcd.print(temps[i_temp_cool_out]);
    lcd.setCursor(4,3);
    lcd.print("      ");
    lcd.setCursor(4,3);
    lcd.print(rpms[averaging]);
    lcd.setCursor(16,3);
    lcd.print("    ");
    lcd.setCursor(16,3);
    lcd.print(ready2start);
    lcd.print(started);
    lcd.print(accelerated);
    lcd.print(start_error);
    
    temps[i_temp]=thermocouple.readCelsius();
    Serial.print(i_temp);
    Serial.print("-");
    Serial.println(temps[i_temp]);

    
}
