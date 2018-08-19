//#include <LiquidCrystal.h>
#include <UTFT.h>
#include <max6675.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define LCD_RS 41
#define LCD_E 39
#define LCD_D4 37
#define LCD_D5 35
#define LCD_D6 33
#define LCD_D7 31

#define thermoDO 5 
#define thermoCS 6 
#define thermoCLK 7 
#define clockHC 4 
#define latchHC 3 
#define dataHC 2 
#define i_temp_head 0
#define i_temp_cool_in 1
#define i_temp_cool_out 2
#define temps_count 4

#define OCR2A_v 110
#define pin_B 16
#define temp2start 20
#define pin_pwm 17
#define averaging 8
#define tiks_per_m 29296
#define rele_pwm_val 1536
#define pwm_rele 0
#define pwm_fan 1
#define pwm_load 2
#define n_rpm 500
#define t_rpm 400
#define ps_in 14
#define ps_out 15

//LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);


int temps[temps_count];
int i_temp = 0;
int shift_data;

int RPM=0;
int rpms[averaging+1];
int amperages[averaging*5+1];
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
boolean runned=false;
int tiks2start_error=0;
int load_pwm_val=512;
int fan_pwm_val=512;
int accel_tiks=0;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
char buffer[5];

extern uint8_t BigFont[];
UTFT myGLCD(CTE32HR,38,39,40,41);

void setup() {
    Serial.begin(9600);

    // Setup the LCD
  myGLCD.InitLCD();
  myGLCD.setFont(BigFont);
  myGLCD.clrScr();
  myGLCD.setColor(0, 255, 0);
  //myGLCD.fillRect(0, 0, 479, 319);
  myGLCD.print("* ROTOR SUMY *", CENTER, 1);

  
    //lcd.begin(20, 4);
    //lcd.setCursor(2, 0);
    //lcd.write("** ROTOR SUMY **");
    //lcd.setCursor(0, 1);
    myGLCD.print("U1-       U2-       U3-",1 ,20);
    myGLCD.print("U4-       U5-       U6-",1 ,37);
    myGLCD.print(" I-      Pow-       En-",1 ,54);
    //lcd.setCursor(0, 2);
    myGLCD.print("T1-      T2-      T3-", 1, 71);
    //lcd.setCursor(0, 3);
    myGLCD.print("RPM-      STATE-" , 1 , 88);

    for(int i = 0; i < temps_count; i++){
        temps[i]=0;
    }
    pinMode(latchHC, OUTPUT);
    pinMode(clockHC, OUTPUT);
    pinMode(dataHC, OUTPUT);

    for (int i=1; i<averaging+1; i++){
        rpms[i]=0;
    }
    for (int i=1; i<averaging*5+1; i++){
        amperages[i]=0;
    }
    pinMode(pin_B, INPUT_PULLUP);
    pinMode(pin_pwm,OUTPUT);
    
    pinMode(ps_in, INPUT);
    pinMode(ps_out, OUTPUT);

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
    pwm.setPWMFreq(1600);
    Wire.setClock(100000);
    for (int i=0; i<16; i++){
      pwm.setPWM(i,0,4096); 
    }

    analogReference(EXTERNAL);
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
  set_averaging(0, val);

}
void set_amperage(int val){
  set_averaging(1, val);
}

void set_averaging(int index, int val){  
    int *temp_arr;
    int aver=averaging;
    if (index==0){
      val=tiks_per_m/val;
      temp_arr=rpms;
    }else{
      temp_arr=amperages;
      aver=aver*5;
    }
    
    int s=0;
    int c=0;
    for (int i=1; i<aver; i++){
        temp_arr[i-1]=temp_arr[i];
        if(temp_arr[i]==0){
            s=0;
            c=0;
        }else{
            s+=temp_arr[i];
            c++;
        }
    }
    s+=val;
    c++;
    temp_arr[aver-1]=val;
    temp_arr[aver]=s/c;
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
      runned=false;
      start_error=false;
      OCR2A=OCR2A_v;
   }
   
    if (temps[i_temp_head]>temp2start){
      ready2start=true;
    }else{
      ready2start=false;
    }
    if (tiks_int>100){
        c_tiks_flag=false;
        tiks_flag=0;                                
        tiks_int=0;
        rpms[averaging]=0;
    }
    if(tiks_flag){
        tiks_flag=0;
        set_rpms(tiks_int2d);
    }
    set_amperage(analogRead(A0));

    if (runned){
        load_pwm_val+=20*(rpms[averaging]-n_rpm);
        if (load_pwm_val>4094){
            load_pwm_val=4094;
        }
        if (load_pwm_val<0){
            load_pwm_val=0;
        }
        fan_pwm_val+=50*(temps[i_temp_cool_in]-42);
        if (fan_pwm_val>1536){
            fan_pwm_val=1536;
        }
        if (fan_pwm_val<384){
            fan_pwm_val=384;
        }
        pwm.setPWM(pwm_fan, 0, fan_pwm_val);
        pwm.setPWM(pwm_load, 0, load_pwm_val);
    }else{
        if (accelerated){
            accel_tiks++;
        }
        if (accel_tiks>3){
            accel_tiks=0;
            runned=true;
        }
        pwm.setPWM(pwm_fan, 0, 4096);
        pwm.setPWM(pwm_load, 0, 4096);
    }

    digitalWrite(ps_out, accelerated*digitalRead(ps_in));
    
    if (accelerating||started&&ready2start){
        if (not(accelerating)){
            accelerating=true;
            pwm.setPWM(pwm_rele, 0, rele_pwm_val);
            delay(300);
        }
        if (rpms[averaging]>t_rpm){
            accelerated=true;
            accelerating=false;
            started=false;
            delay(2000);
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
                runned=false;
                started=false;
                start_error=true;
                tiks2start_error=0;
            }
        }else{
            tiks2start_error=0;
        }
    }

    myGLCD.print("   ", 16*5, 20);
    myGLCD.printNumI(analogRead(A1), 16*4, 20);
    myGLCD.print("   ", 16*14, 20);
    myGLCD.printNumI(analogRead(A2), 16*13, 20);
    myGLCD.print("   ", 16*25, 20);
    myGLCD.printNumI(analogRead(A3), 16*24, 20);
    myGLCD.printNumI(analogRead(A4), 16*4, 37);
    myGLCD.printNumI(analogRead(A5), 16*13, 37);
    myGLCD.printNumI(analogRead(A6), 16*24, 37);

    myGLCD.printNumF((float)(511-amperages[averaging*5])*100/512, 2, 16*4, 54);

    myGLCD.printNumI(temps[i_temp_head], 16*4, 71);
    myGLCD.printNumI(temps[i_temp_cool_in], 16*13, 71);
    myGLCD.printNumI(temps[i_temp_cool_out], 16*22, 71);

    myGLCD.printNumI(rpms[averaging], 16*5, 88);

    myGLCD.printNumI(ready2start, 16*17, 88);
    myGLCD.printNumI(started, 16*18, 88);
    myGLCD.printNumI(accelerated, 16*19, 88);
    myGLCD.printNumI(start_error, 16*20, 88);
    
    temps[i_temp]=thermocouple.readCelsius();
    Serial.print(i_temp);
    Serial.print("-");
    Serial.println(temps[i_temp]);


    
}
