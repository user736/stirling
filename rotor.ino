//#include <LiquidCrystal.h>
#include <UTFT.h>
#include <max6675.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>

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
#define temp2start -1
#define pin_pwm 17
#define averaging 8
#define tiks_per_m 29296
#define rele_pwm_val 1536
#define pwm_rele 0
#define pwm_fan 3
#define pwm_load 1
#define pwm_pump 4
#define pwm_boost1 2
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
int rpms[averaging+1]={0};
int amperages[averaging*5+1]={0};

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
int pump_cicle=0;
int boost1_val=0;
unsigned mhht=0;
unsigned mhhc=0;
byte mhmt=0;
byte mhmc=0;
byte mhst=0;
byte mhsc=0;
unsigned long mhv_eeprom=0;
byte mh_tick=0;
byte mhds=0;
static char str[35];
int U_coef[]={33,34,33,33,34,34};
int U_val[6]={0};
int U_aver[averaging+1]={0};
boolean shnek_v =1;
int shnek_ce=0;
int shnek_cd=0;
float power=0;
float energy=0;
float energy_ws=0;

extern uint8_t BigFont[];
UTFT myGLCD(CTE32HR,38,39,40,41);

unsigned long EEPROM_ulong_read(int addr) {   
  byte raw[4];
  for(byte i = 0; i < 4; i++) raw[i] = EEPROM.read(addr+i);
  unsigned long &num = (unsigned long&)raw;
  return num;
}
 
void EEPROM_ulong_write(int addr, unsigned long num) {
  byte raw[4];
  (unsigned long&)raw = num;
  for(byte i = 0; i < 4; i++) EEPROM.write(addr+i, raw[i]);
}

void init_mh(){
  unsigned long val;
  val=EEPROM_ulong_read(0);
  mhst=val%60;
  val=val/60;
  mhmt=val%60;
  mhht=val/60;  
}

void save_mh(){
  unsigned long val=0;
  val=mhht*3600+mhmt*60+mhst;
  EEPROM_ulong_write(0, val);
}

void setup() {

    pinMode(9, OUTPUT);
    
    Serial.begin(9600);
    //save_mh();
    init_mh();
    // Setup the LCD
    myGLCD.InitLCD();
    myGLCD.setFont(BigFont);
    myGLCD.clrScr();
    myGLCD.setColor(0, 255, 0);
    
    myGLCD.print("* ROTOR SUMY *", CENTER, 1);
    myGLCD.print("U1-       U2-       U3-",1 ,25);
    myGLCD.print("U4-       U5-       U6-",1 ,47);
    myGLCD.print(" I-      Pow-       En-",1 ,69);
    myGLCD.print("MH current  -", 1, 157);
    myGLCD.print("MH in total -", 1, 179);
    

    for(int i = 0; i < temps_count; i++){
        temps[i]=0;
    }
    pinMode(latchHC, OUTPUT);
    pinMode(clockHC, OUTPUT);
    pinMode(dataHC, OUTPUT);

    pinMode(pin_B, INPUT_PULLUP);
    pinMode(pin_pwm,OUTPUT);
    
    pinMode(ps_in, INPUT);
    pinMode(ps_out, OUTPUT);

    // Timer/Counter 1 initialization
    // Clock source: System Clock
    // Clock value: 250,000 kHz
    // Mode: Normal top=0xFFFF
    // OC1A output: Disconnected
    // OC1B output: Disconnected
    // OC1C output: Disconnected
    // Noise Canceler: Off
    // Input Capture on Falling Edge
    // Timer Period: 0,2 s
    // Timer1 Overflow Interrupt: On
    // Input Capture Interrupt: Off
    // Compare A Match Interrupt: Off
    // Compare B Match Interrupt: Off
    // Compare C Match Interrupt: Off
    TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<COM1C1) | (0<<COM1C0) | (0<<WGM11) | (0<<WGM10);
    TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (0<<CS12) | (1<<CS11) | (1<<CS10);
    TCNT1H=0x3C;
    TCNT1L=0xB0;
    ICR1H=0x00;
    ICR1L=0x00;
    OCR1AH=0x00;
    OCR1AL=0x00;
    OCR1BH=0x00;
    OCR1BL=0x00;
    OCR1CH=0x00;
    OCR1CL=0x00;
    // Timer/Counter 1 Interrupt(s) initialization
    TIMSK1=(0<<ICIE1) | (0<<OCIE1C) | (0<<OCIE1B) | (0<<OCIE1A) | (1<<TOIE1);

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
    Wire.setClock(100000);
    for (int i=0; i<16; i++){
      pwm.setPWM(i,0,4096); 
    }

    analogReference(EXTERNAL);
}


ISR(TIMER1_OVF_vect){
  // Reinitialize Timer1 value
  TCNT1H=0x3CB0 >> 8;
  TCNT1L=0x3CB0 & 0xff;
  // Place your code here
  if (runned){
      mh_tick++;
      if (mh_tick==5){
          mh_tick=0;
          mhds++;
      }
  }
  energy_ws=energy_ws+power*0.2;
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
void set_U1(int val){
  set_averaging(2, val);
}

void set_averaging(int index, int val){  
    int *temp_arr;
    int aver=averaging;
    if (index==0){
      val=tiks_per_m/val;
      temp_arr=rpms;
    }else if(index==1){
      temp_arr=amperages;
      aver=aver*5;
    }else{
      temp_arr=U_aver;
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
    s=s+val;
    c++;
    temp_arr[aver-1]=val;
    temp_arr[aver]=s/c;
}

void loop() {
    set_U1(analogRead(1));
    U_val[0]=U_aver[averaging];
    for (int i=1; i<6; i++){
      U_val[i]=analogRead(i+1);
    }
    power=(float)(511-amperages[averaging*5])*100/512*U_val[0]*5/U_coef[0];
    digitalWrite(9, shnek_v);
    if (mhds>0){
      mhsc+=mhds;
      mhst+=mhds;
      mhds=0;
      if(mhsc>59){
        mhmc+=1;
        mhsc-=60;
        if (mhmc>59){
          mhhc+=1;
          mhmc-=60;
        }
      }
      if(mhst>59){
        mhmt+=1;
        mhst-=60;
        if (mhmt>59){
          mhht+=1;
          mhmt-=60;
        }
      }
    }
    boost1_val=analogRead(A8)/2;
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
                OCR2A+=30;
            }
        }
    }

    if (accelerated||accelerating||started){

    if(not(shnek_v)){
      shnek_ce++;
      if (shnek_ce>3){
        shnek_ce=0;
        shnek_v=1;
      }
    }else{
      shnek_cd++;
      if (shnek_cd>12){
        shnek_cd=0;
        shnek_v=0;
      }
    }
    }else{
       shnek_v=1;
    }
    
    if (accelerated||accelerating){  
      pwm.setPWM(pwm_pump, 4096, 0);
      pump_cicle=0;
      pwm.setPWM(pwm_boost1,0,boost1_val);
      //pwm.setPWM(pwm_boost1, 4096, 0);
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
                save_mh();
            }
        }else{
            tiks2start_error=0;
        }
    }else{
        pwm.setPWM(pwm_boost1, 0, 4096);
        pump_cicle++;
        //pwm.setPWM(pwm_pump, 0, 4096);
    }

    if (pump_cicle>20){
      pwm.setPWM(pwm_pump, 0, 4096);
    }
    //myGLCD.print("   ", 16*5, 20);
    myGLCD.printNumF((float)U_val[0]*5/U_coef[0], 2, 16*4, 25);
    //myGLCD.print("   ", 16*14, 20);
    myGLCD.printNumF(U_val[1]*5/U_coef[1], 2,16*13, 25);
    //myGLCD.print("   ", 16*25, 25);
    myGLCD.printNumF(U_val[2]*5/U_coef[2], 2, 16*24, 25);
    myGLCD.printNumF(U_val[3]*5/U_coef[3], 2, 16*4, 47);
    myGLCD.printNumF(U_val[4]*5/U_coef[4], 2, 16*13, 47);
    myGLCD.printNumF(U_val[5]*5/U_coef[5], 2, 16*24, 47);

    myGLCD.printNumF((float)(511-amperages[averaging*5])*100/512, 2, 16*4, 69);
    myGLCD.printNumF(power, 2, 16*13, 69);
    myGLCD.printNumF(energy_ws/3600, 2, 16*24, 69);
    //myGLCD.printNumI(temps[i_temp_head], 16*4, 71);
    //myGLCD.printNumI(temps[i_temp_cool_in], 16*13, 71);
    //myGLCD.printNumI(temps[i_temp_cool_out], 16*22, 71);
    sprintf(str, "TH1-%03d  TH2-%03d   ", temps[i_temp_head], temps[1]);
    myGLCD.print(str, 16*0, 91);
    sprintf(str, "TC1-%03d  TC2-%03d  ", temps[2], temps[3]);
    myGLCD.print(str, 16*0, 113);
    sprintf(str, "RPM-%04d  STATE-%d%d%d%d ",rpms[averaging], ready2start, started, accelerated,start_error);
    myGLCD.print(str , 1 , 135);
    
    sprintf(str, "%05d:%02d:%02d", mhhc, mhmc, mhsc);
    myGLCD.print(str, 16*15, 157);
    Serial.println(str);
    sprintf(str, "%05d:%02d:%02d", mhht, mhmt, mhst);
    myGLCD.print(str, 16*15, 179);
    Serial.println(str);
    Serial.println(temps[i_temp_head]);
    temps[i_temp]=thermocouple.readCelsius();
    //char t4_text[30];
    //sprintf(t4_text, "t4- %03d", temps[3]);
    //myGLCD.print(t4_text, 16*1, 201);
    for (int i=0; i<=averaging; i++){
      Serial.print(U_aver[i]);
      Serial.print(' ');
    }
    Serial.println(' ');
}
