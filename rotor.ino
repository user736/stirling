//#include <LiquidCrystal.h>
#include <UTFT.h>
#include <max6675.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>

#define tc_delta 4


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
#define i_temp_head1 0
#define i_temp_head2 1
#define i_temp_cool_in 2
#define i_temp_cool_out 3
#define temps_count 10

#define OCR2A_v 110
#define pin_B 16
#define temp2start 400
#define pin_pwm 17
#define pin_pump 11
#define averaging 16
#define tiks_per_m 29296
#define rele_pwm_val 1536
#define pwm_rele 5
#define pwm_fan 3
#define pwm_load 1
#define pwm_pump 2
#define pwm_boost1 4
#define t_rpm 350
#define ps_in 15
#define ps_out 14
#define ref_U 5

//LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);


int temps[temps_count];
int i_temp = 0;
int shift_data;
int n_rpm=500;
int n_rpm_delta=500;


int RPM=0;
int rpms[averaging+1]={0};
byte rpms_pos=0;
int amperages[averaging+1]={0};
byte amps_pos=0;

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
int load_pwm_val=0;
int fan_pwm_val=512;
int max_fan_pwm_val=512;
int accel_tiks=0;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
char buffer[5];
int pump_cicle=0;
int boost1_val=0;
int boost1_val_delta=0;
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
int UB_aver[averaging+1]={0};
byte U_pos=0;
byte UB_pos=0; 
boolean shnek_v =1;
int shnek_ce=0;
int shnek_cd=0;
float power=0;
float energy=0;
float energy_ws=0;
byte clock_source=0;
byte tc_counter=0;
byte test_v=0;
byte i_line=0;
byte shnek_e_delta;
byte shnek_d_delta;
byte shnek_e_v;
byte shnek_d_v;

boolean fan_en=false;
boolean acc_dis=false;
boolean acc_charge=true;
byte acc_ch_m=0;
byte acc_ch_h=0;

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
    //analogReference(EXTERNAL);
    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);
    pinMode(pin_pump,OUTPUT);
    digitalWrite(pin_pump,1);
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
  clock_source++;
  energy_ws=energy_ws+power*0.2;
}

ISR (TIMER2_COMPA_vect) {
  digitalWrite(pin_pwm,0);
}

ISR (INT3_vect){
  rpms[rpms_pos]=tiks_per_m/tiks_int;
  rpms_pos++;
  if(rpms_pos>=averaging){
    rpms_pos=0;
  }
  //tiks_int2d=tiks_int;
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

void set_rpms(){
  set_averaging(rpms, rpms_pos);
}
void set_amperage(int val){
  amperages[amps_pos]=val;
  amps_pos++;
    if (amps_pos>averaging){
      amps_pos=0;
    }
  set_averaging(amperages, amps_pos);
}
void set_U1(int val){
  if ((val<30)&&(U_aver[averaging]!=0)){
    set_zero(U_aver,averaging);
  }
  U_aver[U_pos]=val;
  U_pos++;
    if (U_pos>averaging){
      U_pos=0;
    }
  set_averaging(U_aver, U_pos);
}

void set_UB(int val){
  UB_aver[UB_pos]=val;
  UB_pos++;
    if (UB_pos>averaging){
      UB_pos=0;
    }
  set_averaging(UB_aver, UB_pos);
}

void set_zero(int *arr, int a_size){
  for (int i=0; i<a_size; i++) arr[i]=0;
}

void set_averaging(int *temp_arr, int pos){  
    
    int aver=averaging;
    
    int s=0;
    int c=0;
    for (int i=pos; i<aver+pos; i++){
        //if(temp_arr[i%aver]==0){
        //    s=0;
        //    c=0;
        //}else{
            s+=temp_arr[i%aver];
            c++;
        //}
    }
    if (s==0){
      temp_arr[aver]=0;
    }else{
      temp_arr[aver]=s/c;
    }
}

void loop() {
    //test_v=1-test_v;
    //digitalWrite(11,test_v);
    
    
    tc_counter+=clock_source;
    if (tc_counter>=tc_delta){
      temps[i_temp]=thermocouple.readCelsius();
      i_temp++;
      if (i_temp>=temps_count){
          i_temp=0;
      }
      digitalWrite(latchHC, LOW);
      shift_data = 1<<i_temp;
      //Serial.println(shift_data);
      shiftOut(dataHC, clockHC, MSBFIRST, (shift_data >> 8)); 
      shiftOut(dataHC, clockHC, MSBFIRST, shift_data);
      digitalWrite(latchHC, HIGH);
      tc_counter=0;
    }
    shnek_ce+=clock_source;
    shnek_cd+=clock_source;
    clock_source=0;

    set_U1(analogRead(1));
    U_val[0]=U_aver[averaging];
    set_UB(analogRead(2));
    U_val[1]=UB_aver[averaging];
    for (int i=2; i<6; i++){
      U_val[i]=analogRead(i+1);
    }
    power=(float)(amperages[averaging]-507)*100/512*U_val[0]*ref_U/U_coef[0];
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
    boost1_val_delta=analogRead(A15)*2;
    pwm.setPWM(pwm_boost1,0,boost1_val);
    n_rpm=500+analogRead(A14)/5;
   if ((boost1_val!=boost1_val_delta)||(n_rpm_delta!=n_rpm)) {
      boost1_val=boost1_val_delta;
      n_rpm_delta=n_rpm;
      sprintf(str, "BOOST_VAL-%04d  N_RPM-%4d   ", boost1_val, n_rpm);
      myGLCD.print(str, 16*1, 280);
   }
   if (not(digitalRead(pin_B))){
      started=true;
      accelerated=false;
      runned=false;
      start_error=false;
      OCR2A=OCR2A_v;
      acc_charge=true;
      digitalWrite(ps_out, 1);
      pump_cicle=0;
   }
   
    if (temps[i_temp_head1]>temp2start){
      ready2start=true;
    }else{
      ready2start=false;
    }
    if (tiks_int>500){
        //c_tiks_flag=false;
        tiks_flag=0;                                
        tiks_int=0;
        if(rpms[averaging]!=0){
          set_zero(rpms, averaging);
        }
        //rpms[rpms_pos]=0;
        //rpms_pos++;
        //if(rpms_pos>=averaging){
        //  rpms_pos=0;
        //}
        
    }
    //if(tiks_flag){
        //tiks_flag=0;
        //set_rpms();
    //}
    set_rpms();
    set_amperage(analogRead(A0));

    if (rpms[averaging]>350){
      fan_en=true;
      acc_dis=true;
    }else{
      if (rpms[averaging]<230){
        acc_dis=false;
      }
      if (rpms[averaging]<250){
        fan_en=false;
      }
    }

    if (runned and fan_en){  
        if(acc_charge){    
          if ((float)U_val[1]*ref_U/U_coef[1]<28.5){
            digitalWrite(10,0);
          }else{
            digitalWrite(10,1);
            acc_charge=false;
            acc_ch_h=mhhc+1;
            acc_ch_m=mhmc;
          }
        }else{
          if ((acc_ch_h<mhhc)||(acc_ch_h==mhhc&&acc_ch_m<mhmc)){
            acc_charge=true;
          }
        }
        load_pwm_val+=(rpms[averaging]-n_rpm);
        if (load_pwm_val>4094){
            load_pwm_val=4094;
        }
        if (load_pwm_val<0){
            load_pwm_val=0;
        }
        if (temps[i_temp_cool_in]<40){
        max_fan_pwm_val=768;
        }else{
          if(temps[i_temp_cool_in]<43){
        
        max_fan_pwm_val=1024;}else{
             max_fan_pwm_val=1280;
        }
        }
        fan_pwm_val+=50*(temps[i_temp_cool_in]-35);
        if (fan_pwm_val>max_fan_pwm_val){
            fan_pwm_val=max_fan_pwm_val;
        }
        if (fan_pwm_val<384){
            fan_pwm_val=384;
        }
        pwm.setPWM(pwm_fan, 0, fan_pwm_val);
        pwm.setPWM(pwm_load, 0, load_pwm_val);
    }else{
        digitalWrite(10,1);
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

    //digitalWrite(ps_out, accelerated*digitalRead(ps_in)*acc_dis);
    
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
            pwm.setPWM(pwm_rele, 0, 4096);
            OCR2A=OCR2A_v;
        }else{
            if (OCR2A<0xF0&&not(accelerated)){
                OCR2A+=5;
            }
        }
    }else{
      if (rpms[averaging]>200){
        runned=true;
        accelerated=true;
        digitalWrite(ps_out, 0);
      }
    }
    shnek_e_v=analogRead(A8)/16;
    shnek_d_v=analogRead(A9)/8;
    if ((shnek_e_v!=shnek_e_delta) || (shnek_d_v!=shnek_d_delta)) {
      sprintf(str, "SNEK_EN-%02d  SHNEK_DIS-%02d  ", shnek_e_v, shnek_d_v);
      myGLCD.print(str, 16*1, 260);
      shnek_e_delta=shnek_e_v;
      shnek_d_delta=shnek_d_v;
    }
    
    if (accelerated||accelerating||started){
    if(not(shnek_v)){
      if (shnek_ce>shnek_e_delta){
        shnek_cd=0;
        shnek_v=1;
      }
    }else{
      if (shnek_cd>shnek_d_delta){
        shnek_ce=0;
        shnek_v=0;
      }
    }
    }else{
       shnek_v=1;
    }
    
    if (accelerated||accelerating){ 
      digitalWrite(pin_pump,0);
      pwm.setPWM(pwm_pump, 4096, 0);
      pump_cicle=0;
      //pwm.setPWM(pwm_boost1,0,boost1_val);
      //pwm.setPWM(pwm_boost1, 4096, 0);
        if (rpms[averaging]==0){
            tiks2start_error++;
            if (tiks2start_error>70){ //should be refactored
                pwm.setPWM(pwm_rele, 0, 4096);
                OCR2A=OCR2A_v;
                accelerating=false;
                accelerated=false;
                runned=false;
                started=false;
                start_error=true;
                tiks2start_error=0;
                save_mh();
                digitalWrite(ps_out,1);
            }
        }else{
            tiks2start_error=0;
        }
    }else{
        //pwm.setPWM(pwm_boost1, 0, 4096);
        if (not started){
            pump_cicle++;
        }
        //pwm.setPWM(pwm_pump, 0, 4096);
        //digitalWrite(11,1);
    }
Serial.println(pump_cicle);
    if (pump_cicle>2000){
      pwm.setPWM(pwm_pump, 0, 4096);
      digitalWrite(pin_pump,1);
      digitalWrite(ps_out,0);
    }
    //myGLCD.print("   ", 16*5, 20);
    switch (i_line){
    case 0:
      myGLCD.printNumF((float)U_val[0]*ref_U/U_coef[0], 2, 16*4, 25);
    //myGLCD.print("   ", 16*14, 20);
      myGLCD.printNumF((float)U_val[1]*ref_U/U_coef[1], 2,16*13, 25);
    //myGLCD.print("   ", 16*25, 25);
      myGLCD.printNumF(U_val[2]*ref_U/U_coef[2], 2, 16*24, 25);
    break;
    case 1:
      myGLCD.printNumF(U_val[3]*ref_U/U_coef[3], 2, 16*4, 47);
      myGLCD.printNumF(U_val[4]*ref_U/U_coef[4], 2, 16*13, 47);
      myGLCD.printNumF(U_val[5]*ref_U/U_coef[5], 2, 16*24, 47);
    break;
    case 2:
      myGLCD.printNumF((float)(amperages[averaging]-507)*100/512, 2, 16*3, 69);
      myGLCD.printNumF(power, 2, 16*13, 69);
      myGLCD.printNumF(energy_ws/3600, 2, 16*24, 69);
    break;
    case 3:
      sprintf(str, "TH1-%03d  TH2-%03d   ", temps[i_temp_head1], temps[i_temp_head2]);
      myGLCD.print(str, 16*0, 91);
    break;
    case 4:
      sprintf(str, "TC1-%03d  TC2-%03d  TC3-%03d    ", temps[i_temp_cool_in], temps[i_temp_cool_out], temps[4]);
      myGLCD.print(str, 16*0, 113);
    break;
    case 5:
      sprintf(str, "TN1-%03d  TN2-%03d  TN3-%03d    ", temps[5], temps[6], temps[7]);
      myGLCD.print(str, 16*0, 135);
    break;
    case 6:
      sprintf(str, "TN4-%03d  TN5-%03d    ", temps[8], temps[9]);
      myGLCD.print(str, 16*0, 157);
    break;
    case 7:
      sprintf(str, "RPM-%04d  STATE-%d%d%d%d%d ",rpms[averaging], ready2start, started, accelerated,runned, start_error);
      myGLCD.print(str , 1 , 179);   
    break;
    case 8:
      sprintf(str, "MH current - %05d:%02d:%02d", mhhc, mhmc, mhsc);
      myGLCD.print(str, 16*1, 201);
    break;
    case 9:
      sprintf(str, "MH in total - %05d:%02d:%02d", mhht, mhmt, mhst);
      myGLCD.print(str, 16*1, 223);
    break;
    }
    i_line++;
    if (i_line>9){
      i_line=0;
    }
    //char t4_text[30];
    //sprintf(t4_text, "t4- %03d", temps[3]);
    //myGLCD.print(t4_text, 16*1, 223);
    Serial.println(boost1_val);

}
