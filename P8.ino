#include <ModbusSerial.h>
const int TxenPin     = 2;
unsigned int Baudrate=57600;
byte SlaveId = 11;
ModbusSerial mb (Serial, SlaveId, TxenPin); 

byte U1v = 36; //36;
byte U2v = 30; //30;
byte volt = U1v; 


const int RED       = 10;
const int GREEN     = 11;
const int BLUE      = 12;

const int wSine     = 0b0000000000000000;
const int wTriangle = 0b0000000000000010;
const int wSquare   = 0b0000000000101000;
int waveType = wSine;
 
const int SG_fsyncPin = 3;
const int SG_CLK      = 4;
const int SG_DATA     = 5;
const int CS          = 6;


byte raw_fr_count=0;
int correction[60];
unsigned int raw_adc = 0;
unsigned int f,t,prev_maxx,maxx=1,prev=0;
int frequency_shift = 250;    //прыжок частоты после определения пика 
float k,fr=0;
byte debug = 1;  //отладка 0-без вывода. 1-график. 2-текст в виде цифр частот
unsigned long t1,ts=0;
int n=0;
bool z=1,cal_pass=0;//  Г2Д   Г5Д    Г8Д
word f_min=32900;  //31700  32900  36500
word f_max=36500;  //32900  36500  39700
byte reg_counter = 0;
bool brk=0;
unsigned int brk_count=0;

//-----------------------------------------------------------------------------
//10^y
//-----------------------------------------------------------------------------
unsigned long Power(int y) {
  unsigned long t = 1;
  for (byte i = 0; i < y; i++)
    t = t * 10;
  return t;
}
//-----------------------------------------------------------------------------
// Процедура записи в регистры потенциометра
//-----------------------------------------------------------------------------
void Pot_WriteRegister(word data) {
       for (byte b=1; b<=8; b++) 
      {
        digitalWrite(SG_DATA, (data >> (8-b)) & 1 ? HIGH : LOW);
        digitalWrite(SG_CLK, HIGH);
        digitalWrite(SG_CLK, LOW);
       }
     }
//-----------------------------------------------------------------------------
// отправляет в потенциометр значение ползунка
//-----------------------------------------------------------------------------
void MCP4xxxxWrite(byte val) {                 
    digitalWrite(SG_CLK, LOW);
    digitalWrite(CS, LOW);
    Pot_WriteRegister(0b00010001);
    Pot_WriteRegister(val);    
    digitalWrite(CS,HIGH);                    
}
//-----------------------------------------------------------------------------
// Процедура записи в регистры
//-----------------------------------------------------------------------------
void SG_WriteRegister(word dat) {
  digitalWrite(SG_CLK, LOW); digitalWrite(SG_CLK, HIGH); digitalWrite(SG_fsyncPin, LOW);
  for (byte i = 0; i < 16; i++) {
    if (dat & 0x8000)
      digitalWrite(SG_DATA, HIGH);
    else
      digitalWrite(SG_DATA, LOW);
    dat = dat << 1;
    digitalWrite(SG_CLK, HIGH);
    digitalWrite(SG_CLK, LOW);
  }
  digitalWrite(SG_CLK, HIGH);
  digitalWrite(SG_fsyncPin, HIGH);
}
//-----------------------------------------------------------------------------
// процедура сброса
//-----------------------------------------------------------------------------
void SG_Reset() {
  delay(100);
  SG_WriteRegister(0x100);
  delay(100);
}
//-----------------------------------------------------------------------------
// Установка частоты/модуляции со сбросом регистров
//-----------------------------------------------------------------------------
void SG_freqReset(long frequency, int wave) {
  long fl = frequency * 33.554432;//10.73741824; //33-8mhz/10.7 - 25mhz 
  SG_WriteRegister(0x2100);
  SG_WriteRegister((int)(fl & 0x3FFF) | 0x4000);
  SG_WriteRegister((int)((fl & 0xFFFC000) >> 14) | 0x4000);
  SG_WriteRegister(0xC000);
  SG_WriteRegister(wave);
  waveType = wave;
  MCP4xxxxWrite(volt);
}
//-----------------------------------------------------------------------------
// Установка частоты/модуляции/напряжения 
//-----------------------------------------------------------------------------
void SG_freqSet(float frequency, int wave) {
  long fl = frequency * 33.554432;//10.73741824;  //33-8mhz/10.7 - 25mhz 
  SG_WriteRegister(0x2000 | wave);
  SG_WriteRegister((int)(fl & 0x3FFF) | 0x4000);
  SG_WriteRegister((int)((fl & 0xFFFC000) >> 14) | 0x4000);
  MCP4xxxxWrite(volt);
}
//-----------------------------------------------------------------------------
// Точное ADC 
//-----------------------------------------------------------------------------
unsigned int adc(void){unsigned int raw=0; for (int i=0;i<32;i++) {if(Serial.available()) {brk=1;break;}raw = raw + analogRead(A0);delay(2);}raw = int(raw/32); return raw;}
//unsigned int adc(void){unsigned int raw=0; for (int i=0;i<32;i++) {raw = raw + analogRead(A0);delay(2);}raw = int(raw/32); return raw;}
//-----------------------------------------------------------------------------
// Точный поиск
//-----------------------------------------------------------------------------

void slowSearch(byte cell_number)
{
float floatValue = convertToFloat(mb.Ireg(cell_number), mb.Ireg(cell_number+1));
fr = floatValue - 9; // 15Гц отскок частоты от предыдущего значения
float fine_fr=0;
byte count_avr = 1;
if (correction[cell_number]>16) correction[cell_number]=16;
if (correction[cell_number]<-15) correction[cell_number]=-15;
volt = U1v + correction[cell_number];
digitalWrite(GREEN,1);
brk=0;
        while(1)
              { SG_freqSet(fr+k, waveType);
                while (brk_count<20)
                {
                  delay(1);
                  brk_count++;
                  if(Serial.available()) {brk=1;break;}
                }
                  brk_count=0;
                if (analogRead(A0)>330) t=adc();
                else t= ADC_delay_micros(400);
                if(Serial.available()) {brk=1;break;}         

                if (t>350){
                  if (debug==1) Serial.println(t);
                  floatValue = floatValue+fr+k; count_avr++;} 
                if (t>maxx) {fine_fr=fr+k; maxx = t;}
                if ((maxx>250) && (maxx-t>350)) break;
                if ((maxx>250) && (t<100)) break;
                if ((maxx>250) && (t==0)) break;
                if (k>100) break;
                if (t>100)k=k+0.03;
                if ((t>1)&&(t<101))k=k+0.09;
                if (t<2)k=k+0.6;
}
             
             if(!brk) SG_freqSet(fr+300, waveType);
             while (brk_count<500)
                {
                  delay(1);
                  brk_count++;
                  if(Serial.available()) {brk=1;break;}
                }
             brk_count=0;
          if (maxx>150 && maxx<300 && !brk)correction[cell_number]+=2; 
          if (maxx>150 && maxx<400 && !brk)correction[cell_number]++;               
          if (maxx>150 && maxx <530&& !brk )correction[cell_number]++;
          if (maxx>590 && !brk )correction[cell_number]--;
          if (maxx>630 && !brk )correction[cell_number]--;
            
          
          
          maxx = 0;
          k = 0;
          if(count_avr>8) fine_fr = floatValue/count_avr;
          floatValue=0;
          count_avr=0;          
          if (!brk){

                 float oldValue = convertToFloat(mb.Ireg(cell_number), mb.Ireg(cell_number+1));
        int diff = oldValue-fine_fr;
         if (abs(diff)<100) {
          uint32_t floatBits = *(uint32_t*)&(fine_fr); 
          mb.Ireg(cell_number,((floatBits >> 16) & 0xFFFF)); 
          mb.Ireg(cell_number+1,floatBits & 0xFFFF);
         }
          }
          }               

//-----------------------------------------------------------------------------
// Sweep           если гирлянда слабая - вспышки красного. если чуть недотягивает вспышки синего. если датчик зацепился загорается зеленый - по вспышкам зеленого можно считать кол-во датчиков. 
//-----------------------------------------------------------------------------
void Sweep() {
   raw_fr_count=0;
   unsigned int i = 0;
   volt = U2v;
   while(f_min+i < f_max)
           {    SG_freqSet(f_min+i, waveType);
                raw_adc =0;
                for (int m = 0; m < 5; m++) { raw_adc = raw_adc + analogRead(A0); delayMicroseconds(1500); }
                raw_adc =  raw_adc /5;
                if (raw_adc>5 && raw_adc<30) digitalWrite(RED,1);
                if (raw_adc>30 && raw_adc<60){ digitalWrite(RED,0);digitalWrite(BLUE,1);}
                if (debug==1)  Serial.println(raw_adc);// Serial.println(raw_adc); //Раскомментировать для отладки
                if((raw_adc > 59) && (millis() - t1 >400))
                                                         { float ff = f_min+i-8;
                                                           uint32_t floatBits = *(uint32_t*)&(ff); 
                                                           mb.Ireg(raw_fr_count,((floatBits >> 16) & 0xFFFF)); 
                                                           mb.Ireg(raw_fr_count+1,floatBits & 0xFFFF);
                                                           float floatValue = convertToFloat(mb.Ireg(raw_fr_count), mb.Ireg(raw_fr_count+1));
                                          if (debug==2)    Serial.print(floatValue);
                                          if (debug==2)    Serial.print("|");
                                                           raw_fr_count += 2;   
                                                           i=i+frequency_shift;
                                                           SG_freqSet(f_min+i, waveType);
                                                           digitalWrite(BLUE,0);digitalWrite(GREEN,1);
                                                           delay(500);
                                                           t1=millis();
                                                           digitalWrite(GREEN,0);
                                                         }
           i++;
           }
   SG_Reset();
   raw_fr_count=0;
   Serial.println("|");
   }
//-----------------------------------------------------------------------------
// Инициализация генератора
//-----------------------------------------------------------------------------

void InitSigGen(void) {
  digitalWrite(SG_fsyncPin, HIGH);
  digitalWrite(SG_CLK, HIGH);
  SG_Reset();
  SG_freqReset(f_min, waveType);
  delay(100);
  MCP4xxxxWrite(volt);
}
 
//-----------------------------------------------------------------------------
// ADC
//-----------------------------------------------------------------------------

unsigned int ADC_delay_micros(unsigned int miks)
{unsigned int adc_d=0; for (int m = 0; m < 10; m++) {adc_d=adc_d+analogRead(A0); delayMicroseconds(miks); }adc_d = adc_d/10; return adc_d;}

//-----------------------------------------------------------------------------
// Калибровка
//-----------------------------------------------------------------------------
 void calibr()    
{     fr=f_min;
      float cal_i=0;
      unsigned int porog = 300;// порог 80
      unsigned int search_break_counter = 0;
      bool search_fail = 1;

 while(search_fail)
 {     
      while(1)
           {  
              fr++;
              search_break_counter++;
              if (search_break_counter>600) // поиск первого датчика.если прочесали 600 герц и не нашли - возвращаемся на Fmin
              {search_fail=1;
              search_break_counter=0;
               fr=f_min;
                break;
              }
              SG_freqSet(fr, waveType);
              raw_adc = ADC_delay_micros(1000);  
              if (raw_adc>50) {digitalWrite(BLUE,1); raw_adc = ADC_delay_micros(2000); digitalWrite(BLUE,0);}
              if (debug==1) Serial.println(raw_adc);
                           
              if (raw_adc>porog){
                search_fail=0;
                break;
                }
              
           }
           
  porog = porog-50;
 }

                
        // fr=fr-3;
         
        while(1)
           {  fr=fr-cal_i;
              SG_freqSet(fr, waveType);
              raw_adc = ADC_delay_micros(2000);
              delay(20);//
              if(raw_adc>300)delay(80);  
          
              if (debug==1) Serial.println(raw_adc);
               //Serial.print(',');
             //Serial.println(maxx);
              if (raw_adc > prev) {maxx = raw_adc; prev = raw_adc;}
              if ((maxx>400)&& (raw_adc<200)) break;
              if (raw_adc>200)cal_i=0.03;
              if (raw_adc<201)cal_i=0.1;            
            }
           raw_adc=0;prev=0;maxx=0;
           SG_freqSet(fr, waveType);
           delay(100);
          // f_min=fr-250;
    
     
        volt = 28;   //26     
      while(raw_adc<450){SG_freqSet(fr, waveType);volt=volt+1;raw_adc=ADC_delay_micros(5000);
if (debug==1) Serial.println(raw_adc);
      delay(50);
      }
      if (volt>55)volt=55;
      U2v=volt;U1v=volt-27;
    }
//-----------------------------------------------------------------------------
// 2x16bit>>32bitfloat
//-----------------------------------------------------------------------------
float convertToFloat(uint16_t word1, uint16_t word2) {
    uint32_t combined = (uint32_t)word1 << 16 | word2;
    union {
        uint32_t i;
        float f;
    } converter;
    converter.i = combined;
    return converter.f;
}
//-----------------------------------------------------------------------------
// Setup
//----------------------------------------------------------------------------- 
void setup (void) 
{  
  analogReference(INTERNAL4V096);
  //analogReference(INTERNAL2V048);
  pinMode(13, OUTPUT);pinMode(SG_DATA, OUTPUT);pinMode(SG_CLK, OUTPUT);pinMode(SG_fsyncPin, OUTPUT);pinMode(CS, OUTPUT);pinMode(RED, OUTPUT);pinMode(GREEN, OUTPUT);pinMode(BLUE, OUTPUT); 
  pinMode(A1,INPUT_PULLUP);pinMode(A2,INPUT_PULLUP);pinMode(A3,INPUT_PULLUP);pinMode(A4,INPUT_PULLUP);pinMode(A5,INPUT_PULLUP);
  pinMode(A6,INPUT_PULLUP); pinMode(A7,INPUT_PULLUP);

  //SlaveId |= digitalRead(A1) << 0;SlaveId |= digitalRead(A2) << 1;SlaveId |= digitalRead(A3) << 2;SlaveId |= digitalRead(A4) << 3;SlaveId |= digitalRead(A5) << 4;
  if(debug==2){Serial.print("ID = ");Serial.print(SlaveId);Serial.println('!');}
  
  Serial.begin(Baudrate); 
  mb.config (Baudrate);
   
  for(int n=0;n<120;n++) mb.addIreg(n); //Формируем таблицу регистров для Modbus.
  for(int n=0;n<10;n++) mb.addHreg(n); //Формируем таблицу регистров для Modbus.
  for(int n=0;n<60;n++)correction[n]=0;
  mb.Hreg(1,f_min);
  mb.Hreg(2,f_max);
  //mb.Hreg(0,1); 
  InitSigGen();// запускаем генерацию синуса на частоте f_min.
}

void loop() 
{ 
mb.task();
 
  if(mb.Hreg(1)!= f_min) {f_min = mb.Hreg(1);InitSigGen();}
  if(mb.Hreg(2)!= f_max) {f_max = mb.Hreg(2);}
    if (mb.Hreg(8)){   //калибровка (мигание светодиодом) если в регистр управления 8 записать '1'
     ts=millis();
     while(millis()-ts<20000){ // установка времени ручной подстройки конденсаторов балансного плеча трансформатора в мс
     if (debug==1)Serial.println(analogRead(A0));
     if ((analogRead(A0))>1) {digitalWrite(RED,1); delay(3);digitalWrite(RED,0);}
     }  //- подстройка конденсатора
     mb.Hreg(8,0);
     }
 
while(mb.Hreg(0)){
mb.task();
if(mb.Hreg(9)) { cal_pass=0; 
     for(int f=0;f<6;f++) {
      digitalWrite(BLUE,!digitalRead(BLUE)); 
       delay(500);} 
         mb.Hreg(9,0);}
   
   if(!cal_pass){volt = 36;InitSigGen();delay(500);digitalWrite(RED,1); calibr(); InitSigGen(); delay(500); Sweep(); delay(500); cal_pass=1; digitalWrite(RED,0);}
    
     if ((mb.Ireg(reg_counter)) && z){
       slowSearch(reg_counter);
        if(debug==2)
         {float floatValue = convertToFloat(mb.Ireg(reg_counter), mb.Ireg(reg_counter+1));
           Serial.print(floatValue);
            Serial.print("|");
             }
      }
     reg_counter+=2; 
     if (reg_counter>118) {
     if(debug==2) Serial.println('!'); 
     reg_counter=0;}
      
}
digitalWrite(GREEN,0);   
    //if(mb.Hreg(3)!= volt)  {volt =  mb.Hreg(3); InitSigGen();}
}