//#include <avr/pgmspace.h>

//#define Pin2Input(port,pin){port&=~(1<<pin);}
//#define Pin2Output(port,pin){port|=(1<<pin);}
//#define Pin2HIGH(port,pin){port|=(1<<pin);}
//#define Pin2LOW(port,pin){port&=~(1<<pin);}
//#define NOP __asm__ __volatile__ ("nop\n\t")






extern uint32_t long timer0_millis;
uint32_t milli;
uint8_t HOUR=0;
uint16_t vcc;

void (* reboot) (void) = 0; //declare reset function @ address 0

#define FanON { __asm__ __volatile__("sbi 11,0\n\t"); }
#define FanOFF { __asm__ __volatile__("cbi 11,0\n\t"); }

#define ADCon{ PRR&=~(1<<PRADC); ADCSRA|=(1<<ADEN); }
#define ADCoff{ ADCSRA&=~(1<<ADEN);   PRR|=(1<<PRADC);}
#define SerialON{ PRR&=~(1<<PRUSART0); Serial.begin(9600); }
#define SerialOFF{ Serial.end(); PRR|=(1<<PRUSART0);}

#define SetADC(bandgap,input,us){ ADCSRA|=(1<<ADEN);delayMicroseconds(2);ADMUX=(bandgap<<REFS1)|(1<<REFS0)|(0<<ADLAR)|input;delayMicroseconds(us);} // input (0..7,8,14) (bg/vcc analogReference )
#define mRawADC(v,p) { ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|p;do{}while(bit_is_set(ADCSRA,ADSC));v=(ADCH<<8)|ADCL; }
//#define mRawADC2(h,l,p) { ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|p;do{}while(bit_is_set(ADCSRA,ADSC));l=ADCL;h=ADCH;}
//#define mRawADC(v,p) ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|p;do{}while(bit_is_set(ADCSRA,ADSC));v=ADCW; 

byte volatile WDsleep;
byte volatile WDflag;

// Watchdog timeout values : //#define T16MS 0//#define T32MS 1//#define T64MS 2//#define T128MS 3//#define T250MS 4//#define T500MS 5//#define T1S 6
#define T2S 7//#define T4S 8//#define T8S 9
#define setup_watchdog(timeout){cli(); __asm__ __volatile__("wdr\n\t"); MCUSR&=~(1<<WDRF);WDTCSR|=(1<<WDCE)|(1<<WDE);WDTCSR=((1<<WDIE)|timeout);sei();}
 
ISR(WDT_vect) { 
if(WDsleep){WDflag=1; WDsleep=0; } else{ reboot();}
//__asm__ __volatile__("call 0\n\t");
} // Watchdog timer interrupt

/*
void CheckPowerSupply(void)
{
        
      ADCon;
      ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);

      cli();milli=timer0_millis;sei();  
      HOUR = milli/3600000L;
       if (HOUR>=24){
     cli();timer0_millis-=86400000L;sei();  
     HOUR=0;
//         reboot();-не работает
     } 

//      SerialON;

  delayMicroseconds(900);  // Wait for Vref to settle
  ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|2;  while (bit_is_set(ADCSRA,ADSC)); vcc = 1120300L / ADCW;  ADCoff;
  
  if (vcc<4600){  __asm__ __volatile__("sbi 5,5\n\t"); } else { __asm__ __volatile__("cbi 5,5\n\t"); }


//  Serial.println(milli);
  //Serial.println(HOUR);
//   Serial.println(tz);
   //Serial.println(t);
   }
*/


void setup() 
{  

  delay(2500); 
  //SerialON;  Serial.println("Start"); delay(500);  SerialOFF;

  ACSR|=(1<<ACD);// analog comparator off

//  PRR|=(1<<PRTWI)|(1<<PRTIM2)|(1<<PRTIM1)|(1<<PRSPI)|(1<<PRUSART0);
  PRR|=(1<<PRTWI)|(1<<PRTIM2)|(1<<PRTIM1)|(1<<PRSPI)|(1<<PRADC);

//  setup_watchdog(T2S); // если в течении 2s не сбросить сторожевого пса то перезагрузка. (защита от зависаний)
    
  
//    Pin2Output(DDRD,0);
//    Pin2Output(DDRD,1);
//    Pin2Output(DDRD,2);
//    Pin2Output(DDRD,3);
//    Pin2Output(DDRD,4);
//    Pin2Output(DDRD,5);
    
//    Pin2LOW(PORTD,0);
//    Pin2LOW(PORTD,1);
//    Pin2LOW(PORTD,2);
//    Pin2LOW(PORTD,3);
//    Pin2LOW(PORTD,4);
//    Pin2LOW(PORTD,5);

  //  SetADC(1,8,500);  //  select temperature sensor 352
    
    // initial  hour  settings
    
  //  cli();timer0_millis=0L;sei();    // 12 ночи
   // cli();timer0_millis=3600000L;sei();    // 1 ночи
  //  cli();timer0_millis=7200000L;sei();    // 2 ночи

 //   cli();timer0_millis=21600000L;sei();    // 6 утра

//    cli();timer0_millis=28795000L;sei();    // почти 8 утра

  //  cli();timer0_millis=36000000L;sei();    // 10 утра
//   cli();timer0_millis=39600000L;sei();    // 11 утра
//    cli();timer0_millis=43200000L;sei();    // полдень
 cli();timer0_millis=46800000L;sei();    // час дня


//    cli();timer0_millis=50400000L;sei();    // 2 часа дня
  //  cli();timer0_millis=54000000L;sei();    // 3 часа дня
  //  cli();timer0_millis=57600000L;sei();    // 4 часа дня
//    cli();timer0_millis=61200000L;sei();    // 5 вечера
//    cli();timer0_millis=64780000L;sei();    // почти 6 вечера
 //  cli();timer0_millis=64800000L;sei();    // 6 вечера

   // cli();timer0_millis=71000000L;sei();    // почти 8 вечера
//    cli();timer0_millis=68400000L;sei();    // 7 вечера

//    cli();timer0_millis=72000000L;sei();    // 8 вечера
//    cli();timer0_millis=75600000L;sei();    // 9 вечера
//    cli();timer0_millis=78500000L;sei();    // почти 10 вечера
//    cli();timer0_millis=79200000L;sei();    // 10 вечера
//    cli();timer0_millis=82700000L;sei();    // почти 11 вечера
//    cli();timer0_millis=82800000L;sei();    // 11 вечера
//   cli();timer0_millis=86395000L;sei();    // почти полночь
// cli();timer0_millis=86000000L;sei();    // почти полночь


 //CheckPowerSupply();CheckPowerSupply();
}


void  delay500ns(void) __attribute__((noinline)); 
void  delay500ns(void) { __asm__ __volatile__( "delay500:\n\t"    // "ret\n\t"
);} 
void  delay750ns(void) __attribute__((noinline)); 
void  delay750ns(void) {  __asm__ __volatile__( "delay750:\n\t"   "nop\n\t""nop\n\t"  "nop\n\t""nop\n\t"  //"ret\n\t"
);} 
void  delay1000ns(void) __attribute__((noinline)); 
void  delay1000ns(void) { __asm__ __volatile__( "delay1000:\n\t" "call delay500\n\t"   // "ret\n\t"
);} 
void  delay2500ns(void) __attribute__((noinline)); 
void  delay2500ns(void) {  __asm__ __volatile__( "delay2500:\n\t" 
"call delay1000\n\t"//"nop\n\t""nop\n\t"  "nop\n\t""nop\n\t"  //750"nop\n\t""nop\n\t"  "nop\n\t""nop\n\t" //1000
"call delay1000\n\t"//"nop\n\t""nop\n\t"  "nop\n\t""nop\n\t""nop\n\t""nop\n\t"  "nop\n\t""nop\n\t" //1500
//"ret\n\t"
);} 



void delay10500ns(void){__asm__ __volatile__( "delay10500:\n\t" 
"call delay2500\n\t"  "call delay2500\n\t"  "call delay2500\n\t"  "call delay2500\n\t"   
//"ret\n\t" 
); } // 500+10000=10500ns total delay

void delay21500ns(void){__asm__ __volatile__( "delay21500:\n\t" 
"call delay10500\n\t"  "call delay10500\n\t" 
//"ret\n\t" 
); } // 500+21000=21500ns total delay

void delay24000ns(void){__asm__ __volatile__( "delay24000:\n\t" 
"call delay10500\n\t"  "call delay10500\n\t"
"call delay2500\n\t"
//"ret\n\t" 
); } // 500+21000+2500=24000ns total delay

void delay26500ns(void){__asm__ __volatile__( "delay26500:\n\t" 
"call delay10500\n\t"  "call delay10500\n\t"
"call delay2500\n\t""call delay2500\n\t"
//"ret\n\t" 
); } // 500+21000+5000=26500ns total delay

void delay32000ns(void){__asm__ __volatile__( "delay32000:\n\t" 
"call delay10500\n\t"  "call delay10500\n\t" "call delay10500\n\t" 
//"ret\n\t" 
); } // 500+315000=32000ns total delay

void delay86500ns(void){__asm__ __volatile__( "delay86500:\n\t" 
"call delay21500\n\t"  "call delay21500\n\t" "call delay21500\n\t"  "call delay21500\n\t" 
//"ret\n\t" 
); } // 500+86000=86500ns total delay

void delay43500ns(void){__asm__ __volatile__( "delay43500:\n\t" 
"call delay21500\n\t"  "call delay21500\n\t" 
//"ret\n\t" 
); } // 500+43000=43500ns total delay

void delay173500ns(void){__asm__ __volatile__( "delay173500:\n\t" 
"call delay86500\n\t"  "call delay86500\n\t" 
//"ret\n\t" 
); } // 500+86500+86500=173500ns total delay


void delay5500ns(void){__asm__ __volatile__( "delay5500:\n\t" 
"call delay2500\n\t"  "call delay2500\n\t"  
"ret\n\t" ); } // 500+5000=5500ns total delay

void delay3000ns(void){__asm__ __volatile__( "delay3000:\n\t" 
"call delay2500\n\t" 
"ret\n\t" ); } // 500+2500=3000ns total delay



void C235(void) // ночная смена. выводы С2 С4 С5
{
    DDRC=0b11111111; // set C pins to output

    __asm__ __volatile__(
    "ldi r20,0b00000100\n\t"      // C2  +++ -- ---====
    "ldi r22,0b00010000\n\t"      // C4  --+ ++ ---====
    "ldi r23,0b00100000\n\t"      // C5  --- -+ ++-==== +call delay
  //  "lds r30,N\n\t"
    //"lds r31,N+1\n\t"
//    "ldi r30,0xff\n\t"
  //  "ldi r31,0xff\n\t"
    
    "lds r30,StartRuns\n\t" // start runs
    "lds r31,Runs\n\t" // 255 runs

     
    "ldi r18,0\n\t"
    "mov r1,r18\n\t" // r1=0
    "ldi r25,0b00110000\n\t"
"555:\n\t" 
"out 8,r20\n\t" // С2                   
    "ldi r26,0b00010100\n\t"
      "out 8,r26\n\t" // С2&С4
"out 8,r22\n\t"//  С2 OFF С4 ON
      "out 8,r25\n\t" // С4&C5
"out 8,r23\n\t"// C4 OFF C5 ON
//"nop\n\t"
   "dec r30\n\t" // decrease current runs counter
      "out 8,r1\n\t" // port C OFF
//  "call delay2500\n\t"
  "call delay1000\n\t"
  "call delay1000\n\t"
   "brne 555b\n\t"  // 1 clock if not taken (false)

   "dec r31\n\t"
"breq 111f\n\t"  // выход
"call delay32000\n\t" // задержка 32мкс
    "lds r30,NextRuns\n\t" // next runs between pause: 11x3.6мкс потом пауза 32мкс
    "rjmp 555b\n\t"
"111:\n\t"

      
      
//"call delay1000\n\t" 
//"call delay1000\n\t" 
  //    "adiw r30,1\n\t"
  //    "sbiw r30,1\n\t"
//   "brne 555b\n\t"  // 2 clocks if taken 
      );   
//  if (N<65532){N+=1;}
} // включение 1 такт 62.5нс свет - 2 такта - 125нс



byte StartRuns=25;
byte NextRuns=11;
byte Runs=255;

void Light(void) 
{
    DDRD=0b11111111; // set D pins to output
    DDRB=0b11111111; // set B pins to output
    DDRC=0b11111111; // set C pins to output

    __asm__ __volatile__(
    "ldi r18,0\n\t"
    "mov r1,r18\n\t" // r1=0
    "ldi r18,0b00000001\n\t"
    "ldi r19,0b00000010\n\t"
    "ldi r20,0b00000100\n\t"
    "ldi r21,0b00001000\n\t"
    "ldi r22,0b00010000\n\t"
    "ldi r23,0b00100000\n\t"
    "ldi r24,0b01000000\n\t"
    "ldi r25,0b10000000\n\t"
    "lds r30,StartRuns\n\t" // start runs
    "lds r31,Runs\n\t" // 255 runs    
"555:\n\t" 
"out 11,r20\n\t" // D2
    "ldi r26,0b00001100\n\t"
      "out 11,r26\n\t" // D2&D3
"out 11,r21\n\t"//  D2 OFF D3 ON
    "ldi r26,0b00011000\n\t"
      "out 11,r26\n\t" // D3&D4
"out 11,r22\n\t"// D3 OFF D4 ON
    "ldi r26,0b00110000\n\t"
      "out 11,r26\n\t" // D4&D5
"out 11,r23\n\t"// D4 OFF D5 ON
    "ldi r26,0b01100000\n\t"
      "out 11,r26\n\t" // D5&D6
"out 11,r24\n\t"// D5 OFF D6 ON
    "ldi r26,0b11000000\n\t"
      "out 11,r26\n\t" // D6&D7
"out 11,r25\n\t"// D6 OFF D7 ON
"nop\n\t"
      "out 5,r18\n\t" // D7&B0
    "ldi r26,0b00000001\n\t"  // 0 is ON (fan)
"out 11,r26\n\t"// PORTD OFF (except fan pin 0)
    "ldi r26,0b00000011\n\t"
      "out 5,r26\n\t" // B0&B1
"out 5,r19\n\t"// B0 OFF B1 ON
    "ldi r26,0b00000110\n\t"
      "out 5,r26\n\t" // B1&B2
"out 5,r20\n\t"// B1 OFF B2 ON
    "ldi r26,0b00001100\n\t"
      "out 5,r26\n\t" // B2&B3
"out 5,r21\n\t"// B2 OFF B3 ON
    "ldi r26,0b00011000\n\t"
      "out 5,r26\n\t" // B3&B4
"out 5,r22\n\t"// B3 OFF B4 ON
    "ldi r26,0b00110000\n\t"
      "out 5,r26\n\t" // B4&B5
"out 5,r23\n\t"// B4 OFF B5 ON
   "nop\n\t"
      "out 8,r18\n\t" // C0&B5
"nop\n\t"
"out 5,r1\n\t"// PORTB OFF
    "ldi r26,0b00000011\n\t"
      "out 8,r26\n\t" // C0&C1
"out 8,r19\n\t"// C0 OFF C1 ON
    "ldi r26,0b000001010\n\t"
      "out 8,r26\n\t" // C1&C3
"out 8,r21\n\t"// C1 OFF C3 ON
"nop\n\t"
   "dec r30\n\t" // decrease current runs counter
"out 8,r1\n\t"// PORTC OFF
   "brne 555b\n\t"  // 1 clock if not taken (false)
   "dec r31\n\t"
"breq 111f\n\t"  // выход
"call delay32000\n\t" // задержка 32мкс
    "lds r30,NextRuns\n\t" // next runs between pause: 11x3.6мкс потом пауза 32мкс
    "rjmp 555b\n\t"
"111:\n\t"
      ); 
}

void NightLight() // работают все порты (+3ночных).
{
    DDRD=0b11111111; // set D pins to output
    DDRB=0b11111111; // set B pins to output
    DDRC=0b11111111; // set C pins to output

    __asm__ __volatile__(
    "ldi r18,0\n\t"
    "mov r1,r18\n\t" // r1=0 (as it should be)
    "ldi r18,0b00000001\n\t"
    "ldi r19,0b00000010\n\t"
    "ldi r20,0b00000100\n\t"
    "ldi r21,0b00001000\n\t"
    "ldi r22,0b00010000\n\t"
    "ldi r23,0b00100000\n\t"
    "ldi r24,0b01000000\n\t"
    "ldi r25,0b10000000\n\t"
    "lds r30,StartRuns\n\t" // start runs
    "lds r31,Runs\n\t" // number of runs
"555:\n\t" 
"out 11,r20\n\t" // D2
    "ldi r26,0b00001100\n\t"
      "out 11,r26\n\t" // D2&D3
"out 11,r21\n\t"//  D2 OFF D3 ON
    "ldi r26,0b00011000\n\t"
      "out 11,r26\n\t" // D3&D4
"out 11,r22\n\t"// D3 OFF D4 ON
    "ldi r26,0b00110000\n\t"
      "out 11,r26\n\t" // D4&D5
"out 11,r23\n\t"// D4 OFF D5 ON
    "ldi r26,0b01100000\n\t"
      "out 11,r26\n\t" // D5&D6
"out 11,r24\n\t"// D5 OFF D6 ON
    "ldi r26,0b11000000\n\t"
      "out 11,r26\n\t" // D6&D7
"out 11,r25\n\t"// D6 OFF D7 ON
"nop\n\t"
      "out 5,r18\n\t" // D7&B0
"nop\n\t"
"out 11,r1\n\t"// PORTD OFF (even fan pin 0)
    "ldi r26,0b00000011\n\t"
      "out 5,r26\n\t" // B0&B1
"out 5,r19\n\t"// B0 OFF B1 ON
    "ldi r26,0b00000110\n\t"
      "out 5,r26\n\t" // B1&B2
"out 5,r20\n\t"// B1 OFF B2 ON
    "ldi r26,0b00001100\n\t"
      "out 5,r26\n\t" // B2&B3
"out 5,r21\n\t"// B2 OFF B3 ON
    "ldi r26,0b00011000\n\t"
      "out 5,r26\n\t" // B3&B4
"out 5,r22\n\t"// B3 OFF B4 ON
    "ldi r26,0b00110000\n\t"
      "out 5,r26\n\t" // B4&B5
"out 5,r23\n\t"// B4 OFF B5 ON
"nop\n\t"
      "out 8,r18\n\t" // C0
"nop\n\t"
"out 5,r1\n\t"// PORTB OFF
    "ldi r26,0b00000011\n\t"
      "out 8,r26\n\t" // C0&C1
"out 8,r19\n\t"// C0 OFF C1 ON
    "ldi r26,0b00000110\n\t"
      "out 8,r26\n\t" // C1&C2
"out 8,r20\n\t"// C1 OFF C2 ON
    "ldi r26,0b00001100\n\t"
      "out 8,r26\n\t" // C2&C3
"out 8,r21\n\t"// C2 OFF C3 ON
    "ldi r26,0b00011000\n\t"
      "out 8,r26\n\t" // C3&C4
"out 8,r22\n\t"// C3 OFF C4 ON
    "ldi r26,0b00110000\n\t"
      "out 8,r26\n\t" // C4&C5
"out 8,r23\n\t"// C4 OFF C5 ON
"nop\n\t"
"dec r30\n\t"
"out 8,r1\n\t"// PORTC OFF
   "brne 555b\n\t"  // 1 clock if not taken (false)

   "dec r31\n\t"
"breq 111f\n\t"  // выход
"call delay32000\n\t" // задержка 32мкс
    "lds r30,NextRuns\n\t" // next runs between pause: 11x3.6мкс потом пауза 32мкс
    "rjmp 555b\n\t"
"111:\n\t"

      ); 
}




  long milli2;
  long milli3;

void Shine(void)
{
  if ((HOUR>=8)&&(HOUR<=23))
  {
//    if ((HOUR==8)||(HOUR==23)){StageM();}// gradually start/stop hour 
    if ((HOUR==7)||(HOUR==21)){FanOFF;StartRuns=35; NextRuns=3; Runs=255; NightLight();}// gradually start/stop 48w 1700lux (84w 2200lux)
    // хорошая идея микроперерывчики устраивать - блок питания может "собраться с мыслями" и как "пыхнуть"
 //   else if (HOUR==8){StageN8();}// gradually start/stop 
    else if ((HOUR>=8)&&(HOUR<=20)){FanON; StartRuns=55; NextRuns=7; Runs=20; Light();}// gradually start/stop 
//    else if ((HOUR>=9)&&(HOUR<=10)){FanON; SVM();}// gradually start/stop 
//    else if ((HOUR>=11)&&(HOUR<=14)){FanON; SVM();}// gradually start/stop 
//    else if (HOUR==15){FanON; SVM();}// gradually start/stop 
//    else if ((HOUR>=16)&&(HOUR<=19)){FanON; SVM();}// gradually start/stop 
//    else if ((HOUR>=20)&&(HOUR<=22)){FanON; SVM();}// gradually start/stop 
//    else{

      
//  SVL();    // 3360lux 71.5w (46.99) // перемешать паузы в одном 64к пробеге
      
    //   S500(); // 83.7w(42.17) 3530 lux 
//       S3000(); // 74.2w(45.95) 3410 lux
//      S5500(); // 68.5w(48.32) 3310lux 3300 (69.7w 3320)
     // S10500();// 60w(51.33) 3080lux
 //     S20500();// 48.3w(55.27) 2670lux
//      S30500();// 41.3w(57.14) 2360lux
    //  S40500();// 36.3w(57.85) 2100lux
     // S50500();// 32.4w(58.33) 1890 lux
//      S60500();// 29.5w(58.16) 1716 lux
      //StageN();// 84w() (2x + 4.6w)
     
     /* for (word e=0;e<8000;e++)
      {
        StageN7();// 7 3.6us runs
        delay2500ns();
        delay2500ns();
      }*/
    }
  else
{
//  if ((HOUR==0)||(HOUR==7)){C235M();}// gradually start/stop hour 
  //else{
    FanOFF;StartRuns=55; NextRuns=7; Runs=20;  C235();//} //217ns
// milli3=timer0_millis; 
//SerialON;  Serial.println(milli2); Serial.print(" ");  Serial.print(milli3);     Serial.println("<<");   delay(5000);        SerialOFF;
}

//else
 // if (HOUR<=23){
//cli();    milli2=timer0_millis; sei();
  //StageN(0); // 244us 3.72us each of 65536 times
//cli();  milli3=timer0_millis;sei();
//SerialON;  Serial.println(milli2); Serial.print(" ");  Serial.print(milli3);     Serial.print(" ");  Serial.println((milli3-milli2));    delay(1000);        SerialOFF;

//} //16ч //0-256


//  PORTD=0b00000000; // all port D pins to low    
  //  PORTB=0b00000000; // all port B pins to low    
    //PORTC=0b00000000; // all port C pins to low    

//    if ((HOUR>=6)&&(HOUR<18)){Stage1();} //12ч   12345
  //  else if ((HOUR>=18)&&(HOUR<22)){Stage2();}//4ч  12678
    //else {Stage3();} //8ч  1678
}

uint8_t count2s=0;
uint16_t tz,tw;

void loop() {

    __asm__ __volatile__("Start:\n\t");
    __asm__ __volatile__("wdr\n\t");//  wdt_reset();


      cli();milli=timer0_millis;sei();  
      HOUR = milli/3600000L;
       if (HOUR>=24){
     cli();timer0_millis-=86400000L;sei();  
     HOUR=0;
//         reboot();-не работает
     } 

  //ADCon; ADMUX = _BV(REFS0) | _BV(REFS1) | _BV(MUX3);
   //SerialON; // Serial.print("N="); 
  // Serial.println(N);     delay(80);        SerialOFF;
//delay(100);
 //     cli();timer0_millis=0;sei();  
  Shine(); //if (N<65532){
    //214ms
   //   cli();milli=timer0_millis;sei();  

    //SerialON;  Serial.println(milli); delay(1000);        SerialOFF;

    // delayMicroseconds(900);
    /*
    if(N==1)
    {
      SerialON;  Serial.println(timer0_millis); delay(1000);        SerialOFF;
    }
   if(N==999)
   {
      SerialON;  Serial.println(timer0_millis); delay(1000);        SerialOFF;
  }
  delayMicroseconds(9);
  */
  //milli=timer0_millis;
//  N+=1;for(word e=0;e<65535;e++){Shine();delayMicroseconds(50);} //4ms
  //N+=1;//for(word g=0;g<10000;g++){for(word f=0;f<10000;f++){for(word e=0;e<10000;e++){Shine();delayMicroseconds(9);}}} //
//milli2=timer0_millis;
//SerialON;  Serial.println(milli);  Serial.println(milli2);     Serial.print(" ");   Serial.println(N);     delay(1000);        SerialOFF;

//}

  
    // need some delay to start more gradually

//  Shine();Shine();  Shine();  Shine();  Shine();  Shine();  Shine();  Shine();  Shine();
//        cli();long milli2=timer0_millis;sei();  

//            SerialON;  Serial.println(milli);  Serial.println(milli2);     Serial.println(" ");     delay(100);        SerialOFF;

  //delayMicroseconds(6000); // Wait for Vref to settle (если меньше то не успевает)
/*
  ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|2; while (bit_is_set(ADCSRA,ADSC)); tz=ADCW; ADCoff; 


//            SerialON;  Serial.print("temp=");  Serial.println(tz);     delay(1000);        SerialOFF;


//    if (tz>384) {
    if (tz>396) {
      SerialON;  Serial.println(tz);   delay(1000);        SerialOFF;

      //     SerialON;
  //Serial.print("temp=");
  //Serial.println(tz);   
    //    SerialOFF;
//  Serial.end();
//for( byte i=0;i<250;i++){  Serial.println(tz);     delay(100);}// 25 секунд задержка с миганием TX диода
//for( byte i=0;i<25;i++){  __asm__ __volatile__("sbi 5,5\n\t");     delay(50); __asm__ __volatile__("cbi 5,5\n\t");  delay(50); }// 25 секунд задержка с миганием L диода
delay(1500);
  //__asm__ __volatile__("sbi 5,5\n\t");   delay(5000);    //   __asm__ __volatile__("cbi 5,5\n\t"); delay(5000);    
 
    } 

    else{ 

      //Shine(); 
      Shine(); Shine(); Shine(); //Shine(); Shine();

//if(++count2s==3) // ~3 sec
//if(++count2s==60) // ~60 sec
if(++count2s==240) // ~4min
{
  CheckPowerSupply();
  count2s=0;
}


//    if (milli<1800000L){Shine10();}else{Shine();} // первые 30 минут светим в полсилы (в 2 раза реже)  1020lux(10)  68.3w 1120lux(5)   72.6w
   /*Serial.begin(9600);
  Serial.print("2: milli="); // 352-384
    Serial.println(milli);
    Serial.print(" t="); // 352-384
    Serial.println(t);
    
 // Serial.end();
 // delay(1000);
  
//  if (milli<36000000L){Shine();}else{    Shine2();} // короткий 10-ти часовой день для проростов дынь
//Shine();
}*/

   // if (VH()>352) {
  //  Serial.println(VH()); // 352
   // }
    __asm__ __volatile__("rjmp Start\n\t");
}


