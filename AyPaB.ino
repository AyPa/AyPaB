//#include <avr/pgmspace.h>

//#define Pin2Input(port,pin){port&=~(1<<pin);}
//#define Pin2Output(port,pin){port|=(1<<pin);}
//#define Pin2HIGH(port,pin){port|=(1<<pin);}
//#define Pin2LOW(port,pin){port&=~(1<<pin);}
//#define NOP __asm__ __volatile__ ("nop\n\t")


extern uint32_t long timer0_millis;
uint32_t milli;
uint8_t HOUR;
uint16_t MINU;
uint32_t SECU;


uint16_t vcc;

void (* reboot) (void) = 0; //declare reset function @ address 0
byte FanState;

#define FanON { __asm__ __volatile__("sbi 11,0\n\t"); FanState=1; }
#define FanOFF { __asm__ __volatile__("cbi 11,0\n\t"); FanState=0; }

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
if(WDsleep){WDflag=1; WDsleep=0; } else{ __asm__ __volatile__("call 0\n\t");}//reboot();}
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

long nextm;

void setup() 
{  
  DDRD=0b11111111; // set D pins to output
  DDRB=0b11111111; // set B pins to output
  DDRC=0b11111111; // set C pins to output

  delay(1500); 
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
 //   cli();timer0_millis=43200000L;sei();    // полдень
 //cli();timer0_millis=46800000L;sei();    // час дня


 //   cli();timer0_millis=50400000L;sei();    // 2 часа дня
 //   cli();timer0_millis=54000000L;sei();    // 3 часа дня
//    cli();timer0_millis=57600000L;sei();    // 4 часа дня
//    cli();timer0_millis=61200000L;sei();    // 5 вечера
//    cli();timer0_millis=64080000L;sei();    // почти 6 вечера
//   cli();timer0_millis=64800000L;sei();    // 6 вечера

    cli();timer0_millis=68400000L;sei();    // 7 вечера
//    cli();timer0_millis=71000000L;sei();    // почти 8 вечера

    //cli();timer0_millis=72000000L;sei();    // 8 вечера
  //  cli();timer0_millis=75500000L;sei();    // почти 9 вечера
//    cli();timer0_millis=75600000L;sei();    // 9 вечера
//    cli();timer0_millis=78500000L;sei();    // почти 10 вечера
 
  //  cli();timer0_millis=79200000L;sei();    // 10 вечера
//    cli();timer0_millis=82700000L;sei();    // почти 11 вечера
 //   cli();timer0_millis=82800000L;sei();    // 11 вечера
//   cli();timer0_millis=86395000L;sei();    // почти полночь
// cli();timer0_millis=86000000L;sei();    // почти полночь

//milli=timer0_millis;
//  HOUR=milli/3600000L;
//  long tail=milli-HOUR*3600000L;
//  MINU=tail/60000L;
//  tail=tail-MINU*60000L;
//  SECU=tail/1000;

//SerialON;  Serial.println(SECU);Serial.println(MINU);Serial.println(HOUR); delay(500);  SerialOFF;

 //CheckPowerSupply();CheckPowerSupply();
// nextm=timer0_millis+3600000L;
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

byte Longer;

//6500 9000 if Longer flag is set

void Wait(void){__asm__ __volatile__( "wait:\n\t" 
//"call delay32000\n\t"//вялый фотосинтез //2050
//"call delay26500\n\t"// 2180 (2990)
//"call delay21500\n\t" // 2300
//"call delay10500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t"// 18500 всего // 2390
//"call delay10500\n\t"// 2620
//"call delay2500\n\t""call delay2500\n\t""call delay2500\n\t"// 2710
//"call delay2500\n\t""call delay2500\n\t""call delay1000\n\t"
"call delay2500\n\t"//"call delay2500\n\t"//"call delay1000\n\t"
"lds r20,Longer\n\t"
"cpi r20,0\n\t"
"breq 2f\n\t"
"call delay2500\n\t"
"2:\n\t"
//"call delay2500\n\t""call delay2500\n\t"// 2780
//"call delay2500\n\t"// 2860

); }



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
   // DDRC=0b11111111; // set C pins to output

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
  "wdr\n\t"//"nop\n\t"
   "brne 555b\n\t"  // 1 clock if not taken (false)

   "dec r31\n\t"
"breq 111f\n\t"  // выход
"call wait\n\t" // задержка 
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



byte StartRuns;
byte NextRuns;
byte Runs;

void LightAndFan(void) 
{
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
"wdr\n\t"//"nop\n\t"
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
"call wait\n\t" // задержка 
//"call delay32000\n\t" // задержка 32мкс
    "lds r30,NextRuns\n\t" // next runs between pause: 11x3.6мкс потом пауза 32мкс
    "rjmp 555b\n\t"
"111:\n\t"
      ); 
}

void JustLight(void) 
{
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
"wdr\n\t"//"nop\n\t"
      "out 5,r18\n\t" // D7&B0
      "nop\n\t"
    //"ldi r26,0b00000001\n\t"  // 0 is ON (fan)
"out 11,r1\n\t"// PORTD OFF (including fan pin 0)
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
"call wait\n\t" // задержка 
//"call delay32000\n\t" // задержка 32мкс
    "lds r30,NextRuns\n\t" // next runs between pause: 11x3.6мкс потом пауза 32мкс
    "rjmp 555b\n\t"
"111:\n\t"
      ); 
}

void NightLight() // работают все порты (+3ночных).
{
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
"wdr\n\t"//"nop\n\t"
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
"call wait\n\t" // задержка 
    "lds r30,NextRuns\n\t" // next runs between pause: 11x3.6мкс потом пауза 32мкс
    "rjmp 555b\n\t"
"111:\n\t"

      ); 
}

void Shine(void)
{
  if ((HOUR==4)||(HOUR==5)||(HOUR==6)||(HOUR==21)) // предрассветный/предзакатный час. светят все порты.
  {
    FanOFF; StartRuns=1; NextRuns=1; Runs=255; if (MINU&3) {Longer=1;} else {Longer=0;} NightLight(); 
  }// gradually start/stop 48w 1700lux (84w 2200lux)
    // хорошая идея микроперерывчики устраивать - блок питания может "собраться с мыслями" и как "пыхнуть"
 //   else if (HOUR==8){StageN8();}// gradually start/stop 
    else if ((HOUR>=7)&&(HOUR<=20)){ // дневная смена [5..21) (16 часов +2 рассвет/закат)
      
      
    //  if(HOUR&1) //5,7,9,11,13,15,17,19
     // {
     //   StartRuns=11; NextRuns=14; Runs=100;
     // }
     // else // 6,8,10,12,14,16,18,20
     // {
        StartRuns=1; NextRuns=1; Runs=255;
        
//        StartRuns=17; NextRuns=11+(MINU/8); Runs=255;
     // }
      if (MINU&3) {FanON;Longer=1;LightAndFan();}else{FanOFF;Longer=0;JustLight();} // 3 минуты  откачиваем воду из листьев - 1 минуту пауза для входа CO2 - интенсивнее свет


//    StartRuns=1; NextRuns=1; Runs=255; Light(); // ~35мкс одиночные импульсы. 8.9мс х255 цикл
    }
  else { FanOFF;StartRuns=1; NextRuns=1; Runs=255; if (MINU&3) {Longer=1;} else {Longer=0;} C235();} // ночная смена [22..4) (6 часов + 2 рассвет/закат)
}

void PortD(void) // пых лапками 234567 порта D (без откачки Gate Charge портом)
{
  DDRD=0;PORTD=0xff; // значение 1 но на вход - побочная pull ups activation
    __asm__ __volatile__(
    "ldi r18,0\n\t"
    "mov r1,r18\n\t" // r1=0
    //"ldi r18,0b00000001\n\t"
    //"ldi r19,0b00000010\n\t"
    "ldi r20,0b00000100\n\t"
    "ldi r21,0b00001000\n\t"
    "ldi r22,0b00010000\n\t"
    "ldi r23,0b00100000\n\t"
    "ldi r24,0b01000000\n\t"
    "ldi r25,0b10000000\n\t"
    
    "lds r18,FanState\n\t" // лапка 0 управляет вентилятором - пробрасываем ее.
    "or r20,r18\n\t"
    "or r21,r18\n\t"
    "or r22,r18\n\t"
    "or r23,r18\n\t"
    "or r24,r18\n\t"
    "or r25,r18\n\t"

    "ldi r30,0\n\t"
      
"555:\n\t" 
"cli\n\t"
"out 10,r20\n\t" // D2
"call delay1000\n\t"
"out 10,r21\n\t" // D3
"call delay1000\n\t"
"out 10,r22\n\t" // D4
"call delay1000\n\t"
"out 10,r23\n\t" // D5
"call delay1000\n\t"
"out 10,r24\n\t" // D6
"call delay1000\n\t"
"out 10,r25\n\t" // D7
"call delay1000\n\t"
"out 10,r18\n\t" // D0
"call delay1000\n\t"
"sei\n\t"
"dec r30\n\t"
"brne 555b\n\t"
      ); 
}

void PortDD(void) // пых лапками 234567 порта D 
{
 // PORTD=0;DDRD=0xff; // традиционный подход
 // PORTB=0;DDRB=0xff; // традиционный подход
 // PORTC=0;DDRC=0xff; // традиционный подход
    
    if ((HOUR>=7)&&(HOUR<=21)){
      
    FanState=0;
    for (long e=0;e<65535*60L;e++)
    {
      if(e==65534*60L){delay(7);}
    
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
    
    "lds r31,FanState\n\t" // лапка 0 управляет вентилятором - пробрасываем ее.
    "or r20,r31\n\t"
    "or r21,r31\n\t"
    "or r22,r31\n\t"
    "or r23,r31\n\t"
    "or r24,r31\n\t"
    "or r25,r31\n\t"

//    "ldi r30,1\n\t"
      
"555:\n\t" 
"cli\n\t"
"out 11,r20\n\t" // D2
"out 5,r20\n\t" // B2
"out 8,r20\n\t" // C2

// C0 //B0 вместо nops! // нет шума и визга!

// 0.63v 1563 92w

//"nop\n\t""nop\n\t"//"nop\n\t""nop\n\t"//0 - 810 2.24 18.2 <648 14.7w>  1 - 1550 0.88 37.3w 2-1720 0.53 43/1w 3 - 1820 0.42 /shine 281/240 1.04
"out 11,r21\n\t" // D3
"out 5,r21\n\t" // B3
"out 8,r21\n\t" // C3

//"nop\n\t""nop\n\t""nop\n\t"//
//"nop\n\t""nop\n\t"
"out 11,r22\n\t" // D4
"out 5,r22\n\t" // B4
"out 8,r22\n\t" // C4

//"nop\n\t""nop\n\t"
//"nop\n\t""nop\n\t""nop\n\t"
"out 11,r23\n\t" // D5
"out 5,r23\n\t" // B5
"out 8,r23\n\t" // C5

//"nop\n\t""nop\n\t"
//"nop\n\t""nop\n\t""nop\n\t"
"out 11,r24\n\t" // D6
"out 5,r19\n\t" // B1
"out 8,r19\n\t" // C1

//"nop\n\t""nop\n\t"
//"nop\n\t""nop\n\t""nop\n\t"
"out 11,r25\n\t" // D7
"out 5,r18\n\t" // B0
"out 8,r18\n\t" // C0

//"nop\n\t""nop\n\t"
//"nop\n\t""nop\n\t""nop\n\t"
"out 11,r31\n\t" // 
"out 5,r1\n\t" // 
"out 8,r1\n\t" //
"sei\n\t"
//"dec r30\n\t"
//"brne 555b\n\t"
//"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
// 1473 0.77v 86.6w

//"call delay2500\n\t"// 1380 0.91v 81w
//"call delay2500\n\t""call delay2500\n\t"// 1255 1.12v 72.2w
"call delay2500\n\t""call delay2500\n\t""call delay2500\n\t"// 1150 1.29v 65.2w 1125 63.4 63
//"call delay2500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t"// 1071 1.44v 59.5w
//"call delay2500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t"// 1006 1.55v 54.7w
//"call delay2500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t"
// 906 1.75v 47.9w
//"call delay2500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t"
//"call delay2500\n\t""call delay2500\n\t""call delay2500\n\t"
// 795 1.96v 40.9w
//"call delay2500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t"
//"call delay2500\n\t""call delay2500\n\t""call delay2500\n\t"
//"call delay2500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t"
//"call delay2500\n\t""call delay2500\n\t""call delay2500\n\t"
// 581 2.44v 28.8w

      ); 
      
      
    }// for
      //  delay(2000);

  }//if
}


void Lold(void) 
{
    for (long e=0;e<65535*60L;e++)
    {
      if(e==65534*60L){delayMicroseconds(3000);} // 3ms delay
    
    __asm__ __volatile__(
    "ldi r18,0\n\t"
    "mov r1,r18\n\t" // r1=0
    
    // один провод вместо трех до транзистора/трех параллельных транзисторов???
    
    "ldi r20,0b00011100\n\t" // 2,3,4 (порт D)
    "ldi r21,0b11100000\n\t" // 5,6,7 (порт D)
    "ldi r22,0b00000111\n\t" // 0,1,2 (порты C и B)
    "ldi r23,0b00111000\n\t" // 3,4,5 (порты C и B)
    
    "lds r31,FanState\n\t" // лапка 0 управляет вентилятором - пробрасываем ее.
    "or r20,r31\n\t"
    "or r21,r31\n\t"
    "or r18,r31\n\t"
      
"555:\n\t" 
"cli\n\t"
"out 11,r20\n\t" // D 234
"nop\n\t"
"nop\n\t"
"out 11,r21\n\t" // D 567
"nop\n\t"
"nop\n\t"
"out 11,r18\n\t" // выключаем порт D (кроме вентилятора) 

// более ровная распределение нагрузки для блока питания
"sei\n\t""call delay2500\n\t""cli\n\t"

//тут продолжаем вместо задержки прогулку по лапкам.
// средняя температура по транзистору растет с ростом задержки. и он холодный... т.к. его дергают раз в 5-10 мкс. 

"out 5,r22\n\t" // B 012
"nop\n\t"
"nop\n\t"
"out 5,r23\n\t" // B 345
"nop\n\t"
"nop\n\t"
"out 5,r1\n\t" // выключаем порт B 

"sei\n\t""call delay2500\n\t""cli\n\t"

"out 8,r22\n\t" // C 012
"nop\n\t"
"nop\n\t"
"out 8,r23\n\t" // C 345
"nop\n\t"
"nop\n\t"
"out 8,r1\n\t" // выключаем порт C 




// 0.63v 1563 92w

//"nop\n\t""nop\n\t"//"nop\n\t""nop\n\t"//0 - 810 2.24 18.2 <648 14.7w>  1 - 1550 0.88 37.3w 2-1720 0.53 43/1w 3 - 1820 0.42 /shine 281/240 1.04
//"out 11,r21\n\t" // D3
//"out 5,r21\n\t" // B3
//"out 8,r21\n\t" // C3

//"nop\n\t""nop\n\t""nop\n\t"//
//"nop\n\t""nop\n\t"
//"out 11,r22\n\t" // D4
//"out 5,r22\n\t" // B4
//"out 8,r22\n\t" // C4

//"nop\n\t""nop\n\t"
//"nop\n\t""nop\n\t""nop\n\t"
/*"out 11,r23\n\t" // D5
"out 5,r23\n\t" // B5
"out 8,r23\n\t" // C5

//"nop\n\t""nop\n\t"
//"nop\n\t""nop\n\t""nop\n\t"
"out 11,r24\n\t" // D6
"out 5,r19\n\t" // B1
"out 8,r19\n\t" // C1

//"nop\n\t""nop\n\t"
//"nop\n\t""nop\n\t""nop\n\t"
"out 11,r25\n\t" // D7
"out 5,r18\n\t" // B0
"out 8,r18\n\t" // C0

//"nop\n\t""nop\n\t"
//"nop\n\t""nop\n\t""nop\n\t"
"out 11,r31\n\t" // 
"out 5,r1\n\t" // 
"out 8,r1\n\t" //
*/
"sei\n\t"
//"dec r30\n\t"
//"brne 555b\n\t"
//"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
// 1473 0.77v 86.6w

//"call delay2500\n\t"// 1380 0.91v 81w
//"call delay2500\n\t""call delay2500\n\t"// 1255 1.12v 72.2w
"call delay2500\n\t"//"call delay2500\n\t""call delay2500\n\t"// 3100 60.4 3040 59.9 нет разницы?
// 1150 1.29v 65.2w 1125 63.4 63

//"call delay2500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t"// 1071 1.44v 59.5w
//"call delay2500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t"// 1006 1.55v 54.7w
//"call delay2500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t"
// 906 1.75v 47.9w
//"call delay2500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t"
//"call delay2500\n\t""call delay2500\n\t""call delay2500\n\t"
// 795 1.96v 40.9w
//"call delay2500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t"
//"call delay2500\n\t""call delay2500\n\t""call delay2500\n\t"
//"call delay2500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t"
//"call delay2500\n\t""call delay2500\n\t""call delay2500\n\t"
// 581 2.44v 28.8w

      ); 
      
      
    }// for
}


void LightMix75(void) 
{
    for (word e=1;e<65535;e++) 
    {

//            if (!(e&0x3ff)){delayMicroseconds(200);} //3020lux 1.28v 71.1w
            if (!(e&0x7ff)){delayMicroseconds(150);}
      
      byte k=e&7;
 // 30121210     
      if ((k==1)||(k==7)) //0
      {
        
    __asm__ __volatile__(
    
    "ldi r18,0b00000001\n\t"
    "ldi r19,0b00000010\n\t"
    "ldi r20,0b00000100\n\t"
    "ldi r21,0b00001000\n\t"
    "ldi r22,0b00010000\n\t"
    "ldi r23,0b00100000\n\t"
    "ldi r24,0b01000000\n\t"
    "ldi r25,0b10000000\n\t"
        
    "ldi r30,0\n\t"
    "mov r1,r30\n\t" // r1=0
      
"555:\n\t" 
"cli\n\t"
"out 5,r18\n\t" // B0
"out 5,r19\n\t" // B1
"out 5,r20\n\t" // B2
"out 5,r21\n\t" // B3
"out 5,r22\n\t" // B4
"out 5,r23\n\t" // B5
"out 5,r30\n\t" // выключаем порт B

//"sei\n\t""call delay1000\n\t""cli\n\t"
"sei\n\t""jmp 777f\n\t""777:\n\t""cli\n\t"

"out 8,r18\n\t" // C0
"out 8,r19\n\t" // C1
"out 8,r20\n\t" // C2
"out 8,r21\n\t" // C3
"out 8,r22\n\t" // C4
"out 8,r23\n\t" // C5
"out 8,r30\n\t" // выключаем порт C

"sei\n\t""jmp 777f\n\t""777:\n\t"
//"sei\n\t""call delay1000\n\t"

"lds r31,FanState\n\t" // лапка 0 управляет вентилятором.
"or r20,r31\n\t"
"or r21,r31\n\t"
"or r22,r31\n\t"
"or r23,r31\n\t"
"or r24,r31\n\t"
"or r25,r31\n\t"

"cli\n\t"

"out 11,r20\n\t" // D0
"out 11,r21\n\t" // D1
"out 11,r22\n\t" // D2
"out 11,r23\n\t" // D3
"out 11,r24\n\t" // D4
"out 11,r25\n\t" // D5
"out 11,r31\n\t" // выключаем порт D

"sei\n\t"

//"call delay3000\n\t"
"jmp 777f\n\t""777:\n\t"
"jmp 777f\n\t""777:\n\t"
"jmp 777f\n\t""777:\n\t"


// средняя температура по транзистору растет с ростом задержки. и он холодный... т.к. его дергают раз в 5-10 мкс. 

      ); 
    
      }
      else if((k==2)||(k==4)||(k==6)) // 1
      {
        __asm__ __volatile__(
    
    "ldi r18,0b00000001\n\t"
    "ldi r19,0b00000010\n\t"
    "ldi r20,0b00000100\n\t"
    "ldi r21,0b00001000\n\t"
    "ldi r22,0b00010000\n\t"
    "ldi r23,0b00100000\n\t"
    "ldi r24,0b01000000\n\t"
    "ldi r25,0b10000000\n\t"
        
    "ldi r30,0\n\t"
    "mov r1,r30\n\t" // r1=0
      
"555:\n\t" 
"cli\n\t"
"out 5,r18\n\t" // B0
"nop\n\t"
"out 5,r19\n\t" // B1
"nop\n\t"
"out 5,r20\n\t" // B2
"nop\n\t"
"out 5,r21\n\t" // B3
"nop\n\t"
"out 5,r22\n\t" // B4
"nop\n\t"
"out 5,r23\n\t" // B5
"nop\n\t"
"out 5,r30\n\t" // выключаем порт B

"sei\n\t""jmp 777f\n\t""777:\n\t""cli\n\t"

//"sei\n\t""call delay1000\n\t""cli\n\t"

"out 8,r18\n\t" // C0
"nop\n\t"
"out 8,r19\n\t" // C1
"nop\n\t"
"out 8,r20\n\t" // C2
"nop\n\t"
"out 8,r21\n\t" // C3
"nop\n\t"
"out 8,r22\n\t" // C4
"nop\n\t"
"out 8,r23\n\t" // C5
"nop\n\t"
"out 8,r30\n\t" // выключаем порт C

//"sei\n\t""call delay1000\n\t"
"sei\n\t""jmp 777f\n\t""777:\n\t"

"lds r31,FanState\n\t" // лапка 0 управляет вентилятором.
"or r20,r31\n\t"
"or r21,r31\n\t"
"or r22,r31\n\t"
"or r23,r31\n\t"
"or r24,r31\n\t"
"or r25,r31\n\t"

"cli\n\t"

"out 11,r20\n\t" // D0
"nop\n\t"
"out 11,r21\n\t" // D1
"nop\n\t"
"out 11,r22\n\t" // D2
"nop\n\t"
"out 11,r23\n\t" // D3
"nop\n\t"
"out 11,r24\n\t" // D4
"nop\n\t"
"out 11,r25\n\t" // D5
"nop\n\t"
"out 11,r31\n\t" // выключаем порт D

"sei\n\t"

//"call delay2500\n\t"
"jmp 777f\n\t""777:\n\t"
"jmp 777f\n\t""777:\n\t"


      ); 
      
    
      }
      else if((k==3)||(k==5)) // 2
      {
        
    __asm__ __volatile__(
    
    "ldi r18,0b00000001\n\t"
    "ldi r19,0b00000010\n\t"
    "ldi r20,0b00000100\n\t"
    "ldi r21,0b00001000\n\t"
    "ldi r22,0b00010000\n\t"
    "ldi r23,0b00100000\n\t"
    "ldi r24,0b01000000\n\t"
    "ldi r25,0b10000000\n\t"
        
    "ldi r30,0\n\t"
    "mov r1,r30\n\t" // r1=0
      
"555:\n\t" 
"cli\n\t"
"out 5,r18\n\t" // B0
"lds r31,FanState\n\t"// 2clocks nop
"out 5,r19\n\t" // B1
"lds r31,FanState\n\t"// 2clocks nop
"out 5,r20\n\t" // B2
"lds r31,FanState\n\t"// 2clocks nop
"out 5,r21\n\t" // B3
"lds r31,FanState\n\t"// 2clocks nop
"out 5,r22\n\t" // B4
"lds r31,FanState\n\t"// 2clocks nop
"out 5,r23\n\t" // B5
"lds r31,FanState\n\t"// 2clocks nop
"out 5,r30\n\t" // выключаем порт B

//"sei\n\t""call delay1000\n\t""cli\n\t"
"sei\n\t""jmp 777f\n\t""777:\n\t""cli\n\t"

"out 8,r18\n\t" // C0
"lds r31,FanState\n\t"// 2clocks nop
"out 8,r19\n\t" // C1
"lds r31,FanState\n\t"// 2clocks nop
"out 8,r20\n\t" // C2
"lds r31,FanState\n\t"// 2clocks nop
"out 8,r21\n\t" // C3
"lds r31,FanState\n\t"// 2clocks nop
"out 8,r22\n\t" // C4
"lds r31,FanState\n\t"// 2clocks nop
"out 8,r23\n\t" // C5
"lds r31,FanState\n\t"// 2clocks nop
"out 8,r30\n\t" // выключаем порт C

//"sei\n\t""call delay1000\n\t"
"sei\n\t""jmp 777f\n\t""777:\n\t"

"lds r31,FanState\n\t" // лапка 0 управляет вентилятором.
"or r20,r31\n\t"
"or r21,r31\n\t"
"or r22,r31\n\t"
"or r23,r31\n\t"
"or r24,r31\n\t"
"or r25,r31\n\t"

"cli\n\t"

"out 11,r20\n\t" // D0
"lds r31,FanState\n\t"// 2clocks nop
"out 11,r21\n\t" // D1
"lds r31,FanState\n\t"// 2clocks nop
"out 11,r22\n\t" // D2
"lds r31,FanState\n\t"// 2clocks nop
"out 11,r23\n\t" // D3
"lds r31,FanState\n\t"// 2clocks nop
"out 11,r24\n\t" // D4
"lds r31,FanState\n\t"// 2clocks nop
"out 11,r25\n\t" // D5
"lds r31,FanState\n\t"// 2clocks nop
"out 11,r31\n\t" // выключаем порт D

//"call delay1000\n\t"
"sei\n\t""jmp 777f\n\t""777:\n\t"

      ); 
      
      }
      else if (k==0) // 3
      {
    
    __asm__ __volatile__(
    
    "ldi r18,0b00000001\n\t"
    "ldi r19,0b00000010\n\t"
    "ldi r20,0b00000100\n\t"
    "ldi r21,0b00001000\n\t"
    "ldi r22,0b00010000\n\t"
    "ldi r23,0b00100000\n\t"
    "ldi r24,0b01000000\n\t"
    "ldi r25,0b10000000\n\t"
        
    "ldi r30,0\n\t"
    "mov r1,r30\n\t" // r1=0
      
"555:\n\t" 
"cli\n\t"
"out 5,r18\n\t" // B0
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 5,r19\n\t" // B1
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 5,r20\n\t" // B2
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 5,r21\n\t" // B3
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 5,r22\n\t" // B4
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 5,r23\n\t" // B5
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 5,r30\n\t" // выключаем порт B

"sei\n\t""jmp 777f\n\t""777:\n\t""cli\n\t"
//"sei\n\t""call delay1000\n\t""cli\n\t"

"out 8,r18\n\t" // C0
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 8,r19\n\t" // C1
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 8,r20\n\t" // C2
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 8,r21\n\t" // C3
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 8,r22\n\t" // C4
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 8,r23\n\t" // C5
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 8,r30\n\t" // выключаем порт C

"sei\n\t""jmp 777f\n\t""777:\n\t"

//"sei\n\t""call delay1000\n\t"

"lds r31,FanState\n\t" // лапка 0 управляет вентилятором.
"or r20,r31\n\t"
"or r21,r31\n\t"
"or r22,r31\n\t"
"or r23,r31\n\t"
"or r24,r31\n\t"
"or r25,r31\n\t"

"cli\n\t"

"out 11,r20\n\t" // D0
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 11,r21\n\t" // D1
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 11,r22\n\t" // D2
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 11,r23\n\t" // D3
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 11,r24\n\t" // D4
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 11,r25\n\t" // D5
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 11,r31\n\t" // выключаем порт D

"sei\n\t"
      ); 
      }
      
            
    }// for
    
  //  if(e==65534*60L){
//    delayMicroseconds(3000);
//delay(4);  
//} // 4ms delay
    
}

void LightMix100(void) 
{
    for (word e=1;e<65535;e++) 
    {

            if (!(e&0x7ff)){delayMicroseconds(150);}
//            if (!(e&0x3ff)){delayMicroseconds(200);} //2690 65.3
      
      byte k=e&7;
 // 30121210     
      if ((k==1)||(k==7)) //0
      {
        
    __asm__ __volatile__(
    
    "ldi r18,0b00000001\n\t"
    "ldi r19,0b00000010\n\t"
    "ldi r20,0b00000100\n\t"
    "ldi r21,0b00001000\n\t"
    "ldi r22,0b00010000\n\t"
    "ldi r23,0b00100000\n\t"
    "ldi r24,0b01000000\n\t"
    "ldi r25,0b10000000\n\t"
        
    "ldi r30,0\n\t"
    "mov r1,r30\n\t" // r1=0
      
"555:\n\t" 
"cli\n\t"
"out 5,r18\n\t" // B0
"out 5,r19\n\t" // B1
"out 5,r20\n\t" // B2
"out 5,r21\n\t" // B3
"out 5,r22\n\t" // B4
"out 5,r23\n\t" // B5
"out 5,r30\n\t" // выключаем порт B

"sei\n\t""call delay1000\n\t""cli\n\t"

"out 8,r18\n\t" // C0
"out 8,r19\n\t" // C1
"out 8,r20\n\t" // C2
"out 8,r21\n\t" // C3
"out 8,r22\n\t" // C4
"out 8,r23\n\t" // C5
"out 8,r30\n\t" // выключаем порт C

"sei\n\t""call delay1000\n\t"

"lds r31,FanState\n\t" // лапка 0 управляет вентилятором.
"or r20,r31\n\t"
"or r21,r31\n\t"
"or r22,r31\n\t"
"or r23,r31\n\t"
"or r24,r31\n\t"
"or r25,r31\n\t"

"cli\n\t"

"out 11,r20\n\t" // D0
"out 11,r21\n\t" // D1
"out 11,r22\n\t" // D2
"out 11,r23\n\t" // D3
"out 11,r24\n\t" // D4
"out 11,r25\n\t" // D5
"out 11,r31\n\t" // выключаем порт D

"sei\n\t"

"call delay3000\n\t"

// средняя температура по транзистору растет с ростом задержки. и он холодный... т.к. его дергают раз в 5-10 мкс. 

      ); 
    
      }
      else if((k==2)||(k==4)||(k==6)) // 1
      {
        __asm__ __volatile__(
    
    "ldi r18,0b00000001\n\t"
    "ldi r19,0b00000010\n\t"
    "ldi r20,0b00000100\n\t"
    "ldi r21,0b00001000\n\t"
    "ldi r22,0b00010000\n\t"
    "ldi r23,0b00100000\n\t"
    "ldi r24,0b01000000\n\t"
    "ldi r25,0b10000000\n\t"
        
    "ldi r30,0\n\t"
    "mov r1,r30\n\t" // r1=0
      
"555:\n\t" 
"cli\n\t"
"out 5,r18\n\t" // B0
"nop\n\t"
"out 5,r19\n\t" // B1
"nop\n\t"
"out 5,r20\n\t" // B2
"nop\n\t"
"out 5,r21\n\t" // B3
"nop\n\t"
"out 5,r22\n\t" // B4
"nop\n\t"
"out 5,r23\n\t" // B5
"nop\n\t"
"out 5,r30\n\t" // выключаем порт B

"sei\n\t""call delay1000\n\t""cli\n\t"

"out 8,r18\n\t" // C0
"nop\n\t"
"out 8,r19\n\t" // C1
"nop\n\t"
"out 8,r20\n\t" // C2
"nop\n\t"
"out 8,r21\n\t" // C3
"nop\n\t"
"out 8,r22\n\t" // C4
"nop\n\t"
"out 8,r23\n\t" // C5
"nop\n\t"
"out 8,r30\n\t" // выключаем порт C

"sei\n\t""call delay1000\n\t"

"lds r31,FanState\n\t" // лапка 0 управляет вентилятором.
"or r20,r31\n\t"
"or r21,r31\n\t"
"or r22,r31\n\t"
"or r23,r31\n\t"
"or r24,r31\n\t"
"or r25,r31\n\t"

"cli\n\t"

"out 11,r20\n\t" // D0
"nop\n\t"
"out 11,r21\n\t" // D1
"nop\n\t"
"out 11,r22\n\t" // D2
"nop\n\t"
"out 11,r23\n\t" // D3
"nop\n\t"
"out 11,r24\n\t" // D4
"nop\n\t"
"out 11,r25\n\t" // D5
"nop\n\t"
"out 11,r31\n\t" // выключаем порт D

"sei\n\t"

"call delay2500\n\t"


      ); 
      
    
      }
      else if((k==3)||(k==5)) // 2
      {
        
    __asm__ __volatile__(
    
    "ldi r18,0b00000001\n\t"
    "ldi r19,0b00000010\n\t"
    "ldi r20,0b00000100\n\t"
    "ldi r21,0b00001000\n\t"
    "ldi r22,0b00010000\n\t"
    "ldi r23,0b00100000\n\t"
    "ldi r24,0b01000000\n\t"
    "ldi r25,0b10000000\n\t"
        
    "ldi r30,0\n\t"
    "mov r1,r30\n\t" // r1=0
      
"555:\n\t" 
"cli\n\t"
"out 5,r18\n\t" // B0
"lds r31,FanState\n\t"// 2clocks nop
"out 5,r19\n\t" // B1
"lds r31,FanState\n\t"// 2clocks nop
"out 5,r20\n\t" // B2
"lds r31,FanState\n\t"// 2clocks nop
"out 5,r21\n\t" // B3
"lds r31,FanState\n\t"// 2clocks nop
"out 5,r22\n\t" // B4
"lds r31,FanState\n\t"// 2clocks nop
"out 5,r23\n\t" // B5
"lds r31,FanState\n\t"// 2clocks nop
"out 5,r30\n\t" // выключаем порт B

"sei\n\t""call delay1000\n\t""cli\n\t"

"out 8,r18\n\t" // C0
"lds r31,FanState\n\t"// 2clocks nop
"out 8,r19\n\t" // C1
"lds r31,FanState\n\t"// 2clocks nop
"out 8,r20\n\t" // C2
"lds r31,FanState\n\t"// 2clocks nop
"out 8,r21\n\t" // C3
"lds r31,FanState\n\t"// 2clocks nop
"out 8,r22\n\t" // C4
"lds r31,FanState\n\t"// 2clocks nop
"out 8,r23\n\t" // C5
"lds r31,FanState\n\t"// 2clocks nop
"out 8,r30\n\t" // выключаем порт C

"sei\n\t""call delay1000\n\t"

"lds r31,FanState\n\t" // лапка 0 управляет вентилятором.
"or r20,r31\n\t"
"or r21,r31\n\t"
"or r22,r31\n\t"
"or r23,r31\n\t"
"or r24,r31\n\t"
"or r25,r31\n\t"

"cli\n\t"

"out 11,r20\n\t" // D0
"lds r31,FanState\n\t"// 2clocks nop
"out 11,r21\n\t" // D1
"lds r31,FanState\n\t"// 2clocks nop
"out 11,r22\n\t" // D2
"lds r31,FanState\n\t"// 2clocks nop
"out 11,r23\n\t" // D3
"lds r31,FanState\n\t"// 2clocks nop
"out 11,r24\n\t" // D4
"lds r31,FanState\n\t"// 2clocks nop
"out 11,r25\n\t" // D5
"lds r31,FanState\n\t"// 2clocks nop
"out 11,r31\n\t" // выключаем порт D

"sei\n\t"

"call delay1000\n\t"

      ); 
      
      }
      else if (k==0) // 3
      {
    
    __asm__ __volatile__(
    
    "ldi r18,0b00000001\n\t"
    "ldi r19,0b00000010\n\t"
    "ldi r20,0b00000100\n\t"
    "ldi r21,0b00001000\n\t"
    "ldi r22,0b00010000\n\t"
    "ldi r23,0b00100000\n\t"
    "ldi r24,0b01000000\n\t"
    "ldi r25,0b10000000\n\t"
        
    "ldi r30,0\n\t"
    "mov r1,r30\n\t" // r1=0
      
"555:\n\t" 
"cli\n\t"
"out 5,r18\n\t" // B0
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 5,r19\n\t" // B1
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 5,r20\n\t" // B2
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 5,r21\n\t" // B3
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 5,r22\n\t" // B4
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 5,r23\n\t" // B5
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 5,r30\n\t" // выключаем порт B

"sei\n\t""call delay1000\n\t""cli\n\t"

"out 8,r18\n\t" // C0
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 8,r19\n\t" // C1
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 8,r20\n\t" // C2
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 8,r21\n\t" // C3
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 8,r22\n\t" // C4
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 8,r23\n\t" // C5
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 8,r30\n\t" // выключаем порт C

"sei\n\t""call delay1000\n\t"

"lds r31,FanState\n\t" // лапка 0 управляет вентилятором.
"or r20,r31\n\t"
"or r21,r31\n\t"
"or r22,r31\n\t"
"or r23,r31\n\t"
"or r24,r31\n\t"
"or r25,r31\n\t"

"cli\n\t"

"out 11,r20\n\t" // D0
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 11,r21\n\t" // D1
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 11,r22\n\t" // D2
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 11,r23\n\t" // D3
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 11,r24\n\t" // D4
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 11,r25\n\t" // D5
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 11,r31\n\t" // выключаем порт D

"sei\n\t"
      ); 
      }
      
            
    }// for
    
  //  if(e==65534*60L){
//    delayMicroseconds(3000);
//delay(4);  
//} // 4ms delay
    
}

void LightMix85(void) 
{
    for (word e=1;e<65535;e++) 
    {

//            if (!(e&0x1fff)){delayMicroseconds(333);} //2860 66.4 
//            if (!(e&0xfff)){delayMicroseconds(333);} //2860 66.2
//            if (!(e&0xfff)){delayMicroseconds(300);} //2860 66.2
//            if (!(e&0xfff)){delayMicroseconds(200);} //2860 66.3
            if (!(e&0x7ff)){delayMicroseconds(150);}
//            if (!(e&0x3ff)){delayMicroseconds(200);} //2840 65.3

//      if (!(e&0x1ff)){delayMicroseconds(333);} //511 раз из 65535   2630 62.4 
//      if (!(e&0x7f)){delayMicroseconds(333);} //2280 53.7w
//      if (!(e&0x3f)){delayMicroseconds(333);} //1980 46,2w
//        if (!(e&0x1f)){delayMicroseconds(333);} //511 раз из 65535   37.1w    
    
      
      
      byte k=e&7;
 // 30121210     
      if ((k==1)||(k==7)) //0
      {
        
    __asm__ __volatile__(
    
    "ldi r18,0b00000001\n\t"
    "ldi r19,0b00000010\n\t"
    "ldi r20,0b00000100\n\t"
    "ldi r21,0b00001000\n\t"
    "ldi r22,0b00010000\n\t"
    "ldi r23,0b00100000\n\t"
    "ldi r24,0b01000000\n\t"
    "ldi r25,0b10000000\n\t"
        
    "ldi r30,0\n\t"
    "mov r1,r30\n\t" // r1=0
      
"555:\n\t" 
"cli\n\t"
"out 5,r18\n\t" // B0
"out 5,r19\n\t" // B1
"out 5,r20\n\t" // B2
"out 5,r21\n\t" // B3
"out 5,r22\n\t" // B4
"out 5,r23\n\t" // B5
"out 5,r30\n\t" // выключаем порт B

"sei\n\t""call delay500\n\t""cli\n\t"

"out 8,r18\n\t" // C0
"out 8,r19\n\t" // C1
"out 8,r20\n\t" // C2
"out 8,r21\n\t" // C3
"out 8,r22\n\t" // C4
"out 8,r23\n\t" // C5
"out 8,r30\n\t" // выключаем порт C

"sei\n\t""call delay500\n\t"

"lds r31,FanState\n\t" // лапка 0 управляет вентилятором.
"or r20,r31\n\t"
"or r21,r31\n\t"
"or r22,r31\n\t"
"or r23,r31\n\t"
"or r24,r31\n\t"
"or r25,r31\n\t"

"cli\n\t"

"out 11,r20\n\t" // D0
"out 11,r21\n\t" // D1
"out 11,r22\n\t" // D2
"out 11,r23\n\t" // D3
"out 11,r24\n\t" // D4
"out 11,r25\n\t" // D5
"out 11,r31\n\t" // выключаем порт D

"sei\n\t"

"call delay2500\n\t"

// средняя температура по транзистору растет с ростом задержки. и он холодный... т.к. его дергают раз в 5-10 мкс. 

      ); 
    
      }
      else if((k==2)||(k==4)||(k==6)) // 1
      {
        __asm__ __volatile__(
    
    "ldi r18,0b00000001\n\t"
    "ldi r19,0b00000010\n\t"
    "ldi r20,0b00000100\n\t"
    "ldi r21,0b00001000\n\t"
    "ldi r22,0b00010000\n\t"
    "ldi r23,0b00100000\n\t"
    "ldi r24,0b01000000\n\t"
    "ldi r25,0b10000000\n\t"
        
    "ldi r30,0\n\t"
    "mov r1,r30\n\t" // r1=0
      
"555:\n\t" 
"cli\n\t"
"out 5,r18\n\t" // B0
"nop\n\t"
"out 5,r19\n\t" // B1
"nop\n\t"
"out 5,r20\n\t" // B2
"nop\n\t"
"out 5,r21\n\t" // B3
"nop\n\t"
"out 5,r22\n\t" // B4
"nop\n\t"
"out 5,r23\n\t" // B5
"nop\n\t"
"out 5,r30\n\t" // выключаем порт B

"sei\n\t""call delay500\n\t""cli\n\t"

"out 8,r18\n\t" // C0
"nop\n\t"
"out 8,r19\n\t" // C1
"nop\n\t"
"out 8,r20\n\t" // C2
"nop\n\t"
"out 8,r21\n\t" // C3
"nop\n\t"
"out 8,r22\n\t" // C4
"nop\n\t"
"out 8,r23\n\t" // C5
"nop\n\t"
"out 8,r30\n\t" // выключаем порт C

"sei\n\t""call delay500\n\t"

"lds r31,FanState\n\t" // лапка 0 управляет вентилятором.
"or r20,r31\n\t"
"or r21,r31\n\t"
"or r22,r31\n\t"
"or r23,r31\n\t"
"or r24,r31\n\t"
"or r25,r31\n\t"

"cli\n\t"

"out 11,r20\n\t" // D0
"nop\n\t"
"out 11,r21\n\t" // D1
"nop\n\t"
"out 11,r22\n\t" // D2
"nop\n\t"
"out 11,r23\n\t" // D3
"nop\n\t"
"out 11,r24\n\t" // D4
"nop\n\t"
"out 11,r25\n\t" // D5
"nop\n\t"
"out 11,r31\n\t" // выключаем порт D

"sei\n\t"

"call delay1000\n\t"
"call delay1000\n\t"

// средняя температура по транзистору растет с ростом задержки. и он холодный... т.к. его дергают раз в 5-10 мкс. 

      ); 
      
    
      }
      else if((k==3)||(k==5)) // 2
      {
        
    __asm__ __volatile__(
    
    "ldi r18,0b00000001\n\t"
    "ldi r19,0b00000010\n\t"
    "ldi r20,0b00000100\n\t"
    "ldi r21,0b00001000\n\t"
    "ldi r22,0b00010000\n\t"
    "ldi r23,0b00100000\n\t"
    "ldi r24,0b01000000\n\t"
    "ldi r25,0b10000000\n\t"
        
    "ldi r30,0\n\t"
    "mov r1,r30\n\t" // r1=0
      
"555:\n\t" 
"cli\n\t"
"out 5,r18\n\t" // B0
"lds r31,FanState\n\t"// 2clocks nop
"out 5,r19\n\t" // B1
"lds r31,FanState\n\t"// 2clocks nop
"out 5,r20\n\t" // B2
"lds r31,FanState\n\t"// 2clocks nop
"out 5,r21\n\t" // B3
"lds r31,FanState\n\t"// 2clocks nop
"out 5,r22\n\t" // B4
"lds r31,FanState\n\t"// 2clocks nop
"out 5,r23\n\t" // B5
"lds r31,FanState\n\t"// 2clocks nop
"out 5,r30\n\t" // выключаем порт B

"sei\n\t""call delay500\n\t""cli\n\t"

"out 8,r18\n\t" // C0
"lds r31,FanState\n\t"// 2clocks nop
"out 8,r19\n\t" // C1
"lds r31,FanState\n\t"// 2clocks nop
"out 8,r20\n\t" // C2
"lds r31,FanState\n\t"// 2clocks nop
"out 8,r21\n\t" // C3
"lds r31,FanState\n\t"// 2clocks nop
"out 8,r22\n\t" // C4
"lds r31,FanState\n\t"// 2clocks nop
"out 8,r23\n\t" // C5
"lds r31,FanState\n\t"// 2clocks nop
"out 8,r30\n\t" // выключаем порт C

"sei\n\t""call delay500\n\t"

"lds r31,FanState\n\t" // лапка 0 управляет вентилятором.
"or r20,r31\n\t"
"or r21,r31\n\t"
"or r22,r31\n\t"
"or r23,r31\n\t"
"or r24,r31\n\t"
"or r25,r31\n\t"

"cli\n\t"

"out 11,r20\n\t" // D0
"lds r31,FanState\n\t"// 2clocks nop
"out 11,r21\n\t" // D1
"lds r31,FanState\n\t"// 2clocks nop
"out 11,r22\n\t" // D2
"lds r31,FanState\n\t"// 2clocks nop
"out 11,r23\n\t" // D3
"lds r31,FanState\n\t"// 2clocks nop
"out 11,r24\n\t" // D4
"lds r31,FanState\n\t"// 2clocks nop
"out 11,r25\n\t" // D5
"lds r31,FanState\n\t"// 2clocks nop
"out 11,r31\n\t" // выключаем порт D

"sei\n\t"

"call delay500\n\t"
      ); 
      
      }
      else if (k==0) // 3
      {
    
    __asm__ __volatile__(
    
    "ldi r18,0b00000001\n\t"
    "ldi r19,0b00000010\n\t"
    "ldi r20,0b00000100\n\t"
    "ldi r21,0b00001000\n\t"
    "ldi r22,0b00010000\n\t"
    "ldi r23,0b00100000\n\t"
    "ldi r24,0b01000000\n\t"
    "ldi r25,0b10000000\n\t"
        
    "ldi r30,0\n\t"
    "mov r1,r30\n\t" // r1=0
      
"555:\n\t" 
"cli\n\t"
"out 5,r18\n\t" // B0
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 5,r19\n\t" // B1
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 5,r20\n\t" // B2
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 5,r21\n\t" // B3
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 5,r22\n\t" // B4
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 5,r23\n\t" // B5
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 5,r30\n\t" // выключаем порт B

"sei\n\t""call delay500\n\t""cli\n\t"

"out 8,r18\n\t" // C0
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 8,r19\n\t" // C1
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 8,r20\n\t" // C2
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 8,r21\n\t" // C3
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 8,r22\n\t" // C4
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 8,r23\n\t" // C5
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 8,r30\n\t" // выключаем порт C

"sei\n\t""call delay500\n\t"

"lds r31,FanState\n\t" // лапка 0 управляет вентилятором.
"or r20,r31\n\t"
"or r21,r31\n\t"
"or r22,r31\n\t"
"or r23,r31\n\t"
"or r24,r31\n\t"
"or r25,r31\n\t"

"cli\n\t"

"out 11,r20\n\t" // D0
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 11,r21\n\t" // D1
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 11,r22\n\t" // D2
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 11,r23\n\t" // D3
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 11,r24\n\t" // D4
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 11,r25\n\t" // D5
"jmp 999f\n\t""999:\n\t"// 3 clocks nop
"out 11,r31\n\t" // выключаем порт D

"sei\n\t"
      ); 
      }
    }// for
    
    
}

void LightA(void) // 780lux 1.943v 35.4w 22.3 lux/w

// 809lux 1.89v 33.5w 24.1 lux/w
{
  
//  Call-used registers (r18-r27, r30-r31):
//May be allocated by gcc for local data. You may use them freely in assembler subroutines.
//Call-saved registers (r2-r17, r28-r29):  
    for (word e=1;e<65535;e++) 
//    for (word e=1;e<1000;e++) 
    {

//            if (!(e&0x1fff)){delayMicroseconds(333);} //2860 66.4 
//            if (!(e&0xfff)){delayMicroseconds(333);} //2860 66.2
//            if (!(e&0xfff)){delayMicroseconds(300);} //2860 66.2
//            if (!(e&0xfff)){delayMicroseconds(200);} //2860 66.3
      
    
//        if (!(e&0x7ff)){delayMicroseconds(150);}

//            if (!(e&0x3ff)){delayMicroseconds(200);} //2840 65.3

//      if (!(e&0x1ff)){delayMicroseconds(333);} //511 раз из 65535   2630 62.4 
//      if (!(e&0x7f)){delayMicroseconds(333);} //2280 53.7w
//      if (!(e&0x3f)){delayMicroseconds(333);} //1980 46,2w
//        if (!(e&0x1f)){delayMicroseconds(333);} //511 раз из 65535   37.1w    
    
      
      
  //    byte k=e&7;
 // 30121210     
    //  if ((k==1)||(k==7)) //0
      //{
        
    __asm__ __volatile__(
    
    "push r28\n\t"
    "push r29\n\t"
    
    "ldi r18,0b00000001\n\t"  // маски для портов B и C 012345
    "ldi r19,0b00000010\n\t"
    "ldi r20,0b00000100\n\t"
    "ldi r21,0b00001000\n\t"
    "ldi r22,0b00010000\n\t"
    "ldi r23,0b00100000\n\t"

    "ldi r24,0b00000100\n\t"  // маски для порта D 234567 (чтобы вентилятор пробрасывать)
    "ldi r25,0b00001000\n\t"
    "ldi r26,0b00010000\n\t"
    "ldi r27,0b00100000\n\t"
    "ldi r28,0b01000000\n\t"
    "ldi r29,0b10000000\n\t"

    "lds r31,FanState\n\t" // лапка 0 управляет вентилятором.
    "or r24,r31\n\t"
    "or r25,r31\n\t"
    "or r26,r31\n\t"
    "or r27,r31\n\t"
    "or r28,r31\n\t"
    "or r29,r31\n\t"
    
    "ldi r30,0\n\t"  // 256 раз
    "mov r1,r30\n\t" // r1=0
      
"555:\n\t" //~1.6 мкс цикл (5 циклов за 8 мкс) ~625000 в секунду
//  //~1.5 мкс цикл (2 цикла за 3 мкс) ~667000 в секунду без cli|sei (125ns)
"cli\n\t" // с включенными прерываниями мерцает почем зря.
"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

//21x4x62.5ns=5250ns
//+6x62.5ns=375ns
// 5625ns цикл 711111 пыхов в секунду
"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

//21x8x62.5ns=10500ns
//+6x62.5ns=375ns
// 10875ns цикл 735632 пыхов в секунду

"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

//21x9x62.5ns=11812ns
//+6x62.5ns=375ns
// 12187.5ns цикл 738461 пыхов в секунду (минус прерывания)

"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D


//21x20x62.5ns=26250ns
//+6x62.5ns=375ns
// 26625ns цикл 751089 пыхов в секунду (минус прерывания)

"sei\n\t"
"dec r30\n\t"
"breq 111f\n\t"
"rjmp 555b\n\t"
"111:\n\t"
"pop r29\n\t"
"pop r28\n\t"

      ); 
      }//for
}

void LightAA(void)
{
  
//  Call-used registers (r18-r27, r30-r31):
//May be allocated by gcc for local data. You may use them freely in assembler subroutines.
//Call-saved registers (r2-r17, r28-r29):  
    for (word e=1;e<65535;e++) 
    {

    __asm__ __volatile__(
    
    "push r28\n\t"
    "push r29\n\t"
    "push r16\n\t"
    "push r17\n\t"
    
    "ldi r18,0b00000001\n\t"  // маски для портов B и C 012345
    "ldi r19,0b00000011\n\t"
    "ldi r20,0b00000110\n\t"
    "ldi r21,0b00001100\n\t"
    "ldi r22,0b00011000\n\t"
    "ldi r23,0b00110000\n\t"
    "ldi r16,0b00100000\n\t" 

    "ldi r24,0b00000100\n\t"  // маски для порта D 234567 (чтобы вентилятор пробрасывать)
    "ldi r25,0b00001100\n\t"
    "ldi r26,0b00011000\n\t"
    "ldi r27,0b00110000\n\t"
    "ldi r28,0b01100000\n\t"
    "ldi r29,0b11000000\n\t"
    "ldi r17,0b10000000\n\t" 

    "lds r31,FanState\n\t" // лапка 0 управляет вентилятором.
    "or r24,r31\n\t"
    "or r25,r31\n\t"
    "or r26,r31\n\t"
    "or r27,r31\n\t"
    "or r28,r31\n\t"
    "or r29,r31\n\t"
    "or r17,r31\n\t"
    
    "ldi r30,0\n\t"  // 256 раз
    "mov r1,r30\n\t" // r1=0
      
"555:\n\t"//~1.54 мкс цикл ~650000 в секунду.
"cli\n\t" // с включенными прерываниями мерцает почем зря.

"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r16\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r16\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r17\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r16\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r16\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r17\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r16\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r16\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r17\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r16\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r16\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r17\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r16\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r16\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r17\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D


"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r16\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r16\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r17\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r16\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r16\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r17\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r16\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r16\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r17\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r16\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r16\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r17\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r16\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r16\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r17\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D


//24x10x62.5ns=15000ns
//+6x62.5ns=375ns
// 15375ns цикл 650000 пыхов в секунду (минус прерывания)

"sei\n\t"
"dec r30\n\t"
"breq 111f\n\t"
"rjmp 555b\n\t"

"111:\n\t"
"pop r17\n\t"
"pop r16\n\t"
"pop r29\n\t"
"pop r28\n\t"

      ); 
      }//for
}

void LightAA1(void)
{
  
//  Call-used registers (r18-r27, r30-r31):
//May be allocated by gcc for local data. You may use them freely in assembler subroutines.
//Call-saved registers (r2-r17, r28-r29):  
    __asm__ __volatile__(
    
    "push r28\n\t"
    "push r29\n\t"
    "push r16\n\t"
    "push r17\n\t"
    
    "ldi r18,0b00000001\n\t"  // маски для портов B и C 012345
    "ldi r19,0b00000011\n\t"
    "ldi r20,0b00000110\n\t"
    "ldi r21,0b00001100\n\t"
    "ldi r22,0b00011000\n\t"
    "ldi r23,0b00110000\n\t"
    "ldi r16,0b00100000\n\t" 

    "ldi r24,0b00000100\n\t"  // маски для порта D 234567 (чтобы вентилятор пробрасывать)
    "ldi r25,0b00001100\n\t"
    "ldi r26,0b00011000\n\t"
    "ldi r27,0b00110000\n\t"
    "ldi r28,0b01100000\n\t"
    "ldi r29,0b11000000\n\t"
    "ldi r17,0b10000000\n\t" 

    "lds r31,FanState\n\t" // лапка 0 управляет вентилятором.
    "or r24,r31\n\t"
    "or r25,r31\n\t"
    "or r26,r31\n\t"
    "or r27,r31\n\t"
    "or r28,r31\n\t"
    "or r29,r31\n\t"
    "or r17,r31\n\t"
    
    "ldi r30,0\n\t" 
    "mov r1,r30\n\t" // r1=0
      
"555:\n\t"//~1.54 мкс цикл ~650000 в секунду.
"cli\n\t" // с включенными прерываниями мерцает почем зря.

"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r16\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r16\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r17\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

"sei\n\t"

"pop r17\n\t"
"pop r16\n\t"
"pop r29\n\t"
"pop r28\n\t"

      ); 
}


void LightB(void) // 1434lux 0.806v 83.1w  17.25 lux/w
// 1452lux 0.83v 80.9w 17.9 lux/w
{
  
    for (word e=1;e<65535;e++) 
    {
        
    __asm__ __volatile__(
    
    "push r28\n\t"
    "push r29\n\t"
    
    "ldi r18,0b00000001\n\t"  // маски для портов B и C 012345
    "ldi r19,0b00000010\n\t"
    "ldi r20,0b00000100\n\t"
    "ldi r21,0b00001000\n\t"
    "ldi r22,0b00010000\n\t"
    "ldi r23,0b00100000\n\t"

    "ldi r24,0b00000100\n\t"  // маски для порта D 234567 (чтобы вентилятор пробрасывать)
    "ldi r25,0b00001000\n\t"
    "ldi r26,0b00010000\n\t"
    "ldi r27,0b00100000\n\t"
    "ldi r28,0b01000000\n\t"
    "ldi r29,0b10000000\n\t"

    "lds r31,FanState\n\t" // лапка 0 управляет вентилятором.
    "or r24,r31\n\t"
    "or r25,r31\n\t"
    "or r26,r31\n\t"
    "or r27,r31\n\t"
    "or r28,r31\n\t"
    "or r29,r31\n\t"
    
    "ldi r30,0\n\t"  // 256 раз
    "mov r1,r30\n\t" // r1=0
      
"555:\n\t" //~2.75 мкс цикл (чуть больше 3х циклов за 8 мкс) ~360000 в секунду
//~2.55 мкс цикл (чуть больше 4х циклов за 10 мкс) ~392000 в секунду
"cli\n\t"
"out 5,r18\n\t" // B0
"nop\n\t"
"out 5,r19\n\t" // B1
"nop\n\t"
"out 5,r20\n\t" // B2
"nop\n\t"
"out 5,r21\n\t" // B3
"nop\n\t"
"out 5,r22\n\t" // B4
"nop\n\t"
"out 5,r23\n\t" // B5
"nop\n\t"
"out 5,r1\n\t" // выключаем порт B

"out 8,r18\n\t" // C0
"nop\n\t"
"out 8,r19\n\t" // C1
"nop\n\t"
"out 8,r20\n\t" // C2
"nop\n\t"
"out 8,r21\n\t" // C3
"nop\n\t"
"out 8,r22\n\t" // C4
"nop\n\t"
"out 8,r23\n\t" // C5
"nop\n\t"
"out 8,r1\n\t" // выключаем порт C

"out 11,r24\n\t" // D0
"nop\n\t"
"out 11,r25\n\t" // D1
"nop\n\t"
"out 11,r26\n\t" // D2
"nop\n\t"
"out 11,r27\n\t" // D3
"nop\n\t"
"out 11,r28\n\t" // D4
"dec r30\n\t"//"nop\n\t"
"out 11,r29\n\t" // D5
"sei\n\t"//"nop\n\t"
"out 11,r31\n\t" // выключаем порт D

"brne 555b\n\t"

"pop r29\n\t"
"pop r28\n\t"

      ); 
      }//for
}

uint8_t count2s=0;
uint16_t tz,tw;
long LastDark;

void loop() {

    __asm__ __volatile__("Start:\n\t");
    __asm__ __volatile__("wdr\n\t");//  wdt_reset();

      cli();milli=timer0_millis;sei();  
//if(milli>=nextm)
//{
  HOUR=milli/3600000L;
  long tail=milli-HOUR*3600000L;
  MINU=tail/60000L;
  tail=tail-MINU*60000L;
  SECU=tail/1000;

   if (HOUR>=24){ cli();timer0_millis-=86400000L;sei(); HOUR=0;MINU=0;SECU=0;LastDark=0;}
   if ((milli-LastDark)>900000L) {delay(1000);LastDark=milli;}

//if ((!SECU)&&(!(MINU&0x1F))){delay(1000);} // примерно раз в полчаса (1я и 33я минуты) пауза в 1 секунду для сброса/отдыха/перезарядки энзимов и/или вообще для кругозора как оно без света.
//if ((HOUR>=7)&&(HOUR<=21)) {for(word j=0;j<1500;j++){LightMix85();}delay(1000);} // период между пыхами 8.5 мкс
//if ((HOUR>=7)&&(HOUR<=21)) {for(word j=0;j<1500;j++){LightMix100();}delay(1000);} // период между пыхами 10 мкс
if ((HOUR>=6)&&(HOUR<=20)) {

  if ((milli>>16)&0x3){FanON;}else{FanOFF;} // каждую четвертую минуту тушим вентиляторы для облегчения доступа CO2 в листья.


// облачный полдень 8 ноября 800люкс после тройного стеклопакета 

if ((HOUR==6)||(HOUR==20)){for(word r=0;r<65535;r++){LightAA1();delayMicroseconds(7);}}//800 2/3 интенсивности 66% фотонов для плавной раскачки
else {LightAA();}//1220

//for(word r=0;r<65535;r++){LightAA1();delayMicroseconds(1);}//1004
//for(word r=0;r<65535;r++){LightAA1();__asm__ __volatile__("call delay1000\n\t");}//1006

//for(word r=0;r<65535;r++){LightAA1();delayMicroseconds(2);}//970

//for(word r=0;r<65535;r++){LightAA1();delayMicroseconds(3);}//929
//for(word r=0;r<65535;r++){LightAA1();delayMicroseconds(4);}//912
//for(word r=0;r<65535;r++){LightAA1();delayMicroseconds(5);}//863
//for(word r=0;r<65535;r++){LightAA1();delayMicroseconds(6);}//830
//for(word r=0;r<65535;r++){LightAA1();delayMicroseconds(7);}//799
//for(word r=0;r<65535;r++){LightAA1();delayMicroseconds(9);}//742
//for(word r=0;r<65535;r++){LightAA1();delayMicroseconds(11);}//693

//LightAA1();// 139

} // период между пыхами 7.5 мкс 11min


 
    __asm__ __volatile__("rjmp Start\n\t");
}


