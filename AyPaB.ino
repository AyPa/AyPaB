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

long nextm;

void setup() 
{  
  DDRD=0b11111111; // set D pins to output
  DDRB=0b11111111; // set B pins to output
  DDRC=0b11111111; // set C pins to output

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
  //  cli();timer0_millis=43200000L;sei();    // полдень
// cli();timer0_millis=46800000L;sei();    // час дня


  //  cli();timer0_millis=50400000L;sei();    // 2 часа дня
  //  cli();timer0_millis=54000000L;sei();    // 3 часа дня
  //  cli();timer0_millis=57600000L;sei();    // 4 часа дня
 //   cli();timer0_millis=61200000L;sei();    // 5 вечера
   // cli();timer0_millis=64780000L;sei();    // почти 6 вечера
//   cli();timer0_millis=64800000L;sei();    // 6 вечера

 //   cli();timer0_millis=68400000L;sei();    // 7 вечера
    cli();timer0_millis=71000000L;sei();    // почти 8 вечера

    //cli();timer0_millis=72000000L;sei();    // 8 вечера
   // cli();timer0_millis=75600000L;sei();    // 9 вечера
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

void Wait(void){__asm__ __volatile__( "wait:\n\t" 
//"call delay32000\n\t"//вялый фотосинтез
//"call delay21500\n\t"
"call delay26500\n\t"
//"call delay10500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t"// 18500 всего
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
  if ((HOUR==4)||(HOUR==21)) // предрассветный/предзакатный час. светят все порты.
  {
    FanOFF; StartRuns=2; NextRuns=1; Runs=255; NightLight(); 
  }// gradually start/stop 48w 1700lux (84w 2200lux)
    // хорошая идея микроперерывчики устраивать - блок питания может "собраться с мыслями" и как "пыхнуть"
 //   else if (HOUR==8){StageN8();}// gradually start/stop 
    else if ((HOUR>=5)&&(HOUR<=20)){ // дневная смена [5..21) (16 часов +2 рассвет/закат)
      
      
    //  if(HOUR&1) //5,7,9,11,13,15,17,19
     // {
     //   StartRuns=11; NextRuns=14; Runs=100;
     // }
     // else // 6,8,10,12,14,16,18,20
     // {
        StartRuns=17; NextRuns=11+(MINU/8); Runs=255;
     // }
//      word w=(milli>>16); if (w&0x3){FanON;LightAndFan();}else{FanOFF;JustLight();} // 65*3 секунд откачиваем воду из листьев 65 секунд пауза для входа CO2 
        if (MINU&3) {FanON;LightAndFan();}else{FanOFF;JustLight();} // 65*3 секунд откачиваем воду из листьев 65 секунд пауза для входа CO2 
// почему вентилятор стоит несколько секунд?
//      FanOFF;JustLight();//} // 65*3 секунд откачиваем воду из листьев 65 секунд пауза для входа CO2 


//    StartRuns=1; NextRuns=1; Runs=255; Light(); // ~35мкс одиночные импульсы. 8.9мс х255 цикл
    }
  else { FanOFF;StartRuns=17; NextRuns=14; Runs=255;  C235();} // ночная смена [22..4) (6 часов + 2 рассвет/закат)
}

uint8_t count2s=0;
uint16_t tz,tw;

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

   if (HOUR>=24){ cli();timer0_millis-=86400000L;sei(); HOUR=0;MINU=0;SECU=0;}
  //  nextm=(HOUR+1)*3600000L; 
// 65 сек
//      word w=(milli>>16); if (w&0x3){FanON;LightAndFan();}else{FanOFF;JustLight();} 
//1 сек
//      byte b=(milli>>10); if ((b&0x3F)==0){delay(2000);} // каждые 65.5 секунд пауза в 2 секунды чтобы знали каково оно без света.
//      byte b=(milli>>12); if ((b&0x1F)==0){delay(4000);} // каждые 260 секунд пауза в 4 секунды чтобы знали каково оно без света.
    //  byte b=(milli>>11); 
  if ((!SECU)&&(MINU&1)){delay(1000);} // каждую нечетную минуту (120 секунд) пауза в 1 секунду чтобы знали каково оно без света.

//}
Shine();
 
    __asm__ __volatile__("rjmp Start\n\t");
}


