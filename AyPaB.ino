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
 
ISR(WDT_vect) { if(WDsleep){WDflag=1; WDsleep=0; } else{ reboot(); }} // Watchdog timer interrupt

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


void setup() 
{  

  ACSR|=(1<<ACD);// analog comparator off

//  PRR|=(1<<PRTWI)|(1<<PRTIM2)|(1<<PRTIM1)|(1<<PRSPI)|(1<<PRUSART0);
  PRR|=(1<<PRTWI)|(1<<PRTIM2)|(1<<PRTIM1)|(1<<PRSPI)|(1<<PRADC);
  /*
  PRR = (1<<PRTWI)     // turn off TWI
        | (1<<PRTIM2)    // turn off Timer/Counter2
        | (1<<PRTIM1)    // turn off Timer/Counter1 (leave Timer/Counter2 on)
        | (1<<PRSPI)     // turn off SPI
        | (1<<PRUSART0);  // turn off USART (will turn on again when reset)
   //     | (1<<PRTIM0)    // turn off Timer/Counter0
    //    | (1<<PRADC);    // turn off ADC
*/
  
  setup_watchdog(T2S); // если в течении 2s не сбросить сторожевого пса то перезагрузка. (защита от зависаний)
    
  
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
    
//    cli();timer0_millis=3600000L;sei();    // 1 ночи
//    cli();timer0_millis=36000000L;sei();    // 10 утра
//    cli();timer0_millis=21600000L;sei();    // 6 утра
//    cli();timer0_millis=43200000L;sei();    // полдень
// cli();timer0_millis=46800000L;sei();    // час дня

//    cli();timer0_millis=50400000L;sei();    // 2 часа дня
    cli();timer0_millis=54000000L;sei();    // 3 часа дня
//    cli();timer0_millis=61200000L;sei();    // 5 вечера
//    cli();timer0_millis=64800000L;sei();    // 6 вечера

//    cli();timer0_millis=71800000L;sei();    // почти 8 вечера
//    cli();timer0_millis=68400000L;sei();    // 7 вечера

//    cli();timer0_millis=72000000L;sei();    // 8 вечера
//    cli();timer0_millis=79200000L;sei();    // 10 вечера
//    cli();timer0_millis=82800000L;sei();    // 11 вечера
//    cli();timer0_millis=86200000L;sei();    // почти полночь


 CheckPowerSupply();CheckPowerSupply();
}

/*
void  delay500ns(void) __attribute__((noinline)); 
void  delay500ns(void) { __asm__ __volatile__( "delay500:\n\t"     "ret\n\t" );} 
void  delay750ns(void) __attribute__((noinline)); 
void  delay750ns(void) {  __asm__ __volatile__( "delay750:\n\t"   "nop\n\t""nop\n\t"  "nop\n\t""nop\n\t"  "ret\n\t" );} 
void  delay2500ns(void) __attribute__((noinline)); 
void  delay2500ns(void) {  __asm__ __volatile__( "delay2500:\n\t"   "nop\n\t""nop\n\t"  "nop\n\t""nop\n\t"  //750
"nop\n\t""nop\n\t"  "nop\n\t""nop\n\t" //1000
"nop\n\t""nop\n\t"  "nop\n\t""nop\n\t""nop\n\t""nop\n\t"  "nop\n\t""nop\n\t" //1500
"nop\n\t""nop\n\t"  "nop\n\t""nop\n\t""nop\n\t""nop\n\t"  "nop\n\t""nop\n\t" //2000
"nop\n\t""nop\n\t"  "nop\n\t""nop\n\t""nop\n\t""nop\n\t"  "nop\n\t""nop\n\t" //2500
"ret\n\t" );} */

uint8_t m1,m2,m3,m4,m5,m6,m7,m8;

void DayLight(void)
{
    
    PORTD=0b00000000; // all port D pins to low    
    DDRD=0b10111100; // set pins 23457 to output

  //  PORTB=0b00000000; // all port B pins to low    
    //DDRB=0b00001100; // set pins 23 to output

//    PORTC=0b00000000; // all port C pins to low    
  //  DDRC=0b00101100; // set pins 235 to output

  // какими портами светим в зависимости от текущего часа
//  if ((HOUR>=6)&&(HOUR<21)) // 15 часов
  //{
    //1-5 on 6-8 off
    m1=0b00000100; //2
    m2=0b00001000; //3
    m3=0b00010000; //4
    m4=0b00100000; //5
    m5=0b10000000; //7

//    m6=0b00000100; //2
  //  m7=0b00010000; //4
    //m8=0b00100000; //5
//  }// daytime shift 15 hrs
  //else
  //{
//  }// nighttime shift 9 hrs


    __asm__ __volatile__(

"lds r19,m1\n\t"  // загружаем маски светимостей
"lds r20,m2\n\t"
"lds r21,m3\n\t"
"lds r22,m4\n\t"
"lds r23,m5\n\t"
//"lds r24,m6\n\t"
//"lds r25,m7\n\t"
//"lds r26,m8\n\t"

"in r18,0xb\n\t"// port D
"or r19,r18\n\t"
"or r20,r18\n\t"
"or r21,r18\n\t"
"or r22,r18\n\t"
"or r23,r18\n\t"
"andi r18,0b01000011\n\t" // all OFF mask

//"in r27,0x08\n\t"// port C
//"or r24,r27\n\t"
//"or r25,r27\n\t"
//"or r26,r27\n\t"
//"andi r27,0b11010011\n\t" // all OFF mask


    "mov r1,r30\n\t" // r30=0
    "mov r1,r31\n\t" // r31=0
    
"555:\n\t"    
// port D
      "cli\n\t"  //1clk
      "out 0x0b,r19\n\t" // set pin 2 ON (1,2,3 are OFF) //1clk
      "push r18\n\t" "pop r18\n\t" "nop\n\t"     "nop\n\t" // 6clocks 375ns - долго открывается "увесистый" полевик IRLZ44. заряд 66нК.  (66нс при токе 1А) ULN2003? nope even more slow
      "out 0x0b,r20\n\t" // set pin 3 ON (0,2,3 are OFF) //1clk
      "push r18\n\t" "pop r18\n\t" "nop\n\t"     "nop\n\t" // 6clocks
      "out 0x0b,r21\n\t" // set pin 4 ON (0,1,3 are OFF) //1clk
      "push r18\n\t" "pop r18\n\t" "nop\n\t"     "nop\n\t" // 6clocks
       "out 0x0b,r22\n\t" // set pin 5 ON  //1clk
      "push r18\n\t" "pop r18\n\t" "nop\n\t"     "nop\n\t" // 6clocks
       "out 0x0b,r23\n\t" // set pin 7 ON  //1clk
 //     "push r18\n\t" "pop r18\n\t" "nop\n\t"     "nop\n\t" // 6clocks
//        "out 0x0b,r18\n\t" // all portd pins  OFF //1clk       -- 14clk

// pause in the middle
//      "sei\n\t" // 1clk
     //  "call delay500\n\t"
// port C
//      "cli\n\t" // 1clk
  //     "out 0x08,r24\n\t" // set pin 2 ON  //1clk
    //  "push r18\n\t" "pop r18\n\t" "nop\n\t"     "nop\n\t" // 6clocks
      // "out 0x08,r25\n\t" // set pin 3 ON  //1clk
//      "push r18\n\t" "pop r18\n\t" "nop\n\t"     "nop\n\t" // 6clocks
  //     "out 0x08,r26\n\t" // set pin 5 ON  //1clk
//      "push r18\n\t" "pop r18\n\t"  // 6clocks
"nop\n\t"     "nop\n\t"
"nop\n\t"   //  "nop\n\t"
        "adiw r30,1\n\t" // 2 clocks
     "sei\n\t"  //1clk
        "out 0x0b,r18\n\t" // all portd pins  OFF //1clk       -- 14clk
/*
"nop\n\t"     "nop\n\t"
"nop\n\t"     "nop\n\t"
"nop\n\t"     "nop\n\t"
"nop\n\t"     "nop\n\t"
"nop\n\t"     "nop\n\t"



"nop\n\t"     "nop\n\t"
"nop\n\t"     "nop\n\t"
"nop\n\t"     "nop\n\t"
"nop\n\t"     "nop\n\t"
"nop\n\t"     "nop\n\t"
*/
//        "out 0x08,r27\n\t" // all portb pins  OFF //1clk       -- 14clk
// pause in the end
//     "sei\n\t"  //1clk

//      "nop\n\t"    "nop\n\t"     "nop\n\t"    "nop\n\t"  //4clk  

//"nop\n\t"   //1 clk


//       "call delay750 \n\t"
   //    "call delay500\n\t"

//       "nop\n\t"     "nop\n\t"          "nop\n\t"     "nop\n\t"          "nop\n\t"     "nop\n\t"          "nop\n\t"     "nop\n\t"          // 500ns

//  "inc r30\n\t" // 1 clock
//  "sbrs r30,7\n\t" // следующая инструкция выполнится только если бит 7 в r30 сброшен (128 cycles)

  "brne 555b\n\t" // 2 clk if condition is true (not zero flag)   --6clk
      ); 
      // ~3.8 микросекунд цикл
  
}

void NightLight(void)
{
    
//    PORTD=0b00000000; // all port D pins to low    
  //  DDRD=0b10111100; // set pins 23457 to output

  //  PORTB=0b00000000; // all port B pins to low    
    //DDRB=0b00001100; // set pins 23 to output

    PORTC=0b00000000; // all port C pins to low    
    DDRC=0b00110100; // set pins 245 to output

  // какими портами светим в зависимости от текущего часа
//  if ((HOUR>=6)&&(HOUR<21)) // 15 часов
  //{
    //1-5 on 6-8 off
//    m1=0b00000100; //2
  //  m2=0b00001000; //3
    //m3=0b00010000; //4
//    m4=0b00100000; //5
  //  m5=0b10000000; //7

    m6=0b00000100; //2
    m7=0b00010000; //4
    m8=0b00100000; //5
//  }// daytime shift 15 hrs
  //else
  //{
//  }// nighttime shift 9 hrs


    __asm__ __volatile__(

//"lds r19,m1\n\t"  // загружаем маски светимостей
//"lds r20,m2\n\t"
//"lds r21,m3\n\t"
//"lds r22,m4\n\t"
//"lds r23,m5\n\t"
"lds r24,m6\n\t"
"lds r25,m7\n\t"
"lds r26,m8\n\t"

"in r27,0x08\n\t"// port C
"or r24,r27\n\t"
"or r25,r27\n\t"
"or r26,r27\n\t"
"andi r27,0b11010011\n\t" // all OFF mask


    "mov r1,r30\n\t" // r30=0
    "mov r1,r31\n\t" // r31=0
    
"555:\n\t"    
// port C
      "cli\n\t"  //1clk

    "nop\n\t"//   
      "push r18\n\t" "pop r18\n\t" "nop\n\t"     "nop\n\t" // 6clocks 375ns - долго открывается "увесистый" полевик IRLZ44. заряд 66нК.  (66нс при токе 1А) ULN2003? nope even more slow
    "nop\n\t"//   
      "push r18\n\t" "pop r18\n\t" "nop\n\t"     "nop\n\t" // 6clocks
  

       "out 0x08,r24\n\t" // set pin 2 ON  //1clk
      "push r18\n\t" "pop r18\n\t" "nop\n\t"     "nop\n\t" // 6clocks
       "out 0x08,r25\n\t" // set pin 3 ON  //1clk
      "push r18\n\t" "pop r18\n\t" "nop\n\t"     "nop\n\t" // 6clocks
       "out 0x08,r26\n\t" // set pin 5 ON  //1clk
//      "push r18\n\t" "pop r18\n\t"  // 6clocks


  
// port C
//      "cli\n\t" // 1clk
  //     "out 0x08,r24\n\t" // set pin 2 ON  //1clk
    //  "push r18\n\t" "pop r18\n\t" "nop\n\t"     "nop\n\t" // 6clocks
      // "out 0x08,r25\n\t" // set pin 3 ON  //1clk
//      "push r18\n\t" "pop r18\n\t" "nop\n\t"     "nop\n\t" // 6clocks
  //     "out 0x08,r26\n\t" // set pin 5 ON  //1clk
//      "push r18\n\t" "pop r18\n\t"  // 6clocks
"nop\n\t"     "nop\n\t"
"nop\n\t"   //  "nop\n\t"
        "adiw r30,1\n\t" // 2 clocks
     "sei\n\t"  //1clk
//        "out 0x0b,r18\n\t" // all portd pins  OFF //1clk       -- 14clk

        "out 0x08,r27\n\t" // all portb pins  OFF //1clk       -- 14clk
// pause in the end
//     "sei\n\t"  //1clk

//      "nop\n\t"    "nop\n\t"     "nop\n\t"    "nop\n\t"  //4clk  

//"nop\n\t"   //1 clk


//       "call delay750 \n\t"
   //    "call delay500\n\t"

//       "nop\n\t"     "nop\n\t"          "nop\n\t"     "nop\n\t"          "nop\n\t"     "nop\n\t"          "nop\n\t"     "nop\n\t"          // 500ns

//  "inc r30\n\t" // 1 clock
//  "sbrs r30,7\n\t" // следующая инструкция выполнится только если бит 7 в r30 сброшен (128 cycles)

  "brne 555b\n\t" // 2 clk if condition is true (not zero flag)   --6clk
      ); 
      // ~3.8 микросекунд цикл
  
}

void Shine(void)
{
    if ((HOUR>=6)&&(HOUR<21)) // 15 часов
  { DayLight(); } else { NightLight(); }
  

}

void ShineO(void)
{
    
    PORTD=0b00000000; // all port D pins to low    
    DDRD=0b10111100; // set pins 23457 to output

  //  PORTB=0b00000000; // all port B pins to low    
    //DDRB=0b00001100; // set pins 23 to output

    PORTC=0b00000000; // all port C pins to low    
    DDRC=0b00101100; // set pins 235 to output

  // какими портами светим в зависимости от текущего часа
//  if ((HOUR>=6)&&(HOUR<21)) // 15 часов
  //{
    //1-5 on 6-8 off
    m1=0b00000100; //2
    m2=0b00001000; //3
    m3=0b00010000; //4
    m4=0b00100000; //5
    m5=0b10000000; //7

    m6=0b00000100; //2
    m7=0b00001000; //3
    m8=0b00100000; //5
//  }// daytime shift 15 hrs
  //else
  //{
//  }// nighttime shift 9 hrs


    __asm__ __volatile__(

"lds r19,m1\n\t"  // загружаем маски светимостей
"lds r20,m2\n\t"
"lds r21,m3\n\t"
"lds r22,m4\n\t"
"lds r23,m5\n\t"
"lds r24,m6\n\t"
"lds r25,m7\n\t"
"lds r26,m8\n\t"

"in r18,0xb\n\t"// port D
"or r19,r18\n\t"
"or r20,r18\n\t"
"or r21,r18\n\t"
"or r22,r18\n\t"
"or r23,r18\n\t"
"andi r18,0b01000011\n\t" // all OFF mask

"in r27,0x08\n\t"// port C
"or r24,r27\n\t"
"or r25,r27\n\t"
"or r26,r27\n\t"
"andi r27,0b11010011\n\t" // all OFF mask


    "mov r1,r30\n\t" // r30=0
    "mov r1,r31\n\t" // r31=0
    
"555:\n\t"    
// port D
      "cli\n\t"  //1clk
      "out 0x0b,r19\n\t" // set pin 2 ON (1,2,3 are OFF) //1clk
      "push r18\n\t" "pop r18\n\t" "nop\n\t"     "nop\n\t" // 6clocks 375ns - долго открывается "увесистый" полевик IRLZ44. заряд 66нК.  (66нс при токе 1А) ULN2003? nope even more slow
      "out 0x0b,r20\n\t" // set pin 3 ON (0,2,3 are OFF) //1clk
      "push r18\n\t" "pop r18\n\t" "nop\n\t"     "nop\n\t" // 6clocks
      "out 0x0b,r21\n\t" // set pin 4 ON (0,1,3 are OFF) //1clk
      "push r18\n\t" "pop r18\n\t" "nop\n\t"     "nop\n\t" // 6clocks
       "out 0x0b,r22\n\t" // set pin 5 ON  //1clk
      "push r18\n\t" "pop r18\n\t" "nop\n\t"     "nop\n\t" // 6clocks
       "out 0x0b,r23\n\t" // set pin 7 ON  //1clk
      "push r18\n\t" "pop r18\n\t" "nop\n\t"     "nop\n\t" // 6clocks
        "out 0x0b,r18\n\t" // all portd pins  OFF //1clk       -- 14clk

// pause in the middle
//      "sei\n\t" // 1clk
     //  "call delay500\n\t"
// port C
//      "cli\n\t" // 1clk
       "out 0x08,r24\n\t" // set pin 2 ON  //1clk
      "push r18\n\t" "pop r18\n\t" "nop\n\t"     "nop\n\t" // 6clocks
       "out 0x08,r25\n\t" // set pin 3 ON  //1clk
      "push r18\n\t" "pop r18\n\t" "nop\n\t"     "nop\n\t" // 6clocks
       "out 0x08,r26\n\t" // set pin 5 ON  //1clk
//      "push r18\n\t" "pop r18\n\t"  // 6clocks
"nop\n\t"     "nop\n\t"
"nop\n\t"   //  "nop\n\t"
        "adiw r30,1\n\t" // 2 clocks
     "sei\n\t"  //1clk

        "out 0x08,r27\n\t" // all portb pins  OFF //1clk       -- 14clk
// pause in the end
//     "sei\n\t"  //1clk

//      "nop\n\t"    "nop\n\t"     "nop\n\t"    "nop\n\t"  //4clk  

//"nop\n\t"   //1 clk


//       "call delay750 \n\t"
   //    "call delay500\n\t"

//       "nop\n\t"     "nop\n\t"          "nop\n\t"     "nop\n\t"          "nop\n\t"     "nop\n\t"          "nop\n\t"     "nop\n\t"          // 500ns

//  "inc r30\n\t" // 1 clock
//  "sbrs r30,7\n\t" // следующая инструкция выполнится только если бит 7 в r30 сброшен (128 cycles)

  "brne 555b\n\t" // 2 clk if condition is true (not zero flag)   --6clk
      ); 
      // ~3.8 микросекунд цикл
  
}


void ShineOld(void)
{
  // выбор светимости в зависимости от текущего часа
  if ((HOUR>=6)&&(HOUR<20))
  {
    //1-5 on 6-8 off
    m1=0b00000100;
    m2=0b00001000;
    m3=0b00010000;
    m4=0b00100000;
    m5=0b00000001;
    m6=0;
    m7=0;
    m8=0;
  }// daytime shift 14 hrs
  else
  {
    m1=0;
    m2=0;
    m3=0;
    m4=0;
    m5=0;
    m6=0b00000010;
    m7=0b00000100;
    m8=0b00001000;

  }// nighttime shift 10 hrs

    __asm__ __volatile__(

"lds r19,m1\n\t"  // загружаем маски светимостей
"lds r20,m2\n\t"
"lds r21,m3\n\t"
"lds r22,m4\n\t"
"lds r23,m5\n\t"
"lds r24,m6\n\t"
"lds r25,m7\n\t"
"lds r26,m8\n\t"

"in r18,9\n\t"// port D
"or r19,r18\n\t"
"or r20,r18\n\t"
"or r21,r18\n\t"
"or r22,r18\n\t"
"andi r18,0b11000011\n\t" // all OFF mask

"in r27,3\n\t"// port B
"or r23,r27\n\t"
"or r24,r27\n\t"
"or r25,r27\n\t"
"or r26,r27\n\t"
"andi r27,0b11110000\n\t" // all OFF mask


//"mov r19,r18\n\t"
  //   "ori r19, 0b00000100\n\t" // bit 2 is ON
//"mov r20,r18\n\t"
  //   "ori r20, 0b00001000\n\t" // bit 3 is ON
//"mov r21,r18\n\t"
  //   "ori r21, 0b00010000\n\t" // bit 4 is ON
//"mov r22,r18\n\t"
  //   "ori r22, 0b00100000\n\t" // bit 5 is ON

//"mov r23,r18\n\t"
  //   "ori r23, 0b00000001\n\t" // bit 0 is ON
//"mov r24,r18\n\t"
  //   "ori r24, 0b00000010\n\t" // bit 1 is ON
//"mov r25,r18\n\t"
  //   "ori r25, 0b00000100\n\t" // bit 2 is ON
//"mov r26,r18\n\t"
  //   "ori r26, 0b00001000\n\t" // bit 3 is ON

/*
     "ldi r19, 0b00000001\n\t" // bit 0 is ON
     "ldi r20, 0b00000010\n\t" // bit 1 is ON
     "ldi r21, 0b00000100\n\t" // bit 2 is ON
     "ldi r22, 0b00001000\n\t" // bit 3 is ON
     "ldi r23, 0b00010000\n\t" // bit 4 is ON
     "ldi r24, 0b00100000\n\t" // bit 5 is ON
     "ldi r25, 0b01000000\n\t" // bit 6 is ON
     "ldi r26, 0b10000000\n\t" // bit 7 is ON
*/
     
    "mov r1,r30\n\t" // r30=0
    "mov r1,r31\n\t" // r31=0
    
"555:\n\t"    
// port D
      "cli\n\t"  //1clk
      "out 0x0b,r19\n\t" // set pin 2 ON (1,2,3 are OFF) //1clk
      "push r18\n\t" "pop r18\n\t" "nop\n\t"     "nop\n\t" // 6clocks 375ns - долго открывается "увесистый" полевик IRLZ44. заряд 66нК.  (66нс при токе 1А) ULN2003? nope even more slow
      "out 0x0b,r20\n\t" // set pin 3 ON (0,2,3 are OFF) //1clk
      "push r18\n\t" "pop r18\n\t" "nop\n\t"     "nop\n\t" // 6clocks
      "out 0x0b,r21\n\t" // set pin 4 ON (0,1,3 are OFF) //1clk
      "push r18\n\t" "pop r18\n\t" "nop\n\t"     "nop\n\t" // 6clocks
       "out 0x0b,r22\n\t" // set pin 5 ON  //1clk
      "push r18\n\t" "pop r18\n\t" "nop\n\t"     "nop\n\t" // 6clocks
        "out 0x0b,r18\n\t" // all portd pins  OFF //1clk       -- 14clk

// pause in the middle
//      "sei\n\t" // 1clk
     //  "call delay500\n\t"
// port B
//      "cli\n\t" // 1clk
      "out 0x05,r23\n\t" // set pin 0 ON (0,1,3 are OFF) //1clk
      "push r18\n\t" "pop r18\n\t" "nop\n\t"     "nop\n\t" // 6clocks
       "out 0x05,r24\n\t" // set pin 1 ON  //1clk
      "push r18\n\t" "pop r18\n\t" "nop\n\t"     "nop\n\t" // 6clocks
       "out 0x05,r25\n\t" // set pin 2 ON  //1clk
      "push r18\n\t" "pop r18\n\t" "nop\n\t"     "nop\n\t" // 6clocks
       "out 0x05,r26\n\t" // set pin 3 ON  //1clk
//      "push r18\n\t" "pop r18\n\t"  // 6clocks
"nop\n\t"     "nop\n\t"
"nop\n\t"   //  "nop\n\t"
        "adiw r30,1\n\t" // 2 clocks
     "sei\n\t"  //1clk

 
        "out 0x05,r27\n\t" // all portb pins  OFF //1clk       -- 14clk
// pause in the end
//     "sei\n\t"  //1clk

//      "nop\n\t"    "nop\n\t"     "nop\n\t"    "nop\n\t"  //4clk  

//"nop\n\t"   //1 clk


//       "call delay750 \n\t"
   //    "call delay500\n\t"

//       "nop\n\t"     "nop\n\t"          "nop\n\t"     "nop\n\t"          "nop\n\t"     "nop\n\t"          "nop\n\t"     "nop\n\t"          // 500ns

//  "inc r30\n\t" // 1 clock
//  "sbrs r30,7\n\t" // следующая инструкция выполнится только если бит 7 в r30 сброшен (128 cycles)

  "brne 555b\n\t" // 2 clk if condition is true (not zero flag)   --6clk
      ); 
      // ~3.8 микросекунд цикл
  
}

void Shine2(void)
{
    __asm__ __volatile__(
    
    

     "ldi r19, 0b00000001\n\t" // bit 0 is ON
     "ldi r20, 0b00000010\n\t" // bit 1 is ON
     "ldi r21, 0b00000100\n\t" // bit 2 is ON
     "ldi r22, 0b00001000\n\t" // bit 3 is ON
     "ldi r23, 0b00010000\n\t" // bit 4 is ON
     "ldi r24, 0b00100000\n\t" // bit 5 is ON
     "ldi r25, 0b01000000\n\t" // bit 6 is ON
     "ldi r26, 0b10000000\n\t" // bit 7 is ON

     
    "mov r1,r30\n\t" // r30=0
    "mov r1,r31\n\t" // r31=0
    
"555:\n\t"    
// port D
      "cli\n\t"  //1clk
      "out 0x0b,r21\n\t" // set pin 2 ON (1,2,3 are OFF) //1clk
      "push r18\n\t" "pop r18\n\t" "nop\n\t"     "nop\n\t" // 6clocks 375ns - долго открывается "увесистый" полевик IRLZ44. заряд 66нК.  (66нс при токе 1А) ULN2003? nope even more slow
      "out 0x0b,r22\n\t" // set pin 3 ON (0,2,3 are OFF) //1clk
      "push r18\n\t" "pop r18\n\t" "nop\n\t"     "nop\n\t" // 6clocks
      "out 0x0b,r23\n\t" // set pin 4 ON (0,1,3 are OFF) //1clk
      "push r18\n\t" "pop r18\n\t" "nop\n\t"     "nop\n\t" // 6clocks
       "out 0x0b,r24\n\t" // set pin 5 ON  //1clk
      "push r18\n\t" "pop r18\n\t" "nop\n\t"     "nop\n\t" // 6clocks
        "out 0x0b,r1\n\t" // all portd pins  OFF //1clk       -- 14clk

// pause in the middle
//      "sei\n\t" // 1clk
     //  "call delay500\n\t"
// port B
//      "cli\n\t" // 1clk
      "out 0x05,r19\n\t" // set pin 0 ON (0,1,3 are OFF) //1clk
      "push r18\n\t" "pop r18\n\t" "nop\n\t"     "nop\n\t" // 6clocks
     "out 0x05,r1\n\t" // all portb pins  OFF //1clk       -- 14clk
///       "out 0x05,r20\n\t" // set pin 1 ON  //1clk (5й выход короткий день 10 часов)
      "push r18\n\t" "pop r18\n\t" "nop\n\t"     "nop\n\t" // 6clocks
       "out 0x05,r21\n\t" // set pin 2 ON  //1clk
      "push r18\n\t" "pop r18\n\t" "nop\n\t"     "nop\n\t" // 6clocks
       "out 0x05,r22\n\t" // set pin 3 ON  //1clk
//      "push r18\n\t" "pop r18\n\t"  // 6clocks
"nop\n\t"     "nop\n\t"
"nop\n\t"   //  "nop\n\t"
        "adiw r30,1\n\t" // 2 clocks
     "sei\n\t"  //1clk

 
        "out 0x05,r1\n\t" // all portb pins  OFF //1clk       -- 14clk
// pause in the end
//     "sei\n\t"  //1clk

//      "nop\n\t"    "nop\n\t"     "nop\n\t"    "nop\n\t"  //4clk  

//"nop\n\t"   //1 clk


//       "call delay750 \n\t"
   //    "call delay500\n\t"

//       "nop\n\t"     "nop\n\t"          "nop\n\t"     "nop\n\t"          "nop\n\t"     "nop\n\t"          "nop\n\t"     "nop\n\t"          // 500ns

//  "inc r30\n\t" // 1 clock
//  "sbrs r30,7\n\t" // следующая инструкция выполнится только если бит 7 в r30 сброшен (128 cycles)

  "brne 555b\n\t" // 2 clk if condition is true (not zero flag)   --6clk
//  "brne 999b\n\t" // 2 clk if condition is true (not zero flag)   --6clk
  
  //     "sbrs r31,7\n\t" // следующая инструкция выполнится только если бит 7 в r31 сброшен (32768 cycles)
//      "rjmp 555b\n\t"
 
 
      ); // 32clk - 2 microseconds on 16MHz
      // 40clocks - 2.5 microseconds on  16MHz
      // 48clocks - 3 microseconds on  16MHz
      // ~5микросекунд цикл
  
}


uint8_t count2s=0;
uint16_t tz,tw;

void loop() {

    __asm__ __volatile__("Start:\n\t");
    __asm__ __volatile__("wdr\n\t");//  wdt_reset();



  ADCon; 
  ADMUX = _BV(REFS0) | _BV(REFS1) | _BV(MUX3);

//      cli();milli=timer0_millis;sei();  
  Shine(); //160.7ms
//  Shine();Shine();  Shine();  Shine();  Shine();  Shine();  Shine();  Shine();  Shine();
//        cli();long milli2=timer0_millis;sei();  

//            SerialON;  Serial.println(milli);  Serial.println(milli2);     Serial.println(" ");     delay(100);        SerialOFF;

  //delayMicroseconds(6000); // Wait for Vref to settle (если меньше то не успевает)

  ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|2; while (bit_is_set(ADCSRA,ADSC)); tz=ADCW; ADCoff; 



//    if (tz>384) {
    if (tz>396) {
            SerialON;
//     Serial.begin(9600);
  Serial.print("temp=");
  Serial.println(tz);   
        SerialOFF;
//  Serial.end();
//for( byte i=0;i<250;i++){  Serial.println(tz);     delay(100);}// 25 секунд задержка с миганием TX диода
for( byte i=0;i<250;i++){  __asm__ __volatile__("sbi 5,5\n\t");     delay(100); __asm__ __volatile__("cbi 5,5\n\t");  delay(100); }// 50 секунд задержка с миганием L диода

  //__asm__ __volatile__("sbi 5,5\n\t");   delay(5000);    //   __asm__ __volatile__("cbi 5,5\n\t"); delay(5000);    
 
    } 

    else{ 

      //Shine(); 
      Shine(); Shine(); Shine();// 1s// Shine(); Shine(); Shine(); Shine(); //2s

      
      
  

//Serial.begin(9600);
  //Serial.print("milli=");
  //Serial.println(milli);

//t=milli/1000;
  //Serial.print("t=");
 // Serial.println(t);

 //      __asm__ __volatile__("sbi 5,5\n\t");
 // delay(500);
   //    __asm__ __volatile__("cbi 5,5\n\t");
   //delay(500);    
//Shine();// 250ms 
//Shine();Shine();Shine();Shine();Shine();Shine();Shine();
 // delay(100);
//if(++count2s==3) // ~3 sec
//if(++count2s==60) // ~60 sec
if(++count2s==240) // ~4min
{
CheckPowerSupply();
  /*    SerialON;
  Serial.println(milli);
  Serial.println(HOUR);
   Serial.println(tz);
   Serial.println(vcc);
   delay(50);// flush buffer
//   Serial.end();
      SerialOFF;
*/
  count2s=0;
}

//    if (milli<1800000L){Shine10();}else{Shine();} // первые 30 минут светим в полсилы (в 2 раза реже)  1020lux(10)  68.3w 1120lux(5)   72.6w
   /*Serial.begin(9600);
  Serial.print("2: milli="); // 352-384
    Serial.println(milli);
    Serial.print(" t="); // 352-384
    Serial.println(t);
    */
 // Serial.end();
 // delay(1000);
  
//  if (milli<36000000L){Shine();}else{    Shine2();} // короткий 10-ти часовой день для проростов дынь
//Shine();
}

   // if (VH()>352) {
  //  Serial.println(VH()); // 352
   // }
    __asm__ __volatile__("rjmp Start\n\t");
}


