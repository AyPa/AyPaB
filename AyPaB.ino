//#include <avr/pgmspace.h>

//#define Pin2Input(port,pin){port&=~(1<<pin);}
//#define Pin2Output(port,pin){port|=(1<<pin);}
//#define Pin2HIGH(port,pin){port|=(1<<pin);}
//#define Pin2LOW(port,pin){port&=~(1<<pin);}
//#define NOP __asm__ __volatile__ ("nop\n\t")


extern uint32_t long timer0_millis;
uint32_t milli;

void (* reboot) (void) = 0; //declare reset function @ address 0

#define SetADC(bandgap,input,us){ ADCSRA|=(1<<ADEN);delayMicroseconds(2);ADMUX=(bandgap<<REFS1)|(1<<REFS0)|(0<<ADLAR)|input;delayMicroseconds(us);} // input (0..7,8,14) (bg/vcc analogReference )
#define ADCoff{ ADCSRA&=~(1<<ADEN); }
#define mRawADC(v,p) { ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|p;do{}while(bit_is_set(ADCSRA,ADSC));v=(ADCH<<8)|ADCL; }
//#define mRawADC(v,p) ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|p;do{}while(bit_is_set(ADCSRA,ADSC));v=ADCW; 

byte volatile WDsleep;
byte volatile WDflag;

// Watchdog timeout values : //#define T16MS 0//#define T32MS 1//#define T64MS 2//#define T128MS 3//#define T250MS 4//#define T500MS 5//#define T1S 6
#define T2S 7//#define T4S 8//#define T8S 9
#define setup_watchdog(timeout){cli(); __asm__ __volatile__("wdr\n\t"); MCUSR&=~(1<<WDRF);WDTCSR|=(1<<WDCE)|(1<<WDE);WDTCSR=((1<<WDIE)|timeout);sei();}
 
ISR(WDT_vect) { if(WDsleep){WDflag=1; WDsleep=0; } else{ reboot(); }} // Watchdog timer interrupt


void setup() 
{  
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
    
    
    PORTD=0b00000000; // all port D pins to low    
//    DDRD=0b11111111; // all port D pins to output
    DDRD=0b00111100; // pins 2 3 4 5

    PORTB=0b00000000; // all port B pins to low    
//    DDRB=0b00101111; // pins 0 1 2 3 and 5(led)
    DDRB=0b00101111; // pins 0 1 2 3 and 5(led)

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

    SetADC(1,8,500);  //  select temperature sensor 352
//Serial.begin(9600);
}

word VH(void) // чтение внутреннего датчика температуры (atmega328P)
{
  word t1;
    
//    ADCSRA|=(1<<ADEN); //turn on ADC    
//    SetADC(1,8,500);  //  select temperature sensor 352 (need calibration)  
//    mRawADC(t1,2);
//    mRawADC(t1,2);
    
//    SetADC(0,2,500);

    mRawADC(t1,2);
    mRawADC(t1,2);

  //  ADCoff;
//    ADCSRA&=~(1<<ADEN); //turn off ADC 
//    ACSR = (1<<ACD); // turn off analog comparator    
    return t1;
}

void  delay500ns(void) __attribute__((noinline)); 
void  delay500ns(void) { __asm__ __volatile__( "delay500:\n\t"     "ret\n\t" );} 
//void  delay750ns(void) __attribute__((noinline)); 
//void  delay750ns(void) {  __asm__ __volatile__( "delay750:\n\t"   "nop\n\t""nop\n\t"  "nop\n\t""nop\n\t"  "ret\n\t" );} 
void  delay2500ns(void) __attribute__((noinline)); 
void  delay2500ns(void) {  __asm__ __volatile__( "delay2500:\n\t"   "nop\n\t""nop\n\t"  "nop\n\t""nop\n\t"  //750
"nop\n\t""nop\n\t"  "nop\n\t""nop\n\t" //1000
"nop\n\t""nop\n\t"  "nop\n\t""nop\n\t""nop\n\t""nop\n\t"  "nop\n\t""nop\n\t" //1500
"nop\n\t""nop\n\t"  "nop\n\t""nop\n\t""nop\n\t""nop\n\t"  "nop\n\t""nop\n\t" //2000
"nop\n\t""nop\n\t"  "nop\n\t""nop\n\t""nop\n\t""nop\n\t"  "nop\n\t""nop\n\t" //2500
"ret\n\t" );} 

void Shine(void)
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
       "out 0x05,r20\n\t" // set pin 1 ON  //1clk
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

void loop() {
  uint16_t t;
    __asm__ __volatile__("Start:\n\t");
    __asm__ __volatile__("wdr\n\t");//  wdt_reset();

 // 352
 t=VH();
    if (t<385) { 
//    cli();milli=timer0_millis;sei();    
//    if (milli<1800000L){Shine10();}else{Shine();} // первые 30 минут светим в полсилы (в 2 раза реже)  1020lux(10)  68.3w 1120lux(5)   72.6w
Shine();
} else {

Serial.begin(9600);
    Serial.println(t); // 352-384
  Serial.end();
 
  SetADC(1,8,500);  //  select temperature sensor 352
       __asm__ __volatile__("sbi 5,5\n\t");
  delay(500);
       __asm__ __volatile__("cbi 5,5\n\t");
   delay(500);    
 }

   // if (VH()>352) {
  //  Serial.println(VH()); // 352
   // }
    __asm__ __volatile__("rjmp Start\n\t");
}


