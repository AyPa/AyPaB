//macro

#define Pin2Input(port,pin){port&=~(1<<pin);}
#define Pin2Output(port,pin){port|=(1<<pin);}
#define Pin2HIGH(port,pin){port|=(1<<pin);}
#define Pin2LOW(port,pin){port&=~(1<<pin);}
  //\
//#if (state==INPUT) port|=(1<<pin);\
//#elif port&=~(1<<pin);\
//#endif}

#define NOP __asm__ __volatile__ ("nop\n\t")
#define wait1us NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP; // 1 microsecond delay on a 20MHz Arduino
//#define wait500ns NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP; // 500ns delay on a 16MHz Arduino
//#define wait250ns NOP;NOP;NOP;NOP; // 250ns delay on a 16MHz Arduino
//#define wait125ns NOP;NOP; // 125ns delay on a 16MHz Arduino
//#define wait63ns NOP; // 63ns delay on a 16MHz Arduino


// LED overdriving  3-5w leds can go 3..5 times rated current fo 10ms 20%duty  (even x100 1us 0.11duty)
// +24v толстый кабель идет к лампам и разветвляется на 4 или больше ламп.
// управляющие транзисторы вынесены на отдельную плату как можно ближе к лампам. - даже в лампе!!! - к нему только подходит управляющий сигнал. 
// земля также очень толстая и идет одним кабелем от контроллера к лампам

// сам контроллер у коробок с лампами. к нему +5в,+3.3в желательно,+24в,общий провод.

