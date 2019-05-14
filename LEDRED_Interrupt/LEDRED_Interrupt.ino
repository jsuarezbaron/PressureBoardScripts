// Try to hack a timer interrupt without rewriting the energia source files
// The plan: Attach an interrupt to an IOPIN that is driven by the timer. 
// Hopefully this will allow us to get a periodic interrupt
// PF1 drives the RED LED.  This can be driven by T0CCP1
#include <inc/tm4c123gh6pm.h>
#define LED RED_LED
// Period of 80000 should yield an interrupt rate of 1kHz
// This divisor will have to be spread across the match register and prescaler
#define PERIOD 80000  
void initTimer(void)
{
  SYSCTL_RCGCTIMER_R |= 1;
  TIMER0_CTL_R &= ~(1 << 8); // disable timer B
  TIMER0_CFG_R  = 4; 
  TIMER0_TBMR_R = 0b1010; // set T1AMS, T0MR=0
  TIMER0_TBILR_R = PERIOD & 0xffff; // set interval (lower 16 bits) 
  TIMER0_TBPR_R = PERIOD >> 16; // set interval (upper bits)
  TIMER0_TBMATCHR_R = 100; //  set match value
  TIMER0_CTL_R |= (1 << 8); // enable timer B
  GPIO_PORTF_AFSEL_R |= (1 << 1); // select alternate function for PF1 
  GPIO_PORTF_PCTL_R |= (7 << 4);
}
int Count = 0;
void ISR(void)
{
  Count++;
}
void setup() {                
  // initialize the digital pin as an output.
  pinMode(LED, OUTPUT); 
     
  Serial.begin(9600);
  initTimer();
  attachInterrupt(LED, ISR, RISING);
}
// the loop routine runs over and over again forever:
void loop() {
/ Try to hack a timer interrupt without rewriting the energia source files
// The plan: Attach an interrupt to an IOPIN that is driven by the timer. 
// Hopefully this will allow us to get a periodic interrupt
// PF1 drives the RED LED.  This can be driven by T0CCP1
#include <inc/tm4c123gh6pm.h>
#define LED RED_LED
// Period of 80000 should yield an interrupt rate of 1kHz
// This divisor will have to be spread across the match register and prescaler
#define PERIOD 80000  
void initTimer(void)
{
  SYSCTL_RCGCTIMER_R |= 1;
  TIMER0_CTL_R &= ~(1 << 8); // disable timer B
  TIMER0_CFG_R  = 4; 
  TIMER0_TBMR_R = 0b1010; // set T1AMS, T0MR=0
  TIMER0_TBILR_R = PERIOD & 0xffff; // set interval (lower 16 bits) 
  TIMER0_TBPR_R = PERIOD >> 16; // set interval (upper bits)
  TIMER0_TBMATCHR_R = 100; //  set match value
  TIMER0_CTL_R |= (1 << 8); // enable timer B
  GPIO_PORTF_AFSEL_R |= (1 << 1); // select alternate function for PF1 
  GPIO_PORTF_PCTL_R |= (7 << 4);
}
int Count = 0;
void ISR(void)
{
  Count++;
}
void setup() {                
  // initialize the digital pin as an output.
  pinMode(LED, OUTPUT); 
     
  Serial.begin(9600);
  initTimer();
  attachInterrupt(LED, ISR, RISING);
}
// the loop routine runs over and over again forever:
void loop() {
  int snapshot; 
  Count=0;
  delay(1000);                 // wait for a second
  snapshot=Count;              // check to see how far the count got in that time (should be 1000 for 1 ms timebase)
  Serial.println(snapshot);    // report it back  
}
}
