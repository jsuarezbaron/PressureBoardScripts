#include "Energia.h"

int appendToBuffer(int value);
float AddSmoothValue(int value);
void initTimer(void);
float MeasurePreassure();
void setup();
void loop();
void floatToString(char* string, float num );
void ISR(void);

#line 1 "D:/ProyectosCCS/ADSXPressureSensorV2/ADSXPressureSensorV2.ino"





















#include <inc/tm4c123gh6pm.h> 
#include <string.h>
#include <stdio.h>




#define PERIOD            80000000U         
#define CLOCK_FREQUENCY   80000000U   
#define PRESURE_MAX       30
#define PRESURE_MIN       0
#define MEASURE_INTERVAL  5

unsigned int timeDivisor = 5;

char str[6];                   

unsigned long count = 0;          

 
unsigned long timer = 0;        
long loopTime = 5000;   


float dataReceived;
char incomingByte;   


unsigned int presionEnviada;
unsigned int automatico;

unsigned int contadorSerial = 0;

double PressureValue[8];                      
double Pressures[1000];                       

const size_t arrayPressure_size = sizeof(PressureValue) / sizeof(PressureValue[0]);
const size_t num_pressures_size = sizeof(Pressures) / sizeof(Pressures[0]);


const int windowSize = 30;
int circularBuffer[windowSize];
int* circularBufferAccessor = circularBuffer;










int appendToBuffer(int value){
  *circularBufferAccessor = value;
  circularBufferAccessor ++;
  if(circularBufferAccessor  >= circularBuffer + windowSize){
    circularBufferAccessor = circularBuffer;
  }
}

long sum;
float mean;

float AddSmoothValue(int value){
  static int elementCount = 0;
  sum -= *circularBufferAccessor;
  sum += value;
  appendToBuffer(value);

  if(elementCount < windowSize){
    elementCount ++;
    return (float) sum / (float)elementCount;
  }
}


void initTimer(void)
{
  SYSCTL_RCGCTIMER_R |= 1;
  TIMER0_CTL_R &= ~(1 << 8); 
  TIMER0_CFG_R  = 4; 
  TIMER0_TBMR_R = 0b1010; 
  TIMER0_TBILR_R = PERIOD & 0xffff; 
  TIMER0_TBPR_R = PERIOD >> 16; 
  TIMER0_TBMATCHR_R = 100; 
  TIMER0_CTL_R |= (1 << 8); 
  GPIO_PORTF_AFSEL_R |= (1 << 1); 
  GPIO_PORTF_PCTL_R |= (7 << 4);
}



float MeasurePreassure(){

  const int analogInPin = A0;                        
  
  static const float ADCFULLSCALE = 4095;           
  static const float reference_voltage_v = 3.3;     
  const int Pmax = 30;                              
  const int Pmin = 0;                               
  const int analogOutPin = BLUE_LED;                
  float sensorValue = 0;                            
  int outputValue = 0;                              
  float voltage_mv = 0.0;                           
  float voltage_v = 0.0;                            
  float output_pressure = 0.0;                      
  float vacuum_pressure = 0.0;                      

  
  sensorValue = (float)analogRead(analogInPin);                      
  voltage_v = (sensorValue * reference_voltage_v) / ADCFULLSCALE;   
  outputValue = map(sensorValue, 0, 4095, 0, 1023);                 
  analogWrite(analogOutPin, outputValue);                             
  output_pressure = ( ( (voltage_v - (0.10 * (reference_voltage_v) )) * (Pmax - Pmin) ) / (0.8 * (reference_voltage_v) ) ) + Pmin;
  vacuum_pressure = output_pressure - 14.6; 
  return vacuum_pressure;
  
}



void setup() {
  timeDivisor = ( MEASURE_INTERVAL * CLOCK_FREQUENCY ) / PERIOD;               
  
  Serial.begin(9600); 
  analogReadResolution(12);                     
  
  
  pinMode(RED_LED, OUTPUT);  
  pinMode(BLUE_LED,OUTPUT);
  initTimer();
  attachInterrupt(RED_LED, ISR, RISING);
  memset(str, 0, 16);  
}



void loop() {
  static unsigned int countOld = 0;
  
  static float pressure = 0.0;
  
  if(count > countOld || countOld > count ){
    countOld = count; 
    pressure = MeasurePreassure();                    

    if( pressure > PRESURE_MAX ){
      digitalWrite(BLUE_LED, HIGH);
    }
    else{
      digitalWrite(BLUE_LED, LOW);
    }
  }

  
   if( count == 0 &&  presionEnviada == 0 && automatico == 1){
	  Serial.write(timeDivisor);
	  floatToString(str, pressure);
	  Serial.print(str);
	  Serial.print("\r\n");
	  presionEnviada = 1;
  }

   
   while (Serial.available() > 0) {
	   
	  incomingByte = Serial.read();

	  switch(incomingByte){

	  	  case '\n':
	  		  break;

	  	  case 'A':
	  		  automatico = 1;
	  		  break;

	  	 
	  	  case 'B':
			  automatico = 0;
			  break;

	  	  case -1:
	  		  
	  		  Serial.write("\Error\r\n");
	  		  break;

	  	  case 'P':
	  		  floatToString(str, pressure);
	  		  Serial.print(str);
	  		  Serial.print("\r\n");
	  		  break;

	  	  case 'T':
	  		  dataReceived = Serial.parseFloat();
	  		  timeDivisor = (int)(( dataReceived * CLOCK_FREQUENCY ) / PERIOD );
	  		  break;

	  	  default:
	  		  break;
	  }

  }

      
      

      
      
      
      
      
      
      
      
      
      
      

   
      
 }







void floatToString(char* string, float num ){
  int valor = num *100;
  for( int i = 0; i < 4; i++){
     string[i] = (valor % 10) + 48;
     valor /= 10;
  }
  string[4] = '\n';
  string[5] = 0;
}

  









void ISR(void)
{
	if (count == 0){
		count = timeDivisor;
	}

	else{
		count--;
	}
  presionEnviada = 0;
  


}






