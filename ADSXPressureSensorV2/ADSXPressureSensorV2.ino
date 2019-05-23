/*ASDXPressureSensor
   Description:
   Reads an analog input pin. converts and calculates the corresponding
   output pressure. Also prints the results to the serial monitor.
*/

/*
 * The circuit:
 * ASDX pressure sensor pin2 connected to analog pin 0 (PE_3).
 * Pin 1 connected to Vcc (+5v for  ASDXACX030PAAA5) 30 psi, ps: +5 Vdc, Transfer Function limits: A --> 10% to 90%
 * Pin 3 connected to GND
 * The other pins are not connected
 */

/*
   Adapted 22 September 2018 by Juan Carlos Suárez
*/


/* This version includes Serial Port Python Communication */

#include <inc/tm4c123gh6pm.h> 
#include <string.h>
#include <stdio.h>


// Period of 80000 should yield an interrupt rate of 1kHz
// This divisor will have to be spread across the match register and prescaler
#define PERIOD            80000000U         //8000000U for 1 sec
#define CLOCK_FREQUENCY   80000000U   //MCU FREQ
#define PRESURE_MAX       30
#define PRESURE_MIN       0
#define MEASURE_INTERVAL  5

unsigned int timeDivisor = 5;

char str[6];                   //String to send via SerialPort

unsigned long count = 0;          //Count variable

/* */ 
unsigned long timer = 0;        //timer
long loopTime = 5000;   // microseconds


float dataReceived;
char incomingByte;   // for incoming serial data

/* Flags for continuos data send */
unsigned int presionEnviada;
unsigned int automatico;

unsigned int contadorSerial = 0;

double PressureValue[8];                      //Pressure array
double Pressures[1000];                       //Pressure array

const size_t arrayPressure_size = sizeof(PressureValue) / sizeof(PressureValue[0]);
const size_t num_pressures_size = sizeof(Pressures) / sizeof(Pressures[0]);

/***************Median Filter*****************************/
const int windowSize = 30;
int circularBuffer[windowSize];
int* circularBufferAccessor = circularBuffer;

/*
int values[1000];

int getMeasure(){
  size_t static index = 0;
  index++;
  return values[index - 1];
}*/

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


/*****************************************************************/
float MeasurePreassure(){

  const int analogInPin = A0;                        // Analog input pin (PE_3) that the asdx pressure sensor is attached to
  /* Constants to help conver the raw analogue measurement into  pressure in psi */
  static const float ADCFULLSCALE = 4095;           // ADC full scale of Tiva C is 12-bit (0-4095)
  static const float reference_voltage_v = 3.3;     // Vcc is 3300mv (5V) for  
  const int Pmax = 30;                              // 30psi for ASDXACX030PAAA5
  const int Pmin = 0;                               // 0 psi FOR ASDXACX030PAAA5
  const int analogOutPin = BLUE_LED;                // Analog output pin that the LED is attached to
  float sensorValue = 0;                            // value read from the pressure sensor
  int outputValue = 0;                              // value output to the PWM (analog out)
  float voltage_mv = 0.0;                           // pressure sensor voltage in mV
  float voltage_v = 0.0;                            // pressure sensor voltage in volts
  float output_pressure = 0.0;                      // output pressure in psi
  float vacuum_pressure = 0.0;                      // (substract atmospheric pressure from output_preesure) // vacuum pressure in psi

  /* read the analog in value: */
  sensorValue = (float)analogRead(analogInPin);                      // digital value of pressure sensor voltage
  voltage_v = (sensorValue * reference_voltage_v) / ADCFULLSCALE;   // pressure sensor voltage in V
  outputValue = map(sensorValue, 0, 4095, 0, 1023);                 // map it to the range of the analog out:
  analogWrite(analogOutPin, outputValue);                             //Blink led
  output_pressure = ( ( (voltage_v - (0.10 * (reference_voltage_v) )) * (Pmax - Pmin) ) / (0.8 * (reference_voltage_v) ) ) + Pmin;
  vacuum_pressure = output_pressure - 14.6; // subtract atmospheric pressure
  return vacuum_pressure;
  
}

/******************************************************************/

void setup() {
  timeDivisor = ( MEASURE_INTERVAL * CLOCK_FREQUENCY ) / PERIOD;               // Calcular el periodo de envío de cada medida de Presión
  /* initialize serial communications at 9600 bps: */
  Serial.begin(9600); 
  analogReadResolution(12);                     // change the resolution to 12 bits and read A0
  
  //timer = micros();
  pinMode(RED_LED, OUTPUT);  // initialize the digital pin as an output.
  pinMode(BLUE_LED,OUTPUT);
  initTimer();
  attachInterrupt(RED_LED, ISR, RISING);
  memset(str, 0, 16);  //Escribir 0 en los pirmeros 16 bytes de str
}



void loop() {
  static unsigned int countOld = 0;
  //timeSync(loopTime);
  static float pressure = 0.0;
  
  if(count > countOld || countOld > count ){
    countOld = count; 
    pressure = MeasurePreassure();                    //rawMeasure

    if( pressure > PRESURE_MAX ){
      digitalWrite(BLUE_LED, HIGH);
    }
    else{
      digitalWrite(BLUE_LED, LOW);
    }
  }////

  /* Continuos send Pressure*/
   if( count == 0 &&  presionEnviada == 0 && automatico == 1){
	  Serial.write(timeDivisor);
	  floatToString(str, pressure);
	  Serial.print(str);
	  Serial.print("\r\n");
	  presionEnviada = 1;
  }

   // send data only when you receive data:
   while (Serial.available() > 0) {
	   // read the incoming byte:
	  incomingByte = Serial.read();

	  switch(incomingByte){

	  	  case '\n':
	  		  break;

	  	  case 'A':
	  		  automatico = 1;
	  		  break;

	  	 /*Deshabilita medici�n continua */
	  	  case 'B':
			  automatico = 0;
			  break;

	  	  case -1:
	  		  //Serial.print("\Error\r\n");
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

      
      //////////////////////////sprintf(str, "C:%d\n", contadorSerial++);

      ///puts(str);
      //Serial.print("Count:");
      //Serial.println(count);
      /////Serial.println( ( (float)count * (float)PERIOD) / (float)CLOCK_FREQUENCY );
      //////Serial.print("\t");
      ////Serial.print("Pressure:");
      ///Serial.println(med);
      //Serial.print(str);
      //Serial.print("\n");
      //////////////////Serial.print("Count:");
      ////////////////Serial.println(count);
//    }
   
      
 }


/*
 *
 * */


void floatToString(char* string, float num ){
  int valor = num *100;
  for( int i = 0; i < 4; i++){
     string[i] = (valor % 10) + 48;
     valor /= 10;
  }
  string[4] = '\n';
  string[5] = 0;
}

  /*int snapshot; 
  count=0;
  delay(1000);                 // wait for a second
  snapshot=count;              // check to see how far the count got in that time (should be 1000 for 1 ms timebase)
  Serial.println(snapshot);    // report it back  
  */


/*******************************ISR****************/

void ISR(void)
{
	if (count == 0){
		count = timeDivisor;
	}

	else{
		count--;
	}
  presionEnviada = 0;
  /*
  Serial.print("Count:");
  Serial.println(count);*/
}



