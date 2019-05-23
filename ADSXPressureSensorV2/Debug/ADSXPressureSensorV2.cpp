#include "Energia.h"

float AddSmoothValue(float value);
void initTimer(void);
float MeasurePreassure();
void setup();
void loop();
void floatToString(char* string, float num);
void ISR(void);

#line 1 "D:/ProyectosCCS/ADSXPressureSensorV2/ADSXPressureSensorV2.ino"




















#include <inc/tm4c123gh6pm.h> 
#include <string.h>
#include <stdio.h>



#define PERIOD            4000000U         
#define CLOCK_FREQUENCY   80000000U   
#define PRESURE_MAX       30
#define PRESURE_MIN       0
#define MEASURE_INTERVAL  5UL

unsigned int timeDivisor = 5;
char str[6];                   

unsigned long count = 0;          

float dataReceived;
char incomingByte;   


bool presionEnviada;
unsigned int automatico;

unsigned int contadorSerial = 0;


const int analogInPin = A0; 

static const float ADCFULLSCALE = 4095; 
static const float reference_voltage_v = 3.3;     
static const float reference_voltage_mv = 3300.0;    
const int Pmax = 30;                              
const int Pmin = 0;                               
const int analogOutPin = BLUE_LED; 



double PressureValue[8];                      
double Pressures[1000];                       

const size_t arrayPressure_size = sizeof(PressureValue)/ sizeof(PressureValue[0]);
const size_t num_pressures_size = sizeof(Pressures) / sizeof(Pressures[0]);

bool getPressure = 0;


const size_t windowSize = 20;
float circularBuffer[windowSize];
float* circularBufferAccessor = circularBuffer;










float sum;
float mean;

float AddSmoothValue(float value) {
	static float elementCount = 0;
	if (circularBufferAccessor >= circularBuffer + windowSize) {
		circularBufferAccessor = circularBuffer;
	}
	sum -= *circularBufferAccessor;
	sum += value;
	*circularBufferAccessor = value;
	circularBufferAccessor++;

	if (elementCount < windowSize) {
		elementCount++;
	}

	return sum / elementCount;
}

void initTimer(void) {
	SYSCTL_RCGCTIMER_R |= 1;
	TIMER0_CTL_R &= ~(1 << 8); 
	TIMER0_CFG_R = 4;
	TIMER0_TBMR_R = 0b1010; 
	TIMER0_TBILR_R = PERIOD & 0xffff; 
	TIMER0_TBPR_R = PERIOD >> 16; 
	TIMER0_TBMATCHR_R = 100; 
	TIMER0_CTL_R |= (1 << 8); 
	GPIO_PORTF_AFSEL_R |= (1 << 1); 
	GPIO_PORTF_PCTL_R |= (7 << 4);
}


float MeasurePreassure() {
	float sensorValue = 0;                
	
	float voltage_v = 0.0;                   
	float voltage_mv = 0.0;
	float output_pressure = 0.0;                      
	float vacuum_pressure = 0.0; 

	
	sensorValue = (float) analogRead(analogInPin); 
	
	voltage_mv = (sensorValue * reference_voltage_mv) / ADCFULLSCALE;  
	voltage_v = voltage_mv / 1000;
	
	
	
	output_pressure = ( ( (voltage_v - (0.10 * (reference_voltage_mv/1000) )) * (Pmax - Pmin) ) / (0.8 * (reference_voltage_mv/1000) ) ) + Pmin;
	vacuum_pressure = output_pressure - 14.6; 
	return vacuum_pressure;

}



void setup() {
	timeDivisor = MEASURE_INTERVAL * ( CLOCK_FREQUENCY / PERIOD ); 
	
	Serial.begin(9600);
	analogReadResolution(12);    

	
	pinMode(RED_LED, OUTPUT);  
	pinMode(BLUE_LED, OUTPUT);
	initTimer();
	attachInterrupt(RED_LED, ISR, RISING);
	memset(str, 0, 16);  
}

void loop() {
	static float pressure = 0.0;
	static float presionActual = 0.0;
	presionActual = MeasurePreassure();

	if ( getPressure ) {
		pressure = AddSmoothValue(presionActual);                    

		if (pressure > PRESURE_MAX) {
			digitalWrite(BLUE_LED, HIGH);
		} else {
			digitalWrite(BLUE_LED, LOW);
		}

		getPressure = 0;
	}

	
	if (count == 0 && presionEnviada == 1) {
		if (automatico == 1) {
			Serial.write(timeDivisor);
			floatToString(str, pressure);
			Serial.print(str);
			Serial.print("\r\n");
		} else if (automatico == 2) {
			Serial.print("Pressure:");
			Serial.println(pressure);
			Serial.print("\n");
			Serial.print("PressureActual:");
			Serial.println(presionActual);
			Serial.print("\n");
		}
		presionEnviada = 0;
	}

	
	while (Serial.available() > 0) {
		
		incomingByte = Serial.read();

		switch (incomingByte) {

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
			timeDivisor = (int) ( dataReceived * (CLOCK_FREQUENCY / PERIOD) );
			break;

		case 'M':
			automatico = 2;
			break;

		default:
			break;
		}

	}

	

	
	
	
	
	
	
	
	
	
	
	


}





void floatToString(char* string, float num) {
	int valor = num * 100;
	for (int i = 0; i < 4; i++) {
		string[i] = (valor % 10) + 48;
		valor /= 10;
	}
	string[4] = '\n';
	string[5] = 0;
}










void ISR(void) {
	if (count == 0) {
		count = timeDivisor;
		digitalWrite(RED_LED, HIGH);
	}

	else {
		count--;
		digitalWrite(RED_LED, LOW);
	}
	presionEnviada = 1;
	getPressure = 1;

	


}




