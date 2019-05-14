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


/*
 * This version includes Serial Port Python Communication
 * /


/**** Constants and variables ***/

////////////////const int analogInPin = A0;                   // Analog input pin (PE_3) that the asdx pressure sensor is attached to
/*const int ADCFULLSCALE = 4095;                // ADC full scale of Tiva C is 12-bit (0-4095)
const float reference_voltage_mv = 3300.0;    // Vcc is 5000mv (5V) for  ASDXACX030PAAA5
const int Pmax = 30;                          // 30psi for ASDXACX030PAAA5
const int Pmin = 0;                           // 0 psi FOR ASDXACX030PAAA5
const int analogOutPin = BLUE_LED;            // Analog output pin that the LED is attached to
int sensorValue = 0;                          // value read from the pressure sensor
int outputValue = 0;                          // value output to the PWM (analog out)
float voltage_mv = 0.0;                       // pressure sensor voltage in mV
float voltage_v = 0.0;                        // pressure sensor voltage in volts
float output_pressure = 0.0;                  // output pressure in psi
float vacuum_pressure = 0.0;                  // (substract atmospheric pressure from output_preesure) // vacuum pressure in psi*/


double PressureValue[8];                      //Pressure array
double Pressures[1000];                       //Pressure array


const size_t array_size = sizeof(PressureValue) / sizeof(PressureValue[0]);
const size_t num_pressures = sizeof(Pressures) / sizeof(Pressures[0]);

/* */ 
unsigned long timer = 0;
long loopTime = 5000;   // microseconds


/***************Median Filter*****************************/
const int windowSize = 5;
int circularBuffer[windowSize];
int* circularBufferAccessor = circularBuffer;


int values[1000];

int getMeasure(){
  size_t static index = 0;
  index++;
  return values[index - 1];
}

int appendToBuffer(int value){
  *circularBufferAccessor = value;
  circularBufferAccessor ++;
  if(circularBufferAccessor  >= circularBuffer + windowSize){
    circularBufferAccessor = circularBuffer;
  }
}

long sum;
int elementCount;
float mean;

float AddValue(int value){
  sum -= *circularBufferAccessor;
  sum += value;
  appendToBuffer(value);

  if(elementCount < windowSize){
    elementCount ++;
    return (float) sum / elementCount;
  }
}

/********************************timeSync**********************************/

/*
 * timeSync(unsigned long deltaT):
 * 
 */


void timeSync(unsigned long deltaT)
{
  unsigned long currTime = micros();
  long timeToDelay = deltaT - (currTime - timer);
  if (timeToDelay > 5000)
  {
    delay(timeToDelay / 1000);
    delayMicroseconds(timeToDelay % 1000);
  }
  else if (timeToDelay > 0)
  {
    delayMicroseconds(timeToDelay);
  }
  else
  {
      // timeToDelay is negative so we start immediately
  }
  timer = currTime + timeToDelay;
}

/*****************************************************************/
float MeasurePreassure(){
  const int analogInPin = A0;                   // Analog input pin (PE_3) that the asdx pressure sensor is attached to
  analogReadResolution(12);                     // change the resolution to 12 bits and read A0

  /* Constants to help conver the raw analogue measurement into  pressure in psi */
  const int ADCFULLSCALE = 4095;                // ADC full scale of Tiva C is 12-bit (0-4095)
  const float reference_voltage_mv = 3300.0;    // Vcc is 5000mv (5V) for  ASDXACX030PAAA5
  const int Pmax = 30;                          // 30psi for ASDXACX030PAAA5
  const int Pmin = 0;                           // 0 psi FOR ASDXACX030PAAA5
  const int analogOutPin = BLUE_LED;            // Analog output pin that the LED is attached to
  int sensorValue = 0;                          // value read from the pressure sensor
  int outputValue = 0;                          // value output to the PWM (analog out)
  float voltage_mv = 0.0;                       // pressure sensor voltage in mV
  float voltage_v = 0.0;                        // pressure sensor voltage in volts
  float output_pressure = 0.0;                  // output pressure in psi
  float vacuum_pressure = 0.0;                  // (substract atmospheric pressure from output_preesure) // vacuum pressure in psi

    
  /* read the analog in value: */
  sensorValue = analogRead(analogInPin);                             // digital value of pressure sensor voltage
  voltage_mv = (sensorValue * reference_voltage_mv) / ADCFULLSCALE;  // pressure sensor voltage in mV
  voltage_v = voltage_mv / 1000;

  // map it to the range of the analog out:
  outputValue = map(sensorValue, 0, 4095, 0, 1023); 
  //Blink led
  analogWrite(analogOutPin,outputValue);

  output_pressure = ( ( (voltage_v - (0.10 * (reference_voltage_mv/1000) )) * (Pmax - Pmin) ) / (0.8 * (reference_voltage_mv/1000) ) ) + Pmin;
  vacuum_pressure = output_pressure - 14.6; // subtract atmospheric pressure

  return vacuum_pressure;
  
}

/******************************************************************/

void setup() {
  /* initialize serial communications at 9600 bps: */
  Serial.begin(115200); 
  timer = micros();

}

void loop() {

  timeSync(loopTime);
 /* // change the resolution to 12 bits and read A0
  analogReadResolution(12);                                           //

  // read the analog in value:
  sensorValue = analogRead(analogInPin);                             // digital value of pressure sensor voltage
  voltage_mv = (sensorValue * reference_voltage_mv) / ADCFULLSCALE;  // pressure sensor voltage in mV
  voltage_v = voltage_mv / 1000;

  // map it to the range of the analog out:
  outputValue = map(sensorValue, 0, 4095, 0, 1023); 

  //Blink led

  analogWrite(analogOutPin,outputValue);

  // print the results to the serial monitor:
  Serial.print("Voltage = ");
  Serial.println(voltage_v);
  //Serial.print("\r\n");
  double output_pressure = ( ( (voltage_v - (0.10 * (reference_voltage_mv/1000) )) * (Pmax - Pmin) ) / (0.8 * (reference_voltage_mv/1000) ) ) + Pmin;
  vacuum_pressure = output_pressure - 14.6; // subtract atmospheric pressure */

  /******************************Pressure array****************************/
  float pressure = 0;
  pressure = MeasurePreassure();
  /***************Send serial Data**************/
  /*for (int i = 0; i < 300;  i++){
    Pressures[i] = pressure;
    Serial.println(Pressures[i]);
    Serial.print("\t\n");
  }*/

  Serial.print("\t PressureAp = ");
  Serial.println(pressure);
  
 /**********************************************/
  // wait 10 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  //////delay(500);     

  delay(500);
  //sendToPC(&output_pressure);
}


void sendToPC(int* data)
{
  byte* byteData = (byte*)(data);
  Serial.write(byteData, 2);
}
 
void sendToPC(double* data)
{
  byte* byteData = (byte*)(data);
  Serial.write(byteData, 4);
}
