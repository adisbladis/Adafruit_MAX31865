#include <Arduino.h>
#include <Adafruit_MAX31865.h>

/* 
   This example is proposed to use the Adafruit MAX31865 library with 3 different reading modes:
    1) ONE_SHOT_MODE is the simplest one: the RTD bias is enabled, a conversion occurs and the bias is then disabled. 
        depending on the frequency filer (50Hz/60Hz) the reading rate varies between 75ms and 85ms. 
        the code is a blocking code, hence the Arduino will not exacute other task between reading.
        For multiple reading, you will get total conversion time equal to (CONVERSION TIME(75ms or 85ms) x N RTDS)

    2) ASYNCHRONOUS_MODE: similar to the ONE_SHOT_MODE, the RTD bias is enabled, a conversion occurs and the bias is then disabled.
        this mode use an asynchronous read function in order to avoid code blocking. The MCU can execute other actions while waiting 
        the RTD reading to be completed.
        This mode require more coding in the main loop.
        For multiple reading, you will get total conversion time equal to (CONVERSION TIME(75ms or 85ms) )

    3) CONTINUOUS_MODE: in this mode the bias is enabled all the time. This will causeself-heating of the RTD and influence the temperatre measurement. 
        The conversion time is however much faster (20ms). 
         For multiple reading, you will get total conversion time equal to (CONVERSION TIME(20ms) )
*/


//#define ONE_SHOT_MODE //Allow to read all rtds in t = 76ms x n rtds
//#define ASYNCHRONOUS_MODE //Allow to read all RTDs within 76ms
#define CONTINUOUS_MODE   //  conversion rate = 20ms per RTDs, but will cause self-heating 

#define FILTER_50HZ // if you are in a country where AC voltage operates at 50Hz (Europe, AUstralia,most African countries...)

#define THERMO_0_CS_PIN 9  //4-wires RTD
#define THERMO_1_CS_PIN 10 //4-wires RTD
#define THERMO_2_CS_PIN 11 //3-wires RTD
#define THERMO_3_CS_PIN 12 //3-wires RTD


const byte thermo_arrayLen = 4;
Adafruit_MAX31865 thermo_array[thermo_arrayLen] ={  
                                      Adafruit_MAX31865(THERMO_0_CS_PIN),
                                      Adafruit_MAX31865(THERMO_1_CS_PIN), 
                                      Adafruit_MAX31865(THERMO_2_CS_PIN), 
                                      Adafruit_MAX31865(THERMO_3_CS_PIN)                                     
                                     };
 
uint16_t rtd[thermo_arrayLen];
bool condition[thermo_arrayLen]={0};
float temperature[thermo_arrayLen];
float time0; //for conversion timing
long delta_t = 0; //for copnversion timing

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0

void setup() {
    Serial.begin(9600);
    while(!Serial); // ! This is block the programm for debugging purpose (arduino micro)
    Serial.println("Start...");   
   //configure the RTDs
    thermo_array[0].begin(MAX31865_4WIRE);
    thermo_array[1].begin(MAX31865_4WIRE);
    thermo_array[2].begin(MAX31865_3WIRE);
    thermo_array[3].begin(MAX31865_3WIRE);
    #ifdef FILTER_50HZ
        for(byte i=0;i<thermo_arrayLen;i++){
        thermo_array[i].enable50Hz(true);
        }
    #endif
    #ifdef CONTINUOUS_MODE //continuous mode requires more care
        for(byte i=0;i<thermo_arrayLen;i++){
        thermo_array[i].autoConvert(true);
            delay(20); //Note 5 
        thermo_array[i].enableBias(true); //must be enabled for continuous measurements
            delay(10); //Note 4
        }
    #endif
}

void loop() {
//  ONE SHOT MODE IS THE ORIGINAL COMPUTATION PROPOSED BY ADAFRUIT  
  #ifdef ONE_SHOT_MODE
    for(byte i=0;i<thermo_arrayLen;i++){
      float temperature = thermo_array[i].temperature(RNOMINAL,RREF);
      Serial.print(temperature,4);Serial.print("\t");
    }
    //display loop time = conversion time
    delta_t = millis()-time0;
    Serial.print("reading rate:");Serial.println(delta_t);
    time0=millis();
  #endif

// ASYNCHRONOUS MODE ALLOW TO SPEED UP THE READING OF MULTIPLE RTDs
  #ifdef ASYNCHRONOUS_MODE
    for(byte i=0;i<thermo_arrayLen;i++){
        condition[i]=thermo_array[i].readRTDAsync(rtd[i]);
        if(condition[i]) { 
          temperature[i]=thermo_array[i].temperatureAsync(rtd[i],RNOMINAL,RREF);
                
                //reading rate taken on thermo_array[2] time asssuming there is no bug
                // this is not the loop time
                if(condition[2]) { 
                    delta_t = millis()-time0;
                    time0=millis();
                }    
        }
          Serial.print(temperature[i],4);Serial.print("\t");  
    }
    Serial.print("reading rate:");Serial.println(delta_t);
    Serial.println();
  #endif

// CONTINUOUS MODE
  #ifdef CONTINUOUS_MODE
    for(byte i=0;i<thermo_arrayLen;i++){
        temperature[i] = thermo_array[i].temperature(RNOMINAL,RREF);
        Serial.print(temperature[i],4);Serial.print("\t");
    }
      delta_t = millis()-time0;
      time0=millis();    
      Serial.print("reading rate:");
      Serial.println(delta_t);
  #endif
}

