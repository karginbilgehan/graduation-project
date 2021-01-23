#define USE_ARDUINO_INTERRUPTS true   
#include <PulseSensorPlayground.h>  
#include <SoftwareSerial.h>


#include <HX711_ADC.h>
#include <EEPROM.h>

#include <stdio.h>
//pins
const int HX711_dout = 4; //mcu > HX711 dout pin
const int HX711_sck = 5; //mcu > HX711 sck pin

//HX711 constructor
HX711_ADC LoadCell(HX711_dout, HX711_sck);
//Hc-06 conneciton
SoftwareSerial connection(8,7);//RX,TX

const int calVal_eepromAdress = 0;
long t;
char bufWeight[3];

//pulse sensor data
const int PulseWire = 0; // analog pin for pulse sensor
int pulse; 
int Threshold = 510; 
char tempPulse[3];
char totalData[7]; // contains weight and pulse value  
char finalData[7]; // contains data to be sent to STM32 
int flag = 0; // to check if the pulse reading is present
int counterForMeasurementRange = 0;

PulseSensorPlayground pulseSensor; // to create a pulse object

void setup() {
  Serial.begin(57600); delay(10);
  Serial.println();
  Serial.println("Starting...");

  LoadCell.begin();
  long stabilizingtime = 2000; // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = false; //set this to false if you don't want tare to be performed in the next step**
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    LoadCell.setCalFactor(1.0); // user set calibration value (float), initial value 1.0 may be used for this sketch
    Serial.println("Startup is complete");
  }
  while (!LoadCell.update());
  calibrate(); //start calibration procedure

  connection.begin(9600);
  
  pulseSensor.analogInput(PulseWire); //pin for pulse sensor
  pulseSensor.blinkOnPulse(13);       
  pulseSensor.setThreshold(Threshold); 

  if (pulseSensor.begin()) {
    Serial.println("Pulse sensor crated."); 
  } 
  
  //initialize total data 
  totalData[0] = '0';
  totalData[1] = '0';
  totalData[2] = '0';
  totalData[3] = '/';
  totalData[4] = '0';
  totalData[5] = '0';
  totalData[6] = '0';

}

void loop() {
  static boolean newDataReady = 0;
  const int serialPrintInterval = 0; //increase value to slow down serial print activity

  // TO GET PULSE VALUE
  if (pulseSensor.sawStartOfBeat()) { 
    pulse = pulseSensor.getBeatsPerMinute(); 
    Serial.print("BPM: ");
    Serial.print(pulse);

    if(pulse < 100)
    {
      Serial.println(pulse);
      if(pulse > 50){
          itoa(pulse,tempPulse,10);
          totalData[0] = '0';
          totalData[1] = tempPulse[0];
          totalData[2] = tempPulse[1];
          flag = 1;
      }         
    } 
                           
    else
    {
      Serial.println(pulse);
      if(pulse < 120){
          itoa(pulse,tempPulse,10);
          totalData[0] = tempPulse[0];
          totalData[1] = tempPulse[1];
          totalData[2] = tempPulse[2];
          flag=1;  
      }     
    }
    
  }
  totalData[3] = '/';
  // check for new data/start next conversion:
  if (LoadCell.update()) newDataReady = true;

  // get smoothed value from the dataset:
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      float tempWeight = LoadCell.getData();
      int realWeight = (int)(tempWeight/1000.0);
      
      // FOR DEBUGGING;
      Serial.print("Real weight : ");
      Serial.println(realWeight);
      
      itoa(realWeight, bufWeight, 10);
      if (realWeight < 10){
          totalData[4] = '0';
          totalData[5] = '0';
          totalData[6] = bufWeight[0];
      }
      else if (realWeight >= 10 && realWeight < 100){
          totalData[4] = '0';
          totalData[5] = bufWeight[0];
          totalData[6] = bufWeight[1];
      } 
      else {
        totalData[4] = bufWeight[0];
        totalData[5] = bufWeight[1];
        totalData[6] = bufWeight[2];  
      }    
     
      // FOR DEBUGGING
      Serial.print("Load_cell output val: ");
      Serial.println(tempWeight);

      newDataReady = 0;
      t = millis();

    }

    else{
      // Nothing to do.
    }
  
  }

  // receive command from serial terminal
  if (Serial.available() > 0) {
    float i;
    char inByte = Serial.read();
    if (inByte == 't') LoadCell.tareNoDelay(); //tare
    else if (inByte == 'r') calibrate(); //calibrate
    else if (inByte == 'c') changeSavedCalFactor(); //edit calibration value manually
  }

  // check if last tare operation is complete
  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare complete");
  }

  finalData[0] = totalData[0];
  finalData[1] = totalData[1];
  finalData[2] = totalData[2];
  finalData[3] = totalData[3];
  finalData[4] = totalData[4];
  finalData[5] = totalData[5];
  finalData[6] = totalData[6];
  finalData[7] = '\0';

  // FOR DEBUGGING
  Serial.print("Final Data:"); 
  Serial.println(finalData);

  if(flag == 1){
    counterForMeasurementRange++;
  } 

  if (counterForMeasurementRange >= 15){
      totalData[0] = '0';
      totalData[1] = '0';
      totalData[2] = '0';
      flag = 0;
      counterForMeasurementRange = 0;
  }
  connection.print(finalData); // To send data from ESP32 to STM32
  delay(400);
}

/**
 * It needs to calibrate HX711 load cells.
 */
void calibrate() {
  Serial.println("***");
  Serial.println("Start calibration:");
  Serial.println("Place the load cell an a level stable surface.");
  Serial.println("Remove any load applied to the load cell.");
  Serial.println("Send 't' from serial monitor to set the tare offset.");

  boolean _resume = false;
  while (_resume == false) {
    LoadCell.update();
    if (Serial.available() > 0) {
      if (Serial.available() > 0) {
        float i;
        char inByte = Serial.read();
        if (inByte == 't') LoadCell.tareNoDelay();
      }
    }
    if (LoadCell.getTareStatus() == true) {
      Serial.println("Tare complete");
      _resume = true;
    }
  }

  Serial.println("Now, place your known mass on the loadcell.");
  Serial.println("Then send the weight of this mass (i.e. 100.0) from serial monitor.");

  float known_mass = 0;
  _resume = false;
  while (_resume == false) {
    LoadCell.update();
    if (Serial.available() > 0) {
      known_mass = Serial.parseFloat();
      if (known_mass != 0) {
        Serial.print("Known mass is: ");
        Serial.println(known_mass);
        _resume = true;
      }
    }
  }

  LoadCell.refreshDataSet(); //refresh the dataset to be sure that the known mass is measured correct
  float newCalibrationValue = LoadCell.getNewCalibration(known_mass); //get the new calibration value

  Serial.print("New calibration value has been set to: ");
  Serial.print(newCalibrationValue);
  Serial.println(", use this as calibration value (calFactor) in your project sketch.");
  Serial.print("Save this value to EEPROM adress ");
  Serial.print(calVal_eepromAdress);
  Serial.println("? y/n");

  _resume = false;
  while (_resume == false) {
    if (Serial.available() > 0) {
      char inByte = Serial.read();
      if (inByte == 'y') {
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.begin(512);
#endif
        EEPROM.put(calVal_eepromAdress, newCalibrationValue);
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.commit();
#endif
        EEPROM.get(calVal_eepromAdress, newCalibrationValue);
        Serial.print("Value ");
        Serial.print(newCalibrationValue);
        Serial.print(" saved to EEPROM address: ");
        Serial.println(calVal_eepromAdress);
        _resume = true;

      }
      else if (inByte == 'n') {
        Serial.println("Value not saved to EEPROM");
        _resume = true;
      }
    }
  }

  Serial.println("End calibration");
  Serial.println("***");
  Serial.println("To re-calibrate, send 'r' from serial monitor.");
  Serial.println("For manual edit of the calibration value, send 'c' from serial monitor.");
  Serial.println("***");
}

void changeSavedCalFactor() {
  float oldCalibrationValue = LoadCell.getCalFactor();
  boolean _resume = false;
  Serial.println("***");
  Serial.print("Current value is: ");
  Serial.println(oldCalibrationValue);
  Serial.println("Now, send the new value from serial monitor, i.e. 696.0");
  float newCalibrationValue;
  while (_resume == false) {
    if (Serial.available() > 0) {
      newCalibrationValue = Serial.parseFloat();
      if (newCalibrationValue != 0) {
        Serial.print("New calibration value is: ");
        Serial.println(newCalibrationValue);
        LoadCell.setCalFactor(newCalibrationValue);
        _resume = true;
      }
    }
  }
  _resume = false;
  Serial.print("Save this value to EEPROM adress ");
  Serial.print(calVal_eepromAdress);
  Serial.println("? y/n");
  while (_resume == false) {
    if (Serial.available() > 0) {
      char inByte = Serial.read();
      if (inByte == 'y') {
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.begin(512);
#endif
        EEPROM.put(calVal_eepromAdress, newCalibrationValue);
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.commit();
#endif
        EEPROM.get(calVal_eepromAdress, newCalibrationValue);
        Serial.print("Value ");
        Serial.print(newCalibrationValue);
        Serial.print(" saved to EEPROM address: ");
        Serial.println(calVal_eepromAdress);
        _resume = true;
      }
      else if (inByte == 'n') {
        Serial.println("Value not saved to EEPROM");
        _resume = true;
      }
    }
  }
  Serial.println("End change calibration value");
  Serial.println("***");
}
