/*
   -------------------------------------------------------------------------------------
   HX711_ADC
   Arduino library for HX711 24-Bit Analog-to-Digital Converter for Weight Scales
   Olav Kallhovd sept2017
   -------------------------------------------------------------------------------------
*/

/*
   This example file shows how to calibrate the load cell and optionally store the calibration
   value in EEPROM, and also how to change the value manually.
   The result value can then later be included in your project sketch or fetched from EEPROM.

   To implement calibration in your project sketch the simplified procedure is as follow:
       LoadCell.tare();
       //place known mass
       LoadCell.refreshDataSet();
       float newCalibrationValue = LoadCell.getNewCalibration(known_mass);
*/
#define USE_ARDUINO_INTERRUPTS true    // Pulse kütüphanesinin daha doğru ölçüm yapabilmesi için bu ayarı etkinleştiriyoruz
#include <PulseSensorPlayground.h> //Yazının başında bilgisayarımıza kurmuş olduğumuz Pulse Playground kütüphanesini ekliyoruz. 
#include <SoftwareSerial.h>


#include <HX711_ADC.h>
#include <EEPROM.h>

#include <stdio.h>
//pins:
const int HX711_dout = 4; //mcu > HX711 dout pin
const int HX711_sck = 5; //mcu > HX711 sck pin

//HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);
//Hc-06 conneciton
SoftwareSerial connection(8,7);//RX,TX

const int calVal_eepromAdress = 0;
long t;
char buf_weight[3];

//pulse sensor datas
const int PulseWire = 0; // Pulse sensörümüzü bağlamış olduğumuz Analog pinini belirliyoruz
int nabiz; //İçinde dakikadaki nabzı tutacağımız değişkeni oluşturuyoruz.
int Threshold = 510; // Yazının başında belirlemiş olduğumuz eşik değerini bu değişkene atıyoruz.
char nabiz_2_digit[1];
char nabiz_temp[2];
char total_data[7]; // include weight and pulse value  
int nabiz_2;
char final_data[2];
int flag = 0;
int counter = 0;
//Pulse Sensor Constructor
PulseSensorPlayground pulseSensor; //Sensörümüzü kodumuzda kullanabilmek için onu obje olarak oluşturuyoruz.

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
  
  pulseSensor.analogInput(PulseWire); //Pulse sensörünün bağlıu olduğu pini belirliyoruz.
  pulseSensor.blinkOnPulse(13);       //arduino üzerindeki ledin nabzımızla yanıp sönmesini istediğimizi söylüyoruz.
  pulseSensor.setThreshold(Threshold); //Değişkene atamış olduğumuz eşik değerini uyguluyoruz.

  if (pulseSensor.begin()) {
    Serial.println("Pulse sensörü objesini yarattık."); 
  } //Pulse sensörü başarıyla başlatılırsa bilgisayara mesaj gönderioyoruz.

  
  total_data[0] = '0';
  total_data[1] = '0';
  total_data[2] = '0';
  total_data[3] = '/';
  total_data[4] = '0';
  total_data[5] = '0';
  total_data[6] = '0';

}

void loop() {
  static boolean newDataReady = 0;
  const int serialPrintInterval = 0; //increase value to slow down serial print activity


  if (pulseSensor.sawStartOfBeat()) { //Eğer nabız algılarsak
    nabiz = pulseSensor.getBeatsPerMinute(); //Dakikadaki nabzı nabiz değişkenine kaydediyoruz.
    Serial.println("Nabız attı. ");
    Serial.print("BPM: ");
    
    if(nabiz < 100)
    {
      Serial.println(nabiz);
      if(nabiz > 50){
          itoa(nabiz,nabiz_temp,10);
          total_data[0] = '0';
          total_data[1] = nabiz_temp[0];
          total_data[2] = nabiz_temp[1];
          flag = 1;
          //Serial.println(nabiz_2_digit); //Dakikdaki nabız değerini aynıo zamanda bilgisayarımıza da gönderiyoruz.
          //connection.print(nabiz_2_digit);
      }         
    } 
                           
    else
    {
      Serial.println(nabiz); //Dakikdaki nabız değerini aynı zamanda bilgisayarımıza da gönderiyoruz.
      if(nabiz < 120){
          itoa(nabiz,nabiz_temp,10);
          total_data[0] = nabiz_temp[0];
          total_data[1] = nabiz_temp[1];
          total_data[2] = nabiz_temp[2];
          flag=1;
          //connection.print(nabiz);  
          //connection.print(deneme);    
      }
      
    }
    
  }
  total_data[3] = '/';
  // check for new data/start next conversion:
  if (LoadCell.update()) newDataReady = true;

  // get smoothed value from the dataset:
  if (newDataReady) {
    Serial.println("Yeni data hazır");
    if (millis() > t + serialPrintInterval) {
      float i = LoadCell.getData();
      int temp_i = i/1000.0;
      int intweight = (int)temp_i;
      int kgweight = intweight;
      Serial.print("Kilo: ");
      Serial.println(intweight);

      Serial.print("Kilo KG : ");
      Serial.println(kgweight);
      itoa(kgweight, buf_weight, 10);
      if (kgweight < 10){
          total_data[4] = '0';
          total_data[5] = '0';
          total_data[6] = buf_weight[0];
      }
      else if (kgweight >= 10 && kgweight < 100){
          total_data[4] = '0';
          total_data[5] = buf_weight[0];
          total_data[6] = buf_weight[1];
      } 
      //Serial.print("Kilo1: ");
      //Serial.println(buf_weight[0]);
      //Serial.print("Kilo2: ");
      //Serial.println(buf_weight[1]);
      //Serial.print("Kilo3: ");
      //Serial.println(buf_weight[2]);
      else {
        total_data[4] = buf_weight[0];
        total_data[5] = buf_weight[1];
        total_data[6] = buf_weight[2];  
      }    
     
      
      Serial.print("Load_cell output val: ");
      Serial.println(i);
      newDataReady = 0;
      t = millis();
    }
    else{
      Serial.println("Else'e girdim");
      //itoa(mogint,mogData,10);
      //total_data[4] = mogData[0];
      //total_data[5] = mogData[1];
      //total_data[6] = mogData[2];
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

  final_data[0] = total_data[0];
  final_data[1] = total_data[1];
  final_data[2] = total_data[2];
  final_data[3] = total_data[3];
  final_data[4] = total_data[4];
  final_data[5] = total_data[5];
  final_data[6] = total_data[6];
  final_data[7] = '\0';
  Serial.print("Final Data:");
  Serial.println(final_data);

  if(flag == 1){
    counter++;
  } 

  if (counter >= 15){
      total_data[0] = '0';
      total_data[1] = '0';
      total_data[2] = '0';
      flag = 0;
      counter = 0;
  }
  connection.print(final_data);
  
  delay(400);
}

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
