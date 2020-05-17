#define USE_ARDUINO_INTERRUPTS true    // Pulse kütüphanesinin daha doğru ölçüm yapabilmesi için bu ayarı etkinleştiriyoruz
#include <PulseSensorPlayground.h> //Yazının başında bilgisayarımıza kurmuş olduğumuz Pulse Playground kütüphanesini ekliyoruz. 
#include <SoftwareSerial.h>


SoftwareSerial connection(8,7);//RX,TX
char msg[3];
int numb = 1;
const int PulseWire = 0; // Pulse sensörümüzü bağlamış olduğumuz Analog pinini belirliyoruz
int nabiz; //İçinde dakikadaki nabzı tutacağımız değişkeni oluşturuyoruz.
int Threshold = 510; // Yazının başında belirlemiş olduğumuz eşik değerini bu değişkene atıyoruz.
char nabiz_2_digit[1];
char nabiz_temp[2];
char total_data[7]; // include weight and pulse value  
int nabiz_2;
char final_data[4];
PulseSensorPlayground pulseSensor; //Sensörümüzü kodumuzda kullanabilmek için onu obje olarak oluşturuyoruz.

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);
  connection.begin(9600);
  pulseSensor.analogInput(PulseWire); //Pulse sensörünün bağlıu olduğu pini belirliyoruz.
  pulseSensor.blinkOnPulse(13);       //arduino üzerindeki ledin nabzımızla yanıp sönmesini istediğimizi söylüyoruz.
  pulseSensor.setThreshold(Threshold); //Değişkene atamış olduğumuz eşik değerini uyguluyoruz.

  if (pulseSensor.begin()) {
    Serial.println("Pulse sensörü objesini yarattık."); 
  } //Pulse sensörü başarıyla başlatılırsa bilgisayara mesaj gönderioyoruz.
}

void loop() {
  // put your main code here, to run repeatedly:
  //connection.print(numb);
  //numb ++;
  //Serial.print("numb:");
  //Serial.println(numb);
  total_data[0] = '0';
  total_data[1] = '0';
  total_data[2] = '0';
  total_data[3] = '0';
  total_data[4] = '0';
  total_data[5] = '0';
  total_data[6] = '0';
  
  if (pulseSensor.sawStartOfBeat()) { //Eğer nabız algılarsak
    nabiz = pulseSensor.getBeatsPerMinute(); //Dakikadaki nabzı nabiz değişkenine kaydediyoruz.
    Serial.println("Nabız attı. ");
    Serial.print("BPM: ");
    
    if(nabiz < 100)
    {
      Serial.println(nabiz);
      itoa(nabiz,nabiz_temp,10);
      total_data[0] = '0';
      total_data[1] = nabiz_temp[0];
      total_data[2] = nabiz_temp[1];
      //Serial.println(nabiz_2_digit); //Dakikdaki nabız değerini aynıo zamanda bilgisayarımıza da gönderiyoruz.
      //connection.print(nabiz_2_digit);
      
    } 
                           
    else
    {
      Serial.println(nabiz); //Dakikdaki nabız değerini aynı zamanda bilgisayarımıza da gönderiyoruz.
      itoa(nabiz,nabiz_temp,10);
      total_data[0] = nabiz_temp[0];
      total_data[1] = nabiz_temp[1];
      total_data[2] = nabiz_temp[2];
      //connection.print(nabiz);  
      //connection.print(deneme);
    }
    
  }
  total_data[3] = '0';
  total_data[4] = '0';
  total_data[5] = '8';
  total_data[6] = '3';

  final_data[0] = total_data[0];
  final_data[1] = total_data[1];
  final_data[2] = total_data[2];
  final_data[3] = total_data[3];
  final_data[4] = total_data[4];
  final_data[5] = total_data[5];
  final_data[6] = total_data[6];
  
  Serial.println(final_data);
  connection.print(final_data);
  delay(2000);
}
