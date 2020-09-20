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
int nabiz_2;
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

  if (pulseSensor.sawStartOfBeat()) { //Eğer nabız algılarsak
    nabiz = pulseSensor.getBeatsPerMinute(); //Dakikadaki nabzı nabiz değişkenine kaydediyoruz.
    Serial.println("Nabız attı. ");
    Serial.print("BPM: ");
    
    if(nabiz < 100)
    {
      Serial.println(nabiz);
      Serial.println("100den kucuk");
      
      itoa(nabiz,nabiz_temp,10);
      
      nabiz_2_digit[0] = '0';
      nabiz_2_digit[1] = nabiz_temp[0];
      nabiz_2_digit[2] = nabiz_temp[1];
      
      Serial.println(nabiz_2_digit); //Dakikdaki nabız değerini aynıo zamanda bilgisayarımıza da gönderiyoruz.
      connection.print(nabiz_2_digit);
    } 
                           
    else
    {
      Serial.println(nabiz); //Dakikdaki nabız değerini aynıo zamanda bilgisayarımıza da gönderiyoruz.
      connection.print(nabiz);  
    }
    
  }
  delay(2000);
}
