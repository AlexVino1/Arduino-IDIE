//BMP280
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp280;
//DHT11
#include <DHT.h>
DHT dht(8, DHT11);
//OLED 
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);
//Bluetooth
#include <SoftwareSerial.h>
SoftwareSerial mySerial(6, 7);  
// переменная для интервала измерений
#define INTERVAL_GET_DATA 2000  // интервала измерений, мс
unsigned long millis_int1=0;
int pos=0;
int cont_ = 1;

void setup() {
  //инициализация
 while (!bmp280.begin(BMP280_ADDRESS - 1)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); 
  display.display();
  display.clearDisplay();
  delay(2000);
  display.print(F("Could not find a valid BMP280 sensor, check wiring!"));
  }
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); 
  display.display();
  delay(2000);
  dht.begin();
  Serial.begin(9600);
  mySerial.begin(9600);
  

  }

void loop() {
   //Считывание данных
   int humidity = dht.readHumidity();
   int temp = dht.readTemperature();
   float pressure = bmp280.readPressure();
   int altitude = bmp280.readAltitude();
  //Вывод на дисплей
   display.clearDisplay();
   display.setTextSize(1);
   display.setTextColor(WHITE);
   display.setCursor(0,0);
  //Передача на Bluetooth
   if(millis()-millis_int1 >= INTERVAL_GET_DATA) {
      pos=1-pos;
      if(pos==0)  {
        // вывод в монитор последовательного порта
		mySerial.print("=============================================")
        mySerial.print("Плотность= ");mySerial.print((pressure*29)/(8.31*(temp+273)));mySerial.println("kq/m^3");

		//Вывод на дисплей
		 display.clearDisplay();
		 display.setCursor(0,0);
		 display.print("q= ");display.print((pressure*29)/(8.31*(temp+273)));display.println("kq/m^3");
		 display.print(F("Pressure= "));display.print(pressure);display.println(" Pa");
		 display.print(F("Altitude= "));display.print(altitude);display.println(" m");
		 display.display(); 
      }
      else  {
        // вывод в монитор последовательного порта
        mySerial.print("Температура=");mySerial.println(temp);
         mySerial.print(F("Давление = "));mySerial.print(pressure);mySerial.println(" Pa");
         mySerial.print(F("Высота = "));mySerial.print(altitude);mySerial.println(" m");
         mySerial.print("Влажность=");mySerial.println(humidity);
		 //Вывод на дисплей
		 display.clearDisplay();
		 display.setCursor(0,0);
		 display.print("Temperature: ");display.println(temp);
		 display.print("Humidity: ");display.println(humidity);
		 display.display(); 
   }

      // старт интервала отсчета
      millis_int1=millis();
   }
}
