/* Example from Sandeep Mistry 
 * With mods from AliExpress/LilyGo docs
 * For TTGo ESP32 LoRa-OLED board
 * http://www.lilygo.cn/down_view.aspx?TypeId=11&Id=78&Fid=t14:11:14
 * Based on SX1276 LoRa Radio
 * http://www.semtech.com/apps/product.php?pn=SX1276
 * RMB 29Nov2017
 */

#include <SPI.h>
#include <LoRa.h>       // https://github.com/sandeepmistry/arduino-LoRa
#include <U8g2lib.h>   // https://github.com/olikraus/U8g2_Arduino
// #include <U8x8lib.h>
// DHT11
#include <Adafruit_Sensor.h>
#include <DHT.h>

#define OFF 0   // For LED
#define ON 1



// SPI LoRa Radio
#define LORA_SCK 5        // GPIO5 - SX1276 SCK
#define LORA_MISO 19     // GPIO19 - SX1276 MISO
#define LORA_MOSI 27    // GPIO27 -  SX1276 MOSI
#define LORA_CS 18     // GPIO18 -   SX1276 CS
#define LORA_RST 14   // GPIO14 -    SX1276 RST
#define LORA_IRQ 26  // GPIO26 -     SX1276 IRQ (interrupt request)

// I2C OLED Display works with SSD1306 driver
#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16

// DHT11
#define DHTTYPE DHT11
#define DHT_PIN 23

#define UV_PIN 34

/* Pick One. Hardware I2C does NOT work! This article helped: https://robotzero.one/heltec-wifi-kit-32/ 
* TTGo boards similar to Heltec boards, LED_BUILTIN = 2 instead of pin 25
* Some OLED displays don't handle ACK correctly so SW I2C works better. Thank you Olikraus!
* TTGo OLED has pin 16 reset unlike other OLED displays
*/

// UNCOMMENT one of the constructor lines below
//U8X8_SSD1306_128X64_NONAME_SW_I2C Display(/* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* reset=*/ OLED_RST); // Unbuffered, basic graphics, software I2C
//U8G2_SSD1306_128X64_NONAME_1_SW_I2C Display(U8G2_R0, /* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* reset=*/ OLED_RST); // Page buffer, SW I2C
U8G2_SSD1306_128X64_NONAME_F_SW_I2C Display(U8G2_R0, /* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* reset=*/ OLED_RST); // Full framebuffer, SW I2C

const int blueLED = LED_BUILTIN; 
int counter = 0;
//UV Variable
int uv=UV_PIN, lectura;

DHT dht(DHT_PIN, DHTTYPE);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("LoRa Sender");

  Display.begin();
  Display.enableUTF8Print();    // enable UTF8 support for the Arduino print() function
  Display.setFont(u8g2_font_ncenB10_tr);
  
  // Very important for SPI pin configuration!
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS); 
  
  // Very important for LoRa Radio pin configuration! 
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);         

  pinMode(blueLED, OUTPUT); // For LED feedback

  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  
  // The larger the spreading factor the greater the range but slower data rate
  // Send and receive radios need to be set the same
  LoRa.setSpreadingFactor(12); // ranges from 6-12, default 7 see API docs

  // Change the transmit power of the radio
  // Default is LoRa.setTxPower(17, PA_OUTPUT_PA_BOOST_PIN);
  // Most modules have the PA output pin connected to PA_BOOST, gain 2-17
  // TTGO and some modules are connected to RFO_HF, gain 0-14
  // If your receiver RSSI is very weak and little affected by a better antenna, change this!
  LoRa.setTxPower(14, PA_OUTPUT_RFO_PIN);
  
}

void loop() {
 //----------------------------------------------------------------------------------------
 lectura=analogRead(uv);
  float voltaje=lectura*(3.3/669);
  int longOnda=map(lectura,0,669,0,1023);
  int indice;
  if(longOnda<50){
    indice=0;
  }
  else if(longOnda<227){
    indice=1;
  }
  else if(longOnda<318){
    indice=2;
  }
  else if(longOnda<408){
    indice=3;
  }
  else if(longOnda<503){
    indice=4;
  }
  else if(longOnda<606){
    indice=5;
  }
  else if(longOnda<696){
    indice=6;
  }
  else if(longOnda<795){
    indice=7;
  }
  else if(longOnda<881){
    indice=8;
  }
  else if(longOnda<976){
    indice=9;
  }
  else{
    indice=10;
  }
  //----------------------------------------------------------------------------------------

  Serial.print("Sending packet: ");
  Serial.println(counter);


   // Read temperature and Humidity
  float t = dht.readTemperature();
  float h = dht.readHumidity();

  //Print Temperature and Humidity
  Serial.print("Temp:");
  Serial.println(t);
  Serial.print("Humidity:");
  Serial.println(h);
  
  //Print UV index
  Serial.println("Indice UV:"+String(indice)+" - Rango:"+String(longOnda));


  digitalWrite(blueLED, ON);  // Turn blue LED on
  
  // send packet
  LoRa.beginPacket();
  LoRa.print("HeLoRa! ");
  LoRa.print(counter);
  LoRa.print("Indice UV:");
  LoRa.print((String)indice);
  LoRa.print("- Rango:");
  LoRa.print(String(longOnda));
  LoRa.print("Temp");
  LoRa.print((String)t);
  LoRa.print("Humidity");
  LoRa.print((String)h);
  LoRa.endPacket();
  digitalWrite(blueLED, OFF); // Turn blue LED off

  // Display Info
  Display.clearBuffer();  
  Display.setCursor(0,12); Display.print("LoRa Sender");
  Display.setCursor(0,30); Display.print("Humi: " + (String)h);
  Display.setCursor(0,48); Display.print("Temp: " + (String)t);
 
  Display.sendBuffer();

  counter++;

  delay(5000);
}
