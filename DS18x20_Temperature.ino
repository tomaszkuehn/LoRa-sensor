#include <Arduino.h>
#include <SoftwareSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <esp_task_wdt.h>

//enable to have transparent serial connection to LoRa
//#define LoRaConfig 1

//WDT
//3 seconds WDT
#define WDT_TIMEOUT 10
esp_err_t ESP32_ERROR;

//1-wire
// GPIO where the DS18B20 is connected to
const int oneWireBus = 32;     
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

//LoRa
#define LRx 14
#define LTx 12
SoftwareSerial LoRa(LRx, LTx);

void setup() {
  byte RxData;
  int pinM = 0; //LoRa TxRx mode

  // Start the Serial Monitor
  #ifdef LoRaConfig
  Serial.begin(9600); //for testing only
  pinM = 1;
  #else
  Serial.begin(115200); //115200
  #endif

  delay(100);
  Serial.print("Millis ");
  Serial.println(millis());
  Serial.println("Configuring WDT...");
  Serial.print("Watchdog Timeout (in seconds) set to : ");
  Serial.println(WDT_TIMEOUT);
  esp_task_wdt_deinit();
  // Task Watchdog configuration
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT * 1000,                 // Convertin ms
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,  // Bitmask of all cores, https://github.com/espressif/esp-idf/blob/v5.2.2/examples/system/task_watchdog/main/task_watchdog_example_main.c
    .trigger_panic = true                             // Enable panic to restart ESP32
  };
  #ifndef LoRaConfig
  //start watchdog
  ESP32_ERROR = esp_task_wdt_init(&wdt_config);
  Serial.println("Last Reset : " + String(esp_err_to_name(ESP32_ERROR)));
  esp_task_wdt_add(NULL);  //add current thread to WDT watch
  #endif

  // Start the DS18B20 sensor
  sensors.begin();

  //Setup LoRa
  //Reset LoRa
  pinMode(25, OUTPUT); digitalWrite(25, 1);
  pinMode(26, OUTPUT); digitalWrite(26, 1);
  delay(1000);
  LoRa.print("AT+RESET");
  //Set LoRa mode
  delay(4000);
  pinMode(25, OUTPUT); digitalWrite(25, pinM);
  pinMode(26, OUTPUT); digitalWrite(26, pinM);
  LoRa.begin(9600);
  Serial.print("Ready");

#ifdef LoRaConfig
  //LoRa loop connection
  while(1) {
    if (Serial.available()) {      // If anything comes in Serial (USB),
      RxData = Serial.read(); 
      LoRa.write(RxData);
    }
    if(LoRa.available()) {
      RxData = LoRa.read();
      Serial.write(RxData);
    }
  }
#endif
}

int checkMessage(int status) {
  //if message confirmed increase/maintain status
  status=50;
  Serial.println("CheckMessage: Status confirmed");
  return status;
}

void LoRaPower(int power) {
  Serial.println("LoRaPower: power change");
}



void loop() {
  static char mess[20];
  int messi=0;
  byte RxData;
  static int status=50;
  char txtbuf[40];
  int i;

  //WDT reset
  esp_task_wdt_reset();
  delay(2);

  //1-wire read
  sensors.requestTemperatures(); 
  float temperatureC = sensors.getTempCByIndex(0);
  Serial.print(temperatureC);
  Serial.println("ºC");
  sprintf(txtbuf, "%3.3f X\n", temperatureC);
  Serial.println(txtbuf);
  i=0;
  while ((txtbuf[i]!='\n') and (i<10)) {
    LoRa.write(txtbuf[i]);
    i++; }

  //LoRa delivery
  status--;
  Serial.print("Status: ");
  Serial.println(status);
  for(int i=0; i<5000; i++){
    delay(1);
    if(LoRa.available()) {
      RxData = LoRa.read();
      Serial.write(RxData);
      Serial.write(' ');
      mess[messi++]=RxData;
      if(messi>19){messi=0;}
      if(RxData=='X'){
        status=checkMessage(status);
        }
    }
  }
  if(status==40) {
    LoRaPower(20);
  }

  if(status==1){
    Serial.println("Reboot, reset LoRa");
    status=50;
    delay(20000); //trigger watchdog
  }
}