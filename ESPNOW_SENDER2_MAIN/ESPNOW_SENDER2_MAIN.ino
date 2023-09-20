#include <ESP8266WiFi.h>
#include <espnow.h>

#define BOARD_ID 2
#define CHANNEL 1

uint8_t broadcastAddress[]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//temp_sensor
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS D6
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float temperature=30;

//water_level_sensor
const int trigPin = D1;
const int echoPin = D2;
long duration=0;
float distance=0;
 
//tds_sensor
#define TdsSensorPin A0
#define VREF 3.3              // analog reference voltage(Volt) of the ADC
#define SCOUNT  30            // sum of sample point

int analogBuffer[SCOUNT];     // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;

float averageVoltage = 0;
float tdsValue = 0;

int getMedianNum(int bArray[], int iFilterLen){
  int bTab[iFilterLen];
  for (byte i = 0; i<iFilterLen; i++)
  bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0){
    bTemp = bTab[(iFilterLen - 1) / 2];
  }
  else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}

float phValue = 7;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    int id;
    float temp;
    float tds;
    float pH;
    float level;

} struct_message;

//Create a struct_message called myData
struct_message myData;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 1000;        // Interval at which to publish sensor readings

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0){
    Serial.println("Delivery success");
  }
  else{
    Serial.println("Delivery fail");
  }
}

void ScanForSlave(){
  int8_t scanResults = WiFi.scanNetworks();
  for(int i =0;i<scanResults; ++i){
    String SSID = WiFi.SSID(i);
    String BSSIDstr= WiFi.BSSIDstr(i);

    if(SSID.indexOf("RX")==0){
      int mac[6];
      if(6 == sscanf(BSSIDstr.c_str(),"%x:%x:%x:%x:%x:%x", &mac[0],&mac[1],&mac[2],&mac[3],&mac[4],&mac[5])){
        for(int ii = 0; ii<6 ; ++ii){
            broadcastAddress[ii]=(uint8_t)mac[ii];
        }
      }
      break;
    }
  }
}

void setup() {
  //Init Serial Monitor
  Serial.begin(115200);  

  //water_level_sensor
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT); 

  //tds_sensor
  pinMode(TdsSensorPin,INPUT);

  //temp-sensor
  sensors.begin();

  // Set device as a Wi-Fi Station and set channel
  WiFi.mode(WIFI_STA);

  ScanForSlave();
  wifi_promiscuous_enable(1);
  wifi_set_channel(CHANNEL);
  wifi_promiscuous_enable(0);

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);

  esp_now_register_send_cb(OnDataSent);
  
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
}
 
void loop() { 
 //water_level_sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance=duration*0.034/2;

  //temp_sensor
  //sensors.requestTemperatures(); 
  //temperature=sensors.getTempCByIndex(0); 

  //tds_sensor
  static unsigned long analogSampleTimepoint = millis();
  if(millis()-analogSampleTimepoint > 40U){     //every 40 milliseconds,read the analog value from the ADC
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
    analogBufferIndex++;
    if(analogBufferIndex == SCOUNT){ 
      analogBufferIndex = 0;
    }
  }   
  
  static unsigned long printTimepoint = millis();
  if(millis()-printTimepoint > 800U){
    printTimepoint = millis();
    for(copyIndex=0; copyIndex<SCOUNT; copyIndex++){
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
      
      // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 1024.0;
      
      //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0)); 
      float compensationCoefficient = 1.0+0.02*(temperature-25.0);
      //temperature compensation
      float compensationVoltage=averageVoltage/compensationCoefficient;
      
      //convert voltage value to tds value
      tdsValue=(133.42*compensationVoltage*compensationVoltage*compensationVoltage - 255.86*compensationVoltage*compensationVoltage + 857.39*compensationVoltage)*0.5;
    }
  }

  Serial.print("Temp: ");
  Serial.println(temperature);
  Serial.print("TDS: ");
  Serial.println(tdsValue);
  Serial.print("pH: ");
  Serial.println(phValue);
  Serial.print("Level: ");
  Serial.println(distance);
  delay(1000);
  
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;
    //Set values to send
    myData.id = BOARD_ID;
    myData.temp = temperature;
    myData.tds = tdsValue;
    myData.pH = phValue;
    myData.level = distance;
     
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  }
}