#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>


//////////////// ESWP 32      ESP Board MAC Address:  E0:5A:1B:A7:03:24
//////////////// NOCE MCU     ESP Board MAC Address:  40:91:51:51:78:79

// Set your Board ID (ESP32 Sender #1 = BOARD_ID 1, ESP32 Sender #2 = BOARD_ID 2, etc)
#define BOARD_ID 1
#define CHANNEL 1

//temp_sensor
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 13
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float temperature = 20;


//ph_sensor
#define phsensorpin 32
unsigned long int avgValue;  
float b;
int buf[10],temp;
float phValue=7;

//water_level_sensor
const int trigPin = 2;
const int echoPin = 4;
long duration=0;
float distance=100;

//tds_sensor
#define TdsSensorPin 33
#define VREF 3.3              // analog reference voltage(Volt) of the ADC
#define SCOUNT  30            // sum of sample point

int analogBuffer[SCOUNT];     // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;

float averageVoltage = 0;
float tdsValue = 0;

//tds median filtering algorithm
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


esp_now_peer_info_t slave;

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

unsigned int readingId = 0;

// Insert your SSID
constexpr char WIFI_SSID[] = "ESP8266";

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
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
            slave.peer_addr[ii]=(uint8_t)mac[ii];
        }
      }
      slave.channel = CHANNEL;
      slave.encrypt = 0;
      break;
    }
  }
}
void setup() {
  //Init Serial Monitor
  Serial.begin(115200);
  
  //ph_sensor
  pinMode(phsensorpin,OUTPUT);   

  //water_level_sensor
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT); 

  //tds_sensor
  pinMode(TdsSensorPin,INPUT);

  //temp-sensor
  sensors.begin();

  // Set device as a Wi-Fi Station and set channel
  WiFi.mode(WIFI_STA);
  
  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  ScanForSlave();
  //Add peer        
  if (esp_now_add_peer(&slave) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 
void loop() {
  //ph_sensor
  for(int i=0;i<10;i++)       
  { 
    buf[i]=analogRead(phsensorpin);
    delay(10);
  }
  for(int i=0;i<9;i++)        
  {
    for(int j=i+1;j<10;j++)
    {
      if(buf[i]>buf[j])
      {
        temp=buf[i];
        buf[i]=buf[j];
        buf[j]=temp;
      }
    }
  }
  avgValue=0;
  for(int i=2;i<8;i++)                     
    avgValue+=buf[i];
  float phValue=(float)avgValue*5.0/1024/6; 
  phValue=3.5*phValue;                       
  digitalWrite(13, HIGH);       
  delay(500);
  digitalWrite(13, LOW); 

  //water_level_sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance=duration*0.034/2;

  //temp_sensor
  sensors.requestTemperatures(); 
  temperature=sensors.getTempCByIndex(0); 

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
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 4096.0;
      
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
     
    //Send message via ESP-NOW
    esp_err_t result = esp_now_send(slave.peer_addr, (uint8_t *) &myData, sizeof(myData));
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }
  }
}