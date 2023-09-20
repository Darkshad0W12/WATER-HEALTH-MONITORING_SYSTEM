#include <ESP8266WiFi.h>
#include <espnow.h>
#include <HTTPSRedirect.h>

#define BOARD_ID 4
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

float tdsValue = 0;
float phValue = 0;
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
const long interval = 1000;    

const char* ssid     = "M31s";
const char* password = "rvnz0865";

// Enter Google Script Deployment ID:
const char *GScriptId = "AKfycbz_TGO1z_G2v2XFCsN_DHiyHrDNisvI8h_JlFJOwHwBjIjwxoDcRvrF9HwfxqG1XP3b";

String payload_base =  "{\"command\": \"insert_row\", \"sheet_name\": \"Reservoir4\", \"values\": ";
String payload = "";

// Google Sheets setup (do not edit)
const char* host = "script.google.com";
const int httpsPort = 443;
const char* fingerprint = "";
String url = String("/macros/s/") + GScriptId + "/exec";
HTTPSRedirect* client = nullptr;    // Interval at which to publish sensor readings

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0){
    Serial.println("Delivery success");
  }
  else{
    Serial.println("Delivery fail");
  }

  static bool flag = false;
  if (!flag){
    client = new HTTPSRedirect(httpsPort);
    client->setInsecure();
    flag = true;
    client->setPrintResponseBody(true);
    client->setContentTypeHeader("application/json");
  }
  if (client != nullptr){
    if (!client->connected()){
      client->connect(host, httpsPort);
    }
  }
  else{
    Serial.println("Error creating client object!");
  }
  payload = payload_base + "\"" + temperature + "," + tdsValue + "," + phValue + "," + distance + "\"}";
  client->POST(url, host, payload);
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

  WiFi.begin(ssid, password);             
  Serial.print("Connecting to ");
  Serial.print(ssid); Serial.println(" ...");

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println('\n');
  Serial.println("Connection established!");  
  Serial.print("IP address:\t");
  Serial.println(WiFi.localIP());

  client = new HTTPSRedirect(httpsPort);
  client->setInsecure();
  client->setPrintResponseBody(true);
  client->setContentTypeHeader("application/json");
  
  Serial.print("Connecting to ");
  Serial.println(host);

  // Try to connect for a maximum of 5 times
  bool flag = false;
  for (int i=0; i<5; i++){ 
    int retval = client->connect(host, httpsPort);
    if (retval == 1){
       flag = true;
       Serial.println("Connected");
       break;
    }
    else
      Serial.println("Connection failed. Retrying...");
  }
  if (!flag){
    Serial.print("Could not connect to server: ");
    Serial.println(host);
    return;
  }
  delete client;    // delete HTTPSRedirect object
  client = nullptr; // delete HTTPSRedirect object*/
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
  sensors.requestTemperatures(); 
  temperature=sensors.getTempCByIndex(0); 

  tdsValue = random(400,700);
  phValue = random(6.0,9.0);

  Serial.print("Temp: ");
  Serial.println(temperature);
  Serial.print("TDS: ");
  Serial.println(tdsValue);2
  Serial.print("pH: ");
  Serial.println(phValue);
  Serial.print("Level: ");
  Serial.println(distance);
  
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
    delay(1000);
  }
}