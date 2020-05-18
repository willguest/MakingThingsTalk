/* 

---------------------------------------------------------------------------------------------------------
Wekit Vest sensor system
---------------------------------------------------------------------------------------------------------

Author: Mark Ransley markcransley@gmail.com
Author: Fridolin Wild wild@brookes.ac.uk
Author: Ståle Antonsen
Author: Will Guest

The following commands can be sent to the ESP32 
to activate and to stop the motors:

command   action
ml1       start left motor
ml0       stop left motor
mr1       start right motor
mr0       stop right motor

*/

#include <Wire.h>
#include "SSD1306.h"
#include "OLEDDisplayUi.h"
#include "esp32-hal-i2c.h"

#include <Adafruit_Sensor.h>
//#include <DHT.h>
//#include <DHT_U.h>

#define MQTT_MAX_PACKET_SIZE 1024 // WARNING: you have to edit PubSubClient.h and set the packet size there !!! took me two hours to trace that bug !!! (FW)
char c[1024] = ""; // buffer for mqtt messaging

// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
// Pin Defines

#define SDA0_PIN 5
#define SCL0_PIN 4
#define SDA1_PIN 25
#define SCL1_PIN 26
#define TEMP1_DATA_PIN 2
#define TEMP1_CLK_PIN 14
#define TEMP0_DATA_PIN 12
#define TEMP0_CLK_PIN 0
#define PULSE0_PIN 16
#define PULSE1_PIN 39
#define GSR_PIN 36
#define MOTOR0_PIN 13
#define MOTOR1_PIN 15

// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
// Display

SSD1306  display(0x3c, 5, 4); //The display is hardwired to the pins 5 and 4, so it does not make sense to use pin define for this.

// variables for handling received messages:
// command contains either ml0, ml1, mr0, mr1 to
// switch on the vibration motors accordingly

String command = "";
boolean commandoTrigger = false;

// generate a unique client id

uint64_t chipid = ESP.getEfuseMac(); // MAC address of ESP32
uint16_t chip = (uint16_t)(chipid>>32);
char clientid[25];

#define WEKIT_COM_UART 1
#define WEKIT_COM_BLE 2
#define WEKIT_COM_WIFI 3

#define WEKIT_COM  WEKIT_COM_WIFI

#if WEKIT_COM == WEKIT_COM_UART

  // -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
  // UART: 

#elif WEKIT_COM == WEKIT_COM_BLE

  // -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
  // BLE

  #include <BLEDevice.h>
  #include <BLEServer.h>
  #include <BLEUtils.h>
  #include <BLE2902.h>

  #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
  #error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
  #endif

  BLECharacteristic *pWriteCharacteristic;
  BLECharacteristic *pIMUCharacteristic;
  BLECharacteristic *pGSRCharacteristic;
  BLECharacteristic *pTempCharacteristic;
  BLECharacteristic *pHumidityCharacteristic;
  bool deviceConnected = false;
  uint8_t txValue = 0;
  
  #define SERVICE_UUID          "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"  // UART service UUID
  #define COM_CHARACTERISTIC_WR "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // Command Characteristic, for writing to the device
  #define IMU_CHARACTERISTIC_NT "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // Notify characteristics for each sensor
  #define GSR_CHARACTERISTIC_NT "6E400004-B5A3-F393-E0A9-E50E24DCCA9E"
  #define TMP_CHARACTERISTIC_NT "6E400005-B5A3-F393-E0A9-E50E24DCCA9E"
  #define HMT_CHARACTERISTIC_NT "6E400006-B5A3-F393-E0A9-E50E24DCCA9E"

  class MyServerCallbacks: public BLEServerCallbacks {
      void onConnect(BLEServer* pServer) {
        deviceConnected = true;
      };
  
      void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
      }
  };
  
  class MyCallbacks: public BLECharacteristicCallbacks {
      void onWrite(BLECharacteristic *pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();
  
        if (rxValue.length() > 0) {
          Serial.print("Received Value: ");
          for (int i = 0; i < rxValue.length(); i++)
            Serial.print(rxValue[i]);
  
          Serial.println();
        }
      }
  };
  
#elif WEKIT_COM == WEKIT_COM_WIFI

  // -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
  // Wifi

  #include <WiFi.h>
  #include <WiFiMulti.h>

  WiFiMulti wifiMulti;
  
  //#include <ESPmDNS.h>
  #include <PubSubClient.h>
  
  WiFiClient espClient;
  PubSubClient client(espClient);

  const char* mqtt_server = "test.mosquitto.org";
  int mqtt_server_counter = 0;

  char mqtt_servers[][64] = {
    "test.mosquitto.org"
  };

  long lastMsg = 0;
  char msg[1024];
  int value = 0;

  void mqttcallback(char* topic, byte* payload, unsigned int length) {

    Serial.print("mqtt[" + String(topic) + "]: ");
    
    String msg;
    for (int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
      msg += (char)payload[i];
    }

    Serial.println(String(msg));
    //display.drawStringMaxWidth(0,40, 100, "["+String(topic)+"]: " + String(msg));
    //display.display();

    command = msg;
    commandoTrigger = true;
    
  }

  void mqttconnect() {
    
    while (!client.connected()) {

      mqtt_server = mqtt_servers[mqtt_server_counter];
      client.setServer(mqtt_server, 1883);
      client.setCallback(mqttcallback);

      display.clear();
      display.drawStringMaxWidth(0, 40, 120, "mqtt: try "+ String(mqtt_server) ); 
      display.display();

      if ( client.connect(clientid) ) {

        display.clear();
        display.drawStringMaxWidth(0, 40, 120, "mqtt: on "+ String(mqtt_server) ); 
        display.display();

        // publish availability (and clientid) on wekit control channel
        String msg = String(clientid) + " online";
        msg.toCharArray(c, msg.length()+1 );
        c[ msg.length() + 2 ] = '\0';
        client.publish("wekit/control", c);
        
        // resubscribe to control channel
        client.subscribe("wekit/control");
        
      } else {

        display.clear();
        display.drawStringMaxWidth(0, 40, 120, "mqtt: fail "+ String(mqtt_server) ); 
        display.display();

        // try next mqtt server or loop back to first
        mqtt_server_counter++;
        if ( mqtt_server_counter > ( sizeof(mqtt_servers) / sizeof(mqtt_servers[0]) -1) ) {
          mqtt_server_counter = 0;
        }
        mqtt_server = mqtt_servers[mqtt_server_counter];
        
        //Serial.println("trying mqtt server: "+String(mqtt_server_counter) + ".: "+String(mqtt_server));
        delay(2000);
        
        client.setServer(mqtt_server, 1883);
        
      } // if connected
      
    }
    
  } // mqttconnect()

#endif // end of WEKIT_COM branching

// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
// IMUs

//number of MPU9250
#define MPUS 2

// old MPU9250
//#include "MPU9250_asukiaaa.h"

// new MPU9250
#include "quaternionFilters.h"
#include "MPU9250.h"
#define AHRS true         // Set to false for basic data read
#define SERIAL_DEBUG false  // Set to true to get Serial output for debugging

//declare the two MPU9250s
MPU9250 mpu9250[MPUS];

struct IMU {
  float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;
  float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
};

IMU imu[MPUS];

//declare the second i2c bus that will be used for one of the MPU9250s
TwoWire Wire1 = TwoWire(1);

// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
// heart rate

#define MAX_HEARTRATE_DUTY  2000 // the heart should beat at least once per 2000 ms (this is of course editable but a heart rate of less than 30 bpm is seldom)

volatile unsigned long tick[2]; //variable used to calculate heart-rate and to see if the sensor is getting data connected
volatile unsigned int heart_rate[2]; //variables showing the actual heart-rate from sensor 0 and 1

// the intterrupt routine for Pulse0 sensor
void pulse0_ISR() {
    static unsigned long sub0,tick_old0;
    tick[0] = millis();
    if (tick[0]<tick_old0) {
      sub0 = 0xFFFFFFFF - (tick_old0 - tick[0] + 1);
    }
    else
      sub0 = tick[0] - tick_old0;
    if (sub0>MAX_HEARTRATE_DUTY) {
      heart_rate[0] = 0;
    }
    else {
      heart_rate[0] = 60000/sub0;
    }

    tick_old0 = tick[0];
}

// the intterrupt routine for Pulse1 sensor
void pulse1_ISR() {
    static unsigned long sub1,tick_old1;
    tick[1] = millis();
    if (tick[1]<tick_old1) {
      sub1 = 0xFFFFFFFF - (tick_old1 - tick[1] + 1);
    }
    else
      sub1 = tick[1] - tick_old1;
    if (sub1>MAX_HEARTRATE_DUTY) {
      heart_rate[1] = 0;
    }
    else {
      heart_rate[1] = 60000/sub1;
    }

    tick_old1 = tick[1];
}


// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
// SHT 

//Declare the SHT11s here 

struct shtdata_t {
    float humidity;
    float temperature;
};

shtdata_t env[2]; //environment variables - a place to store data from the SHT11s

// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
// GSR

uint16_t gsr;

// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
// Motors

class MOTOR {
private:
  uint8_t _pin;
public:
  MOTOR(uint8_t _pin) {
    this->_pin = _pin;
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, false);
  }
  void run(void) {
    digitalWrite(_pin, true);
  }
  void stop(void) {
    digitalWrite(_pin, false);
  }

};

void copy(int* src, int* dst, int len) {
    memcpy(dst, src, sizeof(src[0])*len);
}

//declaring the motors with their pins
MOTOR ml(MOTOR0_PIN), mr(MOTOR1_PIN);

// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
// cmdHandler (parser for input)
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  

//fuction for checking what the input string is and activate appropriate action
void cmdHandler( String commando = "" ) {
  
  if (commando.equals("ml1"))
    ml.run();
  else if (commando.equals("ml0"))
    ml.stop();
  else if (commando.equals("mr1"))
    mr.run();
  else if (commando.equals("mr0"))
    mr.stop();
        
}

// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
// sensor data formatting

String dataframe = "";

// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
// SETUP
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  

// the setup function runs once when you press reset or power the board
void setup() {  

  Serial.begin(115200);
  delay(100);

  // create a unique ID for the vest (hopefully unique ;)
  snprintf(clientid,25,"WEKIT-VEST-%08X",chip);
  
  char text[30];

  display.init();
  display.resetDisplay();
  display.displayOn();
  
  #if WEKIT_COM == WEKIT_COM_WIFI
    
    const char* ssid = "VM7851243";
    const char* password =  "----------";
    WiFi.begin(ssid, password);

  #elif WEKIT_COM == WEKIT_COM_BLE

    // Bluetooth Component Setup 
    
    BLEDevice::init("WEKIT.SensorHub"); // Create the BLE Device
    BLEServer *pServer = BLEDevice::createServer(); // Create the BLE Server
    pServer->setCallbacks(new MyServerCallbacks()); 
    
    BLEService *pService = pServer->createService(SERVICE_UUID); // Create the BLE Service
  
    BLECharacteristic *pWriteCharacteristic = pService->createCharacteristic(
                         BLEUUID(COM_CHARACTERISTIC_WR),
                         BLECharacteristic::PROPERTY_WRITE);
  
    pIMUCharacteristic = pService->createCharacteristic(
                        BLEUUID(IMU_CHARACTERISTIC_NT),
                        BLECharacteristic::PROPERTY_NOTIFY);  
    pIMUCharacteristic->addDescriptor(new BLE2902());
    pIMUCharacteristic->setCallbacks(new MyCallbacks());                 
    
    pGSRCharacteristic = pService->createCharacteristic(
                        GSR_CHARACTERISTIC_NT,
                        BLECharacteristic::PROPERTY_NOTIFY);
    pGSRCharacteristic->addDescriptor(new BLE2902());
    pGSRCharacteristic->setCallbacks(new MyCallbacks());
  
    pTempCharacteristic = pService->createCharacteristic(
                         TMP_CHARACTERISTIC_NT,
                         BLECharacteristic::PROPERTY_NOTIFY);
    pTempCharacteristic->addDescriptor(new BLE2902());
    pTempCharacteristic->setCallbacks(new MyCallbacks());
  
    pHumidityCharacteristic = pService->createCharacteristic(
                            HMT_CHARACTERISTIC_NT,
                            BLECharacteristic::PROPERTY_NOTIFY);
    pHumidityCharacteristic->addDescriptor(new BLE2902());
    pHumidityCharacteristic->setCallbacks(new MyCallbacks());
                                         
    
    pService->start(); // Start the service
    
    BLEAdvertising* pAdvertising = pServer->getAdvertising();
    pAdvertising->addServiceUUID(BLEUUID(pService->getUUID()));
    pAdvertising->start();
    
  #endif

  //dht.begin(); 
  pinMode(GSR_PIN, INPUT);
  
  //setting upt the pin and interrupt for heart-rate sensors
  pinMode(PULSE0_PIN, INPUT);
  pinMode(PULSE1_PIN, INPUT);
  
  //attachInterrupt(digitalPinToInterrupt(PULSE0_PIN), pulse0_ISR, RISING);
  //attachInterrupt(digitalPinToInterrupt(PULSE1_PIN), pulse1_ISR, RISING);

  // -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
  // set up wiring for MPUs
  // -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  

  // Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_100); // from AHRS code

  // limiting the i2c-bus speed to 400kHz because of the mpu9250s
  Wire.setClock(400000);

  // setting up the first I2C1_bus
  
  Wire1.begin(SDA1_PIN, SCL1_PIN);
  Wire1.setClock(400000);

  // assign I2C bus to the two MPUs
  
  mpu9250[0].setWire(&Wire);
  
  if (MPUS>=2){
    mpu9250[1].setWire(&Wire1);
  }
  
  for (int i = 0; i < MPUS; i++) {

    display.clear();
    display.drawStringMaxWidth(0, 0, 120, "Calibrating MPUs: MPU " + String(i) ); 
    display.drawStringMaxWidth(0, 10, 120, "Self test..." ); 
    display.display();

    mpu9250[i].MPU9250SelfTest(mpu9250[i].selfTest);

    display.clear();
    display.drawStringMaxWidth(0, 0, 120, "Calibrating MPUs: MPU " + String(i) ); 
    display.drawStringMaxWidth(0, 10, 120, "Calibration ..." ); 
    display.display();
    
    mpu9250[i].calibrateMPU9250(mpu9250[i].gyroBias, mpu9250[i].accelBias);

    display.clear();
    display.drawStringMaxWidth(0, 0, 120, "Calibrating MPUs: MPU " + String(i) ); 
    display.drawStringMaxWidth(0, 10, 120, "Init..." ); 
    display.display();
    
    mpu9250[i].initMPU9250();
    
    mpu9250[i].initAK8963(mpu9250[i].factoryMagCalibration);
    
    mpu9250[i].getAres();
    mpu9250[i].getGres();
    mpu9250[i].getMres();

    for (int n=4; n--; n>0) {
      display.clear();
      display.drawStringMaxWidth(0, 0, 120, "Calibrating MPUs: MPU " + String(i) ); 
      display.drawStringMaxWidth(0, 20, 120, "In " + String(n) + "seconds, wave device in a figure 8 until done" ); 
      display.display();
      delay(1000);
    }

    display.clear();
    display.drawStringMaxWidth(0, 0, 120, "Calibrating MPUs: MPU " + String(i) ); 
    display.drawStringMaxWidth(0, 20, 120, "Wave device in a figure 8 /NOW/!" ); 
    display.display();

    cmdHandler("ml1");
    delay(50);
    cmdHandler("ml0");
    delay(50);
    cmdHandler("ml1");
    delay(50);
    cmdHandler("ml0");

    mpu9250[i].magCalMPU9250(mpu9250[i].magBias, mpu9250[i].magScale);

    cmdHandler("ml1");
    delay(50);
    cmdHandler("ml0");

    display.clear();
    display.drawStringMaxWidth(0, 0, 120, "Calibrating MPUs: MPU " + String(i) ); 
    display.drawStringMaxWidth(0, 20, 120, "done." ); 
    display.display();
    
  }
  
} // setup()


// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
// LOOP
//     the loop function runs over and over again until power down or reset
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  

void loop() {

  // -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
  // Output to ESP32's mini display
  // -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  

  display.clear();
  
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_24); 
  display.drawString(0, 0, "SUIT 1.4");
  display.setFont(ArialMT_Plain_10);

  // -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
  // check input channel for control commands
  // -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  

  #if WEKIT_COM == WEKIT_COM_UART

    //poll the serial port for incoming commands
    
    while (Serial.available()) {
      char inChar = (char)Serial.read();
      if (inChar == '\n') {
        commandoTrigger = true;
      }
      else if (inChar != '\r') {
        command += inChar;
      }
    }
  
  #elif WEKIT_COM == WEKIT_COM_BLE

    // they are already handled in the BLE callback functions

  #elif WEKIT_COM == WEKIT_COM_WIFI

    // commands expected on control channel: wekit/control
    // they are already handled in the mqttcallback() function
    if (WiFi.status() == WL_CONNECTED) {
      delay(50);  
    }    
    
  #endif

  // -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
  // handle commands from the control channel
  // -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  

  // now that control messages have been picked up (regardless
  // of communiation protocol: deal with them
  if (commandoTrigger) {

    display.drawStringMaxWidth(60, 30, 120, "cmd: in:" + String(command) ); 
  
    cmdHandler( command ); // e.g.: if command value is "mr1", "mr0", "ml1", or "ml0", then switch on/off according vibration motor
    
    command = "";
    commandoTrigger = false;
    
  }

  // -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
  // now get the sensor data and append to the dataframe string
  // -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  

  dataframe = "{\"client\":\""+String(clientid)+"\",\"time\":" + String(millis()) + ",";

  for (int i=0;i<2;i++){
    if (millis() > tick[i] + MAX_HEARTRATE_DUTY) {
      heart_rate[i] = 0; 
    }
  }
  
  dataframe += "\"imus\":[";
  for (int i = 0; i < MPUS; i++) {

    dataframe += "{";
    
    //update imu data
    mpu9250[i].readAccelData(mpu9250[i].accelCount);
    //mpu9250[i].accelUpdate();
    imu[i].aX = (float)mpu9250[i].accelCount[0] * mpu9250[i].aRes; // - myIMU.accelBias[0];
    imu[i].aY = (float)mpu9250[i].accelCount[1] * mpu9250[i].aRes; // - myIMU.accelBias[1];
    imu[i].aZ = (float)mpu9250[i].accelCount[2] * mpu9250[i].aRes; // - myIMU.accelBias[2];

    dataframe += "\"ax\":"+String(imu[i].aX)+",";
    dataframe += "\"ay\":"+String(imu[i].aY)+",";
    dataframe += "\"az\":"+String(imu[i].aZ)+",";

    mpu9250[i].readGyroData(mpu9250[i].gyroCount);
    imu[i].gX = (float)mpu9250[i].gyroCount[0] * mpu9250[i].gRes;
    imu[i].gY = (float)mpu9250[i].gyroCount[1] * mpu9250[i].gRes;
    imu[i].gZ = (float)mpu9250[i].gyroCount[2] * mpu9250[i].gRes;

    dataframe += "\"gx\":"+String(imu[i].gX)+",";
    dataframe += "\"gy\":"+String(imu[i].gY)+",";
    dataframe += "\"gz\":"+String(imu[i].gZ)+",";

    mpu9250[i].readMagData(mpu9250[i].magCount);
    imu[i].mX = (float)mpu9250[i].magCount[0] * mpu9250[i].mRes * mpu9250[i].factoryMagCalibration[0] - mpu9250[i].magBias[0];
    imu[i].mY = (float)mpu9250[i].magCount[1] * mpu9250[i].mRes * mpu9250[i].factoryMagCalibration[1] - mpu9250[i].magBias[1];
    imu[i].mZ = (float)mpu9250[i].magCount[2] * mpu9250[i].mRes * mpu9250[i].factoryMagCalibration[2] - mpu9250[i].magBias[2];

    dataframe += "\"mx\":"+String(imu[i].mX)+",";
    dataframe += "\"my\":"+String(imu[i].mY)+",";
    dataframe += "\"mz\":"+String(imu[i].mZ)+",";

    mpu9250[i].updateTime();

    // output the quaternion
    MahonyQuaternionUpdate(imu[i].aX, imu[i].aY, imu[i].aZ, imu[i].gX * DEG_TO_RAD, imu[i].gY * DEG_TO_RAD, imu[i].gZ * DEG_TO_RAD, imu[i].mY, imu[i].mX, imu[i].mZ, mpu9250[i].deltat, imu[i].q );

    dataframe += "\"q0\":"+String(imu[i].q[0])+","+"\"q1\":"+String(imu[i].q[1])+","+"\"q2\":"+String(imu[i].q[2])+","+"\"q3\":"+String(imu[i].q[3]);

    dataframe += "}";
    if (i==0 && MPUS > 1) dataframe += ",";

  }
  dataframe += "],";

  // GSR values
  gsr=analogRead(GSR_PIN);

  dataframe += "\"gsr\":"+String(gsr);
  dataframe += "}";

  // -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
  // establish connection and publish (UART, WIFI + mqtt, or BLE)
  // -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
  
  #if WEKIT_COM == WEKIT_COM_UART

    Serial.println(dataframe);

  #elif WEKIT_COM == WEKIT_COM_WIFI

    if (WiFi.status() == WL_CONNECTED) { // wifiMulti.run() == WL_CONNECTED ||

      display.drawString(0, 20, "Wifi: " + WiFi.localIP().toString() );

      // mqtt not connected?
      if (!client.connected()) {
        display.drawStringMaxWidth(0, 40, 120, "mqtt: try " +  String(mqtt_server) ); 
        mqttconnect();
      } else {

        // publish data to topic
        dataframe.toCharArray(c, dataframe.length()+1 );
        c[ dataframe.length() + 2 ] = '\0';
        client.publish( "wekit/vest", c );
        display.drawStringMaxWidth(0, 40, 120, "mqtt: on, "+ String(dataframe.length()) + "/" + String(sizeof(c)) +" sent" ); 

      }
      client.loop();
      
    } else {
      
      display.drawString(0, 20, "Wifi: offline");
      
      // connection lost, restart
      if ( wifiMulti.run() == WL_CONNECTED ) {
        display.drawString(0, 20, "Wifi: " + WiFi.localIP().toString() ); // String(WiFi.SSID())
        display.drawStringMaxWidth(0, 40, 120, "mqtt: try " +  String(mqtt_server) ); 
        mqttconnect();
      }       
      //else {
      //  display.drawString(0, 20, "Wifi: AP: WEKIT-VEST / thereisnospoon" );
      //  const char *ssid = "WEKIT-VEST";
      //  const char *password = "thereisnospoon";
      //  WiFi.softAP(ssid, password);        
      //}

    } // wifi not connected

  #else 

    display.drawString(0, 30, "Wifi: off");
    
  #endif

  #if WEKIT_COM == WEKIT_COM_BLE

    // send via BLE
    if (deviceConnected) {
    char imuString[16];
    
      for (int f = 0; f < 4; f++){
        char imuMid[4];
        dtostrf(imu[0].q[f], 4, 2, imuMid);
        strcat(imuString, imuMid);
      }

      Serial.printf("*** Sent Value: %d ***", imuString);
      pIMUCharacteristic->setValue(imuString);
      pIMUCharacteristic->notify();
      txValue++;
    }
    else {
      display.drawString(0,20,"BLE: waiting for client...");
    }
    /*
    else {
      
      display.drawString(0,20,"BLE connected...");
      
      char gsrString[8]; 
      dtostrf(gsr, 1, 2, gsrString); 
      
      char tempString[8]; 
      dtostrf(temp, 1, 2, tempString); 
      
      char humidityString[8]; 
      dtostrf(humidity, 1, 2, humidityString); 
    
      pGSRCharacteristic->setValue(gsrString);
      pGSRCharacteristic->notify(); 
      
      pTempCharacteristic->setValue(tempString);
      pTempCharacteristic->notify(); 
      
      pHumidityCharacteristic->setValue(humidityString);
      pHumidityCharacteristic->notify(); 
      
    }
    */
  #else
  
    display.drawString(0, 30, "BLE: off");
    
  #endif

  display.drawString(0, 50, "data: " + String(dataframe.length()) + ", on: " + String( millis()/1000 ) ); 
  display.display();

}
