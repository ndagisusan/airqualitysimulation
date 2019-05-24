#include <SoftwareSerial.h>
#include <DHT.h>
#include <DHT_U.h>
#include <TinyGPS.h>

SoftwareSerial mySerial(3,4); //TX=3(Arduino RX), RX=4(Arduino TX)

#define SIMULATION false //change to false when implementing
#define GPS_SIMULATION false //change to false when implementing
#define DHTPIN 10     
#define DHTTYPE DHT11   
DHT dht(DHTPIN, DHTTYPE);
TinyGPS gps;  //Creates a new instance of the TinyGPS object

//CONSTANTS
#define PM25PIN 5  // defines pins numbers
#define PM10PIN 6
#define MQ9SENSOR A0 //analog pins when implementing
#define MQ135SENSOR A1

//For simulation
#define MQ9SENSOR_D 11  //digital pins when simulating
#define MQ135SENSOR_D 12

#define TIMEOUTDELAY 1500000 //(1500000 micro-seconds)1.5 seconds for the pulse sent in the SDS011 for reading

//VARIABLES
int count = 1;  //Counts the number of readings taken

void setup() {
  Serial.begin(9600); // Starts the serial communication
  mySerial.begin(9600); //for the Virtual Terminal
  dht.begin();
  Serial.println("AN IoT AIR QUALITY ANALYSIS AND ALERT SYSTEM:");
  Serial.println(" ");

  mySerial.println("AN IoT AIR QUALITY ANALYSIS AND ALERT SYSTEM:");
  mySerial.println(" ");
  
  delay(1000); // 60s(1 min) for stabilization, of the SDS011
}

void loop() {
  read(); 
  delay(2050);  //sampling time for the unit...VARY THIS
}

int readMQ9(){
   if (!SIMULATION){
      return analogRead(MQ9SENSOR);
   }
   return digitalRead(MQ9SENSOR_D);
}
int readMQ135(){
   if (!SIMULATION){
      return analogRead(MQ135SENSOR);
   }
   return digitalRead(MQ135SENSOR_D);
}

void read(){
  mySerial.print("Count: ");
  //mySerial.println(count++);  //comment this during test of implementation

  Serial.print("Count: ");
  Serial.println(count++);  //comment this during simulation
  //SDS011
  long pm25High = pulseIn(PM25PIN, HIGH, TIMEOUTDELAY)/1000;   //in milliseconds
  long pm10High = pulseIn(PM10PIN, HIGH, TIMEOUTDELAY)/1000;

  //MQ GAS SENSORS
  int mq9Value = readMQ9();
  int mq135Value = readMQ135();

  //GPS MODULE
  bool newData = false;
  unsigned long chars;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;){
    while (Serial.available()){
      char c = Serial.read();
      //Serial.print(c);
      if (gps.encode(c)) 
        newData = true;  
    }
  }

  if (newData){     //If newData is true
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("Latitude = ");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" Longitude = ");
    Serial.println(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    
    #ifdef GPS_SIMULATION
    //For simulation
    mySerial.print("Latitude = ");
    mySerial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    mySerial.print(" Longitude = ");
    mySerial.println(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    #endif
  }
  Serial.println("NO GPS READING...");
  
  float h = dht.readHumidity();
  float t = dht.readTemperature(); // Read temperature in Celsius (default)
    // Check if any reads from the DHT11 failed and exit early (to try again).
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  //for simulation
  mySerial.print("PM2.5 concentration: ");
  mySerial.print(pm25High-2);
  mySerial.println("ug/m3");
  mySerial.print("PM10 concentration: ");
  mySerial.print(pm10High-2);
  mySerial.println("ug/m3");
  mySerial.print("CO concentration: ");
  
  if(mq9Value > 0){
    mySerial.println("GAS DETECTED");
  }else{
    mySerial.println("NO GAS DETECTED");
  }
  
  mySerial.print("CO2 concentration: ");
  if(mq135Value > 0){
    mySerial.println("GAS DETECTED");
  }else{
    mySerial.println("NO GAS DETECTED");
  }
  
  mySerial.print("Humidity: ");
  mySerial.print(h);
  mySerial.println("%");
  mySerial.print("Temperature: ");
  mySerial.print(t);
  mySerial.println("*C");
  mySerial.println(" ");

  //for implementation
  Serial.print("PM2.5 concentration: ");
  Serial.print(pm25High-2);
  Serial.println("ug/m3");
  Serial.print("PM10 concentration: ");
  Serial.print(pm10High-2);
  Serial.println("ug/m3");
  Serial.print("CO concentration: ");
  Serial.print(mq9Value);
  Serial.println("ppm");
  Serial.print("CO2 concentration: ");
  Serial.print(mq135Value);
  Serial.println("ppm"); 
  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.println("%");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.println("*C");
  Serial.println(" ");
}
