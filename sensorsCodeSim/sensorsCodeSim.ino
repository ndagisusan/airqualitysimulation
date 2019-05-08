//This is an Air Quality Monitoring Sytem with gas sensors for measuring PM2.5, PM10, CO and CO2 plus GPS for determining
// the sensor node location.

//It includes both the simulation code and the implementation code

#include <SoftwareSerial.h>
#include <DHT.h>
#include <DHT_U.h>
#include <TinyGPS.h>

SoftwareSerial mySerial(3,4); //TX=3(Arduino RX), RX=4(Arduino TX)

#define SIMULATION true //comment out or change to false when implementing
#define GPS_SIMULATION true //comment out or change to false when implementing
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

//int ledbuz = 8; //red+buzzer led
//int greenLed = 9;

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

//  pinMode(DHTPIN, INPUT);
//  pinMode(PM25PIN, INPUT);
//  pinMode(PM10PIN, INPUT);
//  
//  //pinMode(ledbuz, OUTPUT);
//  //pinMode(greenLed, OUTPUT);
//  pinMode(MQ9SENSOR, INPUT);
//  pinMode(MQ135SENSOR, INPUT);
//
//  pinMode(MQ9SENSOR_D, INPUT);
//  pinMode(MQ135SENSOR_D, INPUT);
  
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
  //SDS011
  long pm25High = pulseIn(PM25PIN, HIGH, TIMEOUTDELAY)/1000;   //in milliseconds
  long pm10High = pulseIn(PM10PIN, HIGH, TIMEOUTDELAY)/1000;

  //MQ GAS SENSORS
  int mq9value = readMQ9();
  int mq135value = readMQ135();

  //GPS MODULE
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

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
  Serial.println(failed);
  
  float h = dht.readHumidity();
  float t = dht.readTemperature(); // Read temperature in Celsius (default)
    // Check if any reads from the DHT11 failed and exit early (to try again).
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    //digitalWrite (ledbuz, HIGH);
    return;
  }

  //for simulation
  mySerial.print("Count: ");
  mySerial.println(count++);
  mySerial.print("PM2.5 concentration: ");
  mySerial.print(pm25High-2);
  mySerial.println("ug/m3");
  mySerial.print("PM10 concentration: ");
  mySerial.print(pm10High-2);
  mySerial.println("ug/m3");
  mySerial.print("MQ9 Sensor Reading: ");\
  
  if(mq9value > 0){
    mySerial.println("GAS DETECTED");
  }else{
    mySerial.println("NO GAS DETECTED");
  }
  
  mySerial.print("MQ135 Sensor Reading: ");
  if(mq135value > 0){
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
  Serial.print("Count: ");
  Serial.println(count++);
  Serial.print("PM2.5 concentration: ");
  Serial.print(pm25High-2);
  Serial.println("ug/m3");
  Serial.print("PM10 concentration: ");
  Serial.print(pm10High-2);
  Serial.println("ug/m3");
  Serial.print("MQ9 Sensor Reading: ");
  Serial.print(mq9value);
  Serial.println("ppm");
  Serial.print("MQ135 Sensor Reading: ");
  Serial.print(mq135value);
  Serial.println("ppm"); 
  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.println("%");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.println("*C");
  Serial.println(" ");

  /*if ((h > 45 || h < 55) && (t > 21 || t < 24) && (analogSensor > 400))  // Checks if it has reached the limits
  {
    digitalWrite(ledbuz, LOW); //no alert
    digitalWrite(greenLed, HIGH);
  }
  else
  {
    digitalWrite (ledbuz, HIGH); //alert ON
    digitalWrite(greenLed, LOW); 
  }*/
}
