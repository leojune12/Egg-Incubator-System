#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <virtuabotixRTC.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

//dht DHT;
SoftwareSerial SIM900(10, 11); // GSM
LiquidCrystal_I2C lcd(0x27,16,2);
Servo myservo;
virtuabotixRTC myRTC(5, 6, 7); 

#define bulb_pin 2
#define fan_pin 4
int time_now[2];
#define DHTPIN 9
#define DHTTYPE DHT11

DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t delayMS;

void setup() {

  // Set the current date, and time in the following format:
  // seconds, minutes, hours, day of the week, day of the month, month, year
  //   myRTC.setDS1302Time(00, 56, 14, 7, 29, 1, 2022);

  // Arduino communicates with SIM900 GSM shield at a baud rate of 115200
  SIM900.begin(115200);

  // Initialize the lcd
  lcd.init();
  lcd.backlight();

  Serial.begin(9600);

  serialPrint("Egg Incubator System");
  lcdPrint(0, 0, "Egg Incubator   ");
  lcdPrint(1, 0, "System          ");

  delay(2000);

  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);
  delayMS = sensor.min_delay / 1000;

  pinMode(bulb_pin, OUTPUT);
  pinMode(fan_pin, OUTPUT);
  digitalWrite(bulb_pin, HIGH);
  digitalWrite(fan_pin, HIGH);
}

void loop() {
  
  check_humidity_and_temperature();

  update_time();

  control_relay();

  move_servo();
  
  delay(delayMS);
}

void control_relay() {

  digitalWrite(bulb_pin, HIGH);

  if (read_temperature() > 39) {

    digitalWrite(bulb_pin, HIGH);
  } else {

    digitalWrite(bulb_pin, LOW);
  }
  if (read_humidity() < 56) {

    digitalWrite(fan_pin, LOW);
  } else {

    digitalWrite(fan_pin, HIGH);
  }
}

float read_temperature() {

  sensors_event_t event;

  dht.temperature().getEvent(&event);
  return event.temperature;
}

float read_humidity() {

  sensors_event_t event;

  dht.humidity().getEvent(&event);
  return event.relative_humidity;
}

int lcdPrint(int x, int y, String message) {
  lcd.setCursor(y, x);
  lcd.print(message);
}

int serialPrint(String message) {
  Serial.println(message);
}

void sendSMS() {
  serialPrint("Sending SMS");
  lcd.setCursor(0, 0);
  lcd.print("Sending SMS     ");
  lcd.setCursor(0, 1);
  lcd.print("                ");
  
  // AT command to set SIM900 to SMS mode
  SIM900.print("AT+CMGF=1\r");
  delay(100);

  // REPLACE THE X's WITH THE RECIPIENT'S MOBILE NUMBER
  // USE INTERNATIONAL FORMAT CODE FOR MOBILE NUMBERS
  SIM900.println("AT + CMGS = \"+639311235160\"");
  delay(100);

  sensors_event_t event;
  dht.humidity().getEvent(&event);
  float humidity = event.relative_humidity;
  dht.temperature().getEvent(&event);
  float temperature = event.temperature;
  
  // REPLACE WITH YOUR OWN SMS MESSAGE CONTENT
  SIM900.println("Egg Incubator System\n\rTemperature: " + String(temperature) + "C\n\rHumidity: " + String(humidity) + "%");
  delay(100);

  // End AT command with a ^Z, ASCII code 26
  SIM900.println((char)26);
  delay(100);
  SIM900.println();

  for (int i=0; i < 5; i++) { 
    lcd.setCursor(11,0);
    lcd.print("          ");
    delay(250);
    lcd.setCursor(11,0);
    lcd.print(".");
    delay(250);
    lcd.print(".");
    delay(250);
    lcd.print(".");
    delay(250);
  }

  serialPrint("SMS sent");
  lcd.setCursor(0,1);
  lcd.print("SMS sent        ");
  delay(3000);
}

void check_humidity_and_temperature() {
  
  Serial.print("Current humidity = ");
  Serial.print(read_humidity());
  Serial.print("%  ");
  lcdPrint(0, 0, "Humidity: ");
  lcdPrint(0, 10, String(read_humidity()) + "%     ");

  Serial.print("temperature = ");
  Serial.print(read_temperature()); 
  Serial.println("C  ");
  lcdPrint(1, 0, "Temp: ");
  lcdPrint(1, 6, String(read_temperature()) + "C     ");
}

void turn_bulb_on() {
  digitalWrite(bulb_pin, HIGH);
}

void turn_bulb_off() {
  digitalWrite(bulb_pin, LOW);
}

void turn_fan_on() {
  digitalWrite(fan_pin, HIGH);
}

void turn_fan_off() {
  digitalWrite(fan_pin, LOW);
}

void move_servo() {

  myservo.attach(3);
  
  for (int pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15 ms for the servo to reach the position
  }
  for (int pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15 ms for the servo to reach the position
  }

  myservo.detach();
}

void update_time() {

    myRTC.updateTime();
    time_now[0] = myRTC.hours;
    time_now[1] = myRTC.minutes;
    time_now[2] = myRTC.seconds;

    String hour_now = "";
    String minutes_now = "";
    String seconds_now = "";
    String hour_alarm = "";
    String minutes_alarm = "";

    if (time_now[0] < 10) {
      hour_now = "0" + String(time_now[0]);
    } else {
      hour_now = String(time_now[0]);
    }
    
    if (time_now[1] < 10) {
      minutes_now = "0" + String(time_now[1]);
    } else {
      minutes_now = String(time_now[1]);
    }

    if (time_now[2] < 10) {
      seconds_now = "0" + String(time_now[2]);
    } else {
      seconds_now = String(time_now[2]);
    }

    Serial.print("Time: ");
    Serial.print(hour_now);
    Serial.print(":");                                                                                     //| 
    Serial.print(minutes_now);
    Serial.print(":");                                                                                     //| 
    Serial.println(seconds_now);
    Serial.print("Date: ");
    Serial.print(myRTC.month);
    Serial.print("/");
    Serial.print(myRTC.dayofmonth);
    Serial.print("/");
    Serial.print(myRTC.year);

    for (int i=0; i < 3; i++) {

//        if (myRTC.minutes == 31 && myRTC.seconds == i) {
    if (myRTC.seconds == i) {
            Serial.println("sending...");
            sendSMS();
        } 
    }
}
