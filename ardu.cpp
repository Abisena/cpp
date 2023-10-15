#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <DHT.h>

const char *ssid = "Local";
const char *password = "Mentegaterbang";

const int suaraPin = A0;
const int lampuPin = D3;
int suaraValue = 0;
int ambangSuara = 250;

const char* serverAddress = "http://be-abi.akmalweb.my.id/add";

#define TRIG_PIN  D6
#define ECHO_PIN  D5
#define SERVO_PIN D2
#define MIN_DISTANCE  1
#define DISTANCE_THRESHOLD  20
#define DHT_PIN D7
#define BUZZER_PIN D1

Servo servo;
int duration_us, distance_cm;
DHT dht(DHT_PIN, DHT22);

unsigned long previousMillisUltrasonic = 0;
const long intervalUltrasonic = 500;  // Interval for ultrasonic sensor (0.5 second)

unsigned long previousMillisMusic = 0;
const long intervalMusic = 1000;  // Interval for music (2 seconds)

void setup() {
  pinMode(lampuPin, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  servo.attach(SERVO_PIN);
  servo.write(0);

  Serial.begin(115200);

  // Connect to WiFi network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
}

void loop() {
  suaraValue = analogRead(suaraPin);
  Serial.println(suaraValue);

  if (suaraValue > ambangSuara) {
    digitalWrite(lampuPin, !digitalRead(lampuPin));

    WiFiClient client;
    HTTPClient http;
    http.begin(client, serverAddress);
    http.addHeader("Content-Type", "application/json");
    String lightStatus = (digitalRead(lampuPin) == HIGH) ? "Nyala" : "Mati";
    String postData = "{\"item\": \"Lampu\", \"status\": \"" + lightStatus + "\"}";

    int httpResponseCode = http.POST(postData);

    Serial.print("Data sent to server: ");
    Serial.println(postData);

    if (httpResponseCode >= 200) {
      String response = http.getString();
      Serial.println("Lamp status successfully sent to the server.");
      Serial.print("Server response: ");
      Serial.println(response);
      Serial.println(httpResponseCode);
    } else {
      Serial.print("Failed to send lamp status to the server. Response code: ");
      Serial.println(httpResponseCode);
    }

    http.end();

    delay(1000);
  }
 
  unsigned long currentMillis = millis();

  // Ultrasonic sensor code
  ultrasonicSensor();

  // DHT sensor code
  dhtSensor();

  // Music code
  if (currentMillis - previousMillisMusic >= intervalMusic) {
    previousMillisMusic = currentMillis;
    // Your melody code here (if you want to play music)
    // For example, playSimpleMelody();
  }
}

void ultrasonicSensor() {
  if (millis() - previousMillisUltrasonic >= intervalUltrasonic) {
    previousMillisUltrasonic = millis();

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    duration_us = pulseIn(ECHO_PIN, HIGH);
    distance_cm = 0.017 * duration_us;

    if (distance_cm > MIN_DISTANCE && distance_cm < DISTANCE_THRESHOLD)
      servo.write(180);
    else
      servo.write(0);

    WiFiClient client;
    HTTPClient http;
    http.begin(client, serverAddress);
    http.addHeader("Content-Type", "application/json");
    String iServo = (digitalRead(distance_cm) > MIN_DISTANCE && < DISTANCE_THRESHOLD) ? "Buka" : "Tutup";
    String postData = "{\"item\": \"Servo\", \"status\": \"" + iServo + "\"}";

    Serial.print("Data sent to server: ");
    Serial.println(postData);

    Serial.print("Ultrasonic distance: ");
    Serial.print(distance_cm);
    Serial.println(" cm");

    if (httpResponseCode >= 200) {
      String response = http.getString();
      Serial.println("Lamp status successfully sent to the server.");
      Serial.print("Server response: ");
      Serial.println(response);
      Serial.println(httpResponseCode);
    } else {
      Serial.print("Failed to send lamp status to the server. Response code: ");
      Serial.println(httpResponseCode);
    }

    http.end();

    delay(1000);
  }
}

void dhtSensor() {
  float temperature = dht.readTemperature();

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");

  if (temperature > 30) {
    playSimpleMelody(); // Play melody when temperature is above 30
  } else {
    noTone(BUZZER_PIN);  // Stop the buzzer
  }

  WiFiClient client;
  HTTPClient http;
  http.begin(client, serverAddress);
  http.addHeader("Content-Type", "application/json");
  String isBuzzer = (digitalRead(temperature) > 30 ? tone(BUZZER_PIN) : noTone(BUZZER_PIN)) ? "Bunyi" : "Mati";
  String postData = "{\"item\": \"Buzzer\", \"status\": \"" + isBuzzer + "\"}";

  Serial.print("Data sent to server: ");
  Serial.println(postData);

  if (httpResponseCode >= 200) {
     String response = http.getString();
     Serial.println("Lamp status successfully sent to the server.");
     Serial.print("Server response: ");
     Serial.println(response);
     Serial.println(httpResponseCode);
   } else {
     Serial.print("Failed to send lamp status to the server. Response code: ");
     Serial.println(httpResponseCode);
   }

    http.end();

    delay(1000);
}

void playSimpleMelody() {
   int melody[] = {
    2000, 0, 2000, 0, 2000, 0
  };

  int noteDurations[] = {
    200, 200, 200, 200, 200, 200
  };

  for (int i = 0; i < 4; i++) {
    int noteDuration = 1000 / noteDurations[i];
    tone(BUZZER_PIN, melody[i], noteDuration);
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    noTone(BUZZER_PIN);
  }
}