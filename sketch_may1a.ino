#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <TinyGPSPlus.h>

// Your WiFi credentials
#define WIFI_SSID "HONOR X7c"
#define WIFI_PASSWORD "***************"

// Firebase credentials
#define API_KEY "AIzaSyAoy9naHxQSJKtbq-kYCHlJPAdObr-9dcI"
#define DATABASE_URL "https://smartcane-c3137-default-rtdb.asia-southeast1.firebasedatabase.app/" 

// Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// GPS
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);  // Using UART1
#define GPS_TX 16
#define GPS_RX 17

// Pins
#define BUTTON_PIN 23
#define BUZZER_PIN 21
#define TRIG_PIN 19
#define ECHO_PIN 18

// Other
bool isSystemOn = false;

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  pinMode(BUTTON_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(BUZZER_PIN, LOW);

  // WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" Connected!");

  // Firebase config
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  auth.user.email = "";
  auth.user.password = "";

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

void loop() {
  isSystemOn = digitalRead(BUTTON_PIN);

  if (isSystemOn) {
    checkUltrasonic();
    readGPSAndSendToFirebase();
  } else {
    digitalWrite(BUZZER_PIN, LOW); // Turn off buzzer when system off
  }

  delay(1000); 
}

void checkUltrasonic() {
  long duration;
  int distance;

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;

  if (distance > 0 && distance < 100) {
    digitalWrite(BUZZER_PIN, HIGH);
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }
}

void readGPSAndSendToFirebase() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isUpdated()) {
    float lat = gps.location.lat();
    float lng = gps.location.lng();

    Serial.printf("Lat: %f, Lng: %f\n", lat, lng);

    Firebase.RTDB.setFloat(&fbdo, "/user/location/latitude", lat);
    Firebase.RTDB.setFloat(&fbdo, "/user/location/longitude", lng);
  }
}
