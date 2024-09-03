#include <project_secrets.h>

/* Change these values based on your calibration values */
#define GrainWet 890   // Define max value we consider soil 'wet'
#define GrainDry 1000   // Define min value we consider soil 'dry'

#define HUMIDITY_CALIBRATION_OFFSET -31.0
unsigned long lastSmsSentMillis = 0;  // Keeps track of the last time an SMS was sent
const unsigned long smsInterval = 3600000;  // 1 hour (3600000 ms) interval between SMS messages
bool smsSent = false;

// Sensor pins
#define sensorPower 14
#define sensorPin A0

#include <SoftwareSerial.h>
#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <WiFiManager.h>
#include <FirebaseESP8266.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
// //Provide the token generation process info.
#include <addons/TokenHelper.h>
// //Provide the RTDB payload printing info and other helper functions.
#include <addons/RTDBHelper.h>

SoftwareSerial mySerial(1, 3); //SIM800L Tx & Rx is connected to ESP8266 #3 & #2

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000); // Update every 60 seconds

LiquidCrystal_I2C lcd(0x27,16,2);

#define DHTPIN 2     // Digital pin connected to the DHT sensor D4
#define BLYNK_PRINT Serial

//Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;

// Uncomment whatever type you're using!
#define DHTTYPE DHT11   // DHT 11
// Initialize DHT sensor.
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  pinMode(sensorPower, OUTPUT);
	
	// Initially keep the sensor OFF
	digitalWrite(sensorPower, LOW);

  Serial.begin(9600);
  mySerial.begin(9600);

  Serial.println("Initializing...");

  // Initialize DHT sensor
  dht.begin();

  // Initialize the LCD
  lcd.init();
  lcd.clear();
  lcd.backlight();

  // Initialize Wi-Fi and Firebase
  WiFiManager wifiManager;
  wifiManager.autoConnect(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("Connected to Wi-Fi");
  String connectedSSID = WiFi.SSID();

  // Initialize the NTP client
  timeClient.begin();
  timeClient.setTimeOffset(0); // Set the timezone offset in seconds (e.g., for UTC+0)

  /* Assign the api key (required) */
  config.api_key = API_KEY;

  /* Assign the user sign in credentials */
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  Firebase.begin(&config, &auth);
  Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

  //Comment or pass false value when WiFi reconnection will control by your code or third party library
  Firebase.reconnectWiFi(true);

  Firebase.setDoubleDigits(5);

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h

  Blynk.begin(BLYNK_AUTH_TOKEN ,connectedSSID.c_str(), WiFi.psk().c_str(), "blynk.cloud", 80);

  // Initialize SIM800L
  mySerial.println("AT");
  updateSerial();

  Serial.println();

  mySerial.println("AT+CSQ");
  updateSerial();

  mySerial.println("AT+CCID");
  updateSerial();

  mySerial.println("AT+CREG?");
  updateSerial();

  mySerial.println("ATI");
  updateSerial();

  mySerial.println("AT+CBC");
  updateSerial();

  mySerial.println("AT+CMGF=1");  // Set SMS mode to text
  updateSerial();

  // send a test message - sends only once on setup
  send_sms(0, 0, 0);
}

void loop() {
  Blynk.run();
  // Wait a few seconds between measurements.
  delay(1000);

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Apply calibration to the humidity reading
  h += HUMIDITY_CALIBRATION_OFFSET;

  lcd.init();
  lcd.clear();         
  lcd.backlight();      // Make sure backlight is on

  //get the reading from the function below 
	int moisture = readSensor();

////////////////////////////////
  timeClient.update();

   // Get the current epoch time in seconds
  unsigned long currentEpoch = timeClient.getEpochTime();

  // Convert to milliseconds (like JavaScript's Date.now())
  unsigned long long timestampMillis = currentEpoch * 1000ULL;

  // Print the timestamp
  Serial.println(timestampMillis);
  String timestamp = String(timestampMillis);
////////////////////////////////

  // Convert the raw sensor value to percentage (0% = dry, 100% = wet)
  int moisturePercent = map(moisture, 1023, 0, 0, 100);

	Serial.print("Grain Moisture: ");
	Serial.print(moisturePercent);
  Serial.println("%");

	// Determine status of our soil
	if (moisture < GrainWet) {
		Serial.println("Status:  Grain moisture is critical");
	} else if (moisture >= GrainWet && moisture < GrainDry) {
		Serial.println("Status: Grain moisture is increasing");
	} else {
		Serial.println("Status: Grain moisture is perfect!");
	}
	
	delay(1000);	// Take a reading every second for testing
	Serial.println(); 

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

   // Check conditions and send SMS if necessary
  if (t >= 30 && h >= 60 && moisturePercent >= 5) {
    send_sms(t, h, moisturePercent);  // Send SMS if conditions are critical
  } else {
    smsSent = false;  // Reset smsSent if conditions are normal
  }

  Blynk.virtualWrite(V0, t);
  Blynk.virtualWrite(V1, h);
  Blynk.virtualWrite(V2, moisturePercent);

  // Print a message on both lines of the LCD.
  lcd.setCursor(2,0);   //Set cursor to character 2 on line 0
  lcd.print("Temp: " + String(t) + " C");
  lcd.setCursor(2,1);   //Set cursor to character 2 on line 1
  lcd.print("Hum: " + String(h) + " %");

  Serial.println(String("Humidity: ") + h + "%" + " __ Temperature: " + t + "°C __ " + f + "°F");

  if (Firebase.ready() && (millis() - sendDataPrevMillis > 15000 || sendDataPrevMillis == 0))
  {
    sendDataPrevMillis = millis();

    // Set current temperature and humidity in Firebase
    Serial.printf("Set Temperature... %s\n", Firebase.setFloat(fbdo, F("/test/temperature"), t) ? "ok" : fbdo.errorReason().c_str());
    Serial.printf("Set Humidity... %s\n", Firebase.setDouble(fbdo, F("/test/humidity"), h) ? "ok" : fbdo.errorReason().c_str());
    Serial.printf("Set Moisture... %s\n", Firebase.setDouble(fbdo, F("/test/moisture"), moisturePercent) ? "ok" : fbdo.errorReason().c_str());

    // Add the current readings to the "readings" node with a unique timestamp
    FirebaseJson json;
    json.set("/temperature", t);
    json.set("/humidity", h);
    json.set("/moisture", moisturePercent);

    String path = "/test/readings/" + timestamp;
    Serial.printf("Set Readings... %s\n", Firebase.set(fbdo, path.c_str(), json) ? "ok" : fbdo.errorReason().c_str());

    Serial.println();
  }

  updateSerial();
}

void send_sms(float t, float h, int moisturePercent) {
  mySerial.println("AT+CMGS=\"+2349072229642\"");
  updateSerial();

  unsigned long currentMillis = millis();

  if (t > 0 && h > 0 && moisturePercent >= 0) {
    if (!smsSent || (currentMillis - lastSmsSentMillis > smsInterval)) {
      String smsMessage = "Danger, storage conditions above threshold";
      mySerial.print(smsMessage);
      mySerial.write(0x1A);  // ASCII code for CTRL+Z to send the SMS
      updateSerial();
      smsSent = true;
      lastSmsSentMillis = currentMillis;  // Update the time the SMS was sent
    }
  } else {
    String smsMessage = "Setup Test";
    mySerial.print(smsMessage);
    mySerial.write(0x1A);  // ASCII code for CTRL+Z to send the SMS
    updateSerial();
  }
}


void updateSerial() {
  delay(500);
   while (Serial.available()) 
  {
    mySerial.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while(mySerial.available()) 
  {
    Serial.write(mySerial.read());//Forward what Software Serial received to Serial Port
  }
}

//  This function returns the analog soil moisture measurement
int readSensor() {
	digitalWrite(sensorPower, HIGH);	// Turn the sensor ON
	delay(10);							// Allow power to settle
	int val = analogRead(sensorPin);	// Read the analog value form sensor
	digitalWrite(sensorPower, LOW);		// Turn the sensor OFF
	return val;							// Return analog moisture value
}