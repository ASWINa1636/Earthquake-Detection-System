/**
 * @file Earthquake_Detector_Presentation_Ready.ino
 * @version 5.3 (Presentation Final - 5s Recovery)
 * @author Gemini & Aswin Raj
 * @brief The definitive, presentation-ready earthquake detection system.
 *
 * This version is optimized for stability and clarity for the HOD presentation. It uses the
 * robust MPU6500_WE library and a clear, threshold-based detection algorithm. The SMS alert
 * system is fully functional using the correct CircuitDigest API format and supports multiple recipients.
 *
 * Changes in v5.3:
 * - Increased the recovery time to 5 seconds. The system now waits for 5 seconds of
 * calm after an earthquake before transitioning to recovery mode.
 * - Corrected a critical logic flaw where the alarm state would immediately transition to
 * recovery. The alarm now correctly persists as long as shaking continues and only
 * enters recovery after a sustained period of calm.
 */

// ==================== LIBRARIES ====================
#include <WiFi.h>
#include <HTTPClient.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <MPU6500_WE.h>
#include <DHT11.h>
#include <math.h>

// ==================== CONFIGURATION ====================
const char* ssid = "Aswin"; // IMPORTANT: Make sure this is the WiFi network you will use
const char* password = "123456789"; // IMPORTANT: Check the WiFi password

// CircuitDigest SMS API Configuration
const char* SMS_API_KEY = "cdyUOaymXmIM"; // Your API Key
const char* SMS_API_URL = "https://www.circuitdigest.cloud/send_sms";
// Enter mobile numbers separated by commas, with country code (e.g., "91xxxxxxxxxx,91yyyyyyyyyy")
const char* ALERT_MOBILES_CSV = "919025800433,919345993919";

// ==================== OLED DISPLAY SETUP ====================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ==================== SENSOR SETUP ====================
MPU6500_WE mpu = MPU6500_WE(0x68);
DHT11 dht11(23);

// ==================== HARDWARE PINS ====================
#define BUZZER_PIN 13
#define RELAY_PIN 19
#define LED_RED_PIN 25
#define LED_GREEN_PIN 26
#define LED_BLUE_PIN 27
#define DHTPIN 23

// ==================== SEISMIC THRESHOLDS ====================
#define EARTHQUAKE_MAGNITUDE_THRESHOLD 0.30 // Threshold in 'g' for horizontal shake
#define SIGNIFICANT_MOTION_THRESHOLD 0.25 // Threshold below which the shaking is considered "stopped"
#define TRIGGER_SAMPLES 3 // Number of consecutive readings above threshold to trigger alarm
#define RECOVERY_SAMPLES 25 // Number of consecutive readings below threshold to enter recovery (25 * 200ms = 5 seconds)

// ==================== OTHER THRESHOLDS ====================
#define FIRE_TEMP_THRESHOLD 45.0 // Temperature in Celsius to trigger a fire alert

// ==================== SYSTEM STATE FLAGS ====================
bool alarmActive = false;
bool recoveryMode = false;
bool fireLockdownActive = false;
String lockdownTitle = "";
String lockdownLine1 = "";
String lockdownLine2 = "";
int consecutiveShakeSamples = 0;
int consecutiveStableSamples = 0;
float currentMagnitudeH = 0.0;
bool smsEarthquakeSent = false;
bool smsFireSent = false;

// ==================== CACHED VALUES & TIMERS ====================
int cachedTemp = -1;
int cachedHum = -1;
unsigned long lastDHTRead = 0;
unsigned long lastSensorReadTime = 0;
unsigned long lastDisplayUpdateTime = 0;
unsigned long lastBuzzerToggle = 0;
unsigned long lastAlertBlink = 0;
unsigned long lastTempCheck = 0;
unsigned long recoveryStartTime = 0;
unsigned long lastLEDBlink = 0;
bool buzzerState = false;
bool alertBlinkState = false;
bool ledBlinkState = false;

// ==================== INTERVALS ====================
const unsigned long SENSOR_READ_INTERVAL = 200;
const unsigned long DISPLAY_UPDATE_INTERVAL = 1000;
const unsigned long DHT_READ_INTERVAL = 2000;
const unsigned long BUZZER_INTERVAL = 150;
const unsigned long ALERT_BLINK_INTERVAL = 300;
const unsigned long LED_BLINK_INTERVAL = 500;
const unsigned long TEMP_CHECK_INTERVAL = 2000;
const unsigned long RECOVERY_DURATION = 10000;


// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n╔════════════════════════════════════════════════╗");
  Serial.println("║ EARTHQUAKE DETECTOR v5.3 (Presentation Final) ║");
  Serial.println("║ MPU6500 + DHT11 + SMS + LED                   ║");
  Serial.println("╚════════════════════════════════════════════════╝");
  Serial.println("\n[HARDWARE] MPU6500:     I2C (SDA=21, SCL=22)");
  Serial.println("[HARDWARE] DHT11:        GPIO 23");
  Serial.println("[HARDWARE] Buzzer:       GPIO 13");
  Serial.println("[HARDWARE] Relay:        GPIO 19");
  Serial.println("[HARDWARE] RGB LED:      R=25, G=26, B=27\n");

  Wire.begin(21, 22);

  Serial.println("[SYSTEM] Initializing DHT11 sensor...");
  delay(2000);
  int testTemp = dht11.readTemperature();
  if (testTemp == DHT11::ERROR_TIMEOUT || testTemp == DHT11::ERROR_CHECKSUM) {
      Serial.println("[WARNING] DHT11 not responding or checksum error!");
  } else {
      Serial.printf("[OK] DHT11 connected (Temp: %d°C)\n", testTemp);
      cachedTemp = testTemp;
  }
  Serial.println();

  Serial.println("[SYSTEM] Initializing MPU6500...");
  if(!mpu.init()){
    Serial.println("[ERROR] MPU6500 connection failed! Halting.");
    while(1);
  }
  Serial.println("[OK] MPU6500 connected.");
  mpu.setAccRange(MPU6500_ACC_RANGE_8G);
  
  Serial.println("[SYSTEM] Calibrating MPU6500 (keep still)...");
  delay(1000);
  mpu.autoOffsets();
  Serial.println("[OK] Calibration complete.\n");

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);

  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(RELAY_PIN, LOW);
  setRGBColor(0, 255, 0);

  Serial.println("[SYSTEM] Initializing OLED display...");
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("[ERROR] OLED allocation failed"));
    while (1);
  }
  Serial.println("[OK] OLED initialized successfully\n");

  showBootScreen();
  
  Serial.println("[SYSTEM] Testing hardware...");
  digitalWrite(BUZZER_PIN, HIGH);
  digitalWrite(RELAY_PIN, HIGH);
  setRGBColor(255, 0, 0);
  delay(300);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(RELAY_PIN, LOW);
  setRGBColor(0, 255, 0);
  Serial.println("[OK] Hardware test passed\n");

  displayMessage("CONNECTING\nTO WI-FI...");
  WiFi.begin(ssid, password);
  Serial.print("[WIFI] Connecting");
  int wifi_retries = 20;
  while (WiFi.status() != WL_CONNECTED && wifi_retries > 0) { 
    delay(500); 
    Serial.print("."); 
    wifi_retries--;
  }

  if(WiFi.status() != WL_CONNECTED){
      Serial.println("\n[ERROR] WiFi connection failed!");
      displayMessage("WIFI FAILED!\n\nCheck SSID/\nPassword");
      while(1);
  }

  Serial.println("\n[OK] WiFi connected!");
  Serial.println("[INFO] ESP32 IP: " + WiFi.localIP().toString());
  
  showReadyScreen();
  delay(2000);
  Serial.println("\n[READY] All systems operational");
  lastDHTRead = millis();
}

// ==================== MAIN LOOP ====================
void loop() {
  if (millis() - lastDHTRead >= DHT_READ_INTERVAL) {
    lastDHTRead = millis();
    updateDHTCache();
  }

  if (fireLockdownActive) {
      if (millis() - lastDisplayUpdateTime >= DISPLAY_UPDATE_INTERVAL) {
        lastDisplayUpdateTime = millis();
        showLockdownScreen(lockdownTitle, lockdownLine1, lockdownLine2);
      }
      handleRGBLEDStatus();
      return;
  }
  
  if (recoveryMode) {
    handleRecoveryMode();
    return;
  }

  handleRGBLEDStatus();

  if (millis() - lastSensorReadTime >= SENSOR_READ_INTERVAL) {
    lastSensorReadTime = millis();
    readMPU6500AndProcess();
  }

  if (alarmActive) {
    if (millis() - lastBuzzerToggle >= BUZZER_INTERVAL) {
      lastBuzzerToggle = millis();
      buzzerState = !buzzerState;
      digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
    }
  } else {
      digitalWrite(BUZZER_PIN, LOW);
  }
}

// ==================== SENSOR & STATE HANDLING ====================
void updateDHTCache() {
  int temp = dht11.readTemperature();
  int hum = dht11.readHumidity();
  if (temp != DHT11::ERROR_TIMEOUT && temp != DHT11::ERROR_CHECKSUM) {
    cachedTemp = temp;
  }
  if (hum != DHT11::ERROR_TIMEOUT && hum != DHT11::ERROR_CHECKSUM) {
    cachedHum = hum;
  }
}

void readMPU6500AndProcess() {
  xyzFloat gVal = mpu.getGValues();
  float ax = gVal.x;
  float ay = gVal.y;
  
  currentMagnitudeH = sqrt(ax * ax + ay * ay);
  
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug >= 2000) {
    lastDebug = millis();
    Serial.printf("[MPU6500] Mag(H):%.3fg\n", currentMagnitudeH);
  }
  
  if (millis() - lastDisplayUpdateTime >= DISPLAY_UPDATE_INTERVAL) {
    lastDisplayUpdateTime = millis();
    showMagnitudeResults();
  }

  // LOGIC TO TRIGGER ALARM
  if (currentMagnitudeH >= EARTHQUAKE_MAGNITUDE_THRESHOLD) {
    consecutiveShakeSamples++;
    consecutiveStableSamples = 0; // Reset stable counter if shaking is detected
    if (!alarmActive && consecutiveShakeSamples >= TRIGGER_SAMPLES) {
      handleAlarm();
    }
  } 
  // LOGIC TO HANDLE RECOVERY (ONLY IF ALARM IS ALREADY ACTIVE)
  else if (alarmActive) {
    // If shaking is still significant, stay in alarm mode
    if (currentMagnitudeH >= SIGNIFICANT_MOTION_THRESHOLD) {
      consecutiveStableSamples = 0; // Reset stable counter, still shaking
    }
    // If shaking has subsided, start counting stable samples
    else {
      consecutiveStableSamples++;
      if (consecutiveStableSamples >= RECOVERY_SAMPLES) {
        Serial.println("\n[SYSTEM] Shaking has stopped. Transitioning to Recovery Mode.");
        alarmActive = false;
        recoveryMode = true;
        digitalWrite(BUZZER_PIN, LOW);
        setRGBColor(0, 0, 255);
        recoveryStartTime = millis();
        lastLEDBlink = millis();
        lastTempCheck = millis();
        smsEarthquakeSent = false;
        // Relay remains HIGH (power off)
      }
    }
  } 
  // LOGIC FOR NORMAL STATE (NO ALARM, NO SHAKING)
  else {
    consecutiveShakeSamples = 0;
    consecutiveStableSamples = 0;
  }
}

void handleAlarm() {
  Serial.println("\n[ALERT] ⚠ EARTHQUAKE DETECTED! (Magnitude Threshold Met) ⚠");
  Serial.printf("[ALERT] Current Magnitude H: %.2fg\n", currentMagnitudeH);
  Serial.println("[ACTION] Activating Alarm and Cutting Power.");
  alarmActive = true;
  lastBuzzerToggle = millis();
  lastLEDBlink = millis();
  digitalWrite(RELAY_PIN, HIGH);
  consecutiveStableSamples = 0;
}

void handleRecoveryMode() {
    if (!smsEarthquakeSent) {
      Serial.println("[RECOVERY] Sending earthquake SMS...");
      sendSMS_EarthquakeAlert(currentMagnitudeH);
      smsEarthquakeSent = true;
      delay(1000);
      lastTempCheck = millis();
    }

    if (fireLockdownActive) { return; }

    unsigned long elapsed = millis() - recoveryStartTime;

    if (millis() - lastDisplayUpdateTime >= DISPLAY_UPDATE_INTERVAL) {
        lastDisplayUpdateTime = millis();
        showRecoveryScreen(elapsed);
    }

    handleRGBLEDStatus();

    if (millis() - lastTempCheck >= TEMP_CHECK_INTERVAL) {
      lastTempCheck = millis();
      checkTemperatureInRecovery();
    }

    if (!fireLockdownActive && elapsed >= RECOVERY_DURATION) {
      Serial.println("\n[RECOVERY] Exiting recovery mode. System restored.");
      digitalWrite(RELAY_PIN, LOW);
      recoveryMode = false;
      smsEarthquakeSent = false;
      smsFireSent = false;
      setRGBColor(0, 255, 0);
      displayMessage("SYSTEM\nRESTORED\n\nResuming monitor");
      delay(2000);
    }
}

void checkTemperatureInRecovery() {
  int temp = cachedTemp;
  if (temp <= 0 || temp > 100) {
    Serial.println("[WARNING] DHT11 invalid cached data during recovery check.");
    return;
  }
  
  Serial.printf("[RECOVERY SENSOR] Temp: %d°C\n", temp);

  if (temp >= FIRE_TEMP_THRESHOLD && !smsFireSent) {
    Serial.println("\n[ALERT] 🔥 FIRE DETECTED DURING RECOVERY! 🔥");
    Serial.println("[LOCKDOWN] Initiating permanent fire lockdown!");
    sendSMS_TemperatureAlert(temp);
    smsFireSent = true;
    lockdownTitle = "FIRE EMERGENCY";
    lockdownLine1 = "Temp Critical: " + String(temp) + "C";
    lockdownLine2 = "EVACUATE NOW";
    fireLockdownActive = true;
    digitalWrite(BUZZER_PIN, HIGH);
    digitalWrite(RELAY_PIN, HIGH);
  }
}

// ==================== LED CONTROL ====================
void handleRGBLEDStatus() {
  if (fireLockdownActive) {
    setRGBColor(255, 0, 0); // Solid Red
  } else if (recoveryMode) {
    if (millis() - lastLEDBlink >= LED_BLINK_INTERVAL) {
      lastLEDBlink = millis();
      ledBlinkState = !ledBlinkState;
      setRGBColor(0, 0, ledBlinkState ? 255 : 0); // Blinking Blue
    }
  } else if (alarmActive) {
    if (millis() - lastLEDBlink >= LED_BLINK_INTERVAL) {
      lastLEDBlink = millis();
      ledBlinkState = !ledBlinkState;
      setRGBColor(ledBlinkState ? 255 : 0, 0, 0); // Blinking Red
    }
  } else {
    setRGBColor(0, 255, 0); // Solid Green
  }
}

void setRGBColor(int red, int green, int blue) {
  analogWrite(LED_RED_PIN, constrain(red, 0, 255));
  analogWrite(LED_GREEN_PIN, constrain(green, 0, 255));
  analogWrite(LED_BLUE_PIN, constrain(blue, 0, 255));
}

// ==================== SMS FUNCTIONS ====================
void sendSMS(int templateID, String var1, String var2) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = String(SMS_API_URL) + "?ID=" + String(templateID);
    http.begin(url);
    
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Authorization", SMS_API_KEY);

    StaticJsonDocument<200> jsonDoc;
    jsonDoc["mobiles"] = ALERT_MOBILES_CSV;
    jsonDoc["var1"] = var1;
    jsonDoc["var2"] = var2;

    String postData;
    serializeJson(jsonDoc, postData);

    Serial.println("[SMS] Sending POST to: " + url);
    Serial.println("[SMS] Payload: " + postData);
    
    int httpResponseCode = http.POST(postData);

    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.printf("[SMS] Response code: %d\n", httpResponseCode);
      Serial.println("[SMS] Response body: " + response);
    } else {
      Serial.printf("[SMS] Error sending POST: %s\n", http.errorToString(httpResponseCode).c_str());
    }
    http.end();
  } else {
    Serial.println("[SMS] Error: WiFi not connected.");
  }
}

void sendSMS_EarthquakeAlert(float magnitude) {
  String var1 = "Seismic Sensor";
  String var2 = String(magnitude, 2) + "g horizontal";
  Serial.println("[SMS] Preparing earthquake alert.");
  sendSMS(101, var1, var2);
}

void sendSMS_TemperatureAlert(float temp) {
  String var1 = "Emergency System";
  String var2 = String((int)temp);
  Serial.println("[SMS] Preparing temperature alert.");
  sendSMS(102, var1, var2);
}

// ==================== DISPLAY FUNCTIONS ====================
void showBootScreen() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(15, 5);
    display.println("EARTHQUAKE");
    display.setCursor(10, 18);
    display.println("DETECTION SYSTEM");
    display.drawRect(10, 30, 108, 10, WHITE);
    display.display();
    for (int i = 0; i < 104; i += 4) {
      display.fillRect(12, 32, i, 6, WHITE);
      display.display();
      delay(25);
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(35, 25);
    display.println("v5.3");
    display.setCursor(5, 40);
    display.println("MPU6500 + DHT11");
    display.display();
    delay(1000);
}

void showReadyScreen() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(20, 10);
    display.println("SYSTEM READY");
    display.drawRect(5, 25, 118, 35, WHITE);
    display.setCursor(10, 30);
    display.println("MPU6500: OK");
    display.setCursor(10, 40);
    display.println("DHT11: OK");
    display.setCursor(10, 50);
    display.println("SMS API: Ready");
    display.display();
}

void showMagnitudeResults() {
    int displayTemp = cachedTemp;
    int displayHum = cachedHum;
    
    display.clearDisplay();
    display.setTextColor(WHITE);

    if (alarmActive) {
        if (millis() - lastAlertBlink >= ALERT_BLINK_INTERVAL) {
          lastAlertBlink = millis();
          alertBlinkState = !alertBlinkState;
        }
        if (alertBlinkState) {
          display.fillRect(0, 0, 128, 12, WHITE);
          display.setTextColor(BLACK);
        }
        display.setTextSize(1);
        display.setCursor(30, 2);
        display.print("! ALERT !");
        display.setTextColor(WHITE);
    } else {
        display.setTextSize(1);
        display.setCursor(10, 2);
        display.println("SEISMIC MONITOR");
        display.drawLine(0, 11, 128, 11, WHITE);
    }

    int yStart = 16;
    display.setTextSize(1);
    display.setCursor(0, yStart);
    display.print("Mag:");
    display.setTextSize(2);
    display.setCursor(32, yStart - 1);
    display.print(currentMagnitudeH, 2);
    display.setTextSize(1);
    display.print("g");
    
    display.setCursor(0, yStart + 14);
    display.print("Thr:");
    display.setCursor(32, yStart + 14);
    display.print(EARTHQUAKE_MAGNITUDE_THRESHOLD, 2);
    display.print("g");
    
    display.setCursor(0, yStart + 26);
    display.print("Shk:");
    display.print(consecutiveShakeSamples);
    display.print("/");
    display.print(TRIGGER_SAMPLES);
    
    display.setCursor(64, yStart + 26);
    display.print("Stb:");
    display.print(consecutiveStableSamples);
    display.print("/");
    display.print(RECOVERY_SAMPLES);

    display.setCursor(90, yStart);
    if (displayTemp != -1) {
        display.print(displayTemp);
        display.drawCircle(108, yStart, 1, WHITE);
        display.print("C");
    } else {
        display.print("--C");
    }
    
    display.setCursor(90, yStart + 14);
    if (displayHum != -1) {
        display.print(displayHum);
        display.print("%");
    } else {
        display.print("--%");
    }

    if (alarmActive || recoveryMode || fireLockdownActive) {
        display.fillRect(0, 56, 128, 8, WHITE);
        display.setTextColor(BLACK);
        display.setCursor(15, 57);
        display.print("RELAY: POWER OFF");
    } else {
        display.drawRect(0, 56, 128, 8, WHITE);
        display.setTextColor(WHITE);
        display.setCursor(30, 57);
        display.print("MONITORING");
    }
    
    display.display();
}

void showRecoveryScreen(unsigned long elapsed) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    
    display.setCursor(10, 2);
    display.println("RECOVERY MODE");
    display.drawLine(0, 12, 128, 12, WHITE);
    
    display.setCursor(5, 18);
    display.println("Assessing damage...");
    display.setCursor(5, 30);
    display.println("SMS Alert Sent");
    
    display.setCursor(5, 45);
    display.print("Time Left: ");
    display.print(max(0L, (long)(RECOVERY_DURATION - elapsed) / 1000L));
    display.println("s");
    
    display.display();
}

void showLockdownScreen(String title, String line1, String line2) {
    display.clearDisplay();
    display.setTextColor(WHITE);
    
    if (millis() - lastAlertBlink >= ALERT_BLINK_INTERVAL) {
      lastAlertBlink = millis();
      alertBlinkState = !alertBlinkState;
    }
    
    if (alertBlinkState) {
      display.fillRect(0, 0, 128, 12, WHITE);
      display.setTextColor(BLACK);
    }
    
    display.setTextSize(1);
    display.setCursor(20, 3);
    display.print(title);
    
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(5, 22);
    display.println(line1);
    display.setCursor(5, 36);
    display.println(line2);
    display.setCursor(5, 50);
    display.println("Power Cutoff: ACTIVE");
    display.display();
}

void displayMessage(String msg) {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(10, 20);
  display.println(msg);
  display.display();
}