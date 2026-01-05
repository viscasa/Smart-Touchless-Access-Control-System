#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <time.h>


/* KONFIGURASI PIN */
#define PIN_IR_SENSOR  13
#define PIN_SERVO      14


/* =KONFIGURASI WIFI & WAKTU */
const char* ssid       = "nama_wifi";    
const char* password   = "password_wifi";


// Konfigurasi NTP Server (WIB = UTC+7)
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 25200;
const int   daylightOffset_sec = 0;


/* VARIABEL GLOBAL */
Servo myServo;
QueueHandle_t accessQueue;


volatile bool isSensorActive = false;
volatile bool isDoorBusy = false; 


// MAC Address Target: 88:57:21:94:69:70 (menyesuaikan)
uint8_t broadcastAddress[] = {0x88, 0x57, 0x21, 0x94, 0x69, 0x70};


typedef struct struct_message {
  int id_kelompok;    
  char status[20];    
  int total_akses;
  char waktu_akses[30];
} struct_message;


struct_message myData;
esp_now_peer_info_t peerInfo;
int accessCounter = 0;


/* HELPER METHOD */
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\n[ESP-NOW] Status Paket Terakhir: ");
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("Delivery Success (Terkirim!)");
  } else {
    Serial.println("Delivery Fail (Gagal Kirim!)");
  }
}
void getLocalTimeStr(char *buffer) {
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    // Jika gagal, pakai placeholder
    strcpy(buffer, "Time Syncing...");
    return;
  }
  strftime(buffer, 30, "%Y-%m-%d %H:%M:%S", &timeinfo);
}


// Fungsi Praktis untuk Print Dashboard (Biar codingan rapi)
void printDashboardLog() {
    Serial.println("=============================================");
    Serial.println(">> UPLOADING TO IOT DASHBOARD...");
    Serial.printf("   Device ID : %d\n", myData.id_kelompok);
    Serial.printf("   Status    : %s\n", myData.status);
    Serial.printf("   Timestamp : %s\n", myData.waktu_akses);
    Serial.printf("   Total Acc : %d\n", myData.total_akses);
    Serial.println("=============================================");
}


/* TASK 1: SENSOR */
void taskSensor(void *pvParameters) {
  pinMode(PIN_IR_SENSOR, INPUT);
  bool lastState = HIGH;


  for (;;) {
    int sensorValue = digitalRead(PIN_IR_SENSOR);


    if (sensorValue == LOW) isSensorActive = true;
    else isSensorActive = false;


    if (sensorValue == LOW && lastState == HIGH) {
       vTaskDelay(pdMS_TO_TICKS(50));
       if (digitalRead(PIN_IR_SENSOR) == LOW) {
           if (isDoorBusy == false) {
               int command = 1;
               if(uxQueueMessagesWaiting(accessQueue) == 0) {
                  xQueueSend(accessQueue, &command, 0);
                  Serial.println("\n[SENSOR] Triggered -> Request Open");
               }
           }
       }
    }
    lastState = sensorValue;
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}


/* ================= TASK 2: KONTROL PINTU ================= */
void taskDoorControl(void *pvParameters) {
  int receivedCommand;
  const unsigned long closeDelay = 3000;
 
  const int posOpen = 0;  
  const int posClose = 90;
  const int speedDelay = 20;


  for (;;) {
    if (xQueueReceive(accessQueue, &receivedCommand, portMAX_DELAY) == pdPASS) {
      if (receivedCommand == 1) {
       
        isDoorBusy = true;
        char currentTime[30];
        getLocalTimeStr(currentTime);


        // 1. BUKA PINTU
        Serial.println("[SERVO] Membuka Pintu...");
        for (int pos = posClose; pos >= posOpen; pos -= 10) {
            myServo.write(pos);
            vTaskDelay(pdMS_TO_TICKS(speedDelay));
        }


        // 2. KIRIM STATUS OPEN + LOG DASHBOARD
        accessCounter++;  
        myData.id_kelompok = 11;
        strcpy(myData.status, "OPEN");
        myData.total_akses = accessCounter;
        strcpy(myData.waktu_akses, currentTime);
       
        esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
        printDashboardLog(); // <--- LOG SAAT BUKA


        // 3. LOGIKA HOLD
        unsigned long lastActiveTime = millis();
        while (true) {
            if (isSensorActive) {
                lastActiveTime = millis();
                vTaskDelay(pdMS_TO_TICKS(100));
            } else {
                if (millis() - lastActiveTime >= closeDelay) break;
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }


        // 4. TUTUP PINTU
        Serial.println("[SERVO] Menutup Pintu...");
       
        // Update Waktu Tutup
        getLocalTimeStr(currentTime);
        strcpy(myData.waktu_akses, currentTime);
        strcpy(myData.status, "CLOSED");
       
        esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
        printDashboardLog(); // <--- LOG SAAT TUTUP
       
        for (int pos = posOpen; pos <= posClose; pos += 5) {
            myServo.write(pos);
            vTaskDelay(pdMS_TO_TICKS(speedDelay));
        }


        // 5. RESET
        xQueueReset(accessQueue);
        isDoorBusy = false;


        if (isSensorActive) {
            int command = 1;
            xQueueSend(accessQueue, &command, 0);
        }
      }
    }
  }
}


/* SETUP */
void setup() {
  Serial.begin(115200);
 
  myServo.attach(PIN_SERVO);
  myServo.write(90);


  // --- SETUP WIFI (TIMEOUT MECHANISM) ---
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
 
  Serial.print("Menghubungkan WiFi");
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 20) {
    delay(500);
    Serial.print(".");
    retry++;
  }


// Di Kodingan SENDER (dalam setup)
if(WiFi.status() == WL_CONNECTED){
    Serial.println("\n[OK] WiFi Terhubung! Sinkronisasi Waktu...");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  
    Serial.print("SENDER CONNECT DI CHANNEL: ");
    Serial.println(WiFi.channel());
  } else {
    Serial.println("\n[SKIP] Gagal WiFi. Masuk Mode Offline.");
  }


  // --- ESP-NOW ---
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error ESP-NOW");
    return;
  }
  esp_now_register_send_cb((esp_now_send_cb_t)OnDataSent);
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);


  // --- RTOS ---
  accessQueue = xQueueCreate(5, sizeof(int));
  xTaskCreate(taskSensor, "SensorTask", 2048, NULL, 2, NULL);
  xTaskCreate(taskDoorControl, "DoorTask", 4096, NULL, 1, NULL);


  Serial.println("Sistem IoT Access Control Siap...");
}


void loop() {
}
