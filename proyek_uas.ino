#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

/* ================= KONFIGURASI PIN ================= */
#define PIN_IR_SENSOR  13
#define PIN_SERVO      14

/* ================= VARIABEL GLOBAL ================= */
Servo myServo;
QueueHandle_t accessQueue; 

volatile bool isSensorActive = false; 
volatile bool isDoorBusy = false; 

// MAC Address Receiver
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

typedef struct struct_message {
  int id_kelompok;    
  char status[20];    
  int total_akses;    
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;
int accessCounter = 0; 

/* ================= CALLBACK ESP-NOW ================= */
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Kosong
}

/* ================= TASK 1: SENSOR ================= */
void taskSensor(void *pvParameters) {
  pinMode(PIN_IR_SENSOR, INPUT);
  bool lastState = HIGH; 

  for (;;) {
    int sensorValue = digitalRead(PIN_IR_SENSOR);

    // 1. Update status Real-Time
    if (sensorValue == LOW) {
      isSensorActive = true; 
    } else {
      isSensorActive = false;
    }

    // 2. Logika Deteksi Perubahan (Edge Trigger)
    if (sensorValue == LOW && lastState == HIGH) {
       vTaskDelay(pdMS_TO_TICKS(50)); // Debounce
       if (digitalRead(PIN_IR_SENSOR) == LOW) {
           // Hanya kirim jika pintu sedang TIDAK sibuk
           if (isDoorBusy == false) {
               int command = 1;
               if(uxQueueMessagesWaiting(accessQueue) == 0) {
                  xQueueSend(accessQueue, &command, 0);
                  Serial.println("[SENSOR] Objek Baru -> Kirim Perintah Buka");
               }
           } 
       }
    }
    lastState = sensorValue;
    vTaskDelay(pdMS_TO_TICKS(50)); 
  }
}

/* ================= TASK 2: KONTROL PINTU (SLOW MOTION) ================= */
void taskDoorControl(void *pvParameters) {
  int receivedCommand;
  const unsigned long closeDelay = 3000; 

  // KONFIGURASI POSISI & KECEPATAN
  const int posOpen = 0;   // Derajat Terbuka
  const int posClose = 90; // Derajat Tertutup
  const int speedDelay = 20; 

  for (;;) {
    if (xQueueReceive(accessQueue, &receivedCommand, portMAX_DELAY) == pdPASS) {
      
      if (receivedCommand == 1) {
        
        // Tandai sibuk
        isDoorBusy = true;

        // --- 1. BUKA PINTU ---
        Serial.println("[SERVO] Membuka Pintu Pelan...");
        
        for (int pos = posClose; pos >= posOpen; pos -= 30) {
            myServo.write(pos);
            vTaskDelay(pdMS_TO_TICKS(speedDelay));
        }
        
        // Update ESP-NOW
        accessCounter++;    
        myData.id_kelompok = 9; 
        strcpy(myData.status, "OPEN");
        myData.total_akses = accessCounter;
        esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

        // --- 2. LOGIKA HOLD (Menunggu Orang Pergi) ---
        Serial.println("[SERVO] Menunggu area clear...");
        unsigned long lastActiveTime = millis();

        while (true) {
            if (isSensorActive) {
                // Jika ada orang, reset timer (Pintu TAHAN BUKA)
                lastActiveTime = millis(); 
                vTaskDelay(pdMS_TO_TICKS(100)); 
            } 
            else {
                // Jika sepi, hitung mundur
                if (millis() - lastActiveTime >= closeDelay) {
                    break; // Keluar loop untuk menutup
                }
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }

        // --- 3. TUTUP PINTU ---
        Serial.println("[SERVO] Waktu habis. Menutup Pintu Pelan...");
        
        strcpy(myData.status, "CLOSED");
        esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

        for (int pos = posOpen; pos <= posClose; pos += 15) {
            myServo.write(pos);
            vTaskDelay(pdMS_TO_TICKS(speedDelay)); // Jeda tiap 15 derajat
        }

        // --- 4. RESET STATUS & AUTO-RETRIGGER ---
        
        // Kosongkan antrian lama agar bersih
        xQueueReset(accessQueue);
        
        // Lepas status sibuk
        isDoorBusy = false;

        // CEK Apakah SEKARANG masih ada orang?
        // Jika ya, langsung masukkan perintah baru ke antrian sendiri
        if (isSensorActive) {
            Serial.println("[AUTO-RETRIGGER] Objek masih terdeteksi setelah tutup. Buka lagi!");
            int command = 1;
            xQueueSend(accessQueue, &command, 0);
        }
      }
    }
  }
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  
  myServo.attach(PIN_SERVO);
  myServo.write(90); // Posisi Awal Tertutup

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error ESP-NOW");
    return;
  }

  esp_now_register_send_cb((esp_now_send_cb_t)OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  accessQueue = xQueueCreate(5, sizeof(int));

  xTaskCreate(taskSensor, "SensorTask", 2048, NULL, 2, NULL);
  xTaskCreate(taskDoorControl, "DoorTask", 4096, NULL, 1, NULL);

  Serial.println("Sistem Siap...");
}

void loop() {
  // Kosong
}