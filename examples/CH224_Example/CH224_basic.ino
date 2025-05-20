#include <Arduino.h>
#include <Wire.h>
#include <CH224.h> // 引入 CH224Q.h

#define PD_EN_PIN 8       // PD_EN 引腳
#define USB_DETECT_PIN 9  // USB_DETECT 引腳
#define SDA_PIN 2         // I2C SDA 引腳
#define SCL_PIN 3         // I2C SCL 引腳
#define UP_BUTTON_PIN 20  // 升壓按鍵引腳
#define DOWN_BUTTON_PIN 21 // 降壓按鍵引腳

int lastUsbDetectState = HIGH; // 上一次 USB_DETECT_PIN 的狀態
uint8_t fixedPdoCount = 0;
// 按鍵去抖動相關變數
unsigned long lastDebounceTimeUp = 0;
unsigned long lastDebounceTimeDown = 0;
const unsigned long debounceDelay = 50; // 去抖動延遲時間 (毫秒)

void setup() {
    // 初始化內建 USB 串列，波特率為 115200
    Serial.begin(115200);
    while (!Serial) {
        delay(10); // 等待串列初始化
    }
    Serial.println("ESP32-WROOM USB Serial Initialized");

    // 設定 PD_EN_PIN 和 USB_DETECT_PIN 的模式
    pinMode(PD_EN_PIN, OUTPUT);
    digitalWrite(PD_EN_PIN, LOW); // 初始狀態設為低電平（關閉 CH224Q 電源）
    pinMode(USB_DETECT_PIN, INPUT_PULLUP); // USB_DETECT_PIN 設為輸入模式，帶內部上拉

    // 設定按鍵引腳模式
    pinMode(UP_BUTTON_PIN, INPUT_PULLUP);   // UP 按鍵
    pinMode(DOWN_BUTTON_PIN, INPUT_PULLUP); // DOWN 按鍵

    CH224_Init(SDA_PIN, SCL_PIN);
    Serial.println("CH224Q Initialized");

    // 檢測 USB 是否已經插入
    int usbDetectState = digitalRead(USB_DETECT_PIN);
    if (usbDetectState == LOW) {
        // USB 已插入，開啟 CH224Q 電源
        digitalWrite(PD_EN_PIN, HIGH);
        // 初始化 CH224Q 數據結構並讀取狀態
        Serial.println("USB detected during setup. CH224Q power ON.");
        delay(500); // 等待 IC 開機完成
        CH224_readSourceCap();
       
    } else {
        Serial.println("USB not detected during setup. CH224Q remains OFF.");
    }

    // 記錄當前 USB_DETECT_PIN 的狀態
    lastUsbDetectState = usbDetectState;
}

void loop() {
    // 檢測 USB_DETECT_PIN 的狀態
    int usbDetectState = digitalRead(USB_DETECT_PIN);
    static uint8_t pdoIndex = 0; // 目前選擇的 PDO 位置

    // 當 USB_DETECT_PIN 狀態改變時執行操作
    if (usbDetectState != lastUsbDetectState) {
        lastUsbDetectState = usbDetectState; // 更新狀態

        if (usbDetectState == LOW) {
            // USB_DETECT_PIN 為 LOW，打開 CH224Q 電源
            digitalWrite(PD_EN_PIN, HIGH);
            Serial.println("USB detected. CH224Q power ON.");
            delay(500); // 等待 IC 開機完成
            // 初始化 CH224Q 數據結構並讀取狀態
       
          if (  CH224_readSourceCap() == true)
          {
              Serial.println("CH224Q read source capability success.");
              fixedPdoCount = CH224_GetFixedPDO_Count(); // 獲取固定電壓 PDO 數量
                if (fixedPdoCount > 0) {
                    pdoIndex = 0; // 重置 PDO 索引
                    Serial.printf("Fixed PDO count: %d\n", fixedPdoCount);
                } else {
                    Serial.println("No fixed voltage PDO available.");
                }
          }
          else
          {
              Serial.println("CH224Q read source capability failed.");
          }
    
          
        
        } else {
            // USB_DETECT_PIN 為 HIGH，關閉 CH224Q 電源
            digitalWrite(PD_EN_PIN, LOW);
            pdoIndex = 0; // 重置 PDO 索引
            Serial.println("USB not detected. CH224Q power OFF.");
        }
    }

    // 只有當 CH224Q 電源打開時才處理按鍵操作
    if (digitalRead(PD_EN_PIN) == HIGH) {
        // 處理 UP 按鍵
        static int lastUpButtonState = HIGH;
        int upButtonState = digitalRead(UP_BUTTON_PIN);
          // 處理 DOWN 按鍵
        static int lastDownButtonState = HIGH;
        int downButtonState = digitalRead(DOWN_BUTTON_PIN);
        
      

        if (fixedPdoCount == 0) {
            // 沒有可用的固定電壓 PDO
            lastUpButtonState = upButtonState;
            lastDownButtonState = digitalRead(DOWN_BUTTON_PIN);
            return;
        }

        if (upButtonState == LOW && lastUpButtonState == HIGH && (millis() - lastDebounceTimeUp > debounceDelay)) {
            lastDebounceTimeUp = millis();
            // 選擇下一個 PDO
            pdoIndex = (pdoIndex + 1) % fixedPdoCount;
            if (CH224_Fixed_Request(PD_Msg[pdoIndex][1]/1000)) {
            Serial.printf("Switched to : %d ",PD_Msg[pdoIndex][1]);
            Serial.println(pdoIndex);
            } else {
            Serial.println("Failed to Request fixed PDO (UP)");
            }
        }
        lastUpButtonState = upButtonState;

      
        if (downButtonState == LOW && lastDownButtonState == HIGH && (millis() - lastDebounceTimeDown > debounceDelay)) {
            lastDebounceTimeDown = millis();
            // 選擇上一個 PDO
            if (pdoIndex == 0)
            pdoIndex = fixedPdoCount - 1;
            else
            pdoIndex--;
             if (CH224_Fixed_Request(PD_Msg[pdoIndex][1]/1000)) {
            Serial.printf("Switched to : %d ",PD_Msg[pdoIndex][1]);
            Serial.println(pdoIndex);
            } else {
            Serial.println("Failed to Request fixed PDO (UP)");
            }
        }
        lastDownButtonState = downButtonState;
    }

    // 延遲 50 毫秒進行下一次檢測
    delay(50);
}