#include <bluefruit.h>
#include <RTClib.h>

#define SYNC_PULSE_PIN D2
#define MANUFACTURER_ID 0x0606

RTC_DS3231 rtc;

// Generate 100 µs sync pulse
void syncPulse() {
  digitalWrite(SYNC_PULSE_PIN, HIGH);
  delayMicroseconds(100);
  digitalWrite(SYNC_PULSE_PIN, LOW);
}

void scan_callback(ble_gap_evt_adv_report_t* report) {
  const uint8_t* data = report->data.p_data;
  uint8_t len = report->data.len;

  // Parse advertisement
  uint8_t index = 0;
  while (index < len) {
    uint8_t field_len = data[index];
    if (field_len == 0 || index + field_len >= len) break;

    uint8_t ad_type = data[index + 1];
    if (ad_type == 0xFF && field_len >= 6) { // Manufacturer specific
      uint16_t mfg_id = data[index + 2] | (data[index + 3] << 8);
      if (mfg_id == MANUFACTURER_ID) {
        uint32_t timestamp;
        memcpy(&timestamp, &data[index + 4], sizeof(uint32_t));

        uint32_t local = rtc.now().unixtime();
        int32_t offset = (int32_t)local - (int32_t)timestamp;

        Serial.print("Adv timestamp: ");
        Serial.print(timestamp);
        Serial.print(" | RTC: ");
        Serial.print(local);
        Serial.print(" | Offset: ");
        Serial.println(offset);

        syncPulse(); // Output 100 µs sync pulse
        break; // Done after first valid match
      }
    }
    index += field_len + 1;
  }
  Bluefruit.Scanner.stop();
  // delay(10);  // Small delay to ensure stack cleans up
  Bluefruit.Scanner.start(0);  // Restart scanning indefinitely
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Node: Starting BLE scan");

  pinMode(SYNC_PULSE_PIN, OUTPUT);
  digitalWrite(SYNC_PULSE_PIN, LOW);

  rtc.begin();
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, setting time to compile time");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80); // 100 ms scan interval, 50% duty
  Bluefruit.Scanner.useActiveScan(false);
  Bluefruit.Scanner.start(0); // Start forever
}

void loop() {
  // Nothing needed in loop
}
