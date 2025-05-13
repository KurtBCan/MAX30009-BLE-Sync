#include <bluefruit.h>
#include <RTClib.h>

RTC_DS3231 rtc;

#define SYNC_PULSE_PIN D2
#define MANUFACTURER_ID 0x0606

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Controller: Setup");

  pinMode(SYNC_PULSE_PIN, OUTPUT);
  digitalWrite(SYNC_PULSE_PIN, LOW);

  if (!rtc.begin()) {
    Serial.println("RTC not found!");
    while (1);
  }

  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("TimeBase");

  // Setup advertising parameters
  // Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setType(BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED);
  Bluefruit.Advertising.setInterval(160, 160); // 100 ms
  Bluefruit.Advertising.setFastTimeout(1); // 1 second, but we'll stop manually
  Bluefruit.Advertising.clearData();
}

void sendSyncAdvertisement(uint32_t timestamp) {
  uint8_t advData[6];
  advData[0] = lowByte(MANUFACTURER_ID);
  advData[1] = highByte(MANUFACTURER_ID);
  memcpy(&advData[2], &timestamp, sizeof(uint32_t));

  Bluefruit.Advertising.clearData();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addData(BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, advData, sizeof(advData));

  Serial.print("Advertising timestamp: ");
  Serial.println(timestamp);

  // Short sync pulse
  digitalWrite(SYNC_PULSE_PIN, HIGH);
  delayMicroseconds(100);
  digitalWrite(SYNC_PULSE_PIN, LOW);

  Bluefruit.Advertising.start(1); // advertise for 1 second
}

void loop() {
  static bool sentFirst = false;
  static bool sentSecond = false;
  static uint32_t lastSendTime = 0;

  uint32_t now = millis();

  if (!sentFirst) {
    sendSyncAdvertisement(rtc.now().unixtime());
    lastSendTime = now;
    sentFirst = true;
  }

  // Wait ~1 second, then send second
  if (sentFirst && !sentSecond && (now - lastSendTime >= 1000)) {
    sendSyncAdvertisement(rtc.now().unixtime());
    sentSecond = true;
    Serial.println("Both sync packets sent");
  }

  if (sentFirst && sentSecond) {
    while (1); // Done
  }
}
