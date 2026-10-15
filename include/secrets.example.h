// Copy to include/secrets.h (gitignored) and fill in. BLE provisioning replaces this later.
#pragma once
#define WIFI_SSID      "your-2.4GHz-ssid"
#define WIFI_PASS      "your-password"
// Plain http for the local backend/simulator, e.g. "http://192.168.1.50:8080"; https for a real server.
#define SERVER_URL     "http://192.168.1.50:8080"
// From the backend provisioning record: deviceId = 16 hex chars, signingKey = 32 hex chars (AES-128 key, used here as HMAC key).
#define DEVICE_ID      "0123456789abcdef"
#define SIGNING_KEY    "00112233445566778899aabbccddeeff"
// Root CA (PEM) for https URLs. Leave empty "" to skip verification (testing only!).
#define SERVER_CA_PEM  ""
