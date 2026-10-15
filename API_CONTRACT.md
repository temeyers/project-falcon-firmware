# HTTP contract used by firmware 0.1 (matches the existing backend, no changes needed)

`POST {SERVER_URL}/v1/ingest` (laundry-sensor-backend/src/app.ts)

Headers: `content-type: application/json`, `x-device-id: <16-hex deviceId>`,
`x-signature: hex HMAC-SHA256(key = signingKey hex-decoded, data = exact raw body)`.

Body: `{"c":<counter u32, NVS-persisted, strictly increasing>,"t":"<event>","v":3,"r":<WiFi RSSI dBm>,"up":<uptime s>,"f":0,"ack":<cmdId, optional>}`
Battery `bv`/`bp` are omitted (the schema has them optional, not nullable; USB-powered, so null).

Event mapping: open→`door_open`, close→`door_close`, moved→`tamper`, running (30 s vibration)→`cycle_start`,
load done→`cycle_end`, `heartbeat` (15 min), `boot`, `ack`.

Reply 200: `{"ok":true,"c":N,"a":0|1,"cmds":[{"cmdId":N,"type":"arm|disarm|set_heartbeat_hrs|reboot","arg":N}]}`.
Firmware acks a cmdId in `ack` on its next message. 409 `{"error":"replay","lastCounter":N}` → resync counter. 401 → bad key/id.
Rate limit 30/min/device; firmware sends ≤1/s.

## Firmware 0.2 behaviour
- 409: counter := lastCounter, then the same event is retried with lastCounter+1 (not dropped).
- Acks sit in a FIFO and go out one per message, oldest first. Extra `ack` events are sent until the FIFO is empty. Only commands that were actually applied are acked: arm, disarm, set_heartbeat_hrs (1–168 h, saved to NVS), and reboot (acked after the reboot). Unknown or invalid commands are not acked.
- At most 1 message per 2 s. On 429, the firmware honours Retry-After, or otherwise backs off exponentially from 4 s up to 5 min. Network errors and 5xx back off the same way.
- The reply's `c` must equal the sent counter. If it doesn't, the commands in that reply are ignored.
- `f` = 0 (only bit2 = charging is defined; unknown on USB).
