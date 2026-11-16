#include "mqtt_logger.h"
#include "secrets.h"
#include "web_ui.h"
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Raw sample streams published to MQTT, batched into fixed-size chunks rather
// than waiting for a wall-clock second and splitting it unevenly. At the IMU's
// 1.92kHz ODR, 1920/128 = 15 chunks/sec/stream exactly, so a chunk's position
// within a "nominal second" is always recoverable later from seq0 alone — no
// window/chunk-index bookkeeping needs to live in firmware. See the imu/meta
// topic for the constants (chunk size, ODR, scale factors) an offline unpacker
// needs to make sense of the raw counts shipped here.

constexpr uint16_t CHUNK_SIZE                 = 128;
constexpr uint32_t ODR_HZ                     = 1920;
constexpr uint16_t CHUNKS_PER_NOMINAL_SECOND  = ODR_HZ / CHUNK_SIZE;  // 15
constexpr uint32_t RINGBUF_CAPACITY           = 2 * ODR_HZ;          // ~2s headroom
constexpr uint32_t MQTT_RETRY_INTERVAL_MS     = 10000;               // matches WIFI_RETRY_INTERVAL_MS
constexpr uint32_t META_INTERVAL_MS           = 5000;
constexpr uint8_t  MAX_CHUNKS_PER_SERVICE_CALL = 2;  // caps backlog catch-up burst after a reconnect
constexpr size_t   JSON_BUF_SIZE              = 8000; // stays under MQTT_MAX_PACKET_SIZE (8192)

struct LowGSample {
    uint32_t tUs;
    int16_t  ax, ay, az, gx, gy, gz;
};

struct HighGSample {
    uint32_t tUs;
    int16_t  hgx, hgy, hgz;
};

// Minimal circular buffer with peek/consume split so a failed publish can
// retry the same chunk next service() call instead of losing it. On overflow
// (buffer full longer than ~2s of outage), the oldest unconsumed sample is
// overwritten and nextSeqToPublish jumps — the resulting seq0 gap is how a
// downstream unpacker detects the (otherwise unavoidable) data loss.
template <typename T, uint32_t CAPACITY>
struct RingBuffer {
    T        buf[CAPACITY];
    uint32_t head = 0;
    uint32_t count = 0;
    uint32_t nextSeqToPublish = 0;

    void push(const T& sample) {
        buf[head] = sample;
        head = (head + 1) % CAPACITY;
        if (count < CAPACITY) {
            count++;
        } else {
            nextSeqToPublish++;
        }
    }

    bool chunkReady(uint16_t chunkSize) const { return count >= chunkSize; }

    void peekChunk(T* out, uint16_t chunkSize, uint32_t& seq0) const {
        seq0 = nextSeqToPublish;
        const uint32_t tail = (head + CAPACITY - count) % CAPACITY;
        for (uint16_t i = 0; i < chunkSize; i++) {
            out[i] = buf[(tail + i) % CAPACITY];
        }
    }

    void consumeChunk(uint16_t chunkSize) {
        count -= chunkSize;
        nextSeqToPublish += chunkSize;
    }
};

static RingBuffer<LowGSample, RINGBUF_CAPACITY>  lowgRing;
static RingBuffer<HighGSample, RINGBUF_CAPACITY> highgRing;

static LowGSample  lowgChunkScratch[CHUNK_SIZE];
static HighGSample highgChunkScratch[CHUNK_SIZE];
static char        jsonScratch[JSON_BUF_SIZE];

static WiFiClient   wifiClient;
static PubSubClient mqttClient(wifiClient);

static uint32_t lastMqttAttemptMs = 0;
static uint32_t lastMetaMs        = 0;
static bool     mqttConfigured    = false;

static char topicLowG[64];
static char topicHighG[64];
static char topicMeta[64];

static void buildTopics() {
    snprintf(topicLowG, sizeof(topicLowG), "%s/imu/lowg_gyro", OTA_HOSTNAME);
    snprintf(topicHighG, sizeof(topicHighG), "%s/imu/highg", OTA_HOSTNAME);
    snprintf(topicMeta, sizeof(topicMeta), "%s/imu/meta", OTA_HOSTNAME);
}

static size_t buildLowGChunkJson(char* out, size_t outSize, uint32_t seq0,
                                  const LowGSample* s, uint16_t n) {
    size_t pos = 0;
    pos += snprintf(out + pos, outSize - pos, "{\"seq0\":%lu,\"n\":%u,\"d\":[",
                     static_cast<unsigned long>(seq0), n);
    for (uint16_t i = 0; i < n && pos < outSize; i++) {
        pos += snprintf(out + pos, outSize - pos, "%s[%lu,%d,%d,%d,%d,%d,%d]",
                         i == 0 ? "" : ",",
                         static_cast<unsigned long>(s[i].tUs),
                         s[i].ax, s[i].ay, s[i].az, s[i].gx, s[i].gy, s[i].gz);
    }
    pos += snprintf(out + pos, outSize - pos, "]}");
    return pos;
}

static size_t buildHighGChunkJson(char* out, size_t outSize, uint32_t seq0,
                                   const HighGSample* s, uint16_t n) {
    size_t pos = 0;
    pos += snprintf(out + pos, outSize - pos, "{\"seq0\":%lu,\"n\":%u,\"d\":[",
                     static_cast<unsigned long>(seq0), n);
    for (uint16_t i = 0; i < n && pos < outSize; i++) {
        pos += snprintf(out + pos, outSize - pos, "%s[%lu,%d,%d,%d]",
                         i == 0 ? "" : ",",
                         static_cast<unsigned long>(s[i].tUs),
                         s[i].hgx, s[i].hgy, s[i].hgz);
    }
    pos += snprintf(out + pos, outSize - pos, "]}");
    return pos;
}

static void publishMetaIfDue() {
    const uint32_t now = millis();
    if (now - lastMetaMs < META_INTERVAL_MS) return;
    lastMetaMs = now;

    char buf[320];
    // BRING-UP PATCH NOTE - keep this metadata matched to imu.cpp ACCEL_SCALE.
    // The temporary 1/32768 g/count experiment was removed because it masked
    // the flat-board axis-shape problem. Low-g float conversion is back to the
    // datasheet ±4g sensitivity; raw chunks still publish unmodified counts.
    snprintf(buf, sizeof(buf),
        "{\"seq0_anchor\":%lu,\"millis_anchor\":%lu,\"chunk_size\":%u,"
        "\"chunks_per_nominal_second\":%u,\"odr_hz\":%lu,"
        "\"accel_scale\":0.000488,\"gyro_scale\":0.070,\"hg_accel_scale\":0.0104,"
        "\"accel_fs_g\":16,\"gyro_fs_dps\":2000,\"hg_fs_g\":256}",
        static_cast<unsigned long>(lowgRing.nextSeqToPublish),
        static_cast<unsigned long>(now),
        CHUNK_SIZE, CHUNKS_PER_NOMINAL_SECOND,
        static_cast<unsigned long>(ODR_HZ));
    mqttClient.publish(topicMeta, buf);
}

static bool reconnectIfNeeded() {
    if (mqttClient.connected()) return true;
    if (WiFi.status() != WL_CONNECTED) return false;

    const uint32_t now = millis();
    if (now - lastMqttAttemptMs < MQTT_RETRY_INTERVAL_MS) return false;
    lastMqttAttemptMs = now;

    dbgLog("[MQTT] Connecting to %s:%d...", MQTT_BROKER_HOST, MQTT_BROKER_PORT);
    const bool ok = (strlen(MQTT_USERNAME) > 0)
                        ? mqttClient.connect(OTA_HOSTNAME, MQTT_USERNAME, MQTT_PASSWORD)
                        : mqttClient.connect(OTA_HOSTNAME);
    if (ok) {
        dbgLog("[MQTT] Connected");
    } else {
        dbgLog("[MQTT] Connect failed, rc=%d", mqttClient.state());
    }
    return ok;
}

void mqttLoggerInit() {
    buildTopics();
    mqttConfigured = strlen(MQTT_BROKER_HOST) > 0 &&
                      strcmp(MQTT_BROKER_HOST, "your-mqtt-broker-host") != 0;
    if (!mqttConfigured) {
        dbgLog("[MQTT] Disabled — set MQTT_BROKER_HOST in src/secrets.h");
        return;
    }
    mqttClient.setServer(MQTT_BROKER_HOST, MQTT_BROKER_PORT);
    dbgLog("[MQTT] Logger configured for %s:%d, topics under %s/imu/*",
           MQTT_BROKER_HOST, MQTT_BROKER_PORT, OTA_HOSTNAME);
}

void mqttLoggerService() {
    if (!mqttConfigured) return;
    if (!reconnectIfNeeded()) return;

    mqttClient.loop();
    publishMetaIfDue();

    uint8_t published = 0;
    while (published < MAX_CHUNKS_PER_SERVICE_CALL && lowgRing.chunkReady(CHUNK_SIZE)) {
        uint32_t seq0;
        lowgRing.peekChunk(lowgChunkScratch, CHUNK_SIZE, seq0);
        const size_t len = buildLowGChunkJson(jsonScratch, sizeof(jsonScratch), seq0,
                                               lowgChunkScratch, CHUNK_SIZE);
        if (!mqttClient.publish(topicLowG, reinterpret_cast<const uint8_t*>(jsonScratch), len)) break;
        lowgRing.consumeChunk(CHUNK_SIZE);
        published++;
    }

    published = 0;
    while (published < MAX_CHUNKS_PER_SERVICE_CALL && highgRing.chunkReady(CHUNK_SIZE)) {
        uint32_t seq0;
        highgRing.peekChunk(highgChunkScratch, CHUNK_SIZE, seq0);
        const size_t len = buildHighGChunkJson(jsonScratch, sizeof(jsonScratch), seq0,
                                                highgChunkScratch, CHUNK_SIZE);
        if (!mqttClient.publish(topicHighG, reinterpret_cast<const uint8_t*>(jsonScratch), len)) break;
        highgRing.consumeChunk(CHUNK_SIZE);
        published++;
    }
}

void mqttLoggerPushLowG(const ImuData& d) {
    LowGSample s;
    s.tUs = d.tsUs;
    s.ax = d.rawAx; s.ay = d.rawAy; s.az = d.rawAz;
    s.gx = d.rawGx; s.gy = d.rawGy; s.gz = d.rawGz;
    lowgRing.push(s);
}

void mqttLoggerPushHighG(const ImuData& d) {
    HighGSample s;
    s.tUs = d.tsHgUs;
    s.hgx = d.rawHgx; s.hgy = d.rawHgy; s.hgz = d.rawHgz;
    highgRing.push(s);
}
