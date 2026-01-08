#include <iostream>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <random>
#include <vector>

/* FreeRTOS includes must be wrapped in extern "C" */
extern "C" {
    #include "FreeRTOS.h"
    #include "task.h"
    #include "queue.h"

    // Hook for stack overflow
    void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
    {
        std::cerr << "FATAL: Stack overflow in task: " << pcTaskName << std::endl;
        for( ;; );
    }

    // Hook for malloc failure (Important!)
    void vApplicationMallocFailedHook( void )
    {
        std::cerr << "FATAL: Malloc failed!" << std::endl;
        for( ;; );
    }
}

// ============================================================================
// RADIO MEDIUM PARAMETERS
// ============================================================================

// Transmission power (dBm) - define as per requirement
#define TX_POWER 20.0f

// Path loss exponent (n): 2.0 for free space, higher for indoor environments
#define PATH_LOSS_EXPONENT 2.5f

// Reference distance (d0 in meters)
#define REFERENCE_DISTANCE 1.0f

// Gaussian noise standard deviation (sigma) for realistic environment simulation
#define NOISE_SIGMA 1.0f

// ============================================================================
// DATA STRUCTURES
// ============================================================================

// Structure for packet sent by Target node to RadioMedium
typedef struct {
    uint32_t messageId;
    float targetX;
    float targetY;
    float txPower;
    uint32_t timestamp;
} Packet;

// Structure for RSSI report received by Sensor from RadioMedium
typedef struct {
    uint32_t messageId;
    float rssi;  // Received Signal Strength Indicator (dBm)
    uint32_t timestamp;
} RSSIReport;

// Structure for detection report (Sensor -> Master)
typedef struct {
    uint32_t sensorId;
    uint32_t messageId;
    float rssi;
    uint32_t timestamp;
} DetectionReport;

// Structure for echo/heartbeat message (Master -> Sensor)
typedef struct {
    uint32_t masterId;
    uint32_t timestamp;
} EchoMessage;

// Structure for RadioMedium node entry
typedef struct {
    QueueHandle_t sensorQueue;
    float posX;
    float posY;
    uint32_t sensorId;
} SensorNodeEntry;

// ============================================================================
// GLOBAL HANDLES AND QUEUES
// ============================================================================

// Queue: Sensor nodes -> Master node (detection reports)
QueueHandle_t xReportQueue = nullptr;

// Queue: Master node -> Sensor nodes (echo/heartbeat for validation)
QueueHandle_t xEchoQueue = nullptr;

// RadioMedium sensor registry
std::vector<SensorNodeEntry> g_sensors;

// Simulation parameters
const uint32_t NUM_SENSORS = 4;
const uint32_t BROADCAST_INTERVAL_MS = 1000;  // Target broadcasts every 2 seconds
const uint32_t ECHO_INTERVAL_MS = 300;       // Master sends echo every 0.5 second
const uint32_t DETECTION_TIMEOUT_MS = 5000;   // Remove sensor if no echo for 5 seconds

// Sensor fixed positions (anchor nodes)
const float SENSOR_POSITIONS[4][2] = {
    {0.0f, 0.0f},      // Sensor 0
    {30.0f, 0.0f},     // Sensor 1
    {15.0f, 26.0f},    // Sensor 2
    {15.0f, -26.0f}    // Sensor 3
};

// Random number generator for Gaussian noise
static std::mt19937 g_rng(std::random_device{}());
static std::normal_distribution<float> g_gauss(0.0f, NOISE_SIGMA);

// ============================================================================
// RADIO MEDIUM FUNCTIONS (C-style)
// ============================================================================

/**
 * Calculate Euclidean distance between two points
 */
float calculateDistance(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

/**
 * Calculate RSSI using Log-Distance Path Loss Model
 * Prx(d) = Ptx - 10*n*log10(d/d0) + Xσ
 * 
 * @param txPower Transmission power in dBm
 * @param distance Distance between transmitter and receiver in meters
 * @return Received power in dBm (before noise)
 */
float calculateRSSI(float txPower, float distance) {
    // Avoid log(0) - ensure minimum distance
    if (distance < REFERENCE_DISTANCE) {
        distance = REFERENCE_DISTANCE;
    }
    
    float pathLoss = 10.0f * PATH_LOSS_EXPONENT * std::log10(distance / REFERENCE_DISTANCE);
    return txPower - pathLoss;
}

/**
 * Calculate distance from RSSI using Log-Distance Path Loss Model
 * d = d0 * 10^((Ptx - Prx)/(10*n))
 * 
 * @param rssi Received power in dBm
 * @return Estimated distance in meters
 */
float rssiToDistance(float rssi) {
    return REFERENCE_DISTANCE *
           powf(10.0f, (TX_POWER - rssi) / (10.0f * PATH_LOSS_EXPONENT));
}


/**
 * Generate Gaussian noise using standard deviation NOISE_SIGMA
 * @return Random noise value with mean 0 and stddev NOISE_SIGMA
 */
float generateGaussianNoise() {
    return g_gauss(g_rng);
}

/**
 * Register a sensor node with the RadioMedium
 */
void radioMedium_registerSensor(uint32_t sensorId, QueueHandle_t sensorQueue, float posX, float posY) {
    SensorNodeEntry entry = {sensorQueue, posX, posY, sensorId};
    g_sensors.push_back(entry);
}

/**
 * Simulate RadioMedium: Broadcast packet to all registered sensors
 * Each sensor receives the packet with calculated RSSI + noise
 */
void radioMedium_broadcast(Packet packet) {
    for (size_t i = 0; i < g_sensors.size(); i++) {
        // Calculate distance from target to this sensor
        float dist = calculateDistance(
            packet.targetX, packet.targetY,
            g_sensors[i].posX, g_sensors[i].posY
        );
        
        // Calculate RSSI at this distance
        float rssi = calculateRSSI(packet.txPower, dist);
        
        // Add realistic Gaussian noise to simulate environment
        rssi += generateGaussianNoise();
        
        // Create RSSI report
        RSSIReport report = {
            packet.messageId,
            rssi,
            xTaskGetTickCount()
        };
        
        // Send to sensor's input queue (simulates "hardware interrupt")
        xQueueSend(g_sensors[i].sensorQueue, &report, 0);
    }
}

// ============================================================================
// TASK 1: TARGET NODE - Broadcasts periodic messages
// ============================================================================
void vTargetNodeTask(void *pvParameters) {
    uint32_t messageId = 0;
    float posX = 15.0f, posY = 0.0f;

    std::cout << "[TARGET] Initialized at position (" << posX << ", " << posY << ", " << ")" << std::endl;

    for (;;) {
        messageId++;
        
        Packet pkt = {
            messageId,
            posX,
            posY,
            TX_POWER,
            xTaskGetTickCount()
        };

        // TO UPDATE DISTANCE LATER WITH A MOVEMENT PATTERN
        // Simulate mobile target (slight movement)
        posX -= 0.1f;
        posY += 0.3f;

        // Broadcast via RadioMedium
        radioMedium_broadcast(pkt);

        std::cout << "[TARGET] Broadcast #" << messageId << " - Position: ("
                  << pkt.targetX << ", " << pkt.targetY << ")" << std::endl;

        vTaskDelay(pdMS_TO_TICKS(BROADCAST_INTERVAL_MS));
    }
}

// ============================================================================
// TASK 2: SENSOR NODE - Receives RSSI and reports detections
// ============================================================================
void vSensorNodeTask(void *pvParameters) {
    uint32_t sensorId = *((uint32_t*)pvParameters);
    QueueHandle_t sensorInputQueue = xQueueCreate(10, sizeof(RSSIReport));
    
    if (sensorInputQueue == nullptr) {
        std::cerr << "[SENSOR_" << sensorId << "] Failed to create input queue!" << std::endl;
        return;
    }

    // Register with RadioMedium
    radioMedium_registerSensor(sensorId, sensorInputQueue, 
                               SENSOR_POSITIONS[sensorId][0], 
                               SENSOR_POSITIONS[sensorId][1]);

    std::cout << "[SENSOR_" << sensorId << "] Started at position ("
              << SENSOR_POSITIONS[sensorId][0] << ", " << SENSOR_POSITIONS[sensorId][1] << ")" << std::endl;

    RSSIReport rssiReport;

    for (;;) {
        // Receive RSSI data from RadioMedium
        if (xQueueReceive(sensorInputQueue, &rssiReport, pdMS_TO_TICKS(500)) == pdPASS) {
            // Only report if signal strength is good enough
            if (rssiReport.rssi > -90.0f) {  // Threshold for minimum acceptable signal
                DetectionReport report = {
                    sensorId,
                    rssiReport.messageId,
                    rssiReport.rssi,
                    xTaskGetTickCount()
                };

                if (xQueueSend(xReportQueue, &report, portMAX_DELAY) == pdPASS) {
                    std::cout << "  [SENSOR_" << sensorId << "] Detected msg #" 
                              << rssiReport.messageId << " (RSSI: " << rssiReport.rssi << " dBm)" << std::endl;
                }
            } else {
                std::cout << "  [SENSOR_" << sensorId << "] Signal too weak for msg #" 
                          << rssiReport.messageId << " (RSSI: " << rssiReport.rssi << " dBm)" << std::endl;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


bool multilateration(
    const DetectionReport* detections,
    uint32_t count,
    float& outX,
    float& outY
) {
    if (count < 3) return false;

    uint32_t refId = detections[0].sensorId;
    float x0 = SENSOR_POSITIONS[refId][0];
    float y0 = SENSOR_POSITIONS[refId][1];
    float d0 = rssiToDistance(detections[0].rssi);

    float ATA[2][2] = {0};
    float ATb[2] = {0};

    for (uint32_t i = 1; i < count; i++) {
        uint32_t id = detections[i].sensorId;

        float xi = SENSOR_POSITIONS[id][0];
        float yi = SENSOR_POSITIONS[id][1];
        float di = rssiToDistance(detections[i].rssi);

        // Reject insane distances (RSSI glitches)
        if (di <= 0.1f || di > 200.0f)
            continue;

        float w = 1.0f / di;  // weight

        float Ai[2];
        Ai[0] = 2.0f * (xi - x0);
        Ai[1] = 2.0f * (yi - y0);

        float bi =
            (d0 * d0 - di * di) +
            (xi * xi - x0 * x0) +
            (yi * yi - y0 * y0);

        // Weighted AᵀA
        ATA[0][0] += w * Ai[0] * Ai[0];
        ATA[0][1] += w * Ai[0] * Ai[1];
        ATA[1][0] += w * Ai[1] * Ai[0];
        ATA[1][1] += w * Ai[1] * Ai[1];

        // Weighted Aᵀb
        ATb[0] += w * Ai[0] * bi;
        ATb[1] += w * Ai[1] * bi;
    }

    float det = ATA[0][0] * ATA[1][1] - ATA[0][1] * ATA[1][0];
    if (fabs(det) < 1e-6f)
        return false;

    float inv[2][2];
    inv[0][0] =  ATA[1][1] / det;
    inv[0][1] = -ATA[0][1] / det;
    inv[1][0] = -ATA[1][0] / det;
    inv[1][1] =  ATA[0][0] / det;

    outX = inv[0][0] * ATb[0] + inv[0][1] * ATb[1];
    outY = inv[1][0] * ATb[0] + inv[1][1] * ATb[1];

    return true;
}


// ============================================================================
// TASK 3: MASTER NODE - Coordinates sensors and determines target position
// ============================================================================
void vMasterNodeTask(void *pvParameters)
{
    std::cout << "[MASTER] Initialized with " << NUM_SENSORS << " sensors" << std::endl;

    static DetectionReport detections[NUM_SENSORS];
    uint32_t detectionCount = 0;

    uint32_t currentMessageId = 0;
    TickType_t messageStartTick = 0;

    const TickType_t MESSAGE_TIMEOUT = pdMS_TO_TICKS(300);

    DetectionReport report;

    for (;;)
    {
        /* ===============================
           PHASE 1: COLLECT DETECTIONS
           =============================== */

        while (xQueueReceive(xReportQueue, &report, 0) == pdPASS)
        {
            /* Start collecting a new message */
            if (currentMessageId == 0)
            {
                currentMessageId = report.messageId;
                messageStartTick = xTaskGetTickCount();
                detectionCount = 0;
            }

            /* Ignore detections from other messages */
            if (report.messageId != currentMessageId)
                continue;

            /* Avoid duplicate sensor entries */
            bool duplicate = false;
            for (uint32_t i = 0; i < detectionCount; i++)
            {
                if (detections[i].sensorId == report.sensorId)
                {
                    duplicate = true;
                    break;
                }
            }

            if (!duplicate && detectionCount < NUM_SENSORS)
            {
                detections[detectionCount++] = report;
            }
        }

        /* ===============================
           PHASE 2: CHECK TIMEOUT
           =============================== */

        if (currentMessageId != 0 &&
            (xTaskGetTickCount() - messageStartTick) > MESSAGE_TIMEOUT)
        {
            if (detectionCount >= 3)
            {
                float estX = 0.0f;
                float estY = 0.0f;

                std::cout << "[MASTER] Solving msg #" << currentMessageId
                          << " with " << detectionCount << " sensors" << std::endl;

                for (uint32_t i = 0; i < detectionCount; i++)
                {
                    std::cout << "  SENSOR_" << detections[i].sensorId
                              << " RSSI=" << detections[i].rssi << " dBm" << std::endl;
                }

                if (multilateration(detections, detectionCount, estX, estY))
                {
                    std::cout << "[MASTER] Target estimate: ("
                              << estX << ", " << estY << ")\n" << std::endl;
                }
                else
                {
                    std::cout << "[MASTER] Multilateration failed\n" << std::endl;
                }
            }
            else
            {
                std::cout << "[MASTER] Msg #" << currentMessageId
                          << " discarded (" << detectionCount
                          << " detections)\n" << std::endl;
            }

            /* Reset state for next message */
            currentMessageId = 0;
            detectionCount = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


// ============================================================================
// MAIN APPLICATION
// ============================================================================
int main(void) {
    // Disable stdout buffering
    setbuf(stdout, NULL);

    std::cout << "========================================" << std::endl;
    std::cout << "  Wireless Sensor Network Localization" << std::endl;
    std::cout << "  (WSN Target Localization System)      " << std::endl;
    std::cout << "========================================" << std::endl << std::endl;

    // Create queues
    xReportQueue = xQueueCreate(50, sizeof(DetectionReport));
    xEchoQueue = xQueueCreate(10, sizeof(EchoMessage));

    if (xReportQueue == nullptr || xEchoQueue == nullptr) {
        std::cerr << "Failed to create queues." << std::endl;
        return 1;
    }
    std::cout << "[MAIN] Created communication queues" << std::endl;

    const uint32_t STACK_SIZE = 2048; // 2048 is in words, not bytes - tried experimentally
    BaseType_t status;

    // Create Target Node task
    status = xTaskCreate(vTargetNodeTask, "TargetNode", STACK_SIZE, nullptr, 2, nullptr);
    if (status != pdPASS) {
        std::cerr << "Failed to create Target Node task!" << std::endl;
        return 1;
    }
    std::cout << "[MAIN] Created Target Node task" << std::endl;

    // Create Sensor Node tasks
    for (uint32_t i = 0; i < NUM_SENSORS; i++) {
        uint32_t *param = (uint32_t*)malloc(sizeof(uint32_t));
        *param = i;
        status = xTaskCreate(vSensorNodeTask, "SensorNode", STACK_SIZE, (void*)param, 1, nullptr);
        if (status != pdPASS) {
            std::cerr << "Failed to create Sensor Node " << i << " task!" << std::endl;
            return 1;
        }
    }
    std::cout << "[MAIN] Created " << NUM_SENSORS << " Sensor Node tasks" << std::endl;

    // Create Master Node task
    status = xTaskCreate(vMasterNodeTask, "MasterNode", STACK_SIZE, nullptr, 2, nullptr);
    if (status != pdPASS) {
        std::cerr << "Failed to create Master Node task!" << std::endl;
        return 1;
    }
    std::cout << "[MAIN] Created Master Node task" << std::endl;

    std::cout << "\n[MAIN] Starting FreeRTOS Scheduler..." << std::endl << std::endl;

    // Start the scheduler
    vTaskStartScheduler();

    // If we get here, something went wrong
    std::cout << "ERROR: Scheduler failed to start." << std::endl;
    return 0;
}
