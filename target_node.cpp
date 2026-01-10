#include <iostream>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <random>
#include <vector>
#include <fstream>
#include <string>
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


struct Point {
    float x;
    float y;
};

struct Kalman2D
{
    float x;
    float y;
    float Pxx;
    float Pyy;
};



// ============================================================================
// GLOBAL HANDLES AND QUEUES
// ============================================================================

/* Loaded from file in main() */
std::vector<Point> g_sensorPositions;
std::vector<Point> g_beaconTrajectory;
bool loadPointsFromFile(const std::string&, std::vector<Point>&);
bool savePointsToFile(const std::string&, const std::vector<Point>&);


/* Written by Master */
std::vector<Point> g_estimatedTrajectory;

/* Control flags */
volatile bool g_trajectoryFinished = false;

// Queue: Sensor nodes -> Master node (detection reports)
QueueHandle_t xReportQueue = nullptr;

// Queue: Master node -> Sensor nodes (echo/heartbeat for validation)
QueueHandle_t xEchoQueue = nullptr;

// RadioMedium sensor registry
std::vector<SensorNodeEntry> g_sensors;

// Simulation parameters
const uint32_t BROADCAST_INTERVAL_MS = 1000;  // Target broadcasts every 2 seconds
const uint32_t ECHO_INTERVAL_MS = 300;       // Master sends echo every 0.5 second
const uint32_t DETECTION_TIMEOUT_MS = 5000;   // Remove sensor if no echo for 5 seconds


// Random number generator for Gaussian noise
static std::mt19937 g_rng(std::random_device{}());
static std::normal_distribution<float> g_gauss(0.0f, NOISE_SIGMA);

long NUM_SENSORS = 0;


// ============================================================================
// RADIO MEDIUM FUNCTIONS 
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
void vTargetNodeTask(void *pvParameters)
{
    uint32_t messageId = 0;
    size_t index = 0;

    std::cout << "[TARGET] Loaded trajectory with "
              << g_beaconTrajectory.size() << " points\n";

    for (;;)
    {
        if (index >= g_beaconTrajectory.size())
        {
            g_trajectoryFinished = true;
            std::cout << "[TARGET] Trajectory finished\n";
            vTaskDelete(NULL);
        }

        Packet pkt = {
            ++messageId,
            g_beaconTrajectory[index].x,
            g_beaconTrajectory[index].y,
            TX_POWER,
            xTaskGetTickCount()
        };

        radioMedium_broadcast(pkt);

        std::cout << "[TARGET] #" << messageId
                  << " (" << pkt.targetX
                  << "," << pkt.targetY << ")\n";

        index++;
        vTaskDelay(pdMS_TO_TICKS(BROADCAST_INTERVAL_MS));
    }
}


// ============================================================================
// TASK 2: SENSOR NODE - Receives RSSI and reports detections
// ============================================================================
void vSensorNodeTask(void *pvParameters)
{
    uint32_t sensorId = *((uint32_t*)pvParameters);
    QueueHandle_t sensorInputQueue = xQueueCreate(10, sizeof(RSSIReport));

    if (!sensorInputQueue)
    {
        std::cerr << "[SENSOR_" << sensorId << "] Queue create failed\n";
        vTaskDelete(NULL);
    }

    radioMedium_registerSensor(
        sensorId,
        sensorInputQueue,
        g_sensorPositions[sensorId].x,
        g_sensorPositions[sensorId].y
    );

    std::cout << "[SENSOR_" << sensorId << "] Position ("
              << g_sensorPositions[sensorId].x << ", "
              << g_sensorPositions[sensorId].y << ")\n";

    RSSIReport rssi;

    for (;;)
    {
        if (xQueueReceive(sensorInputQueue, &rssi, pdMS_TO_TICKS(500)) == pdPASS)
        {
            if (rssi.rssi > -90.0f)
            {
                DetectionReport report {
                    sensorId,
                    rssi.messageId,
                    rssi.rssi,
                    xTaskGetTickCount()
                };

                xQueueSend(xReportQueue, &report, portMAX_DELAY);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ============================================================================
// TASK 3: MASTER NODE - Coordinates sensors and determines target position
// ============================================================================

void kalmanUpdate(Kalman2D& kf, float measX, float measY)
{
    const float R = 4.0f;   // measurement noise
    const float Q = 0.1f;  // process noise

    kf.Pxx += Q;
    kf.Pyy += Q;

    float Kx = kf.Pxx / (kf.Pxx + R);
    float Ky = kf.Pyy / (kf.Pyy + R);

    kf.x += Kx * (measX - kf.x);
    kf.y += Ky * (measY - kf.y);

    kf.Pxx *= (1.0f - Kx);
    kf.Pyy *= (1.0f - Ky);
}


bool multilateration(
    const DetectionReport* detections,
    uint32_t count,
    float& outX,
    float& outY)
{
    if (count < 3) return false;

    uint32_t refId = detections[0].sensorId;

    float x0 = g_sensorPositions[refId].x;
    float y0 = g_sensorPositions[refId].y;
    float d0 = rssiToDistance(detections[0].rssi);

    float ATA[2][2] = {{0,0},{0,0}};
    float ATb[2] = {0,0};

    for (uint32_t i = 1; i < count; i++)
    {
        uint32_t id = detections[i].sensorId;

        float xi = g_sensorPositions[id].x;
        float yi = g_sensorPositions[id].y;
        float di = rssiToDistance(detections[i].rssi);

        if (di < 0.1f || di > 200.0f)
            continue;

        float Ai[2] = {
            2.0f * (xi - x0),
            2.0f * (yi - y0)
        };

        float bi =
            (d0*d0 - di*di) +
            (xi*xi - x0*x0) +
            (yi*yi - y0*y0);

        float w = 1.0f / di;

        ATA[0][0] += w * Ai[0] * Ai[0];
        ATA[0][1] += w * Ai[0] * Ai[1];
        ATA[1][0] += w * Ai[1] * Ai[0];
        ATA[1][1] += w * Ai[1] * Ai[1];

        ATb[0] += w * Ai[0] * bi;
        ATb[1] += w * Ai[1] * bi;
    }

    float det = ATA[0][0]*ATA[1][1] - ATA[0][1]*ATA[1][0];
    if (fabs(det) < 1e-6f)
        return false;

    outX = ( ATA[1][1]*ATb[0] - ATA[0][1]*ATb[1] ) / det;
    outY = ( -ATA[1][0]*ATb[0] + ATA[0][0]*ATb[1] ) / det;

    return true;
}



void vMasterNodeTask(void *pvParameters)
{
    static std::vector<DetectionReport> detections;
    if (detections.empty())
        detections.resize(NUM_SENSORS);

    uint32_t detectionCount = 0;
    uint32_t currentMessageId = 0;
    TickType_t startTick = 0;

    DetectionReport report;

    for (;;)
    {
        if (g_trajectoryFinished && uxQueueMessagesWaiting(xReportQueue) == 0)
        {
            std::cout << "[MASTER] Saving results and stopping\n";
            savePointsToFile("estimated1.txt", g_estimatedTrajectory);
            vTaskEndScheduler();
        }

        while (xQueueReceive(xReportQueue, &report, 0) == pdPASS)
        {
            if (currentMessageId == 0)
            {
                currentMessageId = report.messageId;
                startTick = xTaskGetTickCount();
                detectionCount = 0;
            }

            if (report.messageId != currentMessageId)
                continue;

            bool dup = false;
            for (uint32_t i = 0; i < detectionCount; i++)
                if (detections[i].sensorId == report.sensorId)
                    dup = true;

            if (!dup && detectionCount < detections.size())
                detections[detectionCount++] = report;
        }

        if (currentMessageId &&
            (xTaskGetTickCount() - startTick) > pdMS_TO_TICKS(300))
        {
            if (detectionCount >= 3)
            {
                float x, y;
                if (multilateration(detections.data(), detectionCount, x, y))
                {
                    //static Kalman2D kf;
                    //kalmanUpdate(kf, x, y);
                    Point p;
                    p.x = x;
                    p.y = y;
                    g_estimatedTrajectory.push_back(p);


                    std::cout << "[MASTER] (" << x << "," << y << ")\n";
                }
            }

            currentMessageId = 0;
            detectionCount = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ============================================================================
// MISCELLANEOUS FILE I/O FUNCTIONS
// ============================================================================

bool loadPointsFromFile(const std::string& path, std::vector<Point>& out)
{
    std::ifstream file(path);
    if (!file.is_open())
        return false;

    out.clear();

    std::string line;
    while (std::getline(file, line))
    {
        float x, y;
        if (sscanf(line.c_str(), "%f,%f", &x, &y) == 2)
        {
            out.push_back({x, y});
        }
    }
    return !out.empty();
}

bool savePointsToFile(const std::string& path,
                      const std::vector<Point>& data)
{
    std::ofstream file(path);
    if (!file.is_open())
        return false;

    for (const auto& p : data)
        file << p.x << "," << p.y << "\n";

    return true;
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

    if (!loadPointsFromFile("input/sensor_positions/random_30p_60x60.txt", g_sensorPositions))
    {
        std::cerr << "Failed to load sensor positions\n";
        return 1;
    }

    if (!loadPointsFromFile("input/trajectories/ellipse_60x60_50p.txt", g_beaconTrajectory))
    {
        std::cerr << "Failed to load beacon trajectory\n";
        return 1;
    }
        std::cout << "[MAIN] Loaded "
              << g_sensorPositions.size() << " sensors, "
             << g_beaconTrajectory.size() << " trajectory points\n";
    NUM_SENSORS = g_sensorPositions.size();

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
