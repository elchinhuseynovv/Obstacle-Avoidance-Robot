#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include "config.h"
#include <SD.h>

class DataLogger {
private:
    bool sdAvailable;
    File logFile;
    unsigned long lastLog;
    const char* logFileName = "robot_log.txt";
    
    struct LogStats {
        unsigned long totalEntries;
        unsigned long errorEntries;
        unsigned long lastLogTime;
        size_t logFileSize;
    } stats;

public:
    DataLogger() : sdAvailable(false), lastLog(0) {
        resetStats();
    }

    bool init() {
        sdAvailable = SD.begin(SD_CS_PIN);
        if (sdAvailable) {
            // Create a new log file with timestamp
            char timestamp[15];
            createTimestamp(timestamp);
            String fullFileName = String(timestamp) + "_" + String(logFileName);
            logFile = SD.open(fullFileName.c_str(), FILE_WRITE);
            
            if (logFile) {
                logSystemInfo();
                return true;
            }
        }
        return false;
    }

    void logEvent(const char* event, const char* level = "INFO") {
        if (!sdAvailable) return;

        char timestamp[15];
        createTimestamp(timestamp);
        
        String logEntry = String(timestamp) + " [" + level + "] " + event + "\n";
        
        if (logFile) {
            logFile.print(logEntry);
            logFile.flush();
            
            updateStats(level);
            
            if (DEBUG_MODE) {
                Serial.print("Log: ");
                Serial.println(logEntry);
            }
        }
    }

    void logSensorData(const SensorManager::SensorData& data) {
        if (!sdAvailable || millis() - lastLog < LOG_INTERVAL) return;

        char buffer[100];
        snprintf(buffer, sizeof(buffer), 
                "Front: %d cm, Left: %d cm, Right: %d cm",
                data.frontDistance, data.leftDistance, data.rightDistance);
        
        logEvent(buffer, "SENSOR");
        lastLog = millis();
    }

    void logError(const char* error) {
        logEvent(error, "ERROR");
        stats.errorEntries++;
    }

    void logSystemMetrics(const SystemMetrics& metrics) {
        if (!sdAvailable) return;

        char buffer[200];
        snprintf(buffer, sizeof(buffer),
                "Metrics - Speed: %.2f, Distance: %.2f, Obstacles: %d, Errors: %d",
                metrics.averageSpeed, metrics.totalDistance,
                metrics.obstaclesAvoided, metrics.errorCount);
        
        logEvent(buffer, "METRICS");
    }

    LogStats getStats() const {
        return stats;
    }

    void close() {
        if (logFile) {
            logFile.close();
        }
    }

private:
    void createTimestamp(char* buffer) {
        unsigned long ms = millis();
        unsigned long seconds = ms / 1000;
        unsigned long minutes = seconds / 60;
        unsigned long hours = minutes / 60;
        
        sprintf(buffer, "%02lu:%02lu:%02lu.%03lu",
                hours, minutes % 60, seconds % 60, ms % 1000);
    }

    void logSystemInfo() {
        logEvent("=== System Started ===");
        logEvent("Robot Configuration:");
        
        char buffer[100];
        snprintf(buffer, sizeof(buffer), "Debug Mode: %s", DEBUG_MODE ? "ON" : "OFF");
        logEvent(buffer);
        
        snprintf(buffer, sizeof(buffer), "Battery Threshold: %.2fV", LOW_BATTERY_THRESHOLD);
        logEvent(buffer);
        
        snprintf(buffer, sizeof(buffer), "Safe Distance: %dcm", SAFE_DISTANCE);
        logEvent(buffer);
    }

    void updateStats(const char* level) {
        stats.totalEntries++;
        stats.lastLogTime = millis();
        stats.logFileSize = logFile.size();
        
        if (strcmp(level, "ERROR") == 0) {
            stats.errorEntries++;
        }
    }

    void resetStats() {
        stats.totalEntries = 0;
        stats.errorEntries = 0;
        stats.lastLogTime = 0;
        stats.logFileSize = 0;
    }
};

#endif