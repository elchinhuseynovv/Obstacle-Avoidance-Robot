#ifndef WIRELESS_CONTROLLER_H
#define WIRELESS_CONTROLLER_H

#include "config.h"
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>

class WirelessController {
private:
    ESP8266WebServer server;
    bool isConnected;
    String ssid;
    String password;
    
    struct WirelessStats {
        unsigned long packetsReceived;
        unsigned long packetsSent;
        int signalStrength;
        unsigned long lastCommunication;
        unsigned long connectionDrops;
    } stats;

public:
    WirelessController() : server(80), isConnected(false) {
        resetStats();
    }

    bool init(const char* ssid, const char* password) {
        this->ssid = ssid;
        this->password = password;

        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid, password);

        // Wait for connection
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 20) {
            delay(500);
            attempts++;
        }

        if (WiFi.status() == WL_CONNECTED) {
            setupServer();
            isConnected = true;
            return true;
        }

        return false;
    }

    void update() {
        if (!isConnected) return;

        // Check connection status
        if (WiFi.status() != WL_CONNECTED) {
            handleDisconnection();
            return;
        }

        server.handleClient();
        updateStats();
    }

    void sendTelemetry(const SystemMetrics& metrics) {
        if (!isConnected) return;

        StaticJsonDocument<200> doc;
        doc["speed"] = metrics.averageSpeed;
        doc["distance"] = metrics.totalDistance;
        doc["obstacles"] = metrics.obstaclesAvoided;
        doc["errors"] = metrics.errorCount;
        doc["battery"] = metrics.batteryVoltage;

        String telemetry;
        serializeJson(doc, telemetry);

        // Implement sending telemetry data to remote server
        // This is a placeholder for actual implementation
        stats.packetsSent++;
    }

    bool isWirelessConnected() const {
        return isConnected;
    }

    WirelessStats getStats() const {
        return stats;
    }

private:
    void setupServer() {
        server.on("/status", HTTP_GET, [this]() {
            handleStatusRequest();
        });

        server.on("/control", HTTP_POST, [this]() {
            handleControlRequest();
        });

        server.begin();
    }

    void handleStatusRequest() {
        StaticJsonDocument<400> doc;
        
        doc["connected"] = isConnected;
        doc["signal"] = WiFi.RSSI();
        doc["uptime"] = millis() / 1000;
        doc["packets_rx"] = stats.packetsReceived;
        doc["packets_tx"] = stats.packetsSent;

        String response;
        serializeJson(doc, response);
        
        server.send(200, "application/json", response);
        stats.packetsReceived++;
    }

    void handleControlRequest() {
        if (server.hasArg("plain")) {
            String body = server.arg("plain");
            StaticJsonDocument<200> doc;
            DeserializationError error = deserializeJson(doc, body);

            if (!error) {
                // Process control commands
                // This is a placeholder for actual command processing
                server.send(200, "application/json", "{\"status\":\"ok\"}");
                stats.packetsReceived++;
            } else {
                server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
            }
        } else {
            server.send(400, "application/json", "{\"error\":\"No data\"}");
        }
    }

    void handleDisconnection() {
        isConnected = false;
        stats.connectionDrops++;

        // Attempt to reconnect
        WiFi.begin(ssid.c_str(), password.c_str());
    }

    void updateStats() {
        stats.signalStrength = WiFi.RSSI();
        stats.lastCommunication = millis();
    }

    void resetStats() {
        stats.packetsReceived = 0;
        stats.packetsSent = 0;
        stats.signalStrength = 0;
        stats.lastCommunication = 0;
        stats.connectionDrops = 0;
    }
};

#endif