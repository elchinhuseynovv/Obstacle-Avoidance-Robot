#ifndef ERROR_HANDLER_H
#define ERROR_HANDLER_H

#include "config.h"
#include "display.h"

class ErrorHandler {
private:
    Display& display;
    
    struct ErrorLog {
        ErrorCode code;
        unsigned long timestamp;
        String description;
    };
    
    static const int LOG_SIZE = 10;
    ErrorLog errorLog[LOG_SIZE];
    int logIndex;
    
public:
    ErrorHandler(Display& d) : display(d), logIndex(0) {}
    
    void handleError(ErrorCode code, const char* description) {
        // Log the error
        errorLog[logIndex] = {
            code,
            millis(),
            String(description)
        };
        logIndex = (logIndex + 1) % LOG_SIZE;
        
        // Display error
        display.showError(description);
        
        // Log to Serial if debug mode is enabled
        if (DEBUG_MODE) {
            Serial.println("ERROR: " + String(description));
            Serial.println("Code: " + String(code));
        }
        
        // Take appropriate action based on error type
        handleErrorAction(code);
    }
    
    void clearErrors() {
        logIndex = 0;
    }
    
    String getErrorReport() {
        String report = "Recent Errors:\n";
        for (int i = 0; i < LOG_SIZE; i++) {
            if (errorLog[i].timestamp > 0) {
                report += String(errorLog[i].timestamp / 1000) + "s: ";
                report += errorLog[i].description + "\n";
            }
        }
        return report;
    }

private:
    void handleErrorAction(ErrorCode code) {
        switch (code) {
            case BATTERY_LOW:
                // Initiate safe shutdown
                break;
            case MOTOR_STALL:
                // Attempt motor recovery
                break;
            case IMU_ERROR:
                // Reset IMU
                break;
            case STUCK:
                // Initiate recovery procedure
                break;
            case OVERHEATING:
                // Enable cooling mode
                break;
            default:
                // Default error handling
                break;
        }
    }
};

#endif