#ifndef SENSOR_H
#define SENSOR_H

#include "config.h"

class UltrasonicSensor {
private:
    int trigPin;
    int echoPin;
    unsigned long lastRead;
    int lastDistance;
    bool initialized;
    
    // Enhanced filtering and calibration
    struct FilteredData {
        int values[10];        // Increased buffer size
        int weights[10];       // Weight for each value
        int index;
        bool filled;
        float movingAverage;
        float standardDeviation;
    } filter;
    
    // Sensor calibration data
    struct CalibrationData {
        float temperatureCoeff;
        float humidityCoeff;
        float offsetCorrection;
        float gainCorrection;
        unsigned long lastCalibration;
    } calibration;
    
    // Error tracking
    struct ErrorStats {
        unsigned long totalReadings;
        unsigned long errorReadings;
        unsigned long timeouts;
        unsigned long invalidReadings;
        float errorRate;
        unsigned long lastErrorTime;
    } errors;
    
    // Environmental factors
    struct Environment {
        float temperature;
        float humidity;
        float pressure;
        float speedOfSound;
    } environment;

public:
    UltrasonicSensor() 
        : lastRead(0), lastDistance(0), initialized(false) {
        resetFilter();
        initializeCalibration();
        resetErrors();
        updateEnvironment();
    }

    void init(int trig, int echo) {
        trigPin = trig;
        echoPin = echo;
        pinMode(trigPin, OUTPUT);
        pinMode(echoPin, INPUT);
        initialized = true;
        
        // Perform initial calibration
        calibrateSensor();
    }

    int getDistance() {
        if (!initialized) return -1;
        
        if (millis() - lastRead < 50) { // Limit reading frequency
            return lastDistance;
        }
        
        // Take multiple readings for accuracy
        int readings[3];
        for (int i = 0; i < 3; i++) {
            readings[i] = takeSingleReading();
            delayMicroseconds(100);
        }
        
        // Process readings
        int distance = processReadings(readings, 3);
        
        // Apply environmental corrections
        distance = applyEnvironmentalCorrections(distance);
        
        // Validate and filter reading
        if (isValidReading(distance)) {
            lastDistance = applyFilter(distance);
            lastRead = millis();
            updateStats(true);
        } else {
            updateStats(false);
        }
        
        return lastDistance;
    }
    
    bool selfTest() {
        if (!initialized) return false;
        
        bool success = true;
        
        // Test sensor connectivity
        success &= testConnectivity();
        
        // Test reading stability
        success &= testStability();
        
        // Test response time
        success &= testResponseTime();
        
        return success;
    }
    
    // Enhanced status reporting
    struct SensorStatus {
        float errorRate;
        float signalQuality;
        float temperature;
        unsigned long totalReadings;
        bool isCalibrated;
    };
    
    SensorStatus getStatus() {
        SensorStatus status;
        status.errorRate = errors.errorRate;
        status.signalQuality = calculateSignalQuality();
        status.temperature = environment.temperature;
        status.totalReadings = errors.totalReadings;
        status.isCalibrated = isCalibrated();
        return status;
    }
    
    void calibrateSensor() {
        // Perform comprehensive calibration
        calibrateOffset();
        calibrateGain();
        updateEnvironmentalFactors();
        calibration.lastCalibration = millis();
    }

private:
    int takeSingleReading() {
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);
        
        long duration = pulseIn(echoPin, HIGH, SENSOR_TIMEOUT);
        
        if (duration == 0) {
            errors.timeouts++;
            return -1;
        }
        
        // Calculate distance with temperature compensation
        float speedOfSound = calculateSpeedOfSound();
        return duration * speedOfSound / 20000.0; // Convert to cm
    }
    
    int processReadings(int* readings, int count) {
        // Sort readings
        for (int i = 0; i < count - 1; i++) {
            for (int j = 0; j < count - i - 1; j++) {
                if (readings[j] > readings[j + 1]) {
                    int temp = readings[j];
                    readings[j] = readings[j + 1];
                    readings[j + 1] = temp;
                }
            }
        }
        
        // Return median
        return readings[count / 2];
    }
    
    float calculateSpeedOfSound() {
        // Calculate speed of sound based on temperature and humidity
        float speedOfSound = 331.3 + (0.606 * environment.temperature);
        speedOfSound += (0.0124 * environment.humidity);
        return speedOfSound;
    }
    
    bool isValidReading(int distance) {
        // Enhanced validation
        if (distance < 0 || distance > 400) {
            errors.invalidReadings++;
            return false;
        }
        
        // Check for sudden large changes
        if (lastDistance > 0) {
            float maxChange = lastDistance * 0.3; // Max 30% change
            if (abs(distance - lastDistance) > maxChange) {
                errors.invalidReadings++;
                return false;
            }
        }
        
        // Check if reading is within statistical bounds
        if (filter.filled) {
            float deviation = abs(distance - filter.movingAverage);
            if (deviation > 3 * filter.standardDeviation) {
                errors.invalidReadings++;
                return false;
            }
        }
        
        return true;
    }
    
    int applyFilter(int newValue) {
        // Update circular buffer with weighted values
        filter.values[filter.index] = newValue;
        
        // Update moving average
        float sum = 0;
        float weightSum = 0;
        
        for (int i = 0; i < 10; i++) {
            sum += filter.values[i] * filter.weights[i];
            weightSum += filter.weights[i];
        }
        
        filter.movingAverage = sum / weightSum;
        
        // Update standard deviation
        float variance = 0;
        for (int i = 0; i < 10; i++) {
            float diff = filter.values[i] - filter.movingAverage;
            variance += (diff * diff) * filter.weights[i];
        }
        filter.standardDeviation = sqrt(variance / weightSum);
        
        // Update index
        filter.index = (filter.index + 1) % 10;
        if (filter.index == 0) {
            filter.filled = true;
        }
        
        return round(filter.movingAverage);
    }
    
    void resetFilter() {
        filter.index = 0;
        filter.filled = false;
        filter.movingAverage = 0;
        filter.standardDeviation = 0;
        
        // Initialize weights for weighted average
        for (int i = 0; i < 10; i++) {
            filter.values[i] = 0;
            filter.weights[i] = 10 - i; // More recent values have higher weight
        }
    }
    
    void initializeCalibration() {
        calibration.temperatureCoeff = 1.0;
        calibration.humidityCoeff = 1.0;
        calibration.offsetCorrection = 0.0;
        calibration.gainCorrection = 1.0;
        calibration.lastCalibration = 0;
    }
    
    void resetErrors() {
        errors.totalReadings = 0;
        errors.errorReadings = 0;
        errors.timeouts = 0;
        errors.invalidReadings = 0;
        errors.errorRate = 0;
        errors.lastErrorTime = 0;
    }
    
    void updateEnvironment() {
        // Placeholder for actual environmental sensor readings
        environment.temperature = 25.0;  // Celsius
        environment.humidity = 50.0;     // %
        environment.pressure = 1013.25;  // hPa
        environment.speedOfSound = calculateSpeedOfSound();
    }
    
    void updateStats(bool validReading) {
        errors.totalReadings++;
        if (!validReading) {
            errors.errorReadings++;
            errors.lastErrorTime = millis();
        }
        errors.errorRate = (float)errors.errorReadings / errors.totalReadings;
    }
    
    float calculateSignalQuality() {
        float quality = 100.0;
        
        // Reduce quality based on error rate
        quality -= (errors.errorRate * 50);
        
        // Reduce quality based on standard deviation
        if (filter.filled) {
            quality -= (filter.standardDeviation / 10);
        }
        
        // Reduce quality based on calibration age
        unsigned long calibrationAge = (millis() - calibration.lastCalibration) / 3600000; // hours
        if (calibrationAge > 24) {
            quality -= (calibrationAge - 24) * 0.5;
        }
        
        return constrain(quality, 0, 100);
    }
    
    bool isCalibrated() {
        return (millis() - calibration.lastCalibration) < 86400000; // 24 hours
    }
    
    void calibrateOffset() {
        // Measure known distance for offset calibration
        int sumReadings = 0;
        for (int i = 0; i < 10; i++) {
            sumReadings += takeSingleReading();
            delay(50);
        }
        float avgReading = sumReadings / 10.0;
        calibration.offsetCorrection = 100 - avgReading; // Assuming 100cm reference
    }
    
    void calibrateGain() {
        // Measure multiple known distances for gain calibration
        const int referenceDistances[] = {50, 100, 200};
        float sumGain = 0;
        int validMeasurements = 0;
        
        for (int dist : referenceDistances) {
            int reading = takeSingleReading();
            if (reading > 0) {
                sumGain += dist / (float)reading;
                validMeasurements++;
            }
        }
        
        if (validMeasurements > 0) {
            calibration.gainCorrection = sumGain / validMeasurements;
        }
    }
    
    void updateEnvironmentalFactors() {
        updateEnvironment();
        environment.speedOfSound = calculateSpeedOfSound();
    }
    
    int applyEnvironmentalCorrections(int distance) {
        float corrected = distance;
        corrected *= calibration.gainCorrection;
        corrected += calibration.offsetCorrection;
        corrected *= calibration.temperatureCoeff;
        corrected *= calibration.humidityCoeff;
        return round(corrected);
    }
    
    bool testConnectivity() {
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);
        return digitalRead(echoPin) == LOW;
    }
    
    bool testStability() {
        int readings[5];
        for (int i = 0; i < 5; i++) {
            readings[i] = takeSingleReading();
            delay(50);
        }
        
        // Calculate standard deviation
        float sum = 0;
        for (int i = 0; i < 5; i++) {
            sum += readings[i];
        }
        float mean = sum / 5;
        
        float variance = 0;
        for (int i = 0; i < 5; i++) {
            variance += pow(readings[i] - mean, 2);
        }
        variance /= 5;
        
        return sqrt(variance) < 5; // Less than 5cm deviation
    }
    
    bool testResponseTime() {
        unsigned long start = micros();
        takeSingleReading();
        unsigned long duration = micros() - start;
        
        return duration < 50000; // Less than 50ms response time
    }
};

#endif