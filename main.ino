// Obstacle Avoidance Robot with Dynamic Turning Strategy

// Define motor driver pins
#define ENA 9
#define IN1 8
#define IN2 7
#define IN3 6
#define IN4 5
#define ENB 10

// Define ultrasonic sensor pins
#define TRIG 3
#define ECHO 2

// Function to move forward
void moveForward() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 180);
    analogWrite(ENB, 180);
}

// Function to move backward
void moveBackward() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, 150);
    analogWrite(ENB, 150);
    delay(500);
}

// Function to stop
void stopRobot() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

// Function to turn right
void turnRight() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, 180);
    analogWrite(ENB, 180);
    delay(600);
}

// Function to turn left
void turnLeft() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 180);
    analogWrite(ENB, 180);
    delay(600);
}

// Function to measure distance
int getDistance() {
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);
    return pulseIn(ECHO, HIGH) * 0.034 / 2;
}

void setup() {
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);
    Serial.begin(9600);
}

void loop() {
    int distance = getDistance();
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    
    if (distance > 25) {
        moveForward();
    } else if (distance > 10 && distance <= 25) {
        stopRobot();
        delay(300);
        moveBackward();
        delay(300);
        turnLeft();
    } else {
        stopRobot();
        delay(300);
        moveBackward();
        delay(500);
        turnRight();
    }
}
