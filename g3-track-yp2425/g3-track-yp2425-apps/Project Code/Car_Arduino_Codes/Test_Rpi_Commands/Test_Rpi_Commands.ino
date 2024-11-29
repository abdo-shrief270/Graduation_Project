#include <SoftwareSerial.h>

// Pin assignments for motor driver (update based on your setup)
#define ENA 3 // PWM pin for motor A
#define ENB 6 // PWM pin for motor B
#define IN1 4  // Direction pin for motor A
#define IN2 2  // Direction pin for motor A
#define IN3 7  // Direction pin for motor B
#define IN4 5  // Direction pin for motor B

// Pins for SoftwareSerial
#define RX 9  // Receive pin
#define TX 8  // Transmit pin

SoftwareSerial Rpi(RX, TX);

// Function to set motor speeds
void setMotorSpeeds(int speedLeft, int speedRight) {
  // Ensure speed values are within 0-100
  speedLeft = constrain(speedLeft, -100, 100);
  speedRight = constrain(speedRight, -100, 100);

  // Map speed values to PWM range (0–255)
  int pwmLeft = map(abs(speedLeft), 0, 100, 0, 255);
  int pwmRight = map(abs(speedRight), 0, 100, 0, 255);

  // Set left motor direction and speed
  if (speedLeft >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, pwmLeft);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, pwmLeft);
  }

  // Set right motor direction and speed
  if (speedRight >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, pwmRight);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, pwmRight);
  }
}

// Function to handle commands
void handleCommand(String command) {
  // Command format: "dir,speed"
  // dir: forward, backward, left, right, stop
  // speed: 0-100 (or angle in the case of turns)

  command.trim(); // Remove whitespace or newline
  int commaIndex = command.indexOf(',');
  if (commaIndex == -1) return; // Invalid command format

  String dir = command.substring(0, commaIndex);
  int speed = command.substring(commaIndex + 1).toInt();

  if (dir == "forward") {
    setMotorSpeeds(speed, speed);
  } else if (dir == "backward") {
    setMotorSpeeds(-speed, -speed);
  } else if (dir == "right") {
    setMotorSpeeds(speed * 0.6, speed); // Adjust for turning
  } else if (dir == "left") {
    setMotorSpeeds(speed, speed * 0.6); // Adjust for turning
  } else if (dir == "leftTurn") {
    setMotorSpeeds(speed, 0); // Left turn with specified speed
  } else if (dir == "stop") {
    setMotorSpeeds(0, 0); // Stop the car
  }
}

void setup() {
  // Initialize motor driver pins
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Initialize SoftwareSerial
  Rpi.begin(9600);
  Serial.begin(115200);

  Serial.println("Arduino ready for commands.");
}

void loop() {
  // Check if data is available from the RPI
  if (Rpi.available()) {
    // Serial.println("--------------------");
    String command = Rpi.readStringUntil('\n'); // Read command from RPI
    Rpi.println("Echo: " + command);  // Send back the received command
    Serial.println("Received: " + command);         // Echo for debugging
    handleCommand(command);                         // Process command
  }
}
