#include <TimerOne.h>
#include <Wire.h>
#include <Servo.h>
#include <NewPing.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// Constants
#define MOTOR1 2
#define MOTOR2 3
#define SERVO_PIN 10
#define ULTRASONIC_SENSOR_TRIG 11
#define ULTRASONIC_SENSOR_ECHO 12
#define MOTOR1_DIR 4
#define MOTOR1_EN 5
#define MOTOR2_DIR 6
#define MOTOR2_EN 7
#define MAX_REGULAR_MOTOR_SPEED 75
#define MAX_MOTOR_ADJUST_SPEED 150
#define DISTANCE_TO_CHECK 30
#define RX_PIN 0
#define TX_PIN 1
#define GPS_BAUD 9600
#define DISK_SLOTS 20
#define WHEEL_DIAMETER 0.066
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * 3.14)
#define ROBOT_WIDTH 0.2
#define PI 3.14159265358979323846
#define EARTH_RADIUS 6371.0
#define I2C_ADDRESS 0x08

// Global variables
SoftwareSerial ss(RX_PIN, TX_PIN);
Servo myServo;
NewPing mySensor(ULTRASONIC_SENSOR_TRIG, ULTRASONIC_SENSOR_ECHO, 400);
TinyGPSPlus gps;

unsigned int counter1 = 0, counter2 = 0;
float target, error = 0, integral = 0, derivative = 0, last_error = 0;
float Kp = 11, Ki = 0.09, Kd = 10, angle = 0;
int targetDistance, choose, index = 0;
int received[3];
int gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
boolean set_gyro_angles;
long acc_x, acc_y, acc_z, acc_total_vector;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
float angle_pitch_output, angle_roll_output;
double targetLatitude, targetLongitude;
long loop_timer;
int temp;
float motorSpeed = 127.5;

// Structs
struct DistanceResult {
    float distance1;
    float distance2;
};

struct LatLong {
    double latitude;
    double longitude;
};
union floatConverter {
    byte bytes[4];
    float value;
};
// Sensor and Hardware Setup Functions
void setup_mpu_6050_registers() {
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
    
    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();
    
    Wire.beginTransmission(0x68);
    Wire.write(0x1B);
    Wire.write(0x08);
    Wire.endTransmission();
}

void read_mpu_6050_data() {
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 14);
    while(Wire.available() < 14);
    
    acc_x = Wire.read()<<8|Wire.read();
    acc_y = Wire.read()<<8|Wire.read();
    acc_z = Wire.read()<<8|Wire.read();
    temp = Wire.read()<<8|Wire.read();
    gyro_x = Wire.read()<<8|Wire.read();
    gyro_y = Wire.read()<<8|Wire.read();
    gyro_z = Wire.read()<<8|Wire.read();
}

// Utility Functions
void ISR_count1() { counter1++; }
void ISR_count2() { counter2++; }
void ISR_timerone() {
    Timer1.detachInterrupt();
    Timer1.attachInterrupt(ISR_timerone);
}

DistanceResult getDistanceTravel() {
    Timer1.detachInterrupt();
    
    float rotation1 = (counter1 / DISK_SLOTS) * 60.00;
    float distance1 = rotation1 * WHEEL_CIRCUMFERENCE * 60;
    counter1 = 0;
    
    float rotation2 = (counter2 / DISK_SLOTS) * 60.00;
    float distance2 = rotation2 * WHEEL_CIRCUMFERENCE * 60;
    counter2 = 0;
    
    Timer1.attachInterrupt(ISR_timerone);
    
    return {distance1, distance2};
}

float pidControl(float target1) {
    read_mpu_6050_data();
    
    gyro_x -= gyro_x_cal;
    gyro_y -= gyro_y_cal;
    gyro_z -= gyro_z_cal;
    
    angle_pitch += gyro_x * 0.0000611;
    angle_roll += gyro_y * 0.0000611;
    
    angle_pitch += angle_roll * sin(gyro_z * 0.000001066);
    angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);
    
    acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));
    
    angle_pitch_acc = asin((float)acc_y/acc_total_vector) * 57.296;
    angle_roll_acc = asin((float)acc_x/acc_total_vector) * -57.296;
    
    angle_pitch_acc -= 0.0;
    angle_roll_acc -= 0.0;

    if(set_gyro_angles) {
        angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
        angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;
    } else {
        angle_pitch = angle_pitch_acc;
        angle_roll = angle_roll_acc;
        set_gyro_angles = true;
    }
    
    angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;
    angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;
    
    error = target1 - angle_pitch_output;
    integral = integral + error;
    derivative = error - last_error;
    
    angle = (error * Kp) + (integral * Ki) + (derivative * Kd);
    last_error = error;
    return angle_pitch_output;
}

// Motion Control Functions
void avoid_obstacle() {
    int distance = mySensor.ping_cm();
    
    if (distance > 0 && distance < DISTANCE_TO_CHECK) {
        rotateMotor(0, 0);
        delay(500);
        
        rotateMotor(-MAX_MOTOR_ADJUST_SPEED, -MAX_MOTOR_ADJUST_SPEED);
        delay(200);
        
        rotateMotor(0, 0);
        delay(500);
        
        myServo.write(180);
        delay(500);
        int distanceLeft = mySensor.ping_cm();
        
        myServo.write(0);
        delay(500);
        int distanceRight = mySensor.ping_cm();
        
        myServo.write(90);
        delay(500);
        
        if (distanceLeft == 0) {
            rotateMotor(MAX_MOTOR_ADJUST_SPEED, -MAX_MOTOR_ADJUST_SPEED);
        } else if (distanceRight == 0) {
            rotateMotor(-MAX_MOTOR_ADJUST_SPEED, MAX_MOTOR_ADJUST_SPEED);
        } else if (distanceLeft >= distanceRight) {
            rotateMotor(MAX_MOTOR_ADJUST_SPEED, -MAX_MOTOR_ADJUST_SPEED);
        } else {
            rotateMotor(-MAX_MOTOR_ADJUST_SPEED, MAX_MOTOR_ADJUST_SPEED);
        }
        delay(200);
        rotateMotor(0, 0);
        delay(200);
    } else {
        rotateMotor(MAX_REGULAR_MOTOR_SPEED, MAX_REGULAR_MOTOR_SPEED);
    }
}

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed) {
    digitalWrite(MOTOR2_DIR, rightMotorSpeed >= 0 ? HIGH : LOW);
    digitalWrite(MOTOR1_DIR, leftMotorSpeed >= 0 ? LOW : HIGH);
    
    analogWrite(MOTOR2_EN, abs(rightMotorSpeed));
    analogWrite(MOTOR1_EN, abs(leftMotorSpeed));
}

void forward(float leftM, float rightM) {
    digitalWrite(MOTOR1_DIR, LOW);
    analogWrite(MOTOR1_EN, leftM);
    digitalWrite(MOTOR2_DIR, LOW);
    analogWrite(MOTOR2_EN, rightM);
}

// GPS and Navigation Functions
double toRadians(double degrees) { return degrees * PI / 180.0; }
double toDegrees(double radians) { return radians * 180.0 / PI; }

LatLong read_gps() {
    while (ss.available() > 0) {
        if (gps.encode(ss.read()) && gps.location.isUpdated()) {
            return {gps.location.lat(), gps.location.lng()};
        }
    }
    return {0, 0};
}

double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    lat1 = toRadians(lat1);
    lon1 = toRadians(lon1);
    lat2 = toRadians(lat2);
    lon2 = toRadians(lon2);

    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;
    double a = sin(dlat/2) * sin(dlat/2) +
               cos(lat1) * cos(lat2) * 
               sin(dlon/2) * sin(dlon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return EARTH_RADIUS * c;
}

double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
    lat1 = toRadians(lat1);
    lon1 = toRadians(lon1);
    lat2 = toRadians(lat2);
    lon2 = toRadians(lon2);

    double dlon = lon2 - lon1;
    double y = sin(dlon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) -
               sin(lat1) * cos(lat2) * cos(dlon);
    double bearing = atan2(y, x);

    bearing = toDegrees(bearing);
    if (bearing > 180) {
        bearing -= 360;
    } else if (bearing < -180) {
        bearing += 360;
    }

    return bearing;
}

void receiveEvent(int howMany) {
    static int index = 0;
    static floatConverter converter;
    static int valueIndex = 0;

    while (Wire.available()) {
        byte c = Wire.read();
        
        if (index == 0) {
            choose = c;
            index++;
        } else {
            converter.bytes[valueIndex] = c;
            valueIndex++;
            
            if (valueIndex == 4) {
                if (index == 1) {
                    if (choose == 1) {
                        targetDistance = converter.value;
                    } else if (choose == 2) {
                        targetLongitude = converter.value;
                    }
                } else if (index == 2) {
                    if (choose == 1) {
                        target = converter.value;
                    } else if (choose == 2) {
                        targetLatitude = converter.value;
                    }
                }
                valueIndex = 0;
                index++;
            }
        }
    }
    
    if (index > 2) {
        Serial.println("Received values:");
        if (choose == 1) {
            Serial.print("Target Distance: ");
            Serial.println(targetDistance);
            Serial.print("Target Angle: ");
            Serial.println(target);
        } else if (choose == 2) {
            Serial.print("Target Longitude: ");
            Serial.println(targetLongitude, 6);
            Serial.print("Target Latitude: ");
            Serial.println(targetLatitude, 6);
        }
        Serial.println("--------------------");
        index = 0;
    }
}

void setup() {
    Serial.begin(9600);
    Wire.begin();
    Wire.begin(0x08);
    Wire.onReceive(receiveEvent);
    
    Timer1.initialize(1000000);
    attachInterrupt(digitalPinToInterrupt(MOTOR1), ISR_count1, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR2), ISR_count2, RISING);
    Timer1.attachInterrupt(ISR_timerone);
    
    myServo.attach(SERVO_PIN);
    myServo.write(90);
    ss.begin(GPS_BAUD);
    
    pinMode(MOTOR1_DIR, OUTPUT);
    pinMode(MOTOR2_DIR, OUTPUT);
    pinMode(MOTOR1_EN, OUTPUT);
    pinMode(MOTOR2_EN, OUTPUT);
    
    setup_mpu_6050_registers();
    
    for (int cal_int = 0; cal_int < 1000; cal_int++) {
        read_mpu_6050_data();
        gyro_x_cal += gyro_x;
        gyro_y_cal += gyro_y;
        gyro_z_cal += gyro_z;
        delay(3);
    }
    
    gyro_x_cal /= 1000;
    gyro_y_cal /= 1000;
    gyro_z_cal /= 1000;
    
    rotateMotor(0, 0);
}

void loop() {
    DistanceResult distances = getDistanceTravel();
    float currentDistance = (distances.distance1 + distances.distance2) / 2;

    switch (choose) {
        case 1:
            navigateUsingEncoderAndMPU6050(currentDistance);
            break;
        case 2:
            navigateUsingGPS();
            break;
        default:
            Serial.println("Invalid navigation mode selected");
            break;
    }
}

void navigateUsingEncoderAndMPU6050(float currentDistance) {
    Serial.println("Navigate using encoder, MPU6050");
    while (currentDistance < targetDistance) {
        rotateMotor(motorSpeed, motorSpeed);
        adjustMotorsBasedOnAngle(pidControl(target));
        handleObstacleAvoidance();
        updateCurrentDistance(currentDistance);
    }
}

void navigateUsingGPS() {
    Serial.println("Navigate using GPS");
    LatLong currentPosition = read_gps();
    double targetDistance = calculateDistance(currentPosition.latitude, currentPosition.longitude, targetLatitude, targetLongitude);
    float targetBearing = calculateBearing(currentPosition.latitude, currentPosition.longitude, targetLatitude, targetLongitude);
    
    float currentDistance = 0;
    while (currentDistance < targetDistance) {
        rotateMotor(motorSpeed, motorSpeed);
        adjustMotorsBasedOnAngle(pidControl(targetBearing));
        handleObstacleAvoidance();
        updateCurrentDistance(currentDistance);
    }
}

void adjustMotorsBasedOnAngle(float currentAngle) {
    if (currentAngle > 0) {
        forward(motorSpeed - abs(angle), motorSpeed + abs(angle));
    } else if (currentAngle < 0) {
        forward(motorSpeed + abs(angle), motorSpeed - abs(angle));
    }
}

void handleObstacleAvoidance() {
    int obstacleDistance = mySensor.ping_cm();
    if (obstacleDistance > 0 && obstacleDistance < 30) {
        avoid_obstacle();
    }
}

void updateCurrentDistance(float &currentDistance) {
    DistanceResult distances = getDistanceTravel();
    currentDistance = (distances.distance1 + distances.distance2) / 2;
}