#include <ESP32Servo.h>
#include <cmath>

#define SERVO_COUNT 5
const uint8_t servoPins[SERVO_COUNT] = {31, 30, 29, 28, 27};
Servo myServos[SERVO_COUNT];
uint8_t currentAngles[SERVO_COUNT] = {90, 90, 90, 90, 90};

const double L1 = 121.0;
const double L2 = 122.0;
const double L3 = 95.0;

String inputString = "";

// kinematics struct

struct ThreeDofResult {
    double theta1a, theta2a, theta3a;
    double theta1b, theta2b, theta3b;
    bool valid;
};

struct InverseKinematicsResult {
    double deg1, deg2, deg3, deg4;
    bool valid;
};

struct ForwardKinematicsResult {
    double x, y, z, phi_deg;
};


//function
ThreeDofResult three_dof(double x, double y, double l1, double l2, double l3, double gamma);
InverseKinematicsResult inverse_kinematics(double x, double y, double z, double phi_deg, double l1, double l2, double l3);
ForwardKinematicsResult forward_kinematics(double deg1, double deg2, double deg3, double deg4);
void moveArmJointsArray(const uint8_t angles[]);
void parseAndExecute(String input);
String getValue(String data, char separator, int index);

void setup() {
    Serial.begin(115200);
    for (int i = 0; i < SERVO_COUNT; i++) {
        myServos[i].attach(servoPins[i]);
    }
    moveArmJointsArray(currentAngles);
    delay(500);
    Serial.println("\nReady");
}

// collect instruction
void loop() {
    if (Serial.available() > 0) {
        inputString = Serial.readStringUntil('\n');
        inputString.trim();

        if (inputString.length() > 0) {
            Serial.print("\nCalculating");
            Serial.println(inputString);
            parseAndExecute(inputString);
        }
    }
}

//parser function
void parseAndExecute(String input) {
    input.toLowerCase();

    int firstComma = input.indexOf(',');
    if (firstComma == -1) {
        Serial.println("Error: Format salah.");
        return;
    }

    String command = input.substring(0, firstComma);
    String values = input.substring(firstComma + 1);

    if (command == "ik") {
        // IK Format: x,y,z,phi,gripper
        double x = getValue(values, ',', 0).toFloat();
        double y = getValue(values, ',', 1).toFloat();
        double z = getValue(values, ',', 2).toFloat();
        double phi = getValue(values, ',', 3).toFloat();
        uint8_t gripper = getValue(values, ',', 4).toInt();

        Serial.print("  Starting IK --> Target: (X:");
        Serial.print(x); Serial.print(", Y:"); Serial.print(y);
        Serial.print(", Z:"); Serial.print(z); Serial.print(", Phi:");
        Serial.print(phi); Serial.print("), Gripper: "); Serial.println(gripper);

        InverseKinematicsResult res = inverse_kinematics(x, y, z, phi, L1, L2, L3);

        if (res.valid) {
            currentAngles[0] = (uint8_t)constrain(round(res.deg1), 0, 180); // Base
            currentAngles[1] = (uint8_t)constrain(round(res.deg2), 0, 180); // Shoulder
            currentAngles[2] = (uint8_t)constrain(round(res.deg3), 0, 180); // Elbow
            currentAngles[3] = (uint8_t)constrain(round(res.deg4), 0, 180); // Wrist
            currentAngles[4] = (uint8_t)constrain(gripper, 0, 180);         // Gripper

            moveArmJointsArray(currentAngles);

            Serial.print("  -> IK Sukses. Sudut: Base="); Serial.print(currentAngles[0]);
            Serial.print(", Sh="); Serial.print(currentAngles[1]);
            Serial.print(", El="); Serial.print(currentAngles[2]);
            Serial.print(", Wr="); Serial.println(currentAngles[3]);

        } else {
            Serial.println("target unreachable");
        }

    } else if (command == "fk") {
        // FK Format: base,shoulder,elbow,wrist,gripper
        double b = getValue(values, ',', 0).toFloat();
        double s = getValue(values, ',', 1).toFloat();
        double e = getValue(values, ',', 2).toFloat();
        double w = getValue(values, ',', 3).toFloat();
        uint8_t g = getValue(values, ',', 4).toInt();

        Serial.print("  Starting FK -> Target angle : [B:");
        Serial.print(b); Serial.print(", S:"); Serial.print(s);
        Serial.print(", E:"); Serial.print(e); Serial.print(", W:");
        Serial.print(w); Serial.print("], Gripper: "); Serial.println(g);

        currentAngles[0] = (uint8_t)constrain(b, 0, 180);
        currentAngles[1] = (uint8_t)constrain(s, 0, 180);
        currentAngles[2] = (uint8_t)constrain(e, 0, 180);
        currentAngles[3] = (uint8_t)constrain(w, 0, 180);
        currentAngles[4] = (uint8_t)constrain(g, 0, 180);
        
        ForwardKinematicsResult res = forward_kinematics(b, s, e, w);

        Serial.print("  -> FK Menghasilkan Koor: [X:");
        Serial.print(res.x); Serial.print(", Y:"); Serial.print(res.y);
        Serial.print(", Z:"); Serial.print(res.z); Serial.print(", Phi:");
        Serial.print(res.phi_deg); Serial.println("]");

        moveArmJointsArray(currentAngles);

    } else {
        Serial.println("Error input, use fk or ik");
    }
}

String getValue(String data, char separator, int index) {
    int found = 0;
    int strIndex[] = {0, -1};
    int maxIndex = data.length() - 1;
    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i + 1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

//servo controls

void moveArmJointsArray(const uint8_t angles[]) {
    // Loop melalui SEMUA 5 servo
    for (int i = 0; i < SERVO_COUNT; i++) {
        uint8_t constrainedAngle = constrain(angles[i], 0, 180);
        myServos[i].write(constrainedAngle);
    }
}

// kinematics

// fk start
ForwardKinematicsResult forward_kinematics(double deg1, double deg2, double deg3, double deg4) {
    ForwardKinematicsResult res = {0, 0, 0, 0};

    //convert to rad
    double t0_rad = deg1 * M_PI / 180.0; // Base
    double t1_rad = deg2 * M_PI / 180.0; // Shoulder
    double t2_rad = deg3 * M_PI / 180.0; // Elbow
    double t3_rad = deg4 * M_PI / 180.0; // Wrist

    //end effector
    double r_w = L1 * cos(t1_rad) + L2 * cos(t1_rad + t2_rad);
    double z_w = L1 * sin(t1_rad) + L2 * sin(t1_rad + t2_rad);
    
    double phi_rad = t1_rad + t2_rad + t3_rad;
  
    double r = r_w + L3 * cos(phi_rad);
    double z = z_w + L3 * sin(phi_rad);

    //konvert to x, y
    res.x = r * cos(t0_rad);
    res.y = r * sin(t0_rad);
    res.z = z;
    res.phi_deg = phi_rad * 180.0 / M_PI;

    return res;
}
//fk end

ThreeDofResult three_dof(double x, double y, double l1, double l2, double l3, double gamma) {
    ThreeDofResult result = {0, 0, 0, 0, 0, 0, false};

    //Wrist position
    double x3 = x - l3 * cos(gamma);
    double y3 = y - l3 * sin(gamma);

    // Check reachability
    double d = sqrt(x3 * x3 + y3 * y3);
    if (d > l1 + l2 || d < fabs(l1 - l2)) {
        return result; //invalid
    }

    //theta2
    double cos_theta2 = (x3 * x3 + y3 * y3 - l1 * l1 - l2 * l2) / (2 * l1 * l2);
    if (cos_theta2 > 1.0) cos_theta2 = 1.0;
    if (cos_theta2 < -1.0) cos_theta2 = -1.0;
    double theta2a = acos(cos_theta2);
    double theta2b = -theta2a;

    //theta1
    double theta1a = atan2(y3, x3) - atan2(l2 * sin(theta2a), l1 + l2 * cos(theta2a));
    double theta1b = atan2(y3, x3) - atan2(l2 * sin(theta2b), l1 + l2 * cos(theta2b));
    
    //Theta 3
    double theta3a = gamma - theta1a - theta2a;
    double theta3b = gamma - theta1b - theta2b;

    //Convert to degrees
    result.theta1a = theta1a * 180.0 / M_PI;
    result.theta2a = theta2a * 180.0 / M_PI;
    result.theta3a = theta3a * 180.0 / M_PI;
    result.theta1b = theta1b * 180.0 / M_PI;
    result.theta2b = theta2b * 180.0 / M_PI;
    result.theta3b = theta3b * 180.0 / M_PI;
    result.valid = true;

    return result;
}

// Function to calculate 4-DOF inverse kinematics (with base rotation)
InverseKinematicsResult inverse_kinematics(double x, double y, double z, double phi_deg, double l1, double l2, double l3) {
    InverseKinematicsResult result = {0, 0, 0, 0, false};
    double phi_rad = phi_deg * M_PI / 180.0;
    double r = sqrt(x * x + y * y);
    double deg1 = atan2(y, x) * 180.0 / M_PI;
    ThreeDofResult three_result = three_dof(r, z, l1, l2, l3, phi_rad);
    if (!three_result.valid) {
        return result;
    }

    //choose elbow-up
    result.deg1 = deg1;
    result.deg2 = three_result.theta1b;
    result.deg3 = three_result.theta2b;
    result.deg4 = three_result.theta3b;
    result.valid = true;
    return result;
}