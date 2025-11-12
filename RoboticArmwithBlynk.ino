#define BLYNK_TEMPLATE_ID "TMPL6WWYDNwYo"
#define BLYNK_TEMPLATE_NAME "Robotic Arm"
#define BLYNK_AUTH_TOKEN "3SxvlAf2gZReXwMP78pkn7tXYV0r9YcB"   // <-- ganti

// ===== ESP32 + Blynk: 5 Servo + 5 Gauge, plus FK/IK via Serial =====
#include <ESP32Servo.h>
#include <BlynkSimpleEsp32.h>
#include <cmath>
#include <WiFi.h>

// ---------- Amankan M_PI di Arduino ----------
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ---------- Blynk Credentials ----------
char ssid[] = "Pooh";             // <-- ganti
char pass[] = "hahahaha";         // <-- ganti

// ---------- Virtual Pin Mapping ----------
// Sliders (input dari app)
#define VS_SERVO1  V0
#define VS_SERVO2  V1
#define VS_SERVO3  V2
#define VS_SERVO4  V3
#define VS_SERVO5  V4
// Gauges (output ke app)
#define VG_SERVO1  V10
#define VG_SERVO2  V11
#define VG_SERVO3  V12
#define VG_SERVO4  V13
#define VG_SERVO5  V14

// ---------- Servo config ----------
#define SERVO_COUNT 5

// NOTE: Ganti ke GPIO yang valid untuk ESP32 kamu.
// 27,26,25,33,32 itu umum aman untuk PWM servo.
const uint8_t servoPins[SERVO_COUNT] = {19, 18, 5, 17, 16};  // Base, Shoulder, Elbow, Wrist, Gripper

Servo   myServos[SERVO_COUNT];
uint8_t currentAngles[SERVO_COUNT] = {90, 90, 90, 90, 90};

inline uint8_t clampDeg(int d){ return (d<0)?0:((d>180)?180:d); }

// ===================================================================
//      Bagian KINEMATIKA & LOGIKA kamu (dipakai apa adanya)
// ===================================================================
const double L1 = 121.0;
const double L2 = 122.0;
const double L3 = 95.0;

String inputString = "";

// ---- structs
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

// ---- forward declares
ThreeDofResult three_dof(double x, double y, double l1, double l2, double l3, double gamma);
InverseKinematicsResult inverse_kinematics(double x, double y, double z, double phi_deg, double l1, double l2, double l3);
ForwardKinematicsResult forward_kinematics(double deg1, double deg2, double deg3, double deg4);
void moveArmJointsArray(const uint8_t angles[]);
void parseAndExecute(String input);
String getValue(String data, char separator, int index);

// ===================================================================
//                          BLYNK HELPERS
// ===================================================================
void writeServoAndGauge(int idx, int deg, uint8_t vGauge)
{
  uint8_t a = clampDeg(deg);
  currentAngles[idx] = a;
  myServos[idx].write(a);
  // update gauge yang terkait
  Blynk.virtualWrite(vGauge, a);
}

// Saat perangkat baru konek: minta nilai slider terakhir & seed gauge
BLYNK_CONNECTED()
{
  Blynk.syncVirtual(VS_SERVO1, VS_SERVO2, VS_SERVO3, VS_SERVO4, VS_SERVO5);
  Blynk.virtualWrite(VG_SERVO1, currentAngles[0]);
  Blynk.virtualWrite(VG_SERVO2, currentAngles[1]);
  Blynk.virtualWrite(VG_SERVO3, currentAngles[2]);
  Blynk.virtualWrite(VG_SERVO4, currentAngles[3]);
  Blynk.virtualWrite(VG_SERVO5, currentAngles[4]);
}

// Callback slider (input derajat dari app)
BLYNK_WRITE(VS_SERVO1){ writeServoAndGauge(0, param.asInt(), VG_SERVO1); }
BLYNK_WRITE(VS_SERVO2){ writeServoAndGauge(1, param.asInt(), VG_SERVO2); }
BLYNK_WRITE(VS_SERVO3){ writeServoAndGauge(2, param.asInt(), VG_SERVO3); }
BLYNK_WRITE(VS_SERVO4){ writeServoAndGauge(3, param.asInt(), VG_SERVO4); }
BLYNK_WRITE(VS_SERVO5){ writeServoAndGauge(4, param.asInt(), VG_SERVO5); }

// ===================================================================
//                           SETUP / LOOP
// ===================================================================
// void setup() {
//   Serial.begin(115200);

//   // attach servo ke pin
//   for (int i = 0; i < SERVO_COUNT; i++) {
//     myServos[i].attach(servoPins[i]);
//     myServos[i].write(currentAngles[i]);
//   }

//   // konek Blynk Cloud
//   Blynk.begin("3SxvlAf2gZReXwMP78pkn7tXYV0r9YcB", "OPPO A3 Pro 5G", "k3wxv5ge", "blynk.cloud", 80);

//   // posisi awal
//   moveArmJointsArray(currentAngles);
//   delay(300);
//   Serial.println("\nReady (Blynk + 5 Servo).");
// }

// ===== GANTI SELURUH FUNGSI SETUP ANDA DENGAN INI =====

void setup() {
  Serial.begin(115200);
  Serial.println("\n\nBooting... Board reset."); // Pesan 1: Tanda board hidup

  // attach servo ke pin
  for (int i = 0; i < SERVO_COUNT; i++) {
    myServos[i].attach(servoPins[i]);
    myServos[i].write(currentAngles[i]);
  }
  
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  // Ini adalah fungsi yang benar untuk menghubungkan WiFi dan Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  Serial.println("WiFi connected!");
  Serial.println("Connecting to Blynk server...");

  // posisi awal
  moveArmJointsArray(currentAngles);
  delay(300);
  
  Serial.println("\nReady (Blynk + 5 Servo)."); // Pesan 2: Tanda semua berhasil
}

void loop() {
  Blynk.run();

  // parser perintah via Serial (fk/ik seperti punyamu)
  if (Serial.available() > 0) {
    inputString = Serial.readStringUntil('\n');
    inputString.trim();
    if (inputString.length() > 0) {
      Serial.print("\nCalculating ");
      Serial.println(inputString);
      parseAndExecute(inputString);
      // setiap selesai gerak via parser, push gauge semua
      Blynk.virtualWrite(VG_SERVO1, currentAngles[0]);
      Blynk.virtualWrite(VG_SERVO2, currentAngles[1]);
      Blynk.virtualWrite(VG_SERVO3, currentAngles[2]);
      Blynk.virtualWrite(VG_SERVO4, currentAngles[3]);
      Blynk.virtualWrite(VG_SERVO5, currentAngles[4]);
    }
  }
}

// ===================================================================
//                     FUNGSI KAMU (asli, dipertahankan)
// ===================================================================
void moveArmJointsArray(const uint8_t angles[]) {
  for (int i = 0; i < SERVO_COUNT; i++) {
    uint8_t constrainedAngle = clampDeg(angles[i]);
    myServos[i].write(constrainedAngle);
    currentAngles[i] = constrainedAngle;
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

ForwardKinematicsResult forward_kinematics(double deg1, double deg2, double deg3, double deg4) {
  ForwardKinematicsResult res = {0, 0, 0, 0};
  double t0_rad = deg1 * M_PI / 180.0; // Base
  double t1_rad = deg2 * M_PI / 180.0; // Shoulder
  double t2_rad = deg3 * M_PI / 180.0; // Elbow
  double t3_rad = deg4 * M_PI / 180.0; // Wrist

  double r_w = L1 * cos(t1_rad) + L2 * cos(t1_rad + t2_rad);
  double z_w = L1 * sin(t1_rad) + L2 * sin(t1_rad + t2_rad);
  double phi_rad = t1_rad + t2_rad + t3_rad;

  double r = r_w + L3 * cos(phi_rad);
  double z = z_w + L3 * sin(phi_rad);

  res.x = r * cos(t0_rad);
  res.y = r * sin(t0_rad);
  res.z = z;
  res.phi_deg = phi_rad * 180.0 / M_PI;
  return res;
}

ThreeDofResult three_dof(double x, double y, double l1, double l2, double l3, double gamma) {
  ThreeDofResult result = {0,0,0,0,0,0,false};
  double x3 = x - l3 * cos(gamma);
  double y3 = y - l3 * sin(gamma);

  double d = sqrt(x3*x3 + y3*y3);
  if (d > l1 + l2 || d < fabs(l1 - l2)) return result;

  double cos_theta2 = (x3*x3 + y3*y3 - l1*l1 - l2*l2) / (2 * l1 * l2);
  cos_theta2 = fmax(-1.0, fmin(1.0, cos_theta2));
  double theta2a = acos(cos_theta2);
  double theta2b = -theta2a;

  double theta1a = atan2(y3, x3) - atan2(l2 * sin(theta2a), l1 + l2 * cos(theta2a));
  double theta1b = atan2(y3, x3) - atan2(l2 * sin(theta2b), l1 + l2 * cos(theta2b));
  double theta3a = gamma - theta1a - theta2a;
  double theta3b = gamma - theta1b - theta2b;

  result.theta1a = theta1a * 180.0 / M_PI;
  result.theta2a = theta2a * 180.0 / M_PI;
  result.theta3a = theta3a * 180.0 / M_PI;
  result.theta1b = theta1b * 180.0 / M_PI;
  result.theta2b = theta2b * 180.0 / M_PI;
  result.theta3b = theta3b * 180.0 / M_PI;
  result.valid = true;
  return result;
}

InverseKinematicsResult inverse_kinematics(double x, double y, double z, double phi_deg, double l1, double l2, double l3) {
  InverseKinematicsResult result = {0,0,0,0,false};
  double phi_rad = phi_deg * M_PI / 180.0;
  double r = sqrt(x*x + y*y);
  double deg1 = atan2(y, x) * 180.0 / M_PI;
  ThreeDofResult three_result = three_dof(r, z, l1, l2, l3, phi_rad);
  if (!three_result.valid) return result;

  // pilih elbow-up (varian b)
  result.deg1 = deg1;
  result.deg2 = three_result.theta1b;
  result.deg3 = three_result.theta2b;
  result.deg4 = three_result.theta3b;
  result.valid = true;
  return result;
}

// ===================================================================
//                   Parser perintah "ik,..." / "fk,..."
// ===================================================================
void parseAndExecute(String input) {
  input.toLowerCase();

  int firstComma = input.indexOf(',');
  if (firstComma == -1) { Serial.println("Error: Format salah."); return; }

  String command = input.substring(0, firstComma);
  String values  = input.substring(firstComma + 1);

  if (command == "ik") {
    // IK Format: x,y,z,phi,gripper
    double x   = getValue(values, ',', 0).toFloat();
    double y   = getValue(values, ',', 1).toFloat();
    double z   = getValue(values, ',', 2).toFloat();
    double phi = getValue(values, ',', 3).toFloat();
    uint8_t g  = getValue(values, ',', 4).toInt();

    Serial.printf("  IK Target: (X:%.2f, Y:%.2f, Z:%.2f, Phi:%.2f), Grip:%d\n", x,y,z,phi,g);

    InverseKinematicsResult res = inverse_kinematics(x, y, z, phi, L1, L2, L3);
    if (res.valid) {
      currentAngles[0] = clampDeg((int)round(res.deg1)); // Base
      currentAngles[1] = clampDeg((int)round(res.deg2)); // Shoulder
      currentAngles[2] = clampDeg((int)round(res.deg3)); // Elbow
      currentAngles[3] = clampDeg((int)round(res.deg4)); // Wrist
      currentAngles[4] = clampDeg(g);                    // Gripper

      moveArmJointsArray(currentAngles);

      Serial.printf("  -> IK OK. Angles: B=%d, Sh=%d, El=%d, Wr=%d, Gr=%d\n",
                    currentAngles[0],currentAngles[1],currentAngles[2],currentAngles[3],currentAngles[4]);
    } else {
      Serial.println("  -> IK: target unreachable");
    }

  } else if (command == "fk") {
    // FK Format: base,shoulder,elbow,wrist,gripper
    double b = getValue(values, ',', 0).toFloat();
    double s = getValue(values, ',', 1).toFloat();
    double e = getValue(values, ',', 2).toFloat();
    double w = getValue(values, ',', 3).toFloat();
    uint8_t g = getValue(values, ',', 4).toInt();

    Serial.printf("  FK Target Angles: [B:%.1f, S:%.1f, E:%.1f, W:%.1f], Grip:%d\n", b,s,e,w,g);

    currentAngles[0] = clampDeg((int)round(b));
    currentAngles[1] = clampDeg((int)round(s));
    currentAngles[2] = clampDeg((int)round(e));
    currentAngles[3] = clampDeg((int)round(w));
    currentAngles[4] = clampDeg(g);

    ForwardKinematicsResult res = forward_kinematics(b, s, e, w);
    Serial.printf("  -> FK Pos: X=%.2f, Y=%.2f, Z=%.2f, Phi=%.2f deg\n", res.x,res.y,res.z,res.phi_deg);

    moveArmJointsArray(currentAngles);

  } else {
    Serial.println("Error input, use fk or ik");
  }
}
