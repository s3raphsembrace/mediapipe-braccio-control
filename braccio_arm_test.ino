#include <Braccio.h>
#include <Servo.h>

Servo base, shoulder, elbow, wrist_rot, wrist_ver, gripper;

// --- SAFETY LIMITS ---
// Format: {min, max} for each servo
// [0] base, [1] shoulder, [2] elbow, [3] wrist_rot, [4] wrist_ver, [5] gripper
const int LIMITS[6][2] = {
  {90,   90},  // base
  {45,  90},  // shoulder (avoid hitting frame)
  {0,   180},  // elbow
  {0,   180},  // wrist_rot
  {0,   180},  // wrist_ver
  {10,  73}    // gripper (matches Python script range)
};

// Current angles (start at safe home position)
int current[6] = {90, 90, 90, 90, 90, 10};

int clampAngle(int angle, int joint) {
  return constrain(angle, LIMITS[joint][0], LIMITS[joint][1]);
}

void setup() {
  Serial.begin(115200);
  Braccio.begin();
  // Move to home position on startup
  Braccio.ServoMovement(20, 90, 90, 90, 90, 90, 10);
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    data.trim();

    // --- PARSE ---
    int target[6];
    int foundCount = 0;
    int startIdx = 0;

    for (int i = 0; i <= data.length() && foundCount < 6; i++) {
      if (i == data.length() || data.charAt(i) == ',') {
        target[foundCount] = clampAngle(data.substring(startIdx, i).toInt(), foundCount);
        foundCount++;
        startIdx = i + 1;
      }
    }

    if (foundCount < 6) return; // Incomplete packet — discard

    // --- SMOOTH MOVEMENT ---
    // Interpolate in small steps instead of jumping directly to target.
    // Step size controls speed vs smoothness tradeoff.
    const int STEP = 3;      // degrees per iteration (increase for faster, decrease for smoother)
    const int DELAY = 5;     // ms between steps (lower = faster)

    bool moving = true;
    while (moving) {
      moving = false;
      for (int j = 0; j < 6; j++) {
        int diff = target[j] - current[j];
        if (diff != 0) {
          moving = true;
          current[j] += constrain(diff, -STEP, STEP);
        }
      }
      Braccio.ServoMovement(0, current[0], current[1], current[2], current[3], current[4], current[5]);
      delay(DELAY);
    }
  }
}