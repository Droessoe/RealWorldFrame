#include "Fusion/Fusion.h"
#include "LSM6DSV16XSensor.h"
#include <iostream>
#include <chrono>
#include <cmath>
#include <iomanip>

#define SAMPLE_RATE       (240.0f)   // Hz

// Movement‑detection thresholds (g)
constexpr float MOTION_UPPER_G = 1.02f;
constexpr float MOTION_LOWER_G = 0.99f;
// Number of consecutive “still” samples before we capture reference (0.5 s)
constexpr int   STILL_SAMPLES  = static_cast<int>(20.0f * SAMPLE_RATE);

FusionAhrs   ahrs;
FusionOffset offset;

// Invert a unit quaternion (conjugate)
static inline FusionQuaternion InverseQuaternion(const FusionQuaternion &q) {
    FusionQuaternion inv = q;
    inv.element.x = -inv.element.x;
    inv.element.y = -inv.element.y;
    inv.element.z = -inv.element.z;
    return inv;
}

int main() {
    // 1) IMU init
    LSM6DSV16XSensor sensor(0, 5000000, 5000000, ODR_240Hz);
    if (!sensor.begin()) {
        std::cerr << "Failed to initialize IMU sensor!\n";
        return -1;
    }

    // 2) AHRS init
    FusionAhrsInitialise(&ahrs);
    FusionOffsetInitialise(&offset, SAMPLE_RATE);
    FusionAhrsSettings settings = {
        .convention            = FusionConventionNwu,
        .gain                  = 0.1f,
        .gyroscopeRange        = 2000.0f,
        .accelerationRejection = 10.0f,
        .magneticRejection     = 0.0f,
        .recoveryTriggerPeriod = static_cast<unsigned int>(5 * SAMPLE_RATE)
    };
    FusionAhrsSetSettings(&ahrs, &settings);

    // 3) Timing
    auto prevTime = std::chrono::high_resolution_clock::now();

    // 4) Calibration state
    bool            refSet     = false;
    FusionQuaternion q_ref      = {1,0,0,0};
    FusionQuaternion q_output   = q_ref;
    int             stillCount = 0;

    while (true) {
        // wait for new data
        if (!sensor.isDataAvailable()) continue;

        // compute dt
        auto    now  = std::chrono::high_resolution_clock::now();
        float   dt   = std::chrono::duration<float>(now - prevTime).count();
        prevTime     = now;

        // read IMU
        IMUData imu = sensor.GetAccelGyroAxes();
        FusionVector gyro = { imu.gyroX, imu.gyroY, imu.gyroZ };

        // convert accel to g
        float accelX_g = imu.accelX / 1000.0f;
        float accelY_g = imu.accelY / 1000.0f;
        float accelZ_g = imu.accelZ / 1000.0f;
        FusionVector accelerometer = { accelX_g, accelY_g, accelZ_g };

        // compute accel magnitude
        float accelMag = std::sqrt(
            accelX_g*accelX_g +
            accelY_g*accelY_g +
            accelZ_g*accelZ_g
        );

        // detect motion by accel only
        bool isMoving = (accelMag > MOTION_UPPER_G) ||
                        (accelMag < MOTION_LOWER_G);

        // --- Calibration phase: run filter but wait for STILL_SAMPLES of stillness ---
        if (!refSet) {
            // always update AHRS so it converges
            gyro = FusionOffsetUpdate(&offset, gyro);
            FusionAhrsUpdateNoMagnetometer(&ahrs, gyro, accelerometer, dt);
            q_output = FusionAhrsGetQuaternion(&ahrs);

            if (!isMoving) {
                if (++stillCount >= STILL_SAMPLES) {
                    // capture reference after X samples of stillness
                    q_ref = q_output;
                    refSet = true;
                    std::cout << "✦ Reference captured after "
                              << (STILL_SAMPLES / SAMPLE_RATE)
                              << " s of stillness.\n";
                }
            } else {
                stillCount = 0;  // reset counter if movement detected
            }
            continue;
        }

        // --- Tracking phase: ALWAYS update AHRS each frame (no motion check) ---
        gyro = FusionOffsetUpdate(&offset, gyro);
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyro, accelerometer, dt);
        q_output = FusionAhrsGetQuaternion(&ahrs);

        // compute delta quaternion: q_delta = q_ref⁻¹ * q_output
        FusionQuaternion q_delta = FusionQuaternionMultiply(
            InverseQuaternion(q_ref),
            q_output
        );
        // convert delta to Euler angles (degrees)
        FusionEuler deltaE = FusionQuaternionToEuler(q_delta);

        // print accel magnitude + Δ-angles
        std::cout << std::fixed << std::setprecision(3)
                  << "AccMag: " << accelMag << " g | "
                  << std::setprecision(2)
                  << "ΔRoll: "  << deltaE.angle.roll  << "°, "
                  << "ΔPitch: " << deltaE.angle.pitch << "°, "
                  << "ΔYaw: "   << deltaE.angle.yaw   << "°\n";
    }

    return 0;
}
