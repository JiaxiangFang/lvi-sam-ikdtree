//=====================================================================================================
// MahonyAHRS.c
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include "mahonyMine.h"
#include "imu_time.h"
#include <cmath>
//---------------------------------------------------------------------------------------------------
// Definitions

#define twoKpDef	(2.0f * 5.0f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions
Mahony::Mahony(float gravity) : expectedGravity(gravity)
{
    if (!std::isfinite(expectedGravity) || expectedGravity <= 0.0f)
        expectedGravity = 9.80511f;
    reset();
}

void Mahony::reset()
{
	twoKp = twoKpDef;	// 2 * proportional gain (Kp)
	twoKi = twoKiDef;	// 2 * integral gain (Ki)
	q0 = 1.0f;          // quaternion of sensor frame relative to auxiliary frame
	q1 = 0.0f;
	q2 = 0.0f;
	q3 = 0.0f;
	integralFBx = 0.0f; // integral error terms scaled by Ki
	integralFBy = 0.0f; 
	integralFBz = 0.0f;
	initialized = false;
    initDuration = initAx = initAy = initAz = 0.0f;
}

bool Mahony::gravityReliable(float ax, float ay, float az) const
{
    const double norm = std::sqrt(double(ax) * ax + double(ay) * ay + double(az) * az);
    return std::isfinite(norm) && std::fabs(norm - expectedGravity) <= 0.15 * expectedGravity;
}

bool Mahony::initializeIfStationary(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    // Require a short near-stationary segment instead of treating one arbitrary
    // acceleration sample as gravity. This cannot detect every external force.
    const double gyroNorm = std::sqrt(double(gx) * gx + double(gy) * gy + double(gz) * gz);
    if (!gravityReliable(ax, ay, az) || gyroNorm > 0.35)
    {
        initDuration = initAx = initAy = initAz = 0.0f;
        return false;
    }
    initDuration += dt;
    initAx += ax * dt;
    initAy += ay * dt;
    initAz += az * dt;
    if (initDuration < 0.1f)
        return false;
    initOrientationFromAccel(initAx / initDuration, initAy / initDuration, initAz / initDuration);
    return initialized;
}

// Roll/pitch from a validated gravity estimate; yaw remains unobservable.
void Mahony::initOrientationFromAccel(float ax, float ay, float az) {
	if (!gravityReliable(ax, ay, az))
		return;
	float roll  = atan2f(ay, az);
	float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
	float cr = cosf(roll * 0.5f),  sr = sinf(roll * 0.5f);
	float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
	q0 = cr * cp;
	q1 = sr * cp;
	q2 = cr * sp;
	q3 = -sr * sp;
	integralFBx = integralFBy = integralFBz = 0.0f;
    initialized = true;
}
			

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void Mahony::MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt) {
    if (!imu_safety::validInterval(dt) || !std::isfinite(gx) || !std::isfinite(gy) || !std::isfinite(gz) ||
        !std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(az))
        return;
    if (!initialized)
    {
        initializeIfStationary(gx, gy, gz, ax, ay, az, dt);
        return;
    }
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if (!std::isfinite(mx) || !std::isfinite(my) || !std::isfinite(mz) ||
        ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))) {
        MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az, dt);
        return;
    }

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(gravityReliable(ax, ay, az)) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        // Estimated direction of gravity and magnetic field
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        // Error is sum of cross product between estimated direction and measured direction of field vectors
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        // Compute and apply integral feedback if enabled
        if(twoKi > 0.0f) {
            integralFBx += twoKi * halfex * dt;	// integral error scaled by Ki
            integralFBy += twoKi * halfey * dt;
            integralFBz += twoKi * halfez * dt;
            gx += integralFBx;	// apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else {
            integralFBx = 0.0f;	// prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt);		// measured timestamp interval
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void Mahony::MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    if (!imu_safety::validInterval(dt) || !std::isfinite(gx) || !std::isfinite(gy) || !std::isfinite(gz) ||
        !std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(az))
        return;
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Do not publish an uninitialized attitude during motion or missing gravity.
    if (!initialized) {
        initializeIfStationary(gx, gy, gz, ax, ay, az, dt);
        return;
    }

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(gravityReliable(ax, ay, az)) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if(twoKi > 0.0f) {
            integralFBx += twoKi * halfex * dt;	// integral error scaled by Ki
            integralFBy += twoKi * halfey * dt;
            integralFBz += twoKi * halfez * dt;
            gx += integralFBx;	// apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else {
            integralFBx = 0.0f;	// prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt);		// measured timestamp interval
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Portable inverse square root. Invalid norms are never type-punned or read
// past the float object (long is 64-bit on LP64 platforms).

float Mahony::invSqrt(float x) {
    return std::isfinite(x) && x > 0.0f ? 1.0f / std::sqrt(x) : 0.0f;
}

//====================================================================================================
// END OF CODE
//====================================================================================================
