//=====================================================================================================
// MahonyAHRS.h
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
#ifndef MahonyAHRS_h
#define MahonyAHRS_h

//----------------------------------------------------------------------------------------------------
// Variable declaration
class Mahony{
private:
    float twoKp;			// 2 * proportional gain (Kp)
    float twoKi;			// 2 * integral gain (Ki)
    float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
    float integralFBx, integralFBy, integralFBz;
    bool initialized;
    float expectedGravity;
    float initDuration, initAx, initAy, initAz;
    bool initializeIfStationary(float gx, float gy, float gz, float ax, float ay, float az, float dt);
    bool gravityReliable(float ax, float ay, float az) const;
public:
    explicit Mahony(float gravity = 9.80511f);
    void reset();
    bool isInitialized() const { return initialized; }
    void initOrientationFromAccel(float ax, float ay, float az);
    void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
    void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt);
    static float invSqrt(float x);
    float getQuaternionX() {
		return q1;
	}
    float getQuaternionY() {
		return q2;
	}
    float getQuaternionZ() {
		return q3;
	}
    float getQuaternionW() {
		return q0;
	}
};

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
