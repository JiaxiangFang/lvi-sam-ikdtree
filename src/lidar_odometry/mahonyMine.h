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
    bool initialized;       // 首帧加速度初始化标志(消除大倾角启动瞬态)
public:
	Mahony();
    void initOrientationFromAccel(float ax, float ay, float az); // 用首帧重力方向直接置四元数
	void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
    float invSqrt(float x);
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
