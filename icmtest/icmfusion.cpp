#include "icm20948driver/ICM20948.h"
#include <iostream>
#include <unistd.h>
#include <chrono>

extern "C"
{
#include "Fusion/Fusion.h"
}

using namespace std;
using namespace std::chrono;
 
//=============================================================================

int main()
{
	//-------------------------------------------------------------------------

	ICM20948 imu;
	if (!imu.Init(0,0)) { printf("Init failed"); return 1;}

	unsigned int me = imu.whoami();
	printf("WhoAmI: 0x%x\n",me);
	unsigned int meMag = imu.whoamiMag();
	printf("WhoAmIMagnetometer: 0x%x\n",meMag);

	imu.EnableGyro(ICM20948::GyroSampleRate::LPF_12HZ,ICM20948::GyroScale::MAX_500DPS);
	imu.EnableAcc(ICM20948::AccSampleRate::LPF_12HZ,ICM20948::AccScale::MAX_4G);
	//imu.EnableMag(ICM20948::MagSampleRate::Mode10Hz);

	FusionBias fusionBias;
	FusionAhrs fusionAhrs;

	float samplePeriod = 1.0f/12.0f; // replace this value with actual sample period in seconds

	// replace these values with actual sensitivity in degrees per second per lsb as specified in gyroscope datasheet
	FusionVector3 gyroscopeSensitivity;
	gyroscopeSensitivity.axis.x = imu.GetGyroSensitivity(); // 1.0f;
	gyroscopeSensitivity.axis.y = imu.GetGyroSensitivity(); // 1.0f;
	gyroscopeSensitivity.axis.z = imu.GetGyroSensitivity(); // 1.0f;

/*
minX: -16476 maxX:  16276  dx:  -200  LSB/g: 16376.00000 
minY: -16452 maxY:  16170  dx:  -282  LSB/g: 16311.00000 
minZ: -16740 maxZ:  16380  dx:  -360  LSB/g: 16560.00000 
*/
	// replace these values with actual sensitivity in g per lsb as specified in accelerometer datasheet
	FusionVector3 accelerometerSensitivity;
	accelerometerSensitivity.axis.x = 1.0/16376; //imu.GetAccSensitivity(); // 1.0f;
	accelerometerSensitivity.axis.y = 1.0/16311 ; //imu.GetAccSensitivity(); // 1.0f;
	accelerometerSensitivity.axis.z = 1.0/16560 ; //imu.GetAccSensitivity(); // 1.0f;

	// replace these values with actual hard-iron bias in uT if known
	// FusionVector3 hardIronBias;
	// hardIronBias.axis.x = 0.0f;
	// hardIronBias.axis.y = 0.0f;
	// hardIronBias.axis.z = 0.0f;

    // Initialise gyroscope bias correction algorithm
    FusionBiasInitialise(&fusionBias, 0.5f, samplePeriod); // stationary threshold = 0.5 degrees per second

    // Initialise AHRS algorithm
    FusionAhrsInitialise(&fusionAhrs, 0.5f); // gain = 0.5

    // Set optional magnetic field limits
    //FusionAhrsSetMagneticField(&fusionAhrs, 20.0f, 70.0f); // valid magnetic field range = 20 uT to 70 uT

	useconds_t sleeptime;
	while (true)
	{
	    high_resolution_clock::time_point sampleTime = high_resolution_clock::now();
//		double gx,gy,gz,ax,ay,az,mx,my,mz;
//		imu.ReadAccGyr(ax,ay,az,gx,gy,gz);
		int16_t acc[3],gyr[3];
		imu.ReadAccGyrRaw(acc,gyr);
		//imu.ReadMag(mx,my,mz);

		// printf("Gyr(dps): %3.0f %3.0f %3.0f - Acc(g): %4.2f %4.2f %4.2f - Mag(uT) %4.2f %4.2f %4.2f\n",gx,gy,gz,ax,ay,az,mx,my,mz);
		// printf("Gyr(dps): %5d %5d %5d - Acc(g): %5d %5d %5d - Mag(uT) %4.2f %4.2f %4.2f\n",gyr[0],gyr[1],gyr[2],acc[0],acc[1],acc[2],mx,my,mz);

        // Calibrate gyroscope
        FusionVector3 uncalibratedGyroscope;
        uncalibratedGyroscope.axis.x = gyr[0]; //0.0f; /* replace this value with actual gyroscope x axis measurement in lsb */
        uncalibratedGyroscope.axis.y = gyr[1]; //0.0f; /* replace this value with actual gyroscope y axis measurement in lsb */
        uncalibratedGyroscope.axis.z = gyr[2]; //0.0f; /* replace this value with actual gyroscope z axis measurement in lsb */
        FusionVector3 calibratedGyroscope = FusionCalibrationInertial(uncalibratedGyroscope, FUSION_ROTATION_MATRIX_IDENTITY, gyroscopeSensitivity, FUSION_VECTOR3_ZERO);

        // Calibrate accelerometer
        FusionVector3 uncalibratedAccelerometer;
        uncalibratedAccelerometer.axis.x = acc[0]; //0.0f; /* replace this value with actual accelerometer x axis measurement in lsb */
        uncalibratedAccelerometer.axis.y = acc[1]; //0.0f; /* replace this value with actual accelerometer y axis measurement in lsb */
        uncalibratedAccelerometer.axis.z = acc[2]; //1.0f; /* replace this value with actual accelerometer z axis measurement in lsb */

        FusionVector3 calibratedAccelerometer = FusionCalibrationInertial(uncalibratedAccelerometer, FUSION_ROTATION_MATRIX_IDENTITY, accelerometerSensitivity, FUSION_VECTOR3_ZERO);

        // Calibrate magnetometer
        // FusionVector3 uncalibratedMagnetometer;
        // uncalibratedMagnetometer.axis.x = 0.5f; /* replace this value with actual magnetometer x axis measurement in uT */
        // uncalibratedMagnetometer.axis.y = 0.0f; /* replace this value with actual magnetometer y axis measurement in uT */
        // uncalibratedMagnetometer.axis.z = 0.0f; /* replace this value with actual magnetometer z axis measurement in uT */

        // FusionVector3 calibratedMagnetometer = FusionCalibrationMagnetic(uncalibratedMagnetometer, FUSION_ROTATION_MATRIX_IDENTITY, hardIronBias);

        // Update gyroscope bias correction algorithm
        calibratedGyroscope = FusionBiasUpdate(&fusionBias, calibratedGyroscope);


        // Update AHRS algorithm
        FusionAhrsUpdateWithoutMagnetometer(&fusionAhrs, calibratedGyroscope, calibratedAccelerometer, samplePeriod);
//        FusionAhrsUpdate(&fusionAhrs, calibratedGyroscope, calibratedAccelerometer, calibratedMagnetometer, samplePeriod);

        // Print Euler angles
        FusionEulerAngles eulerAngles = FusionQuaternionToEulerAngles(FusionAhrsGetQuaternion(&fusionAhrs));
        printf("Roll = %0.1f, Pitch = %0.1f, Yaw = %0.1f\r\n", eulerAngles.angle.roll, eulerAngles.angle.pitch, eulerAngles.angle.yaw);

		sleeptime = (samplePeriod-duration_cast<duration<float>>(high_resolution_clock::now()-sampleTime).count())*1000000;
		usleep(sleeptime);
	}

	return 0;
}
