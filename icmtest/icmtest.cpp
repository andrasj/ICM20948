#include "icm20948driver/ICM20948.h"
#include <iostream>
#include <unistd.h>
#include <chrono>

using namespace std;
using namespace std::chrono;
 
//=============================================================================

void findMinMaxAcc(ICM20948& imu);

int main()
{
	//-------------------------------------------------------------------------

	ICM20948 imu;
	if (!imu.Init(0,0)) { printf("Init failed"); return 1;}

	unsigned int me = imu.whoami();
	printf("WhoAmI: 0x%x\n",me);
	unsigned int meMag = imu.whoamiMag();
	printf("WhoAmIMagnetometer: 0x%x\n",meMag);

	imu.EnableGyro(ICM20948::GyroSampleRate::LPF_24HZ,ICM20948::GyroScale::MAX_250DPS);
	imu.EnableAcc(ICM20948::AccSampleRate::LPF_24HZ,ICM20948::AccScale::MAX_4G);
	imu.EnableMag(ICM20948::MagSampleRate::Mode10Hz);
	usleep(200*1000);
	double ax,ay,az,mx,my,mz;
	imu.ReadAcc(ax,ay,az);
	printf("%f %f %f\n",ax,ay,az);

	double gs = imu.GetGyroSensitivity();
	double as = imu.GetAccSensitivity();
	double ms = imu.GetMagSensitivity();

	int16_t acc[3],gyr[3],mag[3];
	while (true)
	{
		imu.ReadAccGyrMagRaw(acc,gyr,mag);
		//imu.ReadMag(mx,my,mz);
		//printf("          %4.4f       %04X     %+5d                      Mag(uT) %4.2f %4.2f %4.2f\n",ms,mag[2] & 0xffff,mag[2],mx,my,mz);
		printf("Gyr(dps): %4.2f %4.2f %4.2f - Acc(g): %4.2f %4.2f %4.2f - Mag(uT) %4.2f %4.2f %4.2f\n",gyr[0]*gs,gyr[1]*gs,gyr[2]*gs,acc[0]*as,acc[1]*as,acc[2]*as,mag[0]*ms,mag[1]*ms,mag[2]*ms);
		usleep(200*1000);
	}
	return 0;
}
