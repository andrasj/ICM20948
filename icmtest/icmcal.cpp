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

	imu.EnableGyro(ICM20948::GyroSampleRate::LPF_12HZ,ICM20948::GyroScale::MAX_500DPS);
	imu.EnableAcc(ICM20948::AccSampleRate::LPF_12HZ,ICM20948::AccScale::MAX_4G);
	imu.EnableMag(ICM20948::MagSampleRate::Mode10Hz);

	findMinMaxAcc(imu);

	return 0;
}

int16_t takeSamples(ICM20948& imu,size_t nbSamples,size_t ixData)
{
	int16_t acc[3],gyr[3];
	int	sum=0;
	for (size_t i = 0; i < nbSamples; i++)
	{
		imu.ReadAccGyrRaw(acc,gyr);
		sum += acc[ixData];
		usleep(200*1000);
	}
	int16_t val = (sum / (int)nbSamples);
	printf("Sampled avg of %d (sum: %d) measurementes: %dn",nbSamples,sum,val);
	return val;
}
extern "C" {
	#include <termios.h>
	}
void RestoreKeyboardBlocking(struct termios *initial_settings)
{
	tcsetattr(0, TCSANOW, initial_settings);
}
void SetKeyboardNonBlock(struct termios *initial_settings)
{

    struct termios new_settings;
    tcgetattr(0,initial_settings);

    new_settings = *initial_settings;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    new_settings.c_lflag &= ~ISIG;
    new_settings.c_cc[VMIN] = 0;
    new_settings.c_cc[VTIME] = 0;

    tcsetattr(0, TCSANOW, &new_settings);
}

void findMinMaxAcc(ICM20948& imu)
{
    struct termios term_settings;
	
	int16_t minX,minY,minZ,maxX,maxY,maxZ;
	minX=minY=minZ=maxX=maxY=maxZ=0;

	printf("place device for minX acceration\n");
	printf("press enter twice, first to find minX, second time, the calibration point will be taken\n");
	char tmp;
	while (std::cin.read(&tmp,1) && tmp != '\n') usleep(100*1000);
    auto nextPrintTime = system_clock::now() + milliseconds(500);
		SetKeyboardNonBlock(&term_settings);
	while (true)
	{
		char cmd=getchar();

			bool done = false;
			switch (cmd)
			{
			case 's':
				done=true;
				break;
			case 'x':
				minX = takeSamples(imu,5,0);
				break;
			case 'X':
				maxX = takeSamples(imu,5,0);
				break;
			case 'y':
				minY = takeSamples(imu,5,1);
				break;
			case 'Y':
				maxY = takeSamples(imu,5,1);
				break;
			case 'z':
				minZ = takeSamples(imu,5,2);
				break;
			case 'Z':
				maxZ = takeSamples(imu,5,2);
				break;
			default:
				break;
			}
			if (done){
				RestoreKeyboardBlocking(&term_settings);
				break;
			}

		int16_t acc[3],gyr[3];
		imu.ReadAccGyrRaw(acc,gyr);

		
		auto now = system_clock::now();
		if (now > nextPrintTime)
		{
			printf("Acc(g): %5d %5d %5d\n",acc[0],acc[1],acc[2]);
			nextPrintTime = now+milliseconds(500);
		}

		usleep(100*1000);
	}
	printf("Here are your settings:\n");
	printf("minX: %6d maxX: %6d  dx: %5d  LSB/g: %8.5f \n",minX,maxX,maxX+minX,(maxX-minX)/2.0);
	printf("minY: %6d maxY: %6d  dx: %5d  LSB/g: %8.5f \n",minY,maxY,maxY+minY,(maxY-minY)/2.0);
	printf("minZ: %6d maxZ: %6d  dx: %5d  LSB/g: %8.5f \n",minZ,maxZ,maxZ+minZ,(maxZ-minZ)/2.0);


}
