

#include "rodos.h"
#include <stdio.h>
#include "hal.h"
#include "math.h"
#include "matlib.h"

static Application module01("Template", 2001);

#define LED_GREEN GPIO_060
#define LED_ORANGE GPIO_061
#define LED_RED GPIO_062
#define LED_BLUE GPIO_063

#define BT_UART UART_IDX2
#define USB_UART UART_IDX3

#define IMU_I2C I2C_IDX2

HAL_GPIO GreenLED(LED_GREEN);
HAL_GPIO pa0(GPIO_000);

//HAL_I2C imu(I2C_IDX2);
HAL_GPIO imu_EN(GPIO_055);	// PD7, IMU Power Enable
HAL_GPIO cs_G(GPIO_018);	// PB2, Gyro Chip Select
HAL_GPIO cs_XM(GPIO_032);	// PC0, XM Chip Select
HAL_SPI imu(SPI_IDX1);
//HAL_UART BT2UART(UART_IDX2);


#define LSM9DS0_G				0x6B            // Gyro 7-bit I2C address
#define LSM9DS0_XM				0x1D			// IMU 7-bit I2C address

#define OUT_X_L_G				0x28
#define OUT_X_L_A				0x28
#define OUT_X_L_M				0x08
#define OUT_TEMP_L_XM			0x05

#define MSB						0x80

#define DPS_MAX					2000
#define ACC_MAX					2
#define MAG_MAX					2

#define DPS_DOUBLE(x)			((double)x*DPS_MAX/INT16_MAX)
#define ACC_DOUBLE(x)			((double)x*ACC_MAX/INT16_MAX)
#define TEMP_DOUBLE(x)			((double)x/4)
#define MAG_DOUBLE(x)			((double)x*MAG_MAX/INT16_MAX)
#define MAG_NORMALIZE(x, min, max)	((MAG_DOUBLE(x)-MAG_DOUBLE(min))/(MAG_DOUBLE(max)-MAG_DOUBLE(min))*2-1)

#define RAD2GRAD(x)			(x*180/M_PI)
#define GRAD2RAD(x)			(x*M_PI/180)

uint8_t LSM9DS0_G_DATA[1] = {OUT_X_L_G | MSB | 0x40};
uint8_t LSM9DS0_A_DATA[1] = {OUT_X_L_A | MSB | 0x40};
uint8_t LSM9DS0_M_DATA[1] = {OUT_X_L_M | MSB | 0x40};
uint8_t LSM9DS0_T_DATA[1] = {OUT_TEMP_L_XM | MSB | 0x40};

//uint8_t LSM9DS0_WHO_AM_I_G[1] = { 0x0F | 0x80 }; /* Gyro WHO_AM_I register address*/

uint8_t CTRL_REG1_G[2] = { 0x20, 0x4F };
uint8_t CTRL_REG4_G[2] = { 0x23, 0xA0 };
uint8_t CTRL_REG1_XM[2] = { 0x20, 0x7F };
uint8_t CTRL_REG2_XM[2] = { 0x21, 0xC0 };
uint8_t CTRL_REG5_XM[2] = { 0x24, 0x94 };
uint8_t CTRL_REG6_XM[2] = { 0x25, 0x00 };
uint8_t CTRL_REG7_XM[2] = { 0x26, 0x80 };

namespace RODOS
{
extern HAL_UART uart_stdout;
}

#define TeleUART uart_stdout

struct SensorData
{
	double gyro_x, gyro_y, gyro_z;
	double acc_x, acc_y, acc_z;
	double mag_x, mag_y, mag_z;
	double pitchAccMag, rollAccMag, headingAccMag;
	double pitchGyro, rollGyro, headingGyro;
	double pitch, roll, heading;
	double temp;
};

struct TelecommandData
{
	char msg_id;
	char msg[11];
	char *msg_end;
};

Topic<SensorData> SensorDataTopic(-1, "Sensor Data");
CommBuffer<SensorData> SensorDataBuffer;
Subscriber SensorDataSubscriber(SensorDataTopic, SensorDataBuffer);

Topic<int> calibrationTopic(-1, "Calibration");
CommBuffer<int> calibrationBuffer;
Subscriber calibrationSubscriber(calibrationTopic, calibrationBuffer);

Topic<double> SignalTimeTopic(-1, "Signal Time");
CommBuffer<double> SignalTimeBuffer;
Subscriber SignalTimeSubscriber(SignalTimeTopic, SignalTimeBuffer);

Topic<double> TeleTimeTopic(-1, "Telemetry Time");
CommBuffer<double> TeleTimeBuffer;
Subscriber TeleTimeSubscriber(TeleTimeTopic, TeleTimeBuffer);

Topic<double> AlphaTopic(-1, "Gain alpha");
CommBuffer<double> AlphaBuffer;
Subscriber AlphaSubscriber(AlphaTopic, AlphaBuffer);

class SignalProc: public Thread
{
	double gyro_offx;
	double gyro_offy;
	double gyro_offz;
	double acc_offx;
	double acc_offy;
	double acc_offz;
	int16_t min_x, max_x, min_y, max_y, min_z, max_z;

	double pitchAccMag, rollAccMag, headingAccMag;
	double pitchGyro, rollGyro, headingGyro;
	double pitch, roll, heading;

	double T;

	double alpha;

	SensorData sd;

public:

	SignalProc(const char* name) :
			Thread(name)
	{
		gyro_offx = 0;
		gyro_offy = 0;
		gyro_offz = 0;
		acc_offx = 0;
		acc_offy = 0;
		acc_offz = 0;
		min_x = -INT16_MAX/2;
		max_x = INT16_MAX/2;
		min_y = -INT16_MAX/2;
		max_y = INT16_MAX/2;
		min_z = -INT16_MAX/2;
		max_z = INT16_MAX/2;

		pitchAccMag = 0;
		rollAccMag = 0;
		headingAccMag = 0;
		pitchGyro = 0;
		rollGyro = 0;
		headingGyro = 0;
		pitch = 0;
		roll = 0;
		heading = 0;

		T = 10;
		alpha = 0.9;
	}

	void init()
	{
		GreenLED.init(true, 1, 0);
		pa0.init(false, 1, 0);
		imu.init();
		imu_EN.init(true, 1, 1);
		cs_G.init(true, 1, 1);
		cs_XM.init(true, 1, 1);
	}

	void imuReset()
	{
		imu_EN.setPins(0);
		imu.reset();

		suspendCallerUntil(NOW()+ 10 * MILLISECONDS);

		imu.init(400000);
		imu_EN.setPins(1);
	}

	void imuInit()
	{
		uint8_t xff = 0xFF;

		cs_G.setPins(0);
		imu.write(CTRL_REG1_G, 2);
		cs_G.setPins(1);
		imu.write(&xff, 1);

		cs_G.setPins(0);
		imu.write(CTRL_REG4_G, 2);
		cs_G.setPins(1);

		cs_XM.setPins(0);
		imu.write(CTRL_REG1_XM, 2);
		cs_XM.setPins(1);
		imu.write(&xff, 1);

		cs_XM.setPins(0);
		imu.write(CTRL_REG2_XM, 2);
		cs_XM.setPins(1);
		imu.write(&xff, 1);

		cs_XM.setPins(0);
		imu.write(CTRL_REG5_XM, 2);
		cs_XM.setPins(1);
		imu.write(&xff, 1);

		cs_XM.setPins(0);
		imu.write(CTRL_REG6_XM, 2);
		cs_XM.setPins(1);
		imu.write(&xff, 1);

		cs_XM.setPins(0);
		imu.write(CTRL_REG7_XM, 2);
		cs_XM.setPins(1);

	}


	void read(HAL_GPIO &pin, uint8_t *targetregister, int16_t *dataarray, int amountofbits){
		pin.setPins(0);
		imu.write(targetregister, 1);
		imu.read((uint8_t*)dataarray , amountofbits);
		//PRINTF("Raw data X: %d, Y: %d, Z: %d \n", dataarray[0], dataarray[1], dataarray[2]);
		pin.setPins(1);

	}
	void calibrateGyro()
	{
		PRINTF("Gyro Calibration\nPut the board on even ground and press the blue button to calibrate.\n");
		while (pa0.readPins() != 1);
		PRINTF("Calibrating...\n");
		double value[3] = {0.0};
		int16_t data[3];
		for (int i = 0; i < 10000; i++)
		{
			read(cs_G, LSM9DS0_G_DATA, data, 6);
			//imu.writeRead(LSM9DS0_G, LSM9DS0_G_DATA, 1, (uint8_t*)data, 6);
			value[0] += DPS_DOUBLE(data[0]);
			value[1] += DPS_DOUBLE(data[1]);
			value[2] += DPS_DOUBLE(data[2]);
			suspendCallerUntil(NOW() + 100 * NANOSECONDS);
		}
		gyro_offx = value[0] / 10000;
		gyro_offy = value[1] / 10000;
		gyro_offz = value[2] / 10000;

		PRINTF("Calibration finished.\n");
		PRINTF("Offset X: %f, Y: %f, Z: %f \n\n", gyro_offx, gyro_offy, gyro_offz);

		pitch = 0;
		roll = 0;
		heading = 0;
	}

	void calibrateAcc()
	{
		double value[3] = {0.0};
		int16_t data[3];

		PRINTF("Acc Calibration\nLet the x-axis point to the sky, then push the blue button.\n");
		while (pa0.readPins() != 1);
		PRINTF("Calibrating...\n");
		for (int i = 0; i < 10000; i++)
		{
			read(cs_XM, LSM9DS0_A_DATA, data, 6);
			//imu.writeRead(LSM9DS0_XM, LSM9DS0_A_DATA, 1, (uint8_t*)data, 6);
			value[1] += ACC_DOUBLE(data[1]);
			value[2] += ACC_DOUBLE(data[2]);
			suspendCallerUntil(NOW() + 100 * NANOSECONDS);
		}

		PRINTF("Let the y-axis point to the sky, then push the blue button.\n");
		while (pa0.readPins() != 1);
		PRINTF("Calibrating...\n");
		for (int i = 0; i < 10000; i++)
		{
			read(cs_XM, LSM9DS0_A_DATA, data, 6);
			//imu.writeRead(LSM9DS0_XM, LSM9DS0_A_DATA, 1, (uint8_t*)data, 6);
			value[0] += ACC_DOUBLE(data[0]);
			value[2] += ACC_DOUBLE(data[2]);
			suspendCallerUntil(NOW() + 100 * NANOSECONDS);
		}

		PRINTF("Let the z-axis point to the sky, then push the blue button.\n");
		while (pa0.readPins() != 1);
		PRINTF("Calibrating...\n");
		for (int i = 0; i < 10000; i++)
		{
			read(cs_XM, LSM9DS0_A_DATA, data, 6);
			//imu.writeRead(LSM9DS0_XM, LSM9DS0_A_DATA, 1, (uint8_t*)data, 6);
			value[0] += ACC_DOUBLE(data[0]);
			value[1] += ACC_DOUBLE(data[1]);
			suspendCallerUntil(NOW() + 100 * NANOSECONDS);
		}


		acc_offx = value[0] / 20000;
		acc_offy = value[1] / 20000;
		acc_offz = value[2] / 20000;

		PRINTF("Calibration finished.\n");
		PRINTF("Offset X: %f, Y: %f, Z: %f \n\n", acc_offx, acc_offy, acc_offz);

		pitch = 0;
		roll = 0;
		heading = 0;
	}

	void calibrateMag()
	{
		min_x = INT16_MAX;
		max_x = 0;
		min_y = INT16_MAX;
		max_y = 0;
		min_z = INT16_MAX;
		max_z = 0;
		int16_t data[3];

		PRINTF("Mag Calibration\nPush the blue button, then rotate around the x-Axis for 3 seconds (at least two rotations).\n");
		while (pa0.readPins() != 1);
		PRINTF("Calibrating...\n");
		for (int i = 0; i < 30000; i++)
		{
			read(cs_XM, LSM9DS0_M_DATA, data, 6);
			//imu.writeRead(LSM9DS0_XM, LSM9DS0_M_DATA, 1, (uint8_t*)data, 6);
			if (data[0] < min_x) min_x = data[0];
			if (data[0] > max_x) max_x = data[0];
			if (data[1] < min_y) min_y = data[1];
			if (data[1] > max_y) max_y = data[1];
			if (data[2] < min_z) min_z = data[2];
			if (data[2] > max_z) max_z = data[2];
			suspendCallerUntil(NOW() + 100 * NANOSECONDS);
		}

		PRINTF("Push the blue button, then rotate around the y-Axis for 3 seconds (at least two rotations).\n");
		while (pa0.readPins() != 1);
		PRINTF("Calibrating...\n");
		for (int i = 0; i < 30000; i++)
		{
			read(cs_XM, LSM9DS0_M_DATA, data, 6);
			//imu.writeRead(LSM9DS0_XM, LSM9DS0_M_DATA, 1, (uint8_t*)data, 6);
			if (data[0] < min_x) min_x = data[0];
			if (data[0] > max_x) max_x = data[0];
			if (data[1] < min_y) min_y = data[1];
			if (data[1] > max_y) max_y = data[1];
			if (data[2] < min_z) min_z = data[2];
			if (data[2] > max_z) max_z = data[2];
			suspendCallerUntil(NOW() + 100 * NANOSECONDS);
		}

		PRINTF("Push the blue button, then rotate around the z-Axis for 3 seconds (at least two rotations).\n");
		while (pa0.readPins() != 1);
		PRINTF("Calibrating...\n");
		for (int i = 0; i < 30000; i++)
		{
			read(cs_XM, LSM9DS0_M_DATA, data, 6);
			//imu.writeRead(LSM9DS0_XM, LSM9DS0_M_DATA, 1, (uint8_t*)data, 6);
			if (data[0] < min_x) min_x = data[0];
			if (data[0] > max_x) max_x = data[0];
			if (data[1] < min_y) min_y = data[1];
			if (data[1] > max_y) max_y = data[1];
			if (data[2] < min_z) min_z = data[2];
			if (data[2] > max_z) max_z = data[2];
			suspendCallerUntil(NOW() + 100 * NANOSECONDS);
		}

		PRINTF("Calibration finished.\n");
		PRINTF("MIN X: %f ", MAG_DOUBLE(min_x));
		PRINTF("MAX X: %f \n", MAG_DOUBLE(max_x));
		PRINTF("MIN Y: %f ", MAG_DOUBLE(min_y));
		PRINTF("MAX Y: %f \n", MAG_DOUBLE(max_y));
		PRINTF("MIN Z: %f ", MAG_DOUBLE(min_z));
		PRINTF("MAX Z: %f \n\n", MAG_DOUBLE(max_z));

		pitch = 0;
		roll = 0;
		heading = 0;
	}

	void calculateFilteredEulerAngles(double ax, double ay, double az, double mx, double my, double mz, double wx, double wy, double wz, double T, double alpha)
	{
		pitchAccMag = RAD2GRAD(atan2(ax, sqrt(ay*ay+az*az)));
		rollAccMag = RAD2GRAD(atan2(ay, sqrt(ax*ax+az*az)));

		double mxh = mx*cos(GRAD2RAD(pitch)) + mz*sin(GRAD2RAD(pitch));
		double myh = mx*sin(GRAD2RAD(roll))*sin(GRAD2RAD(pitch)) + my*cos(GRAD2RAD(roll)) - mz*sin(GRAD2RAD(roll))*cos(GRAD2RAD(pitch));

		headingAccMag = RAD2GRAD(atan2(myh, mxh));

		double dp = (cos(GRAD2RAD(roll))*cos(GRAD2RAD(pitch))*wy - sin(GRAD2RAD(roll))*cos(GRAD2RAD(pitch))*wz) / cos(GRAD2RAD(pitch));
		double dr = (cos(GRAD2RAD(pitch))*wx + sin(GRAD2RAD(roll))*sin(GRAD2RAD(pitch))*wy + cos(GRAD2RAD(roll))*sin(GRAD2RAD(pitch))*wz) / cos(GRAD2RAD(pitch));
		double dh = (sin(GRAD2RAD(roll))*wy + cos(GRAD2RAD(roll))*wz) / cos(GRAD2RAD(pitch));

		pitchGyro = fmod(pitch +  RAD2GRAD(dp) * T/1000, 360);
		rollGyro = fmod(roll + RAD2GRAD(dr) * T/1000, 360);
		headingGyro = fmod(heading + RAD2GRAD(dh) * T/1000, 360);

		pitch = alpha * pitchGyro + (1 - alpha) * pitchAccMag;
		roll = alpha * rollGyro + (1 - alpha) * rollAccMag;
		heading = alpha * headingGyro + (1 - alpha) * headingAccMag;
	}

	void run()
	{
		calibrationTopic.publishConst(0);
		SignalTimeTopic.publish(T);

		int16_t data[3];
		int16_t imuData[3];
		int16_t magData[3];
		int16_t tempData;
		imuInit();
		while (1)
		{
			GreenLED.setPins(~GreenLED.readPins());
			//imu.writeRead(LSM9DS0_G, LSM9DS0_WHO_AM_I_G, 1, data, 6);

			//calibrateGyro();
			int f;
			calibrationBuffer.get(f);
			if (f == 1)
			{
				calibrateGyro();
				calibrationTopic.publishConst(0.f);
			}
			else if(f == 2)
			{
				calibrateAcc();
				calibrationTopic.publishConst(0.f);
			}
			else if (f == 3)
			{
				calibrateMag();
				calibrationTopic.publishConst(0.f);
			}

			read(cs_G, LSM9DS0_G_DATA, data, 6);
			read(cs_XM, LSM9DS0_A_DATA, imuData, 6);
			read(cs_XM, LSM9DS0_M_DATA, magData, 6);
			read(cs_XM, LSM9DS0_T_DATA, &tempData, 6);

			//PRINTF("Raw data Gyro X: %f, Y: %f, Z: %f \n", DPS_DOUBLE(data[0]), DPS_DOUBLE(data[1]), DPS_DOUBLE(data[2]));
			//PRINTF("Raw data Acc X: %f, Y: %f, Z: %f \n", ACC_DOUBLE(imuData[0]), ACC_DOUBLE(imuData[1]), ACC_DOUBLE(imuData[2]));
			//PRINTF("Raw data Mag X: %f, Y: %f, Z: %f \n", MAG_NORMALIZE(magData[0], min_x, max_x), MAG_NORMALIZE(magData[1], min_y, max_y), MAG_NORMALIZE(magData[2], min_z, max_z));
			//imu.writeRead(LSM9DS0_G, LSM9DS0_G_DATA, 1, (uint8_t*)data, 6);
			//imu.writeRead(LSM9DS0_XM, LSM9DS0_A_DATA, 1, (uint8_t*)imuData, 6);
			//imu.writeRead(LSM9DS0_XM, LSM9DS0_M_DATA, 1, (uint8_t*)magData, 6);
			//imu.writeRead(LSM9DS0_XM, LSM9DS0_T_DATA, 1, (uint8_t*)&tempData, 2);

			tempData = (tempData & 0x800) ? (-1 ^ 0xFFF) | tempData : tempData;

			sd.gyro_x = DPS_DOUBLE(data[0]) - gyro_offx;
			sd.gyro_y = DPS_DOUBLE(data[1]) - gyro_offy;
			sd.gyro_z = DPS_DOUBLE(data[2]) - gyro_offz;

			sd.acc_x = ACC_DOUBLE(imuData[0]) - acc_offx;
			sd.acc_y = ACC_DOUBLE(imuData[1]) - acc_offy;
			sd.acc_z = ACC_DOUBLE(imuData[2]) - acc_offz;

			sd.mag_x = MAG_NORMALIZE(magData[0], min_x, max_x);
			sd.mag_y = MAG_NORMALIZE(magData[1], min_y, max_y);
			sd.mag_z = MAG_NORMALIZE(magData[2], min_z, max_z);

			sd.temp = TEMP_DOUBLE(tempData);

			calculateFilteredEulerAngles(sd.acc_x, sd.acc_y, sd.acc_z, sd.mag_x, sd.mag_y, sd.mag_z, sd.gyro_x, sd.gyro_y, sd.gyro_z, T, alpha);


			sd.pitchAccMag = pitchAccMag;
			sd.rollAccMag = rollAccMag;
			sd.headingAccMag = headingAccMag;
			sd.pitchGyro = pitchGyro;
			sd.rollGyro = rollGyro;
			sd.headingGyro = headingGyro;
			sd.pitch = pitch;
			sd.heading = heading;
			sd.roll = roll;

			SensorDataTopic.publish(sd);

			suspendCallerUntil(NOW()+T*MILLISECONDS);
		}
	}
};
SignalProc SignalProc("SignalProcessing");



class IMUTelemetry: public Thread
{
	SensorData sd;

	double timeToSeconds(uint64_t start, uint64_t end)
	{
		return (double)(end - start) / SECONDS;
	}

public:

	IMUTelemetry(const char* name) :
			Thread(name)
	{
	}

	void init()
	{
	}

	void run()
	{

		while (1)
			//WHILE (1) !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
		{
			int f;
			calibrationBuffer.get(f);
			if (f == 0.f)
			{
				SensorDataBuffer.get(sd);
				PRINTF("and the gyro data is X: %f, Y: %f, Z: %f \n", sd.gyro_x, sd.gyro_y, sd.gyro_z);
				PRINTF("and the acc data is X: %f, Y: %f, Z: %f \n", sd.acc_x, sd.acc_y, sd.acc_z);
				PRINTF("and the mag data is X: %f, Y: %f, Z: %f \n", sd.mag_x, sd.mag_y, sd.mag_z);
				PRINTF("and the temperature is: %f\n\n", sd.temp);
				PRINTF("Pitch, Roll, Heading AccMag: %f, %f, %f \n\n", sd.pitchAccMag, sd.rollAccMag, sd.headingAccMag);
				PRINTF("Pitch, Roll, Heading Gyro: %f, %f, %f \n\n", sd.pitchGyro, sd.rollGyro, sd.headingGyro);
				PRINTF("Pitch, Roll, Heading Filtered: %f, %f, %f \n\n", sd.pitch, sd.roll, sd.heading);
			}

			suspendCallerUntil(NOW()+1000*MILLISECONDS);
		}
	}
};
IMUTelemetry IMUTelemetry("IMUTelemetry");



class IMUTelecommand: public Thread
{
	TelecommandData data;

	enum states
	{
		MSG_START, MSG_ID, MSG_DATA, MSG_END, MSG_ERROR, MSG_TIMEOUT
	} state;

public:

	IMUTelecommand(const char* name) :
			Thread(name)
	{
		state = MSG_START;
	}

	void init()
	{
	}

	bool decodeMessage()
	{
		int f;
		sscanf(data.msg, "%d", &f);
		switch (data.msg_id)
		{
		case 'c': case 'C':
			if (f == 1 || f == 2 || f == 3)
				calibrationTopic.publish(f);
			else
				return false;
			return true;
		default:
			return false;
		}
	}

	void run()
	{

		while (1)
		{

			TeleUART.suspendUntilDataReady();

			int current = TeleUART.getcharNoWait();

			if (state == MSG_START)
			{
				if (current == '$')
					state = MSG_ID;
				//else state = MSG_ERROR;
			}

			while (state == MSG_ID || state == MSG_DATA)
			{
				TeleUART.suspendUntilDataReady(NOW()+ 5000 * MILLISECONDS);
				current = TeleUART.getcharNoWait();
				if (current == -1)
					state = MSG_TIMEOUT;

				if (state == MSG_ID)
				{
					data.msg_id = current;
					state = MSG_DATA;
					data.msg_end = data.msg;
				}
				else if (state == MSG_DATA)
				{
					if (current == '#')
					{
						*data.msg_end = '\0';
						state = MSG_END;
					}
					else
					{
						if (data.msg_end - data.msg < 10)
						{
							*data.msg_end = current;
							data.msg_end++;
						}
						else
						{
							state = MSG_ERROR;
						}
					}
				}
			}

			if (state == MSG_END)
			{
				if (decodeMessage())
					state = MSG_START;
				else
					state = MSG_ERROR;

			}

			if (state == MSG_ERROR)
			{
				PRINTF("Message Corrupted\n");
				state = MSG_START;
			}
			else if (state == MSG_TIMEOUT)
			{
				PRINTF("Message Timeout\n");
				state = MSG_START;
			}
		}
	}
};
IMUTelecommand IMUTelecommand("IMUTelecommand");
