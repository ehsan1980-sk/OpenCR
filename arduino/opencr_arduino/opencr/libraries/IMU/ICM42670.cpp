#include <Arduino.h>
#include <SPI.h>
#include "ICM42670.h"




                            
#define ICM42670_ADDRESS    0x68
#define MPU_CALI_COUNT      512


#define ACC_ORIENTATION(X, Y, Z)  {accADC[PITCH]  =  Y; accADC[ROLL]  =  X; accADC[YAW]  =   Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[PITCH] =  Y; gyroADC[ROLL] =  X; gyroADC[YAW] =   Z;}







cICM42670::cICM42670()
{
	calibratingG = 0;
	calibratingA = 0;
  calibratingM = 0;
  bConnected   = false;
}

void cICM42670::selectBank(uint8_t bank)
{
}

void cICM42670::spiRead(uint16_t addr, uint8_t *p_data, uint32_t length)
{
  uint8_t reg_addr;
 
  reg_addr = (uint8_t) (addr & 0x7F);

  digitalWrite( BDPIN_SPI_CS_IMU, LOW);
 
  MPU_SPI.transfer(0x80 | reg_addr);
  MPU_SPI.transfer(NULL, (void *)p_data, (size_t)length);
 
  digitalWrite( BDPIN_SPI_CS_IMU, HIGH);
}

uint8_t cICM42670::spiReadByte(uint16_t addr)
{
  uint8_t data;

  spiRead(addr, &data, 1);

	return data;
}

void cICM42670::spiWriteByte(uint16_t addr, uint8_t data)
{
  uint8_t reg_addr;
 
  reg_addr = (uint8_t) (addr & 0x7F);
 
 
  digitalWrite( BDPIN_SPI_CS_IMU, LOW); 

  MPU_SPI.transfer(reg_addr & 0x7F);
  MPU_SPI.transfer(data);
 
  digitalWrite( BDPIN_SPI_CS_IMU, HIGH); 
}

bool cICM42670::begin()
{
  uint8_t data;

  pinMode( BDPIN_SPI_CS_IMU, OUTPUT );

  MPU_SPI.begin();
  MPU_SPI.setDataMode( SPI_MODE3 );
  MPU_SPI.setBitOrder( MSBFIRST );
  MPU_SPI.setClockDivider( SPI_CLOCK_DIV128 ); // 108Mhz/128 = 0.8MHz
  digitalWrite(BDPIN_SPI_CS_IMU, HIGH);
  delay( 100 );


  data = spiReadByte(ICM42670_REG_WHO_AM_I);

  if(data == ICM42670_WHO_AM_I)
  {
    bConnected = true;
    init();
    gyro_init();
    acc_init();
    mag_init();

    MPU_SPI.setClockDivider( SPI_CLOCK_DIV4 ); // 6.5MHz
  }



  return bConnected;
}

void cICM42670::init( void )
{
  uint8_t state;
  uint8_t data;
  uint8_t response[3] = {0, 0, 0};


  //ICM42670 Reset
  spiWriteByte(ICM42670_REG_SIGNAL_PATH_RESET, 0x10);
	delay(100);

	//ICM42670 Set Sensors
  data  = ICM42670_GYRO_ENABLE_LN_MODE  << ICM42670_GYRO_MODE_SHIFT;
  data |= ICM42670_ACCEL_ENABLE_LN_MODE << ICM42670_ACCEL_MODE_SHIFT;
  spiWriteByte(ICM42670_REG_PWR_MGMT0, data);
	delay(1);



	//ICM42670 Gyro 

  data  = ICM42670_GYRO_RANGE_2000DPS << ICM42670_GYRO_UI_FS_SEL_SHIFT;
  data |= ICM42670_GYRO_ODR_400HZ     << ICM42670_GYRO_ODR_SHIFT;
  spiWriteByte(ICM42670_REG_GYRO_CONFIG0, data);

  data = ICM42670_GYRO_LFP_53HZ << ICM42670_GYRO_UI_FILT_BW_SHIFT;
  spiWriteByte(ICM42670_REG_GYRO_CONFIG1, data);
	delay(1);


	//ICM42670 Accel
	
  data  = ICM42670_ACCEL_RANGE_2G  << ICM42670_ACCEL_UI_FS_SEL_SHIFT;
  data |= ICM42670_ACCEL_ODR_400HZ << ICM42670_ACCEL_ODR_SHIFT;
  spiWriteByte(ICM42670_REG_ACCEL_CONFIG0, data);

  data  = ICM42670_ACCEL_AVG_2X   << ICM42670_ACCEL_UI_AVG_SHIFT;
  data |= ICM42670_ACCEL_LFP_53HZ << ICM42670_ACCEL_UI_FILT_BW_SHIFT;
  spiWriteByte(ICM42670_REG_ACCEL_CONFIG1, data);
	delay(50);
}

void cICM42670::gyro_init( void )
{
	uint8_t i;


	for( i=0; i<3; i++ )
	{
    gyroADC[i]  = 0;
		gyroZero[i] = 0;
		gyroRAW[i]  = 0;
	}

	calibratingG = MPU_CALI_COUNT;
}

void cICM42670::gyro_get_adc( void )
{
	int16_t x = 0;
	int16_t y = 0;
	int16_t z = 0;

  uint8_t rawADC[6];

  if( bConnected == true )
  {
    spiRead(ICM42670_REG_GYRO_DATA_X1, &rawADC[0], 6);

 		x = (((int16_t)rawADC[0]) << 8) | rawADC[1];
  	y = (((int16_t)rawADC[2]) << 8) | rawADC[3];
  	z = (((int16_t)rawADC[4]) << 8) | rawADC[5];

  	gyroRAW[0] = x;
  	gyroRAW[1] = y;
  	gyroRAW[2] = z;

  	GYRO_ORIENTATION( x, y,z );
  }

  gyro_common();
}

void cICM42670::gyro_cali_start()
{
	calibratingG = MPU_CALI_COUNT;
}

void cICM42670::acc_init( void )
{
  uint8_t i;


  for( i=0; i<3; i++ )
  {
    accADC[i]   = 0;
		accZero[i]  = 0;
    accRAW[i]   = 0;
  }
}

void cICM42670::acc_get_adc( void )
{
	int16_t x = 0;
	int16_t y = 0;
	int16_t z = 0;
  uint8_t rawADC[6];

  if( bConnected == true )
  {    
    spiRead(ICM42670_REG_ACCEL_DATA_X1, &rawADC[0], 6);

    x = (((int16_t)rawADC[0]) << 8) | rawADC[1];
    y = (((int16_t)rawADC[2]) << 8) | rawADC[3];
    z = (((int16_t)rawADC[4]) << 8) | rawADC[5];

    accRAW[0] = x;
    accRAW[1] = y;
    accRAW[2] = z;
    
		ACC_ORIENTATION( x,	y, z );
	}

	acc_common();
}

void cICM42670::gyro_common()
{
	static int16_t previousGyroADC[3];
	static int32_t g[3];
	uint8_t axis, tilt=0;

	memset(previousGyroADC, 0, 3);

	if (calibratingG>0)
	{
		for (axis = 0; axis < 3; axis++)
		{
			if (calibratingG == MPU_CALI_COUNT)
			{ // Reset g[axis] at start of calibration
				g[axis]=0;
				previousGyroADC[axis] = gyroADC[axis];
			}
			if (calibratingG % 10 == 0)
			{
				previousGyroADC[axis] = gyroADC[axis];
			}
			g[axis] += gyroADC[axis]; // Sum up 512 readings
			gyroZero[axis]=g[axis]>>9;

			if (calibratingG == 1)
			{
			}
		}

		if(tilt)
		{
			calibratingG=1000;
		}
		else
		{
			calibratingG--;
		}
		return;
	}


  for (axis = 0; axis < 3; axis++)
  {
    gyroADC[axis] -= gyroZero[axis];
    previousGyroADC[axis] = gyroADC[axis];
  }
}

void cICM42670::acc_common()
{
	static int32_t a[3];

	if (calibratingA>0)
	{
		calibratingA--;
		for (uint8_t axis = 0; axis < 3; axis++)
		{
			if (calibratingA ==(MPU_CALI_COUNT-1)) a[axis]=0;  // Reset a[axis] at start of calibration
			a[axis] += accADC[axis];             // Sum up 512 readings
			accZero[axis] = a[axis]>>9;          // Calculate average, only the last itteration where (calibratingA == 0) is relevant
		}
		if (calibratingA == 0)
		{
      accZero[YAW] = 0;
		}
	}

  accADC[ROLL]  -=  accZero[ROLL] ;
  accADC[PITCH] -=  accZero[PITCH];
  accADC[YAW]   -=  accZero[YAW] ;
}

void cICM42670::mag_init( void )
{
  uint8_t i;

  for( i=0; i<3; i++ )
  {
    magADC[i]   = 0;
		magZero[i]  = 0;
    magRAW[i]   = 0;
  }
}

void cICM42670::mag_get_adc( void )
{
  return;
}

void cICM42670::mag_common()
{
  magADC[0] = magRAW[0];
  magADC[1] = magRAW[1];
  magADC[2] = magRAW[2];
}

void cICM42670::acc_cali_start()
{
	calibratingA = MPU_CALI_COUNT;
}

bool cICM42670::acc_cali_get_done()
{
	if( calibratingA == 0 ) return true;
	else                    return false;
}

bool cICM42670::gyro_cali_get_done()
{
	if( calibratingG == 0 ) return true;
	else                    return false;
}
