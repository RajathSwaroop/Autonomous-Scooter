#include "LeddarM16.h"

LeddarM16::LeddarM16()
{
    kI2CBus = 0 ;           // Default I2C bus for Lidar-Lite on Jetson TK1
    error = 0 ;
}

LeddarM16::~LeddarM16()
{
    closeLeddar() ;
}

// Returns true if device file descriptor opens correctly, false otherwise

bool LeddarM16::openLeddar()
{
    char fileNameBuffer[32];
    sprintf(fileNameBuffer,"/dev/i2c-%d", kI2CBus);
    kI2CFileDescriptor = open("/dev/i2c-0", O_RDWR);
        
    if (kI2CFileDescriptor < 0) {
        // Could not open the file
        error = errno ;
    
        return false ;
    }
    if (ioctl(kI2CFileDescriptor, I2C_SLAVE, kLidarLiteI2CAddress) < 0) {
        // Could not open the device on the bus
        error = errno ;
    
        return false ;
    }
    
    return true ;
}

void Leddar::closeLeddar()
{
    if (kI2CFileDescriptor > 0) {
        close(kI2CFileDescriptor);
        // WARNING - This is not quite right, need to check for error first
        kI2CFileDescriptor = -1 ;
    }
}

// Read the given register on the Lidar-Lite
int Leddar::readLeddar(int readRegister)
{
    // Do not use i2c_smbus_read_byte_data here ; LidarLite V2 needs STOP between write and read
    int toReturn ;
    /*
    toReturn = i2c_smbus_write_byte(kI2CFileDescriptor, readRegister) ;
    if (toReturn < 0) {
        error = errno ;
        toReturn = -1 ;
    }
    */
    toReturn = i2c_smbus_read_byte(kI2CFileDescriptor) ;
    if (toReturn < 0) {
        error = errno ;
        toReturn = -1 ;
    }
    return toReturn ;
}

/*

// Write the the given value to the given register on the Lidar-Lite
int Leddar::writeLeddar(int writeRegister, int writeValue)
{
    int toReturn = i2c_smbus_write_byte_data(kI2CFileDescriptor, writeRegister, writeValue);
    // Wait a little bit to make sure it settles
    usleep(10000);
    if (toReturn < 0) {
        error = errno ;
        toReturn = -1 ;
    }
    return toReturn ;

}

*/

int Leddar::getValue()
{
	int reg = 0x75;
	int test = readLeddar(reg);
	return test;
}

/*
int Leddar::accel_config()
{  
                writeLidarLite(PWR_MGMT_1,0x80);

                writeLidarLite(PWR_MGMT_1,0x01);

                writeLidarLite(ACCEL_CONFIG_2,0x02);
                //Set it for 2G value
                writeLidarLite(ACCEL_CONFIG,0x00);
	
}
int LidarLite::accel_data()
{  
                  int         rxBuffer;
                  int8_t     accel_buffer[10];
                  int16_t temp;
                  int16_t Accel_x, Accel_y,Accel_z;
                  rxBuffer=readLidarLite(INT_STATUS);
                   if(rxBuffer & 0x0000001)
                   {
                     accel_buffer[0]=readLidarLite(ACCEL_XOUT_H);
                     accel_buffer[1]=readLidarLite(ACCEL_XOUT_L);
                     accel_buffer[2]=readLidarLite(ACCEL_YOUT_H);
                     accel_buffer[3]=readLidarLite(ACCEL_YOUT_L);
                     accel_buffer[4]=readLidarLite(ACCEL_ZOUT_H);
                     accel_buffer[5]=readLidarLite(ACCEL_ZOUT_L);
                    }
               temp = (int16_t)accel_buffer[0] << 8 | accel_buffer[1];
               Accel_x = temp;
	       printf("Accel_x is %d \t",Accel_x);
               temp = (int16_t)accel_buffer[2] << 8 | accel_buffer[3];
               Accel_y = temp;
	       printf("Accel_y is %d \t",Accel_y);
               temp = (int16_t)accel_buffer[4] << 8 | accel_buffer[5];
               Accel_z = temp;
               printf("Accel_z is %d \n",Accel_z);
	
}
// Return the current calculated distance in centimeters
int LidarLite::getDistance()
{
    int ioResult ;
    int msb, lsb ;
    ioResult = writeLidarLite(kLidarLiteCommandControlRegister,kLidarLiteMeasure);
    if (ioResult < 0) {
        return ioResult ;
    }
    ioResult = readLidarLite(kLidarLiteCalculateDistanceMSB);
    if (ioResult < 0) {
        return ioResult ;
    } else {
        msb = ioResult ;
    }
    ioResult = readLidarLite(kLidarLiteCalculateDistanceLSB);
    if (ioResult < 0) {
        return ioResult ;
    } else {
        lsb = ioResult ;
    }

    int distance = (msb << 8) + lsb ;

    return distance ;
}

// Return the previous measurement in centimeters
int LidarLite::getPreviousDistance() {

    int ioResult ;
    int msb, lsb ;
    ioResult = readLidarLite(kLidarLitePreviousMeasuredDistanceMSB);
    if (ioResult < 0) {
        return ioResult ;
    } else {
        msb = ioResult ;
    }
    ioResult = readLidarLite(kLidarLitePreviousMeasuredDistanceLSB);
    if (ioResult < 0) {
        return ioResult ;
    } else {
        lsb = ioResult ;
    }

    int distance = (msb << 8) + lsb ;

    return distance ;
}

// Return the velocity (rate of change) in centimeters; +/-
// Velocity is returned from the Lidar-Lite as an 8-bit 2's complement number
// The returned value is converted to a signed integer
int LidarLite::getVelocity()
{
    int ioResult = readLidarLite(kLidarLiteVelocityMeasurementOutput);
    if (ioResult == 255) {
        return 0 ;
    }
    if (ioResult > 127) {

        return  ioResult - 256 ;
    }
    return ioResult ;
}



// Return the Lidar-Lite hardware version
int LidarLite::getHardwareVersion()
{
    return readLidarLite(kLidarLiteHardwareVersion) ;
}

// Return the Lidar-Lite software version
int LidarLite::getSoftwareVersion() {
    return readLidarLite(kLidarLiteSoftwareVersion) ;
}
*/
// Return the last i/o error
int LidarLite::getError()
{
    return error ;
}
