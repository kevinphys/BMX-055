// Wire Master Reader
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Reads data from an I2C/TWI slave device
// Refer to the "Wire Slave Sender" example for use with this

// Created 29 March 2006

// This example code is in the public domain.
#include <Wire.h>
#include <BMX055_headmouse.h>


// BMX055 data sheet http://ae-bst.resource.bosch.com/media/products/dokumente/bmx055/BST-BMX055-DS000-01v2.pdf
// The BMX055 is a conglomeration of three separate motion sensors packaged together but
// addressed and communicated with separately by design
// Accelerometer registers
#define BMX055_ACC_WHOAMI        0x00   // should return 0xFA
//#define BMX055_ACC_Reserved    0x01
#define BMX055_ACC_D_X_LSB       0x02
#define BMX055_ACC_D_X_MSB       0x03
#define BMX055_ACC_D_Y_LSB       0x04
#define BMX055_ACC_D_Y_MSB       0x05
#define BMX055_ACC_D_Z_LSB       0x06
#define BMX055_ACC_D_Z_MSB       0x07
#define BMX055_ACC_D_TEMP        0x08
#define BMX055_ACC_INT_STATUS_0  0x09
#define BMX055_ACC_INT_STATUS_1  0x0A
#define BMX055_ACC_INT_STATUS_2  0x0B
#define BMX055_ACC_INT_STATUS_3  0x0C
//#define BMX055_ACC_Reserved    0x0D
#define BMX055_ACC_FIFO_STATUS   0x0E
#define BMX055_ACC_PMU_RANGE     0x0F
#define BMX055_ACC_PMU_BW        0x10
#define BMX055_ACC_PMU_LPW       0x11
#define BMX055_ACC_PMU_LOW_POWER 0x12
#define BMX055_ACC_D_HBW         0x13
#define BMX055_ACC_BGW_SOFTRESET 0x14
//#define BMX055_ACC_Reserved    0x15
#define BMX055_ACC_INT_EN_0      0x16
#define BMX055_ACC_INT_EN_1      0x17
#define BMX055_ACC_INT_EN_2      0x18
#define BMX055_ACC_INT_MAP_0     0x19
#define BMX055_ACC_INT_MAP_1     0x1A
#define BMX055_ACC_INT_MAP_2     0x1B
//#define BMX055_ACC_Reserved    0x1C
//#define BMX055_ACC_Reserved    0x1D
#define BMX055_ACC_INT_SRC       0x1E
//#define BMX055_ACC_Reserved    0x1F
#define BMX055_ACC_INT_OUT_CTRL  0x20
#define BMX055_ACC_INT_RST_LATCH 0x21
#define BMX055_ACC_INT_0         0x22
#define BMX055_ACC_INT_1         0x23
#define BMX055_ACC_INT_2         0x24
#define BMX055_ACC_INT_3         0x25
#define BMX055_ACC_INT_4         0x26
#define BMX055_ACC_INT_5         0x27
#define BMX055_ACC_INT_6         0x28
#define BMX055_ACC_INT_7         0x29
#define BMX055_ACC_INT_8         0x2A
#define BMX055_ACC_INT_9         0x2B
#define BMX055_ACC_INT_A         0x2C
#define BMX055_ACC_INT_B         0x2D
#define BMX055_ACC_INT_C         0x2E
#define BMX055_ACC_INT_D         0x2F
#define BMX055_ACC_FIFO_CONFIG_0 0x30
//#define BMX055_ACC_Reserved    0x31
#define BMX055_ACC_PMU_SELF_TEST 0x32
#define BMX055_ACC_TRIM_NVM_CTRL 0x33
#define BMX055_ACC_BGW_SPI3_WDT  0x34
//#define BMX055_ACC_Reserved    0x35
#define BMX055_ACC_OFC_CTRL      0x36
#define BMX055_ACC_OFC_SETTING   0x37
#define BMX055_ACC_OFC_OFFSET_X  0x38
#define BMX055_ACC_OFC_OFFSET_Y  0x39
#define BMX055_ACC_OFC_OFFSET_Z  0x3A
#define BMX055_ACC_TRIM_GPO      0x3B
#define BMX055_ACC_TRIM_GP1      0x3C
//#define BMX055_ACC_Reserved    0x3D
#define BMX055_ACC_FIFO_CONFIG_1 0x3E
#define BMX055_ACC_FIFO_DATA     0x3F

// BMX055 Gyroscope Registers
#define BMX055_GYRO_WHOAMI           0x00  // should return 0x0F
//#define BMX055_GYRO_Reserved       0x01
#define BMX055_GYRO_RATE_X_LSB       0x02
#define BMX055_GYRO_RATE_X_MSB       0x03
#define BMX055_GYRO_RATE_Y_LSB       0x04
#define BMX055_GYRO_RATE_Y_MSB       0x05
#define BMX055_GYRO_RATE_Z_LSB       0x06
#define BMX055_GYRO_RATE_Z_MSB       0x07
//#define BMX055_GYRO_Reserved       0x08
#define BMX055_GYRO_INT_STATUS_0  0x09
#define BMX055_GYRO_INT_STATUS_1  0x0A
#define BMX055_GYRO_INT_STATUS_2  0x0B
#define BMX055_GYRO_INT_STATUS_3  0x0C
//#define BMX055_GYRO_Reserved    0x0D
#define BMX055_GYRO_FIFO_STATUS   0x0E
#define BMX055_GYRO_RANGE         0x0F
#define BMX055_GYRO_BW            0x10
#define BMX055_GYRO_LPM1          0x11
#define BMX055_GYRO_LPM2          0x12
#define BMX055_GYRO_RATE_HBW      0x13
#define BMX055_GYRO_BGW_SOFTRESET 0x14
#define BMX055_GYRO_INT_EN_0      0x15
#define BMX055_GYRO_INT_EN_1      0x16
#define BMX055_GYRO_INT_MAP_0     0x17
#define BMX055_GYRO_INT_MAP_1     0x18
#define BMX055_GYRO_INT_MAP_2     0x19
#define BMX055_GYRO_INT_SRC_1     0x1A
#define BMX055_GYRO_INT_SRC_2     0x1B
#define BMX055_GYRO_INT_SRC_3     0x1C
//#define BMX055_GYRO_Reserved    0x1D
#define BMX055_GYRO_FIFO_EN       0x1E
//#define BMX055_GYRO_Reserved    0x1F
//#define BMX055_GYRO_Reserved    0x20
#define BMX055_GYRO_INT_RST_LATCH 0x21
#define BMX055_GYRO_HIGH_TH_X     0x22
#define BMX055_GYRO_HIGH_DUR_X    0x23
#define BMX055_GYRO_HIGH_TH_Y     0x24
#define BMX055_GYRO_HIGH_DUR_Y    0x25
#define BMX055_GYRO_HIGH_TH_Z     0x26
#define BMX055_GYRO_HIGH_DUR_Z    0x27
//#define BMX055_GYRO_Reserved    0x28
//#define BMX055_GYRO_Reserved    0x29
//#define BMX055_GYRO_Reserved    0x2A
#define BMX055_GYRO_SOC           0x31
#define BMX055_GYRO_A_FOC         0x32
#define BMX055_GYRO_TRIM_NVM_CTRL 0x33
#define BMX055_GYRO_BGW_SPI3_WDT  0x34
//#define BMX055_GYRO_Reserved    0x35
#define BMX055_GYRO_OFC1          0x36
#define BMX055_GYRO_OFC2          0x37
#define BMX055_GYRO_OFC3          0x38
#define BMX055_GYRO_OFC4          0x39
#define BMX055_GYRO_TRIM_GP0      0x3A
#define BMX055_GYRO_TRIM_GP1      0x3B
#define BMX055_GYRO_BIST          0x3C
#define BMX055_GYRO_FIFO_CONFIG_0 0x3D
#define BMX055_GYRO_FIFO_CONFIG_1 0x3E

// BMX055 magnetometer registers
#define BMX055_MAG_WHOAMI         0x40  // should return 0x32
#define BMX055_MAG_Reserved       0x41
#define BMX055_MAG_XOUT_LSB       0x42
#define BMX055_MAG_XOUT_MSB       0x43
#define BMX055_MAG_YOUT_LSB       0x44
#define BMX055_MAG_YOUT_MSB       0x45
#define BMX055_MAG_ZOUT_LSB       0x46
#define BMX055_MAG_ZOUT_MSB       0x47
#define BMX055_MAG_ROUT_LSB       0x48
#define BMX055_MAG_ROUT_MSB       0x49
#define BMX055_MAG_INT_STATUS     0x4A
#define BMX055_MAG_PWR_CNTL1      0x4B
#define BMX055_MAG_PWR_CNTL2      0x4C
#define BMX055_MAG_INT_EN_1       0x4D
#define BMX055_MAG_INT_EN_2       0x4E
#define BMX055_MAG_LOW_THS        0x4F
#define BMX055_MAG_HIGH_THS       0x50
#define BMX055_MAG_REP_XY         0x51
#define BMX055_MAG_REP_Z          0x52
/* Trim Extended Registers */
#define BMM050_DIG_X1             0x5D // needed for magnetic field calculation
#define BMM050_DIG_Y1             0x5E
#define BMM050_DIG_Z4_LSB         0x62
#define BMM050_DIG_Z4_MSB         0x63
#define BMM050_DIG_X2             0x64
#define BMM050_DIG_Y2             0x65
#define BMM050_DIG_Z2_LSB         0x68
#define BMM050_DIG_Z2_MSB         0x69
#define BMM050_DIG_Z1_LSB         0x6A
#define BMM050_DIG_Z1_MSB         0x6B
#define BMM050_DIG_XYZ1_LSB       0x6C
#define BMM050_DIG_XYZ1_MSB       0x6D
#define BMM050_DIG_Z3_LSB         0x6E
#define BMM050_DIG_Z3_MSB         0x6F
#define BMM050_DIG_XY2            0x70
#define BMM050_DIG_XY1            0x71

// Using the Teensy Mini Add-On board, SDO1 = SDO2 = CSB3 = GND as designed
// Seven-bit device addresses are ACC = 0x18, GYRO = 0x68, MAG = 0x10
#define BMX055_ACC_ADDRESS  0x18   // Address of BMX055 accelerometer
#define BMX055_GYRO_ADDRESS 0x68   // Address of BMX055 gyroscope
#define BMX055_MAG_ADDRESS  0x10   // Address of BMX055 magnetometer
#define MS5637_ADDRESS      0x76   // Address of altimeter

int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[4];    // Stores the 16-bit signed magnetometer sensor output

float ax, ay, az, gx, gy, gz, mx, my, mz;


// Set initial input parameters
// define X055 ACC full scale options
#define AFS_2G  0x03
#define AFS_4G  0x05
#define AFS_8G  0x08
#define AFS_16G 0x0B



enum ACCBW {    // define BMX055 accelerometer bandwidths
  ABW_8Hz,      // 7.81 Hz,  64 ms update time
  ABW_16Hz,     // 15.63 Hz, 32 ms update time
  ABW_31Hz,     // 31.25 Hz, 16 ms update time
  ABW_63Hz,     // 62.5  Hz,  8 ms update time
  ABW_125Hz,    // 125   Hz,  4 ms update time
  ABW_250Hz,    // 250   Hz,  2 ms update time
  ABW_500Hz,    // 500   Hz,  1 ms update time
  ABW_100Hz     // 1000  Hz,  0.5 ms update time
};

// Specify sensor full scale
//uint8_t Gscale = GFS_125DPS;       // set gyro full scale to highest resolution +/-125 degrees/second
//uint8_t GODRBW = G_200Hz23Hz;      // set gyro ODR to 200 Hz and lowest bandwidth od 23 Hz
uint8_t Ascale = AFS_2G;           // set accel full scale to highest resolution +/-2 gs
uint8_t ACCBW  = 0x80 & ABW_8Hz;   // Choose a 8 Hz bandwidth for a 200 Hz ACC ODR
//uint8_t Mmode  = lowPower;         // Choose either 14-bit or 16-bit magnetometer resolution
//uint8_t MODR   = MODR_10Hz;        // set magnetometer data rate to 20 Hz
float aRes, gRes, mRes;            // scale resolutions per LSB for the sensors







void setup()
{
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
  Serial.println("Type something to send:");
  
  getAres();
}

void loop()
{
  writeByte(0x10, 0x4B, 0x01);

  readAccelData(accelCount);  // Read the x/y/z adc values
  
  ax = (float)accelCount[0]*aRes; // - accelBias[0];  // get actual g value, this depends on scale being set
  ay = (float)accelCount[1]*aRes; // - accelBias[1];   
  az = (float)accelCount[2]*aRes; // - accelBias[2]; 
  //Wire.beginTransmission(0x18); 
  
  //Wire.requestFrom(0x18, 6);    // request 6 bytes from slave device #2

  //Wire.write(0x18);
  //Wire.endTransmission(false);


  uint8_t c = readByte(0x10, 0x40); // receive a byte as character
  Serial.println(ax);  // print the character
  Serial.println(ay);
  Serial.println(az);
  Serial.println("---");

  delay(500);
}





void initBMX055()
{  
   // start with all sensors in default mode with all registers reset
//   writeByte(BMX055_ACC_ADDRESS,  BMX055_ACC_BGW_SOFTRESET, 0xB6);  // reset accelerometer
//   delay(1000); // Wait for all registers to reset 

   // Configure accelerometer
   writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_PMU_RANGE, Ascale & 0x0F); // Set accelerometer full range
   writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_PMU_BW, ACCBW & 0x0F);     // Set accelerometer bandwidth
   writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_D_HBW, 0x00);              // Use filtered data

//   writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_INT_EN_1, 0x10);           // Enable ACC data ready interrupt
//   writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_INT_OUT_CTRL, 0x04);       // Set interrupts push-pull, active high for INT1 and INT2
//   writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_INT_MAP_1, 0x02);        // Define INT1 (intACC1) as ACC data ready interrupt
//   writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_INT_MAP_1, 0x80);          // Define INT2 (intACC2) as ACC data ready interrupt

//   writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_BGW_SPI3_WDT, 0x06);       // Set watchdog timer for 50 ms
 
 // Configure Gyro
 // start by resetting gyro, better not since it ends up in sleep mode?!
// writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_BGW_SOFTRESET, 0xB6); // reset gyro
// delay(100);
 // Three power modes, 0x00 Normal, 
 // set bit 7 to 1 for suspend mode, set bit 5 to 1 for deep suspend mode
 // sleep duration in fast-power up from suspend mode is set by bits 1 - 3
 // 000 for 2 ms, 111 for 20 ms, etc.
//  writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_LPM1, 0x00);  // set GYRO normal mode
//  set GYRO sleep duration for fast power-up mode to 20 ms, for duty cycle of 50%
//  writeByte(BMX055_ACC_ADDRESS, BMX055_GYRO_LPM1, 0x0E);  
 // set bit 7 to 1 for fast-power-up mode,  gyro goes quickly to normal mode upon wake up
// can set external wake-up interrupts on bits 5 and 4
// auto-sleep wake duration set in bits 2-0, 001 4 ms, 111 40 ms
//  writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_LPM2, 0x00);  // set GYRO normal mode
// set gyro to fast wake up mode, will sleep for 20 ms then run normally for 20 ms 
// and collect data for an effective ODR of 50 Hz, other duty cycles are possible but there
// is a minimum wake duration determined by the bandwidth duration, e.g.,  > 10 ms for 23Hz gyro bandwidth
//  writeByte(BMX055_ACC_ADDRESS, BMX055_GYRO_LPM2, 0x87);   

 //writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_RANGE, Gscale);  // set GYRO FS range
 //writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_BW, GODRBW);     // set GYRO ODR and Bandwidth

// writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_INT_EN_0, 0x80);  // enable data ready interrupt
// writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_INT_EN_1, 0x04);  // select push-pull, active high interrupts
// writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_INT_MAP_1, 0x80); // select INT3 (intGYRO1) as GYRO data ready interrupt 

// writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_BGW_SPI3_WDT, 0x06); // Enable watchdog timer for I2C with 50 ms window
/*

// Configure magnetometer 
writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL1, 0x82);  // Softreset magnetometer, ends up in sleep mode
delay(100);
writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL1, 0x01); // Wake up magnetometer
delay(100);

writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL2, MODR << 3); // Normal mode
//writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL2, MODR << 3 | 0x02); // Forced mode

writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_INT_EN_2, 0x84); // Enable data ready pin interrupt, active high

// Set up four standard configurations for the magnetometer
  switch (Mmode)
  {
    case lowPower:
         // Low-power
          writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_XY, 0x01);  // 3 repetitions (oversampling)
          writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_Z,  0x02);  // 3 repetitions (oversampling)
          break;
    case Regular:
          // Regular
          writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_XY, 0x04);  //  9 repetitions (oversampling)
          writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_Z,  0x16);  // 15 repetitions (oversampling)
          break;
    case enhancedRegular:
          // Enhanced Regular
          writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_XY, 0x07);  // 15 repetitions (oversampling)
          writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_Z,  0x22);  // 27 repetitions (oversampling)
          break;
    case highAccuracy:
          // High Accuracy
          writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_XY, 0x17);  // 47 repetitions (oversampling)
          writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_Z,  0x51);  // 83 repetitions (oversampling)
          break;
  }

*/

}


void getAres() {
  switch (Ascale)
  {
 	// Possible accelerometer scales (and their register bit settings) are:
	// 2 Gs (0011), 4 Gs (0101), 8 Gs (1000), and 16 Gs  (1100). 
        // BMX055 ACC data is signed 12 bit
    case AFS_2G:
          aRes = 2.0/2048.0;
          break;
    case AFS_4G:
          aRes = 4.0/2048.0;
          break;
    case AFS_8G:
          aRes = 8.0/2048.0;
          break;
    case AFS_16G:
          aRes = 16.0/2048.0;
          break;
  }
}




void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(BMX055_ACC_ADDRESS, BMX055_ACC_D_X_LSB, 6, &rawData[0]);       // Read the six raw data registers into data array
  if ((rawData[0] & 0x01) && (rawData[2] & 0x01) && (rawData[4] & 0x01)) { // Check that all 3 axes have new data
    destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]) >> 4;           // Turn the MSB and LSB into a signed 12-bit value
    destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]) >> 4;
    destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]) >> 4;
  }
}








void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}


uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data; // `data` will store the register data
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);	                 // Put slave register address in Tx buffer
  Wire.endTransmission();        // Send the Tx buffer, but send a restart to keep connection alive
  //	Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  //	Wire.requestFrom(address, 1);  // Read one byte from slave register address
  Wire.requestFrom(address, (size_t) 1);   // Read one byte from slave register address
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission();            // Send the Tx buffer, but send a restart to keep connection alive
  //	Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  //        Wire.requestFrom(address, count);  // Read bytes from slave register address
  Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address
  while (Wire.available()) {
    dest[i++] = Wire.read();
  }         // Put read results in the Rx buffer
}
