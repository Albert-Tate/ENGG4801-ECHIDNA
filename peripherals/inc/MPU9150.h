/**********************************************************
 *  File: MPU9150.h
 *  Description: Header file for IMU, uses I2C
 *  Author: Albert Tate
 *  Date: 05/05/15
 **********************************************************
 */
 
#ifndef __MPU9150_H__
#define __MPU9150_H__

    #define MPU9150_TEMP_CONVERT(X) ((float)X/340.0 + 35)

    //I2C address depends on physical pin wiring (7 bits)
    //AD0 is pulled to ground
    #define MPU9150_ADDRESS 0b11010000
    
    #define MPU9150_ON LATDbits.LATD6 = 1
    #define MPU9150_OFF LATDbits.LATD6 = 0

    #define MPU9150_INIT_PIN TRISDbits.TRISD6 = 0
    
    //I2C LSB for command type
    #define MPU9150_WRITE 0x00
    #define MPU9150_READ 0x01
    

    void MPU9150_init(void);
    
    uint8_t MPU9150_read_byte(uint8_t addr);
    void MPU9150_write_byte(uint8_t addr, uint8_t byte);
    
    void MPU9150_read_buffer(uint8_t addr, uint8_t* buff, uint8_t len);
    void MPU9150_write_buffer(uint8_t addr, uint8_t* buff, uint8_t len);
    
    //Buffer must have enough memory for 3 16 bit numbers
    void MPU9150_read_ACC(int16_t* X, int16_t* Y, int16_t* Z);
    void MPU9150_read_GYRO(int16_t* X, int16_t* Y, int16_t* Z);
    int16_t MPU9150_read_TEMP(void); //Temp/340 + 35 to get actual value
    int16_t MPU9150_convert_TEMP(int16_t TEMP);
    
    void MPU9150_sleep(void);
    void MPU9150_wake(void);
    
    
    //Registers
    #define MPU9150_SELF_TEST_X     0x0D
    #define MPU9150_SELF_TEST_Y     0x0E
    #define MPU9150_SELF_TEST_Z     0x0F
    #define MPU9150_SELF_TEST_A     0x10
    #define MPU9150_SMPLRT_DIV      0x19
    #define MPU9150_CONFIG          0x1A
    #define MPU9150_GYRO_CONFIG     0x1B
    #define MPU9150_ACCEL_CONFIG    0x1C
    #define MPU9150_FIFO_EN         0x23
    #define MPU9150_I2C_MST_CTRL    0x24
    #define MPU9150_I2C_SLV0_ADDR   0x25
    #define MPU9150_I2C_SLV0_REG    0x26
    #define MPU9150_I2C_SLV0_CTRL   0x27
    #define MPU9150_I2C_SLV1_ADDR   0x28
    #define MPU9150_I2C_SLV1_REG    0x29
    #define MPU9150_I2C_SLV1_CTRL   0x2A
    #define MPU9150_I2C_SLV2_ADDR   0x2B
    #define MPU9150_I2C_SLV2_REG    0x2C
    #define MPU9150_I2C_SLV2_CTRL   0x2D
    #define MPU9150_I2C_SLV3_ADDR   0x2E
    #define MPU9150_I2C_SLV3_REG    0x2F
    #define MPU9150_I2C_SLV3_CTRL   0x30
    #define MPU9150_I2C_SLV4_ADDR   0x31
    #define MPU9150_I2C_SLV5_REG    0x32
    #define MPU9150_I2C_SLV4_DO     0x33
    #define MPU9150_I2C_SLV4_CTRL   0x34
    #define MPU9150_I2C_SLV4_DI     0x35    //R
    #define MPU9150_I2C_MST_STATUS  0x36    //R
    #define MPU9150_INT_PIN_CFG     0x37
    #define MPU9150_INT_ENABLE      0x38
    #define MPU9150_INT_STATUS      0x3A    //R
    
    #define MPU9150_ACCEL_XOUT_H    0x3B    //R
    #define MPU9150_ACCEL_XOUT_L    0x3C    //R
    #define MPU9150_ACCEL_YOUT_H    0x3D    //R
    #define MPU9150_ACCEL_YOUT_L    0x3E    //R
    #define MPU9150_ACCEL_ZOUT_H    0x3F    //R
    #define MPU9150_ACCEL_ZOUT_L    0x40    //R
    #define MPU9150_TEMP_OUT_H      0x41    //R
    #define MPU9150_TEMP_OUT_L      0x42    //R
    #define MPU9150_GYRO_XOUT_H     0x43    //R
    #define MPU9150_GYRO_XOUT_L     0x44    //R
    #define MPU9150_GYRO_YOUT_H     0x45    //R
    #define MPU9150_GYRO_YOUT_L     0x46    //R
    #define MPU9150_GYRO_ZOUT_H     0x47    //R
    #define MPU9150_GYRO_ZOUT_L     0x48    //R
    
    #define MPU9150_EXT_SENS_DATA_00 0x49   //R
    #define MPU9150_EXT_SENS_DATA_01 0x4A   //R
    #define MPU9150_EXT_SENS_DATA_02 0x4B   //R
    #define MPU9150_EXT_SENS_DATA_03 0x4C   //R
    #define MPU9150_EXT_SENS_DATA_04 0x4D   //R
    #define MPU9150_EXT_SENS_DATA_05 0x4E   //R
    #define MPU9150_EXT_SENS_DATA_06 0x4F   //R
    #define MPU9150_EXT_SENS_DATA_07 0x50   //R
    #define MPU9150_EXT_SENS_DATA_08 0x51   //R
    #define MPU9150_EXT_SENS_DATA_09 0x52   //R
    #define MPU9150_EXT_SENS_DATA_10 0x53   //R
    #define MPU9150_EXT_SENS_DATA_11 0x54   //R
    #define MPU9150_EXT_SENS_DATA_12 0x55   //R
    #define MPU9150_EXT_SENS_DATA_13 0x56   //R
    #define MPU9150_EXT_SENS_DATA_14 0x57   //R
    #define MPU9150_EXT_SENS_DATA_15 0x58   //R
    #define MPU9150_EXT_SENS_DATA_16 0x59   //R
    #define MPU9150_EXT_SENS_DATA_17 0x5A   //R
    #define MPU9150_EXT_SENS_DATA_18 0x5B   //R
    #define MPU9150_EXT_SENS_DATA_19 0x5C   //R
    #define MPU9150_EXT_SENS_DATA_20 0x5D   //R
    #define MPU9150_EXT_SENS_DATA_21 0x5E   //R
    #define MPU9150_EXT_SENS_DATA_22 0x5F   //R
    #define MPU9150_EXT_SENS_DATA_23 0x60   //R
    
    #define MPU9150_I2C_SLV0_DO     0x63
    #define MPU9150_I2C_SLV1_DO     0x64
    #define MPU9150_I2C_SLV2_DO     0x65
    #define MPU9150_I2C_SLV3_DO     0x66
    #define MPU9150_I2C_MST_DELAY_CTRL  0x67
    #define MPU9150_SIGNAL_PATH_RESET   0x68
    #define MPU9150_USER_CTRL       0x6A
    #define MPU9150_PWR_MGMT_1      0x6B
    #define MPU9150_PWR_MGMT_2      0x6C
    #define MPU9150_FIFO_COUNTH     0x72
    #define MPU9150_FIFO_COUNTL     0x73
    #define MPU9150_FIFO_R_W        0x74
    #define MPU9150_WHO_AM_I        0x75    //R

#endif