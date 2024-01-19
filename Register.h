
#define BMP_ADDR 0x77

#define BMP085_ULTRALOWPOWER 0 //!< Ultra low power mode
#define BMP085_STANDARD 1      //!< Standard mode
#define BMP085_HIGHRES 2       //!< High-res mode
#define BMP085_ULTRAHIGHRES 3  //!< Ultra high-res mode
#define BMP085_CAL_AC1 0xAA    //!< R   Calibration data (16 bits)
#define BMP085_CAL_AC2 0xAC    //!< R   Calibration data (16 bits)
#define BMP085_CAL_AC3 0xAE    //!< R   Calibration data (16 bits)
#define BMP085_CAL_AC4 0xB0    //!< R   Calibration data (16 bits)
#define BMP085_CAL_AC5 0xB2    //!< R   Calibration data (16 bits)
#define BMP085_CAL_AC6 0xB4    //!< R   Calibration data (16 bits)
#define BMP085_CAL_B1 0xB6     //!< R   Calibration data (16 bits)
#define BMP085_CAL_B2 0xB8     //!< R   Calibration data (16 bits)
#define BMP085_CAL_MB 0xBA     //!< R   Calibration data (16 bits)
#define BMP085_CAL_MC 0xBC     //!< R   Calibration data (16 bits)
#define BMP085_CAL_MD 0xBE     //!< R   Calibration data (16 bits)

#define BMP085_CONTROL 0xF4         //!< Control register
#define BMP085_TEMPDATA 0xF6        //!< Temperature data register
#define BMP085_PRESSUREDATA 0xF6    //!< Pressure data register
#define BMP085_READTEMPCMD 0x2E     //!< Read temperature control register value
#define BMP085_READPRESSURECMD 0x34 //!< Read pressure control register value



#define MPU_ADDR 0x68

#define MPU6050_SELF_TEST_X 0x0D ///< Self test factory calibrated values register
#define MPU6050_SELF_TEST_Y 0x0E ///< Self test factory calibrated values register
#define MPU6050_SELF_TEST_Z 0x0F ///< Self test factory calibrated values register
#define MPU6050_SELF_TEST_A 0x10 ///< Self test factory calibrated values register
#define MPU6050_SMPLRT_DIV 0x19  ///< sample rate divisor register
#define MPU6050_CONFIG 0x1A      ///< General configuration register
#define MPU6050_GYRO_CONFIG 0x1B ///< Gyro specfic configuration register
#define MPU6050_ACCEL_CONFIG 0x1C ///< Accelerometer specific configration register
#define MPU6050_INT_PIN_CONFIG 0x37 ///< Interrupt pin configuration register
#define MPU6050_INT_ENABLE 0x38     ///< Interrupt enable configuration register
#define MPU6050_INT_STATUS 0x3A     ///< Interrupt status register
#define MPU6050_WHO_AM_I 0x75       ///< Divice ID register
#define MPU6050_SIGNAL_PATH_RESET 0x68 ///< Signal path reset register
#define MPU6050_USER_CTRL 0x6A         ///< FIFO and I2C Master control register
#define MPU6050_PWR_MGMT_1 0x6B ///< Primary power/sleep control register
#define MPU6050_PWR_MGMT_2 0x6C ///< Secondary power/sleep control register
#define MPU6050_TEMP_H 0x41     ///< Temperature data high byte register
#define MPU6050_TEMP_L 0x42     ///< Temperature data low byte register
#define MPU6050_ACCEL_OUT 0x3B  ///< base address for sensor data reads
#define MPU6050_MOT_THR 0x1F    ///< Motion detection threshold bits [7:0]
#define MPU6050_MOT_DUR 0x20 ///< Duration counter threshold for motion int. 1 kHz rate, LSB = 1 ms

#define MPU6050_FSYNC_OUT_DISABLED 0
#define MPU6050_FSYNC_OUT_TEMP 1
#define MPU6050_FSYNC_OUT_GYROX 2
#define MPU6050_FSYNC_OUT_GYROY 3
#define MPU6050_FSYNC_OUT_GYROZ 4
#define MPU6050_FSYNC_OUT_ACCELX 5
#define MPU6050_FSYNC_OUT_ACCELY 6
#define MPU6050_FSYNC_OUT_ACCEL_Z 7

#define MPU6050_INTR_8MHz 0
#define MPU6050_PLL_GYROX 1
#define MPU6050_PLL_GYROY 2
#define MPU6050_PLL_GYROZ 3
#define MPU6050_PLL_EXT_32K 4
#define MPU6050_PLL_EXT_19MHz 5
#define MPU6050_STOP 7

#define MPU6050_RANGE_2_G 0b00  ///< +/- 2g (default value)
#define MPU6050_RANGE_4_G = 0b01  ///< +/- 4g
#define MPU6050_RANGE_8_G = 0b10  ///< +/- 8g
#define MPU6050_RANGE_16_G = 0b11 ///< +/- 16g

#define MPU6050_RANGE_250_DEG 0  ///< +/- 250 deg/s (default value)
#define MPU6050_RANGE_500_DEG 1  ///< +/- 500 deg/s
#define MPU6050_RANGE_1000_DEG 2 ///< +/- 1000 deg/s
#define MPU6050_RANGE_2000_DEG 3 ///< +/- 2000 deg/s

#define MPU6050_BAND_260_HZ 0 ///< Docs imply this disables the filter
#define MPU6050_BAND_184_HZ 1 ///< 184 Hz
#define MPU6050_BAND_94_HZ 2  ///< 94 Hz
#define MPU6050_BAND_44_HZ 3  ///< 44 Hz
#define MPU6050_BAND_21_HZ 4  ///< 21 Hz
#define MPU6050_BAND_10_HZ 5  ///< 10 Hz
#define MPU6050_BAND_5_HZ 6   ///< 5 Hz

#define MPU6050_HIGHPASS_DISABLE 0
#define MPU6050_HIGHPASS_5_HZ 1
#define MPU6050_HIGHPASS_2_5_HZ 2
#define MPU6050_HIGHPASS_1_25_HZ 3
#define MPU6050_HIGHPASS_0_63_HZ 4
#define MPU6050_HIGHPASS_UNUSED 5
#define MPU6050_HIGHPASS_HOLD 6

#define MPU6050_CYCLE_1_25_HZ 0 ///< 1.25 Hz
#define MPU6050_CYCLE_5_HZ 1    ///< 5 Hz
#define MPU6050_CYCLE_20_HZ 2  ///< 20 Hz
#define MPU6050_CYCLE_40_HZ 3  ///< 40 Hz
