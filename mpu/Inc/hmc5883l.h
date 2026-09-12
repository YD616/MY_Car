#ifndef __HMC5883L_H
#define __HMC5883L_H

#include "sys.h"

/* HMC5883L I2C address */
#define HMC5883L_ADDR               0x1E

/* HMC5883L Registers */
#define HMC5883L_CONFIG_A           0x00
#define HMC5883L_CONFIG_B           0x01
#define HMC5883L_MODE_REG           0x02
#define HMC5883L_DATA_X_H           0x03
#define HMC5883L_DATA_X_L           0x04
#define HMC5883L_DATA_Z_H           0x05
#define HMC5883L_DATA_Z_L           0x06
#define HMC5883L_DATA_Y_H           0x07
#define HMC5883L_DATA_Y_L           0x08
#define HMC5883L_STATUS_REG         0x09
#define HMC5883L_ID_A               0x0A
#define HMC5883L_ID_B               0x0B
#define HMC5883L_ID_C               0x0C

/* Config Register A bits */
#define MA_SAMPLE_AVG_1             (0x00 << 5)
#define MA_SAMPLE_AVG_2             (0x01 << 5)
#define MA_SAMPLE_AVG_4             (0x02 << 5)
#define MA_SAMPLE_AVG_8             (0x03 << 5)

#define DO_RATE_0_75HZ              (0x00 << 2)
#define DO_RATE_1_5HZ               (0x01 << 2)
#define DO_RATE_3HZ                 (0x02 << 2)
#define DO_RATE_7_5HZ               (0x03 << 2)
#define DO_RATE_15HZ                (0x04 << 2)
#define DO_RATE_30HZ                (0x05 << 2)
#define DO_RATE_75HZ                (0x06 << 2)

#define MA_MEASURE_NORMAL           0x00
#define MA_MEASURE_POS_BIAS         0x01
#define MA_MEASURE_NEG_BIAS         0x02

/* Config Register B gain */
#define GAIN_0_88GA                 (0x00 << 5)  /* 0.73 mG/LSB */
#define GAIN_1_3GA                  (0x01 << 5)  /* 0.92 mG/LSB */
#define GAIN_1_9GA                  (0x02 << 5)  /* 1.22 mG/LSB */
#define GAIN_2_5GA                  (0x03 << 5)  /* 1.52 mG/LSB */
#define GAIN_4_0GA                  (0x04 << 5)  /* 2.27 mG/LSB */
#define GAIN_4_7GA                  (0x05 << 5)  /* 2.56 mG/LSB */
#define GAIN_5_6GA                  (0x06 << 5)  /* 3.03 mG/LSB */
#define GAIN_8_1GA                  (0x07 << 5)  /* 4.35 mG/LSB */

/* Mode Register */
#define MODE_CONTINUOUS             0x00
#define MODE_SINGLE                 0x01
#define MODE_IDLE                   0x02

/* Function prototypes */
u8 HMC5883L_Init(void);
u8 HMC5883L_Read(short *mx, short *my, short *mz);
u8 HMC5883L_Read_ID(void);

#endif
