
#define LIS3MDL_TWI_ADDRESS 0b0011100

#define RANGE_4         0
#define RANGE_8         1
#define RANGE_12        2
#define RANGE_16        3

#define CTRL_REG1       0x20
#define CTRL_REG2       0x21
#define CTRL_REG3       0x22
#define CTRL_REG4       0x23
#define CTRL_REG5       0x24

#define OUT_X           0x28
#define OUT_Y           0x2A
#define OUT_Z           0x2C

#define ADR_FS_4        0x00
#define ADR_FS_8        0x20
#define ADR_FS_12       0x40
#define ADR_FS_16       0x60

#define SENS_FS_4       6842
#define SENS_FS_8       3421
#define SENS_FS_12      2281
#define SENS_FS_16      1711

uint8_t _compass_addr;
uint8_t _compass_ctrlReg1;
uint8_t _compass_ctrlReg2;
uint16_t _compass_mult;

void compass_setRange(uint8_t range);
void compass_writeCtrlReg2();

void compass_init(uint8_t addr) {
  _compass_addr = addr;
  _compass_ctrlReg1 = 0x0;
  
  twi_write_byte(addr, CTRL_REG2, 0x4);
  compass_setRange(RANGE_4);

  twi_write_byte(addr, CTRL_REG3, 0);
  twi_write_byte(addr, CTRL_REG1, 0x1C);
  
}

void compass_setRange(uint8_t range) {
  
  switch (range) {
        case RANGE_4: {
            _compass_ctrlReg2 = ADR_FS_4;
            _compass_mult = SENS_FS_4;
            break;
        }
        case RANGE_8: {
            _compass_ctrlReg2 = ADR_FS_8;
            _compass_mult = SENS_FS_8;
            break;
        }
        case RANGE_12: {
            _compass_ctrlReg2 = ADR_FS_12;
            _compass_mult = SENS_FS_12;
            break;
        }
        case RANGE_16: {
            _compass_ctrlReg2 = ADR_FS_16;
            _compass_mult = SENS_FS_16;
            break;
        }
        default: {
            _compass_mult = SENS_FS_4;    
        }
        break;
    }
    compass_writeCtrlReg2();
}

void compass_writeCtrlReg2() {
  twi_write_byte(_compass_addr, CTRL_REG2, _compass_ctrlReg2);
}

int16_t compass_readAxis(uint8_t reg)
{
    uint8_t lowByte, highByte;
    twi_read_byte(_compass_addr, reg, &lowByte);
    twi_read_byte(_compass_addr, reg+1, &highByte);

    return ((int16_t)highByte << 8) | lowByte;
}

float _compass_x_cal, _compass_y_cal, _compass_z_cal;

void compass_readXYZ_Calib() {

    _compass_x_cal = compass_readAxis(OUT_X);
    _compass_y_cal = compass_readAxis(OUT_Y);
    _compass_z_cal = compass_readAxis(OUT_Z);

    return;


    float calibration_matrix[3][3] = 
    {
        {2.446, 0.074, 0.006},
        {0.07, 2.317, -0.006},
        {-0.027, -0.12, 2.458}  
    };

    float bias[3] = 
    {
        3600,
        -2750,
        1500
    };

    float result[3] = {0, 0, 0};
    float uncalibrated_values[3];
    uncalibrated_values[0] = (float)compass_readAxis(OUT_X) - bias[0];
    uncalibrated_values[1] = (float)compass_readAxis(OUT_Y) - bias[1];
    uncalibrated_values[2] = (float)compass_readAxis(OUT_Z) - bias[2];

    uint8_t i,j;
    for (i = 0; i < 3; ++i) {
        for (j = 0; j < 3; ++j) {
        result[i] += calibration_matrix[i][j] * uncalibrated_values[j];
        }
    }

    _compass_x_cal = result[0];
    _compass_y_cal = result[1];
    _compass_z_cal = result[2];
}

float compass_readYaw()
{
    float heading = atan2(_compass_y_cal, _compass_x_cal);

    if(heading < 0)
    heading += 2*M_PI;

    if(heading > 2*M_PI)
    heading -= 2*M_PI;

    float headingDegrees = heading * 180/M_PI * 10;

    return headingDegrees;
}
