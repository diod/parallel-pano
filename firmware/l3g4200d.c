#define GYRO_CTRL_REG1       0x20
#define GYRO_CTRL_REG2       0x21
#define GYRO_CTRL_REG1_PD    (1 << 3)

#define GYRO_CTRL_REG4       0x23

#define GYRO_OUT_X           0x28
#define GYRO_OUT_Y           0x2A
#define GYRO_OUT_Z           0x2C

#define GYRO_ADR_FS_250     0x00
#define GYRO_ADR_FS_500     0x10
#define GYRO_ADR_FS_2000    0x20

#define GYRO_SENS_FS_250        0.00875
#define GYRO_SENS_FS_500        0.0175
#define GYRO_SENS_FS_2000       0.07

#define GYRO_RANGE_250 0
#define GYRO_RANGE_500 2
#define GYRO_RANGE_2000 3


#define GYRO_STATUS_REG 0x27
#define GYRO_ZOR_REG 0b01000000
#define GYRO_ZDA_REG 0b00000100

void gyro_setRange(uint8_t range);


uint8_t _gyro_addr;
float   _gyro_mult;


void gyro_init(uint8_t addr) 
{
    _gyro_addr = addr;

    uint8_t magick = 0;
    twi_read_byte(addr, 0x0F, &magick);
    if (magick!=0xd3) {
      printf("giro_init: failed to detect l3g4200d\n");
      return;
    }


    twi_write_byte(addr, GYRO_CTRL_REG1, 0xF);     //_ctrlReg1 = 0x3; // default according to datasheet
    twi_write_byte(addr, GYRO_CTRL_REG2, 0);
    gyro_setRange(GYRO_RANGE_250);
}

void gyro_setRange(uint8_t range)
{
    uint8_t _ctrlReg4;
    
    switch (range) {
        case GYRO_RANGE_250: {
            _ctrlReg4 = GYRO_ADR_FS_250;
            _gyro_mult = GYRO_SENS_FS_250;
            break;
        }
        case GYRO_RANGE_500: {
            _ctrlReg4 = GYRO_ADR_FS_500;
            _gyro_mult = GYRO_SENS_FS_500;
            break;
        }
        case GYRO_RANGE_2000: {
            _ctrlReg4 = GYRO_ADR_FS_2000;
            _gyro_mult = GYRO_SENS_FS_2000;
            break;
        }
        default: {
            _gyro_mult = GYRO_SENS_FS_250;    
            return;
        }
        break;
    }
    twi_write_byte(_gyro_addr, GYRO_CTRL_REG4, _ctrlReg4 | 0x80);
}
/*
float L3G4200D_TWI::readX_DegPerSec()
{
    return readX()*_mult;
}

float L3G4200D_TWI::readY_DegPerSec()
{
    return readY()*_mult;
}

float L3G4200D_TWI::readZ_DegPerSec()
{
    return readZ()*_mult;
}
*/
int16_t gyro_readAxis(uint8_t reg)
{
    uint8_t lowByte, highByte;
    twi_read_byte(_gyro_addr, reg+1, &highByte);
    twi_read_byte(_gyro_addr, reg, &lowByte);
    
    return ((int16_t)highByte << 8) | lowByte;
}

