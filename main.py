from machine import I2C, Pin
import machine
from collections import OrderedDict
from math import pow, sqrt
import uctypes
import time

#led = machine.Pin(25, Pin.OUT)
#sclPin = machine.Pin(5, Pin.OPEN_DRAIN)
# sdaPin = machine.Pin(4,  Pin.OPEN_DRAIN)

sclPin = machine.Pin("I2C_SCL1", Pin.OPEN_DRAIN)
sdaPin = machine.Pin("I2C_SDA1",  Pin.OPEN_DRAIN)


i2c = machine.I2C(scl=sclPin,sda=sdaPin,freq=100000)
# i2c = machine.I2C(id=0,scl=sclPin,sda=sdaPin,freq=100000)


internal_temperature_union = {
    "UINT16": uctypes.UINT16,
    "INT16": uctypes.INT16,
}

raw_voltage_union = {
    "UINT32": uctypes.UINT32,
    "INT32": uctypes.INT32,
}

register_config_0 = {
    "PGA_BYPASS": uctypes.BFUINT8 | 0 | 0 << uctypes.BF_POS | 1 << uctypes.BF_LEN,
    "GAIN": uctypes.BFUINT8       | 0 | 1 << uctypes.BF_POS | 3 << uctypes.BF_LEN,
    "MUX": uctypes.BFUINT8        | 0 | 4 << uctypes.BF_POS | 4 << uctypes.BF_LEN,
    "all": uctypes.BFUINT8        | 0 | 0 << uctypes.BF_POS | 8 << uctypes.BF_LEN,
}


register_config_1 = {
    "TS": uctypes.BFUINT8 | 0 | 0 << uctypes.BF_POS | 1 << uctypes.BF_LEN,
    "VREF": uctypes.BFUINT8 | 0 | 1 << uctypes.BF_POS | 2 << uctypes.BF_LEN,
    "CM": uctypes.BFUINT8 | 0 | 3 << uctypes.BF_POS | 1 << uctypes.BF_LEN,
    "MODE": uctypes.BFUINT8 | 0 | 4 << uctypes.BF_POS | 1 << uctypes.BF_LEN,
    "DR": uctypes.BFUINT8 | 0 | 5 << uctypes.BF_POS | 3 << uctypes.BF_LEN,
    "all": uctypes.BFUINT8 | 0 | 0 << uctypes.BF_POS | 8 << uctypes.BF_LEN,
}

register_config_2 = {
    "IDAC": uctypes.BFUINT8     | 0 | 0 << uctypes.BF_POS | 3 << uctypes.BF_LEN,
    "BCS": uctypes.BFUINT8      | 0 | 3 << uctypes.BF_POS | 1 << uctypes.BF_LEN,
    "CRCbits": uctypes.BFUINT8  | 0 | 4 << uctypes.BF_POS | 2 << uctypes.BF_LEN,
    "DCNT": uctypes.BFUINT8     | 0 | 6 << uctypes.BF_POS | 1 << uctypes.BF_LEN,
    "DRDY": uctypes.BFUINT8     | 0 | 7 << uctypes.BF_POS | 1 << uctypes.BF_LEN,
    "all": uctypes.BFUINT8      | 0 | 0 << uctypes.BF_POS | 8 << uctypes.BF_LEN,
}

register_config_3 = {
    "RESERVED": uctypes.BFUINT8 | 0 | 0 << uctypes.BF_POS | 2 << uctypes.BF_LEN,
    "I2MUX": uctypes.BFUINT8    | 0 | 2 << uctypes.BF_POS | 3 << uctypes.BF_LEN,
    "I1MUX": uctypes.BFUINT8    | 0 | 5 << uctypes.BF_POS | 3 << uctypes.BF_LEN,
    "all": uctypes.BFUINT8      | 0 | 0 << uctypes.BF_POS | 8 << uctypes.BF_LEN,
}

print(i2c.scan())

a = bytearray([0])


### CONSTANTS
#### --- Configuration Register 0
ADS122C04_4WIRE_MODE         = 0x0
ADS122C04_3WIRE_MODE         = 0x1
ADS122C04_2WIRE_MODE         = 0x2
ADS122C04_TEMPERATURE_MODE   = 0x3
ADS122C04_RAW_MODE           = 0x4
ADS122C04_4WIRE_HI_TEMP      = 0x5
ADS122C04_3WIRE_HI_TEMP      = 0x6
ADS122C04_2WIRE_HI_TEMP      = 0x7

### COMMANDS
ADS122C04_RESET_CMD          = 0x06     #0000 011x      Reset
ADS122C04_START_CMD          = 0x08     #0000 100x      Start/Sync
ADS122C04_POWERDOWN_CMD      = 0x02     #0000 001x      PowerDown
ADS122C04_RDATA_CMD          = 0x10     #0001 xxxx      RDATA
ADS122C04_RREG_CMD           = 0x20     #0010 rrxx      Read REG rr= register address 00 to 11
ADS122C04_WREG_CMD           = 0x40     #0100 rrxx      Write REG rr= register address 00 to 11

### INPUT MUX CONFIG
ADS122C04_MUX_AIN0_AIN1      = 0x0
ADS122C04_MUX_AIN0_AIN2      = 0x1
ADS122C04_MUX_AIN0_AIN3      = 0x2
ADS122C04_MUX_AIN1_AIN0      = 0x3
ADS122C04_MUX_AIN1_AIN2      = 0x4
ADS122C04_MUX_AIN1_AIN3      = 0x5
ADS122C04_MUX_AIN2_AIN3      = 0x6
ADS122C04_MUX_AIN3_AIN2      = 0x7
ADS122C04_MUX_AIN0_AVSS      = 0x8
ADS122C04_MUX_AIN1_AVSS      = 0x9
ADS122C04_MUX_AIN2_AVSS      = 0xa
ADS122C04_MUX_AIN3_AVSS      = 0xb
ADS122C04_MUX_REFPmREFN      = 0xc
ADS122C04_MUX_AVDDmAVSS      = 0xd
ADS122C04_MUX_SHORTED        = 0xe

#### --- Configuration Register 1
### GAIN CONFIG
ADS122C04_GAIN_1             = 0x0
ADS122C04_GAIN_2             = 0x1
ADS122C04_GAIN_4             = 0x2
ADS122C04_GAIN_8             = 0x3
ADS122C04_GAIN_16            = 0x4
ADS122C04_GAIN_32            = 0x5
ADS122C04_GAIN_64            = 0x6
ADS122C04_GAIN_128           = 0x7

### PGA Bypass (PGA is disabled when the PGA_BYPASS bit is set)
ADS122C04_PGA_DISABLED       = 0x1
ADS122C04_PGA_ENABLED        = 0x0

### DATA RATE CONFIG
ADS122C04_DATA_RATE_20SPS    = 0x0
ADS122C04_DATA_RATE_45SPS    = 0x1
ADS122C04_DATA_RATE_90SPS    = 0x2
ADS122C04_DATA_RATE_175SPS   = 0x3
ADS122C04_DATA_RATE_330SPS   = 0x4
ADS122C04_DATA_RATE_600SPS   = 0x5
ADS122C04_DATA_RATE_1000SPS  = 0x6

### OPERATING MODE CONFIG
ADS122C04_OP_MODE_NORMAL     = 0x0
ADS122C04_OP_MODE_TURBO      = 0x1

### CONVERSION MODE CONFIG
ADS122C04_CONVERSION_MODE_SINGLE_SHOT  = 0x0
ADS122C04_CONVERSION_MODE_CONTINUOUS   = 0x1

### VOLTAGE REFERENCE SELECTION
ADS122C04_VREF_INTERNAL           = 0x0 # 2.048V internal
ADS122C04_VREF_EXT_REF_PINS       = 0x1 # REFp and REFn external
ADS122C04_VREF_AVDD               = 0x2 # Analog Supply AVDD and AVSS

### TEMP SENSOR MODE
ADS122C04_TEMP_SENSOR_OFF         = 0x0
ADS122C04_TEMP_SENSOR_ON          = 0x1

#### --- Configuration Register 2
### DATA COUNTER ENABLE
ADS122C04_DCNT_DISABLE            = 0x0
ADS122C04_DCNT_ENABLE             = 0x1

### DATA INTEGRITY CHECK
ADS122C04_CRC_DISABLED            = 0x0
ADS122C04_CRC_INVERTED            = 0x1
ADS122C04_CRC_CRC16_ENABLED       = 0x2

### BURNOUT CURRENT SOURCE
ADS122C04_BURN_OUT_CURRENT_OFF    = 0x0
ADS122C04_BURN_OUT_CURRENT_ON     = 0x1

### IDAC CURRENT SETTING
ADS122C04_IDAC_CURRENT_OFF        = 0x0
ADS122C04_IDAC_CURRENT_10_UA      = 0x1
ADS122C04_IDAC_CURRENT_50_UA      = 0x2
ADS122C04_IDAC_CURRENT_100_UA     = 0x3
ADS122C04_IDAC_CURRENT_250_UA     = 0x4
ADS122C04_IDAC_CURRENT_500_UA     = 0x5
ADS122C04_IDAC_CURRENT_1000_UA    = 0x6
ADS122C04_IDAC_CURRENT_1500_UA    = 0x7

#### --- Configuration Register 3
### IDAC1 ROUTING CONFIGURATION
ADS122C04_IDAC1_DISABLED          = 0x0
ADS122C04_IDAC1_AIN0              = 0x1
ADS122C04_IDAC1_AIN1              = 0x2
ADS122C04_IDAC1_AIN2              = 0x3
ADS122C04_IDAC1_AIN3              = 0x4
ADS122C04_IDAC1_REFP              = 0x5
ADS122C04_IDAC1_REFN              = 0x6

### IDAC2 ROUTING CONFIGURATION
ADS122C04_IDAC2_DISABLED          = 0x0
ADS122C04_IDAC2_AIN0              = 0x1
ADS122C04_IDAC2_AIN1              = 0x2
ADS122C04_IDAC2_AIN2              = 0x3
ADS122C04_IDAC2_AIN3              = 0x4
ADS122C04_IDAC2_REFP              = 0x5
ADS122C04_IDAC2_REFN              = 0x6


##### CONSTANTS
PT100_REFERENCE_RESISTOR = 1620.0

# Amplifier gain setting
# ** MAKE SURE THE CONFIG REGISTER 0 GAIN IS THE SAME AS THIS **
PT100_AMPLIFIER_GAIN = 8.0
PT100_AMP_GAIN_HI_TEMP = 4.0

# Internal temperature sensor resolution
# One 14-bit LSB equals 0.03125°C
TEMPERATURE_SENSOR_RESOLUTION = 0.03125

# r = uctypes.struct(uctypes.addressof(a), config)
def twos_comp(val, bits):
    """compute the 2's complement of int value val"""
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val


class ADS122C04(object):
    def __init__(self, address=69):
        self.address = address

        self.data0 = bytearray([0])
        self.data1 = bytearray([0])
        self.data2 = bytearray([0])
        self.data3 = bytearray([0])
        
        self.temp0 = bytearray([0])
        self.volt0 = bytearray([0])
        
        self.config_reg = [
            uctypes.struct(uctypes.addressof(self.data0), register_config_0),
            uctypes.struct(uctypes.addressof(self.data1), register_config_1),
            uctypes.struct(uctypes.addressof(self.data2), register_config_2),
            uctypes.struct(uctypes.addressof(self.data3), register_config_3)
        ]
        
        self.int_temp = uctypes.struct(uctypes.addressof(self.temp0), internal_temperature_union)
        self.raw_v = uctypes.struct(uctypes.addressof(self.temp0), raw_voltage_union)
        
        self.ADS122C04_read_reg(0)
        self.ADS122C04_read_reg(1)
        self.ADS122C04_read_reg(2)
        self.ADS122C04_read_reg(3)
        params = {
            "inputMux": 0,
            "gainLevel": 0,
            "pgaBypass": 0,
            "dataRate": 0,
            "opMode": 0,
            "convMode": 0,
            "selectVref": 0,
            "tempSensorEn": 0,
            "dataCounterEn": 0,
            "dataCRCen": 0,
            "burnOutEn": 0,
            "idacCurrent": 0,
            "routeIDAC1": 0,
            "routeIDAC2": 0,
        }
        
    # TODO
    def ADS122C04_init(self, initParams):
        print(initParams)
        self.config_reg[0].MUX = initParams["inputMux"]
        self.config_reg[0].GAIN = initParams["gainLevel"]
        self.config_reg[0].PGA_BYPASS = initParams["pgaBypass"]

        self.config_reg[1].DR = initParams["dataRate"]
        self.config_reg[1].MODE = initParams["opMode"]
        self.config_reg[1].CM = initParams["convMode"]
        self.config_reg[1].VREF = initParams["selectVref"]
        self.config_reg[1].TS = initParams["tempSensorEn"]

        self.config_reg[2].DCNT = initParams["dataCounterEn"]
        self.config_reg[2].CRCbits = initParams["dataCRCen"]
        self.config_reg[2].BCS = initParams["burnOutEn"]
        self.config_reg[2].IDAC = initParams["idacCurrent"]

        self.config_reg[3].I1MUX = initParams["routeIDAC1"]
        self.config_reg[3].I2MUX = initParams["routeIDAC2"]

        # Write config to ADC
        self.ADS122C04_write_reg(0, self.config_reg[0].all)
        self.ADS122C04_write_reg(1, self.config_reg[1].all)
        self.ADS122C04_write_reg(2, self.config_reg[2].all)
        self.ADS122C04_write_reg(3, self.config_reg[3].all)
        
    def to_bin_string(self, intval):
        return '{0:0>8b}'.format(intval)

    def ADS122C04_read_reg(self, register):
        read_command = 0x20 | (register << 2)
        i2c.writeto(self.address, bytearray([read_command]))
        data = i2c.readfrom(self.address, 1)
        print(data)
        data_int = list(data)[0]
        self.config_reg[register].all = data_int
        return data_int

    def ADS122C04_send_command(self, command):
        i2c.writeto(self.address, bytearray([command]))
        return True

    def ADS122C04_write_reg(self, register, value):
        write_command = 0x40 | (register << 2)
        i2c.writeto(self.address, bytearray([write_command, value]))
        self.ADS122C04_read_reg(register)
        return True

    def ADS122C04_send_command(self, command):
        result = i2c.writeto(self.address, bytearray([command]))
        print(result)
        return result

    def ADS122C04_start(self):
        self.ADS122C04_send_command(0x08)
        
    def checkDataReady(self):
        self.ADS122C04_read_reg(2)
        ready = self.config_reg[2].DRDY
        return ready

    def getConversionMode(self):
        self.ADS122C04_read_reg(1)
        mode = self.config_reg[1].CM
        return mode

    def setConversionMode(self, mode):
        self.ADS122C04_read_reg(1)
        self.config_reg[1].CM = mode
        ADS122C04_write_reg(1,self.config_reg[1].all)
        
    def getConversionData(self):
        self.ADS122C04_send_command(0x10)
        data = i2c.readfrom(self.address, 3)
        return data

    def setGain(self, gain):
        old_gain = self.getGain()
        if old_gain == gain:
            print("no change in gain!")
        self.ADS122C04_read_reg(0)
        self.config_reg[0].GAIN = gain
        self.ADS122C04_write_reg(0,self.config_reg[0].all)
        return True

    def getGain(self):
        self.ADS122C04_read_reg(0)
        gain = self.config_reg[0].GAIN
        return gain

    def getInputMultiplexer(self):
        self.ADS122C04_read_reg(0)
        mux = self.config_reg[0].MUX
        return mux
        
    def setInputMultiplexer(self, mux_config):      
        self.ADS122C04_read_reg(0)
        self.config_reg[0].MUX = mux_config
        self.ADS122C04_write_reg(0, self.config_reg[0].all)
        return True

    # TODO
    def configureADCmode(self, wire_mode, rate=0):
        initParams = {}
        if wire_mode == ADS122C04_4WIRE_MODE:
            initParams["inputMux"] = ADS122C04_MUX_AIN1_AIN0; # Route AIN1 to AINP and AIN0 to AINN
            initParams["gainLevel"] = ADS122C04_GAIN_8; # Set the gain to 8
            initParams["pgaBypass"] = ADS122C04_PGA_ENABLED; # The PGA must be enabled for gains >= 8
            initParams["dataRate"] = rate; # Set the data rate (samples per second). Defaults to 20
            initParams["opMode"] = ADS122C04_OP_MODE_NORMAL; # Disable turbo mode
            initParams["convMode"] = ADS122C04_CONVERSION_MODE_SINGLE_SHOT; # Use single shot mode
            initParams["selectVref"] = ADS122C04_VREF_EXT_REF_PINS; # Use the external REF pins
            initParams["tempSensorEn"] = ADS122C04_TEMP_SENSOR_OFF; # Disable the temperature sensor
            initParams["dataCounterEn"] = ADS122C04_DCNT_DISABLE; # Disable the data counter
            initParams["dataCRCen"] = ADS122C04_CRC_DISABLED; # Disable CRC checking
            initParams["burnOutEn"] = ADS122C04_BURN_OUT_CURRENT_OFF; # Disable the burn-out current
            initParams["idacCurrent"] = ADS122C04_IDAC_CURRENT_1000_UA; # Set the IDAC current to 1mA
            initParams["routeIDAC1"] = ADS122C04_IDAC1_AIN3; # Route IDAC1 to AIN3
            initParams["routeIDAC2"] = ADS122C04_IDAC2_DISABLED; # Disable IDAC2
        elif wire_mode == ADS122C04_4WIRE_HI_TEMP:
            initParams["inputMux"] = ADS122C04_MUX_AIN1_AIN0; # Route AIN1 to AINP and AIN0 to AINN
            initParams["gainLevel"] = ADS122C04_GAIN_4; # Set the gain to 4
            initParams["pgaBypass"] = ADS122C04_PGA_ENABLED; # Enable the PGA
            initParams["dataRate"] = rate; # Set the data rate (samples per second). Defaults to 20
            initParams["opMode"] = ADS122C04_OP_MODE_NORMAL; # Disable turbo mode
            initParams["convMode"] = ADS122C04_CONVERSION_MODE_SINGLE_SHOT; # Use single shot mode
            initParams["selectVref"] = ADS122C04_VREF_EXT_REF_PINS; # Use the external REF pins
            initParams["tempSensorEn"] = ADS122C04_TEMP_SENSOR_OFF; # Disable the temperature sensor
            initParams["dataCounterEn"] = ADS122C04_DCNT_DISABLE; # Disable the data counter
            initParams["dataCRCen"] = ADS122C04_CRC_DISABLED; # Disable CRC checking
            initParams["burnOutEn"] = ADS122C04_BURN_OUT_CURRENT_OFF; # Disable the burn-out current
            initParams["idacCurrent"] = ADS122C04_IDAC_CURRENT_1000_UA; # Set the IDAC current to 1mA
            initParams["routeIDAC1"] = ADS122C04_IDAC1_AIN3; # Route IDAC1 to AIN3
            initParams["routeIDAC2"] = ADS122C04_IDAC2_DISABLED; # Disable IDAC2
        elif wire_mode == ADS122C04_3WIRE_MODE:
            initParams["inputMux"] = ADS122C04_MUX_AIN1_AIN0; # Route AIN1 to AINP and AIN0 to AINN
            initParams["gainLevel"] = ADS122C04_GAIN_8; # Set the gain to 8
            initParams["pgaBypass"] = ADS122C04_PGA_ENABLED; # The PGA must be enabled for gains >= 8
            initParams["dataRate"] = rate; # Set the data rate (samples per second). Defaults to 20
            initParams["opMode"] = ADS122C04_OP_MODE_NORMAL; # Disable turbo mode
            initParams["convMode"] = ADS122C04_CONVERSION_MODE_SINGLE_SHOT; # Use single shot mode
            initParams["selectVref"] = ADS122C04_VREF_EXT_REF_PINS; # Use the external REF pins
            initParams["tempSensorEn"] = ADS122C04_TEMP_SENSOR_OFF; # Disable the temperature sensor
            initParams["dataCounterEn"] = ADS122C04_DCNT_DISABLE; # Disable the data counter
            initParams["dataCRCen"] = ADS122C04_CRC_DISABLED; # Disable CRC checking
            initParams["burnOutEn"] = ADS122C04_BURN_OUT_CURRENT_OFF; # Disable the burn-out current
            initParams["idacCurrent"] = ADS122C04_IDAC_CURRENT_500_UA; # Set the IDAC current to 0.5mA
            initParams["routeIDAC1"] = ADS122C04_IDAC1_AIN2; # Route IDAC1 to AIN2
            initParams["routeIDAC2"] = ADS122C04_IDAC2_AIN3; # Route IDAC2 to AIN3
        elif wire_mode == ADS122C04_3WIRE_HI_TEMP:
            initParams["inputMux"] = ADS122C04_MUX_AIN1_AIN0; # Route AIN1 to AINP and AIN0 to AINN
            initParams["gainLevel"] = ADS122C04_GAIN_4; # Set the gain to 4
            initParams["pgaBypass"] = ADS122C04_PGA_ENABLED; # Enable the PGA
            initParams["dataRate"] = rate; # Set the data rate (samples per second). Defaults to 20
            initParams["opMode"] = ADS122C04_OP_MODE_NORMAL; # Disable turbo mode
            initParams["convMode"] = ADS122C04_CONVERSION_MODE_SINGLE_SHOT; # Use single shot mode
            initParams["selectVref"] = ADS122C04_VREF_EXT_REF_PINS; # Use the external REF pins
            initParams["tempSensorEn"] = ADS122C04_TEMP_SENSOR_OFF; # Disable the temperature sensor
            initParams["dataCounterEn"] = ADS122C04_DCNT_DISABLE; # Disable the data counter
            initParams["dataCRCen"] = ADS122C04_CRC_DISABLED; # Disable CRC checking
            initParams["burnOutEn"] = ADS122C04_BURN_OUT_CURRENT_OFF; # Disable the burn-out current
            initParams["idacCurrent"] = ADS122C04_IDAC_CURRENT_500_UA; # Set the IDAC current to 0.5mA
            initParams["routeIDAC1"] = ADS122C04_IDAC1_AIN2; # Route IDAC1 to AIN2
            initParams["routeIDAC2"] = ADS122C04_IDAC2_AIN3; # Route IDAC2 to AIN3
        elif wire_mode == ADS122C04_2WIRE_MODE:
            initParams["inputMux"] = ADS122C04_MUX_AIN1_AIN0; # Route AIN1 to AINP and AIN0 to AINN
            initParams["gainLevel"] = ADS122C04_GAIN_8; # Set the gain to 8
            initParams["pgaBypass"] = ADS122C04_PGA_ENABLED; # The PGA must be enabled for gains >= 8
            initParams["dataRate"] = rate; # Set the data rate (samples per second). Defaults to 20
            initParams["opMode"] = ADS122C04_OP_MODE_NORMAL; # Disable turbo mode
            initParams["convMode"] = ADS122C04_CONVERSION_MODE_SINGLE_SHOT; # Use single shot mode
            initParams["selectVref"] = ADS122C04_VREF_EXT_REF_PINS; # Use the external REF pins
            initParams["tempSensorEn"] = ADS122C04_TEMP_SENSOR_OFF; # Disable the temperature sensor
            initParams["dataCounterEn"] = ADS122C04_DCNT_DISABLE; # Disable the data counter
            initParams["dataCRCen"] = ADS122C04_CRC_DISABLED; # Disable CRC checking
            initParams["burnOutEn"] = ADS122C04_BURN_OUT_CURRENT_OFF; # Disable the burn-out current
            initParams["idacCurrent"] = ADS122C04_IDAC_CURRENT_1000_UA; # Set the IDAC current to 1mA
            initParams["routeIDAC1"] = ADS122C04_IDAC1_AIN3; # Route IDAC1 to AIN3
            initParams["routeIDAC2"] = ADS122C04_IDAC2_DISABLED; # Disable IDAC2
        elif wire_mode == ADS122C04_2WIRE_HI_TEMP:
            initParams["inputMux"] = ADS122C04_MUX_AIN1_AIN0; # Route AIN1 to AINP and AIN0 to AINN
            initParams["gainLevel"] = ADS122C04_GAIN_4; # Set the gain to 4
            initParams["pgaBypass"] = ADS122C04_PGA_ENABLED; # Enable the PGA
            initParams["dataRate"] = rate; # Set the data rate (samples per second). Defaults to 20
            initParams["opMode"] = ADS122C04_OP_MODE_NORMAL; # Disable turbo mode
            initParams["convMode"] = ADS122C04_CONVERSION_MODE_SINGLE_SHOT; # Use single shot mode
            initParams["selectVref"] = ADS122C04_VREF_EXT_REF_PINS; # Use the external REF pins
            initParams["tempSensorEn"] = ADS122C04_TEMP_SENSOR_OFF; # Disable the temperature sensor
            initParams["dataCounterEn"] = ADS122C04_DCNT_DISABLE; # Disable the data counter
            initParams["dataCRCen"] = ADS122C04_CRC_DISABLED; # Disable CRC checking
            initParams["burnOutEn"] = ADS122C04_BURN_OUT_CURRENT_OFF; # Disable the burn-out current
            initParams["idacCurrent"] = ADS122C04_IDAC_CURRENT_1000_UA; # Set the IDAC current to 1mA
            initParams["routeIDAC1"] = ADS122C04_IDAC1_AIN3; # Route IDAC1 to AIN3
            initParams["routeIDAC2"] = ADS122C04_IDAC2_DISABLED; # Disable IDAC2
        elif wire_mode == ADS122C04_TEMPERATURE_MODE:
            initParams["inputMux"] = ADS122C04_MUX_AIN1_AIN0; # Route AIN1 to AINP and AIN0 to AINN
            initParams["gainLevel"] = ADS122C04_GAIN_1; # Set the gain to 1
            initParams["pgaBypass"] = ADS122C04_PGA_DISABLED;
            initParams["dataRate"] = rate; # Set the data rate (samples per second). Defaults to 20
            initParams["opMode"] = ADS122C04_OP_MODE_NORMAL; # Disable turbo mode
            initParams["convMode"] = ADS122C04_CONVERSION_MODE_SINGLE_SHOT; # Use single shot mode
            initParams["selectVref"] = ADS122C04_VREF_INTERNAL; # Use the internal 2.048V reference
            initParams["tempSensorEn"] = ADS122C04_TEMP_SENSOR_ON; # Enable the temperature sensor
            initParams["dataCounterEn"] = ADS122C04_DCNT_DISABLE; # Disable the data counter
            initParams["dataCRCen"] = ADS122C04_CRC_DISABLED; # Disable CRC checking
            initParams["burnOutEn"] = ADS122C04_BURN_OUT_CURRENT_OFF; # Disable the burn-out current
            initParams["idacCurrent"] = ADS122C04_IDAC_CURRENT_OFF; # Disable the IDAC current
            initParams["routeIDAC1"] = ADS122C04_IDAC1_DISABLED; # Disable IDAC1
            initParams["routeIDAC2"] = ADS122C04_IDAC2_DISABLED; # Disable IDAC2
        elif wire_mode == ADS122C04_RAW_MODE:
            initParams["inputMux"] = ADS122C04_MUX_AIN1_AIN0 # Route AIN1 to AINP and AIN0 to AINN
            initParams["gainLevel"] = ADS122C04_GAIN_1 # Set the gain to 1
            initParams["pgaBypass"] = ADS122C04_PGA_DISABLED
            initParams["dataRate"] = rate # Set the data rate (samples per second). Defaults to 20
            initParams["opMode"] = ADS122C04_OP_MODE_NORMAL; # Disable turbo mode
            initParams["convMode"] = ADS122C04_CONVERSION_MODE_SINGLE_SHO # Use single shot mode
            initParams["selectVref"] = ADS122C04_VREF_INTERNAL # Use the internal 2.048V reference
            initParams["tempSensorEn"] = ADS122C04_TEMP_SENSOR_OFF # Disable the temperature sensor
            initParams["dataCounterEn"] = ADS122C04_DCNT_DISABLE # Disable the data counter
            initParams["dataCRCen"] = ADS122C04_CRC_DISABLED # Disable CRC checking
            initParams["burnOutEn"] = ADS122C04_BURN_OUT_CURRENT_OFF # Disable the burn-out current
            initParams["idacCurrent"] = ADS122C04_IDAC_CURRENT_OFF # Disable the IDAC current
            initParams["routeIDAC1"] = ADS122C04_IDAC1_DISABLED # Disable IDAC1
            initParams["routeIDAC2"] = ADS122C04_IDAC2_DISABLED # Disable IDAC2

        self.ADS122C04_init(initParams)

    # TODO: test this with negative temps again
    def readInternalTemperature(self):
        self.configureADCmode(ADS122C04_TEMPERATURE_MODE)
        self.ADS122C04_start()
        count = 0
        while adc.checkDataReady() == 0:
            time.sleep(1)
            count += 1
            if count > 10:
                print("timeout")
                break
        data = self.getConversionData()
        temp_uint16 = int.from_bytes(data, 'big', False) >> 10
        self.int_temp.UINT16 = temp_uint16

        # # Check if the MS bit is 1
        # if (temp_uint16 & 0x2000) == 0x2000:
        #     # Value is negative so pad with 1's
        #     temp_uint16 |= 0xC000
        # else:
        #     # Value is positive so make sure the two MS bits are 0
        #     temp_uint16 &= 0x3FFF 
        temp_c = self.int_temp.INT16 * TEMPERATURE_SENSOR_RESOLUTION
        return temp_c

    def readPT100F(self):
        return (self.readPT100Centigrade() * 1.8) + 32.0
    
    def readPT100Centigrade(self):
        self.configureADCmode(ADS122C04_3WIRE_MODE)
        self.ADS122C04_start()
        count = 0
        while adc.checkDataReady() == 0:
            time.sleep(1)
            count += 1
            if count > 10:
                print("timeout")
                break
            
        data = self.getConversionData()
        rawv_uint32 = int.from_bytes(data, 'big', False)
        self.raw_v.UINT32 = rawv_uint32
        if ((rawv_uint32 & 0x00800000) == 0x00800000):
            rawv_uint32 |= 0xFF000000
            
        RTD = rawv_uint32 / 8388608.0
        RTD *= PT100_REFERENCE_RESISTOR
        
        RTD /= PT100_AMPLIFIER_GAIN
        
        RTD *= 2.0
        
        ret_val = RTD * -23.10e-9
        ret_val += 17.5848089e-6
        ret_val = sqrt(ret_val)
        ret_val -= 3.9083e-3
        ret_val /= -1.155e-6

        if ret_val >= 0:
            return ret_val
    
        ret_val = -242.02
        ret_val += 2.2228 * RTD
        POLY = RTD * RTD; # Load the polynomial with RTD^2
        ret_val += 2.5859e-3 * POLY
        POLY *= RTD; # Load the polynomial with RTD^3
        ret_val -= 4.8260e-6 * POLY
        POLY *= RTD; # Load the polynomial with RTD^4
        ret_val -= 2.8183e-8 * POLY
        POLY *= RTD; # Load the polynomial with RTD^5
        ret_val += 1.5243e-10 * POLY

        return ret_val

adc = ADS122C04()
adc.ADS122C04_read_reg(0)
#adc.to_bin_string(adc.config_reg[0].all)

#adc.readInternalTemperature()
print("temp PT100")
print(adc.readPT100F())
