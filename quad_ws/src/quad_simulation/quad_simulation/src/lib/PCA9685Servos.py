'''
    PCA9685 RC Servo Library

    Limited functionality designed to drive RC servos.

    For more robust features, such as bus collision avoidance,
    see Adafruit's PCA9685 library: https://github.com/adafruit/Adafruit_CircuitPython_PCA9685

'''
from smbus2 import SMBus
from time import sleep

# Register addresses
PCA9685_MODE1_REG = 0x00
PCA9685_MODE2_REG = 0x01
PCA9685_PRESCALE_REG = 0xFE
PCA9685_LED0_REG = 0x06

# Mode1 register values
PCA9685_MODE1_RESTART = 0x80
PCA9685_MODE1_EXTCLK = 0x40
PCA9685_MODE1_AUTOINC = 0x20
PCA9685_MODE1_SLEEP = 0x10
PCA9685_MODE1_SUBADR1 = 0x08
PCA9685_MODE1_SUBADR2 = 0x04
PCA9685_MODE1_SUBADR3 = 0x02
PCA9685_MODE1_ALLCALL = 0x01

# Mode2 register values
PCA9685_MODE2_OUTDRV_TPOLE = 0x04
PCA9685_MODE2_INVRT = 0x10
PCA9685_MODE2_OUTNE_TPHIGH = 0x01
PCA9685_MODE2_OUTNE_HIGHZ = 0x02
PCA9685_MODE2_OCH_ONACK = 0x08

# General values
PCA9685_SW_RESET = 0x06

# Special address
PCA9685_BROADCAST_ADDR = 0x00

# Masks
PCA9685_PWM_MASK = 0x0FFF

class PCA9685Servos:
    """
    Class interfacing with an PCA9685 16-channel PWM driver.
    :param int bus_index: The RPi I2C bus number: Default bus: 1.    
    :param int address: The I2C address of the PCA9685. Default address: 0x40.    
    """
    def __init__(
        self, 
        bus_index = 1,
        address=0x40      
    ):
        self.bus_index = bus_index
        self.address = address
        self.reference_clock_speed_hz = 25000000
        self.frequency_hz = 50  
        self.min_pulse_width = 0
        self.max_pulse_width = 3000      
                   
        bus = SMBus(self.bus_index)

        # Set PWM frequency
        bus.write_byte_data(self.address, PCA9685_MODE1_REG, PCA9685_MODE1_SLEEP)
        prescale_val = int((self.reference_clock_speed_hz / (4096 * self.frequency_hz)) - 1)    
        bus.write_byte_data(self.address, PCA9685_PRESCALE_REG, prescale_val)
        bus.write_byte_data(self.address, PCA9685_MODE1_REG, ~PCA9685_MODE1_SLEEP) 
        sleep(0.005)       

        bus.close()

    def set_pulse_width(self, channel, pulse_width):

        bus = SMBus(self.bus_index)
        '''
        :param int channel: 0 - 16 PCA9685 channel.  
        :param int pulse_width: Servo pulse width typically 1500 - 2500 microseconds.  
        '''
        if channel < 0 | channel > 15:
            return
        
        if pulse_width < self.min_pulse_width:
            pulse_width = self.min_pulse_width

        if pulse_width > self.max_pulse_width:
            pulse_width = self.max_pulse_width

        reg = PCA9685_LED0_REG + (channel * 0x04)
               
        phase_begin = 0
        #phase_end = (phase_begin + pulse_width) & PCA9685_PWM_MASK
        phase_end = int(((4016 / 20000) * pulse_width)) & PCA9685_PWM_MASK

        bus.write_byte_data(self.address, reg, phase_begin & 0xFF)
        bus.write_byte_data(self.address, reg + 1, (phase_begin >> 8) & 0xFF)
        bus.write_byte_data(self.address, reg + 2, phase_end & 0xFF)
        bus.write_byte_data(self.address, reg + 3, (phase_end >> 8) & 0xFF)

        bus.close()