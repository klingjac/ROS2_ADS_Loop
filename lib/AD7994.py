from enum import IntEnum
import smbus2
import time

class AD7994():
    """
    Simple driver for AD7993 operation in command mode operation

    Other functionalities not used but implementable in software:
        - Alert functionality
        - Cyclic conversion operation
    """
    _device_addr = None
    _available_addr = [0x20, 0x21, 0x22, 0x23, 0x24]

    def __init__(self, address=0x23, smbus_num=1):
        if address not in self._available_addr:
            raise Exception("Possible addresses are 0x20, 0x21, and 0x22, 0x23")
        
        self._device_addr = address
        self._i2c_bus = smbus2.SMBus(smbus_num)

    def change_address(self, address):
        if address not in self._available_addr:
            raise Exception("Possible addresses are 0x20, 0x21, and 0x22")
        
        self._device_addr = address

    def get_data(self):
        """
        Returns a list containing raw results in the following order: [Vin1, Vin2, Vin3, Vin4]

        Result need to be converted to a voltage, then the appropriate conversion function must 
        be applied to convert the raw voltage to the correct units.
        """
        data = self._i2c_bus.read_i2c_block_data(self._device_addr, 0xF0, 8)

        newdata = []
        for i in range(0,len(data), 2):
            newdata.append((data[i]<<8) + data[i+1])

        rawCounts = [((channelBytes >> 2) & 0x3ff) for channelBytes in newdata]

        return rawCounts