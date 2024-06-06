import serial
import time

# Set up the serial connection to the Arduino
arduino_port = "/dev/ttyAMA0"  # Update this to the correct port for your system
baud_rate = 115200  # Match this to the baud rate in your Arduino sketch

# Initialize the serial connection
ser = serial.Serial(
    port='/dev/serial0',  # Change this according to connection methods, e.g. /dev/ttyUSB0
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

def send_bytes(data):
    """
    Send a list of bytes to the Arduino.
    
    :param data: List of bytes to send
    """
    byte_data = bytearray(data)
    ser.write(byte_data)
    print(f"Sent: {byte_data}")

try:
    # Example data to send
    data_to_send = [0x01, 0x02, 0x03, 0x04, 0x05]
    
    while True:
        send_bytes(data_to_send)
        time.sleep(0.1)  # Wait for a second before sending the next batch of bytes

except KeyboardInterrupt:
    print("Serial communication interrupted by user.")
finally:
    ser.close()
    print("Serial port closed.")
