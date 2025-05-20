import serial
import time
ser = serial.Serial('/dev/esp32', 9600 ,timeout=2)
ser.close()
time.sleep(1)
ser.open()

ser.write(b'A')
time.sleep(5)
ser.close()