import pickle
import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 9600)
time.sleep(2)

data_to_send = {"name": "Robot", "task": "navigation"}

serialized_data = pickle.dumps(data_to_send)

ser.write(serialized_data)
ser.close()
print("Data sent successfully.")