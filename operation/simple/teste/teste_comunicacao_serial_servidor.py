import pickle
import serial

ser = serial.Serial('/dev/ttyUSB1', 9600)

received_data = ser.read(ser.in_waiting)
ser.close()

data_received = pickle.loads(received_data)
print("Received data:", data_received)