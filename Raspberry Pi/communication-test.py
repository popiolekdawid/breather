import serial
import time
import struct

ser = serial.Serial ("/dev/ttyAMA0", 9600)    #Open port with baud rate

data1='hello'
data2='00001100'
data='1'
write_timeout = 1
ser.write("ziomeczku nie uwierzysz co sie dzieje".encode())
mode = 1
flow = 0
pressure = 0
PEEP = 40
inhale_exhale_ratio = 1.05
breath_per_minute = 10
data_to_send = [0]*7

while True:
	inhale_time = int((inhale_exhale_ratio * (60000/breath_per_minute)) / (1 + inhale_exhale_ratio))
	exhale_time = int((60000/breath_per_minute) - inhale_time)
	
	data_to_send[0] = 255
	data_to_send[1] = mode
	data_to_send[2] = inhale_time & 15
	data_to_send[3] = (inhale_time >> 4) & 15
	data_to_send[4] = exhale_time & 15
	data_to_send[5] = (exhale_time >> 4) & 15
	data_to_send[6] = PEEP
	
	
	ser.write(bytes(data_to_send))
	
	

	if int.from_bytes(ser.read(),"big") == 255:
		flow = int.from_bytes(ser.read(),"big")
		pressure = int.from_bytes(ser.read(),"big")  
	print (f"{flow} {pressure}")

#ser.write(received_data)                #transmit data serially 
