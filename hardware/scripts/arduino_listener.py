import serial
import json
import socket
import time

UDP_IP = "127.0.0.1"
UDP_PORT = 9870

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

ser = serial.Serial('/dev/ttyUSB1', 9600)
ser.flushInput()

start_time = time.time()

print('recording')

with open('sensor.log', 'w') as f:
    while True:
        try:
            ser_bytes = ser.readline()
            decoded_bytes = float(ser_bytes[0:len(ser_bytes)-2].decode("utf-8"))
         
            timestamp = time.time()-start_time
            #message = dict(timestamp=timestamp, data=decoded_bytes)
            #raw = json.dumps(message)
            #sock.sendto(raw.encode('utf-8'), (UDP_IP, UDP_PORT))
        
            f.write("{} {}\n".format(timestamp, decoded_bytes));
        
            #print(timestamp, decoded_bytes)
        except KeyboardInterrupt:
            break
        except:
            pass
