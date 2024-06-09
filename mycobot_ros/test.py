import serial
import time

# 시리얼 포트 설정
port = '/dev/ttyUSB0'
baudrate = 9600

try:
    ser = serial.Serial(port, baudrate, timeout=1)
    time.sleep(2)  # 시리얼 포트 초기화 대기
    print("Serial port opened successfully")
except serial.SerialException as e:
    print(f"Failed to open serial port: {e}")
    exit()

while True:
    if ser.in_waiting > 0:
        data = ser.readline().decode('utf-8').strip()
        print(f"Received data: {data}")
    else:
        time.sleep(1)