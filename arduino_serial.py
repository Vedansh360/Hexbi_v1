import serial
import time

# Adjust the port based on your system
arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)  # Allow time for Arduino to reset

def send_command(command):
    arduino.write(command.encode())  # Send as bytes
    print(f"Sent command: {command}")

try:
    while True:
        cmd = input("Enter command (F/B/L/R/S): ").strip().upper()
        if cmd in ['F', 'B', 'L', 'R', 'S']:
            send_command(cmd)
        else:
            print("Invalid command!")

except KeyboardInterrupt:
    print("\nStopping...")
    send_command('S')
    arduino.close()
