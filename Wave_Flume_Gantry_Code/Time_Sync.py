import serial
from datetime import datetime, timezone

def get_epoch_time():
    now = datetime.now(timezone.utc)
    epoch_time = int(now.timestamp())
    return epoch_time

def send_time_to_arduino(port):
    with serial.Serial(port, 9600, timeout=1) as ser:
        epoch_time = get_epoch_time()
        ser.write(f"{epoch_time}\n".encode('utf-8'))
        print(f"Sent time: {epoch_time}")
        response = ser.readline().decode('utf-8').strip()
        print(f"Arduino response: {response}")

if __name__ == "__main__":
    arduino_port = 'COM6'  # Update this with your actual Arduino port
    send_time_to_arduino(arduino_port)
