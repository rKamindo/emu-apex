import serial
import time

# Arduino Communication
arduino_port = 'COM9'

try:
  arduino = serial.Serial(port=arduino_port, baudrate=9600, timeout=.1)
  print(f"Connected to Arduino on port {arduino_port}")
except serial.SerialException as e:
  print(f"Error connecting to Arduino: {e}")
  arduino = None

initial_ascent_rate = 6 # m/s  
deceleration_rate = 0.0167 # m/s^2, the deceleration of the balloon during venting
altitude = 0 # starting altitude in meters

vent_open = False
vent_open_time = 0
current_ascent_rate = initial_ascent_rate
last_ascent_rate = initial_ascent_rate

last_request_time = None # time of the last altitude request

def update_ascent_rate(time_elapsed):
  global current_ascent_rate, vent_open_time
  if vent_open:
    vent_open_time += time_elapsed
    new_rate = current_ascent_rate - (deceleration_rate * time_elapsed)
    current_ascent_rate = max(new_rate, -50) # terminal velocity
  print(f"Current ascent rate: {current_ascent_rate:.2f}, Vent open time: {vent_open_time:2f}")
    
# calculate the altitude change since last altitude request
# uses displacement equation
def calculate_altitude_change(time_elapsed):
  global last_ascent_rate, deceleration_rate
  
  v0 = last_ascent_rate

  a = -deceleration_rate

  altitude_change = v0 * time_elapsed + 0.5 * a * time_elapsed**2
  return altitude_change

def measure_altitude():
  global altitude, last_ascent_rate, current_ascent_rate, last_request_time
  current_time = time.time()
  if last_request_time is not None:
    time_elapsed = current_time - last_request_time

    # store current ascent rate to last ascent rate
    last_ascent_rate = current_ascent_rate

    # update ascent rate
    update_ascent_rate(time_elapsed)

    # calculate altitude change
    altitude_change = calculate_altitude_change(time_elapsed)

    # update altitude
    altitude += altitude_change
  
  last_request_time = current_time

def send_altitude_to_arduino():
  if arduino: # only send if the connection is succesful
    try:
      data_string = f"{altitude:2f}\n" # format data as string
      arduino.write(data_string.encode()) # encode and send
      print(f"Sent to Arduino: {data_string.strip()}")
    except serial.SerialException as e:
      print(f"Error sending data to Arduino: {e}")

# simulate opening and closing the vent
def open_vent():
  global vent_open
  vent_open = True
  print(f"Vent opened, altitude is {altitude:.2f}")

def close_vent():
  global vent_open, vent_open_time
  vent_open = False
  print(f"Vent closed, altitude is {altitude:.2f}")
  print(f"Vent was open for {vent_open_time:.2f}")

if __name__ == "__main__":  
  print("Python program started")
  while True:
    if arduino and arduino.in_waiting > 0:
      try:
        command = arduino.readline().decode('utf-8').strip()
        print(f"Received command from Arduino: {command}")

        if command == "VENT_OPEN":
          open_vent()
        elif command == "VENT_CLOSE":
          close_vent()
        elif command == "REQUEST_ALTITUDE":
          measure_altitude()
          send_altitude_to_arduino()
        else:
          print(f"Unknown command from Arduino: {command}")
      except serial.SerialException as e:
        print(f"Error reading from Arduino: {e}")
    
    time.sleep(0.01) # don't hog the cpu
