from gpiozero import MotionSensor
from time import sleep
import time
import csv
import datetime
import os


pir = MotionSensor(17)
i = 0
current_time = datetime.datetime.now()
curr = time.time()

# Data to write
data = [
    ["Time", "Movement", "Temp"],  # Header row
]

# File path
file_path = "data.csv"

# Append data with headers
file_exists = os.path.isfile(file_path)

if not file_exists:
    # Write data to the CSV file
    with open(file_path, mode="w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerows(data)  # Write multiple rows at once

    print(f"Data successfully written to {file_path}")

while True:
    if (pir.motion_detected):
        #print (current_time.hour, current_time.minute, ":")
        print (str(curr) + ":You moved")
        m = 1
    else:
        print (str(curr) + ":No Motion Detected")
        m = 0
    # Data to append
    new_data = [
        [curr, m, "20"]
    ]

    # Append data to the CSV file
    with open(file_path, mode="a", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerows(new_data)

    print("New data successfully appended to the CSV file.") 
    curr = time.time()
    sleep (1)
    i = i +1