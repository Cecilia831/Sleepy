import csv
import os

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


# Data to append
new_data = [
    [curr, m, "20"]
]

# Append data to the CSV file
with open(file_path, mode="a", newline="") as csv_file:
    writer = csv.writer(csv_file)
    writer.writerows(new_data)

print("New data successfully appended to the CSV file.")

