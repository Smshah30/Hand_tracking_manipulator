import pandas as pd
import matplotlib.pyplot as plt
import re


def parse_time_string(time_string):
    # Use regular expression to extract sec and nanosec values
    pattern = r'sec=(\d+), nanosec=(\d+)'
    match = re.search(pattern, time_string)

    if match:
        sec = float(match.group(1))
        nanosec = float(match.group(2))

        new_time = sec + (nanosec/1000000000)
        return new_time
    else:
        # Handle if the pattern doesn't match the provided string
        return None
# Read the CSV file
    
file_path = '/home/smit/qwe_ws/position_log_asd2.csv'  # Replace 'your_file.csv' with your file path
data = pd.read_csv(file_path)

# Assuming the CSV columns are named 'time', 'targetposition', and 'currentposition'
str_time = data['Time']
target_position = data['Target Position']
current_position = data['Current Position']

time = []
for i in range(len(str_time)):
    ti = parse_time_string(str_time[i])
    time.append(ti)

print(type(time[0]))

normalized_time = [x - time[0] for x in time]
# Create a graph
plt.figure(figsize=(10, 6))
plt.plot(normalized_time, target_position, label='Target Position', color='blue')
plt.plot(normalized_time, current_position, label='Current Position', color='red')
plt.xlabel('Time')
plt.ylabel('Position')
plt.title('Target Position vs Current Position over Time')
plt.legend()
plt.grid(True)
plt.show()
