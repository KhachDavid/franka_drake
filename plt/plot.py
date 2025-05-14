import numpy as np
import re
import matplotlib.pyplot as plt
file_path = 'logs'
pattern = re.compile(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?')
with open(file_path, 'r') as f:
    file_lines = f.readlines()

# Parse the lines into float rows, filtering lines with exactly 18 numeric values
parsed_data = []
for line in file_lines:
    # Find all numbers in the line
    numbers = pattern.findall(line)
    # Convert to float and check if the line has exactly 18 numbers
    
    if len(numbers) == 18:
        parsed_data.append(list(map(float, numbers)))

# Convert to NumPy array
log_array = np.array(parsed_data)

# Plot each column
plt.figure(figsize=(15, 8))
dictionary = {6: 'joint_7_pos', 15: 'joint_7_vel'}
for col in dictionary.keys():
    plt.plot(log_array[:, col], label=f'{dictionary[col]}', alpha=0.7)
plt.legend(loc='upper right', ncol=2)
plt.title('Franka joint 7')
plt.xlabel('Sample Index')
plt.ylabel('rad')
plt.grid(True)
plt.tight_layout()
plt.show()
