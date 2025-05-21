import numpy as np
import re
import matplotlib.pyplot as plt
file_path = 'log_static'
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
dictionary = {
    0: 'joint_1_pos',
    1: 'joint_2_pos',
    2: 'joint_3_pos',
    3: 'joint_4_pos',
    4: 'joint_5_pos',
    5: 'joint_6_pos',
    6: 'joint_7_pos',
    7: 'joint_8_pos',
    8: 'joint_9_pos',
   # 9: 'joint_1_vel',
   # 10: 'joint_2_vel',
   # 11: 'joint_3_vel',
   # 12: 'joint_4_vel',
   # 13: 'joint_5_vel',
   # 14: 'joint_6_vel',
   # 15: 'joint_7_vel',
   # 16: 'joint_8_vel',
   # 17: 'joint_9_vel'
}
for col in dictionary.keys():
    plt.plot(log_array[:, col], label=f'{dictionary[col]}', alpha=0.7)
plt.legend(loc='upper right', ncol=2)
plt.title('Time Step: 0.01, with PID')
plt.xlabel('Sample Index')
plt.ylabel('rad')
plt.grid(True)
plt.tight_layout()
plt.show()
