#!/usr/bin/env python3
import argparse
import numpy as np
import matplotlib.pyplot as plt
import re
import sys

def parse_simulation_log(log_str):
    """
    Parses simulation log text into three NumPy arrays:
      • times:     shape (N,)
      • positions: shape (N, 7)
      • torques:   shape (N, 7)

    Assumptions:
    - Each “position” line has exactly 1 time + 7 joint positions (8 columns total).
    - The next seven lines after each position line read:
        tau 1 = <value>  [Nm]
        ...
        tau 7 = <value>  [Nm]
    """
    lines = log_str.strip().splitlines()
    times = []
    positions = []
    torques = []
    i = 0

    while i < len(lines):
        line = lines[i].strip()
        # Identify a “position” line by checking for 8 tokens
        if not line.startswith("tau") and len(line.split()) == 8:
            parts = line.split()
            try:
                t = float(parts[0])
                q = [float(val) for val in parts[1:]]  # 7 joint positions
            except ValueError:
                # If conversion fails, skip
                i += 1
                continue

            times.append(t)
            positions.append(q)

            # Parse the next 7 “tau” lines
            torque_vals = []
            for j in range(1, 8):
                if i + j >= len(lines):
                    break
                tau_line = lines[i + j].strip()
                # Extract the numeric torque value
                m = re.search(r"=\s*([-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?)", tau_line)
                if m:
                    torque_vals.append(float(m.group(1)))
            if len(torque_vals) == 7:
                torques.append(torque_vals)
            else:
                # If we didn’t find exactly 7 torques, pad with NaNs
                torque_vals += [np.nan] * (7 - len(torque_vals))
                torques.append(torque_vals)

            # Move index past this block (1 pos line + 7 torque lines)
            i += 8
        else:
            i += 1

    if len(times) == 0:
        print("Error: No valid position lines found in the log.", file=sys.stderr)
        sys.exit(1)

    return np.array(times), np.array(positions), np.array(torques)


def main():
    parser = argparse.ArgumentParser(
        description="Parse a joint‐position+torque log and plot joint positions & torques over time."
    )
    parser.add_argument(
        "logfile",
        help="Path to the text file containing your simulation log",
    )
    args = parser.parse_args()

    # Read entire file
    try:
        with open(args.logfile, "r") as f:
            log_text = f.read()
    except Exception as e:
        print(f"Error opening '{args.logfile}': {e}", file=sys.stderr)
        sys.exit(1)

    # Parse it
    times, positions, torques = parse_simulation_log(log_text)

    # Plot joint positions
    plt.figure(figsize=(8, 5))
    for joint_idx in range(positions.shape[1]):
        plt.plot(times, positions[:, joint_idx], label=f'Joint {joint_idx+1} Position')
    plt.xlabel('Time [s]')
    plt.ylabel('Position [rad]')
    plt.title('Joint Positions Over Time')
    plt.legend(loc='best', fontsize='small')
    plt.grid(True)

    # Plot joint torques
    plt.figure(figsize=(8, 5))
    for joint_idx in range(torques.shape[1]):
        plt.plot(times, torques[:, joint_idx], label=f'Joint {joint_idx+1} Torque')
    plt.xlabel('Time [s]')
    plt.ylabel('Torque [Nm]')
    plt.title('Joint Torques Over Time')
    plt.legend(loc='best', fontsize='small')
    plt.grid(True)

    plt.show()


if __name__ == "__main__":
    main()
