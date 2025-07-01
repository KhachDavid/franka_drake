#!/usr/bin/env python3
import argparse
import numpy as np
import matplotlib.pyplot as plt
import re
import sys

def parse_simulation_log(log_str):
    """
    Parses a simulation log string into NumPy arrays with arbitrary joint count.

    Returns:
        times (N,):     time stamps
        positions (N,J): joint positions, where J is inferred from each position line
        torques (N,J):   joint torques (missing values padded with NaN)

    Expected log block format::

        <time>  q1 q2 ... qJ
        tau 1 = <value>  [Nm]
        ...
        tau J = <value>  [Nm]

    The parser identifies a position line whenever the first token is a float
    and the line does *not* start with "tau". The number of remaining tokens
    determines the number of joints (J) for that sample.
    """
    lines = log_str.strip().splitlines()
    times: list[float] = []
    positions: list[list[float]] = []
    torques: list[list[float]] = []
    i = 0

    # Regex that matches a floating-point number (with optional sign / exponent)
    float_re = r"[-+]?(?:[0-9]*\.[0-9]+|[0-9]+)(?:[eE][-+]?[0-9]+)?"

    while i < len(lines):
        line = lines[i].strip()
        if not line:
            i += 1
            continue

        # Attempt to parse the first token as time (float)
        tokens = line.split()
        try:
            t_val = float(tokens[0])
        except ValueError:
            # Not a position line → skip
            i += 1
            continue

        if line.startswith("tau") or len(tokens) < 2:
            # Likely a malformed line, not a position entry
            i += 1
            continue

        # We have a valid position line
        time_stamp = t_val
        try:
            q_vals = [float(tok) for tok in tokens[1:]]
        except ValueError:
            # One of the joint values failed to parse; skip this line
            i += 1
            continue

        num_joints = len(q_vals)
        times.append(time_stamp)
        positions.append(q_vals)

        # Collect the following `num_joints` torque lines
        joint_torques: list[float] = []
        for j in range(1, num_joints + 1):
            if i + j >= len(lines):
                break  # Reached end of log unexpectedly
            tau_line = lines[i + j].strip()
            m = re.search(float_re, tau_line)
            if m:
                try:
                    joint_torques.append(float(m.group(0)))
                    continue
                except ValueError:
                    pass
            # If we didn't append (failed to parse), append NaN placeholder
            joint_torques.append(float('nan'))

        # Pad with NaNs if fewer than expected were found
        if len(joint_torques) < num_joints:
            joint_torques += [float('nan')] * (num_joints - len(joint_torques))

        torques.append(joint_torques)

        # Advance index: 1 position line + num_joints torque lines
        i += 1 + num_joints

    if not times:
        print("Error: No valid position lines found in the log.", file=sys.stderr)
        sys.exit(1)

    # Ensure all rows have the same length by padding with NaNs where necessary
    max_joints = max(len(p) for p in positions)
    for row in positions:
        if len(row) < max_joints:
            row += [np.nan] * (max_joints - len(row))
    for row in torques:
        if len(row) < max_joints:
            row += [np.nan] * (max_joints - len(row))

    return np.array(times), np.array(positions), np.array(torques)


def main():
    parser = argparse.ArgumentParser(
        description="Parse a joint-position+torque log and plot joint positions & torques over time."
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
    plt.title(f'Joint Positions Over Time: {args.logfile}')
    plt.legend(loc='best', fontsize='small')
    plt.grid(True)

    # Plot joint torques
    plt.figure(figsize=(8, 5))
    for joint_idx in range(torques.shape[1]):
        plt.plot(times, torques[:, joint_idx], label=f'Joint {joint_idx+1} Torque')
    plt.xlabel('Time [s]')
    plt.ylabel('Torque [Nm]')
    plt.title(f'Joint Torques Over Time: {args.logfile}')
    plt.legend(loc='best', fontsize='small')
    plt.grid(True)

    plt.show()


if __name__ == "__main__":
    main()
