#!/usr/bin/env python3
"""
Continuous performance data generator for testing the monitoring system.
Simulates real-time updates from the Drake simulation.
"""

import json
import time
import random
import os
import signal
import sys
from pathlib import Path

def generate_sample_performance_data():
    """Generate realistic sample performance data for turbo mode."""
    # Simulate realistic performance metrics for turbo mode
    rtf = random.uniform(5.0, 25.0)  # Real-time factor between 5x and 25x (turbo mode)
    physics_time = random.uniform(0.04, 0.2)  # Physics time between 0.04ms and 0.2ms (turbo mode)
    freq = 1000.0 / physics_time  # Calculate frequency from physics time
    
    return {
        "rtf": rtf,
        "physics": physics_time,
        "freq": freq,
        "timestamp": time.time()
    }

def signal_handler(sig, frame):
    print("\nStopping continuous performance monitor...")
    sys.exit(0)

def main():
    script_dir = Path(__file__).parent
    
    # Set up signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    
    print("=== Continuous Performance Monitor ===")
    print(f"Writing to: {script_dir}")
    print("Generating performance data every 1 second...")
    print("Press Ctrl+C to stop")
    print()
    
    update_interval = 1.0  # Update every 1 second
    
    try:
        while True:
            # Generate sample data
            data = generate_sample_performance_data()
            
            # Write to JSON file
            json_file = script_dir / "performance.json"
            with open(json_file, 'w') as f:
                json.dump(data, f, indent=2)
            
            # Print current metrics
            print(f"\r[PERFORMANCE] RTF: {data['rtf']:6.3f}x | "
                  f"Avg physics: {data['physics']:6.3f}ms | "
                  f"Freq: {data['freq']:7.0f} Hz | "
                  f"Target: 1000 Hz", end='', flush=True)
            
            # Wait for next update
            time.sleep(update_interval)
            
    except KeyboardInterrupt:
        print("\nStopping continuous performance monitor...")

if __name__ == "__main__":
    main()
