#!/usr/bin/env python3
"""
Test script for the performance monitoring system.
Generates sample performance data to test the web dashboard.
"""

import json
import time
import random
import os
from pathlib import Path

def generate_sample_performance_data():
    """Generate realistic sample performance data."""
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

def main():
    script_dir = Path(__file__).parent
    
    print("Generating sample performance data...")
    print(f"Writing to: {script_dir}")
    
    # Generate sample data
    data = generate_sample_performance_data()
    
    # Write to JSON file
    json_file = script_dir / "performance.json"
    with open(json_file, 'w') as f:
        json.dump(data, f, indent=2)
    
    print(f"Sample data written to {json_file}")
    print(f"RTF: {data['rtf']:.3f}x")
    print(f"Physics time: {data['physics']:.3f}ms")
    print(f"Frequency: {data['freq']:.0f} Hz")
    
    # Check if HTML file exists
    html_file = script_dir / "drake_performance_monitor.html"
    if html_file.exists():
        print(f"\nHTML file found: {html_file}")
        print("You can now run the performance monitor server:")
        print("python3 scripts/performance_monitor_server.py")
        print("Then open: http://localhost:8080/drake_performance_monitor.html")
    else:
        print(f"\nHTML file not found: {html_file}")
        print("Run the Drake simulation first to generate the HTML file.")

if __name__ == "__main__":
    main()
