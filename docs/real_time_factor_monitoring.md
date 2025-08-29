# Real-Time Factor Monitoring

The Drake simulation now includes comprehensive real-time factor monitoring that displays performance metrics both in the console and in a web-based dashboard.

## Features

- **Console Output**: Real-time performance metrics displayed in the terminal
- **Web Dashboard**: Beautiful HTML interface showing performance data
- **Auto-refresh**: Metrics update every 1 seconds
- **Performance Status**: Color-coded status indicators (Excellent/Good/Poor)

## Metrics Displayed

- **Real-Time Factor (RTF)**: Ratio of simulation time to wall-clock time
  - Target: 1.0x (real-time)
  - > 1.0x: Faster than real-time (good for research)
  - < 1.0x: Slower than real-time (may affect control performance)

- **Average Physics Time**: Time per simulation step in milliseconds
  - Target: < 1.0ms for 1kHz control
  - Lower is better

- **Effective Frequency**: Actual simulation frequency in Hz
  - Target: 1000 Hz for 1kHz control
  - Higher is better

## Usage

### 1. Build and Run the Simulation

```bash
cd franka_drake
mkdir -p build && cd build
cmake ..
make -j$(nproc)

# Run the simulation with performance monitoring
./bin/franka-fci-sim-embed-example
```

### 2. View Web Dashboard

1. Start the performance monitor server:
   ```bash
   python3 scripts/perf/performance_monitor_server.py
   ```

2. Open your browser and navigate to:
   ```
   http://localhost:8080/drake_performance_monitor.html
   ```

3. The dashboard will auto-refresh every 1 second with live performance data.

## Performance Interpretation

### Real-Time Factor (RTF)
- **≥ 5.0x**: Excellent - Turbo mode, great for research
- **1.0x - 5.0x**: Good - Faster than real-time, good for research
- **0.95x - 1.0x**: Good - Near real-time
- **0.8x - 0.95x**: Acceptable - Slightly below real-time
- **< 0.8x**: Poor - Significantly below real-time

### Physics Time
- **< 0.5ms**: Excellent - Plenty of headroom for 1kHz control
- **0.5ms - 1.0ms**: Good - Adequate for 1kHz control
- **> 1.0ms**: Poor - May struggle to maintain 1kHz

### Effective Frequency
- **≥ 950 Hz**: Excellent - Near target 1kHz
- **800 Hz - 950 Hz**: Good - Acceptable for most applications
- **< 800 Hz**: Poor - May affect control performance

### Headless Mode
For maximum performance without visualization:

```bash
# Set environment variable for headless mode
export HEADLESS=1
./bin/franka-fci-sim-embed-example
```

This removes Meshcat visualization overhead, typically improving performance by 10-20%.

## Troubleshooting

### Low Real-Time Factor
1. Check if other processes are consuming CPU
2. Try headless mode to reduce visualization overhead
3. Consider using turbo mode for research applications
4. Monitor system resources (CPU, memory, disk I/O)

### Web Dashboard Not Updating
1. Ensure the Python server is running
2. Check browser console for JavaScript errors
3. Verify the JSON file is being written to `scripts/perf/performance.json`
4. Try refreshing the page

### 404 Error (File Not Found)
1. Make sure the Drake simulation has been run at least once to generate the HTML file
2. Check that `scripts/drake_performance_monitor.html` exists
3. Verify the server is running from the correct directory
4. Try running the test script first: `python3 scripts/test_performance_monitor.py`

### Port Conflicts
If port 8080 is in use, modify the port in `scripts/performance_monitor_server.py`:

```python
PORT = 8081  # Change to available port
```

The web dashboard uses vanilla JavaScript. Do we really need React anymore?
