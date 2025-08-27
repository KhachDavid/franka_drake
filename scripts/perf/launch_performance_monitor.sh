#!/bin/bash

# Launcher script for Drake performance monitoring system

echo "=== Drake Performance Monitor Launcher ==="

# Check if we're in the right directory
if [ ! -f "scripts/performance_monitor_server.py" ]; then
    echo "Error: Please run this script from the franka_drake directory"
    exit 1
fi

# Check for continuous mode argument
CONTINUOUS_MODE=false
if [ "$1" = "--continuous" ] || [ "$1" = "-c" ]; then
    CONTINUOUS_MODE=true
    echo "Continuous mode enabled - will generate live performance data"
fi

# Generate sample data if HTML file doesn't exist
if [ ! -f "scripts/drake_performance_monitor.html" ]; then
    echo "HTML file not found. Generating sample data..."
    python3 scripts/test_performance_monitor.py
fi

# Generate fresh sample data
echo "Generating fresh performance data..."
python3 scripts/test_performance_monitor.py

echo ""
echo "Starting performance monitor server..."
echo "Open http://localhost:8080/drake_performance_monitor.html in your browser"
echo "Press Ctrl+C to stop the server"
echo ""

if [ "$CONTINUOUS_MODE" = true ]; then
    echo "Starting continuous performance data generator in background..."
    python3 scripts/continuous_performance_monitor.py &
    CONTINUOUS_PID=$!
    echo "Continuous monitor PID: $CONTINUOUS_PID"
    echo ""
fi

# Start the server
python3 scripts/performance_monitor_server.py

# Clean up continuous monitor if it was started
if [ "$CONTINUOUS_MODE" = true ] && [ ! -z "$CONTINUOUS_PID" ]; then
    echo "Stopping continuous performance monitor..."
    kill $CONTINUOUS_PID 2>/dev/null || true
fi
