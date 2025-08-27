#!/usr/bin/env python3
"""
Simple HTTP server for Drake performance monitoring.
Serves the HTML performance monitor and JSON data files.
"""

import http.server
import socketserver
import os
import sys
from pathlib import Path

class PerformanceMonitorHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        # Add CORS headers to allow the HTML to fetch JSON data
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        super().end_headers()
    
    def do_OPTIONS(self):
        # Handle preflight requests
        self.send_response(200)
        self.end_headers()
    
    def log_message(self, format, *args):
        # Custom logging to show what files are being served
        print(f"[{self.log_date_time_string()}] {format % args}")
    
    def do_GET(self):
        # Log the request
        print(f"Request: {self.path}")
        super().do_GET()

def main():
    # Change to the directory containing this script
    script_dir = Path(__file__).parent
    os.chdir(script_dir)
    
    # Ensure the HTML file exists
    html_file = script_dir / "drake_performance_monitor.html"
    if not html_file.exists():
        print(f"Error: {html_file} not found!")
        print("Make sure to run the Drake simulation first to generate the HTML file.")
        sys.exit(1)
    
    # Create a simple server
    PORT = 8080
    
    try:
        with socketserver.TCPServer(("", PORT), PerformanceMonitorHandler) as httpd:
            print(f"Performance monitor server running at http://localhost:{PORT}")
            print(f"Open http://localhost:{PORT}/drake_performance_monitor.html in your browser")
            print("Press Ctrl+C to stop the server")
            httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nServer stopped.")
    except OSError as e:
        if e.errno == 48:  # Address already in use
            print(f"Port {PORT} is already in use. Try a different port or stop the existing server.")
        else:
            print(f"Error starting server: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
