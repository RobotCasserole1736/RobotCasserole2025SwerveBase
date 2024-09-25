import sys
import io
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import os

class BufferedConsoleServer:
    def __init__(self, log_file: str = 'console_output.log', port: int = 8080):
        """
        Initializes the console server, redirects stdout and stderr, and sets up disk-based buffering.
        :param log_file: Path to the log file where stdout/stderr will be buffered.
        :param port: Port on which to run the HTTP server.
        """
        self.log_file = log_file
        self.port = port

        # Open the log file in append mode
        self.log_file_handle = open(self.log_file, 'a')

        # Redirect stdout and stderr to custom buffering stream
        sys.stdout = self.BufferingStream(self.log_file_handle)
        sys.stderr = self.BufferingStream(self.log_file_handle)

        # Start the server in a separate thread
        self.server_thread = threading.Thread(target=self._run_server)
        self.server_thread.daemon = True
        self.server_thread.start()

    class BufferingStream(io.StringIO):
        def __init__(self, file_handle):
            super().__init__()
            self.file_handle = file_handle

        def write(self, data: str):
            # Write data to the file
            super().write(data)
            self.file_handle.write(data)
            self.file_handle.flush()  # Ensure data is written to disk immediately

    class ConsoleHTTPRequestHandler(BaseHTTPRequestHandler):
        def _set_headers(self):
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()

        def do_GET(self):
            if self.path == '/logs':
                # Serve the log file content as HTML
                self._set_headers()
                with open(self.server.log_file, 'r') as log_file:
                    logs = log_file.read().replace('\n', '<br>')
                    self.wfile.write(json.dumps(logs).encode('utf-8'))
            elif self.path == '/':
                # Serve a simple HTML page for the console
                self.send_response(200)
                self.send_header('Content-type', 'text/html')
                self.end_headers()
                self.wfile.write(self._console_page().encode('utf-8'))
            else:
                self.send_response(404)

        def _console_page(self):
            return """
            <!DOCTYPE html>
            <html lang="en">
            <head>
                <meta charset="UTF-8">
                <meta http-equiv="X-UA-Compatible" content="IE=edge">
                <meta name="viewport" content="width=device-width, initial-scale=1.0">
                <title>Console Output</title>
                <style>
                    body { font-family: monospace; background-color: #333; color: #eee; }
                    #console { border: 1px solid #888; padding: 10px; width: 100%; height: 500px; background-color: #000; overflow-y: scroll; }
                </style>
            </head>
            <body>
                <div id="console"></div>
                <script>
                    function fetchLogs() {
                        fetch('/logs').then(response => response.json()).then(data => {
                            // Update console content and scroll to the bottom
                            const consoleElement = document.getElementById('console');
                            consoleElement.innerHTML = data;
                            consoleElement.scrollTop = consoleElement.scrollHeight;
                            setTimeout(fetchLogs, 1000);  // Poll every 1 second
                        });
                    }
                    fetchLogs();
                </script>
            </body>
            </html>
            """

    def _run_server(self):
        server_address = ('', self.port)
        httpd = HTTPServer(server_address, self.ConsoleHTTPRequestHandler)
        httpd.server = self  # Attach this instance to the handler
        print(f'Starting server on port {self.port}...')
        httpd.serve_forever()

    def close(self):
        """
        Closes the log file and stops the HTTP server gracefully.
        """
        if self.log_file_handle:
            self.log_file_handle.close()


# Example usage of the BufferedConsoleServer class
if __name__ == "__main__":
    console_server = BufferedConsoleServer(log_file='tmp_console_output.log', port=8080)

    # Simulate some logging in the main program
    try:
        for i in range(10):
            print(f"Simulated log message {i}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure proper cleanup
        console_server.close()
