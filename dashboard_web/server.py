"""
LOCAL SERVER DASHBOARD SKRIPSI
Menjalankan Web Dashboard Analisis Data Skripsi pada http://localhost:8080
Otomatis membaca seluruh file CSV dari folder 'hasil_pengujian'.
"""

import os
import sys
import json
import urllib.parse
from http.server import HTTPServer, SimpleHTTPRequestHandler
import webbrowser

PORT = 8080
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
CSV_DIR = os.path.abspath(os.path.join(CURRENT_DIR, "..", "hasil_pengujian"))

class DashboardRequestHandler(SimpleHTTPRequestHandler):
    def do_GET(self):
        parsed_url = urllib.parse.urlparse(self.path)
        path = parsed_url.path
        query = urllib.parse.parse_qs(parsed_url.query)

        # 1. API: List Semua File CSV di hasil_pengujian
        if path == "/api/files":
            files = []
            if os.path.exists(CSV_DIR):
                for f in sorted(os.listdir(CSV_DIR), reverse=True):
                    if f.endswith(".csv"):
                        files.append(f)
            
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(files).encode("utf-8"))
            return

        # 2. API: Mengambil Isi File CSV Tertentu
        elif path == "/api/file":
            filename = query.get("name", [""])[0]
            # Mencegah directory traversal
            clean_name = os.path.basename(filename)
            file_path = os.path.join(CSV_DIR, clean_name)

            if os.path.exists(file_path):
                self.send_response(200)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                with open(file_path, "rb") as f:
                    self.wfile.write(f.read())
            else:
                self.send_response(404)
                self.end_headers()
                self.wfile.write(b"File not found")
            return

        # 3. Serve Frontend (index.html)
        elif path == "/" or path == "/index.html":
            index_path = os.path.join(CURRENT_DIR, "index.html")
            if os.path.exists(index_path):
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.end_headers()
                with open(index_path, "rb") as f:
                    self.wfile.write(f.read())
                return

        # Default handler untuk file statis lainnya
        return super().do_GET()

def run():
    os.chdir(CURRENT_DIR)
    server_address = ("", PORT)
    httpd = HTTPServer(server_address, DashboardRequestHandler)
    url = f"http://localhost:{PORT}"

    print("=" * 60)
    print("  DASHBOARD ANALISIS DATA SKRIPSI ESP32")
    print(f"  Server berjalan pada : {url}")
    print(f"  Folder CSV terbaca   : {CSV_DIR}")
    print("=" * 60)
    print("Membuka browser otomatis...")

    # Buka browser otomatis
    webbrowser.open(url)

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nMenutup server dashboard...")
        httpd.server_close()

if __name__ == "__main__":
    run()
