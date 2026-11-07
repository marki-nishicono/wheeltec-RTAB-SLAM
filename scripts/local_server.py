#!/usr/bin/env python3
"""Local-only static renderer and same-origin proxy to SSH forwarded edge API."""
import argparse
import http.client
import queue
import time
import pathlib
import urllib.request
import urllib.error
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer


class Handler(SimpleHTTPRequestHandler):
    upstream = 'http://127.0.0.1:18765'
    control_pool = queue.LifoQueue(maxsize=4)
    protocol_version = 'HTTP/1.1'
    disable_nagle_algorithm = True
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(pathlib.Path(__file__).resolve().parents[1]/'web'), **kwargs)
    def authorized(self):
        host = self.headers.get('Host','')
        return host in ('127.0.0.1:8080','localhost:8080') and self.headers.get('Origin','http://'+host) == 'http://'+host
    def do_GET(self):
        if not self.authorized(): self.send_error(403); return
        if self.path.startswith('/api/'): self.proxy('GET')
        else: super().do_GET()
    def do_POST(self):
        if not self.authorized() or self.path not in ('/api/drive','/api/map-session') or self.headers.get('X-Lab-Control')!='deadman-v1':
            self.send_error(403); return
        self.proxy('POST')
    def proxy(self, method):
        path = self.path[4:]
        if path.split('?')[0] not in ('/control','/state','/map','/maps','/map-session','/features','/rgb.jpg','/rgb.mjpg','/video.h264','/drive'):
            self.send_error(404); return
        if path in ('/control','/drive'):
            return self.control_proxy(path,method)
        try:
            n = int(self.headers.get('Content-Length',0))
            if n>1024: self.send_error(413); return
            data = self.rfile.read(n) if method=='POST' else None
            req = urllib.request.Request(self.upstream+path, data=data, method=method,
                   headers={'X-Lab-Control':'deadman-v1','Content-Type':'application/json','Accept-Encoding':'gzip'})
            with urllib.request.urlopen(req, timeout=3) as r:
                if path.split('?')[0] in ('/rgb.mjpg','/video.h264'):
                    self.send_response(r.status)
                    self.send_header('Content-Type',r.headers['Content-Type'])
                    self.send_header('Cache-Control','no-store');self.send_header('Connection','close'); self.end_headers()
                    self.close_connection=True
                    self.connection.settimeout(2)
                    try:
                        while True:
                            chunk=r.read1(65536)
                            if not chunk:break
                            self.wfile.write(chunk); self.wfile.flush()
                    except OSError: pass
                    return
                body = r.read(); self.send_response(r.status)
                self.send_header('Content-Type',r.headers.get('Content-Type','application/json'))
                if r.headers.get('Content-Encoding'): self.send_header('Content-Encoding',r.headers['Content-Encoding'])
                self.send_header('Content-Length',str(len(body)));self.send_header('Cache-Control','no-store')
                self.end_headers(); self.wfile.write(body)
        except urllib.error.HTTPError as e:
            body=e.read();self.send_response(e.code)
            self.send_header('Content-Type',e.headers.get('Content-Type','text/plain'))
            self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
        except (urllib.error.URLError,TimeoutError,OSError):
            try: self.send_error(502,'Vehicle connection unavailable')
            except OSError: pass
    def control_proxy(self,path,method):
        conn=None
        try:
            n=int(self.headers.get('Content-Length',0))
            if not 0<=n<=1024:self.send_error(413);return
            data=self.rfile.read(n) if method=='POST' else None
            try:
                conn,created=self.control_pool.get_nowait()
                if time.monotonic()-created>1.5:
                    conn.close()
                    conn=http.client.HTTPConnection('127.0.0.1',18766,timeout=.8)
            except queue.Empty:conn=http.client.HTTPConnection('127.0.0.1',18766,timeout=.8)
            conn.request(method,path,body=data,headers={'X-Lab-Control':'deadman-v1','Content-Type':'application/json'})
            r=conn.getresponse();body=r.read()
            self.send_response(r.status);self.send_header('Content-Type','application/json')
            self.send_header('Content-Length',str(len(body)));self.send_header('Cache-Control','no-store')
            self.end_headers();self.wfile.write(body)
            try:self.control_pool.put_nowait((conn,time.monotonic()));conn=None
            except queue.Full:pass
        except (OSError,http.client.HTTPException,ValueError):
            # Never retry a movement request: the browser obtains a fresh lease.
            try:self.send_error(502,'Control connection unavailable')
            except OSError:pass
        finally:
            if conn:conn.close()
    def log_message(self,*args): pass


class LocalServer(ThreadingHTTPServer):
    request_queue_size = 64
    daemon_threads = True


if __name__ == '__main__':
    LocalServer(('127.0.0.1',8080),Handler).serve_forever()
