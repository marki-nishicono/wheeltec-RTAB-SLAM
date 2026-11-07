#!/usr/bin/env python3
"""Record actual edge outputs on the local PC; no sensor processing here."""
import argparse
import datetime
import gzip
import json
import pathlib
import time
import urllib.request


def get(endpoint):
    req = urllib.request.Request('http://127.0.0.1:18765/'+endpoint,headers={'Accept-Encoding':'gzip'})
    with urllib.request.urlopen(req,timeout=5) as r:
        data=r.read()
        if r.headers.get('Content-Encoding')=='gzip': data=gzip.decompress(data)
        return json.loads(data)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--seconds',type=int,default=60);p.add_argument('--output',default='')
    args=p.parse_args()
    dest=pathlib.Path(args.output or 'results/captures/'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
    dest.mkdir(parents=True,exist_ok=True)
    for i in range(args.seconds):
        started=time.monotonic()
        try:
            d={'local_timestamp':time.time(),'state':get('state'),'map':get('map'),'features':get('features')}
            with gzip.open(dest/('%04d.json.gz'%i),'wt') as f:json.dump(d,f,allow_nan=False)
        except Exception as e:print('capture error:',e,flush=True)
        time.sleep(max(0,1-(time.monotonic()-started)))
    print(dest)
