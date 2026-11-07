#!/usr/bin/env python3
"""Read-only telemetry probe. Never publishes drive or stop commands."""
import http.client,json,time,uuid,pathlib,argparse
p=argparse.ArgumentParser();p.add_argument('--seconds',type=int,default=45);p.add_argument('--output',default='results/control_probe.json');a=p.parse_args()
c=http.client.HTTPConnection('127.0.0.1',8080,timeout=1);rows=[];end=time.monotonic()+a.seconds;client=uuid.uuid4().hex;seq=0
while time.monotonic()<end:
 start=time.monotonic()
 try:
  c.request('GET','/api/control');r=c.getresponse();reply=json.loads(r.read())
  rows.append(dict(ms=round(1000*(time.monotonic()-start),1),status=r.status,session=reply['control']['session'],ages=reply['ages'],block=reply['drive_block']))
 except Exception as e:rows.append(dict(error=str(e)));c.close();c=http.client.HTTPConnection('127.0.0.1',8080,timeout=1)
 time.sleep(max(0,.1-(time.monotonic()-start)))
v=sorted(x['ms'] for x in rows if 'ms' in x);report=dict(samples=len(rows),errors=sum('error' in x for x in rows),sessions=len(set(x.get('session') for x in rows)),p50_ms=v[len(v)//2] if v else None,p95_ms=v[int(len(v)*.95)] if v else None,max_ms=max(v) if v else None,rows=rows)
pathlib.Path(a.output).write_text(json.dumps(report,indent=2));print({k:v for k,v in report.items() if k!='rows'})
