#!/usr/bin/env python3
"""Render recorded real features, graph and clouds locally, to PNG/GIF/PLY.

Candidate word matches are explicitly NOT called RANSAC inliers. No synthetic
trajectory or successful loop closure is invented when none was observed.
"""
import argparse,base64,gzip,io,json,pathlib
import numpy as np
from PIL import Image,ImageDraw,ImageFont

FONT=ImageFont.load_default()

def feature_image(f):
    c=f.get('current')
    if not c:return None
    a=Image.open(io.BytesIO(base64.b64decode(c['jpeg']))).convert('RGB')
    prev=f.get('previous'); b=Image.open(io.BytesIO(base64.b64decode(prev['jpeg']))).convert('RGB') if prev else Image.new('RGB',a.size,'#14212c')
    im=Image.new('RGB',(a.width+b.width,a.height+54),'#101c26');im.paste(a,(0,54));im.paste(b,(a.width,54));dr=ImageDraw.Draw(im)
    for x,y,*_ in c['keypoints']:dr.ellipse((x-3,y+51,x+3,y+57),outline='#5fffc3',width=1)
    for x,y in f.get('pairs',[]):dr.line((x[0],x[1]+54,y[0]+a.width,y[1]+54),fill='#d9af4c',width=1)
    dr.text((12,8),'RTAB-Map actual ORB features | frame %s / %s'%(c['id'],prev['id'] if prev else 'none'),font=FONT,fill='white')
    dr.text((12,28),'Shared-word candidates (NOT verified inliers) | accepted loop ID: %s'%f.get('accepted_loop_id',0),font=FONT,fill='#ffcd76')
    return im.resize((960,400))

def map_image(d,bounds):
    im=Image.new('RGB',(960,640),'#08121b');dr=ImageDraw.Draw(im);xmin,xmax,ymin,ymax=bounds
    scale=min(880/max(3,xmax-xmin),530/max(3,ymax-ymin));cx=(xmin+xmax)/2;cy=(ymin+ymax)/2
    def pt(x,y):return (480+(x-cx)*scale,350-(y-cy)*scale)
    for p in d['map'].get('cloud',[]):
        q=pt(*p[:2]);dr.point(q,fill=tuple(int(v) for v in p[3:6]))
    graph=d['map'].get('graph',{})
    for e in d['map'].get('links',[]):
        a,b=graph.get(str(e['a'])),graph.get(str(e['b']))
        if a and b:dr.line((*pt(a[0][3],a[1][3]),*pt(b[0][3],b[1][3])),fill='#61e3bc' if e['type']==0 else '#ff5074',width=2)
    for p in d['state'].get('scan',[]):dr.point(pt(*p[:2]),fill='#77d9ed')
    dr.text((20,15),'Actual laboratory data | laser + RGB-D | planar pose model',font=FONT,fill='white')
    dr.text((20,35),'Nodes: %s | points: %s | accepted loop: %s'%(d['state'].get('graph_nodes',0),len(d['map'].get('cloud',[])),d['state'].get('loop_id',0)),font=FONT,fill='#ffcd76')
    return im


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('recording');ap.add_argument('--output',default='results/visuals');args=ap.parse_args()
    out=pathlib.Path(args.output);out.mkdir(parents=True,exist_ok=True)
    files=sorted(pathlib.Path(args.recording).glob('*.json.gz'))
    data=[json.load(gzip.open(p,'rt')) for p in files]
    if not data:raise SystemExit('No recorded data')
    cloud=np.asarray(data[-1]['map'].get('cloud',[]))
    if len(cloud):
        bounds=(cloud[:,0].min()-.5,cloud[:,0].max()+.5,cloud[:,1].min()-.5,cloud[:,1].max()+.5)
        with open(out/'map.ply','w') as f:
            f.write('ply\nformat ascii 1.0\nelement vertex %s\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n'%len(cloud))
            for p in cloud:f.write('%.4f %.4f %.4f %d %d %d\n'%tuple(p))
    else:bounds=(-3,3,-3,3)
    features=[x for x in (feature_image(d['features']) for d in data) if x is not None]
    maps=[map_image(d,bounds) for d in data]
    for name,frames in [('features',features),('mapping',maps)]:
        if not frames:continue
        frames[-1].save(out/(name+'.png'))
        frames[0].save(out/(name+'.gif'),save_all=True,append_images=frames[1:],duration=1000,loop=0)
    stats={}; seen_updates=set()
    for d in data:
        state=d['state']
        # Polling can return the same RTAB update more than once. A ref ID is
        # unique within a control session; exclude stale backend snapshots.
        update=(state.get('control',{}).get('session'),state.get('ref_id'))
        if update[1] is None or update in seen_updates or state.get('ages',{}).get('rtabmap',999)>3:continue
        seen_updates.add(update)
        for k,v in d['state'].get('stats',{}).items():
            if 'Timing/' in k:stats.setdefault(k,[]).append(v)
    report={'samples':len(data),'unique_sampled_backend_updates':len(seen_updates),
            'timing_scope':'sampled fresh unique ref IDs; not all backend updates',
            'accepted_loop_ids':sorted({d['state'].get('loop_id',0) for d in data}-{0}),
            'max_graph_nodes':max(d['state'].get('graph_nodes',0) for d in data),
            'timing_ms':{k:{'median':float(np.median(v)),'p95':float(np.percentile(v,95)),'max':float(max(v))} for k,v in stats.items()}}
    (out/'metrics.json').write_text(json.dumps(report,indent=2))
    print(out)
