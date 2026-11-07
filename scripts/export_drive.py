#!/usr/bin/env python3
"""Export a real attended drive from RTAB SQLite + recorded edge telemetry.

Read-only database access; no invented feature correspondences or ground truth.
Trajectory fit is SE(2), no scale correction, and measures disagreement only.
"""
import argparse
import base64
import collections
import json
import pathlib
import sqlite3
import numpy as np
from PIL import Image, ImageDraw
from export_visuals import feature_image, FONT


def align_se2(source, target):
    a, b = source.mean(axis=0), target.mean(axis=0)
    u, _, vt = np.linalg.svd((source-a).T @ (target-b))
    fix = np.eye(2); fix[-1, -1] = np.linalg.det(u @ vt)
    rotation = u @ fix @ vt
    return (source-a) @ rotation+b


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('database'); ap.add_argument('telemetry'); ap.add_argument('--output', required=True)
    args = ap.parse_args(); out = pathlib.Path(args.output); out.mkdir(parents=True, exist_ok=True)
    rows = [json.loads(l) for l in pathlib.Path(args.telemetry).read_text().splitlines()]
    moving = [r for r in rows if abs(r.get('wheel_speed', 0)) > .01]
    if not moving: raise SystemExit('No measured vehicle motion in telemetry')
    start, end = moving[0]['timestamp']-1, moving[-1]['timestamp']+1
    rows = [r for r in rows if start <= r['timestamp'] <= end]
    sessions = {r['control']['session'] for r in rows}
    if len(sessions) != 1: raise SystemExit('Split recordings at sensor restart before alignment')
    db = sqlite3.connect(pathlib.Path(args.database).resolve().as_uri()+'?mode=ro', uri=True)
    frames, frame_ids, previous = [], [], None
    query = '''SELECT Node.id, Node.stamp, Data.image FROM Node JOIN Data ON Node.id=Data.id
               WHERE Node.stamp BETWEEN ? AND ? ORDER BY Node.stamp'''
    for node, stamp, jpeg in db.execute(query, (start, end)):
        features = list(db.execute('SELECT word_id,pos_x,pos_y,size FROM Feature WHERE node_id=?', (node,)))
        counts = collections.Counter(f[0] for f in features)
        words = {w:[x,y] for w,x,y,_ in features if counts[w] == 1}
        current = dict(id=node, jpeg=base64.b64encode(jpeg).decode(),
                       keypoints=[[x,y,size] for _,x,y,size in features])
        pairs = [[words[w], previous[1][w]] for w in sorted(words.keys() & previous[1].keys())][:100] if previous else []
        nearest = min(rows, key=lambda r:abs(r['timestamp']-stamp))
        image = feature_image(dict(current=current, previous=previous[0] if previous else None,
                                   pairs=pairs, accepted_loop_id=nearest.get('loop_id',0)))
        frames.append(image); frame_ids.append(node); previous = current, words
    if frames:
        frames[0].save(out/'drive_features.gif', save_all=True, append_images=frames[1:], duration=500, loop=0)
        frames[len(frames)//2].save(out/'drive_features.png')

    rows = [r for r in rows if all(k in r for k in ['pose','wheel','ekf'])]
    slam = np.array([r['pose'][:2] for r in rows])
    paths = {'SLAM':slam}
    report = dict(motion_telemetry_samples=len(moving), route_rows=len(rows),
                  duration_s=end-start, retained_image_node_ids=frame_ids,
                  feature_gif_note='Retained drive keyframes, 0.5 s each; not original timing. Yellow lines are unique shared-word candidates, not verified inliers.',
                  evaluation_note='No ground truth. Best-fit SE(2), no scale; correlated baselines. Snapshot callbacks are asynchronous (about 1 Hz telemetry). Errors are trajectory disagreement, not accuracy.',
                  accepted_loop_ids=sorted({r.get('loop_id',0) for r in rows}-{0}),
                  max_measured_speed_m_s=max(abs(r.get('wheel_speed',0)) for r in moving))
    for key in ['wheel','ekf']:
        aligned = align_se2(np.array([r[key][:2] for r in rows]), slam)
        paths[key] = aligned; residual = np.linalg.norm(aligned-slam, axis=1)
        report[key+'_vs_slam_disagreement_m'] = dict(rmse=float(np.sqrt(np.mean(residual**2))),max=float(residual.max()))
    allpoints = np.concatenate(list(paths.values())); lo,hi = allpoints.min(axis=0),allpoints.max(axis=0)
    center = (lo+hi)/2; scale = min(800/max(hi[0]-lo[0],1),460/max(hi[1]-lo[1],1))
    def project(p): return tuple((np.array([480,365])+(p-center)*[scale,-scale]).tolist())
    animation=[]
    for i in range(len(rows)):
        im=Image.new('RGB',(960,640),'#08121b'); draw=ImageDraw.Draw(im)
        draw.text((24,18),'Actual drive | trajectory comparison | NOT ground-truth accuracy',font=FONT,fill='white')
        draw.text((24,42),'Best-fit SE(2) alignment per baseline; shared wheel/IMU inputs',font=FONT,fill='#ffcd76')
        for j,(key,points) in enumerate(paths.items()):
            color=['#5fffc3','#ad92ef','#faaf62'][j]
            draw.text((24+j*180,68),key,font=FONT,fill=color)
            coords=[project(p) for p in points[:i+1]]
            if len(coords)>1:draw.line(coords,fill=color,width=3)
            x,y=coords[-1];draw.ellipse((x-4,y-4,x+4,y+4),fill=color)
        draw.text((24,610),'Elapsed %.1f s | wheel speed %.3f m/s'%(rows[i]['timestamp']-start, rows[i].get('wheel_speed',0)),font=FONT,fill='white')
        animation.append(im)
    animation[0].save(out/'drive_trajectories.gif',save_all=True,append_images=animation[1:],duration=500,loop=0)
    animation[-1].save(out/'drive_trajectories.png')
    (out/'drive_metrics.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))


if __name__=='__main__': main()
