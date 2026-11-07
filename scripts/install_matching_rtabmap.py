#!/usr/bin/env python3
"""Run on ROS Noetic ARM64 vehicle. Extract matching ROS binaries locally.

The system dpkg database is not changed. Download metadata from the final ROS
Noetic snapshot. Existing apt trust can verify the signed index externally;
each downloaded package is checked against the selected index SHA256 here.
"""
import gzip,hashlib,pathlib,subprocess,urllib.request

root=pathlib.Path.home()/'wheeltec_slam_runtime/vendor'
root.mkdir(parents=True,exist_ok=True)
base='http://snapshots.ros.org/noetic/final/ubuntu/'
raw=urllib.request.urlopen(base+'dists/focal/main/binary-arm64/Packages.gz',timeout=45).read()
index=gzip.decompress(raw).decode();(root/'Packages').write_text(index)
names=['ros-noetic-rtabmap','ros-noetic-gtsam','ros-noetic-rtabmap-util',
       'ros-noetic-rtabmap-conversions','ros-noetic-rtabmap-sync','ros-noetic-rtabmap-slam','ros-noetic-rtabmap-msgs']
for name in names:
    paragraph=next(p for p in index.split('\n\n') if p.startswith('Package: '+name+'\n'))
    values=dict(l.split(': ',1) for l in paragraph.splitlines() if ': ' in l)
    data=urllib.request.urlopen(base+values['Filename'],timeout=90).read()
    if hashlib.sha256(data).hexdigest()!=values['SHA256']:raise RuntimeError('Checksum mismatch: '+name)
    target=root/(name+'.deb');target.write_bytes(data)
    subprocess.run(['dpkg-deb','-x',str(target),str(root/'rtabmap-0.21.13')],check=True)
    print(name,values['Version'],values['SHA256'])
