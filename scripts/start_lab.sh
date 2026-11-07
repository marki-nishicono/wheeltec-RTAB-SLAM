#!/usr/bin/env bash
# Run from a local terminal. SSH prompts for credentials when needed; none saved here.
set -euo pipefail
cd "$(dirname "$0")/.."
robot="${1:-wheeltec@192.168.10.101}"
data_socket=/tmp/wheeltec-slam-ssh-%C
control_socket=/tmp/wheeltec-control-ssh-%C
for socket in "$control_socket" "$data_socket"; do
 if ! ssh -S "$socket" -O check "$robot" 2>/dev/null; then
  ssh -M -S "$socket" -o ConnectTimeout=8 -o ControlPersist=3600 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 -fN "$robot"
 fi
done
forward() {
 local socket="$1" port="$2" remote="$3"
 if ! ss -ltnH "sport = :$port" | grep -q .; then
  ssh -S "$socket" -O forward -L "127.0.0.1:$port:127.0.0.1:$remote" "$robot"
 fi
}
forward "$control_socket" 18766 8766
forward "$control_socket" 18767 8767
forward "$data_socket" 18765 8765
ssh -S "$control_socket" "$robot" 'systemctl --user start wheeltec-lab-master wheeltec-lab-sensors wheeltec-lab-camera wheeltec-lab-mapping'
mkdir -p results
if ! ss -ltnH 'sport = :8080' | grep -q .; then
 nohup /usr/bin/python3 scripts/local_server.py >results/local_server.log 2>&1 </dev/null &
fi
if ! ss -ltnH 'sport = :8081' | grep -q .; then
 relay_python=$(command -v python3)
 if ! "$relay_python" -c 'import websockets.legacy.server' 2>/dev/null; then
  relay_python=/home/marki/anaconda3/bin/python3
 fi
 "$relay_python" -c 'import websockets.legacy.server'
 nohup "$relay_python" scripts/control_relay.py >results/control_relay.log 2>&1 </dev/null &
fi
echo '实验台入口：http://127.0.0.1:8080/ 。等待设备检查通过后，新建地图或选择旧地图继续。'
if command -v xdg-open >/dev/null; then xdg-open http://127.0.0.1:8080/ >/dev/null 2>&1 || true; fi
