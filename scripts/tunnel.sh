#!/usr/bin/env bash
set -euo pipefail
robot="${1:-wheeltec@192.168.10.101}"
# Separate TCP transports: video packet loss must not queue driving packets.
ssh -N -o ControlMaster=auto -o ControlPersist=3600 -o ControlPath=/tmp/wheeltec-control-ssh-%C -o ExitOnForwardFailure=yes -o ServerAliveInterval=5 -o ServerAliveCountMax=3 -L 127.0.0.1:18766:127.0.0.1:8766 -L 127.0.0.1:18767:127.0.0.1:8767 "$robot" &
control_pid=$!
trap 'kill "$control_pid" 2>/dev/null || true' EXIT
ssh -N -o ControlMaster=auto -o ControlPersist=3600 -o ControlPath=/tmp/wheeltec-slam-ssh-%C -o ExitOnForwardFailure=yes -o ServerAliveInterval=5 -o ServerAliveCountMax=3 -L 127.0.0.1:18765:127.0.0.1:8765 "$robot"
