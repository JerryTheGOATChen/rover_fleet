#!/usr/bin/env bash
# Film a fleet run in RViz, off-screen, and make a captioned, sped-up video.
#
#   terminal 1:  ros2 launch rover_bringup fleet.launch.py gui:=false
#   terminal 2:  tools/rviz_recorder/record_fleet.sh [seconds=510] [out_dir=./rviz_out]
#
# Start terminal 2 right after terminal 1. Both must use the same ROS_DOMAIN_ID.
# 510 s covers the 240 s mission plus the drive home at Gazebo's ~0.65x real time
# with 3 robots. Output: fleet_raw.mp4, fleet_rviz.mp4 (cropped, 4x, captioned) and
# fleet_rviz.gif. Needs only Docker.
set -euo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)"
SECONDS_TO_RECORD=${1:-510}
OUT="$(mkdir -p "${2:-rviz_out}" && cd "${2:-rviz_out}" && pwd)"
IMAGE=rover-rviz-recorder

docker build -q -t "$IMAGE" "$DIR" >/dev/null
docker run --rm --net=host --ipc=host -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 -v "$DIR":/cfg:ro -v "$OUT":/out \
  "$IMAGE" bash /cfg/record.sh fleet_raw "$SECONDS_TO_RECORD"

printf 'Rover Fleet  ·  3 robots  ·  learned policy  ·  5 m radio  ·  4x speed' > "$OUT/top.txt"
printf 'green lines: radio links     thin lines: Nav2 paths     light floor: what the base station knows' > "$OUT/bottom.txt"
docker run --rm -v "$OUT":/out --entrypoint bash "$IMAGE" -c '
F=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf
# skip RViz starting up, crop to the map, 4x speed, caption bars
ffmpeg -loglevel error -y -ss 10 -i /out/fleet_raw.mp4 \
  -vf "crop=900:628:190:62,setpts=PTS/4,fps=25,pad=900:700:0:36:color=0x1b1f23,drawtext=fontfile=$F:textfile=/out/top.txt:x=14:y=10:fontsize=18:fontcolor=white,drawtext=fontfile=$F:textfile=/out/bottom.txt:x=14:y=673:fontsize=15:fontcolor=0xcfd8dc" \
  -c:v libx264 -preset slow -crf 26 -pix_fmt yuv420p -movflags +faststart /out/fleet_rviz.mp4
ffmpeg -loglevel error -y -i /out/fleet_rviz.mp4 \
  -vf "setpts=PTS/2,fps=5,scale=480:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=48[p];[b][p]paletteuse=dither=none" \
  /out/fleet_rviz.gif'
echo "wrote $OUT/fleet_rviz.mp4 and $OUT/fleet_rviz.gif"
