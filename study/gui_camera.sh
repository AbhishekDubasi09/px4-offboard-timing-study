#!/usr/bin/env bash
# Follow camera on the x500: gui_camera.sh <x> <y> <z> offset from the vehicle (world frame, metres).
export GZ_IP=127.0.0.1
gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 3000 --req 'data: "x500_0"'
gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean --timeout 3000 --req "x: $1 y: $2 z: $3"
