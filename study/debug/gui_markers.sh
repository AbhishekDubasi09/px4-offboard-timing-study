#!/usr/bin/env bash
# Add reference objects to the running Gazebo world: posts at the corners of the 4 m flight square, a landing pad.
# usage: gui_markers.sh <world name>
export GZ_IP=127.0.0.1
W=$1
add() { # name, x, y, z, shape-xml, rgb
  gz service -s /world/$W/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 3000 \
    --req "sdf: \"<sdf version='1.9'><model name='$1'><static>true</static><pose>$2 $3 $4 0 0 0</pose><link name='l'><visual name='v'><geometry>$5</geometry><material><ambient>$6 1</ambient><diffuse>$6 1</diffuse></material></visual></link></model></sdf>\"" | tr -d '\n'; echo " $1"
}
POST="<cylinder><radius>0.12</radius><length>2.4</length></cylinder>"
add post_a 0 4 1.2 "$POST" "1 0.45 0.05"
add post_b 4 4 1.2 "$POST" "1 0.45 0.05"
add post_c 4 0 1.2 "$POST" "1 0.45 0.05"
add post_d 0 -2 1.2 "$POST" "0.1 0.4 1"
add pad 0 0 0.01 "<box><size>2 2 0.02</size></box>" "0.95 0.95 0.95"
