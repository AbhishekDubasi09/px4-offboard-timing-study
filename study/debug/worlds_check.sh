#!/usr/bin/env bash
cd ~/PX4-Autopilot/Tools/simulation/gz/worlds
for w in lawn walls forest windy; do echo "== $w ($(wc -l < $w.sdf) lines)"; grep -oE "<(albedo_map|texture|diffuse|uri|mesh|box|name)[^>]*>[^<]{0,70}" $w.sdf | sort | uniq -c | sort -rn | head -6; done
