#!/usr/bin/env bash
cd ~/study_results
sed 's/\x1b\[2K//g; s/pxh> //g' demo_px4.log | tr '\r' '\n' | grep -a -B4 -A2 "ivated" | cut -c1-200
echo ---
awk -F, '$2=="track"{n++; if (n%50==1) print $1,"sent="$3,"cmd="$4","$5,"v="$9","$10,"nav="$13}' demo.csv | head -14
