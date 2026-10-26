#!/usr/bin/env bash
cd ~/study_results
awk -F, 'NR>1 && $3==1 {if (p>0) {d=$1-p; if (d>0.12) print "gap",d,"at",p}; p=$1}' demo.csv | head
echo "sent count:"; awk -F, 'NR>1 && $3==1' demo.csv | wc -l
echo "last rows:"; tail -3 demo.csv | cut -d, -f1,2,3,13
sed 's/\x1b\[2K//g; s/pxh> //g' demo_px4.log | tr '\r' '\n' | grep -a -iE "time offset|synchron|reset|lockstep" | head -5 | cut -c1-160
echo UXRCE_DDS_SYNCT: $(~/PX4-Autopilot/build/px4_sitl_default/bin/px4-param show UXRCE_DDS_SYNCT 2>&1 | tail -1)
