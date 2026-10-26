#!/usr/bin/env bash
cd ~/study_results
awk -F, 'NR>1 && $1>21.5 && $1<24.3 {n++; if (n%25==1) print $1,$2,"sent="$3,"cmd="$4","$5}' demo.csv | head -14
echo "max tick dt:"; awk -F, 'NR>2 {d=$1-p; if (d>m) {m=d; at=$1}} {p=$1} END {print m, at}' demo.csv
