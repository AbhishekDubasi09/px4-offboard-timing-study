#!/usr/bin/env bash
cd ~/study_results
tr -d '\n ' < demo.json | cut -c1-260; echo
awk -F, 'NR>1 && NR%200==2 && $15!="" {printf "%s off=%.3f s phase=%s nav=%s\n", $1, ($16-$15)/1e6, $2, $13}' demo.csv | head -20
