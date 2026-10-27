#!/usr/bin/env bash
S=~/.local/lib/python3.12/site-packages
echo "entries in user site-packages: $(ls $S | wc -l)"
echo "entries OLDER than my session (could be yours):"; find $S -maxdepth 1 -not -newermt '2026-10-26' -not -path "$S" | sed 's#.*/##' | head -20
echo "--- other user-site dirs / venvs that existed before?"; ls -d ~/.local/lib/python3.*/ 2>/dev/null
echo "--- ~/.wslconfig on Windows:"; cat /mnt/c/Users/abhib/.wslconfig 2>/dev/null
