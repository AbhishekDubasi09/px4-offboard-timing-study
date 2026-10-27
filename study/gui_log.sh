#!/usr/bin/env bash
grep -av "gz_frame_id" /tmp/gzgui.log | sed 's/\x1b\[[0-9;]*m//g' | grep -aiE "err|unable|fail|mesh|not found|missing|texture|resource" | head -12 | cut -c1-220
echo "--- env in server process:"; tr '\0' '\n' < /proc/$(pgrep -x gz | head -1)/environ 2>/dev/null | grep -E "GZ_SIM_RESOURCE_PATH" | cut -c1-200
echo "--- gz procs:"; pgrep -fa "gz sim" | cut -c1-110
