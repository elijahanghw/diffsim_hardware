#!/usr/bin/env bash
#
# sync_cnn.sh — scp the (uncommitted) CNN encoder files to the drone for quick
# iteration, without a git commit/bundle. The CNN is a generated artifact, so
# committing every weight tweak is just noise; copy the two files and rebuild.
#
#   scripts/sync_cnn.sh                          # to the default host below
#   scripts/sync_cnn.sh radxa@192.168.1.50       # to another host
#   scripts/sync_cnn.sh radxa@host /opt/diffsim  # host + remote repo path
#
# Or set a default host once:  export RADXA_HOST=radxa@radxa-cubie-a7z
#
# After it copies, rebuild on the drone:
#   make USE_RELAY=1 USE_VO=1 NO_DISPLAY=1
# Note: the copied files leave the drone's working tree dirty; before a later
# bundle/pull, run  git checkout -- src/cnn/cnn_encoder.*  there if git complains.

set -euo pipefail

host="${1:-${RADXA_HOST:-radxa@radxa-cubie-a7z}}"
remote="${2:-~/diffsim_hardware}"

root="$(git -C "$(dirname "$0")" rev-parse --show-toplevel)"
src="$root/src/cnn"
for f in cnn_encoder.c cnn_encoder.h; do
    [ -f "$src/$f" ] || { echo "error: missing $src/$f" >&2; exit 1; }
done

echo "copying cnn_encoder.{c,h} -> $host:$remote/src/cnn/"
scp "$src/cnn_encoder.c" "$src/cnn_encoder.h" "$host:$remote/src/cnn/"
echo "done. on $host:  cd $remote && make USE_RELAY=1 USE_VO=1 NO_DISPLAY=1"
