#!/bin/bash
# Build base + sim + lab images locally for one platform.
#
#   scripts/build.sh linux/arm64                  # Apple Silicon, native: for testing
#   scripts/build.sh linux/amd64                  # student PCs (emulated on a Mac, slow)
#   scripts/build.sh linux/amd64 lab1_week5       # only these labs (default: all)
#   SKIP_SIM=1 scripts/build.sh linux/arm64 lab2_week6   # labs only, reuse local base
#
# Tags are the same as CI's (ghcr.io/crta-lab/...:jazzy), so class/compose.yaml
# runs whatever you built last. Images for the other platform get overwritten.
set -euo pipefail

PLATFORM=${1:?usage: $0 <linux/amd64|linux/arm64> [lab ...]}
shift
export REGISTRY=${REGISTRY:-ghcr.io/crta-lab}
export TAG=${TAG:-jazzy}
ROOT=$(cd "$(dirname "$0")/.." && pwd)

if [ -z "${SKIP_SIM:-}" ]; then
    "$ROOT/simulator/docker/build-platform.sh" "$PLATFORM"
fi

cd "$ROOT/class"
LABS=("$@")
if [ ${#LABS[@]} -eq 0 ]; then
    for dir in labs/*/; do LABS+=("$(basename "$dir")"); done
fi

for lab in "${LABS[@]}"; do
    docker build --platform "$PLATFORM" -f docker/lab.Dockerfile \
        --build-arg LAB="$lab" \
        --build-arg BASE_IMAGE="$REGISTRY/f1tenth-base:$TAG" \
        -t "$REGISTRY/f1tenth-lab:$lab" .
    echo "$REGISTRY/f1tenth-lab:$lab: $(docker run --rm --platform "$PLATFORM" \
        --entrypoint dpkg "$REGISTRY/f1tenth-lab:$lab" --print-architecture)"
done
