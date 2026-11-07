#!/bin/bash
# Make an offline bundle (no GHCR needed) from images built with scripts/build.sh:
#
#   scripts/bundle.sh linux/amd64 lab1_week5 [lab ...]
#
# Writes dist/pas_roboracer_<arch>/ with the images tarball, compose.yaml,
# .env (first lab selected) and the student README. dist/ is git-ignored.
set -euo pipefail

PLATFORM=${1:?usage: $0 <linux/amd64|linux/arm64> <lab> [lab ...]}
shift
[ $# -gt 0 ] || { echo "name at least one lab" >&2; exit 1; }
REGISTRY=${REGISTRY:-ghcr.io/crta-lab}
TAG=${TAG:-jazzy}
ARCH=${PLATFORM#linux/}
ROOT=$(cd "$(dirname "$0")/.." && pwd)
OUT="$ROOT/dist/pas_roboracer_$ARCH"

IMAGES=("$REGISTRY/f1tenth-sim:$TAG")
for lab in "$@"; do IMAGES+=("$REGISTRY/f1tenth-lab:$lab"); done

for image in "${IMAGES[@]}"; do
    arch=$(docker run --rm --platform "$PLATFORM" --entrypoint dpkg "$image" --print-architecture)
    [ "$arch" = "$ARCH" ] || { echo "$image is $arch, not $ARCH: rebuild with scripts/build.sh $PLATFORM" >&2; exit 1; }
done

mkdir -p "$OUT"
cp "$ROOT/class/compose.yaml" "$ROOT/class/README.md" "$OUT/"
printf 'REGISTRY=%s\nLAB=%s\n' "$REGISTRY" "$1" > "$OUT/.env"
docker save --platform "$PLATFORM" "${IMAGES[@]}" | gzip -1 > "$OUT/pas_roboracer_$ARCH.tar.gz"
echo "Bundle: $OUT"
echo "On the target machine: docker load -i pas_roboracer_$ARCH.tar.gz && docker compose up -d"
ls -lh "$OUT"
