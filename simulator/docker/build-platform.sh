#!/bin/bash
# Build the base and sim images for one platform, e.g. linux/amd64 on an
# Apple Silicon Mac (emulated, slow). `docker compose build` with
# DOCKER_DEFAULT_PLATFORM is not reliable for this: it can tag an image as
# amd64 that was built on the native arm64 base.
#
#   docker/build-platform.sh linux/amd64
#   REGISTRY=ghcr.io/crta-lab TAG=jazzy docker/build-platform.sh linux/amd64
set -euo pipefail

PLATFORM=${1:?usage: $0 <platform, e.g. linux/amd64>}
REGISTRY=${REGISTRY:-ghcr.io/crta-lab}
TAG=${TAG:-jazzy}
GYM_REF=${GYM_REF:-dev-jax}
cd "$(dirname "$0")/.."

docker build --platform "$PLATFORM" -f docker/base.Dockerfile \
    -t "$REGISTRY/f1tenth-base:$TAG" docker

docker build --platform "$PLATFORM" \
    --build-context base="docker-image://$REGISTRY/f1tenth-base:$TAG" \
    --build-arg GYM_REF="$GYM_REF" \
    -t "$REGISTRY/f1tenth-sim:$TAG" .

for image in f1tenth-base f1tenth-sim; do
    echo "$REGISTRY/$image:$TAG: $(docker run --rm --platform "$PLATFORM" \
        --entrypoint dpkg "$REGISTRY/$image:$TAG" --print-architecture)"
done
