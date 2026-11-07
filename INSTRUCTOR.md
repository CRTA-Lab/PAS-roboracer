# PAS RoboRacer

F1TENTH simulator and weekly labs for the PAS class (ROS 2 Jazzy, Docker).

**Students:** everything you need is in [class/](class/README.md). Weekly slides and homework are in `vjezbe_tjedan_N/`.

## Layout

```
PAS-roboracer/
├── simulator/                  f1tenth_gym_ros (Jazzy) + the base and sim images
│   ├── docker/base.Dockerfile  → ghcr.io/crta-lab/f1tenth-base   ROS 2 + all apt deps
│   └── Dockerfile              → ghcr.io/crta-lab/f1tenth-sim    gym + bridge
├── class/                      what students clone and run
│   ├── compose.yaml, .env      sim + lab + noVNC; LAB= picks the week
│   ├── docker/lab.Dockerfile   → ghcr.io/crta-lab/f1tenth-lab:<lab>
│   └── labs/
│       └── lab1_week5/         wall_follow
├── scripts/
│   ├── build.sh                local builds for linux/arm64 or linux/amd64
│   └── bundle.sh               offline tarball bundle → dist/
└── .github/workflows/
    ├── simulator.yml           builds + pushes base and sim (amd64)
    └── labs.yml                builds + pushes one image per class/labs/ folder (amd64)
```

The lab images and the sim share the `f1tenth-base` layers, so a new lab is a
download of about 100 KB on a student PC.

## Building

GitHub Actions builds linux/amd64 (what students run) and pushes to GHCR:

| Change | Workflow | Result |
|---|---|---|
| `simulator/**` on `main` | `simulator.yml`, then `labs.yml` | new base + sim, and every lab rebuilt on that base |
| `class/**` on `main` | `labs.yml` | every lab image rebuilt |
| git tag `vX.Y` | `simulator.yml` | base + sim also tagged `:vX.Y` |

Locally (tags match CI, so `class/compose.yaml` runs what you built last):

```bash
scripts/build.sh linux/arm64
```

```bash
scripts/build.sh linux/amd64
```

arm64 is native on Apple Silicon and is what you test with. amd64 on a Mac is
emulated (about 30 min for the sim). Don't use `DOCKER_DEFAULT_PLATFORM` with
`docker compose build`: it can label an arm64 build as amd64. The scripts check
the real architecture of each image they build.

Run it like a student:

```bash
cd class && docker compose up -d
```

Offline bundle for machines without GHCR access (images must be built for that platform first):

```bash
scripts/bundle.sh linux/amd64 lab1_week5
```

## Adding a lab

1. Create `class/labs/labN_weekM/` with its ROS 2 package(s). The folder name is the image tag.
2. Push to `main`. `labs.yml` publishes `ghcr.io/crta-lab/f1tenth-lab:labN_weekM`.
3. Students set `LAB=labN_weekM` in `class/.env` and run `docker compose up -d`.

To fix a lab students may already have started, publish it under a new name
(`lab1_week5_v2`): an existing local tag is only refreshed by `docker compose pull lab`,
and students' `./work/<lab>` copies are never overwritten.

Anything a lab needs from apt belongs in `simulator/docker/base.Dockerfile`, so all labs share it.

## One-time GitHub setup

- Repo → Settings → Actions → General → Workflow permissions: **Read and write**.
- Org → Settings → Packages: allow members to create public packages.
- After the first run, set the `f1tenth-base`, `f1tenth-sim` and `f1tenth-lab` packages to **Public**.
- On the very first push both workflows start together and `labs.yml` fails because
  the base doesn't exist yet. It re-runs by itself when `simulator.yml` succeeds.
