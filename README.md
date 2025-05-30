# Piper Development on Jetson

## Prerequisites

1. [astral uv](https://docs.astral.sh/uv/getting-started/installation/)

2. install can tool

```bash
sudo apt update && sudo apt install can-utils ethtool
```

## Get started

1. Install dependencies
```bash
source piper/bin/activate
uv pip install -e .
```