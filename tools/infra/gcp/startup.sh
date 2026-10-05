#!/bin/bash
# 러너 VM 시작 스크립트 (인스턴스 템플릿 metadata.startup-script).
# Docker와 GitHub Actions 러너를 설치하고, 메타데이터 jit-config로 일회용 러너를 등록한다.
# JIT 러너는 작업 하나를 끝내면 스스로 등록을 지우고 끝난다. 그 뒤 VM을 끈다(워크플로가 VM을 지운다).
# 로그: sudo journalctl -u google-startup-scripts
set -euo pipefail

metadata() {
  curl -fsS -H 'Metadata-Flavor: Google' \
    "http://metadata.google.internal/computeMetadata/v1/instance/attributes/$1"
}

JIT_CONFIG="$(metadata jit-config)"

# Docker 엔진 + buildx + compose 플러그인
curl -fsSL https://get.docker.com | sh

useradd -m -s /bin/bash runner || true
usermod -aG docker runner

# Artifact Registry 로그인 도우미 (VM 서비스 계정으로 이미지 캐시를 읽고 쓴다)
sudo -u runner gcloud auth configure-docker asia-northeast3-docker.pkg.dev --quiet

# 최신 러너. 오래된 러너는 GitHub이 작업을 주지 않는다
version="$(curl -fsS https://api.github.com/repos/actions/runner/releases/latest | grep -oP '"tag_name": "v\K[^"]+')"
dir=/home/runner/actions-runner
mkdir -p "$dir"
curl -fsSL "https://github.com/actions/runner/releases/download/v${version}/actions-runner-linux-x64-${version}.tar.gz" \
  | tar xz -C "$dir"
"$dir/bin/installdependencies.sh"
chown -R runner:runner "$dir"

cd "$dir"
sudo -u runner ./run.sh --jitconfig "$JIT_CONFIG" || true

# 작업이 끝났다. 끄기만 해도 컴퓨팅 요금은 멈춘다. 디스크까지 지우는 것은 워크플로 stop-runner가 한다
poweroff
