#!/usr/bin/env bash
# Сборка проекта. Патч fontdue (E0599 cast_signed) лежит в vendor/fontdue-0.9.5
# и подключается через [patch.crates-io] в Cargo.toml — ничего чинить в ~/.cargo не нужно.
set -e
CMD="${1:-build}"
if [ "$CMD" = "run" ] || [ "$CMD" = "build" ]; then
  cargo $CMD --release "${@:2}"
else
  cargo $CMD "${@:2}"
fi
