#!/usr/bin/env bash

root_dir="$(dirname "$0")"

cd "$root_dir/build" || return

find "../include" "../src" -regextype posix-extended -regex '.*/.*\.(c|h|cpp|hpp)$' -exec clang-tidy {} +
