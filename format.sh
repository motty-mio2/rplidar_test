#!/usr/bin/env bash

root_dir="$(dirname "$0")"

find "$root_dir/include" "$root_dir/src" -regextype posix-extended -regex '.*/.*\.(c|h|cpp|hpp)$' -exec clang-format --style=file -i {} +
