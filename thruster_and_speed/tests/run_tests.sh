#!/bin/sh
set -eu

test_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
build_dir=$(mktemp -d "${TMPDIR:-/tmp}/mqtt-command-gate.XXXXXX")
cleanup() {
  find "$build_dir" -depth -delete
}
trap cleanup EXIT HUP INT TERM

"${CXX:-c++}" \
  -std=c++11 \
  -Wall \
  -Wextra \
  -Werror \
  "$test_dir/mqtt_command_gate_test.cpp" \
  -o "$build_dir/mqtt_command_gate_test"

"$build_dir/mqtt_command_gate_test"
