#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
foc_src="${SIMPLEFOC_SRC:-.pio/libdeps/esp32dev/Arduino-FOC/src}"
binary=$(mktemp /tmp/attitude-sensor-test.XXXXXX)
trap 'rm -f "$binary"' EXIT
c++ -std=c++17 -Wall -Wextra -Werror -Iinclude -Itests/wheel_host -I"$foc_src" \
  tests/attitude_host/sensor_test.cpp "$foc_src/common/base_classes/Sensor.cpp" -o "$binary"
"$binary"
