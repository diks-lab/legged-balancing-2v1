#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
foc_src="${SIMPLEFOC_SRC:-.pio/libdeps/wheel_diagnostic/Arduino-FOC/src}"
if [ ! -f "$foc_src/common/base_classes/Sensor.h" ]; then
  echo 'Build wheel_diagnostic first, or set SIMPLEFOC_SRC to SimpleFOC 2.4.0 src.' >&2
  exit 1
fi
binary=$(mktemp /tmp/wheel-diagnostic-test.XXXXXX)
trap 'rm -f "$binary"' EXIT
c++ -std=c++17 -Wall -Wextra -Werror -Itests/wheel_host -I"$foc_src" tests/wheel_host/test.cpp -o "$binary"
for scenario in invalid wrap missing bus_init timer_create driver_fail motor_fail foc_fail pp_fail init_stop init_motor init_partial init_timeout init_sensor right nack short magnet weak strong data during_control other_sensor timer_start loop_timeout blocked_tx clock_wrap stop; do
  "$binary" "$scenario"
done
