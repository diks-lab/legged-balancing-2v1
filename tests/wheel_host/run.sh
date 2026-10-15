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
c++ -std=c++17 -Wall -Wextra -Werror -Itests/wheel_host -I"$foc_src" tests/wheel_host/test.cpp "$foc_src/common/base_classes/Sensor.cpp" -o "$binary"
for scenario in invalid wrap missing bus_init timer_create driver_fail motor_fail foc_fail pp_fail init_stop init_motor init_partial init_timeout init_sensor right nack short data during_control other_sensor timer_start loop_timeout blocked_tx clock_wrap stop both_stop both_negative both_restart both_left_sensor both_right_sensor both_timer both_loop_timeout both_missing both_timer_create both_timer_start both_first_driver both_first_foc both_first_pp both_first_timeout both_first_stop both_first_partial both_first_queued both_second_driver both_second_motor both_second_foc both_second_pp both_second_sensor both_second_timeout both_second_stop both_second_partial both_second_queued; do
  "$binary" "$scenario"
done
