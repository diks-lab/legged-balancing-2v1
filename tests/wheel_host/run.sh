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
extra_flags=()
if [[ "${WHEEL_SANITIZERS:-0}" == 1 ]]; then
  extra_flags+=(-fsanitize=address,undefined -fno-omit-frame-pointer)
fi
if [[ "${WHEEL_MCPWM_METADATA:-0}" == 1 ]]; then
  extra_flags+=(-DESP_H -DARDUINO_ARCH_ESP32 -DSOC_MCPWM_SUPPORTED)
fi
c++ "${extra_flags[@]}" -std=c++17 -Wall -Wextra -Werror -Iinclude -Itests/wheel_host -I"$foc_src" tests/wheel_host/test.cpp "$foc_src/common/base_classes/Sensor.cpp" -o "$binary"
for scenario in invalid wrap missing bus_init timer_create driver_fail motor_fail foc_fail pp_fail init_stop init_motor init_partial init_timeout init_sensor right left_negative right_positive nack short data during_control other_sensor timer_start loop_timeout blocked_tx clock_wrap stop both_stop both_negative both_restart both_reuse_left both_reuse_right both_left_sensor both_right_sensor both_timer both_loop_timeout both_missing both_timer_create both_timer_start both_first_driver both_first_motor both_first_sensor both_first_foc both_first_pp both_first_timeout both_first_stop both_first_partial both_first_queued both_second_driver both_second_motor both_second_foc both_second_pp both_second_sensor both_second_timeout both_second_stop both_second_partial both_second_queued; do
  "$binary" "$scenario"
done


c++ "${extra_flags[@]}" -std=c++17 -Wall -Wextra -Werror -Iinclude -Itests/wheel_host -I"$foc_src" tests/wheel_host/fixed.cpp "$foc_src/common/base_classes/Sensor.cpp" -o "$binary"
for scenario in queued partial_end left_first left_first_both invalid_idle enable_timeout left_plus left_minus right_plus right_minus missing config direction zero poles pp crc short read_error open_error newer_config timer_create sensor_start driver motor first_foc second_foc changed_foc init_sensor init_timeout init_input init_partial timer_start stop left_sensor right_sensor nonfinite independent reject invalid clock_wrap fixed_speed speed_fixed_speed; do
  "$binary" "$scenario"
done
