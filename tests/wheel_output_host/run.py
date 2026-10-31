"""Real pinned SimpleFOC motor/driver implementation; only GPIO/PWM HAL is fake."""
from pathlib import Path
import os, subprocess, tempfile
foc=Path(os.environ.get('SIMPLEFOC_SRC','.pio/libdeps/esp32dev/Arduino-FOC/src'))
with tempfile.TemporaryDirectory() as td:
    exe=Path(td)/'test'
    files=['BLDCMotor.cpp','common/base_classes/FOCMotor.cpp','common/base_classes/Sensor.cpp',
           'common/base_classes/CurrentSense.cpp','common/pid.cpp','common/lowpass_filter.cpp',
           'common/foc_utils.cpp','common/time_utils.cpp','drivers/BLDCDriver3PWM.cpp']
    flags=['-fsanitize=address,undefined','-fno-omit-frame-pointer'] if os.environ.get('SANITIZERS') else []
    subprocess.run(['c++','-std=c++17','-DSIMPLEFOC_DISABLE_DEBUG','-ffunction-sections','-fdata-sections',
      '-Itests/wheel_output_host','-Iinclude','-I'+str(foc),'tests/wheel_output_host/test.cpp',
      *[str(foc/f) for f in files],'-Wl,--gc-sections','-o',str(exe),*flags],check=True)
    subprocess.run([str(exe)],check=True)

# Execute the actual MCPWM allocator with the ESP32's two groups. HAL allocation
# itself still requires ESP32; this checks the alleged two-driver sharing case.
backend=(foc/'drivers/hardware_specific/esp32/esp32_driver_mcpwm.cpp').read_text()
def function(name):
    import re
    match=re.search(r'^(?:bool|uint8_t|int) '+name+r'\([^;]*?\)\s*\{',backend,re.M)
    assert match,name
    pos=match.end();depth=1
    while depth:
        depth+=(backend[pos]=='{')-(backend[pos]=='}');pos+=1
    return backend[match.start():pos]
code='''#include <cassert>
#include <cstdint>
#include <cstddef>
#define SOC_MCPWM_GROUPS 2
void* timers[2][3]={};uint8_t group_pins_used[2]={};
'''
code+='\n'.join(function(n) for n in ['_hasAvailablePins','_findLastTimer','_findNextTimer','_findBestGroup'])
code+='''
int main(){
 int group=-1,timer=-1;
 assert(_findBestGroup(3,25000,&group,&timer)==1&&group==0&&timer==0);
 group_pins_used[group]+=3;timers[group][timer]=&group;
 assert(_findBestGroup(3,25000,&group,&timer)==1&&group==1&&timer==0);
}
'''
with tempfile.TemporaryDirectory() as td:
    cpp=Path(td)/'allocator.cpp';cpp.write_text(code);exe=Path(td)/'allocator'
    subprocess.run(['c++','-std=c++17',str(cpp),'-o',str(exe),*flags],check=True)
    subprocess.run([str(exe)],check=True)
print('PASS: real MCPWM allocator: first 3PWM group0/timer0, second group1/timer0 (no shared timer/operator)')

# Integration ordering is part of the capture contract, even though this host
# executable does not run the complete WiFi/PS4 application loop.
app=Path('src/main.cpp').read_text();loop=app[app.index('void loop() {'):app.index('// Motor / Sensor Control')]
assert loop.index('motor2.loopFOC()')<loop.index('wheel_output_observation.foc(')<loop.index('applyWheelOutputs(right_output,left_output)')
assert '/wheels/output-capture' in app and '/api/wheels/output' in app
assert 'Serial.' not in Path('include/WheelOutputObservation.h').read_text()
