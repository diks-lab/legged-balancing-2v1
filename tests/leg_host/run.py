from pathlib import Path
import subprocess,tempfile,os
source=Path('tests/leg_host/test.cpp')
with tempfile.TemporaryDirectory() as tmp:
    binary=Path(tmp)/'leg-test'
    flags=['-fsanitize=address,undefined','-fno-omit-frame-pointer'] if os.environ.get('LEG_SANITIZERS') else []
    subprocess.run(['c++','-std=c++17','-Wall','-Wextra','-Werror','-Iinclude',str(source),'-o',str(binary),*flags],check=True)
    subprocess.run([str(binary)],check=True)

    subprocess.run(['c++','-std=c++17','-Wall','-Wextra','-Werror','-Iinclude',
        'tests/leg_host/reply.cpp','-o',str(binary),*flags],check=True)
    subprocess.run([str(binary)],check=True)

    subprocess.run(['c++','-std=c++17','-Wall','-Wextra','-Werror','-Iinclude',
        'tests/leg_host/position.cpp','-o',str(binary),*flags],check=True)
    subprocess.run([str(binary)],check=True)

    subprocess.run(['c++','-std=c++17','-Wall','-Wextra','-Werror','-Iinclude',
        'tests/leg_host/acceptance.cpp','-o',str(binary),*flags],check=True)
    subprocess.run([str(binary)],check=True)

    # The opt-in flag and measured sign are build settings, not inferred at runtime.
    probe=Path(tmp)/'confirmed.cpp'
    probe.write_text('#include "LegControl.h"\n#include <cassert>\nint main(){\n static_assert(leg::RollDirectionConfirmed,"confirmed build");\n static_assert(leg::ImuRollSign==EXPECTED_SIGN,"measured sign");\n leg::Control c;assert(c.seed(leg::position(true,15),leg::position(false,15),1000));\n c.activate(1000);c.input(4,true);c.tick(1050,true,false,true,-2);\n assert(c.state.requested_roll>0&&c.state.right_percent>c.state.left_percent);\n}\n')
    for sign in (1,-1):
        subprocess.run(['c++','-std=c++17','-Wall','-Wextra','-Werror','-Iinclude',
            '-DLEG_ROLL_DIRECTION_CONFIRMED=1',f'-DLEG_IMU_ROLL_SIGN={sign}',f'-DEXPECTED_SIGN={sign}',
            str(probe),'-o',str(binary)],check=True)
        subprocess.run([str(binary)],check=True)
    invalid=subprocess.run(['c++','-std=c++17','-Iinclude','-DLEG_IMU_ROLL_SIGN=0',
        str(probe),'-o',str(binary)],capture_output=True)
    assert invalid.returncode and b'LEG_IMU_ROLL_SIGN must be +1 or -1' in invalid.stderr
    print('PASS: explicit confirmed +/- sign builds and invalid sign rejection')

    acceptance=Path(tmp)/'acceptance-flags.cpp'
    acceptance.write_text('#include "LegControl.h"\n#include <cassert>\nint main(){leg::Control c;assert(c.seed(1977,2082,100));assert(c.state.recovery==1);assert(c.target(true)==1977&&c.target(false)==2082);}\n')
    configured=['-DLEG_ACCEPT_EXTRA_CONFIRMED=1','-DLEG_ACCEPT_RIGHT_MIN=1950',
        '-DLEG_ACCEPT_RIGHT_MAX=2100','-DLEG_ACCEPT_LEFT_MIN=2000','-DLEG_ACCEPT_LEFT_MAX=2110']
    subprocess.run(['c++','-std=c++17','-Iinclude',*configured,str(acceptance),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
    for flags in (['-DLEG_ACCEPT_EXTRA_CONFIRMED=1'],configured[:-1],
                  [*configured,'-ULEG_ACCEPT_RIGHT_MIN','-DLEG_ACCEPT_RIGHT_MIN=2020']):
        result=subprocess.run(['c++','-std=c++17','-Iinclude',*flags,str(acceptance),'-o',str(binary)],capture_output=True)
        assert result.returncode
    print('PASS: default extra acceptance disabled, explicit four-limit build, incomplete/invalid configurations fail build')
