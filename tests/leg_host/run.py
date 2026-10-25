from pathlib import Path
import subprocess,tempfile,os
source=Path('tests/leg_host/test.cpp')
with tempfile.TemporaryDirectory() as tmp:
    binary=Path(tmp)/'leg-test'
    flags=['-fsanitize=address,undefined','-fno-omit-frame-pointer'] if os.environ.get('LEG_SANITIZERS') else []
    subprocess.run(['c++','-include','tests/leg_host/fixture.h','-std=c++17','-Wall','-Wextra','-Werror','-Iinclude',str(source),'-o',str(binary),*flags],check=True)
    subprocess.run([str(binary)],check=True)

    subprocess.run(['c++','-include','tests/leg_host/fixture.h','-std=c++17','-Wall','-Wextra','-Werror','-Iinclude',
        'tests/leg_host/reply.cpp','-o',str(binary),*flags],check=True)
    subprocess.run([str(binary)],check=True)

    subprocess.run(['c++','-include','tests/leg_host/fixture.h','-std=c++17','-Wall','-Wextra','-Werror','-Iinclude',
        'tests/leg_host/position.cpp','-o',str(binary),*flags],check=True)
    subprocess.run([str(binary)],check=True)

    subprocess.run(['c++','-include','tests/leg_host/fixture.h','-std=c++17','-Wall','-Wextra','-Werror','-Iinclude',
        'tests/leg_host/acceptance.cpp','-o',str(binary),*flags],check=True)
    subprocess.run([str(binary)],check=True)

    subprocess.run(['c++','-include','tests/leg_host/fixture.h','-std=c++17','-Wall','-Wextra','-Werror','-Iinclude',
        'tests/leg_host/measurement.cpp','-o',str(binary),*flags],check=True)
    subprocess.run([str(binary)],check=True)

    subprocess.run(['c++','-include','tests/leg_host/fixture.h','-std=c++17','-Wall','-Wextra','-Werror','-Iinclude',
        'tests/leg_host/geometry.cpp','-o',str(binary),*flags],check=True)
    subprocess.run([str(binary)],check=True)

    subprocess.run(['c++','-include','tests/leg_host/fixture.h','-std=c++17','-Wall','-Wextra','-Werror','-Iinclude',
        'tests/leg_host/analog.cpp','-o',str(binary),*flags],check=True)
    subprocess.run([str(binary)],check=True)

    import sys
    sys.dont_write_bytecode=True
    import legacy
    legacy.check(tmp,flags)

    # The opt-in flag and measured sign are build settings, not inferred at runtime.
    probe=Path(tmp)/'confirmed.cpp'
    probe.write_text('#include "LegControl.h"\n#include <cassert>\nint main(){\n static_assert(leg::RollDirectionConfirmed,"confirmed build");\n static_assert(leg::ImuRollSign==EXPECTED_SIGN,"measured sign");\n leg::Control c;assert(c.seed(leg::position(true,15),leg::position(false,15),1000));\n c.activate(1000);c.input(4,true);c.tick(1050,true,false,true,-2);\n assert(c.state.requested_roll>0&&c.state.right_percent>c.state.left_percent);\n}\n')
    for sign in (1,-1):
        subprocess.run(['c++','-include','tests/leg_host/fixture.h','-std=c++17','-Wall','-Wextra','-Werror','-Iinclude',
            '-DLEG_ROLL_DIRECTION_CONFIRMED=1',f'-DLEG_IMU_ROLL_SIGN={sign}',f'-DEXPECTED_SIGN={sign}',
            str(probe),'-o',str(binary)],check=True)
        subprocess.run([str(binary)],check=True)
    invalid=subprocess.run(['c++','-include','tests/leg_host/fixture.h','-std=c++17','-Iinclude','-DLEG_IMU_ROLL_SIGN=0',
        str(probe),'-o',str(binary)],capture_output=True)
    assert invalid.returncode and b'LEG_IMU_ROLL_SIGN must be +1 or -1' in invalid.stderr
    print('PASS: explicit confirmed +/- sign builds and invalid sign rejection')

    import configparser,shlex
    ini=configparser.ConfigParser();ini.read('platformio.ini')
    production=shlex.split(ini['env:esp32dev']['build_flags'])
    subprocess.run(['c++','-include','tests/leg_host/fixture.h','-std=c++17','-Wall','-Wextra','-Werror','-Iinclude',*production,
        'tests/leg_host/measured.cpp','-o',str(binary),*flags],check=True)
    subprocess.run([str(binary)],check=True)
