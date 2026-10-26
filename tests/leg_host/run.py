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

    subprocess.run(['c++','-include','tests/leg_host/fixture.h','-std=c++17','-Wall','-Wextra','-Werror','-Iinclude',
        'tests/leg_host/following.cpp','-o',str(binary),*flags],check=True)
    subprocess.run([str(binary)],check=True)

    subprocess.run(['c++','-include','tests/leg_host/fixture.h','-std=c++17','-Wall','-Wextra','-Werror','-Iinclude',
        'tests/leg_host/logging.cpp','-o',str(binary),*flags],check=True)
    subprocess.run([str(binary)],check=True)

    # Compare the same real controller under the previous single constant and current P.
    header=Path('include/LegControl.h').read_text()
    assert header.count('constexpr float RollGain=3;')==1
    outputs=[]
    for gain in (2,3):
        (Path(tmp)/'LegControl.h').write_text(header.replace('constexpr float RollGain=3;',f'constexpr float RollGain={gain};'))
        subprocess.run(['c++','-include','tests/leg_host/fixture.h','-std=c++17','-Wall','-Wextra','-Werror',
            '-I'+tmp,'-Iinclude',f'-DEXPECTED_GAIN={gain}','tests/leg_host/gain.cpp','-o',str(binary),*flags],check=True)
        outputs.append([float(v) for v in subprocess.check_output([str(binary)],text=True).split()])
    assert len(outputs[0])==len(outputs[1])==30
    assert all(abs(new-1.5*old)<.0001 for old,new in zip(*outputs))
    print('PASS: real P2/P3 control comparison, 1.5x unclipped correction at +/-0.25/0.5/1 deg, all 1/5/10/20/50ms periods; 8 pp/s, height and range caps')

    # Same UART path at the same 50ms cadence: reproduce the old rounding chatter.
    chatter=[]
    for hysteresis in ('0.f','.25f'):
        (Path(tmp)/'LegControl.h').write_text(header.replace('TargetHysteresisTicks=.25f',f'TargetHysteresisTicks={hysteresis}'))
        (Path(tmp)/'LegServoBus.h').write_text(Path('include/LegServoBus.h').read_text())
        subprocess.run(['c++','-include','tests/leg_host/fixture.h','-std=c++17','-Wall','-Wextra','-Werror',
            '-I'+tmp,'-Iinclude','tests/leg_host/smooth.cpp','-o',str(binary),*flags],check=True)
        chatter.append([int(v) for v in subprocess.check_output([str(binary)],text=True).split()])
    assert chatter[0][0]>=90 and chatter[0][1]>=180 and chatter[1]==[0,0]
    print('PASS: same 50ms stream, rounding-boundary chatter old/new frames/travel=',chatter,
          '; both directions/return monotonic, <=0.75 tick error, exact initial/outside behavior')

    subprocess.run(['c++','-include','tests/leg_host/fixture.h','-std=c++17','-Wall','-Wextra','-Werror','-Iinclude',
        'tests/leg_host/feel.cpp','-o',str(binary),*flags],check=True)
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
