from pathlib import Path
import subprocess,tempfile,os
with tempfile.TemporaryDirectory() as d:
    binary=Path(d)/'leg-tx'
    flags=['-fsanitize=address,undefined','-fno-omit-frame-pointer'] if os.environ.get('LEG_SANITIZERS') else []
    subprocess.run(['c++','-Os','-fno-rtti','-ffunction-sections','-fdata-sections','-Wl,--gc-sections','-std=c++17','-Wall','-Wextra','-Werror','-DARDUINO=100',
        '-Itests/leg_tx_host','-Isrc','-Iinclude','tests/leg_tx_host/test.cpp','src/Servo_STS3032.cpp',
        '-o',str(binary),*flags],check=True)
    subprocess.run([str(binary)],check=True)
