from pathlib import Path
import subprocess,tempfile,os
source=Path('tests/leg_host/test.cpp')
with tempfile.TemporaryDirectory() as tmp:
    binary=Path(tmp)/'leg-test'
    flags=['-fsanitize=address,undefined','-fno-omit-frame-pointer'] if os.environ.get('LEG_SANITIZERS') else []
    subprocess.run(['c++','-std=c++17','-Wall','-Wextra','-Werror','-Iinclude',str(source),'-o',str(binary),*flags],check=True)
    subprocess.run([str(binary)],check=True)
