from pathlib import Path
import subprocess,tempfile,os
with tempfile.TemporaryDirectory() as d:
    binary=Path(d)/'calibration'
    flags=['-fsanitize=address,undefined','-fno-omit-frame-pointer'] if os.environ.get('LEG_SANITIZERS') else []
    subprocess.run(['c++','-std=c++17','-Wall','-Wextra','-Werror','-Iinclude','tests/leg_calibration_host/test.cpp','-o',str(binary),*flags],check=True)
    subprocess.run([str(binary)],check=True)

# Route tokens must never collide with wheel calibration/mode/start commands.
source=Path('src/main.cpp').read_text()
for index,name in enumerate(['start','candidate','right-zero','right-full','left-zero','left-full','save','abort'],1):
    route=source.split(f'"/legs/calibration/{name}"',1)[1].split('});',1)[0]
    assert f"enqueueHttpCommand('{index}')" in route
