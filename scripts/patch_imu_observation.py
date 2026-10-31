"""Observe existing MPU6050_tockn 1.5.2 update IO; never change filter/read flow."""
from pathlib import Path
import hashlib

COMMIT = '217f66ba47416ccfc6e4f5048035ac2ee3096fe6'
ORIGINAL = '81787433e5998fd13e5218220bd01e3a0f5208e7f3be2b9e324c680511bfbb78'
CHANGES = [
    ('#include "Arduino.h"', '#include "Arduino.h"\nextern "C" void observeImuTransaction(unsigned char tx, unsigned char rx);\nextern "C" void observeImuCalibration(unsigned char tx, unsigned char rx);'),
    ('void MPU6050::update(){\n\twire->beginTransmission(MPU6050_ADDR);\n\twire->write(0x3B);\n\twire->endTransmission(false);\n\twire->requestFrom((int)MPU6050_ADDR, 14);',
     'void MPU6050::update(){\n\twire->beginTransmission(MPU6050_ADDR);\n\twire->write(0x3B);\n\tconst unsigned char observed_tx = wire->endTransmission(false);\n\tconst unsigned char observed_rx = wire->requestFrom((int)MPU6050_ADDR, 14);\n\tobserveImuTransaction(observed_tx, observed_rx);'),
    ('    wire->endTransmission(false);\n    wire->requestFrom((int)MPU6050_ADDR, 6);',
     '    const unsigned char observed_tx = wire->endTransmission(false);\n    const unsigned char observed_rx = wire->requestFrom((int)MPU6050_ADDR, 6);\n    observeImuCalibration(observed_tx, observed_rx);'),
]

def sha(data):
    return hashlib.sha256(data).hexdigest()

def transform(text):
    for before, after in CHANGES:
        if text.count(before) != 1:
            raise RuntimeError('MPU observation anchor mismatch; refusing patch')
        text = text.replace(before, after, 1)
    return text

# Filled from the verified original; the result is also checked, not just markers.
PATCHED = 'f32752bf3a093ada52d658c9e6d520d7c494bea280fe85b627a72048e642d03b'

def patch(path):
    path = Path(path)
    raw = path.read_bytes()
    normalized = raw.replace(b'\r\n', b'\n')
    version = (path.parent.parent / 'library.properties').read_text(encoding='utf-8')
    digest = sha(normalized)
    if 'version=1.5.2\n' not in version.replace('\r\n', '\n') or digest not in (ORIGINAL, PATCHED):
        raise RuntimeError(f'MPU observation target mismatch version=1.5.2-required sha256={sha(raw)} '
                           f'normalized_sha256={digest}; refusing patch')
    if digest == PATCHED:
        return False
    out = transform(normalized.decode('utf-8')).encode('utf-8')
    if sha(out) != PATCHED:
        raise RuntimeError('MPU observation output SHA mismatch; refusing patch')
    path.write_bytes(out.replace(b'\n', b'\r\n') if b'\r\n' in raw else out)
    return True

try:
    Import('env')
except NameError:
    env = None
if env is not None:
    patch(Path(env.subst('$PROJECT_LIBDEPS_DIR')) / env.subst('$PIOENV') / 'MPU6050_tockn/src/MPU6050_tockn.cpp')

# A source edit alone is insufficient: verify the actual linked update calls us.
def verify_link(disassembly):
    import re
    match = re.search(r'<MPU6050::update\(\)>:\n(.*?)(?=\n[0-9a-f]+ <|\Z)', disassembly, re.S)
    if not match or not re.search(r'call\w*\s+.*<observeImuTransaction>', match.group(1)):
        raise RuntimeError('MPU observation callback not called by linked update; refusing unobserved firmware')
    calibration = re.search(r'<MPU6050::calcGyroOffsets\(.*?\)>:\n(.*?)(?=\n[0-9a-f]+ <|\Z)', disassembly, re.S)
    if not calibration or not re.search(r'call\w*\s+.*<observeImuCalibration>', calibration.group(1)):
        raise RuntimeError('MPU calibration callback not linked; refusing unchecked auto startup')

if env is not None:
    def verify_elf(source, target, env):
        import subprocess,os
        suffix = '.exe' if os.name == 'nt' else ''
        objdump = Path(env.PioPlatform().get_package_dir('toolchain-xtensa-esp32')) / 'bin' / ('xtensa-esp32-elf-objdump' + suffix)
        text = subprocess.check_output([str(objdump), '-d', '-C', str(target[0])], text=True)
        verify_link(text)
        print('MPU observation: linked existing-IO callback verified')
    env.AddPostAction('$BUILD_DIR/${PROGNAME}.elf', env.VerboseAction(verify_elf, 'Verify linked MPU transaction observation'))
