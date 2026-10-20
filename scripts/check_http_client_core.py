"""Read-only compatibility check for the TX-only NetworkClient adapter.

No core/SDK source is patched. LF/CRLF normalization only; unknown layouts or
lifetime implementations stop the build before compiling the adapter.
"""
from pathlib import Path
import hashlib,json
EXPECTED={
 'NetworkClient.h':'80cab67302b28f7317946c7f98f5a10f70be6e33ae8129457b52589f8fbf3ebf',
 'NetworkClient.cpp':'88cddcbcc3b44860edc98985389abd15b6e1ef50ecc815e32b05a1d4a1c043b6',
}
def check(root):
 root=Path(root)
 version=json.loads((root/'package.json').read_text(encoding='utf-8'))['version']
 if version!='3.0.3':raise RuntimeError('Unsupported Arduino core version for HTTP TX-only adapter: '+version)
 for name,expected in EXPECTED.items():
  raw=(root/'libraries/Network/src'/name).read_bytes()
  digest=hashlib.sha256(raw.replace(b'\r\n',b'\n')).hexdigest()
  if digest!=expected:raise RuntimeError('Unsupported HTTP client ownership source: '+name+' LF_SHA256='+digest)
try:Import('env')
except NameError:env=None
if env is not None:
 check(env.PioPlatform().get_package_dir('framework-arduinoespressif32'))
elif __name__=='__main__':
 import argparse
 parser=argparse.ArgumentParser();parser.add_argument('core',type=Path);args=parser.parse_args();check(args.core)
