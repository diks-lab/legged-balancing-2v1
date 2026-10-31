"""Core compatibility guard is read-only, LF/CRLF portable, and fails closed."""
from pathlib import Path
import tempfile,importlib.util,json,os
spec=importlib.util.spec_from_file_location('guard','scripts/check_http_client_core.py')
guard=importlib.util.module_from_spec(spec);spec.loader.exec_module(guard)
core=Path(os.environ.get('PLATFORMIO_CORE_DIR','/workspace/pio-core'))/'packages/framework-arduinoespressif32'
guard.check(core)
with tempfile.TemporaryDirectory() as td:
 root=Path(td);target=root/'libraries/Network/src';target.mkdir(parents=True)
 package=root/'package.json';package.write_text(json.dumps({'version':'3.0.3'}))
 for crlf in [False,True]:
  for name in guard.EXPECTED:
   data=(core/'libraries/Network/src'/name).read_bytes().replace(b'\r\n',b'\n')
   (target/name).write_bytes(data.replace(b'\n',b'\r\n') if crlf else data)
  before={p:p.read_bytes() for p in target.iterdir()};guard.check(root);guard.check(root)
  assert all(p.read_bytes()==data for p,data in before.items())
  for name in guard.EXPECTED:
   p=target/name;original=p.read_bytes();p.write_bytes(original+b' unexpected')
   try:guard.check(root);raise AssertionError('accepted unknown ownership source')
   except RuntimeError as e:assert name in str(e) and 'LF_SHA256=' in str(e)
   assert p.read_bytes()==original+b' unexpected';p.write_bytes(original)
 package.write_text(json.dumps({'version':'3.0.4'}))
 try:guard.check(root);raise AssertionError('accepted unknown core version')
 except RuntimeError as e:assert '3.0.4' in str(e)
print('PASS: actual core version/full-source guard, LF/CRLF, idempotent read-only check, unknown version/header/implementation refusal')
