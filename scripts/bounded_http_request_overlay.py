"""Compile a checked WebServer parser overlay; installed core/SDK stay untouched."""
from pathlib import Path
import hashlib,json,subprocess,os
ORIGINAL_SHA='49d5e129bf25a543b8800bfe364a8a0b683d2e91215f5b589413abc9703db1ee'
def transform(raw):
 text=raw.replace(b'\r\n',b'\n').decode('utf-8')
 digest=hashlib.sha256(text.encode()).hexdigest()
 if digest!=ORIGINAL_SHA:raise RuntimeError('Unsupported WebServer Parsing.cpp LF_SHA256='+digest)
 a=text.index('static char *readBytesWithTimeout(');b=text.index('bool WebServer::_parseRequest',a)
 text=text[:a]+text[b:]
 text=text.replace('#include "WebServer.h"','#include "WebServer.h"\n#include "BoundedHttpRequest.h"')
 text=text.replace("  String req = client.readStringUntil('\\r');\n  client.readStringUntil('\\n');",'  BoundedHttpRequest<NetworkClient> input(client);\n  String req;\n  if(!input.line(req))return false;',1)
 a=text.index('bool WebServer::_parseRequest');b=text.index('bool WebServer::_collectHeader',a)
 body=text[a:b]
 old="      req = client.readStringUntil('\\r');\n      client.readStringUntil('\\n');"
 if body.count(old)!=2:raise RuntimeError('Unexpected header loop count')
 body=body.replace(old,'      if(!input.line(req))return false;')
 body=body.replace("      if (headerDiv == -1) {\n        break;\n      }",'      if (headerDiv == -1) return input.reject(HttpRequestEnd::BadSyntax);')
 body=body.replace('    if (!isForm && _currentHandler', '    if(!input.headers(true))return false;\n    if(isForm)return input.reject(HttpRequestEnd::Unsupported);\n    if (!isForm && _currentHandler',1)
 # This project registers no upload/raw endpoints; don't leave their legacy readers reachable.
 def reject_block(text,marker):
  start=text.index(marker);brace=text.index('{',start);pos=brace+1;depth=1
  while depth:
   depth+=(text[pos]=='{')-(text[pos]=='}');pos+=1
  return text[:brace+1]+'\n      return input.reject(HttpRequestEnd::Unsupported);\n    '+text[pos-1:]
 body=reject_block(body,'if (!isForm && _currentHandler')
 body=reject_block(body,'} else {\n      // it IS a form')
 body=body.replace('readBytesWithTimeout(client, _clientContentLength, plainLength, HTTP_MAX_POST_WAIT)','input.body(size_t(_clientContentLength), plainLength)')
 body=body.replace('if (plainLength < _clientContentLength)', 'if (_clientContentLength < 0 || plainLength != size_t(_clientContentLength) || (_clientContentLength && !plainBuf))')
 body=body.replace('    _parseArguments(searchStr);\n  }\n  client.flush();','    if(!input.headers(false))return false;\n    _parseArguments(searchStr);\n  }\n  client.flush();')
 body=body.replace('  return true;\n}', '  return input.complete();\n}',1)
 if 'readStringUntil' in body or 'readBytesWithTimeout' in body:raise RuntimeError('Unbounded request line remains')
 return (text[:a]+body+text[b:]).encode('utf-8')

def install(env):
 core=Path(env.PioPlatform().get_package_dir('framework-arduinoespressif32'))
 if json.loads((core/'package.json').read_text())['version']!='3.0.3':raise RuntimeError('Unsupported Arduino core version')
 original=core/'libraries/WebServer/src/Parsing.cpp'
 output=Path(env.subst('$BUILD_DIR'))/'bounded_request/Parsing.cpp'
 output.parent.mkdir(parents=True,exist_ok=True);data=transform(original.read_bytes())
 if not output.exists() or output.read_bytes()!=data:output.write_bytes(data)
 env.Append(CPPPATH=[env.subst('$PROJECT_INCLUDE_DIR'),str(original.parent)])
 def replace(build_env,node):
  if Path(node.srcnode().get_abspath()).resolve()==original.resolve():
   return build_env.File(str(output.resolve()))
  return node
 env.AddBuildMiddleware(replace)
 def verify(source,target,env):
  nm=str(Path(env.PioPlatform().get_package_dir('toolchain-xtensa-esp32'))/'bin'/('xtensa-esp32-elf-nm.exe' if os.name=='nt' else 'xtensa-esp32-elf-nm'))
  result=subprocess.run([nm,'-C',str(target[0])],capture_output=True,text=True,check=True).stdout
  if 'BoundedHttpRequest<NetworkClient>::line<String>' not in result:raise RuntimeError('Bounded parser missing from linked ELF')
  if result.count(' T WebServer::_parseRequest(NetworkClient&)')!=1:raise RuntimeError('Unexpected linked parser count')
  print('Verified bounded request reader in linked WebServer parser (core/SDK source unchanged).')
 env.AddPostAction('$BUILD_DIR/${PROGNAME}.elf',env.VerboseAction(verify,'Verify bounded HTTP request parser in linked ELF'))
try:Import('env')
except NameError:env=None
if env is not None:install(env)
