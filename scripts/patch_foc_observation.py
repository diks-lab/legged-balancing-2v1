"""Observation-only patch for SimpleFOC v2.4.0 / 4f072b3. Fail closed on drift."""
from pathlib import Path
import hashlib
import re
import sys

ORIGINAL_SHA256 = 'c119b0e44313f4e36a3886ba40762b7e4b5244e0f82f9d7497efee3a4b765d1f'
PATCHED_SHA256 = '6b970b2ece6416f36f22bf1fd1c91bcbb98c663e6914159485e0b095117412a4'
CHANGES = [
    ('#include "FOCMotor.h"',
     '''#include "FOCMotor.h"
#include "FocAlignmentObservation.h"'''),
    ('''    disable();
  }

  return exit_flag;''',
     '''    disable();
    observeFocLibraryDisable(_micros());
  }

  return exit_flag;'''),
    ('''    // find natural direction
''',
     '''    observeFocAlignPoint(this, FocAlignPoint::ForwardBegin, sensor->getAngle(), _micros());
    // find natural direction
'''),
    ('    float mid_angle = sensor->getAngle();',
     '''    float mid_angle = sensor->getAngle();
    observeFocAlignPoint(this, FocAlignPoint::ForwardEnd, mid_angle, _micros());'''),
    ('    float end_angle = sensor->getAngle();',
     '''    float end_angle = sensor->getAngle();
    observeFocAlignPoint(this, FocAlignPoint::BackwardEnd, end_angle, _micros());'''),
    ('    if( pp_check_result==false ) {',
     '''    observeFocAlignPp(this, moved, pp_check_result);
    if( pp_check_result==false ) {'''),
    ('  if(!_isset(zero_electric_angle)){',
     '''  if(!_isset(zero_electric_angle)){
    observeFocAlignPoint(this, FocAlignPoint::ZeroBegin, 0.0f, _micros());'''),
    ('''    _delay(200);
  } else { SIMPLEFOC_MOTOR_DEBUG("Skip offset calib."); }''',
     '''    _delay(200);
    observeFocAlignPoint(this, FocAlignPoint::ZeroEnd, zero_electric_angle, _micros());
  } else { SIMPLEFOC_MOTOR_DEBUG("Skip offset calib."); }'''),
]

EXPECTED_VERSION = '2.4.0'
EXPECTED_COMMIT = '4f072b365f6e0185adca544071e595834405babc'
DEFAULT_TARGET = '.pio/libdeps/esp32dev/Arduino-FOC/src/common/base_classes/FOCMotor.cpp'


def _sha(raw):
    return hashlib.sha256(raw).hexdigest()


def _library_identity(path):
    # Read only version and detached checkout ID. Never print paths, source,
    # author fields, remotes, environment, or arbitrary metadata values.
    if [path.parent.name, path.parent.parent.name, path.parent.parent.parent.name] != ['base_classes', 'common', 'src']:
        return 'unknown', 'unknown'
    root = path.parent.parent.parent.parent
    version = 'unknown'
    commit = 'unknown'
    try:
        for line in (root / 'library.properties').read_bytes().decode('utf-8').splitlines():
            if line.startswith('version='):
                candidate = line.partition('=')[2].strip()
                if re.fullmatch(r'[0-9]+(?:\.[0-9]+){1,3}(?:[-+][0-9A-Za-z.-]+)?', candidate) and len(candidate) <= 64:
                    version = candidate
                break
    except (OSError, UnicodeError):
        pass
    try:
        candidate = (root / '.git/HEAD').read_bytes().decode('ascii').strip()
        if re.fullmatch(r'[0-9a-fA-F]{40}', candidate):
            commit = candidate.lower()
    except (OSError, UnicodeError):
        pass
    return version, commit


def inspect_source(path):
    """Read only. Canonical identity changes ONLY CRLF into LF, never code/text."""
    path = Path(path)
    raw = path.read_bytes()
    crlf = raw.count(b'\r\n')
    lf = raw.count(b'\n') - crlf
    bare_cr = raw.count(b'\r') - crlf
    endings = 'CR' if bare_cr else ('MIXED' if crlf and lf else ('CRLF' if crlf else ('LF' if lf else 'NONE')))
    normalized = raw.replace(b'\r\n', b'\n')
    text = None
    encoding = 'UTF-8'
    try:
        text = normalized.decode('utf-8', errors='strict')
        if raw.startswith(b'\xef\xbb\xbf'):
            encoding = 'UTF-8-BOM'
    except UnicodeError:
        encoding = 'INVALID-UTF-8'
    version, commit = _library_identity(path)
    digest = _sha(normalized)
    original_match = digest == ORIGINAL_SHA256
    patched_match = digest == PATCHED_SHA256
    reason = None
    if encoding != 'UTF-8':
        reason = 'encoding: expected UTF-8 without BOM'
    elif endings not in ('LF', 'CRLF'):
        reason = 'line_endings: expected uniform LF or CRLF'
    elif not (original_match or patched_match):
        reason = 'source_hash: neither original nor already-patched SHA-256 matches'
    return dict(raw=raw, text=text, raw_sha256=_sha(raw), normalized_sha256=digest,
                line_endings=endings, crlf=crlf, lf=lf, bare_cr=bare_cr,
                encoding=encoding, version=version, commit=commit,
                original_match=original_match, patched_match=patched_match,
                state='REJECTED' if reason else ('PATCHED' if patched_match else 'ORIGINAL'),
                reason=reason)


def print_diagnostics(info, reason=None):
    print('SimpleFOC observation patch diagnostics (read-only metadata):')
    print('  declared_version=' + info['version'] + ' expected_version=' + EXPECTED_VERSION)
    print('  checkout_commit=' + info['commit'] + ' expected_commit=' + EXPECTED_COMMIT)
    print('  raw_sha256=' + info['raw_sha256'])
    print('  lf_normalized_sha256=' + info['normalized_sha256'])
    print('  line_endings=' + info['line_endings'] +
          ' CRLF=' + str(info['crlf']) + ' LF_only=' + str(info['lf']) + ' bare_CR=' + str(info['bare_cr']))
    print('  encoding=' + info['encoding'] + ' state=' + info['state'])
    print('  original_sha256_match=' + str(info['original_match']) +
          ' patched_sha256_match=' + str(info['patched_match']))
    print('  expected_original_sha256=' + ORIGINAL_SHA256)
    print('  expected_patched_sha256=' + PATCHED_SHA256)
    if reason or info['reason']:
        print('  rejected_condition=' + (reason or info['reason']))
    if info['text'] is not None:
        for index, (before, after) in enumerate(CHANGES, 1):
            print('  anchor[' + str(index) + '] original_count=' + str(info['text'].count(before)) +
                  ' patched_count=' + str(info['text'].count(after)))
    # Anchors and metadata are diagnostics only. Never accept an unknown full hash.


def _reject(info, reason):
    print_diagnostics(info, reason)
    raise RuntimeError('Unexpected SimpleFOC alignment source; refusing unverified observation patch: ' + reason)


def patch(path):
    path = Path(path)
    info = inspect_source(path)
    if info['reason']:
        _reject(info, info['reason'])
    if info['patched_match']:
        return False  # No rewrite or newline conversion on repeated application.
    text = info['text']
    for index, (before, after) in enumerate(CHANGES, 1):
        if text.count(before) != 1:
            _reject(info, 'anchor[' + str(index) + ']: expected exactly one original occurrence')
        text = text.replace(before, after, 1)
    normalized_output = text.encode('utf-8')
    if _sha(normalized_output) != PATCHED_SHA256:
        _reject(info, 'output_hash: generated patch does not match expected SHA-256')
    output = normalized_output.replace(b'\n', b'\r\n') if info['line_endings'] == 'CRLF' else normalized_output
    # Binary writes avoid Windows newline translation and locale re-encoding.
    # All checks precede this sole mutation. Preserve the original newline style.
    path.write_bytes(output)
    return True


def check_cli():
    import argparse
    parser = argparse.ArgumentParser(description='Read-only SimpleFOC observation patch diagnosis')
    parser.add_argument('--check', action='store_true', required=True, help='inspect only; never patch/write')
    parser.add_argument('path', nargs='?', default=DEFAULT_TARGET)
    args = parser.parse_args()
    try:
        info = inspect_source(args.path)
    except OSError:
        print('SimpleFOC check failed: target file is not readable (no file modified).', file=sys.stderr)
        return 2
    print_diagnostics(info)
    return 1 if info['reason'] else 0


try:
    Import('env')
except NameError:
    env = None
if env is not None:
    env.Append(CPPPATH=[env.subst('$PROJECT_INCLUDE_DIR')])
    patch(Path(env.subst('$PROJECT_LIBDEPS_DIR')) / env.subst('$PIOENV') / 'Arduino-FOC/src/common/base_classes/FOCMotor.cpp')
elif __name__ == '__main__':
    sys.exit(check_cli())
