"""Observation-only patch for SimpleFOC v2.4.0 / 4f072b3. Fail closed on drift."""
from pathlib import Path
import hashlib

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

def patch(path):
    path=Path(path); text=path.read_text()
    digest=hashlib.sha256(text.encode()).hexdigest()
    if digest==PATCHED_SHA256: return False
    if digest!=ORIGINAL_SHA256:
        raise RuntimeError('Unexpected SimpleFOC alignment source; refusing unverified observation patch')
    for before,after in CHANGES:
        if text.count(before)!=1: raise RuntimeError('Ambiguous SimpleFOC observation anchor')
        text=text.replace(before,after,1)
    if hashlib.sha256(text.encode()).hexdigest()!=PATCHED_SHA256:
        raise RuntimeError('Unexpected SimpleFOC observation output')
    path.write_text(text);return True

try:
    Import('env')
except NameError:
    env=None
if env is not None:
    env.Append(CPPPATH=[env.subst('$PROJECT_INCLUDE_DIR')])
    patch(Path(env.subst('$PROJECT_LIBDEPS_DIR'))/env.subst('$PIOENV')/'Arduino-FOC/src/common/base_classes/FOCMotor.cpp')
