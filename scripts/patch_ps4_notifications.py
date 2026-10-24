"""Restore disconnect and first-input notifications in PS4_Controller_Host 1.1.1.

Only notification delivery changes. Idempotent and fail closed if the
upstream function differs; no Bluetooth addresses or control settings change.
Also importable by the host regression test without loading SCons.
"""
from pathlib import Path

ORIGINAL = '''void ps4ConnectEvent(uint8_t is_connected) {
  if (is_connected) {
    ps4Enable();
  } else {
    is_active = false;
  }
}'''
PATCHED = '''void ps4ConnectEvent(uint8_t is_connected) {
  if (is_connected) {
    ps4Enable();
  } else {
    // Project patch: the public attachOnDisconnect API was never notified.
    const bool was_active = is_active;
    is_active = false;
    if (was_active) {
      if (ps4_connection_cb != NULL) ps4_connection_cb(false);
      if (ps4_connection_object_cb != NULL && ps4_connection_object != NULL) {
        ps4_connection_object_cb(ps4_connection_object, false);
      }
    }
  }
}'''


ORIGINAL_PACKET = '''void ps4PacketEvent(ps4_t ps4, ps4_event_t event) {
  // Trigger packet event, but if this is the very first packet
  // after connecting, trigger a connection event instead
  if (is_active) {
    if (ps4_event_cb != NULL) {
      ps4_event_cb(ps4, event);
    }

    if (ps4_event_object_cb != NULL && ps4_event_object != NULL) {
      ps4_event_object_cb(ps4_event_object, ps4, event);
    }
  } else {
    is_active = true;

    if (ps4_connection_cb != NULL) {
      ps4_connection_cb(is_active);
    }

    if (ps4_connection_object_cb != NULL && ps4_connection_object != NULL) {
      ps4_connection_object_cb(ps4_connection_object, is_active);
    }
  }
}'''
PATCHED_PACKET = '''void ps4PacketEvent(ps4_t ps4, ps4_event_t event) {
  // Project patch: keep connection-before-input ordering, but do not discard
  // the first report (which may contain a short Cross stop request).
  if (!is_active) {
    is_active = true;
    if (ps4_connection_cb != NULL) ps4_connection_cb(is_active);
    if (ps4_connection_object_cb != NULL && ps4_connection_object != NULL) {
      ps4_connection_object_cb(ps4_connection_object, is_active);
    }
  }
  if (ps4_event_cb != NULL) ps4_event_cb(ps4, event);
  if (ps4_event_object_cb != NULL && ps4_event_object != NULL) {
    ps4_event_object_cb(ps4_event_object, ps4, event);
  }
}'''


def patch(path):
    original = text = Path(path).read_text()
    for before, after in [(ORIGINAL, PATCHED), (ORIGINAL_PACKET, PATCHED_PACKET)]:
        if text.count(after) == 1:
            continue
        if text.count(before) != 1:
            raise RuntimeError('Unexpected PS4 1.1.1 notification implementation; refusing unverified build')
        text = text.replace(before, after, 1)
    if text == original:
        return False
    # Validate both functions before changing either on disk.
    Path(path).write_text(text)
    return True


try:
    Import('env')
except NameError:
    env = None

if env is not None:
    source = Path(env.subst('$PROJECT_LIBDEPS_DIR')) / env.subst('$PIOENV') / 'PS4_Controller_Host/src/ps4.c'
    patch(source)
