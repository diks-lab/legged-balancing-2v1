"""Fail-closed PS4_Controller_Host 1.1.1 L2CAP allocation/session repair."""
from pathlib import Path
import hashlib

ORIGINAL_SHA256 = 'e87932d6613987b8afb1e42983306c3d6a6e94f0db3730c098fddd83f3a1c016'
PATCHED_SHA256 = '51b2eada6cbcd4f0b2722411a8ed742508aa7d3fb911110c97739d7b0fd58683'
CHANGES = [('uint16_t l2cap_interrupt_channel = 0;',
  'uint16_t l2cap_interrupt_channel = 0;\n'
  '\n'
  '// Only the BT callback context owns these transport/session fields.\n'
  'static bool control_configured = false;\n'
  'static bool interrupt_configured = false;\n'
  'static bool session_aborted = false;\n'
  '\n'
  'static void ps4_l2cap_abort(void) {\n'
  '  const uint16_t control = l2cap_control_channel;\n'
  '  const uint16_t interrupt = l2cap_interrupt_channel;\n'
  '  // Invalidate before stack calls/callbacks. Late data/config cannot revive it.\n'
  '  session_aborted = true;\n'
  '  is_connected = false;\n'
  '  control_configured = false;\n'
  '  interrupt_configured = false;\n'
  '  l2cap_control_channel = 0;\n'
  '  l2cap_interrupt_channel = 0;\n'
  '  ps4ConnectEvent(false); // Existing notification route, no motor/stop call.\n'
  '  if (control) L2CA_DisconnectReq(control);\n'
  '  if (interrupt && interrupt != control) L2CA_DisconnectReq(interrupt);\n'
  '}'),
 ('void ps4_l2cap_send_hid(hid_cmd_t *hid_cmd, uint8_t len) {\n'
  '  uint8_t result;\n'
  '  BT_HDR *p_buf;\n'
  '\n'
  '  p_buf = (BT_HDR *)osi_malloc(BT_DEFAULT_BUFFER_SIZE);\n'
  '\n'
  '  if (!p_buf) {\n'
  '    ESP_LOGE(PS4_TAG, "[%s] allocating buffer for sending the command failed", __func__);\n'
  '  }\n'
  '\n'
  '  p_buf->length = len + (sizeof(*hid_cmd) - sizeof(hid_cmd->data));\n'
  '  p_buf->offset = L2CAP_MIN_OFFSET;\n'
  '\n'
  '  memcpy((uint8_t *)(p_buf + 1) + p_buf->offset, (uint8_t *)hid_cmd, p_buf->length);\n'
  '\n'
  '  if (l2cap_control_channel == 0) {\n'
  '    ESP_LOGE(PS4_TAG, "[%s] l2cap_control_channel not initialized.", __func__);\n'
  '  }\n'
  '  result = L2CA_DataWrite(l2cap_control_channel, p_buf);\n'
  '\n'
  '  if (result == L2CAP_DW_SUCCESS)\n'
  '    ESP_LOGI(PS4_TAG, "[%s] sending command: success", __func__);\n'
  '\n'
  '  if (result == L2CAP_DW_CONGESTED)\n'
  '    ESP_LOGW(PS4_TAG, "[%s] sending command: congested", __func__);\n'
  '\n'
  '  if (result == L2CAP_DW_FAILED)\n'
  '    ESP_LOGE(PS4_TAG, "[%s] sending command: failed", __func__);\n'
  '}\n',
  'void ps4_l2cap_send_hid(hid_cmd_t *hid_cmd, uint8_t len) {\n'
  "  // Also suppress ps4Enable's second (LED) send after the first send aborts.\n"
  '  if (session_aborted || !is_connected || !l2cap_control_channel) return;\n'
  '  if (!hid_cmd || len > sizeof(hid_cmd->data)) {\n'
  '    ps4_l2cap_abort();\n'
  '    return;\n'
  '  }\n'
  '  BT_HDR *p_buf = (BT_HDR *)osi_malloc(BT_DEFAULT_BUFFER_SIZE);\n'
  '  if (!p_buf) {\n'
  '    // Failure-only, one line per aborted session. No addresses/pairing data.\n'
  '    ESP_LOGE(PS4_TAG, "HID alloc failed requested=%u caps=DEFAULT(0x%x) free=%u largest=%u '
  'internal8_free=%u internal8_largest=%u; abort",\n'
  '             (unsigned)BT_DEFAULT_BUFFER_SIZE, (unsigned)MALLOC_CAP_DEFAULT,\n'
  '             (unsigned)heap_caps_get_free_size(MALLOC_CAP_DEFAULT),\n'
  '             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT),\n'
  '             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),\n'
  '             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));\n'
  '    ps4_l2cap_abort();\n'
  '    return;\n'
  '  }\n'
  '  p_buf->length = len + (sizeof(*hid_cmd) - sizeof(hid_cmd->data));\n'
  '  p_buf->offset = L2CAP_MIN_OFFSET;\n'
  '  memcpy((uint8_t *)(p_buf + 1) + p_buf->offset, (uint8_t *)hid_cmd, p_buf->length);\n'
  '  // L2CA_DataWrite consumes the buffer on success, congestion AND failure.\n'
  '  const uint8_t result = L2CA_DataWrite(l2cap_control_channel, p_buf);\n'
  '  if (result != L2CAP_DW_SUCCESS && result != L2CAP_DW_CONGESTED) {\n'
  '    ESP_LOGE(PS4_TAG, "HID DataWrite failed; abort");\n'
  '    ps4_l2cap_abort();\n'
  '  }\n'
  '}\n'),
 ('  /* Send a Configuration Request. */\n'
  '  L2CA_CONFIG_REQ(l2cap_cid, &ps4_cfg_info);\n'
  '\n'
  '  if (psm == BT_PSM_HID_CONTROL) {\n'
  '    l2cap_control_channel = l2cap_cid;\n'
  '  } else if (psm == BT_PSM_HID_INTERRUPT) {\n'
  '    l2cap_interrupt_channel = l2cap_cid;\n'
  '  }',
  '  if (psm == BT_PSM_HID_CONTROL) {\n'
  '    if (l2cap_control_channel) { L2CA_DisconnectReq(l2cap_cid); return; }\n'
  '    // A new control channel is the only start of a new session.\n'
  '    if (l2cap_interrupt_channel) ps4_l2cap_abort();\n'
  '    session_aborted = false;\n'
  '    control_configured = interrupt_configured = false;\n'
  '    l2cap_control_channel = l2cap_cid;\n'
  '  } else if (psm == BT_PSM_HID_INTERRUPT && l2cap_control_channel && !session_aborted) {\n'
  '    if (l2cap_interrupt_channel) { L2CA_DisconnectReq(l2cap_cid); return; }\n'
  '    l2cap_interrupt_channel = l2cap_cid;\n'
  '  } else {\n'
  '    L2CA_DisconnectReq(l2cap_cid);\n'
  '    return;\n'
  '  }\n'
  '  /* Send a Configuration Request only after recording the channel. */\n'
  '  if (!L2CA_CONFIG_REQ(l2cap_cid, &ps4_cfg_info)) ps4_l2cap_abort();'),
 ('  /* The PS4 controller is connected after    */\n'
  '  /* receiving the second config confirmation */\n'
  '  bool prev_is_connected = is_connected;\n'
  '  is_connected = l2cap_cid == l2cap_interrupt_channel;\n'
  '  if (prev_is_connected != is_connected) {\n'
  '    ps4ConnectEvent(is_connected);\n'
  '  }',
  '  if (session_aborted || !l2cap_cid ||\n'
  '      (l2cap_cid != l2cap_control_channel && l2cap_cid != l2cap_interrupt_channel)) return;\n'
  '  if (!p_cfg || p_cfg->result != L2CAP_CFG_OK) {\n'
  '    ESP_LOGE(PS4_TAG, "HID configuration failed; abort");\n'
  '    ps4_l2cap_abort();\n'
  '    return;\n'
  '  }\n'
  '  if (l2cap_cid == l2cap_control_channel) control_configured = true;\n'
  '  if (l2cap_cid == l2cap_interrupt_channel) interrupt_configured = true;\n'
  '  if (!is_connected && control_configured && interrupt_configured) {\n'
  '    is_connected = true;\n'
  '    ps4ConnectEvent(true); // Enable can synchronously abort on allocation failure.\n'
  '  }'),
 ('  is_connected = false;\n'
  '  if (ack_needed) {\n'
  '    L2CA_DisconnectRsp(l2cap_cid);\n'
  '  }\n'
  '  ps4ConnectEvent(is_connected);',
  '  if (ack_needed) L2CA_DisconnectRsp(l2cap_cid);\n'
  '  if (l2cap_cid && (l2cap_cid == l2cap_control_channel || l2cap_cid == l2cap_interrupt_channel)) {\n'
  '    // The peer has already removed this channel; close only its companion.\n'
  '    if (l2cap_cid == l2cap_control_channel) l2cap_control_channel = 0;\n'
  '    if (l2cap_cid == l2cap_interrupt_channel) l2cap_interrupt_channel = 0;\n'
  '    ps4_l2cap_abort();\n'
  '  }'),
 ('static void ps4_l2cap_disconnect_cfm_cback(uint16_t l2cap_cid, uint16_t result) {\n'
  '  ESP_LOGI(PS4_TAG, "[%s] l2cap_cid: 0x%02x\\n  result: %d", __func__, l2cap_cid, result);\n'
  '}',
  'static void ps4_l2cap_disconnect_cfm_cback(uint16_t l2cap_cid, uint16_t result) {\n'
  '  ESP_LOGI(PS4_TAG, "[%s] l2cap_cid: 0x%02x\\n  result: %d", __func__, l2cap_cid, result);\n'
  '  if (l2cap_cid && (l2cap_cid == l2cap_control_channel || l2cap_cid == l2cap_interrupt_channel)) {\n'
  '    if (l2cap_cid == l2cap_control_channel) l2cap_control_channel = 0;\n'
  '    if (l2cap_cid == l2cap_interrupt_channel) l2cap_interrupt_channel = 0;\n'
  '    ps4_l2cap_abort();\n'
  '  }\n'
  '}'),
 ('  if (p_buf->length > 2) {\n    parsePacket(p_buf->data);\n  }\n\n  osi_free(p_buf);',
  '  if (!p_buf) return;\n'
  '  if (!session_aborted && is_connected && l2cap_cid == l2cap_interrupt_channel && p_buf->length > 2) {\n'
  '    parsePacket(p_buf->data);\n'
  '  }\n'
  '  osi_free(p_buf);'),
 ('  ESP_LOGI(PS4_TAG, "[%s] l2cap_cid: 0x%02x\\n  p_cfg->result: %d", __func__, l2cap_cid, p_cfg->result);',
  '  ESP_LOGI(PS4_TAG, "[%s] config confirmation", __func__);'),
 ('  ESP_LOGI(PS4_TAG, "[%s] bd_addr: %s\\n  l2cap_cid: 0x%02x\\n  psm: %d\\n  id: %d", __func__, bd_addr, '
  'l2cap_cid, psm, l2cap_id);',
  '  ESP_LOGI(PS4_TAG, "[%s] incoming HID channel", __func__);'),
 ('  L2CA_CONNECT_RSP(bd_addr, l2cap_id, l2cap_cid, L2CAP_CONN_PENDING, L2CAP_CONN_PENDING, NULL, NULL);',
  '  if (!L2CA_CONNECT_RSP(bd_addr, l2cap_id, l2cap_cid, L2CAP_CONN_PENDING, L2CAP_CONN_PENDING, NULL, '
  'NULL)) {\n'
  '    L2CA_DisconnectReq(l2cap_cid);\n'
  '    return;\n'
  '  }'),
 ('  L2CA_CONNECT_RSP(bd_addr, l2cap_id, l2cap_cid, L2CAP_CONN_OK, L2CAP_CONN_OK, NULL, NULL);',
  '  if (!L2CA_CONNECT_RSP(bd_addr, l2cap_id, l2cap_cid, L2CAP_CONN_OK, L2CAP_CONN_OK, NULL, NULL)) {\n'
  '    L2CA_DisconnectReq(l2cap_cid);\n'
  '    return;\n'
  '  }'),
 ('static void ps4_l2cap_connect_cfm_cback(uint16_t l2cap_cid, uint16_t result) {\n'
  '  ESP_LOGI(PS4_TAG, "[%s] l2cap_cid: 0x%02x\\n  result: %d", __func__, l2cap_cid, result);\n'
  '}',
  'static void ps4_l2cap_connect_cfm_cback(uint16_t l2cap_cid, uint16_t result) {\n'
  '  ESP_LOGI(PS4_TAG, "[%s] l2cap_cid: 0x%02x\\n  result: %d", __func__, l2cap_cid, result);\n'
  '  if (result != L2CAP_CONN_OK && l2cap_cid &&\n'
  '      (l2cap_cid == l2cap_control_channel || l2cap_cid == l2cap_interrupt_channel)) {\n'
  '    ps4_l2cap_abort();\n'
  '  }\n'
  '}'),
 ('  ESP_LOGI(PS4_TAG, "[%s] l2cap_cid: 0x%02x\\n  p_cfg->result: %d\\n  p_cfg->mtu_present: %d\\n  '
  'p_cfg->mtu: %d", __func__, l2cap_cid, p_cfg->result, p_cfg->mtu_present, p_cfg->mtu);\n'
  '\n'
  '  p_cfg->result = L2CAP_CFG_OK;\n'
  '\n'
  '  L2CA_ConfigRsp(l2cap_cid, p_cfg);',
  '  if (session_aborted || !l2cap_cid ||\n'
  '      (l2cap_cid != l2cap_control_channel && l2cap_cid != l2cap_interrupt_channel)) return;\n'
  '  if (!p_cfg) { ps4_l2cap_abort(); return; }\n'
  '  p_cfg->result = L2CAP_CFG_OK;\n'
  '  if (!L2CA_ConfigRsp(l2cap_cid, p_cfg)) ps4_l2cap_abort();')]


def patch(path):
    path = Path(path)
    raw = path.read_bytes()
    normalized = raw.replace(b'\r\n', b'\n')
    digest = hashlib.sha256(normalized).hexdigest()
    # Full SHA, not anchor-only matching. Explicit UTF-8, binary writes.
    if digest == PATCHED_SHA256:
        return False
    if digest != ORIGINAL_SHA256:
        raise RuntimeError('Unexpected PS4 1.1.1 L2CAP source SHA-256=' + digest + '; refusing unverified patch')
    text = normalized.decode('utf-8')
    for before, after in CHANGES:
        if text.count(before) != 1:
            raise RuntimeError('PS4 L2CAP patch anchor mismatch')
        text = text.replace(before, after, 1)
    output = text.encode('utf-8')
    if hashlib.sha256(output).hexdigest() != PATCHED_SHA256:
        raise RuntimeError('PS4 L2CAP patch output mismatch')
    if b'\r\n' in raw:
        output = output.replace(b'\n', b'\r\n')
    path.write_bytes(output)
    return True

try:
    Import('env')
except NameError:
    env = None
if env is not None:
    root = Path(env.subst('$PROJECT_LIBDEPS_DIR')) / env.subst('$PIOENV') / 'PS4_Controller_Host'
    if 'version=1.1.1' not in (root / 'library.properties').read_bytes().decode('utf-8').splitlines():
        raise RuntimeError('Unexpected PS4 library version; refusing L2CAP patch')
    patch(root / 'src/ps4_l2cap.c')
