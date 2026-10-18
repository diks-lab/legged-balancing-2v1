#pragma once
#include <esp_bt.h>
#include <esp32-hal-bt.h>

// Boot-only. Never accept an already-running controller whose mode is unknown,
// and never stop/restart Bluetooth or alter pairing addresses here.
enum class Ps4ClassicStartup { Ready, AlreadyInitialized, StartFailed, NotEnabled };
inline Ps4ClassicStartup preparePs4ClassicBluetooth() {
  if (esp_bt_controller_get_status() != ESP_BT_CONTROLLER_STATUS_IDLE) {
    return Ps4ClassicStartup::AlreadyInitialized;
  }
  // Arduino's public API selects cfg.mode=CLASSIC_BT, releases unused BLE
  // controller memory before init, and enables that same mode.
  if (!btStartMode(BT_MODE_CLASSIC_BT)) return Ps4ClassicStartup::StartFailed;
  if (esp_bt_controller_get_status() != ESP_BT_CONTROLLER_STATUS_ENABLED) {
    return Ps4ClassicStartup::NotEnabled;
  }
  return Ps4ClassicStartup::Ready;
}
