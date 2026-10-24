"""Exercise the actual app gate and installed Arduino btStartMode implementation."""
from pathlib import Path
import re, subprocess, tempfile, os
core=Path(os.environ.get('PLATFORMIO_CORE_DIR','/workspace/pio-core'))/'packages/framework-arduinoespressif32/cores/esp32/esp32-hal-bt.c'
text=core.read_text(encoding='utf-8')
m=re.search(r'^bool btStartMode\(bt_mode mode\) \{',text,re.M);assert m
end=m.end();depth=1
while depth:depth+=(text[end]=='{')-(text[end]=='}');end+=1
function=text[m.start():end]
# Pin the mechanism we rely on without modifying the Arduino core.
assert 'cfg.mode = esp_bt_mode;' in function
assert 'esp_bt_controller_mem_release(ESP_BT_MODE_BLE);' in function
code=r'''
#include <cassert>
#include <cstdio>
#define CONFIG_IDF_TARGET_ESP32 1
#define log_e(...) ((void)0)
enum bt_mode {BT_MODE_DEFAULT,BT_MODE_BLE,BT_MODE_CLASSIC_BT,BT_MODE_BTDM};
enum esp_bt_mode_t {ESP_BT_MODE_BLE=1,ESP_BT_MODE_CLASSIC_BT=2,ESP_BT_MODE_BTDM=3};
enum {ESP_BT_CONTROLLER_STATUS_IDLE,ESP_BT_CONTROLLER_STATUS_INITED,ESP_BT_CONTROLLER_STATUS_ENABLED};
using esp_err_t=int;
constexpr int ESP_OK=0;
#define BT_MODE ESP_BT_MODE_BTDM
struct esp_bt_controller_config_t {esp_bt_mode_t mode=ESP_BT_MODE_BTDM;};
#define BT_CONTROLLER_INIT_CONFIG_DEFAULT() esp_bt_controller_config_t{}
int state=0,release_calls=0,init_calls=0,enable_calls=0;
bool fail_init=false,fail_enable=false,misreport_enabled=false;
int esp_bt_controller_get_status(){return state;}
int esp_bt_controller_mem_release(esp_bt_mode_t mode){assert(mode==ESP_BT_MODE_BLE&&state==0);++release_calls;return 0;}
int esp_bt_controller_init(esp_bt_controller_config_t*cfg){assert(cfg->mode==ESP_BT_MODE_CLASSIC_BT);++init_calls;if(fail_init)return -1;state=1;return 0;}
int esp_bt_controller_enable(esp_bt_mode_t mode){assert(mode==ESP_BT_MODE_CLASSIC_BT);++enable_calls;if(fail_enable)return -1;state=misreport_enabled?1:2;return 0;}
bool btStartMode(bt_mode);
#include "Ps4ClassicBluetooth.h"
'''+function+r'''
void reset(){state=release_calls=init_calls=enable_calls=0;fail_init=fail_enable=misreport_enabled=false;}
int main(){
 reset();assert(preparePs4ClassicBluetooth()==Ps4ClassicStartup::Ready);
 assert(state==2&&release_calls==1&&init_calls==1&&enable_calls==1);
 assert(preparePs4ClassicBluetooth()==Ps4ClassicStartup::AlreadyInitialized);
 assert(release_calls==1&&init_calls==1&&enable_calls==1);
 reset();state=1;assert(preparePs4ClassicBluetooth()==Ps4ClassicStartup::AlreadyInitialized&&release_calls==0&&init_calls==0);
 reset();fail_init=true;assert(preparePs4ClassicBluetooth()==Ps4ClassicStartup::StartFailed&&enable_calls==0);
 reset();fail_enable=true;assert(preparePs4ClassicBluetooth()==Ps4ClassicStartup::StartFailed&&state==1);
 reset();misreport_enabled=true;assert(preparePs4ClassicBluetooth()==Ps4ClassicStartup::StartFailed);
 puts("PASS: actual Arduino Classic mode/BLE release order, idle gate, repeated init refusal, init/enable failures; no core patch");
}
'''
with tempfile.TemporaryDirectory() as td:
 root=Path(td)
 for name in ['esp_bt.h','esp32-hal-bt.h']:(root/name).write_text('// Host API declarations precede include.\n')
 cpp=root/'test.cpp';cpp.write_text(code,encoding='utf-8');exe=root/'test'
 subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-I'+str(root),'-Iinclude',str(cpp),'-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
