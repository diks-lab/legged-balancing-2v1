"""Compile actual transport/caller functions; inject OOM/stack failures."""
from pathlib import Path
import hashlib, importlib.util, os, re, subprocess, sys, tempfile
sys.dont_write_bytecode = True
spec = importlib.util.spec_from_file_location('patcher', 'scripts/patch_ps4_l2cap.py')
p = importlib.util.module_from_spec(spec); spec.loader.exec_module(p)
lib=Path('.pio/libdeps/esp32dev/PS4_Controller_Host/src')
raw=(lib/'ps4_l2cap.c').read_bytes().replace(b'\r\n',b'\n')
original=raw.decode('utf-8')
if hashlib.sha256(raw).hexdigest()==p.PATCHED_SHA256:
    for before,after in reversed(p.CHANGES): original=original.replace(after,before,1)
assert hashlib.sha256(original.encode()).hexdigest()==p.ORIGINAL_SHA256
with tempfile.TemporaryDirectory() as td:
    root=Path(td); target=root/'ps4_l2cap.c'
    for eol in ['\n','\r\n']:
        target.write_bytes(original.replace('\n',eol).encode())
        assert p.patch(target)
        expected=target.read_bytes(); stamp=target.stat().st_mtime_ns
        assert not p.patch(target) and target.read_bytes()==expected and target.stat().st_mtime_ns==stamp
        target.write_bytes(expected+b'// unexpected\n')
        try: p.patch(target)
        except RuntimeError: pass
        else: raise AssertionError('unknown source accepted')
        assert target.read_bytes()==expected+b'// unexpected\n'
    target.write_bytes(original.encode()); p.patch(target)
    source=target.read_text(encoding='utf-8')
    callers=(lib/'ps4.c').read_text(encoding='utf-8')
    def fn(text,name):
        m=re.search(r'^(?:static )?(?:void|bool)\s+'+name+r'\([^;]*?\)\s*\{',text,re.M); assert m,name
        i=m.end();depth=1
        while depth: depth+=(text[i]=='{')-(text[i]=='}');i+=1
        return text[m.start():i]
    prefix=r'''
#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <vector>
#include <string>
#include <algorithm>
using BD_ADDR=uint8_t[6];
struct BT_HDR {uint16_t event,length,offset,layer_specific;uint8_t data[1];};
struct tL2CAP_CFG_INFO {int result=0;bool mtu_present=false;int mtu=0;};
static tL2CAP_CFG_INFO ps4_cfg_info;
struct hid_cmd_t {uint8_t code,identifier,data[77];};
struct ps4_cmd_t {uint8_t smallRumble=0,largeRumble=0,r=0,g=0,b=0,flashOn=0,flashOff=0;};
struct ps4_t{};struct ps4_event_t{};
constexpr int BT_DEFAULT_BUFFER_SIZE=4112,L2CAP_MIN_OFFSET=13;
constexpr int L2CAP_DW_SUCCESS=1,L2CAP_DW_CONGESTED=2,L2CAP_DW_FAILED=0;
constexpr int L2CAP_CFG_OK=0,L2CAP_CONN_PENDING=1,L2CAP_CONN_OK=0;
constexpr int BT_PSM_HID_CONTROL=17,BT_PSM_HID_INTERRUPT=19;
constexpr int MALLOC_CAP_DEFAULT=4096,MALLOC_CAP_INTERNAL=2048,MALLOC_CAP_8BIT=4;
constexpr int hid_cmd_code_set_report=0x50,hid_cmd_code_type_feature=3,hid_cmd_identifier_ps4_enable=0xf4;
constexpr int hid_cmd_code_type_output=2,hid_cmd_identifier_ps4_control=0x11;
constexpr int ps4_control_packet_index_small_rumble=5,ps4_control_packet_index_large_rumble=6;
constexpr int ps4_control_packet_index_red=7,ps4_control_packet_index_green=8,ps4_control_packet_index_blue=9;
constexpr int ps4_control_packet_index_flash_on_time=10,ps4_control_packet_index_flash_off_time=11;
static const uint8_t hid_cmd_payload_ps4_enable[]={0x43,0x02};
int allocations=0,frees=0,writes=0,fail_at=0,write_result=1,packets=0,connects=0,disconnects=0;
bool connect_rsp_accept=true,config_rsp_accept=true,cfg_accept=true,disconnect_accept=true,is_active=false;
std::vector<int> close_requests,acks;std::vector<unsigned> lengths;std::vector<void*> live;
std::vector<std::string> logs;
void log_error(const char*fmt,...){logs.push_back(fmt);}
#define ESP_LOGE(tag,...) log_error(__VA_ARGS__)
#define ESP_LOGI(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
size_t heap_caps_get_free_size(int){return 1000;}
size_t heap_caps_get_largest_free_block(int){return 800;}
void* osi_malloc(size_t n){assert(n==4112);++allocations;if(allocations==fail_at)return nullptr;
 void*q=calloc(1,n);live.push_back(q);return q;}
void osi_free(void*q){auto it=std::find(live.begin(),live.end(),q);assert(it!=live.end());live.erase(it);free(q);++frees;}
int L2CA_DataWrite(uint16_t cid,BT_HDR*q){assert(cid && q);++writes;lengths.push_back(q->length);osi_free(q);return write_result;}
bool L2CA_DisconnectReq(uint16_t cid){close_requests.push_back(cid);return disconnect_accept;}
void L2CA_DisconnectRsp(uint16_t cid){acks.push_back(cid);}
#define L2CA_CONNECT_RSP(...) connect_rsp_accept
bool L2CA_ConfigReq(uint16_t,tL2CAP_CFG_INFO*){return cfg_accept;}
#define L2CA_CONFIG_REQ L2CA_ConfigReq
bool L2CA_ConfigRsp(uint16_t,tL2CAP_CFG_INFO*){return config_rsp_accept;}
void parsePacket(uint8_t*){++packets;}
void connection(uint8_t active){if(active)++connects;else ++disconnects;}
void (*ps4_connection_cb)(uint8_t)=connection;
void (*ps4_connection_object_cb)(void*,uint8_t)=nullptr;void*ps4_connection_object=nullptr;
void (*ps4_event_cb)(ps4_t,ps4_event_t)=nullptr;
void (*ps4_event_object_cb)(void*,ps4_t,ps4_event_t)=nullptr;void*ps4_event_object=nullptr;
void ps4ConnectEvent(uint8_t);void ps4Enable();void ps4SetLed(uint8_t,uint8_t,uint8_t);
void ps4Cmd(ps4_cmd_t);void ps4PacketEvent(ps4_t,ps4_event_t);
'''
    state=source[source.index('bool is_connected ='):source.index('/********************************************************************************/',source.index('bool is_connected ='))]
    code=prefix+'\n'+state+'\n'
    for name in ['ps4_l2cap_send_hid','ps4_l2cap_connect_ind_cback','ps4_l2cap_config_cfm_cback',
                 'ps4_l2cap_connect_cfm_cback','ps4_l2cap_config_ind_cback',
                 'ps4_l2cap_disconnect_ind_cback','ps4_l2cap_disconnect_cfm_cback','ps4_l2cap_data_ind_cback']:
        code+=fn(source,name)+'\n'
    for name in ['ps4Enable','ps4Cmd','ps4SetLed','ps4ConnectEvent','ps4PacketEvent']:
        code+=fn(callers,name)+'\n'
    code+=r'''
void reset(){assert(live.empty());l2cap_control_channel=l2cap_interrupt_channel=0;
 is_connected=control_configured=interrupt_configured=session_aborted=is_active=false;
 allocations=frees=writes=fail_at=packets=connects=disconnects=0;write_result=1;connect_rsp_accept=config_rsp_accept=cfg_accept=disconnect_accept=true;
 close_requests.clear();acks.clear();lengths.clear();logs.clear();}
void incoming(int cid,int psm){BD_ADDR bd{};ps4_l2cap_connect_ind_cback(bd,cid,psm,1);}
void setup_channels(){incoming(10,17);incoming(11,19);}
void config(int cid,int result=0){tL2CAP_CFG_INFO c;c.result=result;ps4_l2cap_config_cfm_cback(cid,&c);}
void input(int cid=11){auto*q=(BT_HDR*)calloc(1,128);q->length=3;live.push_back(q);ps4_l2cap_data_ind_cback(cid,q);}
void start(){setup_channels();config(10);config(11);}
int main(){
 // First enable allocation fails: no dereference, LED retry, write, or activation.
 reset();fail_at=1;start();assert(allocations==1&&writes==0&&session_aborted&&!is_connected&&!is_active);
 assert(close_requests.size()==2&&logs.size()==1);assert(logs[0].find("requested=%u")!=std::string::npos);
 input();config(11);ps4Enable();assert(allocations==1&&packets==0&&connects==0);
 // Actual enable succeeds, then LED fails; abort without false connection.
 reset();fail_at=2;start();assert(allocations==2&&writes==1&&lengths[0]==4&&session_aborted);
 input();assert(!is_active&&connects==0&&live.empty());
 // Both reports succeed; first accepted input drives existing active notification.
 reset();start();assert(writes==2&&lengths[0]==4&&lengths[1]==79&&is_connected);
 ps4PacketEvent({},{});assert(is_active&&connects==1);
 // Runtime command OOM notifies disconnection exactly once through existing path.
 fail_at=allocations+1;ps4SetLed(1,2,3);assert(!is_active&&disconnects==1&&session_aborted);
 ps4_l2cap_disconnect_ind_cback(11,true);ps4_l2cap_disconnect_cfm_cback(10,0);
 assert(disconnects==1&&acks.size()==1);input();assert(packets==0);
 // Explicit reconnect restores transport only via fresh channels/config.
 setup_channels();config(11);assert(!is_connected);config(10);assert(is_connected&&writes==4);
 input();assert(packets==1);ps4PacketEvent({},{});assert(connects==2);
 // Failed DataWrite consumes buffer: no double free, no LED following failed enable.
 reset();write_result=0;start();assert(writes==1&&allocations==1&&frees==1&&session_aborted);
 // Congestion is accepted, stack owns memory; no manual free/retry.
 reset();write_result=2;start();assert(writes==2&&is_connected&&live.empty());
 // Config failure/interruption/stale confirmations never activate or send.
 reset();setup_channels();config(10,1);config(11);input();assert(session_aborted&&allocations==0&&packets==0);
 reset();setup_channels();ps4_l2cap_disconnect_ind_cback(10,true);config(11);input();assert(allocations==0&&packets==0);
 reset();cfg_accept=false;incoming(10,17);assert(session_aborted&&close_requests.size()==1);
 reset();connect_rsp_accept=false;incoming(10,17);assert(!l2cap_control_channel&&!is_connected&&allocations==0&&close_requests.size()==1);
 reset();setup_channels();ps4_l2cap_connect_cfm_cback(10,1);assert(session_aborted&&allocations==0);
 reset();setup_channels();config_rsp_accept=false;tL2CAP_CFG_INFO cf;ps4_l2cap_config_ind_cback(10,&cf);assert(session_aborted);
 // Failure to request disconnect remains locally aborted, no memory/retry loop.
 reset();disconnect_accept=false;fail_at=1;start();input();ps4Enable();assert(session_aborted&&allocations==1&&packets==0);
 // Unknown/late channels and duplicate config cannot change accepted session.
 reset();start();config(999);config(10);assert(writes==2&&is_connected);
 input(999);assert(packets==0);ps4_l2cap_disconnect_ind_cback(999,true);assert(is_connected);
 ps4_l2cap_data_ind_cback(11,nullptr);config(0);ps4_l2cap_config_cfm_cback(11,nullptr);assert(session_aborted);
 assert(live.empty());puts("PASS: real HID enable/LED callers, OOM first/second/runtime, abort, reconnect, config/disconnect failure, stale data, ownership, congestion, LF/CRLF/idempotence/drift");
}
'''
    cpp=root/'test.cpp';cpp.write_text(code,encoding='utf-8')
    exe=root/'test'
    sanitizer_flags = ['-fsanitize=address,undefined','-fno-omit-frame-pointer'] if os.environ.get('PS4_L2CAP_SANITIZERS') else []
    subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Wno-unused-parameter','-Wno-unused-function','-Wno-missing-field-initializers',str(cpp),'-o',str(exe)]+sanitizer_flags,check=True)
    subprocess.run([str(exe)],check=True)
