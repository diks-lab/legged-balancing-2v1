"""Compile the actual pinned/patched library notification functions and FIFO."""
from pathlib import Path
import importlib.util
import re
import sys
sys.dont_write_bytecode = True
import subprocess
import tempfile

spec = importlib.util.spec_from_file_location('disconnect_patch', 'scripts/patch_ps4_notifications.py')
patcher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(patcher)
lib = Path('.pio/libdeps/esp32dev/PS4_Controller_Host/src')
c_source = (lib/'ps4.c').read_text()
if patcher.ORIGINAL in c_source:
    c_source = c_source.replace(patcher.ORIGINAL, patcher.PATCHED, 1)
if patcher.ORIGINAL_PACKET in c_source:
    c_source = c_source.replace(patcher.ORIGINAL_PACKET, patcher.PATCHED_PACKET, 1)
assert patcher.PATCHED in c_source and patcher.PATCHED_PACKET in c_source
cpp_source = (lib/'PS4Controller.cpp').read_text()

def function(source, name):
    m = re.search(r'^(?:void|bool)\s+' + re.escape(name) + r'\([^;]*?\)\s*\{', source, re.M)
    assert m, name
    i, depth = m.end(), 1
    while depth:
        depth += (source[i]=='{') - (source[i]=='}')
        i += 1
    return source[m.start():i]

code = r'''
#include "Ps4EventInbox.h"
#include <cassert>
#include <cstring>
#include <mutex>
#include <thread>
#include <atomic>
#include <vector>
#include <cstdio>
struct Lock {std::mutex value;void lock(){value.lock();}void unlock(){value.unlock();}};
using Inbox=Ps4EventInbox<Lock,4>;
struct ps4_t {int value;};struct ps4_event_t {int value;};
struct PS4Controller {
  using callback_t=void(*)();
  ps4_t data{};ps4_event_t event{};
  void (*_callback_event)()=nullptr;
  void (*_callback_connect)()=nullptr;
  void (*_callback_disconnect)()=nullptr;
  void attach(void(*)());void attachOnConnect(void(*)());void attachOnDisconnect(void(*)());
  static void _event_callback(void*,ps4_t,ps4_event_t);
  static void _connection_callback(void*,uint8_t);
} controller;
bool is_active=false;int enables=0,delayed_ms=0,c_disconnects=0;
void delay(int ms){delayed_ms+=ms;}
void ps4Enable(){++enables;}
void (*ps4_connection_cb)(uint8_t)=nullptr;
void (*ps4_connection_object_cb)(void*,uint8_t)=PS4Controller::_connection_callback;
void* ps4_connection_object=&controller;
void (*ps4_event_cb)(ps4_t,ps4_event_t)=nullptr;
void (*ps4_event_object_cb)(void*,ps4_t,ps4_event_t)=PS4Controller::_event_callback;
void* ps4_event_object=&controller;
std::vector<int> notifications;
void connected(){notifications.push_back(1);}
void input(){notifications.push_back(controller.data.value);}
void disconnected(){assert(!is_active);notifications.push_back(-1);}
void c_disconnected(uint8_t active){if(!active)++c_disconnects;}
'''
for name in ['PS4Controller::attach','PS4Controller::attachOnConnect','PS4Controller::attachOnDisconnect',
             'PS4Controller::_event_callback','PS4Controller::_connection_callback']:
    code += function(cpp_source, name) + '\n'
for name in ['ps4ConnectEvent','ps4PacketEvent']:
    code += function(c_source, name) + '\n'
code += r'''
int main(){
  controller.attachOnConnect(connected);controller.attachOnDisconnect(disconnected);controller.attach(input);
  ps4_connection_cb=c_disconnected;
  ps4ConnectEvent(true);assert(enables==1 && !is_active && notifications.empty());
  ps4PacketEvent({10},{0}); // First input must be delivered after connection.
  assert((notifications==std::vector<int>{1,10}) && controller.data.value==10 && delayed_ms==250);
  ps4PacketEvent({42},{0});assert((notifications==std::vector<int>{1,10,42}));
  ps4ConnectEvent(false);assert((notifications==std::vector<int>{1,10,42,-1}) && c_disconnects==1);
  ps4ConnectEvent(false);assert(notifications.size()==4 && c_disconnects==1); // Two L2CAP channels, one edge.
  ps4PacketEvent({3},{0});ps4PacketEvent({99},{0});
  assert((notifications==std::vector<int>{1,10,42,-1,1,3,99}));
  Inbox fifo;Inbox::Batch batch{};Ps4InputState p{};
  fifo.connect(1);fifo.input(p,2);p.options=true;fifo.input(p,3);p.options=false;fifo.input(p,4);
  fifo.drain(batch);assert(batch.count==4 && batch.events[2].input.options && !batch.events[3].input.options);
  for(size_t i=1;i<batch.count;++i)assert(Inbox::after(batch.events[i].sequence,batch.events[i-1].sequence));
  for(int i=0;i<100;++i)fifo.input(p,5+i);
  fifo.drain(batch);assert(batch.count==0 && batch.duplicates==100);
  for(int i=0;i<8;++i){p.left_stick_y=-127;p.r2=i;fifo.input(p,110+i);}
  p.cross=true;fifo.input(p,120);p.cross=false;fifo.input(p,121);
  fifo.input({},122);fifo.disconnect(123);fifo.connect(124);fifo.input({},125);fifo.drain(batch);
  assert(batch.count==4 && batch.overflowed && batch.overflows>0 && batch.cross_pending && batch.disconnect_pending);
  assert(batch.connected && batch.connects==2 && batch.disconnects==1);
  assert(batch.lost_presses&1u);assert(batch.lost_releases&1u);
  assert(batch.presses>=2 && batch.releases>=2 && batch.neutral_pending);
  fifo.input({},126);fifo.drain(batch);assert(batch.count==1); // Fresh identical resync is not suppressed.
  assert(Inbox::after(0,UINT32_MAX));assert(!Inbox::after(UINT32_MAX,0));
  // Exercise multiple callback producers and a draining consumer with real mutexes.
  Inbox concurrent;std::atomic<int> finished{0};
  std::thread a([&]{Ps4InputState v{};for(int i=0;i<10000;++i){v.r2=i%255;concurrent.input(v,i);}++finished;});
  std::thread b([&]{for(int i=0;i<5000;++i){concurrent.connect(i);concurrent.disconnect(i);}++finished;});
  std::thread consumer([&]{Inbox::Batch out{};while(finished.load()!=2)concurrent.drain(out);});
  a.join();b.join();consumer.join();concurrent.drain(batch);
  assert(batch.received==20000 && batch.connects==5000 && batch.disconnects==5000);
  puts("PASS: real PS4 1.1.1 notification APIs/disconnect patch, first-packet behavior, FIFO ordering/duplicates/overflow latches, concurrent producers");
}
'''
with tempfile.TemporaryDirectory(prefix='ps4-events-') as directory:
    path = Path(directory)/'ps4.c'
    path.write_text(patcher.ORIGINAL+"\n"+patcher.ORIGINAL_PACKET)
    assert patcher.patch(path) and not patcher.patch(path)
    path.write_text(patcher.ORIGINAL+"\nunknown implementation")
    try:
        patcher.patch(path)
    except RuntimeError:
        assert path.read_text() == patcher.ORIGINAL+'\nunknown implementation'
    else:
        raise AssertionError('An unrecognized library must not silently build')
    cpp=Path(directory)/'test.cpp';binary=Path(directory)/'test';cpp.write_text(code)
    subprocess.run(['c++','-std=c++17','-Wall','-Wextra','-Werror','-pthread','-Iinclude',str(cpp),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)

main=Path('src/main.cpp').read_text()
for name in ['onPs4Connect','onPs4Disconnect','onPs4Input']:
    callback=function(main,name)
    for forbidden in ['Serial','Web','motor','requestLqr','execute','stopBalance','handlePs4Connection']:
        assert forbidden not in callback,(name,forbidden)
assert 'PS4.' not in function(main,'updatePs4Controller')
assert 'readPs4Input' not in function(main,'updatePs4Controller')
assert 'PS4.attachOnConnect(onPs4Connect)' in main
assert 'PS4.attachOnDisconnect(onPs4Disconnect)' in main
assert 'PS4.attach(onPs4Input)' in main
