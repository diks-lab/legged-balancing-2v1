"""Actual fixed-format storage and recovery helpers with NVS faults injected."""
from pathlib import Path
import subprocess,tempfile
source=r'''
#include "WheelCalibration.h"
#include "FallAutoRecovery.h"
#include <array>
#include <vector>
#include <cassert>
#include <cstdio>
struct Backend {
 std::array<std::vector<unsigned char>,2> slots;
 bool fail=false,partial=false,readback=false;
 size_t read(int slot,void*p,size_t size){
  const auto&v=slots[slot];if(readback)return 0;
  if(v.size()==size)memcpy(p,v.data(),size);return v.size();
 }
 bool write(int slot,const void*p,size_t size){
  if(fail)return false;
  const auto*b=static_cast<const unsigned char*>(p);
  slots[slot].assign(b,b+(partial?size/2:size));return !partial;
 }
};
int main(){
 Backend b;wheelcal::Record current,candidate,out;int slot=-1,other=-1;
 candidate.wheels[0]={-1,7,.49f,1};candidate.wheels[1]={1,7,1.2f,1};
 assert(wheelcal::load(b,42,current,slot)==wheelcal::Load::Missing);
 assert(wheelcal::save(b,42,candidate,current,slot));assert(slot==0);
 // New reader/global state models power-cycle restoration.
 assert(wheelcal::load(b,42,out,other)==wheelcal::Load::Valid);
 assert(out.wheels[0].direction==-1&&out.wheels[1].zero==1.2f);
 const auto good=b.slots[0];candidate.wheels[0].zero=.7f;
 b.readback=true;assert(!wheelcal::save(b,42,candidate,current,slot));b.readback=false;
 assert(b.slots[0]==good && current.wheels[0].zero==.49f);
 b.slots[1].clear();
 b.fail=true;assert(!wheelcal::save(b,42,candidate,current,slot));b.fail=false;
 assert(b.slots[0]==good && current.wheels[0].zero==.49f);
 b.partial=true;assert(!wheelcal::save(b,42,candidate,current,slot));b.partial=false;
 assert(wheelcal::load(b,42,out,other)==wheelcal::Load::Valid && other==0);
 assert(wheelcal::save(b,42,candidate,current,slot));assert(slot==1);
 b.slots[1][0]^=1;assert(wheelcal::load(b,42,out,other)==wheelcal::Load::Valid&&other==0);
 b.slots[0][0]^=1;assert(wheelcal::load(b,42,out,other)==wheelcal::Load::Invalid);
 b.slots={};slot=-1;assert(wheelcal::save(b,42,candidate,current,slot));
 assert(wheelcal::load(b,43,out,other)==wheelcal::Load::Invalid);
 for(int kind=0;kind<7;++kind){
  auto r=current;
  if(kind==0)r.version++;
  if(kind==1)r.wheels[0].direction=0;
  if(kind==2)r.wheels[0].zero=NAN;
  if(kind==3)r.wheels[1].zero=-1;
  if(kind==4)r.wheels[1].zero=7;
  if(kind==5)r.wheels[0].pp=0;
  if(kind==6)r.wheels[1].poles=8;
  r.checksum=wheelcal::crc(r);assert(!wheelcal::valid(r,42));
 }
 FallAutoRecovery f;assert(!f.tick(true,true,10000));f.arm();
 assert(!f.tick(true,true,0));assert(!f.tick(true,true,1499));
 assert(!f.tick(true,false,1500));assert(!f.tick(true,true,1501));
 assert(f.tick(true,true,3001));assert(!f.tick(true,true,9000));
 f.arm();assert(!f.tick(false,true,9001)&&!f.pending);
 f.arm();f.tick(true,true,0xfffffff0);assert(f.tick(true,true,0x5cc));
 puts("PASS: 52-byte dual-slot records, restore, CRC/version/config/range/PP validation, write/power-loss faults preserve previous slot; 1500 ms recovery, interrupted hold and wrap");
}
'''
with tempfile.TemporaryDirectory() as td:
 p=Path(td)/'test.cpp';p.write_text(source);exe=Path(td)/'test'
 subprocess.run(['c++','-std=c++17','-Wall','-Wextra','-Werror','-Wno-misleading-indentation','-Iinclude',str(p),'-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
