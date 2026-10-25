#pragma once
#include "LegGeometry.h"
#include <stddef.h>
#include <string.h>
namespace leg {
// Separate from wheel FOC: leg_range / slot0, slot1. All integers are LE on ESP32.
enum class CalResult:uint8_t { Ok,StopRequired,Busy,NotEditing,Missing,Range,Direction,Span,ReadFailed,StorageOpen,StorageWrite,StorageVerify,NoSaved,Corrupt,Aborted,WorkerUnavailable };
inline const char* calResult(CalResult r){
 static const char*const names[]={"OK","SUPPORTED_STOP_TORQUE_OFF_REQUIRED","BUSY","NOT_EDITING","FOUR_POINTS_REQUIRED","TICK_OUT_OF_RANGE","DIRECTION_INVALID","STROKE_OR_NORMAL_RANGE_INVALID","READ_FAILED","NVS_OPEN_FAILED","NVS_WRITE_FAILED","NVS_VERIFY_FAILED","NOT_SAVED","NO_VALID_SLOT","ABORTED","SAVE_WORKER_UNAVAILABLE"};
 return unsigned(r)<sizeof(names)/sizeof(*names)?names[unsigned(r)]:"UNKNOWN";
}
inline CalResult validateGeometry(const Geometry&g){
 const int16_t v[]={g.right_zero,g.right_full,g.left_zero,g.left_full};
 for(auto n:v)if(n<0||n>4095)return CalResult::Range;
 if(g.right_zero<=g.right_full||g.left_zero>=g.left_full)return CalResult::Direction;
 return validGeometry(g)?CalResult::Ok:CalResult::Span;
}
inline uint32_t legCrc(const void*data,size_t n){
 uint32_t crc=~uint32_t(0);const auto*p=static_cast<const uint8_t*>(data);
 while(n--){crc^=*p++;for(unsigned i=0;i<8;++i)crc=(crc>>1)^(0xedb88320u&uint32_t(-int32_t(crc&1)));}return ~crc;
}
struct LegRecord {
 uint32_t magic=0x4c454731,generation=0;
 uint16_t version=1,bytes=sizeof(LegRecord);
 Geometry geometry;
 uint16_t right_span=0,left_span=0;
 uint8_t right_id=1,left_id=2,margin=10,normal_percent=30;
 uint32_t crc=0;
};
static_assert(sizeof(LegRecord)==32,"persistent layout changed: bump version and tests");
inline LegRecord recordFor(const Geometry&g,uint32_t generation){
 LegRecord r;r.geometry=g;r.generation=generation;r.right_span=g.right_zero-g.right_full;r.left_span=g.left_full-g.left_zero;
 r.crc=legCrc(&r,offsetof(LegRecord,crc));return r;
}
inline bool validRecord(const LegRecord&r){
 return r.magic==0x4c454731&&r.version==1&&r.bytes==sizeof(r)&&r.right_id==1&&r.left_id==2&&r.margin==10&&r.normal_percent==30&&
  validateGeometry(r.geometry)==CalResult::Ok&&r.right_span==r.geometry.right_zero-r.geometry.right_full&&
  r.left_span==r.geometry.left_full-r.geometry.left_zero&&r.crc==legCrc(&r,offsetof(LegRecord,crc));
}
inline int newestSlot(const LegRecord(&r)[2],const bool(&valid)[2]){
 if(!valid[0])return valid[1]?1:-1;
 if(!valid[1])return 0;
 // Wrap-safe for successive writes; equal/half-range generations prefer slot0.
 return int32_t(r[1].generation-r[0].generation)>0?1:0;
}
struct StoredLegs { Geometry geometry;uint32_t generation=0;CalResult result=CalResult::NoSaved;uint8_t slot=255,valid_slots=0; };
template<class Store> StoredLegs readOpenLegs(Store&store,bool&read_failed){
 StoredLegs out;LegRecord r[2];bool valid[2]{};bool present=false;read_failed=false;
 for(unsigned i=0;i<2;++i){
  const char*key=i?"slot1":"slot0";const size_t n=store.getBytesLength(key);present|=n!=0;
  if(n==sizeof(LegRecord)){
   const size_t received=store.getBytes(key,&r[i],sizeof(r[i]));
   if(received!=sizeof(r[i]))read_failed=true;
   else valid[i]=validRecord(r[i]);
  }
  if(valid[i])out.valid_slots|=1u<<i;
 }
 int slot=newestSlot(r,valid);
 if(slot<0){out.result=present?CalResult::Corrupt:CalResult::NoSaved;return out;}
 out.geometry=r[slot].geometry;out.generation=r[slot].generation;out.slot=slot;out.result=CalResult::Ok;return out;
}
template<class Store> StoredLegs loadLegs(Store&store){
 StoredLegs out;if(!store.begin("leg_range",true))return out;
 bool failed;out=readOpenLegs(store,failed);store.end();
 if(failed&&out.result!=CalResult::Ok)out.result=CalResult::StorageVerify;
 return out;
}
// Write only the inactive slot. NVS commits each blob; a torn/CRC-invalid slot is
// ignored on boot. The previous valid slot is never removed or overwritten.
template<class Store> StoredLegs saveLegs(Store&store,const Geometry&g){
 StoredLegs out;const auto validation=validateGeometry(g);
 if(validation!=CalResult::Ok){out.result=validation;return out;}
 if(!store.begin("leg_range",false)){out.result=CalResult::StorageOpen;return out;}
 bool failed;out=readOpenLegs(store,failed);
 if(failed){out.result=CalResult::StorageVerify;store.end();return out;}
 const unsigned slot=out.slot==0?1:0;const char*key=slot?"slot1":"slot0";
 const LegRecord r=recordFor(g,out.result==CalResult::Ok?out.generation+1:1);LegRecord verify;
 if(store.putBytes(key,&r,sizeof(r))!=sizeof(r))out.result=CalResult::StorageWrite;
 else if(store.getBytesLength(key)!=sizeof(r)||store.getBytes(key,&verify,sizeof(verify))!=sizeof(verify)||
    !validRecord(verify)||memcmp(&r,&verify,sizeof(r)))out.result=CalResult::StorageVerify;
 else {out.geometry=g;out.generation=r.generation;out.slot=slot;out.valid_slots|=1u<<slot;out.result=CalResult::Ok;}
 store.end();return out;
}
struct LegCalibration {
 Geometry candidate;
 uint32_t acquired_ms[4]{},revision=0,generation=0,event_ms=0;
 uint8_t mask=0,phase=0,point=255,source=0,valid_slots=0,imported=0;
 CalResult result=CalResult::NoSaved,storage=CalResult::NoSaved;
 // phase: idle, editing, reading, saving, saved/resync, aborted. source: none/NVS.
 bool active()const{return phase>=1&&phase<=3;}
 CalResult canSave()const{
  if(phase!=1)return phase==2||phase==3?CalResult::Busy:CalResult::NotEditing;
  if(mask!=15)return CalResult::Missing;
  if(result==CalResult::ReadFailed)return result;
  return validateGeometry(candidate);
 }
 void event(CalResult r,uint32_t now){result=r;event_ms=now;++revision;}
 bool begin(bool safe,uint32_t now){
  if(active()){event(CalResult::Busy,now);return false;}
  if(!safe){event(CalResult::StopRequired,now);return false;}
  candidate={};mask=0;imported=0;for(auto&t:acquired_ms)t=0;phase=1;point=255;event(CalResult::Ok,now);return true;
 }
 bool confirmed(uint32_t now){
  if(phase!=1){event(active()?CalResult::Busy:CalResult::NotEditing,now);return false;}
  candidate=ConfirmedCandidate;mask=15;imported=15;for(auto&t:acquired_ms)t=now;event(CalResult::Ok,now);return true;
 }
 bool capture(unsigned index,uint32_t now){
  if(phase!=1||index>3){event(active()?CalResult::Busy:CalResult::NotEditing,now);return false;}
  point=index;imported&=uint8_t(~(1u<<index));mask&=uint8_t(~(1u<<index));phase=2;event(CalResult::Ok,now);return true;
 }
 void received(int16_t tick,bool success,uint32_t now){
  if(phase!=2)return;
  if(success){switch(point){case 0:candidate.right_zero=tick;break;case 1:candidate.right_full=tick;break;case 2:candidate.left_zero=tick;break;case 3:candidate.left_full=tick;break;}
   acquired_ms[point]=now;mask|=1u<<point;}
  phase=1;event(success?CalResult::Ok:CalResult::ReadFailed,now);
 }
 bool abort(uint32_t now){if(phase==3){event(CalResult::Busy,now);return false;}if(!active()){event(CalResult::NotEditing,now);return false;}phase=5;event(CalResult::Aborted,now);return true;}
};
// ':' introduces a bounded line; no characters of an invalid/partial command
// can leak into legacy single-character motor commands.
struct LegCalSerialParser {
 char line[32]{};uint8_t used=0;bool active=false,overflow=false;
 bool feed(char c,char&action){
  action=0;if(!active){if(c!=':')return false;active=true;used=0;overflow=false;return true;}
  if(c=='\r'||c=='\n'){
   if(!overflow){line[used]=0;static const char*const names[]={"leg begin","leg candidate","leg r0","leg r100","leg l0","leg l100","leg save","leg abort"};
    for(unsigned i=0;i<8;++i)if(!strcmp(line,names[i]))action=char('a'+i);}
   if(!action)action='?';
   active=false;used=0;return true;
  }
  if(size_t(used)+1<sizeof(line)&&!overflow)line[used++]=c;else overflow=true;
  return true;
 }
};
}
