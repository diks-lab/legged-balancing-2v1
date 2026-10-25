#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>
#include "GyroVibration.h"
#include "HealthDiagnostics.h"
#include "WheelReadDiagnostics.h"
#include "LegControl.h"
namespace diagnostic {
constexpr unsigned FieldCount=32;
constexpr uint32_t PermanentMask=(1u<<0)|(1u<<1)|(1u<<14)|(1u<<15);
static_assert(FieldCount<=32,"subscription mask capacity");
static const char* const fields[FieldCount]={
 "angle_deg","reference_deg","angle_error_deg","gyro_deg_s",
 "right_position_rad","left_position_rad","right_velocity_rad_s","left_velocity_rad_s","mean_velocity_rad_s",
 "angle_term_v","gyro_term_v","position_term_v","speed_term_v","total_output_v",
 "right_command_v","left_command_v","right_foc_q_v","left_foc_q_v",
 "control_dt_ms","loop_us","right_foc_us","left_foc_us","imu_us","move_us","internal_free_b","internal_largest_b",
 "control_angle_error_deg","position_error_rad","target_speed_rad_s","target_tilt_deg","yaw_command_v","speed_raw_rad_s"};
struct Snapshot {
 uint32_t sample_ms=0,source_seq=0,foc_capture_us=0,stop_ms=0;
 uint32_t faults=0; bool running=false,disabled=true,ps4=false,foc_ready=false;
 char stop_reason[64]="NONE";
 uint32_t imu_ms=0,imu_seq=0,command_ms=0;
 bool imu_valid=false,reference_valid=false,command_valid=false;
 uint8_t imu_reason=1,state=0,imu_tx=255,imu_rx=0,recovery_checks=0;
 char calibration[32]="NOT_LOADED",recovery[32]="NONE";
 char startup_reason[32]="WEB_WAIT",right_error[24]="",left_error[24]="";
 leg::State legs;
 WheelReadDiagnostics wheel_reads[2];
 HealthReport health;
 GyroWindow gyro_window;
 uint8_t imu_config=255;
 float values[FieldCount]{};
};
// Reasons describe acquisition, not socket traffic. No I2C or heap here.
inline const char* imuReason(unsigned reason){
 static const char*const names[]={"OK","NOT_INITIALIZED","CALIBRATING","NO_SAMPLE",
 "RIGHT_WHEEL_FAULT","LEFT_WHEEL_FAULT","BOTH_WHEEL_FAULT","NONFINITE","LOOP_STALE","IMU_I2C_FAILED","CALIBRATION_FAILED"};
 return reason<11?names[reason]:"UNKNOWN";
}
inline const char* stateName(unsigned state){
 static const char*const names[]={"INITIALIZING","READY_WAIT","BALANCE_WAIT","BALANCING",
 "EXPLICIT_STOP","FALL_STOP","FAULT","STARTUP_FAILED","AUTO_WAIT","CALIBRATION_REQUIRED","CALIBRATING","FALL_RECOVERY_WAIT"};
 return state<12?names[state]:"UNKNOWN";
}
inline void freshness(Snapshot&s,uint32_t now){
 if(!s.imu_seq&&s.imu_reason==0)s.imu_reason=3;
 if(s.imu_reason==0&&uint32_t(now-s.imu_ms)>250)s.imu_reason=8;
 s.imu_valid=s.imu_reason==0;
 if(uint32_t(now-s.command_ms)>250)s.command_valid=false;
 if(!s.command_valid&&!s.disabled)s.values[14]=s.values[15]=NAN;
 if(!s.imu_valid){s.values[0]=s.values[2]=s.values[3]=s.values[26]=NAN;}
}
struct Json {
 char* data;size_t capacity,used=0;bool ok=true;
 Json(char*p,size_t n):data(p),capacity(n){if(n)data[0]=0;else ok=false;}
 void add(const char* fmt,...){if(!ok)return;va_list ap;va_start(ap,fmt);int n=vsnprintf(data+used,capacity-used,fmt,ap);va_end(ap);if(n<0||size_t(n)>=capacity-used){ok=false;return;}used+=n;}
 void string(const char*p){add("\"");for(;*p&&ok;++p){unsigned char c=*p;if(c=='"'||c=='\\')add("\\%c",c);else if(c<32)add("\\u%04x",c);else add("%c",c);}add("\"");}
};
// Strict small JSON grammar. Unknown/duplicate keys, escapes, fractional periods,
// oversized strings/arrays and trailing input are rejected atomically.
struct Subscription { uint32_t request=0,mask=0;bool stop=false; };
class SubscriptionParser {
 const char*p;void ws(){while(*p==' '||*p=='\r'||*p=='\n'||*p=='\t')++p;}
 bool token(char c){ws();if(*p!=c)return false;++p;return true;}
 bool text(char*out,size_t size){if(!token('"'))return false;size_t n=0;while(*p&&*p!='"'){if(*p=='\\'||unsigned(*p)<32||n+1>=size)return false;out[n++]=*p++;}if(*p!='"')return false;++p;out[n]=0;return true;}
 bool number(uint32_t& v){ws();v=0;if(*p<'0'||*p>'9')return false;if(*p=='0'&&p[1]>='0'&&p[1]<='9')return false;while(*p>='0'&&*p<='9'){unsigned d=*p++-'0';if(v>(UINT32_MAX-d)/10)return false;v=v*10+d;}return true;}
 public:
 explicit SubscriptionParser(const char* input):p(input){}
 bool parse(Subscription& out){Subscription s;unsigned seen=0;char type[12]="";if(!token('{'))return false;
  do{char key[16];if(!text(key,sizeof(key))||!token(':'))return false;
   unsigned bit=0;
   if(!strcmp(key,"type")){bit=1;if(!text(type,sizeof(type)))return false;}
   else if(!strcmp(key,"request_id")){bit=2;if(!number(s.request))return false;}
   else if(!strcmp(key,"period_ms")){bit=4;uint32_t period;if(!number(period)||period!=50)return false;}
   else if(!strcmp(key,"fields")){bit=8;if(!token('['))return false;ws();if(*p!=']')do{char name[40];if(!text(name,sizeof(name)))return false;unsigned i=0;while(i<FieldCount&&strcmp(name,fields[i]))++i;if(i==FieldCount||(s.mask&(1u<<i)))return false;s.mask|=1u<<i;ws();if(*p!=',')break;++p;}while(true);if(!token(']'))return false;}
   else return false;
   if(seen&bit)return false;
   seen|=bit;ws();if(*p!=',')break;++p;
  }while(true);
  if(!token('}'))return false;
  ws();if(*p)return false;
  if(!strcmp(type,"subscribe")){if(seen!=15||!s.mask)return false;}
  else if(!strcmp(type,"stop")){if(seen!=3)return false;s.stop=true;}
  else return false;
  out=s;return true;
 }
};
inline bool sample(char*dest,size_t capacity,const Snapshot&s,const char*boot,uint32_t generation,uint32_t seq,uint32_t dropped,uint32_t mask,uint32_t now){
 Json j(dest,capacity);j.add("{\"type\":\"%s\",\"boot_id\":\"%s\",\"subscription_id\":%lu,\"seq\":%lu,\"source_seq\":%lu,\"sample_ms\":%lu,\"foc_capture_us\":%lu,\"age_ms\":%lu,\"dropped\":%lu,\"stop_ms\":%lu,\"stop_reason\":",mask?"sample":"status",boot,(unsigned long)generation,(unsigned long)seq,(unsigned long)s.source_seq,(unsigned long)s.sample_ms,(unsigned long)s.foc_capture_us,(unsigned long)(now-s.sample_ms),(unsigned long)dropped,(unsigned long)s.stop_ms);j.string(s.stop_reason);
 j.add(",\"imu_ms\":%lu,\"imu_seq\":%lu,\"imu_valid\":%s,\"imu_reason\":\"%s\",\"reference_valid\":%s,\"state\":\"%s\",\"startup_reason\":",(unsigned long)s.imu_ms,(unsigned long)s.imu_seq,s.imu_valid?"true":"false",imuReason(s.imu_reason),s.reference_valid?"true":"false",stateName(s.state));
 j.string(s.startup_reason);j.add(",\"calibration\":");j.string(s.calibration);j.add(",\"recovery\":");j.string(s.recovery);j.add(",\"recovery_checks\":%u",unsigned(s.recovery_checks));j.add(",\"imu_tx\":%u,\"imu_rx\":%u",unsigned(s.imu_tx),unsigned(s.imu_rx));j.add(",\"command_ms\":%lu,\"command_valid\":%s",(unsigned long)s.command_ms,s.command_valid?"true":"false");j.add(",\"right_error\":");j.string(s.right_error);j.add(",\"left_error\":");j.string(s.left_error);
 const auto&h=s.health;const auto&m=h.mpu;
 j.add(",\"diagnosis\":[%lu,",(unsigned long)h.first_ms);j.string(h.first);
 for(auto stage:h.stages)j.add(",%u",unsigned(stage));
 j.add(",%u,%u,%u,%u,%u,%u,%d,%d,%d,%d,%u]",unsigned(h.barrier),unsigned(h.foc_reason[0]),unsigned(h.foc_reason[1]),unsigned(m.failure),unsigned(m.reg),unsigned(m.expected),int(m.actual),int(m.tx),int(m.rx),int(m.read),unsigned(h.imu_failure));
 if(mask&(1u<<3)) {
  const auto&w=s.gyro_window;
  j.add(",\"gyro_window\":[%u,%lu,%lu,%lu,%lu,%lu,%lu,",unsigned(s.imu_config),(unsigned long)w.end_ms,(unsigned long)w.duration_us,(unsigned long)w.count,(unsigned long)w.invalid,(unsigned long)w.min_dt_us,(unsigned long)w.max_dt_us);
  if(s.imu_valid&&isfinite(w.rms)&&uint32_t(now-w.end_ms)<=500)j.add("%.7g,%.7g]",double(w.rms),double(w.peak));else j.add("null,null]");
 }
 j.add(",\"faults\":%lu,\"running\":%s,\"disabled\":%s,\"ps4\":%s,\"foc_ready\":%s,\"values\":{",(unsigned long)s.faults,s.running?"true":"false",s.disabled?"true":"false",s.ps4?"true":"false",s.foc_ready?"true":"false");
 bool first=true;for(unsigned i=0;i<FieldCount;++i)if(mask&(1u<<i)){j.add("%s\"%s\":",first?"":",",fields[i]);if(isfinite(s.values[i]))j.add("%.7g",double(s.values[i]));else j.add("null");first=false;}
 j.add("}}");return j.ok;
}
// Separate bounded diagnostic frame: never enlarge the 2048-byte sample budget.
inline bool wheelReadMessage(char*dest,size_t capacity,const Snapshot&s,const char*boot,uint32_t source_age_ms=0){
 Json j(dest,capacity);j.add("{\"type\":\"wheel_diagnostics\",\"boot_id\":\"%s\"",boot);
 for(unsigned side=0;side<2;++side){
  const auto&d=s.wheel_reads[side];if(!d.latched)continue;
  j.add(",\"wheel%u\":[",side);
  const WheelReadRecord* records[]={&d.first,&d.probe};
  bool first=true;
  for(const auto*r:records){
   j.add("%s%lu,%lu,%lu,%d,%d,%d,%d,%d,%u,%u,%u,%u",first?"":",",
    (unsigned long)r->ms,(unsigned long)r->elapsed_us,(unsigned long)r->fall_ms,
    int(r->tx),int(r->received),int(r->available),int(r->high),int(r->low),
    unsigned(r->reg),unsigned(r->requested),unsigned(r->context),unsigned(r->result));
   first=false;
  }
  j.add(",%lu,%lu]",(unsigned long)d.probes,(unsigned long)d.failures);
 }
 const auto&l=s.legs;
 j.add(",\"leg_ms\":%lu,\"legs\":[",(unsigned long)s.sample_ms);
 const float values[]={l.height,l.requested_roll,l.applied_roll,l.measured_roll,l.command_valid?leg::percent(true,l.right_command):NAN,l.command_valid?leg::percent(false,l.left_command):NAN};
 bool first=true;for(float value:values){j.add("%s",first?"":",");if(isfinite(value))j.add("%.4g",double(value));else j.add("null");first=false;}
 j.add(",%d,%d,%lu,%lu,%u,%u,%u,%u,%u,%u,%lu,%lu,%lu]",int(l.right_command),int(l.left_command),
  (unsigned long)l.command_ms,(unsigned long)l.sync_ms,unsigned(l.reason),unsigned(l.torque),
  unsigned(l.active),unsigned(l.synced),unsigned(l.limited),unsigned(l.command_valid),
  (unsigned long)l.tx_frames,(unsigned long)l.tx_busy,(unsigned long)l.max_service_us);
 j.add(",\"leg_mode\":%u,\"leg_roll_confirmed\":%s,\"leg_roll_sign\":%d",unsigned(l.mode),
  leg::RollDirectionConfirmed?"true":"false",int(leg::ImuRollSign));
 const auto&p=l.positions;
 j.add(",\"leg_position\":[%lu,%u,%u,%u",(unsigned long)p.attempt_ms,
  unsigned(p.read_only),unsigned(p.acquired),unsigned(p.current));
 for(unsigned side=0;side<2;++side){
  const bool right=side==0,valid=(p.acquired&(1u<<side))!=0;
  const int16_t decoded=leg::decodePosition(p.word[side]);
  const int home=leg::position(right,0),end=leg::position(right,30);
  j.add(",[" );
  if(valid)j.add("%lu,%u,%u,%u,%d,%.8g",(unsigned long)p.received_ms[side],unsigned(p.word[side]),
   unsigned(p.word[side]&255),unsigned(p.word[side]>>8),int(decoded),double(leg::percent(right,decoded)));
  else j.add("null,null,null,null,null,null");
  j.add(",%d,%d,",int(fminf(home,end)),int(fmaxf(home,end)));
  if(valid)j.add("%s",leg::inRange(right,decoded)?"true":"false");else j.add("null");
  j.add(",%d,%d]",right?leg::RightHome:leg::LeftHome,right?leg::RightExtended:leg::LeftExtended);
 }
 j.add("]");
 const auto&o=l.operation;const auto&c=o.confirmed;
 j.add(",\"leg_age_ms\":%lu,\"leg_state\":",(unsigned long)source_age_ms);
 if(!o.committed)j.add("null");
 else {
  j.add("[%lu,%lu,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%lu,%u,%lu,%u,[",
   (unsigned long)c.sequence,(unsigned long)o.confirmed_ms,unsigned(o.mode),unsigned(o.allowed),
   unsigned(o.operation),unsigned(o.blocker),unsigned(c.raw),unsigned(c.keys),unsigned(c.flags),
   unsigned(c.phase),unsigned(c.gates),unsigned(o.failure),(unsigned long)o.failure_ms,
   unsigned(o.failure_source),(unsigned long)o.overwritten,unsigned(o.source));
  for(unsigned i=0;i<o.count;++i){const auto&e=o.events[(o.next+4-o.count+i)%4];
   j.add("%s[%lu,%lu,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u]",i?",":"",
    (unsigned long)e.sequence,(unsigned long)e.ms,unsigned(e.source),unsigned(e.from_reason),unsigned(e.reason),
    unsigned(e.from_phase),unsigned(e.phase),unsigned(e.raw),unsigned(e.keys),unsigned(e.flags),unsigned(e.gates),unsigned(e.mode));
  }
  j.add("]]");
 }
 const auto&a=l.acceptance;const auto&cfg=l.configuration;
 j.add(",\"leg_accept\":[%u,%u,%d,%d,%d,%d,%u,%u],\"leg_config\":[%lu,%lu,%lu,%u,%u,%u,%u,%u,%u",
  unsigned(a.extra),unsigned(a.valid()),int(a.right_min),int(a.right_max),int(a.left_min),int(a.left_max),
  unsigned(l.recovery),unsigned(l.recovery?leg::returnDirections(l.right_percent,l.left_percent):0),(unsigned long)cfg.started_ms,(unsigned long)cfg.finished_ms,
  (unsigned long)cfg.rejected_ms,unsigned(cfg.status),unsigned(cfg.error),unsigned(cfg.step),unsigned(cfg.acquired),unsigned(cfg.current),unsigned(cfg.reject_reason));
 for(unsigned side=0;side<2;++side){
  const unsigned limits=side*2,mode=limits+1;
  j.add(",[%lu,%lu,",(unsigned long)cfg.sample_ms[limits],(unsigned long)cfg.sample_ms[mode]);
  if(cfg.acquired&(1u<<limits))j.add("%u,%u",unsigned(cfg.minimum[side]),unsigned(cfg.maximum[side]));else j.add("null,null");
  j.add(",");if(cfg.acquired&(1u<<mode))j.add("%u",unsigned(cfg.mode[side]));else j.add("null");
  j.add("]");
 }
 j.add("]");
 j.add(",\"leg_geometry\":[");
 for(unsigned side=0;side<2;++side){const bool right=side==0;
  j.add("%s[%d,%d,%d,%d,%d,%d,%d,%d]",side?",":"",leg::zeroTick(right),leg::fullTick(right),
   leg::retractMargin(right),leg::extendMargin(right),leg::safeMinTick(right),leg::safeMaxTick(right),
   leg::normalMinTick(right),leg::normalMaxTick(right));
 }
 j.add(",[%.0f,%.0f]]",double(leg::MinPercent),double(leg::MaxPercent));
 const auto&m=l.measurement;
 j.add(",\"leg_measure\":[%lu,%lu,%lu,%lu,%u,%u,%u,%u,%u,%u",
  (unsigned long)m.started_ms,(unsigned long)m.finished_ms,(unsigned long)m.rejected_ms,(unsigned long)m.revision,
  unsigned(m.status),unsigned(m.acquired),unsigned(m.valid),unsigned(m.side),unsigned(m.error),unsigned(m.reject_reason));
 for(unsigned side=0;side<2;++side){
  j.add(",[%lu,%lu,",(unsigned long)m.sample_ms[side],(unsigned long)m.count[side]);
  if(m.acquired&(1u<<side))j.add("%d,%d,%d,%.6g,%.6g,%.6g",int(m.latest[side]),int(m.minimum[side]),int(m.maximum[side]),
   double(leg::percent(side==0,m.latest[side])),double(leg::percent(side==0,m.minimum[side])),double(leg::percent(side==0,m.maximum[side])));
  else j.add("null,null,null,null,null,null");
  j.add("]");
 }
 j.add("]");
 const auto&r=l.rx_failure;
 j.add(",\"leg_rx\":");
 if(!r.issues)j.add("null");
 else {
  j.add("[%lu,%lu,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,\"",
   (unsigned long)r.ms,(unsigned long)r.elapsed_ms,unsigned(r.target),unsigned(r.reg),unsigned(r.requested),unsigned(r.received),
   unsigned(r.issues),unsigned(r.frame_size),unsigned(r.expected_checksum),
   unsigned(r.echoes),unsigned(r.skipped),unsigned(r.truncated));
  for(unsigned i=0;i<r.raw_size&&i<sizeof(r.raw);++i)j.add("%02X",unsigned(r.raw[i]));
  j.add("\",\"");
  for(unsigned i=0;i<r.frame_size&&i<sizeof(r.frame);++i)j.add("%02X",unsigned(r.frame[i]));
  j.add("\",%u,%u]",unsigned(r.received_checksum),unsigned(r.checksum_ready));
 }
 j.add("}");return j.ok;
}
inline bool ack(char*dest,size_t cap,const Subscription&s,uint32_t generation,const char*boot){Json j(dest,cap);j.add("{\"type\":\"subscribed\",\"boot_id\":\"%s\",\"request_id\":%lu,\"subscription_id\":%lu,\"period_ms\":50,\"streaming\":%s,\"fields\":[",boot,(unsigned long)s.request,(unsigned long)generation,s.stop?"false":"true");bool first=true;for(unsigned i=0;i<FieldCount;++i)if(s.mask&(1u<<i)){j.add("%s\"%s\"",first?"":",",fields[i]);first=false;}j.add("]}");return j.ok;}
}
