#include "LegSerialDiagnostics.h"
#include <cassert>
#include <string>
#include <cstdio>
using namespace leg;
struct Sink {std::string text;int room=128;unsigned writes=0;int availableForWrite(){return room;}size_t write(const uint8_t*p,size_t n){assert(n<=48&&n<=unsigned(room));text.append((const char*)p,n);++writes;return n;}};
struct Fixture {
 SerialDiagnostics log;SerialOutput out;Sink sink;State s;
 Fixture(){s.active=s.synced=s.command_valid=true;s.operation.committed=true;s.operation.allowed=1;s.operation.blocker=255;
  s.operation.gates=Ready|Bench|Imu|Connected;s.stick.flags=97;s.right_command=s.tx.calculated_right=1990;s.left_command=s.tx.calculated_left=2130;s.tx.calculated_valid=true;
  s.command_ms=1000;s.tx.goals=1;s.measured_roll=.7f;}
 void drain(){for(unsigned i=0;i<300;++i){if(!log.detailed())out.quiet();if(out.idle()){SerialDiagnostics::Record r;uint32_t lost;if(log.pop(r,lost))out.begin(r,lost);else out.loss(lost);}out.service(sink);}}
 void tick(uint32_t now){log.capture(s,8,now);drain();}
 void fresh(unsigned side,uint32_t t){auto&f=s.following.side[side];f.status=2;f.sample_ms=t;f.request_ms=t-1;f.command_ms=s.command_ms;f.command=side?s.left_command:s.right_command;f.tick=f.command-2;}
};
int main(){
 {Fixture f;assert(!f.log.detailed());f.tick(1000);f.sink.text.clear();
  for(unsigned t=1100;t<7000;t+=100){f.fresh((t/100)%2,t);f.tick(t);}assert(f.sink.text.empty());
  f.log.request('i',f.s,8,7000);f.drain();assert(f.sink.text.find("ON ACCEPTED")!=std::string::npos);
  f.s.operation.raw_keys=1;f.s.stick.applied_height=1;f.tick(7100);f.fresh(0,7700);f.tick(7700);
  assert(f.sink.text.find("LEG FEEDBACK:")!=std::string::npos||f.sink.text.find("PROGRESS")!=std::string::npos);
  f.sink.text.clear();f.log.request('j',f.s,8,7800);f.tick(7800);
  for(unsigned t=7900;t<11000;t+=100){f.fresh((t/100)%2,t);f.tick(t);}
  assert(f.sink.text.find("OFF ACCEPTED")!=std::string::npos);
  assert(f.sink.text.find("PROGRESS")==std::string::npos&&f.sink.text.find("LEG FEEDBACK:")==std::string::npos&&f.sink.text.find("PS4 AXES")==std::string::npos);
  f.sink.text.clear();f.log.request('k',f.s,8,11000);f.s.right_command=1234;f.drain();
  assert(f.sink.text.find("STATUS ACCEPTED")!=std::string::npos&&f.sink.text.find("UART_RL=1990/2130")!=std::string::npos);
  assert(f.sink.text.find("LEG STATUS READ")!=std::string::npos&&f.sink.text.find("AngleX=")!=std::string::npos&&f.sink.text.find("LEG GATES: ready=1 balance=0 support=1")!=std::string::npos);
 }
 {Fixture f;f.tick(1000);f.sink.text.clear();f.s.operation.raw_keys=4;f.s.requested_roll=2;f.tick(1100);
  f.s.operation.raw_keys=0;f.tick(1200);assert(f.sink.text.find("LEG SUMMARY:")==std::string::npos);
  f.s.requested_roll=0;f.s.command_ms=1600;++f.s.tx.goals;f.tick(1600);
  f.fresh(0,1700);f.tick(1700);assert(f.sink.text.find("LEG SUMMARY:")==std::string::npos);
  f.fresh(1,1800);f.tick(1800);assert(f.sink.text.find("NEW_FEEDBACK (not arrival confirmed)")!=std::string::npos);
  const auto text=f.sink.text;f.tick(1900);f.tick(3000);assert(f.sink.text==text);
 }
 {Fixture f;f.tick(1000);f.s.operation.raw_keys=4;f.s.requested_roll=3;f.tick(1100);f.s.operation.raw_keys=0;f.tick(1200);
  f.tick(5199);assert(f.sink.text.find("TIME_LIMIT")==std::string::npos);f.tick(5200);assert(f.sink.text.find("COMMAND_CONTINUING")!=std::string::npos);
 }
 {Fixture f;f.tick(1000);f.s.operation.raw_keys=1;f.tick(1100);f.s.operation.raw_keys=0;f.tick(1200);
  f.s.operation.raw_keys=4;f.tick(1300);f.tick(1800);assert(f.sink.text.find("id=1 SUPERSEDED")!=std::string::npos);
  f.s.operation.raw_keys=0;f.tick(1900);f.tick(5900);assert(f.sink.text.find("id=2 TIME_LIMIT")!=std::string::npos);
 }
 {Fixture f;f.tick(1000);f.sink.text.clear();f.sink.room=0;const auto writes=f.sink.writes;
  for(unsigned t=1500;t<=6000;t+=500){auto&r=f.s.following.side[0];r.status=3;r.issues=RxTimeout;r.request_ms=t;f.tick(t);}
  assert(f.sink.writes==writes);f.sink.room=128;f.drain();
  const auto first=f.sink.text.find("FEEDBACK ERROR");assert(first!=std::string::npos&&f.sink.text.find("FEEDBACK ERROR",first+1)==std::string::npos);
  f.fresh(0,6500);f.tick(6500);assert(f.sink.text.find("FEEDBACK RECOVERED")!=std::string::npos&&f.sink.text.find("failures=10")!=std::string::npos);
 }
 // OFF cancels an unsent periodic frame and never drops an important frame.
 {SerialOutput out;Sink sink;sink.room=0;SerialDiagnostics::Record r;r.type=SerialDiagnostics::FollowingRead;out.begin(r,0);out.service(sink);out.quiet();assert(out.idle());
  r.type=SerialDiagnostics::SyncFailed;r.reason=uint8_t(Reason::ReplyTimeout);out.begin(r,0);out.quiet();sink.room=128;
  for(int i=0;i<20;++i){out.service(sink);}
  assert(sink.text.find("SYNC: FAILED")!=std::string::npos);
 }
 // Finish only a line already started when OFF arrives; never leak its next line.
 {SerialOutput out;Sink sink;SerialDiagnostics::Record r;r.type=SerialDiagnostics::Sticks;
  out.begin(r,0);sink.room=7;out.service(sink);out.quiet();sink.room=128;
  for(unsigned i=0;i<20;++i){out.service(sink);}
  assert(sink.text.find("PS4 AXES:")!=std::string::npos&&sink.text.find("PS4 APPLIED:")==std::string::npos&&out.idle());
 }
 {SerialDiagnostics log;State s;
  for(unsigned i=1;i<=8;++i){s.positions.attempt_ms=i;log.capture(s,2,i);}
  log.request('k',s,2,9);SerialDiagnostics::Record r;uint32_t lost;
  assert(log.pop(r,lost)&&r.type==SerialDiagnostics::LogControl&&r.reason==1&&!lost);
  log.request('j',s,2,10);assert(log.pop(r,lost)&&r.type==SerialDiagnostics::LogControl&&!r.reason);
  unsigned kept=0;while(log.pop(r,lost)){assert(!lost&&r.type==SerialDiagnostics::SyncStart);++kept;}assert(kept==8);
 }
 // Missing or failed feedback is reported once at the bounded deadline.
 {Fixture f;f.tick(1000);f.s.operation.raw_keys=1;f.tick(1100);f.s.operation.raw_keys=0;f.tick(1200);
  auto&r=f.s.following.side[1];r.status=3;r.issues=RxTimeout;r.request_ms=1500;f.tick(1500);
  f.tick(5200);assert(f.sink.text.find("FEEDBACK_INCOMPLETE")!=std::string::npos&&f.sink.text.find("error=UNKNOWN")!=std::string::npos);
  auto text=f.sink.text;f.tick(6200);assert(f.sink.text==text);
 }
 // Burst commands each receive an ACK; overflow rejects actions without waiting.
 {SerialDiagnostics log;State s;for(char action:std::string("ijij"))log.request(action,s,0,1000);
  log.request('i',s,0,1001);assert(!log.detailed());
  for(char action:std::string("ijij")){SerialDiagnostics::Record r;uint32_t lost;assert(log.pop(r,lost)&&!lost&&r.type==SerialDiagnostics::LogControl&&r.allowed==uint8_t(action));}
  SerialDiagnostics::Record r;uint32_t lost;assert(log.pop(r,lost)&&r.allowed=='?'&&r.data.motion.tx.goals==1);
 }
 // Existing colon parser consumes malformed/overlong lines without motor-key leaks.
 {LegCalSerialParser p;char action=0;unsigned count=0;
  for(char c:std::string(":leg log on\r\n:leg log off\n:leg status\r\n")){bool consumed=p.feed(c,action);if(action){assert(consumed);assert(action==char('i'+count));++count;}}
  assert(count==3);for(char c:std::string(":leg ")+std::string(80,'u')+"\n"){assert(p.feed(c,action));if(action)assert(action=='?');}
 }
 printf("PASS: default quiet, log ON/OFF/status, immutable snapshot, post-command fresh paired summary/timeout/supersede, fault latch/recovery counts, USB backpressure, bounded colon LF/CRLF parser\n");
}
