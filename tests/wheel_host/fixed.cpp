#include "../../src/wheel_diagnostic.cpp"
#include <cassert>
#include <iostream>
unsigned long _micros() { return micros(); }
void input(const std::string& s) { Serial.feed(s); while(Serial.available()) loop(); }
void disabled() { assert(!pins[27]&&!pins[32]); }
void drain() { while(txRead!=txWrite) pumpLog(); }
int main(int argc,char** argv) {
  assert(argc==2); const std::string test=argv[1];
  setup(); disabled();
  if(test=="left_first"||test=="left_first_both") {
    input("motor left +\n");input("stop\n");
    if(test=="left_first_both") {input("motor both +\n");input("stop\n");}
    const auto calls=leftMotor.initCalls+rightMotor.initCalls;
    input("voltage left +\n");disabled();
    assert(!running&&!nvsOpens&&leftMotor.initCalls+rightMotor.initCalls==calls);return 0;
  }
  if(test=="invalid_idle") {
    input("voltage both +\nvoltage left ++\nvoltage left +x\nvoltage right 0.5\nvoltage left + \n");
    disabled();assert(!nvsOpens&&!rightMotor.initCalls&&!leftMotor.initCalls);return 0;
  }
  wheelcal::Record rec;rec.config=savedConfig();rec.generation=5;
  rec.wheels[0]={-1,7,1.4419413f,1}; rec.wheels[1]={-1,7,4.3810487f,1};
  if(test=="config") rec.config++;
  if(test=="direction") rec.wheels[1].direction=0;
  if(test=="zero") rec.wheels[1].zero=NAN;
  if(test=="poles") rec.wheels[1].poles=8;
  if(test=="pp") rec.wheels[1].pp=0;
  rec.checksum=wheelcal::crc(rec);
  if(test=="crc") ++rec.checksum;
  const auto* bytes=reinterpret_cast<const uint8_t*>(&rec);
  if(test!="missing") nvsSlots[0].assign(bytes,bytes+sizeof(rec));
  if(test=="short") nvsSlots[0].resize(30);
  if(test=="read_error") nvsReadOK=false;
  if(test=="open_error") nvsOpenOK=false;
  // A valid newer configuration mismatch must not fall back to old matching slot.
  if(test=="newer_config") {
    auto newer=rec;newer.config++;newer.generation++;newer.checksum=wheelcal::crc(newer);
    const auto* b=reinterpret_cast<const uint8_t*>(&newer);
    nvsSlots[1].assign(b,b+sizeof(newer));
  }
  const auto before=nvsSlots;
  bool speedFirst=test=="speed_fixed_speed";
  if(speedFirst) { input("motor both +\n"); input("stop\n"); }
  if(test=="timer_create") timerReady=false;
  if(test=="sensor_start") rightSensor.fault=true;
  if(test=="driver") driverInitOK=false;
  if(test=="motor") motorInitOK=false;
  unsigned count=0;
  duringFOC=[&]{
    ++count;
    assert(rightMotor.controller==MotionControlType::torque);
    if(count==1) {assert(!pins[32]);assert(rightMotor.sensor_direction==CCW);assert(rightMotor.zero_electric_angle==rec.wheels[0].zero);}
    if(count==2) {assert(!pins[27]);assert(leftMotor.sensor_direction==CCW);assert(leftMotor.zero_electric_angle==rec.wheels[1].zero);}
    if(test=="first_foc"||(test=="second_foc"&&count==2)) focInitOK=false;
    if(test=="changed_foc") rightMotor.zero_electric_angle=0;
    if(test=="init_sensor"&&count==2) leftBus.nack=2;
    if(test=="init_timeout") delay(kAlignMs);
    if(test=="init_input") Serial.feed("stop\n");
    if(test=="init_partial") Serial.feed("voltage left");
    if(test=="timer_start"&&count==2) startTimerOK=false;
  };
  const bool negative=test=="left_minus"||test=="right_minus";
  const bool isRight=test=="right_plus"||test=="right_minus";
  duringEnable=[&]{if(test=="enable_timeout"&&fixedActive) delay(500);};
  if(test=="clock_wrap") nowMs=UINT32_MAX-2200;
  input(std::string("voltage ")+(isRight?"right ":"left ")+(negative?"-\n":"+\n"));
  assert(nvsSlots==before);
  if(!running) {
    assert(test!="left_plus"&&test!="left_minus"&&test!="right_plus"&&test!="right_minus"&&
        test!="stop"&&test!="left_sensor"&&test!="right_sensor"&&test!="nonfinite"&&
        test!="independent"&&test!="reject"&&test!="invalid"&&test!="clock_wrap"&&
        test!="fixed_speed"&&test!="speed_fixed_speed"&&test!="queued"&&test!="partial_end");
    disabled(); assert(!fixedActive); assert(rightMotor.moveCalls==(speedFirst?1:0));
    if(count==0 && test!="driver"&&test!="motor"&&test!="timer_create"&&test!="sensor_start"&&test!="enable_timeout")
      assert(rightDriver.initCalls==0&&leftDriver.initCalls==0);
    if(test=="init_partial") {input(" +\n");disabled();}
    drain(); std::cout<<"PASS fixed refusal "<<test<<"\n";return 0;
  }
  assert(runningBoth&&fixedActive&&pins[27]&&pins[32]);
  assert((driverInitEvents==std::vector<int>{27,32}));
  assert(fakeTimer.duration==500);
  for(auto* w:{&right,&left}) {
    assert(w->savedMode&&w->motor.controller==MotionControlType::torque);
    assert(w->motor.torque_controller==TorqueControlType::voltage);
    assert(w->motor.foc_modulation==FOCModulationType::SinePWM&&w->motor.modulation_centered==1);
    assert(w->motor.voltage_limit==1);
    assert(w->motor.LPF_velocity.Tf==DEF_VEL_FILTER_Tf);
  }
  assert(fixedSamples[isRight?0:1].command==(negative?-.5f:.5f));
  assert(fixedSamples[isRight?1:0].command==0);
  leftBus.raw=100;rightBus.raw=200;
  loop();
  assert(fixedSamples[0].end<fixedSamples[0].start&&fixedSamples[1].end<fixedSamples[1].start);
  const auto outputAtStart=Serial.output;
  if(test=="reject") input("motor both -\nvoltage right -\nstatus\n");
  if(test=="invalid") input("voltage left ++\nvoltage right +x\n");
  assert(Serial.output==outputAtStart); // no output during fixed pulse
  if(test=="queued") {
    Serial.feed(std::string(200,'x')+"\nvoltage right -\n");
    nowMs=runStarted+500;loop();
    while(Serial.available()) loop();
  }
  else if(test=="partial_end") {
    input("voltage right");
    nowMs=runStarted+500;loop();input(" -\n");
  }
  else if(test=="stop") input("stop\n");
  else if(test=="left_sensor") {leftBus.nack=2;loop();}
  else if(test=="right_sensor") {rightBus.count=1;loop();}
  else if(test=="nonfinite") {leftMotor.voltage.q=NAN;loop();}
  else if(test=="independent") {delay(500);disabled();assert(running);loop();}
  else {
    nowMs=runStarted+499;loop();assert(running&&fixedActive);
    nowMs=runStarted+500;loop();
  }
  disabled();assert(!running&&!fixedActive);drain();
  assert(Serial.output.find("FIXED RESULT")!=std::string::npos);
  assert(Serial.output.find("EN_after=0/0")!=std::string::npos);
  const auto output=Serial.output; loop();drain();assert(Serial.output==output);
  if(test=="reject") assert(fixedRejected==3);
  assert(nvsSlots==before);
  if(test=="fixed_speed"||speedFirst) {
    duringFOC=nullptr;
    input("motor both -\n");assert(runningBoth&&!fixedActive);
    assert(leftMotor.controller==MotionControlType::velocity&&rightMotor.controller==MotionControlType::velocity);
    assert(leftMotor.target==-6&&rightMotor.target==-6);
    assert(leftMotor.sensor_direction==CW&&rightMotor.sensor_direction==CW);
    input("stop\n");disabled();
  }
  std::cout<<"PASS fixed "<<test<<"\n";
}
