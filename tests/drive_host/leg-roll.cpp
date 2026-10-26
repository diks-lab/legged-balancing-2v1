int main(){
 static_assert(leg::RollDirectionConfirmed&&leg::ImuRollSign==-1,"production measured roll settings");
 leg::activeGeometry()=leg::ConfirmedCandidate;
 auto cycle=[&](){++nowMs;noteImuMeasurement();updatePs4Controller();serviceLegControl();
  if(leg_control.state.active&&!leg_control.state.recovery){assert(leg::inRange(true,leg_control.target(true))&&leg::inRange(false,leg_control.target(false)));}};
 auto send=[&](const Ps4InputState&p){PS4.input=p;PS4.input_cb();cycle();};
 auto wait=[&](unsigned ms){while(ms--)cycle();};
 Ps4InputState center{},left{},right{},up{},cross{},share{};
 left.left_stick_x=-128;right.left_stick_x=127;up.left_stick_y=127;cross.cross=true;share.share=true;
 auto fixture=[&](int r=1992,int l=2133){
  setupCase();diagnostic_imu_initialized.store(true);diagnostic_imu_io_ok=diagnostic_imu_io_seen=true;resetPs4MotionInput();ps4_event_inbox.drain(ps4_event_batch);ps4_controls_barrier_valid=false;ps4_event_rearm_required=false;
  ps4_was_connected=false;PS4.ok=true;setupPs4Controller();PS4.connect_cb();cycle();
  send(center);send(cross);send(center);send(share);send(center);
  Serial2.mode=5;Serial2.measuredRight=r;Serial2.measuredLeft=l;
  mpu6050.roll=-9.1f;send(up);for(unsigned i=0;i<350&&!leg_control.state.active;++i)cycle();assert(leg_control.state.active);
  assert(leg_control.state.right_command==r&&leg_control.state.left_command==l); // no correction during initial preload
  send(center);wait(100);assert(leg_control.state.right_command==r&&leg_control.state.left_command==l);
 };
 auto frame=[&](){assert(Serial2.last_size==24);
  assert(leg::decodePosition(uint16_t(Serial2.last[9])|(uint16_t(Serial2.last[10])<<8))==leg_control.state.right_command);
  assert(leg::decodePosition(uint16_t(Serial2.last[17])|(uint16_t(Serial2.last[18])<<8))==leg_control.state.left_command);};
 fixture();mpu6050.roll=0;send(left);wait(1000);
 assert(leg_control.state.requested_roll>1.99f&&leg_control.state.requested_roll<2.01f);
 assert(leg_control.state.right_command<1992&&leg_control.state.left_command<2133);frame();
 send(center);wait(1600);assert(leg_control.state.requested_roll==0);
 // Measured negative AngleX is left lean: correction retracts right/extends left.
 auto r=leg_control.state.right_command,l=leg_control.state.left_command;
 mpu6050.roll=-9.1f;wait(1000);assert(leg_control.state.measured_roll==9.1f);
 assert(leg_control.state.right_command>r&&leg_control.state.left_command>l);frame();
 r=leg_control.state.right_command;l=leg_control.state.left_command;mpu6050.roll=9.1f;wait(1000);
 assert(leg_control.state.right_command<r&&leg_control.state.left_command<l);frame();
 fixture();mpu6050.roll=0;send(right);wait(1000);assert(leg_control.state.requested_roll< -1.99f);
 assert(leg_control.state.right_command>1992&&leg_control.state.left_command>2133);frame();
 // Diagonal height+roll and end limits never leave normal range or alter height for tilt.
 Ps4InputState diagonal=left;diagonal.left_stick_y=127;send(diagonal);wait(2500);
 const float height=leg_control.state.height;send(left);wait(2000);assert(leg_control.state.height==height&&leg_control.state.limited);
 frame();
 for(const auto&pair:{std::pair<int,int>{1940,2181},{2034,2094}}){
  fixture(pair.first,pair.second);mpu6050.roll=0;const float h=leg_control.state.height;
  send(left);wait(1800);assert(leg_control.state.height==h&&leg_control.state.limited);
  assert(leg::inRange(true,leg_control.state.right_command)&&leg::inRange(false,leg_control.state.left_command));
 }
 fixture(1697,2408);const auto goals=Serial2.goals;send(left);wait(1000);
 assert(Serial2.goals==goals&&leg_control.state.requested_roll==0&&leg_control.state.applied_roll==0);
 send(center);Ps4InputState down{};down.left_stick_y=-128;send(down);wait(6200);
 assert(!leg_control.state.recovery&&leg_control.state.requested_roll==0);send(center);wait(100);
 r=leg_control.state.right_command;l=leg_control.state.left_command;wait(100);assert(r==leg_control.state.right_command&&l==leg_control.state.left_command);
 // Reconnect cannot replay the held left target.
 fixture();mpu6050.roll=0;send(left);wait(500);PS4.disconnect_cb();cycle();
 assert(leg_control.state.requested_roll==0&&leg_control.state.applied_roll==0);
 const auto disconnected=Serial2.goals;PS4.connect_cb();PS4.input=left;PS4.input_cb();cycle();wait(500);
 assert(Serial2.goals==disconnected&&leg_control.state.requested_roll==0);
 send(center);send(left);wait(400);assert(leg_control.state.requested_roll>0);
 // Same coherent snapshot carries raw/sign/target diagnostics without extra reads.
 publishDiagnosticSnapshot(true);char json[2048];assert(diagnostic::wheelReadMessage(json,sizeof(json),diagnostic_latest,"roll"));
 assert(strstr(json,"\"leg_roll_confirmed\":true")&&strstr(json,"\"leg_roll_sign\":-1"));
 leg::SerialDiagnostics records;leg::SerialOutput output;records.capture(leg_control.state,leg_bus.stage(leg_control),nowMs);
 Serial.output.clear();for(unsigned i=0;i<300;++i){
  if(output.idle()){leg::SerialDiagnostics::Record rec;uint32_t lost=0;if(records.pop(rec,lost))output.begin(rec,lost);}
  output.service(Serial);
 }
 assert(Serial.output.find("LEG ROLL: AngleX=")!=std::string::npos&&Serial.output.find("target_requested=")!=std::string::npos);
 // Stop/fall transition clears targets; new sync requires release and fresh input.
 leg_control.stop(leg::Reason::Stopped);assert(leg_control.state.requested_roll==0&&!leg_control.state.active);
 std::cout<<"PASS: production roll flags; PS4 FIFO -> control -> UART, measured AngleX signs, target/feedback direction, center ramp, diagonal/end bounds, exact preload, outside return, reconnect barriers\n";
}
