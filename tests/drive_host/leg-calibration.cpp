  // Exercise the actual STOP, PS4, calibration command, UART, HTTP NVS worker
  // and control-owner adoption paths together. Saving alone cannot drive.
  bootFixture();serviceBootAutoStart();step();
  executeOperatorCommand(OperatorCommand::FullStop);
  for(unsigned i=0;i<60;++i){++nowMs;noteImuMeasurement();serviceLegControl();}
  assert(leg_control.state.torque==2&&emergency_stop_active);
  const auto oldGeometry=leg::activeGeometry();
  const auto goalsBeforeCalibration=Serial2.goals,onsBeforeCalibration=Serial2.ons;
  char legHttpAction='1';xQueueSend(http_command_queue,&legHttpAction,0);serviceHttpRequests();
  assert(leg_control.state.calibration.phase==1);
  assert(!executeOperatorCommand(OperatorCommand::EnableDrivers));
  assert(!executeOperatorCommand(OperatorCommand::Start));
  assert(!requestLqrDrive(.01f,'>'));
  assert(!startBalanceTest(false));
  for(unsigned i=0;i<100;++i){
    Ps4InputState held{};held.left_stick_y=-128;held.right_stick_y=-128;
    held.share=true;held.triangle=true;held.dpad_up=true;
    ++nowMs;noteImuMeasurement();processPs4Input(held,nowMs);serviceLegControl();
    assert(!lqr_direction_test_armed&&!leg_control.state.active&&emergency_stop_active);
  }
  assert(Serial2.goals==goalsBeforeCalibration&&Serial2.ons==onsBeforeCalibration);
  // All four recorded positions come through the production ServoReply parser.
  Serial2.mode=5;
  for(unsigned point=0;point<4;++point){
    Serial2.measuredRight=point==1?1697:2044;Serial2.measuredLeft=point==3?2408:2084;
    legHttpAction=char('3'+point);xQueueSend(http_command_queue,&legHttpAction,0);serviceHttpRequests();
    for(unsigned i=0;i<80;++i){++nowMs;noteImuMeasurement();serviceLegControl();}
    assert(leg_control.state.calibration.phase==1&&(leg_control.state.calibration.mask&(1u<<point)));
  }
  assert(leg_control.state.calibration.canSave()==leg::CalResult::Ok);
  assert(!memcmp(&leg::activeGeometry(),&oldGeometry,sizeof(oldGeometry)));
  legCalibrationCommand('h');assert(leg_control.state.calibration.phase==5);
  assert(!memcmp(&leg::activeGeometry(),&oldGeometry,sizeof(oldGeometry)));
  legCalibrationCommand('a');legCalibrationCommand('b');
  http_task_ready.store(true);Preferences::failWrite=true;legCalibrationCommand('g');assert(leg_control.state.calibration.phase==3);
  serviceLegSaveWorker();serviceLegCalibrationResult();Preferences::failWrite=false;
  assert(leg_control.state.calibration.phase==1&&leg_control.state.calibration.result==leg::CalResult::StorageWrite);
  assert(!memcmp(&leg::activeGeometry(),&oldGeometry,sizeof(oldGeometry)));
  legHttpAction='7';xQueueSend(http_command_queue,&legHttpAction,0);serviceHttpRequests();serviceLegSaveWorker();
  assert(!memcmp(&leg::activeGeometry(),&oldGeometry,sizeof(oldGeometry))); // Not worker-owned.
  serviceLegCalibrationResult();
  assert(leg_control.state.calibration.phase==4&&leg_control.state.calibration.source==1);
  assert(leg::zeroTick(true)==2044&&leg::fullTick(true)==1697&&leg::zeroTick(false)==2084&&leg::fullTick(false)==2408);
  assert(!memcmp(&leg_control.state.geometry,&leg::ConfirmedCandidate,sizeof(leg::Geometry)));
  assert(emergency_stop_active&&!lqr_direction_test_armed&&!leg_control.state.active&&!leg_control.want_sync);
  assert(!leg_control.state.command_valid&&!ps4_controls_released);
  assert(Serial2.goals==goalsBeforeCalibration&&Serial2.ons==onsBeforeCalibration);
  // Held left after save/Share cannot skip the release barrier or start sync.
  Ps4InputState heldAfterSave{};heldAfterSave.left_stick_y=-128;heldAfterSave.share=true;
  for(unsigned i=0;i<20;++i){++nowMs;noteImuMeasurement();processPs4Input(heldAfterSave,nowMs);serviceLegControl();}
  assert(!leg_control.want_sync&&!leg_control.state.active&&emergency_stop_active);
  publishDiagnosticSnapshot(true);char calJson[2048];
  assert(diagnostic::legCalibrationMessage(calJson,sizeof(calJson),diagnostic_latest,"saved"));
  assert(strstr(calJson,"\"values\":[2044,1697,2084,2408]")&&strstr(calJson,"\"phase\":4"));
  const char*calOutput=getenv("LEG_CALIBRATION_JSON_OUTPUT");
  if(calOutput){FILE*f=fopen(calOutput,"w");assert(f);fputs(calJson,f);fclose(f);}
  // Boot restore does not command or enable either actuator.
  leg::activeGeometry()={};leg_control=leg::Control{};
  loadLegCalibration();assert(leg::zeroTick(true)==2044&&leg_control.state.calibration.source==1);
  assert(Serial2.goals==goalsBeforeCalibration&&Serial2.ons==onsBeforeCalibration);
  std::cout<<"PASS: production STOP/PS4/read/candidate/abort/NVS worker/save/adoption/boot and held-input barriers; calibration/save emit no position or ON\n";
