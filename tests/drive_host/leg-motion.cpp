  // Reported hardware case, through actual callbacks/FIFO/controller/UART.
  {
    const auto previousGeometry=leg::activeGeometry();leg::activeGeometry()=leg::ConfirmedCandidate;
    setupCase();resetPs4MotionInput();
    leg_serial_records=leg::SerialDiagnostics{true};leg_serial_output=leg::SerialOutput{};
    ps4_diagnostic_serial_offset=ps4_diagnostic_serial_size;release_memory_serial_active=false;
    ps4_event_inbox.drain(ps4_event_batch);ps4_controls_barrier_valid=false;ps4_event_rearm_required=false;
    auto cycle=[&](){++nowMs;noteImuMeasurement();updatePs4Controller();serviceLegControl();serviceLegSerial();};
    auto send=[&](const Ps4InputState&packet){PS4.input=packet;PS4.input_cb();cycle();};
    auto wait=[&](unsigned ms){for(unsigned i=0;i<ms;++i)cycle();};
    ps4_was_connected=false;PS4.ok=true;setupPs4Controller();PS4.connect_cb();cycle();
    Ps4InputState center{},stop{},share{},down{};stop.cross=true;share.share=true;down.left_stick_y=-128;
    send(center);send(stop);send(center);send(share);send(center);
    Serial2.mode=5;Serial2.measuredRight=1697;Serial2.measuredLeft=2408;
    const auto beforeGoals=Serial2.goals,beforeOns=Serial2.ons;
    send(down);wait(350);
    assert(leg_control.state.recovery==1&&leg_control.state.right_command==1697&&leg_control.state.left_command==2408);
    assert(Serial2.goals==beforeGoals+1&&Serial2.ons==beforeOns+1);
    assert(leg_control.state.tx.goals==1&&leg_control.state.tx.on_sent==1);
    send(center);wait(60);assert(leg_control.state.recovery==2);
    const auto start=nowMs;send(down);wait(2999);
    assert(nowMs-start==3000&&leg_control.state.stick.applied_height==-1);
    const int targetR=leg_control.target(true),targetL=leg_control.target(false);
    assert(targetR>=1946&&targetR<=1948&&targetL>=2174&&targetL<=2176);
    assert(leg_control.state.tx.calculated_right==targetR&&leg_control.state.tx.calculated_left==targetL);
    assert(Serial2.goals>beforeGoals+30&&leg_control.state.right_command>1697&&leg_control.state.left_command<2408);
    assert(leg_control.state.tx.goals==Serial2.goals-beforeGoals);
    assert(!leg_control.state.tx.goal_wait&&!leg_control.state.tx.goal_failed);
    send(center);wait(1800);
    assert(leg_control.state.right_command==leg_control.target(true)&&leg_control.state.left_command==leg_control.target(false));
    assert(Serial.output.find("LEG MOTION: START")!=std::string::npos&&Serial.output.find("LEG MOTION: END")!=std::string::npos);
    assert(Serial.output.find("ON_SENT_UNCONFIRMED")!=std::string::npos&&Serial.output.find("goal_sent=")!=std::string::npos);
    assert(Serial.output.find("短縮して通常範囲へ戻す必要あり")!=std::string::npos);
    assert(Serial.output.find("LEG PROFILE: max_height=24 pp/s")!=std::string::npos);
    assert(Serial.output.find("last_RL=")!=std::string::npos&&Serial.output.find("calc_RL=")!=std::string::npos);
    assert(!lqr_direction_test_armed&&!lqr_auto_drive_active);
    leg_serial_records.setDetailed(false);
    std::cout<<"PASS: actual PS4 100% -> release/new DOWN 3s: calculated="<<targetR<<"/"<<targetL
      <<" UART="<<leg_control.state.right_command<<"/"<<leg_control.state.left_command<<" goal_frames="<<leg_control.state.tx.goals<<" ON_sent="<<leg_control.state.tx.on_sent<<" (no servo ACK)\n";
    leg::activeGeometry()=previousGeometry;
  }
