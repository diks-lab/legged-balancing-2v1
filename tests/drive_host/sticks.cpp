  // Production callbacks -> FIFO -> new left/right routing, not a hand-wired model.
  {
    auto cycle=[&](){++nowMs;noteImuMeasurement();updatePs4Controller();serviceLegControl();publishDiagnosticSnapshot(true);};
    auto send=[&](const Ps4InputState&packet){PS4.input=packet;PS4.input_cb();cycle();};
    auto wait=[&](unsigned n){for(unsigned i=0;i<n;++i)cycle();};
    auto fixture=[&](bool bench,bool fullAcceptance=false){
      setupCase();resetPs4MotionInput();
      if(fullAcceptance)leg_control.state.acceptance={1675,2007,2071,2412,true};
      ps4_event_inbox.drain(ps4_event_batch);ps4_controls_barrier_valid=false;ps4_event_rearm_required=false;
      ps4_was_connected=false;PS4.ok=true;setupPs4Controller();PS4.connect_cb();cycle();
      Ps4InputState zero{};send(zero);
      if(bench){Ps4InputState stop{},share{};stop.cross=true;share.share=true;send(stop);send(zero);send(share);send(zero);}
      Serial2.mode=1;
    };
    Ps4InputState zero{},up{},down{},left{},diagonal{},forward{},yaw{},reserved{};
    up.left_stick_y=127;down.left_stick_y=-128;left.left_stick_x=-128;
    diagonal=up;diagonal.left_stick_x=-128;forward.right_stick_y=-127;yaw.right_stick_x=127;
    // D-pad has no actuator effect and does not act as a drive start/release condition.
    fixture(false);const auto dReads=Serial2.reads,dGoals=Serial2.goals;
    for(unsigned mask=1;mask<16;++mask){reserved={};reserved.dpad_up=mask&1;reserved.dpad_down=mask&2;reserved.dpad_left=mask&4;reserved.dpad_right=mask&8;
      send(reserved);wait(2);assert(!leg_control.want_sync&&!leg_control.state.active&&!lqr_drive_start_pending&&!lqr_auto_drive_active);}
    assert(Serial2.reads==dReads&&Serial2.goals==dGoals);send(zero);
    reserved.dpad_up=reserved.dpad_down=reserved.dpad_left=reserved.dpad_right=true;
    send(reserved);reserved.right_stick_y=-127;send(reserved);assert(lqr_drive_start_pending);
    // Left X-only is rejected, then diagonal height is accepted without a new X release.
    fixture(true);const auto xReads=Serial2.reads;send(left);wait(250);
    assert(Serial2.reads==xReads&&!leg_control.state.active&&leg_control.state.operation.blocker==uint8_t(leg::Reason::RollUnconfirmed));
    send(diagonal);wait(400);assert(leg_control.state.active&&!lqr_direction_test_armed&&!lqr_drive_start_pending);
    const auto oldHeight=leg_control.state.height;wait(500);assert(leg_control.state.height>oldHeight);
    assert(leg_control.state.stick.applied_height==1&&leg_control.state.stick.applied_roll==0&&leg_control.state.requested_roll==0);
    assert(leg_control.state.operation.allowed&&leg_control.state.operation.operation==uint8_t(leg::Operation::Extend));
    send(zero);wait(2000);near(leg_control.state.height,leg::homeHeight());
    // Curve-position goal: holding does not integrate, and center returns HOME.
    fixture(true);Ps4InputState half{};half.left_stick_y=72;send(half);wait(1500);assert(leg_control.state.active);
    near(leg_control.state.height,leg::heightTarget(leg::heightInput(56.f/111.f)));
    const auto heldHeight=leg_control.state.height;wait(1000);near(leg_control.state.height,heldHeight);
    send(zero);wait(1500);near(leg_control.state.height,leg::homeHeight());
    Ps4InputState tiny{};tiny.left_stick_y=17;send(tiny);wait(1000);
    near(leg_control.state.height,leg::heightTarget(leg::heightInput(1.f/111.f)));
    // Bench right input does not start wheels or interrupt accepted leg operation.
    send(forward);wait(100);send(yaw);wait(50);
    assert(!lqr_direction_test_armed&&!balance_wait_active&&!lqr_drive_start_pending&&leg_control.state.active);
    send(up);wait(300);assert(leg_control.state.active);
    // Simultaneous left/right follows both owners; no new sync or ON on drive end.
    fixture(false);send(up);wait(350);assert(leg_control.state.active&&!lqr_drive_start_pending&&!lqr_auto_drive_active);
    const auto onBeforeDrive=Serial2.ons;
    Ps4InputState simultaneous=up;simultaneous.right_stick_y=-127;simultaneous.right_stick_x=127;send(simultaneous);
    assert(lqr_drive_start_pending&&leg_control.state.active&&ps4_right_input_active&&leg_control.state.mode==3);
    startPendingLqrDrive();wait(300);assert(lqr_auto_drive_active&&ps4_yaw_request>0);
    assert(leg_control.state.stick.drive>0&&leg_control.state.active&&leg_control.state.operation.allowed);
    send(up);assert(lqr_soft_stop_active||ps4_drive_state==Ps4DriveState::Neutral);
    for(unsigned i=0;i<800&&ps4_drive_state!=Ps4DriveState::Neutral;++i){step();nowMs+=50;cycle();}
    assert(!lqr_soft_stop_active&&!lqr_auto_drive_active&&!lqr_drive_start_pending);
    assert(leg_control.state.active&&Serial2.ons==onBeforeDrive);
    send(zero);wait(2000);near(leg_control.state.height,leg::homeHeight());
    // Exceptional wheel recovery still inhibits and requires central re-arm.
    drive_recovery_state=DriveRecoveryState::AttitudeOnly;cycle();assert(!leg_control.state.active);
    drive_recovery_state=DriveRecoveryState::Inactive;send(up);wait(100);assert(!leg_control.state.active);
    send(zero);send(up);wait(300);assert(leg_control.state.active);
    Ps4InputState band{};band.left_stick_y=-15;
    // FOC's fresh-event barrier closes the left gate as well as the right drive gate.
    fixture(true);rearmPs4AfterFoc();send(up);wait(150);
    assert(!leg_control.state.active&&!leg_control.want_sync&&!leg_control.state.operation.allowed);
    send(zero);send(up);wait(350);assert(leg_control.state.active);
    // Error while left held: no repeated sync; only strict center then new intent retries.
    fixture(true);Serial2.mode=3;const auto errRead=Serial2.reads;send(up);wait(400);
    assert(!leg_control.state.active&&Serial2.reads==errRead+2);wait(400);assert(Serial2.reads==errRead+2);
    band.left_stick_y=-16;send(band);send(up);wait(150);assert(Serial2.reads==errRead+2);
    Serial2.mode=1;send(zero);send(up);wait(350);assert(leg_control.state.active);
    // Disconnect/reconnect with deflection never resumes; centered new intent is needed.
    PS4.disconnect_cb();cycle();const auto disconnectGoals=Serial2.goals;
    PS4.connect_cb();PS4.input=up;PS4.input_cb();cycle();wait(300);assert(Serial2.goals==disconnectGoals);
    send(zero);send(up);wait(300);assert(leg_control.state.stick.applied_height==1);
    // Disconnect during a pending first READ must cancel preload/ON entirely.
    fixture(true);Serial2.mode=0;send(up);
    for(unsigned i=0;i<100&&leg_bus.stage(leg_control)!=2;++i)cycle();
    assert(leg_bus.stage(leg_control)==2);const auto pendingOns=Serial2.ons,pendingGoals=Serial2.goals;
    PS4.disconnect_cb();cycle();Serial2.mode=1;PS4.connect_cb();PS4.input=up;PS4.input_cb();cycle();wait(150);
    assert(!leg_control.state.active&&!leg_control.want_sync&&!leg_bus.busy()&&Serial2.ons==pendingOns&&Serial2.goals==pendingGoals);
    send(zero);send(up);wait(350);assert(leg_control.state.active);
    // Full-stop barriers, fall OFF/recovery, and suspended wheel reads all demand new left center.
    Ps4InputState cross{};cross.cross=true;send(cross);send(up);wait(100);
    assert(emergency_stop_active&&!leg_control.state.active&&!leg_control.want_sync);
    send(zero);Ps4InputState share{};share.share=true;send(share);send(zero);send(up);wait(350);assert(leg_control.state.active);
    wheel_comm_suspended=true;cycle();wheel_comm_suspended=false;wait(100);assert(!leg_control.state.active&&!leg_control.want_sync);
    send(zero);send(up);wait(350);assert(leg_control.state.active);
    fixture(false);send(up);wait(350);float ro=0,lo=0;stopForActualFall(ro,lo);cycle();
    LQR_angle=0;noteImuMeasurement();serviceFallAutoRecovery();wait(100);
    assert(lqr_direction_test_armed&&!leg_control.state.active&&!leg_control.want_sync);
    send(zero);send(up);wait(350);assert(leg_control.state.active);
    // Full measured endpoints: exact hold first; outer DZ band cannot count as center.
    fixture(true,true);Serial2.mode=4;
    const auto endpointsGoals=Serial2.goals;send(zero);send(up);wait(350);
    assert(leg_control.state.recovery==1&&Serial2.goals==endpointsGoals+1&&leg_control.state.right_command==1675&&leg_control.state.left_command==2412);
    send(band);send(down);wait(100);assert(leg_control.state.recovery==1&&Serial2.goals==endpointsGoals+1);
    send(zero);send(down);wait(400);assert(leg_control.state.right_command>1675&&leg_control.state.left_command<2412);
    assert(!lqr_direction_test_armed&&!balance_wait_active);
    for(bool conflict:{false,true}){
      fixture(true,true);Serial2.mode=5;Serial2.measuredRight=1675;Serial2.measuredLeft=conflict?2071:2081;
      const auto g=Serial2.goals;send(up);wait(350);assert(Serial2.goals==g+1&&leg_control.state.recovery==1);
      send(zero);send(down);wait(100);
      assert(Serial2.goals==g+1&&!leg_control.state.operation.allowed&&leg_control.state.reason==(conflict?leg::Reason::ReturnConflict:leg::Reason::PairLimit));
    }
    fixture(true);send(down);wait(4000);
    const auto minR=leg_control.state.right_command,minL=leg_control.state.left_command;const auto minGoals=Serial2.goals;
    wait(300);assert(Serial2.goals==minGoals&&leg_control.state.right_command==minR&&leg_control.state.left_command==minL);
    assert(leg::inRange(true,minR)&&leg::inRange(false,minL)&&fabsf(leg_control.state.height-leg::homeLow())<.001f);
    std::cout<<"PASS: actual PS4 sticks/D-pad routing, HOME/curved-position height, drive+leg co-control, bench no drive, errors/disconnect/stop/fall/wheel retry barriers, exact outer hold\n";
  }
