  // Production callbacks: HOME goals and left/right co-control at retained wheel limits.
  {
    const auto geometry=leg::activeGeometry();leg::activeGeometry()=leg::ConfirmedCandidate;
    setupCase();resetPs4MotionInput();diagnostic_imu_initialized.store(true);diagnostic_imu_io_ok=diagnostic_imu_io_seen=true;
    mpu6050.roll=0;ps4_event_inbox.drain(ps4_event_batch);ps4_controls_barrier_valid=false;ps4_event_rearm_required=false;
    auto cycle=[&](){++nowMs;noteImuMeasurement();updatePs4Controller();serviceLegControl();publishDiagnosticSnapshot(true);
      if(leg_control.state.active&&!leg_control.state.recovery)assert(leg::inRange(true,leg_control.target(true))&&leg::inRange(false,leg_control.target(false)));
    };
    auto send=[&](const Ps4InputState&p){PS4.input=p;PS4.input_cb();cycle();};
    auto wait=[&](unsigned ms){while(ms--)cycle();};
    ps4_was_connected=false;PS4.ok=true;setupPs4Controller();PS4.connect_cb();cycle();Ps4InputState center{},up{},mix{};
    up.left_stick_y=127;send(center);
    Serial2.mode=5;Serial2.measuredRight=1992;Serial2.measuredLeft=2133;
    send(up);for(unsigned i=0;i<350&&!leg_control.state.active;++i)cycle();
    assert(leg_control.state.active&&leg_control.state.right_command==1992&&leg_control.state.left_command==2133);
    send(center);wait(200);assert(leg_control.state.right_command==1992&&leg_control.state.left_command==2133); // sync alone does not start HOME
    // Explicit Y operation, then neutral HOME; direct height does not integrate.
    Ps4InputState half{};half.left_stick_y=72;send(half);wait(2000);
    assert(fabsf(leg_control.state.height-leg::heightTarget(leg::heightInput(56.f/111.f)))<.001f);
    const float h=leg_control.state.height;wait(2000);assert(leg_control.state.height==h);
    send(center);wait(2000);assert(fabsf(leg_control.state.height-leg::homeHeight())<.001f);
    const auto on=Serial2.ons;
    for(int y:{-32,-72,-127}){ // right-speed levels, while Y and X leg inputs change
      mix={};mix.right_stick_y=y;mix.right_stick_x=64;mix.left_stick_y=72;mix.left_stick_x=-72;
      send(mix);if(lqr_drive_start_pending)startPendingLqrDrive();wait(1500);
      assert(lqr_auto_drive_active&&leg_control.state.active&&leg_control.state.mode==3&&leg_control.state.operation.allowed);
      assert(ps4_drive_speed_request>0&&ps4_drive_speed_request<=kPs4DriveMaxSpeed&&ps4_yaw_request>0);
      assert(fabsf(leg_control.state.height-leg::heightTarget(leg::heightInput(56.f/111.f)))<.001f);
      if(leg::RollDirectionConfirmed)assert(fabsf(leg_control.state.requested_roll-.9375f)<.001f);
      assert(diagnostic_latest.legs.mode==3&&diagnostic_latest.legs.command_valid);
      const auto reads=Serial2.reads;wait(1100);assert(Serial2.reads>reads&&Serial2.ons==on);
    }
    mix.left_stick_y=0;mix.left_stick_x=0;send(mix);wait(2000);
    assert(lqr_auto_drive_active&&leg_control.state.active&&fabsf(leg_control.state.height-leg::homeHeight())<.001f&&leg_control.state.requested_roll==0);
    assert(Serial2.ons==on);
    // The old maximum drive speed remains unchanged; this is not a high-speed stability test.
    static_assert(kPs4DriveMaxSpeed==.05f&&leg::ServoSpeed==150&&leg::ServoAcceleration==15&&leg::SendIntervalMs==50);
    Ps4InputState cross{};cross.cross=true;send(cross);assert(emergency_stop_active&&!leg_control.state.active&&!lqr_direction_test_armed);
    leg::activeGeometry()=geometry;
    std::cout<<"PASS: PS4 HOME hold/neutral return and first exact preload; leg+drive/yaw at three right-stick levels, continued async READ/no extra ON, full stop; physical stability untested\n";
  }
