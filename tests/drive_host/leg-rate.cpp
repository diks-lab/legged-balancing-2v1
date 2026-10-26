  // Actual PS4 pipeline at different control periods, with unchanged UART cadence.
  {
    static_assert(leg::HeightRate==12&&leg::SendIntervalMs==50&&leg::ServoSpeed==150&&leg::ServoAcceleration==15&&!leg::RollDirectionConfirmed,"height rate only; preserve bus/servo/roll settings");
    const auto old=leg::activeGeometry();leg::activeGeometry()=leg::ConfirmedCandidate;
    uint32_t period=1;
    auto cycle=[&](){
      nowMs+=period;noteImuMeasurement();updatePs4Controller();const auto sent=Serial2.goals;serviceLegControl();
      if(Serial2.goals!=sent){
        assert(Serial2.last_size==24);
        const int r=leg::decodePosition(uint16_t(Serial2.last[9])|(uint16_t(Serial2.last[10])<<8));
        const int l=leg::decodePosition(uint16_t(Serial2.last[17])|(uint16_t(Serial2.last[18])<<8));
        assert(r>=1697&&r<=2044&&l>=2084&&l<=2408);
        if(!leg_control.state.recovery)assert(r>=1940&&r<=2034&&l>=2094&&l<=2181);
      }
    };
    auto send=[&](const Ps4InputState&p){PS4.input=p;PS4.input_cb();cycle();};
    auto wait=[&](unsigned ms){for(unsigned i=0;i<ms/period;++i)cycle();};
    Ps4InputState center{},up{},down{},half{},cross{},share{};
    up.left_stick_y=127;down.left_stick_y=-128;half.left_stick_y=72;cross.cross=true;share.share=true;
    auto fixture=[&](int r,int l){
      period=1;setupCase();resetPs4MotionInput();
      ps4_event_inbox.drain(ps4_event_batch);ps4_controls_barrier_valid=false;ps4_event_rearm_required=false;
      ps4_was_connected=false;PS4.ok=true;setupPs4Controller();PS4.connect_cb();cycle();
      send(center);send(cross);send(center);send(share);send(center);
      Serial2.mode=5;Serial2.measuredRight=r;Serial2.measuredLeft=l;
      send(up);wait(350);send(center);wait(100);assert(leg_control.state.active);
    };
    for(unsigned dt: {1u,2u,5u,10u,20u,50u})for(const auto&input:{up,down,half}){
      fixture(1992,2133);period=dt;
      const float initial=leg_control.state.height;float low,high;leg::movementBounds(leg_control.state.right_percent,leg_control.state.left_percent,low,high,false);
      const auto before=nowMs;send(input);wait(1000-dt);
      assert(nowMs-before==1000);
      const float target=leg::clamp(leg::heightTarget(leg::heightInput(ps4_control::legAxis(input.left_stick_y))),initial+low,initial+high);
      assert(fabsf(leg_control.state.height-initial-leg::clamp(target-initial,-12.f,12.f))<.01f);
      wait(2000);assert(fabsf(leg_control.state.height-target)<.01f); // held input is a position
      send(center);wait(2000);const auto r=leg_control.state.right_command,l=leg_control.state.left_command;
      assert(fabsf(leg_control.state.height-leg::homeHeight())<.001f);
      assert(r==leg_control.target(true)&&l==leg_control.target(false));
      wait(500);assert(leg_control.state.right_command==r&&leg_control.state.left_command==l);
      assert(leg::inRange(true,r)&&leg::inRange(false,l));
      assert(!lqr_direction_test_armed&&!lqr_auto_drive_active);
    }
    // Full physical endpoint stays EXACTLY the acquired ticks until new inward input.
    fixture(1697,2408);assert(leg_control.state.right_command==1697&&leg_control.state.left_command==2408);
    const auto preload=Serial2.goals;send(up);wait(500);
    assert(Serial2.goals==preload&&leg_control.state.reason==leg::Reason::RecoveryDirection);
    assert(leg::recoveryHint(leg_control.state)==1);
    publishDiagnosticSnapshot(true);char profile[2048];assert(diagnostic::wheelReadMessage(profile,sizeof(profile),diagnostic_latest,"rate"));
    assert(strstr(profile,"\"leg_rate\":12")&&strstr(profile,"\"leg_hint\":1"));
    send(center);send(down);wait(1000);
    assert(fabsf(leg_control.state.right_percent-88)<.03f&&fabsf(leg_control.state.left_percent-88)<.03f);
    wait(7500); // enter normal range and stop at the paired retracted endpoint
    assert(!leg_control.state.recovery&&leg_control.state.limited);
    const auto atEnd=Serial2.goals;const int r=leg_control.state.right_command,l=leg_control.state.left_command;
    assert(leg::inRange(true,r)&&leg::inRange(false,l));wait(500);
    assert(Serial2.goals==atEnd&&leg_control.state.right_command==r&&leg_control.state.left_command==l);
    leg::activeGeometry()=old;
    std::cout<<"PASS: PS4 -> UART 12 pp/s bounded HOME-position input at 1/2/5/10/20/50 ms; central HOME return, exact preload, return hint and paired endpoint stop\n";
  }
