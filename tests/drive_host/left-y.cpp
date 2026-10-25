  // Physical UP=positive LY, through production callbacks/FIFO -> UART frames.
  {
    const auto previousGeometry=leg::activeGeometry();leg::activeGeometry()=leg::ConfirmedCandidate;
    Ps4InputState center{},up{},down{},diagonal{};
    up.left_stick_y=127;down.left_stick_y=-128;diagonal=up;diagonal.left_stick_x=-128;
    auto cycle=[&](){++nowMs;noteImuMeasurement();updatePs4Controller();serviceLegControl();};
    auto send=[&](const Ps4InputState&p){PS4.input=p;PS4.input_cb();cycle();};
    auto wait=[&](unsigned ms){for(unsigned i=0;i<ms;++i)cycle();};
    auto fixture=[&](int16_t right,int16_t left,bool release=true){
      setupCase();resetPs4MotionInput();
      ps4_event_inbox.drain(ps4_event_batch);ps4_controls_barrier_valid=false;ps4_event_rearm_required=false;
      ps4_was_connected=false;PS4.ok=true;setupPs4Controller();PS4.connect_cb();cycle();
      Ps4InputState stop{},share{};stop.cross=true;share.share=true;
      send(center);send(stop);send(center);send(share);send(center);
      Serial2.mode=5;Serial2.measuredRight=right;Serial2.measuredLeft=left;
      send(up);wait(350);assert(leg_control.state.active);
      if(release){send(center);wait(100);}
    };
    auto verifyFrame=[&](){
      assert(Serial2.last_size==24&&Serial2.last[4]==0x83&&Serial2.last[5]==41);
      const int r=leg::decodePosition(uint16_t(Serial2.last[9])|(uint16_t(Serial2.last[10])<<8));
      const int l=leg::decodePosition(uint16_t(Serial2.last[17])|(uint16_t(Serial2.last[18])<<8));
      assert(r==leg_control.state.right_command&&l==leg_control.state.left_command);
      unsigned sum=0;for(unsigned i=2;i<24;++i)sum+=Serial2.last[i];assert((sum&255)==255);
    };
    fixture(2000,2120);
    auto r=leg_control.state.right_command,l=leg_control.state.left_command;auto goals=Serial2.goals;
    send(up);wait(1000);assert(leg_control.keys==1&&leg_control.state.stick.height==1&&leg_control.state.stick.applied_height==1);
    assert(leg_control.state.operation.confirmed.raw==1&&leg_control.state.operation.operation==uint8_t(leg::Operation::Extend));
    assert(leg_control.state.right_command<r&&leg_control.state.left_command>l&&Serial2.goals>goals);verifyFrame();
    send(center);wait(100);r=leg_control.state.right_command;l=leg_control.state.left_command;goals=Serial2.goals;
    wait(600);assert(Serial2.goals==goals&&leg_control.state.right_command==r&&leg_control.state.left_command==l);
    send(down);wait(1000);assert(leg_control.keys==2&&leg_control.state.stick.height==-1&&leg_control.state.stick.applied_height==-1);
    assert(leg_control.state.operation.confirmed.raw==2&&leg_control.state.operation.operation==uint8_t(leg::Operation::Retract));
    assert(leg_control.state.right_command>r&&leg_control.state.left_command<l);verifyFrame();
    send(center);wait(100);r=leg_control.state.right_command;l=leg_control.state.left_command;
    send(diagonal);wait(500);assert(leg_control.state.right_command<r&&leg_control.state.left_command>l);
    assert(leg_control.state.stick.roll==1&&leg_control.state.stick.applied_roll==0&&leg_control.state.requested_roll==0);verifyFrame();
    assert(!lqr_direction_test_armed&&!lqr_auto_drive_active&&!lqr_drive_start_pending);
    // Extended outside normal: initial exact hold, wrong UP never moves. A DOWN
    // before recenter is also refused; centered new DOWN alone shortens.
    fixture(1697,2408,false);goals=Serial2.goals;
    assert(leg_control.state.recovery==1&&leg_control.state.right_command==1697&&leg_control.state.left_command==2408);
    send(up);wait(300);send(down);wait(300);assert(Serial2.goals==goals&&leg_control.state.recovery==1);
    send(center);send(up);wait(300);assert(Serial2.goals==goals&&leg_control.state.reason==leg::Reason::RecoveryDirection);
    send(center);send(down);wait(1000);
    assert(Serial2.goals>goals&&leg_control.state.right_command>1697&&leg_control.state.left_command<2408);verifyFrame();
    // Retracted margin: outward DOWN refused; centered new UP moves inward.
    fixture(2044,2084,false);goals=Serial2.goals;send(center);send(down);wait(300);
    assert(Serial2.goals==goals&&leg_control.state.right_command==2044&&leg_control.state.left_command==2084);
    send(center);send(up);wait(1000);assert(Serial2.goals>goals);
    assert(leg_control.state.right_command<2044&&leg_control.state.left_command>2084);
    assert(leg::inRange(true,leg_control.state.right_command)&&leg::inRange(false,leg_control.state.left_command));verifyFrame();
    // Disconnect/reconnect cannot replay a held positive LY; new center required.
    fixture(2000,2120);send(up);wait(200);PS4.disconnect_cb();cycle();goals=Serial2.goals;
    PS4.connect_cb();PS4.input=up;PS4.input_cb();cycle();wait(400);
    assert(Serial2.goals==goals&&!leg_control.want_sync&&leg_control.state.stick.applied_height==0&&!leg_control.state.operation.allowed); // Existing disconnect may retain active position hold.
    send(center);send(up);wait(400);assert(leg_control.state.active&&leg_control.state.stick.applied_height==1);verifyFrame();
    leg::activeGeometry()=previousGeometry;
    std::cout<<"PASS: physical positive-LY UP/negative-LY DOWN through UART; normal tick signs, both outer margins, exact hold/recenter, diagonal X disabled, disconnect/reconnect held-input barrier\n";
  }
