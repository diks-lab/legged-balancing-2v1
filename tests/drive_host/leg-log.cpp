  // Same PS4/UART/Web trace with logging off/on and a blocked USB sink.
  {
    const auto geometry=leg::activeGeometry();leg::activeGeometry()=leg::ConfirmedCandidate;
    auto trial=[&](bool details,bool blocked){
      setupCase();resetPs4MotionInput();leg_serial_records=leg::SerialDiagnostics{};leg_serial_output=leg::SerialOutput{};
      Serial.room=blocked?0:128;ps4_diagnostic_serial_offset=ps4_diagnostic_serial_size;release_memory_serial_active=false;
      diagnostic_imu_initialized.store(true);diagnostic_imu_io_ok=diagnostic_imu_io_seen=true;
      ps4_event_inbox.drain(ps4_event_batch);ps4_controls_barrier_valid=false;ps4_event_rearm_required=false;
      ps4_was_connected=false;PS4.ok=true;setupPs4Controller();
      const auto firstReads=Serial2.reads;std::vector<int>trace;
      auto cycle=[&](){++nowMs;noteImuMeasurement();updatePs4Controller();auto goals=Serial2.goals;serviceLegControl();publishDiagnosticSnapshot(true);serviceLegSerial();
        if(goals!=Serial2.goals){trace.push_back(leg_control.state.right_command);trace.push_back(leg_control.state.left_command);}
        assert(diagnostic_latest.legs.right_command==leg_control.state.right_command);
      };
      auto send=[&](const Ps4InputState&p){PS4.input=p;PS4.input_cb();cycle();};
      auto wait=[&](unsigned ms){while(ms--)cycle();};
      auto command=[&](const std::string&line){leg::LegCalSerialParser parser;for(char c:line){char a=0;assert(parser.feed(c,a)||c=='\n');if(a)handleLegSerialAction(a);}};
      if(details)command(":leg log on\r\n");else command(":leg log off\n");
      PS4.connect_cb();cycle();Ps4InputState center{},cross{},share{},down{};cross.cross=true;share.share=true;down.left_stick_y=-72;
      send(center);send(cross);send(center);send(share);send(center);
      Serial2.mode=5;Serial2.measuredRight=1992;Serial2.measuredLeft=2133;
      send(down);wait(350);send(center);wait(100);send(down);wait(500);
      command(":leg status\r\n");cycle();send(center);wait(1300);
      command(":leg log off\r\n");cycle();wait(800);
      assert(leg_control.state.active&&leg_control.state.following.side[0].sample_ms&&leg_control.state.following.side[1].sample_ms);
      trace.push_back(int(Serial2.reads-firstReads));trace.push_back(leg_control.state.tx.goals);
      if(!blocked){assert(Serial.output.find("STATUS ACCEPTED")!=std::string::npos);assert(Serial.output.find("LEG STATUS READ")!=std::string::npos);}
      Serial.room=128;return trace;
    };
    const auto quiet=trial(false,false),detail=trial(true,false),blocked=trial(false,true);
    assert(quiet==detail&&quiet==blocked);
    leg::activeGeometry()=geometry;
    std::cout<<"PASS: actual colon parser/action ACKs, LF/CRLF, PS4 -> UART/feedback/Web invariant under quiet/detail/status/blocked USB\n";
  }
