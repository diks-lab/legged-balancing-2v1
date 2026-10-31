  // Actual production fixed-comparison helpers/dispatcher and normal move path.
  {
    const bool ownCommandQueue=!http_command_queue,ownStatusQueue=!http_status_queue;
    if(ownCommandQueue)http_command_queue=xQueueCreate(1,sizeof(char));
    if(ownStatusQueue)http_status_queue=xQueueCreate(1,sizeof(HttpStatus));
    auto stopped=[] {
      setupCase();step(0);
      assert(executeOperatorCommand(OperatorCommand::FullStop));
      assert(emergency_stop_active&&!pins[27]&&!pins[32]);
      motor1.zero_electric_angle=1.4419413f;motor2.zero_electric_angle=4.3810487f;
    };
    auto arm=[&] {
      stopped();armPolarityComparison();
      assert(polarity_fixed_voltage_mode&&polarity_test_active&&!polarity_pulse_active);
      assert(!pins[27]&&!pins[32]);
    };
    auto raw=[](float r,float l){float a=99,b=-99;polarityPulseOutput(a,b);near(a,r);near(b,l);return std::make_pair(a,b);};
    // Running balance, merely enabled, unhealthy and absent timer cannot arm.
    setupCase();armPolarityComparison();assert(!polarity_fixed_voltage_mode);
    stopped();polarity_explicit_stop=false;armPolarityComparison();assert(!polarity_fixed_voltage_mode);
    stopped();sensor2.fault=true;armPolarityComparison();assert(!polarity_fixed_voltage_mode);
    stopped();wheelAlignTimer=nullptr;armPolarityComparison();assert(!polarity_fixed_voltage_mode);
    polarity_test_voltage=.27f; // Fixed mode must not adopt or change this legacy value.
    const char commands[]={'5','6','7','8','+','-'};
    const float expected[][2]={{.5f,0},{-.5f,0},{0,.5f},{0,-.5f},{-.5f,.5f},{.5f,-.5f}};
    for(unsigned trial=0;trial<6;++trial){
      arm();Serial.output.clear();
      onSerialOutput=[] {assert(!pins[27]&&!pins[32]);};
      Serial.feed(std::string(1,commands[trial]));handleSerialCommand();
      assert(polarity_pulse_active&&fakeTimer.armed&&fakeTimer.duration==500);
      assert(pins[27]&&pins[32]);
      const uint32_t start=polarity_pulse_start_ms;
      // Poison all excluded controller contributions. Only fixed voltages win.
      test_LQR_u=99;ps4_yaw_output=.1f;right_target_voltage=77;left_target_voltage=-88;
      near(polarity_test_voltage,.27f);
      auto v=raw(expected[trial][0],expected[trial][1]);
      applyWheelOutputs(v.first,v.second);
      near(motor1.target,v.first);near(motor2.current_sp,v.second);
      // Same normal move N -> next FOC cache N+1 contract. Hardware/FOC is
      // doubled here; wheel_output_host separately links the real library.
      delay(10);motor1.voltage.q=motor1.current_sp;motor2.voltage.q=motor2.current_sp;
      motor1.shaft_angle+=.1f;motor2.shaft_angle-=.2f;
      motor1.shaft_velocity=2;motor2.shaft_velocity=-3;
      last_control_update_us=micros();captureFocOutputSnapshot();capturePolaritySample();
      near(polarity_comparison.wheels[0].q,v.first);near(polarity_comparison.wheels[1].q,v.second);
      assert(!finishPolarityPulseIfNeeded(start+499));
      const auto before=Serial.output;
      Serial.feed("J+5u");for(int i=0;i<4;++i)handleSerialCommand();
      beginPolarityPulse('7');assert(Serial.output==before);
      assert(polarity_comparison.rejected==5);raw(v.first,v.second);
      assert(!executeOperatorCommand(OperatorCommand::Start));assert(Serial.output==before);
      // Existing output snapshot is still requestable and receives actual values.
      assert(wheel_output_observation.request(micros()));applyWheelOutputs(v.first,v.second);
      assert(wheel_output_observation.waitingFoc());
      wheel_output_observation.interrupt();
      delay(490); // Independent callback gates BOTH, with no loop invocation.
      assert(!pins[27]&&!pins[32]&&polarity_guard_expired);
      assert(finishPolarityPulseIfNeeded(millis()));
      assert(polarity_comparison.pending&&!polarity_pulse_active&&emergency_stop_active);
      assert(!strcmp(polarity_comparison.reason,"INDEPENDENT_TIMER"));
      assert(motor1.current_sp==0&&motor2.current_sp==0);
      assert(polarity_comparison.motor_enabled==3&&polarity_comparison.driver_en==3&&polarity_comparison.stopped_en==0);
      assert(polarity_comparison.end_us-polarity_comparison.start_us==500000);
      near(polarity_comparison.wheels[0].end_angle-polarity_comparison.wheels[0].start_angle,.1f);
      near(polarity_comparison.wheels[1].velocity,-3);assert(polarity_comparison.samples==1);
      // Buffered requests cannot become a new run after deadline/result.
      Serial.feed("dJ+\n");servicePolarityResult();
      assert(Serial.available()==0&&!polarity_comparison.pending&&!polarity_fixed_voltage_mode);
      const size_t size=Serial.output.size();servicePolarityResult();assert(Serial.output.size()==size);
      assert(Serial.output.find("FIXED RESULT reason=INDEPENDENT_TIMER")!=std::string::npos);
      armPolarityComparison();assert(!polarity_fixed_voltage_mode); // new explicit d required
      assert(!boot_auto_start.pending()&&!fall_auto_recovery.pending);
      near(motor1.zero_electric_angle,1.4419413f);near(motor2.zero_electric_angle,4.3810487f);
      onSerialOutput=nullptr;
    }
    // Main-loop deadline remains a separate guard if callback has not run.
    arm();beginPolarityPulse('+');fakeTimer.armed=false;nowMs+=500;noteImuMeasurement();
    assert(finishPolarityPulseIfNeeded(millis()));assert(!strcmp(polarity_comparison.reason,"LOOP_DEADLINE"));
    assert(!pins[27]&&!pins[32]);servicePolarityResult();
    // Every interruption leaves zero outputs, no auto balance recovery, one result.
    for(unsigned kind=0;kind<8;++kind){
      arm();beginPolarityPulse('+');auto v=raw(-.5f,.5f);applyWheelOutputs(v.first,v.second);
      if(kind==0){Serial.feed("d");handleSerialCommand();}
      if(kind==1){Serial.feed("s");handleSerialCommand();}
      if(kind==2){LQR_angle=polarity_reference_angle+10.01f;assert(finishPolarityPulseIfNeeded(millis()));}
      if(kind==3){I2Cone.nack=2;assert(!acquireRuntimeMeasurements());I2Cone.nack=0;}
      if(kind==4){I2Ctwo.count=1;assert(!acquireRuntimeMeasurements());I2Ctwo.count=2;}
      if(kind==5){diagnostic_imu_io_ok=false;assert(!acquireRuntimeMeasurements());}
      if(kind==6){ps4_was_connected=true;handlePs4Connection(false);}
      if(kind==7){executeOperatorCommand(OperatorCommand::FullStop);}
      assert(emergency_stop_active&&!pins[27]&&!pins[32]);
      assert(!polarity_pulse_active&&!boot_auto_start.pending()&&!fall_auto_recovery.pending);
      assert(polarity_comparison.pending);
      if(kind==3)assert(polarity_comparison.faults&1);
      if(kind==4)assert(polarity_comparison.faults&2);
      if(kind==5)assert(polarity_comparison.faults&4);
      if(kind==6)assert(!strcmp(polarity_comparison.reason,"PS4_DISCONNECT"));
      const auto reason=polarity_comparison.reason;disableBothDrivers();assert(polarity_comparison.reason==reason);
      servicePolarityResult();assert(!polarity_comparison.pending);
    }
    // Timer expiry inside the first enable cannot enable the second bridge.
    arm();std::vector<int> enabled;
    onMotorEnable=[&](int pin){enabled.push_back(pin);delay(500);};
    beginPolarityPulse('+');onMotorEnable=nullptr;
    assert((enabled==std::vector<int>{27}));assert(!pins[27]&&!pins[32]);
    assert(polarity_comparison.pending&&polarity_comparison.first_move_us==0);servicePolarityResult();
    // Arrival-time HTTP gates reject motion/calibration, not capture requests.
    arm();beginPolarityPulse('8');
    const bool queued=http_command_queue->full;
    enqueueHttpCommand('t');assert(wifi_log_server.status==409&&http_command_queue->full==queued);
    handleWheelCalibrationRequest();assert(wifi_log_server.status==409);
    handleWheelOutputCapture();assert(wifi_log_server.status==202);wheel_output_observation.interrupt();
    polarity_comparison.reason="TEST_STOP";stopPolarityPulse();servicePolarityResult();
    // A request queued before arming is rejected when serviced during output.
    arm();beginPolarityPulse('5');
    char queuedCommand=0;while(xQueueReceive(http_command_queue,&queuedCommand,0)==pdTRUE){}
    const char queuedStart='t';assert(xQueueSend(http_command_queue,&queuedStart,0)==pdTRUE);
    serviceHttpRequests();assert(!http_command_queue->full&&!lqr_direction_test_armed);
    assert(polarity_pulse_active);stopPolarityPulse();servicePolarityResult();
    // Actual PS4 inbox disconnect notification also gates the serial-origin test.
    arm();beginPolarityPulse('+');onPs4Disconnect();updatePs4Controller();
    assert(!pins[27]&&!pins[32]&&polarity_comparison.pending);
    assert(!strcmp(polarity_comparison.reason,"PS4_DISCONNECT"));assert(polarity_comparison.faults&16);
    servicePolarityResult();
    // Millisecond and microsecond wrap retain the same 500 ms deadline.
    arm();nowMs=UINT32_MAX-250;noteImuMeasurement();beginPolarityPulse('5');
    delay(500);assert(!pins[27]&&!pins[32]);finishPolarityPulseIfNeeded(millis());
    assert(polarity_comparison.end_us-polarity_comparison.start_us==500000);servicePolarityResult();
    // Failed independent timer never enables. Raw command stays unapplied.
    arm();startTimerOK=false;beginPolarityPulse('5');
    assert(!pins[27]&&!pins[32]&&polarity_comparison.pending);
    assert(!strcmp(polarity_comparison.reason,"TIMER_START_FAILED"));
    assert(polarity_comparison.first_move_us==0&&polarity_comparison.wheels[0].command==0);
    servicePolarityResult();startTimerOK=true;
    polarity_test_voltage=kPolarityTestVoltage;
    if(ownCommandQueue){delete http_command_queue;http_command_queue=nullptr;}
    if(ownStatusQueue){delete http_status_queue;http_status_queue=nullptr;}
    std::cout<<"PASS: production fixed 0.5V/500ms six commands, explicit STOP, shared move/FOC capture, no PID/yaw, timer/loop deadline, manual/tilt/sensors/IMU/disconnect, reject/no replay, one post-gate result and preserved saved zeros\n";
  }
