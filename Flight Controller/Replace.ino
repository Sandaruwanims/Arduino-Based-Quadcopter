  /*while(true){
    for(start = 0; start < 200; start++){
      ypr_read();
      pitch += ypr_angles[1];
      roll += ypr_angles[2];
    }
    pitch_average = pitch/200;
    roll_average = roll/200;
    
    if(((abs(previous_pitch_average)-abs(pitch_average)) < 0.01) && ((abs(previous_roll_average)-abs(roll_average)) < 0.01)){
      break;
      return true;
    }

    previous_pitch_average = pitch_average;
    previous_roll_average = roll_average;
  }*/
