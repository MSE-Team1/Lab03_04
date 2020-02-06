#ifndef DRIVE_FUNCTIONS_H
#define DRIVE_FUNCTIONS_H

//drives straight with line
void GoStraightLine() {
  switch (ui_Line_Tracker_Mode) {
    case 0:
      ui_Left_Motor_Speed = ui_Motor_Speed_Medium_Forward;
      ui_Right_Motor_Speed = ui_Motor_Speed_Medium_Forward;
      break;
    case 1:
      ui_Right_Motor_Speed = ui_Motor_Speed_Medium_Forward;
      ui_Left_Motor_Speed = ui_Motor_Speed_Slow_Forward;
      break;
    case 2:
      ui_Right_Motor_Speed = ui_Motor_Speed_Medium_Forward;
      ui_Left_Motor_Speed = ui_Motor_Speed_Stop;
      break;
    case 3:
      ui_Right_Motor_Speed = ui_Motor_Speed_Slow_Forward;
      ui_Left_Motor_Speed = ui_Motor_Speed_Medium_Forward;
      break;
    case 4:
      ui_Right_Motor_Speed = ui_Motor_Speed_Stop;
      ui_Left_Motor_Speed = ui_Motor_Speed_Medium_Forward;
      break;
    case 5:
      ui_Right_Motor_Speed = ui_Motor_Speed_Stop;
      ui_Left_Motor_Speed = ui_Motor_Speed_Stop;
      ui_Course_Stage++;
      break;
  }
}

//turns right without a line
void TurnRightNoLine() {
  //if sees no line
  if (ui_Line_Tracker_Mode == 5) {
    //left turn
    ui_Left_Motor_Speed = ui_Motor_Speed_Medium_Forward;
    ui_Right_Motor_Speed = ui_Motor_Speed_Slow_Forward;
  }
  //line has been found
  else {
    ui_Right_Motor_Speed = ui_Motor_Speed_Stop;
    ui_Left_Motor_Speed = ui_Motor_Speed_Stop;
    ui_Course_Stage++;
  }
}

//turns right without a line
void TurnLeftNoLine() {
  //if sees no line
  if (ui_Line_Tracker_Mode == 5) {
    //left turn
    ui_Right_Motor_Speed = ui_Motor_Speed_Medium_Forward;
    ui_Left_Motor_Speed = ui_Motor_Speed_Slow_Forward;
  }
  //line has been found
  else {
    ui_Right_Motor_Speed = ui_Motor_Speed_Stop;
    ui_Left_Motor_Speed = ui_Motor_Speed_Stop;
    ui_Course_Stage++;
  }
}

void FollowCurveSlow(){

  unsigned int ui_Motor_Speed_Adjustment = 500;
  
  switch (ui_Line_Tracker_Mode) {
    case 0:
      ui_Left_Motor_Speed = ui_Motor_Speed_Medium_Forward;
      ui_Right_Motor_Speed = ui_Motor_Speed_Medium_Forward;
      break;
    case 1:
      ui_Right_Motor_Speed = ui_Motor_Speed_Slow_Forward + ui_Motor_Speed_Adjustment;
      ui_Left_Motor_Speed = ui_Motor_Speed_Slow_Forward;
      break;
    case 2:
      ui_Right_Motor_Speed = ui_Motor_Speed_Slow_Forward + ui_Motor_Speed_Adjustment;
      ui_Left_Motor_Speed = ui_Motor_Speed_Stop;
      break;
    case 3:
      ui_Right_Motor_Speed = ui_Motor_Speed_Slow_Forward;
      ui_Left_Motor_Speed = ui_Motor_Speed_Slow_Forward + ui_Motor_Speed_Adjustment;
      break;
    case 4:
      ui_Right_Motor_Speed = ui_Motor_Speed_Stop;
      ui_Left_Motor_Speed = ui_Motor_Speed_Slow_Forward + ui_Motor_Speed_Adjustment;
      break;
    case 5:
      ui_Right_Motor_Speed = ui_Motor_Speed_Stop;
      ui_Left_Motor_Speed = ui_Motor_Speed_Stop;
      ui_Course_Stage++;
      break;
  }
}

//drives a certain amount of rotations
//returns true if the position is reached
//encoder must be zeroed in previous stage
bool LeftMotorEncoderDrive(float f_Rotations){
  bool b_state;
  
  //if position has not been reached yet
  if(encoder_LeftMotor.getPosition() < f_Rotations){
    ui_Left_Motor_Speed = ui_Motor_Speed_Slow_Forward;
    b_state = false;
  }
  else{
    ui_Left_Motor_Speed = ui_Motor_Speed_Stop;
    b_state = true;
  }
  return b_State;
}

//drives a certain amount of rotations
//returns true if the position is reached
//encoder must be zeroed in previous stage
bool RightMotorEncoderDrive(float f_Rotations){
  bool b_state;
  
  //if position has not been reached yet
  if(encoder_RightMotor.getPosition() < f_Rotations){
    ui_Right_Motor_Speed = ui_Motor_Speed_Slow_Forward;
    b_state = false;
  }
  else{
    ui_Right_Motor_Speed = ui_Motor_Speed_Stop;
    b_state = true;
  }
  return b_State;
}

#endif
