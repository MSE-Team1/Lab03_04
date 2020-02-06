#ifndef LAB3_FUNCTIONS_H
#define LAB3_FUNCTIONS_H

//if the line tracker is seeing white
// left(0), middle(2), right(3)
bool SeesWhite(unsigned int ui_Line_Tracker) {
  unsigned int ui_Dark_Calibration_Value[] = {ui_Left_Line_Tracker_Dark, ui_Middle_Line_Tracker_Dark, ui_Right_Line_Tracker_Dark};
  unsigned int ui_Light_Calibration_Value[] = {ui_Left_Line_Tracker_Light, ui_Middle_Line_Tracker_Light, ui_Right_Line_Tracker_Light};
  unsigned int ui_Line_Tracker_Data[] = {ui_Left_Line_Tracker_Data, ui_Middle_Line_Tracker_Data, ui_Right_Line_Tracker_Data};

  bool result = false;

  if (ui_Line_Tracker_Data[ui_Line_Tracker] < (ui_Dark_Calibration_Value[ui_Line_Tracker] - ui_Line_Tracker_Tolerance))
    result = true;

  return result;
}

//determine line follow mode
void LineFollowModeSelect() {

  //LINE TRACKER MODES
  // 0 : go straight with compensation
  // 1 : go left slow
  // 2 : go left fast
  // 3 : go right slow
  // 4 : go right fast
  // 5 : stop

  //0,1,0
  if (!SeesWhite(0) && SeesWhite(1) && !SeesWhite(2)) {
    ui_Line_Tracker_Mode = 0;
  }
  //1,1,0
  else if (SeesWhite(0) && SeesWhite(1) && !SeesWhite(2)) {
    ui_Line_Tracker_Mode = 1;
  }
  //1,0,0
  else if (SeesWhite(0) && !SeesWhite(1) && !SeesWhite(2)) {
    ui_Line_Tracker_Mode = 2;
  }
  //0,1,1
  else if (!SeesWhite(0) && SeesWhite(1) && SeesWhite(2)) {
    ui_Line_Tracker_Mode = 3;
  }
  //0,0,1
  else if (!SeesWhite(0) && !SeesWhite(1) && SeesWhite(2)) {
    ui_Line_Tracker_Mode = 4;
  }
  //0,0,0
  else if (!(SeesWhite(0) || SeesWhite(1) || SeesWhite(3))) {
    ui_Line_Tracker_Mode = 5;
  }
  //1,1,1
  else if (SeesWhite(0) && SeesWhite(1) && SeesWhite(2)){
    //do nothing
  }
  else {
#ifdef DEBUG_LINE_FOLLOW
    Serial.println("NO CASE SATISFIED.");
#endif
  }

#ifdef DEBUG_LINE_FOLLOW
  Serial.print("LINE FOLLOW MODE: ");
  Serial.print(ui_Line_Tracker_Mode);
  Serial.print(", COURSE MODE:");
  Serial.println(ui_Course_Stage);
#endif
}

#endif
