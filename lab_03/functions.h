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


  /*
    //unsigned int ui_Threshold[] = (ui_Dark_Calibration_Value[ui_Line_Tracker] + ui_Light_Calibration_Value[ui_Line_Tracker]) / 2; //threshold between light and dark
    bool b_Result; //if the line tracker sees white

    if (ui_Tracker_Data[ui_Line_Tracker] < (ui_Dark_Calibration_Value[ui_Line_Tracker] - ui_Line_Tracker_Tolerance)) {
      b_Result = true;
    }
    else {
      b_Result = false;
    }
  */
  return true;
}
