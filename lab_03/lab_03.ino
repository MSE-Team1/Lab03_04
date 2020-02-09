/*

  MSE 2202 MSEBot base code for Labs 3 and 4
  Language: Arduino
  Authors: Michael Naish, Eugen Porter, and Kyle Inzunza
  Date: 06-02-2020

  Rev 1 - Initial version
  Rev 2 - Update for MSEduino v. 2
  Rev 3 - Added line follow functionality

*/



#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>
#include "definitions.h"
#include "functions.h"
#include "drive_fun.h"


void setup() {
  Wire.begin();        // Wire library required for I2CEncoder library
  Serial.begin(9600);

  CharliePlexM::setBtn(ci_Charlieplex_LED1, ci_Charlieplex_LED2,
                       ci_Charlieplex_LED3, ci_Charlieplex_LED4, ci_Mode_Button);

  // set up ultrasonic
  pinMode(ci_Ultrasonic_Ping, OUTPUT);
  pinMode(ci_Ultrasonic_Data, INPUT);

  // set up drive motors
  pinMode(ci_Right_Motor, OUTPUT);
  servo_RightMotor.attach(ci_Right_Motor);
  pinMode(ci_Left_Motor, OUTPUT);
  servo_LeftMotor.attach(ci_Left_Motor);

  // set up arm motors
  pinMode(ci_Arm_Motor, OUTPUT);
  servo_ArmMotor.attach(ci_Arm_Motor);
  pinMode(ci_Grip_Motor, OUTPUT);
  servo_GripMotor.attach(ci_Grip_Motor);

  // set up motor enable switch
  pinMode(ci_Motor_Enable_Switch, INPUT);

  // set up encoders. Must be initialized in order that they are chained together,
  // starting with the encoder directly connected to the Arduino. See I2CEncoder docs
  // for more information
  encoder_LeftMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_RightMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_RightMotor.setReversed(true);  // adjust for positive count when moving forward

  // set up line tracking sensors
  pinMode(ci_Right_Line_Tracker, INPUT);
  pinMode(ci_Middle_Line_Tracker, INPUT);
  pinMode(ci_Left_Line_Tracker, INPUT);
  ui_Line_Tracker_Tolerance = ci_Line_Tracker_Tolerance;

  // read saved values from EEPROM
  b_LowByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Left_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Left_Line_Tracker_Light_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Left_Line_Tracker_Light = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Middle_Line_Tracker_Dark_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Middle_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Middle_Line_Tracker_Light_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Middle_Line_Tracker_Light = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Right_Line_Tracker_Dark_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Right_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Right_Line_Tracker_Light_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Right_Line_Tracker_Light = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Left_Motor_Offset_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Motor_Offset_Address_H);
  ui_Left_Motor_Offset = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Right_Motor_Offset_Address_L);
  b_HighByte = EEPROM.read(ci_Right_Motor_Offset_Address_H);
  ui_Right_Motor_Offset = word(b_HighByte, b_LowByte);
}

void loop()
{
  if ((millis() - ul_3_Second_timer) > 3000)
  {
    bt_3_S_Time_Up = true;
  }

  // button-based mode selection
  if (CharliePlexM::ui_Btn)
  {
    if (bt_Do_Once == false)
    {
      bt_Do_Once = true;
      ui_Robot_State_Index++;
      ui_Robot_State_Index = ui_Robot_State_Index & 7;
      ul_3_Second_timer = millis();
      bt_3_S_Time_Up = false;
      bt_Cal_Initialized = false;
      ui_Cal_Cycle = 0;
    }
  }
  else
  {
    bt_Do_Once = LOW;
  }

  // check if drive motors should be powered
  bt_Motors_Enabled = digitalRead(ci_Motor_Enable_Switch);

  // modes
  // 0 = default after power up/reset
  // 1 = Press mode button once to enter. Run robot.
  // 2 = Press mode button twice to enter. Calibrate line tracker light level.
  // 3 = Press mode button three times to enter. Calibrate line tracker dark level.
  // 4 = Press mode button four times to enter. Calibrate motor speeds to drive straight.
  switch (ui_Robot_State_Index)
  {
    case 0:    //Robot stopped
      {
        readLineTrackers();
        Ping();
        servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
        servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
        servo_ArmMotor.write(ci_Arm_Servo_Retracted);
        servo_GripMotor.write(ci_Grip_Motor_Closed);
        encoder_LeftMotor.zero();
        encoder_RightMotor.zero();
        ui_Mode_Indicator_Index = 0;
        break;
      }

    case 1:    //Robot Run after 3 seconds
      {
        if (bt_3_S_Time_Up)
        {
          readLineTrackers();

#ifdef DEBUG_ENCODERS
          l_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
          l_Right_Motor_Position = encoder_RightMotor.getRawPosition();

          Serial.print("Encoders L: ");
          Serial.print(l_Left_Motor_Position);
          Serial.print(", R: ");
          Serial.println(l_Right_Motor_Position);
#endif

          // set motor speeds
          //ui_Left_Motor_Speed = constrain(ui_Motors_Speed + ui_Left_Motor_Offset, 1600, 2100);
          //ui_Right_Motor_Speed = constrain(ui_Motors_Speed + ui_Right_Motor_Offset, 1600, 2100);

          /***************************************************************************************
            Add line tracking code here.
            Adjust motor speed according to information from line tracking sensors and
            possibly encoder counts.
            /*************************************************************************************/


          LineFollowModeSelect();

          //stage select
          switch (ui_Course_Stage) {
            case 0:
              encoder_LeftMotor.zero();
              encoder_RightMotor.zero();
              ui_Course_Stage++;
              break;
            case 1:
              if (RightMotorEncoderDrive(1000) && LeftMotorEncoderDrive(1000)) {
                ui_Course_Stage++;
              }
              break;
            case 2:
              encoder_LeftMotor.zero();
              encoder_RightMotor.zero();
              GoStraightLine();
              break;
            case 3:
              if (RightMotorEncoderDrive(1000) && LeftMotorEncoderDrive(1000)) {
                ui_Course_Stage++;
              }
            case 4:
              encoder_LeftMotor.zero();
              encoder_RightMotor.zero();
              GoStraightLine();
              break;
            case 5:
              if (RightMotorEncoderDrive(1000) && LeftMotorEncoderDrive(1000)) {
                ui_Course_Stage++;
              }
            case 6:
              ui_Left_Motor_Speed = ci_Motor_Stop;
              ui_Right_Motor_Speed = ci_Motor_Stop;
          }


          //END OF LINE TRACKING CODE

#ifdef DEBUG_MOTOR_OFFSET
          Serial.print("Left offset: ");
          Serial.print(ui_Left_Motor_Offset);
          Serial.print(" Right offset: ");
          Serial.println(ui_Right_Motor_Offset);
#endif

          // set motor speeds
          if (ui_Left_Motor_Speed != ui_Motor_Speed_Stop) {
            //ui_Left_Motor_Speed = constrain(ui_Left_Motor_Speed + ui_Left_Motor_Offset, 1600, 2100);

          }
          if (ui_Right_Motor_Speed != ui_Motor_Speed_Stop) {
            //ui_Right_Motor_Speed = constrain(ui_Right_Motor_Speed + ui_Right_Motor_Offset, 1600, 2100);
          }
          if (bt_Motors_Enabled)
          {

            if (ui_Left_Motor_Speed != ci_Left_Motor_Stop) {
              servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed + ui_Left_Motor_Offset);
            }
            else {
              servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
            }

            if (ui_Right_Motor_Speed != ci_Right_Motor_Stop) {
              servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed + ui_Right_Motor_Offset);
            }
            else {
              servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
            }

          }
          else
          {
            servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
            servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
          }
#ifdef DEBUG_MOTORS
          Serial.print("Motors enabled: ");
          Serial.print(bt_Motors_Enabled);
          Serial.print(", Default: ");
          Serial.print(ui_Motors_Speed);
          Serial.print(", Left = ");
          Serial.print(ui_Left_Motor_Speed);
          Serial.print(", Right = ");
          Serial.println(ui_Right_Motor_Speed);
#endif
          ui_Mode_Indicator_Index = 1;
        }
        break;
      }

    case 2:    //Calibrate line tracker light levels after 3 seconds
      {
        if (bt_3_S_Time_Up)
        {
          if (!bt_Cal_Initialized)
          {
            bt_Cal_Initialized = true;
            ui_Left_Line_Tracker_Light = 0;
            ui_Middle_Line_Tracker_Light = 0;
            ui_Right_Line_Tracker_Light = 0;
            ul_Calibration_Time = millis();
            ui_Cal_Count = 0;
          }
          else if ((millis() - ul_Calibration_Time) > ci_Line_Tracker_Calibration_Interval)
          {
            ul_Calibration_Time = millis();
            readLineTrackers();
            ui_Left_Line_Tracker_Light += ui_Left_Line_Tracker_Data;
            ui_Middle_Line_Tracker_Light += ui_Middle_Line_Tracker_Data;
            ui_Right_Line_Tracker_Light += ui_Right_Line_Tracker_Data;
            ui_Cal_Count++;
          }
          if (ui_Cal_Count == ci_Line_Tracker_Cal_Measures)
          {
            ui_Left_Line_Tracker_Light /= ci_Line_Tracker_Cal_Measures;
            ui_Middle_Line_Tracker_Light /= ci_Line_Tracker_Cal_Measures;
            ui_Right_Line_Tracker_Light /= ci_Line_Tracker_Cal_Measures;
#ifdef DEBUG_LINE_TRACKER_CALIBRATION
            Serial.print("Light Levels: Left = ");
            Serial.print(ui_Left_Line_Tracker_Light, DEC);
            Serial.print(", Middle = ");
            Serial.print(ui_Middle_Line_Tracker_Light, DEC);
            Serial.print(", Right = ");
            Serial.println(ui_Right_Line_Tracker_Light, DEC);
#endif
            EEPROM.write(ci_Left_Line_Tracker_Light_Address_L, lowByte(ui_Left_Line_Tracker_Light));
            EEPROM.write(ci_Left_Line_Tracker_Light_Address_H, highByte(ui_Left_Line_Tracker_Light));
            EEPROM.write(ci_Middle_Line_Tracker_Light_Address_L, lowByte(ui_Middle_Line_Tracker_Light));
            EEPROM.write(ci_Middle_Line_Tracker_Light_Address_H, highByte(ui_Middle_Line_Tracker_Light));
            EEPROM.write(ci_Right_Line_Tracker_Light_Address_L, lowByte(ui_Right_Line_Tracker_Light));
            EEPROM.write(ci_Right_Line_Tracker_Light_Address_H, highByte(ui_Right_Line_Tracker_Light));
            ui_Robot_State_Index = 0;    // go back to Mode 0
          }
          ui_Mode_Indicator_Index = 2;
        }
        break;
      }

    case 3:    // Calibrate line tracker dark levels after 3 seconds
      {
        if (bt_3_S_Time_Up)
        {
          if (!bt_Cal_Initialized)
          {
            bt_Cal_Initialized = true;
            ui_Left_Line_Tracker_Dark = 0;
            ui_Middle_Line_Tracker_Dark = 0;
            ui_Right_Line_Tracker_Dark = 0;
            ul_Calibration_Time = millis();
            ui_Cal_Count = 0;
          }
          else if ((millis() - ul_Calibration_Time) > ci_Line_Tracker_Calibration_Interval)
          {
            ul_Calibration_Time = millis();
            readLineTrackers();
            ui_Left_Line_Tracker_Dark += ui_Left_Line_Tracker_Data;
            ui_Middle_Line_Tracker_Dark += ui_Middle_Line_Tracker_Data;
            ui_Right_Line_Tracker_Dark += ui_Right_Line_Tracker_Data;
            ui_Cal_Count++;
          }
          if (ui_Cal_Count == ci_Line_Tracker_Cal_Measures)
          {
            ui_Left_Line_Tracker_Dark /= ci_Line_Tracker_Cal_Measures;
            ui_Middle_Line_Tracker_Dark /= ci_Line_Tracker_Cal_Measures;
            ui_Right_Line_Tracker_Dark /= ci_Line_Tracker_Cal_Measures;
#ifdef DEBUG_LINE_TRACKER_CALIBRATION
            Serial.print("Dark Levels: Left = ");
            Serial.print(ui_Left_Line_Tracker_Dark, DEC);
            Serial.print(", Middle = ");
            Serial.print(ui_Middle_Line_Tracker_Dark, DEC);
            Serial.print(", Right = ");
            Serial.println(ui_Right_Line_Tracker_Dark, DEC);
#endif
            EEPROM.write(ci_Left_Line_Tracker_Dark_Address_L, lowByte(ui_Left_Line_Tracker_Dark));
            EEPROM.write(ci_Left_Line_Tracker_Dark_Address_H, highByte(ui_Left_Line_Tracker_Dark));
            EEPROM.write(ci_Middle_Line_Tracker_Dark_Address_L, lowByte(ui_Middle_Line_Tracker_Dark));
            EEPROM.write(ci_Middle_Line_Tracker_Dark_Address_H, highByte(ui_Middle_Line_Tracker_Dark));
            EEPROM.write(ci_Right_Line_Tracker_Dark_Address_L, lowByte(ui_Right_Line_Tracker_Dark));
            EEPROM.write(ci_Right_Line_Tracker_Dark_Address_H, highByte(ui_Right_Line_Tracker_Dark));
            ui_Robot_State_Index = 0;    // go back to Mode 0
          }
          ui_Mode_Indicator_Index = 3;
        }
        break;
      }

    case 4:    //Calibrate motor straightness after 3 seconds.
      {
        if (bt_3_S_Time_Up)
        {
          if (!bt_Cal_Initialized)
          {
            bt_Cal_Initialized = true;
            encoder_LeftMotor.zero();
            encoder_RightMotor.zero();
            ul_Calibration_Time = millis();
            servo_LeftMotor.writeMicroseconds(ui_Motors_Speed);
            servo_RightMotor.writeMicroseconds(ui_Motors_Speed);
          }
          else if ((millis() - ul_Calibration_Time) > ci_Motor_Calibration_Time)
          {
            servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
            servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
            l_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
            l_Right_Motor_Position = encoder_RightMotor.getRawPosition();
            if (l_Left_Motor_Position > l_Right_Motor_Position)
            {
              // May have to update this if different calibration time is used
              ui_Right_Motor_Offset = 0;
              ui_Left_Motor_Offset = (l_Left_Motor_Position - l_Right_Motor_Position) / 4;
            }
            else
            {
              // May have to update this if different calibration time is used
              ui_Right_Motor_Offset = (l_Right_Motor_Position - l_Left_Motor_Position) / 4;
              ui_Left_Motor_Offset = 0;
            }

#ifdef DEBUG_MOTOR_CALIBRATION
            Serial.print("Motor Offsets: Left = ");
            Serial.print(ui_Left_Motor_Offset);
            Serial.print(", Right = ");
            Serial.println(ui_Right_Motor_Offset);
#endif
            EEPROM.write(ci_Right_Motor_Offset_Address_L, lowByte(ui_Right_Motor_Offset));
            EEPROM.write(ci_Right_Motor_Offset_Address_H, highByte(ui_Right_Motor_Offset));
            EEPROM.write(ci_Left_Motor_Offset_Address_L, lowByte(ui_Left_Motor_Offset));
            EEPROM.write(ci_Left_Motor_Offset_Address_H, highByte(ui_Left_Motor_Offset));

            ui_Robot_State_Index = 0;    // go back to Mode 0
          }
#ifdef DEBUG_MOTOR_CALIBRATION
          Serial.print("Encoders L: ");
          Serial.print(encoder_LeftMotor.getRawPosition());
          Serial.print(", R: ");
          Serial.println(encoder_RightMotor.getRawPosition());
#endif
          ui_Mode_Indicator_Index = 4;
        }
        break;
      }
  }

  if ((millis() - ul_Display_Time) > ci_Display_Time)
  {
    ul_Display_Time = millis();

#ifdef DEBUG_MODE_DISPLAY
    Serial.print("Mode: ");
    Serial.println(ui_Mode_Indicator[ui_Mode_Indicator_Index], DEC);
#endif
    bt_Heartbeat = !bt_Heartbeat;
    CharliePlexM::Write(ci_Heartbeat_LED, bt_Heartbeat);
    digitalWrite(13, bt_Heartbeat);
    Indicator();
  }
}

// set mode indicator LED state
void Indicator()
{
  //display routine, if true turn on led
  CharliePlexM::Write(ci_Indicator_LED, !(ui_Mode_Indicator[ui_Mode_Indicator_Index] &
                                          (iArray[iArrayIndex])));
  iArrayIndex++;
  iArrayIndex = iArrayIndex & 15;
}

// read values from line trackers and update status of line tracker LEDs
void readLineTrackers()
{
  ui_Left_Line_Tracker_Data = analogRead(ci_Left_Line_Tracker);
  ui_Middle_Line_Tracker_Data = analogRead(ci_Middle_Line_Tracker);
  ui_Right_Line_Tracker_Data = analogRead(ci_Right_Line_Tracker);

  if (ui_Left_Line_Tracker_Data < (ui_Left_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))
  {
    CharliePlexM::Write(ci_Left_Line_Tracker_LED, HIGH);
  }
  else
  {
    CharliePlexM::Write(ci_Left_Line_Tracker_LED, LOW);
  }
  if (ui_Middle_Line_Tracker_Data < (ui_Middle_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))
  {
    CharliePlexM::Write(ci_Middle_Line_Tracker_LED, HIGH);
  }
  else
  {
    CharliePlexM::Write(ci_Middle_Line_Tracker_LED, LOW);
  }
  if (ui_Right_Line_Tracker_Data < (ui_Right_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))
  {
    CharliePlexM::Write(ci_Right_Line_Tracker_LED, HIGH);
  }
  else
  {
    CharliePlexM::Write(ci_Right_Line_Tracker_LED, LOW);
  }

#ifdef DEBUG_LINE_TRACKERS
  Serial.print("Trackers: Left = ");
  Serial.print(ui_Left_Line_Tracker_Data, DEC);
  Serial.print(", Middle = ");
  Serial.print(ui_Middle_Line_Tracker_Data, DEC);
  Serial.print(", Right = ");
  Serial.println(ui_Right_Line_Tracker_Data, DEC);
#endif

}

// measure distance to target using ultrasonic sensor
void Ping()
{
  //Ping Ultrasonic
  //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
  digitalWrite(ci_Ultrasonic_Ping, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(ci_Ultrasonic_Ping, LOW);
  //use command pulseIn to listen to Ultrasonic_Data pin to record the
  //time that it takes from when the Pin goes HIGH until it goes LOW
  ul_Echo_Time = pulseIn(ci_Ultrasonic_Data, HIGH, 10000);

  // Print Sensor Readings
#ifdef DEBUG_ULTRASONIC
  Serial.print("Time (microseconds): ");
  Serial.print(ul_Echo_Time, DEC);
  Serial.print(", Inches: ");
  Serial.print(ul_Echo_Time / 148); //divide time by 148 to get distance in inches
  Serial.print(", cm: ");
  Serial.println(ul_Echo_Time / 58); //divide time by 58 to get distance in cm
#endif
}
