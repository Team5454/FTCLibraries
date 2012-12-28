//////////////////////////////////////////////////////////////////////////////////////////////////////
///
///Program Name: MoveCommands_Protoboard
///Date of Creation: The beginning of time
///Creator: God/CB
///Version#: 1
///
///Editor of Previous Version: CB
///Editor of Current Version : CB
///
///Issues:
///-None
///-
///
///Currently Functioning:
///-Everything
///-
///
///Changes From Previous:
///-added multiplexor code
///-commented out the HTPBsetupIO on lines 574/581
///- commented out D_I_G_I_T_A_L = data; on line 582
///-commented out mode2 = (int)(data & 0b00000111); on line 584
///-commented out the rest of protoboard funtions
///
////////////////////////////////////////////////////////////////////////////////////////////////////
//use this include statement if you are using the regular programming laptop
//use this include statement if you are using the Coleman laptop
//#include "C:\Users\FTC\Dropbox\Programming\Sample Code\drivers\HTPB-driver.h"
//#include "Drivers/HTSMUX-driver.h"
//#include "Drivers/HTMAG-driver.h"
//#include "Drivers/HTIRS2-driver.h"
//#include "HTEOPD-driver.h"
//#include "Drivers/common.h"

#define OUR_CPR 1440 // 2880
#define Wheel_Base 17.0
#define Wheel_Size 3.0
#define PI 3.14159
#define CIRCUMFERENCE_3 9.42477  // 2 * PI * 3 in
#define CIRCUMFERENCE_4 12.5664  // 2 * PI * 4 in

// Sensor Multiplexor
//const tMUXSensor HTMAG = msensor_S3_1;
// int MagnetCal;

typedef struct
{
  long rate;
  long total;
  long dt;
  long q_err;
  long zero;
  long bias;
  long dzmin;
  long dzmax;
} gyro_data;

typedef struct
{
  int rightSwitch;
  int leftSwitch;
  int team;
  int mode;
} proto_data;

#define GyroSensor S4  // Gyro Sensor Port
#define GyroTimer T4   // Timer used by Gyro processing

gyro_data gyro;        // Gyro data variable
long stempA;
long stempB;
long stempC;
// int D_I_G_I_T_A_L = -1;
// int mode2;
// proto_data proto;

// function with no return value
//that stops all motors
void stop_all_motors()
{
  motor[RightMotor] = 0;
  motor[LeftMotor] = 0;
  return;
}

// function to compute the number of clicks based on the
// distance to move for our REM bot
int compute_clicks_per_distance(float distance, float wheel_diameter)
{
  // float wheel_circumference;
  int cpr;

  // wheel_circumference = PI * wheel_diameter;
  // Lego NXT motor encoder is in 1 degree increments
  // so the number of encoder clicks per revolution is
  // 360
  cpr = OUR_CPR * (distance / CIRCUMFERENCE_3);
  return cpr;
}

// function to compute the number of clicks for a
// two wheel turn for our REM bot
// the wheel_base is the distance between the wheels
// instead of the center of rotation being on of the wheels,
// the REM robot will rotate about the center of the wheel_base,
// or 1/2 the wheel base distance
//
// The wheel base is measured from the center of
// the wheel to the center of the other wheel
// Wheel_radius
// Angle to turn
int compute_click_for_two_wheelturn(float wheel_base_distance, float wheel_diameter, float angle)
{
  // float wheel_circumference;
  float radial_distance;
  int cpr;

  // wheel_circumference = PI * wheel_diameter;

  // Using half the wheel_base as our radius, we want to turn angle degrees
  // To compute the distance we need to use the equation
  // perimeter = angle * radius
  //   where angle is in radians, and radius in inches
  //   to convert from degrees to radians use
  //     radians = (2 * Pi) / 360 * degrees
  radial_distance = (PI / 180.0) * angle * (wheel_base_distance / 2.0);

  // Lego NXT motor encoder is in 1 degree increments
  // so the number of encoder clicks per revolution is
  // 360
  cpr = OUR_CPR * (radial_distance / CIRCUMFERENCE_3);

  return cpr;
}

void reset_encoder()
{
  // Initialize the motor encoders
  nMotorEncoder[LeftMotor] = 0;
  nMotorEncoder[RightMotor] = 0;
  //wait1Msec(500);
   wait1Msec(1000);

}

void move_forward(float dist, int tstop, int lspeed, int rspeed)
{
  int clicks_to_move;
  int stval;  // start time value
  int dtval;  // delta time value
//  int encdiff;

  reset_encoder();
  // Move away from the wall
  clicks_to_move = compute_clicks_per_distance(dist, Wheel_Size);

  // Start the timer to make sure we dont overshoot
  // stop the motors if we never get to our
  // distance goal so we don't burn out the motors
  //
  time1[T1] = 0;      // Start the timer at 0 msec
  stval = time1[T1];  // get number of msec
  dtval = 0;          // compute delta time

  motor[LeftMotor] = lspeed;
  motor[RightMotor] = rspeed;
  while ((nMotorEncoder[RightMotor] < clicks_to_move) && (dtval < tstop)) {
    dtval = time1[T1] - stval;
    wait1Msec(1);
  }
  stop_all_motors();
}

void turn_left (float rangle, int tstop, int lspeed, int rspeed)
{
  int clicks_to_move;
  int stval;  // start time value
  int dtval;  // delta time value

  reset_encoder();
  // Now lets do a two wheel turn
  clicks_to_move = compute_click_for_two_wheelturn(Wheel_Base, Wheel_Size, rangle);
  // again remember depending on which way to want to
  // turn, enable the correct motor direction
  // Use that motor as the encoder test
  time1[T1] = 0;      // Start the timer at 0 msec
  stval = time1[T1];  // get number of msec
  dtval = 0;          // compute delta time
  motor[LeftMotor] = -lspeed;
  motor[RightMotor] =  rspeed;
  while ((nMotorEncoder[RightMotor] > -clicks_to_move) && (dtval < tstop)) {
    dtval = time1[T1] - stval;
    wait1Msec(1);
  }
  stop_all_motors();
}



void turngyro_right(float ang, int spd)
{
  gyro.total = 0;

  if ((gyro.total/1000.0) < ang)
  {
    motor[LeftMotor] = spd;
    motor[RightMotor] = -spd;
    while ((gyro.total/1000.0) > -ang)
    {
      nxtDisplayCenteredTextLine(1, "Agyro=%d", gyro.total/1000);
      wait1Msec(1);
    }
    stop_all_motors();
  }
  /*
  if ((gyro.total/1000.0) < -ang)
  {
    motor[LeftMotor] = -spd;
    motor[RightMotor] = spd;
    while ((gyro.total/1000.0) < -ang)
    {
      nxtDisplayCenteredTextLine(1, "Bgyro=%d", gyro.total/1000);
      wait1Msec(1);
    }
    stop_all_motors();
  }
  */
}

void turngyro_left_oneWheel(float ang, int spd)
{
  long startangle;
  long currentangle;
  long endangle;

  startangle = gyro.total / 1000;
  currentangle = startangle;
  endangle = startangle + ang;

  if (currentangle > endangle)
  {
    motor[LeftMotor] = 0;
    motor[RightMotor] = spd;

    while (currentangle > endangle)
    {
      currentangle = gyro.total / 1000;
      wait1Msec(1);
    }
    stop_all_motors();
  }
  if (currentangle < endangle)
  {
    motor[LeftMotor] = 0;
    motor[RightMotor] = -spd;
    while (currentangle < endangle)
    {
      currentangle = gyro.total / 1000;
      wait1Msec(1);
    }
    stop_all_motors();
  }
}

void turn_right (float rangle, int tstop, int lspeed, int rspeed)
{
  int clicks_to_move;
  int stval;  // start time value
  int dtval;  // delta time value

  reset_encoder();
  // Now lets do a two wheel turn
  // Turn is 35 degrees
  clicks_to_move = compute_click_for_two_wheelturn(Wheel_Base, Wheel_Size, rangle);
  // again remember depending on which way to want to
  // turn, enable the correct motor direction
  // Use that motor as the encoder test
  time1[T1] = 0;      // Start the timer at 0 msec
  stval = time1[T1];  // get number of msec
  dtval = 0;          // compute delta time
  motor[LeftMotor] = lspeed;
  motor[RightMotor] = -rspeed;
  while ((nMotorEncoder[LeftMotor] < clicks_to_move) && (dtval < tstop)) {
    dtval = time1[T1] - stval;
    wait1Msec(1);
  }
  stop_all_motors();

}

void turngyro_left(float ang, int spd)
{
 gyro.total = 0;

  nMotorEncoder[RightMotor] = 0;
  nMotorEncoder[LeftMotor] = 0;

  if ((gyro.total/1000.0) > ang)
  {
    motor[LeftMotor] = -spd;
    motor[RightMotor] = spd;
    while ((gyro.total/1000.0) > ang)
    {
      nxtDisplayCenteredTextLine(3, "A gyro=%d", gyro.total/1000.0);
      wait1Msec(1);
    }
    stop_all_motors();
  }
  if ((gyro.total/1000.0) < ang)
  {
    motor[LeftMotor] = spd;
    motor[RightMotor] = -spd;
    while ((gyro.total/1000.0) < ang)
    {
      nxtDisplayCenteredTextLine(3, "B gyro=%d", gyro.total/1000.0);
      wait1Msec(1);
    }
    stop_all_motors();
  }
}

void turngyro_right_oneWheel(float ang, int spd)
{
  long startangle;
  long currentangle;
  long endangle;

  startangle = gyro.total / 1000;
  currentangle = startangle;
  endangle = startangle + ang;

  if (currentangle < endangle)
  {
    motor[LeftMotor] = spd;
    motor[RightMotor] = 0;
    while (currentangle < endangle)
    {
      currentangle = gyro.total / 1000;
      wait1Msec(1);
    }
    stop_all_motors();
  }
  if ((gyro.total/1000.0) > endangle)
  {
    motor[LeftMotor] = -spd;
    motor[RightMotor] = 0;
    while (currentangle > endangle)
    {
      currentangle = gyro.total / 1000;
      wait1Msec(1);
    }
    stop_all_motors();
  }
}

void move_backwards(float dist, int tstop, int lspeed, int rspeed)
{
  int clicks_to_move;
  int stval;  // start time value
  int dtval;  // delta time value

  reset_encoder();

  // Move to the goal
  // remember going backwards
  //clicks_to_move = compute_clicks_per_distance(dist, Wheel_Size);
  clicks_to_move = -compute_clicks_per_distance(dist, Wheel_Size);
  time1[T1] = 0;      // Start the timer at 0 msec
  stval = time1[T1];  // get number of msec
  dtval = 0;          // compute delta time
  motor[LeftMotor] = -lspeed;
  motor[RightMotor] = -rspeed;
  while ((nMotorEncoder[LeftMotor] > clicks_to_move) && (dtval < tstop)) {
    dtval = time1[T1] - stval;
    wait1Msec(1);
  }
  stop_all_motors();
}

void back_turn_left(float rangle, int tstop, int lspeed, int rspeed)
{
  int clicks_to_move;
  int stval;  // start time value
  int dtval;  // delta time value

  reset_encoder();

  // Now lets do a two wheel turn
  // remember going backwards
  clicks_to_move = compute_click_for_two_wheelturn(Wheel_Base, Wheel_Size, rangle);
  time1[T1] = 0;      // Start the timer at 0 msec
  stval = time1[T1];  // get number of msec
  dtval = 0;          // compute delta time
  // again remember depending on which way to want to
  // turn, enable the correct motor direction
  motor[RightMotor] = rspeed;
  motor[LeftMotor] = -lspeed;
  while ((nMotorEncoder[RightMotor] < clicks_to_move) && (dtval < tstop)) {
    dtval = time1[T1] - stval;
    wait1Msec(1);
  }
  stop_all_motors();
}

void back_turn_right (float rangle, int tstop, int lspeed, int rspeed)
{
  int clicks_to_move;
  int stval;  // start time value
  int dtval;  // delta time value

  reset_encoder();

  // Now lets do a two wheel turn
  // remember going backwards
  clicks_to_move = compute_click_for_two_wheelturn(Wheel_Base, Wheel_Size, rangle);
  time1[T1] = 0;      // Start the timer at 0 msec
  stval = time1[T1];  // get number of msec
  dtval = 0;          // compute delta time
  // again remember depending on which way to want to
  // turn, enable the correct motor direction
  motor[RightMotor] = -rspeed;
  motor[LeftMotor] = lspeed;
  while ((nMotorEncoder[LeftMotor] < clicks_to_move) && (dtval < tstop)) {
    dtval = time1[T1] - stval;
    wait1Msec(1);
  }
  stop_all_motors();
}

//void initializeRobot()
//{
//  // Place code here to sinitialize servos to starting positions.
//  // Sensors are automatically configured and setup by ROBOTC. They may need a brief time to stabilize.
//  //the servos are initialzed here so that in the event that a crate is held in autonomous it will not be dropped

//  motor[RightMotor] = 0;
//  motor[LeftMotor] = 0;


//  // Read Magnet Sensor Calibration

//  //nMotorEncoder[drawBridge] = 0;
//  //motor[TrapDoor] = -100;
//  //wait1Msec(500);

//  return;
//}

void initialize_gyro()
{
  long stempI;
  long dzmin;
  long dzmax;

  dzmin = 999999;
  dzmax = 0;

  if (SensorType[GyroSensor] != sensorRawValue) {
    SetSensorType(GyroSensor, sensorRawValue);
    wait1Msec(100);
  }

  // Measure gyro bias - 20 samples every 5 msec = 100 msec;
  stempA = 0;
  for (stempI = 1; stempI < 40; stempI++)
  {
    stempB = SensorValue[GyroSensor];
    // compute Deadzone
    if (stempB > dzmax)
      dzmax = stempB;
    if (stempB < dzmin)
      dzmin = stempB;

    stempB -= stempA;
    stempB /= stempI;

    stempA += stempB;
    wait1Msec(5);
  }

  // Deadzone calculation
  // Find differnce between average and min
  // and max, whichever is greater use that
  // difference * 1.25
  // Use the average (stempA) +/- the above
  // for deadband
  if (abs(stempA - dzmax) > abs(stempA - dzmin))
  {
    stempI = abs(stempA - dzmax) * 1.25;
  }
  else
  {
    stempI = abs(stempA - dzmin) * 1.25;
  }
  gyro.dzmin = stempA - stempI;
  gyro.dzmax = stempA + stempI;

  // Initialize gyro parameters
  gyro.total = 0;
  gyro.dt    = 0;
  gyro.q_err = 0;
  gyro.zero  = 0;
  gyro.bias  = stempA * 1000;

  gyro.rate  = SensorValue[GyroSensor];   // calculate the t0 rate so the first
  // Deadzone computation
  if ((gyro.rate > gyro.dzmax) ||
      (gyro.rate < gyro.dzmin))
  {
    gyro.rate *= 1000;                      // average is good
    gyro.rate -= gyro.bias;
  }
  else
  {
    gyro.rate = 0;
  }

  time1[GyroTimer] = 0;                   // Use a timer to calculate dt
}

task process_gyro
{
  while (true)
  {
    stempA = SensorValue[GyroSensor];
    // Deadzone computation
    if ((stempA > gyro.dzmax) ||
        (stempA < gyro.dzmin))
    {
      // Update the gyro heading
      stempA *= 1000;                      // average is good
      stempA -= gyro.bias;
    }
    else
    {
     stempA = 0;
    }

    gyro.dt = time1[GyroTimer];
    time1[GyroTimer] = 0;

    stempB = gyro.rate;            // retrieve last value
    gyro.rate = stempA;            // save the new value

    // Verify stempA <= stempB
    if (stempB < stempA)
    {
      stempC = stempA;        // swap A <-> B
      stempA = stempB;
      stempB = stempC;
    }

    // Find change in angle by integration (area under curve...
    // well actually the trapezoid since we only have 2 points)
    stempC = stempB - stempA;      // (1/2 (B-A) + A ) * dt
    stempC /= 2;
    stempC += stempA;

    stempC *= gyro.dt;             // * dt (msec)
    stempC += gyro.q_err;          // add back the quantization error from
    stempA = stempC;               // the last calculation
    stempA /= 1000;                // convert msec to sec

    stempB = stempA;               // calculate the remainder
    stempB *= 1000;
    stempC -= stempB;

    gyro.q_err = stempC;           // the remainder is a quantization error that
                                   // we should keep for the next calculation

    stempB = gyro.total;           // update the integrated angle value
    stempB += stempA;
    gyro.total = stempB;

    stempB /= 1000;                // convert from 1/1000th degree to degrees

    // only process the gyro every 10 msec
    while (time1[GyroTimer] < 10)
    {
      wait1Msec(1);
    }
  }
}
