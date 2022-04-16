#define _NARFDUINO_ENABLE_BRUSHLESS_9
#define _NARFDUINO_ENABLE_BRUSHLESS_10
#define BOUNCE_WITH_PROMPT_DETECTION

#include <Bounce2.h>
#include <NarfduinoBattery.h>
#include <NarfduinoBrushless.h>
#include <NarfduinoBridge.h>


// Pin Definitions
#define PIN_TRIGGER_HALF 8
#define PIN_TRIGGER_FULL 7
#define PIN_TRIGGER_REV 13
#define PIN_CYCLE_CONTROL 4
#define PIN_PUSHER_RUN 5
#define PIN_PUSHER_STOP 15
#define PIN_SPEEDPOT A2

// Configuration Options
#define MOTOR_KV 3000
#define MOTOR_CQB_RPM_SPEED 35000 // RPM calculated by 37800 / (Battery S x 4.2 x motor kv) for percentage of full power that we want the motors near in CQB mode
byte MotorSpeedFull = 100; // For full-pull
byte MotorSpeedHalf = 30; // For half-pull. This will be calculated into CQB percentage later
byte MotorSpeedIdle = 30;

// Modes
#define MODE_NORMAL 0
#define MODE_PUSHER_JAM 1
#define MODE_LOWBATTERY 2
byte SystemMode = MODE_NORMAL;


// Physical Switch Status
bool RevTriggerPressed = false; // Rev Trigger is Depressed
bool FireHalfTriggerPressed = false; // Front trigger is Depressed
bool FireFullTriggerPressed = false; // Rear Trigger is Depressed
bool CycleControlPressed = false; // Cycle Control Switch is Depressed

// Firing Controls
#define FIRE_MODE_IDLE 0
#define FIRE_MODE_AUTO 1
#define FIRE_MODE_SINGLE 2
#define FIRE_MODE_LASTSHOT 3
#define FIRE_MODE_RETRACT 4
byte CurrentFireMode = FIRE_MODE_SINGLE; // This is the user request based on the button state
byte ProcessingFireMode = FIRE_MODE_IDLE; // This is what will actually be fired.
bool ExecuteFiring = false; // Set to true when the Solenoid is supposed to move
int TimeBetweenShots = 0; // Calculated to lower ROF
long ShotsToFire = 0; // Number of shots in the queue
unsigned long LastShot = 0; // When the last shot took place.
bool RequestShot = false; // Set to true to request the firing sequence to begin
bool RequestRev = false; // Set to true to request the rev sequence to begin
bool prevRequestRev = false;
bool RequestAutoStop = false; // Set to true to stop Full Auto
bool FiringLastShot = false;

// Motor Controls
#define MOTOR_SPINUP_LAG 110 // How long we give the motors before we know that have spun up.
#define MOTOR_SPINUP_CQB_LAG 90 // How long we give the motors in CQB before firing a semi auto shot
#define MOTOR_REV_DOWN_DELAY 250 // How long from command to fully rev down till revdown begins
#define MOTOR_SPINDOWN_2S 3000
#define MOTOR_SPINDOWN_3S 3000
#define MOTOR_SPINDOWN_4S 4000
#define MOTOR_SPINUP_2S 0
#define MOTOR_SPINUP_3S 0
#define MOTOR_SPINUP_4S 0
#define MOTOR_MAX_SPEED 1855
int MaxMotorSpeed = MOTOR_MAX_SPEED;
int DecelerateTime = 0;
int AccelerateTime = 0;
long MotorRampUpPerMS = 0;
long MotorRampDownPerMS = 0;
int MinMotorSpeed = 1000;
int CurrentMotorSpeed = MinMotorSpeed;
int TargetMotorSpeed = MinMotorSpeed;
byte SetMaxSpeed = 100; // in percent.
unsigned long TimeLastMotorSpeedChanged = 0;
unsigned long TimeSinceRevRequest = 0;
unsigned long MotorSpinupLag = 0;
#define COMMAND_REV_NONE 0
#define COMMAND_REV_HALF 1
#define COMMAND_REV_FULL 2
#define COMMAND_REV_IDLE 3
byte CommandRev = COMMAND_REV_NONE;
byte PrevCommandRev = COMMAND_REV_NONE;
bool AutoRev = false; // True when the computer is managing the rev process.
bool CQBMode = false;


// Inputs
#define DebounceWindow 5 // Debounce Window = 5ms
#define POT_READ_INTERVAL 100
Bounce FireHalfTriggerBounce = Bounce();
Bounce FireFullTriggerBounce = Bounce();
Bounce RevTriggerBounce = Bounce();
Bounce CycleControlBounce = Bounce();


//  Battery Monitoring
NarfduinoBattery Battery = NarfduinoBattery();

//  Setup Brushless Motors
NarfduinoBrushless Flywheels = NarfduinoBrushless();

//  Setup Pusher
NarfduinoBridge Pusher = NarfduinoBridge(5, 15);

// Pusher controls
/*bool PusherStopping = false;
  bool PusherRetracting = false;
  bool PusherPWMFETOn = false; // Keep track of the PWM FET
  bool PusherBrakeFETOn = false; // Keep track of the brake fet
  bool PusherRequest = false; // False = stop pusher, true = run pusher
  #define PusherStop 0     // Pusher brake is on, PWM is off
  #define PusherTransition 1  // Pusher brake is off, PWM is off, waiting for fet caps to discharge
  #define PusherRun 2         // Pusher brake is off, PWM is on
  #define PusherOnTransitionTime 10 // 100ms transition time
  #define PusherOffTransitionTime 2 // 100ms transition time */
#define PusherMaxCycleTime 2000   // Max number of time between cycles before we recognise a jam
bool JamDetected = false;
bool RunningFiringSequence = false;
int PusherSpeedPWM[] = {40, 65, 100, 40, 25, 15,
                        33, 55, 100, 33, 25, 15,
                        30, 50, 80, 30, 25, 15
                       }; // 2s: Slow, Med, High, Single, last shot, retract; 3S; 4S
#define ROF_SLOW 0
#define ROF_MEDIUM 1
#define ROF_HIGH 2
#define ROF_SINGLE 3
#define ROF_LASTSHOT 4
#define ROF_RETRACT 5
byte PusherSpeedIndex = ROF_MEDIUM; // 0 = Slow, 1 = Med, 2 = High (Add base 3 for 3s)
byte PusherROF = 0; // ROF Percentage
byte GetPusherSpeed( byte PSI ) {
  return PusherSpeedPWM[ PSI + (Battery.GetBatteryS() - 2) ]; // Mini function to determine pusher speed based on battery size
}


/*
   This is a boot time init sub to calcualte the Acceleration and
   deceleration ramp rates of the motors.
*/
void CalculateRampRates()
{
  long SpeedRange = (long)(MaxMotorSpeed - MinMotorSpeed) * 1000; // Multiply by 1000 to give some resolution due to integer calculations
  if ( AccelerateTime == 0 )
  {
    MotorRampUpPerMS = SpeedRange;  // For instant acceleration
  }
  else
  {
    MotorRampUpPerMS = SpeedRange / AccelerateTime;  // Use when Accelerating
  }

  if ( DecelerateTime == 0 )
  {
    MotorRampDownPerMS = SpeedRange;  // For instant acceleration
  }
  else
  {
    MotorRampDownPerMS = SpeedRange / DecelerateTime;  // Use when Decelerating
  }


  Serial.print( F("Ramp Up per MS = ") );
  Serial.println( MotorRampUpPerMS );

  Serial.print( F("Ramp Down per MS = ") );
  Serial.println( MotorRampDownPerMS );
}

void setup() {

  // Set up comms
  Serial.begin(57600);
  Serial.println( F("Booting.. ") );

  // Set up debouncing
  Serial.println( F("Configuring Debouncing") );

  pinMode(PIN_TRIGGER_REV, INPUT_PULLUP);
  RevTriggerBounce.attach( PIN_TRIGGER_REV, INPUT_PULLUP );
  RevTriggerBounce.interval( DebounceWindow );

  pinMode(PIN_TRIGGER_HALF, INPUT_PULLUP);
  FireHalfTriggerBounce.attach( PIN_TRIGGER_HALF, INPUT_PULLUP );
  FireHalfTriggerBounce.interval( DebounceWindow );

  pinMode(PIN_TRIGGER_FULL, INPUT_PULLUP);
  FireFullTriggerBounce.attach( PIN_TRIGGER_FULL, INPUT_PULLUP );
  FireFullTriggerBounce.interval( DebounceWindow );

  pinMode(PIN_CYCLE_CONTROL, INPUT_PULLUP);
  CycleControlBounce.attach( PIN_CYCLE_CONTROL, INPUT_PULLUP );
  CycleControlBounce.interval( DebounceWindow );

  pinMode( PIN_SPEEDPOT, INPUT );

  Serial.println( F("Debouncing Configured") );

  // Setup Pusher
  Pusher.Init();
  Pusher.DisableAntiJam();

  // Set up battery
  Serial.println( F("Setting up Battery") );
  Battery.Init();
  Battery.SetupSelectBattery();
  /* float BatteryV = Battery.GetCurrentVoltage();
    while( BatteryV == 99.0 ) // Hold up continuing setup till Battery is plugged in
    {
     delay( 1000 );
     BatteryV = Battery.GetCurrentVoltage();
    }
  */
  byte BatteryS = Battery.GetBatteryS();

  // Configure Motor Speeds
  switch ( BatteryS )
  {
    case 2:
      DecelerateTime = MOTOR_SPINDOWN_2S;
      AccelerateTime = MOTOR_SPINUP_2S;
      break;
    case 3:
      DecelerateTime = MOTOR_SPINDOWN_3S;
      AccelerateTime = MOTOR_SPINUP_3S;
      break;
    case 4:
      DecelerateTime = MOTOR_SPINDOWN_4S;
      AccelerateTime = MOTOR_SPINUP_4S;
    default:
      break;
  }

  Serial.print( F("Battery Connected S = ") );
  Serial.println( Battery.GetBatteryS() );

  Flywheels.Init(); // Initialize motors

  SystemMode = MODE_NORMAL;
  CalculateRampRates();

  // Set CQB Motor Speed
  byte MotorkV = MOTOR_KV;
  byte MotorHalfRPM = MOTOR_CQB_RPM_SPEED;
  MotorSpeedHalf = 100 * (MotorHalfRPM / (Battery.GetCurrentVoltage() * MotorkV));
  if ( MotorSpeedHalf >= 100) MotorSpeedHalf = 100;
  Serial.println( MotorSpeedHalf );

  // Now wait until the trigger is high
  Serial.println( F("Waiting for trigger safety") );

  // Bounce2.h is buggy and turning off input pullups. Turn it on.
  PORTB = PORTB | 0b00100001; // Force input pullup for Rev and Half pull
  PORTD = PORTD | 0b10010000; // Force input pullup for Cycle Control and Full pull

  FireFullTriggerBounce.update();
  FireHalfTriggerBounce.update();
  while ( FireFullTriggerBounce.read() == LOW && FireHalfTriggerBounce.read() == LOW )
  {
    delay(10);
    FireFullTriggerBounce.update();
    FireHalfTriggerBounce.update();
  }
  delay(1000);

  Serial.println( F("Booted") );
}

void loop() {
  // put your main code here, to run repeatedly:

  ProcessButtons(); // Get User and Sensor input
  Battery.ProcessBatteryMonitor(); // Check battery voltage
  ProcessSystemMode(); // Find out what the system should be doing

  // Override Button commands if bridge is stopped yet pusher is out of battery
  ProcessOutOfBattery();

  // Detected a change to the command. Reset the last speed change timer.
  if ( PrevCommandRev != CommandRev )
  {
    TimeLastMotorSpeedChanged = millis();
    PrevCommandRev = CommandRev;
  }

  // Process speed control
  ProcessSpeedControl();
  // Calcualte the new motor speed
  ProcessMotorSpeed();
  // Send the speed to the ESC
  ProcessMainMotors();

  ProcessFiring();
  Pusher.ProcessBridge();
}

/*
   Process input from Buttons and Sensors.
*/
void ProcessButtons()
{
  /*  Set the delay till rev down & lengthen the delay
   *  if pusher is returning to battery in order to allow
   *  idling motors to clear a dart out of the cage.
   */
  unsigned long MotorRevDownDelay = 0;
  unsigned long TimeSinceRevDownCommand = 0;
  if ( !(CurrentFireMode == FIRE_MODE_RETRACT) )
  {
    MotorRevDownDelay = MOTOR_REV_DOWN_DELAY;
  }
  else
  {
    MotorRevDownDelay = 500;
  }
  
  // Bounce2.h is buggy and turning off input pullups. Turn it on.
  PORTB = PORTB | 0b00100001; // Force input pullup for Rev and Half pull
  PORTD = PORTD | 0b10010000; // Force input pullup for Cycle Control and Full pull

  RevTriggerBounce.update(); // Update the pin bounce state
  CycleControlBounce.update(); // Update the pin bounce state
  FireFullTriggerBounce.update(); // Update the pin bounce state
  FireHalfTriggerBounce.update(); // Update the pin bounce state

  RevTriggerPressed = !(RevTriggerBounce.read());
  //Serial.println( RevTriggerPressed );
  CycleControlPressed = !(CycleControlBounce.read());
  FireFullTriggerPressed = !(FireFullTriggerBounce.read());
  //Serial.println( FireFullTriggerPressed );
  FireHalfTriggerPressed = !(FireHalfTriggerBounce.read());
 // Debugging commands for port states
    /*Serial.println( F("HalfTriggerPressed Pin states") );
    Serial.println( !FireHalfTriggerPressed );
    Serial.println( digitalRead( PIN_TRIGGER_HALF ) );
    Serial.println( PINB&0b00000001 );
    Serial.println( PORTB&0b00000001 );*/
  
  if ( CycleControlBounce.fell() )
  {
    if ( CommandRev == COMMAND_REV_NONE )
    {
      ShotsToFire = 0;
      Serial.println( F("CHECK FIRE! CHECK FIRE!") );
      Serial.println( F("EMERGENCY HALT") );
    }
    Pusher.PusherHeartbeat();
    Serial.println( F("Bump Bump") );
    
    if ( ShotsToFire >= 1 )
    {
      ShotsToFire--;
    }
    else
    {
      ShotsToFire = 0;
    }
  }
  else if ( CycleControlPressed )
  {
    Pusher.PusherHeartbeat();
  }

  if ( !(SystemMode == MODE_NORMAL) ) // Something has gone wrong
  {
    RequestShot = false;
    RequestRev = false;
    ShotsToFire = 0;
    return;
  }

  if ( RevTriggerPressed )
  {
    CQBMode = true;
    MotorSpinupLag = MOTOR_SPINUP_CQB_LAG;
    Serial.println( F("Limit motor speed for lower velocity and fire darts sooner") );
  }
  else
  {
    CQBMode = false;
    MotorSpinupLag = MOTOR_SPINUP_LAG;
    Serial.println( F("Motors will rev to maximum allowed speed and pusher will wait longer to give motors time to fully rev") );
  }

  if ( FireHalfTriggerBounce.fell() ) // Handle requesting motor revving
  {
    RequestRev = true;
    Serial.println( F("Rev Requested") );
    TimeSinceRevRequest = millis();
  }
  else if ( FireHalfTriggerPressed )
  {
    if( millis() - TimeSinceRevRequest <= MotorSpinupLag )
    {
      RequestShot = false;
    }
    else
    {
      RequestShot = true;
      CurrentFireMode = FIRE_MODE_SINGLE;
      Serial.println( F("Fire Mission, Single Shot. Over.") );
    }
  }
  
  if ( !FireHalfTriggerPressed )
  {
    RequestRev = false;
    RequestShot = false;
    Serial.println( F("Rounds complete. Rev Stop") );
  }

  if ( FireFullTriggerBounce.fell() )
  {
    if ( !RequestRev ) // In case for some reason motor's aren't still being ordered to spin
    {
      RequestRev = true;
    }
    CurrentFireMode = FIRE_MODE_AUTO;
    RequestShot = true; // auto shot request
    Serial.println(F("Fire for effect!"));
  }

  // Handle Rev Requests
  if ( RequestRev )
  {
    if ( CQBMode )
    {
      CommandRev = COMMAND_REV_HALF;
    }
    else
    {
      CommandRev = COMMAND_REV_FULL;
    }
  }
  else
  {
    if ( (millis() - TimeSinceRevDownCommand >= MotorRevDownDelay) && !(CurrentFireMode == FIRE_MODE_RETRACT) )
    {
      CommandRev = COMMAND_REV_NONE;
    }
    else if ( CurrentFireMode == FIRE_MODE_RETRACT )
    {
      CommandRev = COMMAND_REV_IDLE;
    }
  }
  
  // This is purely for debugging
  if ( RequestShot )
  {
    Serial.println( F("Shot over") );
  }
}

// We are checking for pusher out of battery
void ProcessOutOfBattery()
{
  if ( !CycleControlPressed && CurrentFireMode == FIRE_MODE_IDLE ) // Pusher Stuck out of battery
  {
    RequestShot = true;
    CurrentFireMode = FIRE_MODE_RETRACT;
  }
}

// We are toggling between different system states here..
void ProcessSystemMode()
{
  static byte LastSystemMode = MODE_NORMAL;

  if ( Battery.IsBatteryFlat() ) // Battery low
  {
    SystemMode = MODE_LOWBATTERY;
    Serial.println(F("System Mode: Low Battery"));
  }

  else if ( Pusher.HasJammed() ) // Jam is detected
  {
    SystemMode = MODE_PUSHER_JAM;

    if ( LastSystemMode != MODE_PUSHER_JAM )
    {
      SystemMode = MODE_PUSHER_JAM;

      ShotsToFire = 0;
      Serial.println( F("Halt pusher") );
      Serial.println( F("Pusher Jam"));
      Pusher.StopBridge();

      CommandRev = COMMAND_REV_NONE;
      CurrentFireMode = FIRE_MODE_IDLE;
      RequestShot = false;

      Serial.println( F("Pusher Jam Detected") );
    }
  }
  else
  {
    SystemMode = MODE_NORMAL;
  }

  if ( LastSystemMode != SystemMode )
  {
    Serial.print( F("New System Mode = ") );
    Serial.println( SystemMode );
    LastSystemMode = SystemMode;
  }
}

// Update the motors with the new speed
void ProcessMainMotors()
{
  static int PreviousMotorSpeed = 1000;

  if ( PreviousMotorSpeed != CurrentMotorSpeed )
  {
    // Debugging output
    //Serial.println(CurrentMotorSpeed);

    // Use this for Servo Library
    if ( CurrentMotorSpeed > MOTOR_MAX_SPEED )
      Flywheels.UpdateSpeed( MOTOR_MAX_SPEED );
    else
      Flywheels.UpdateSpeed( CurrentMotorSpeed );

    PreviousMotorSpeed = CurrentMotorSpeed;
  }
}

/*
   Calculate the desired motor speed
*/
void ProcessMotorSpeed()
{
  // Don't do anything if the motor is already running at the desired speed.
  if ( CurrentMotorSpeed == TargetMotorSpeed )
  {
    return;
  }

  unsigned long CurrentTime = millis(); // Need a base time to calcualte from
  unsigned long MSElapsed = CurrentTime - TimeLastMotorSpeedChanged;
  if ( MSElapsed == 0 ) // No meaningful time has elapsed, so speed will not change
  {
    return;
  }
  if ( CurrentMotorSpeed < TargetMotorSpeed )
  {
    long SpeedDelta = (MSElapsed * MotorRampUpPerMS / 1000);
    if ( SpeedDelta < 1 ) return; // Not enough cycles have passed to make an appreciable difference to speed.
    int NewMotorSpeed = CurrentMotorSpeed + SpeedDelta; // Calclate the new motor speed..

    // If it's within 1% (which is 10) of target, then just set it
    if ( NewMotorSpeed + 10 >= TargetMotorSpeed )
    {
      NewMotorSpeed = TargetMotorSpeed;
    }

    TimeLastMotorSpeedChanged = CurrentTime;
    CurrentMotorSpeed = NewMotorSpeed;
  }
  if ( CurrentMotorSpeed > TargetMotorSpeed )
  {
    //Serial.println( MSElapsed );
    long SpeedDelta = (MSElapsed * MotorRampDownPerMS / 1000);
    if ( SpeedDelta < 1 ) return; // Not enough cycles have passed to make an appreciable difference to speed.
    int NewMotorSpeed = CurrentMotorSpeed - SpeedDelta; // Calclate the new motor speed..

    // If it's within 1% (which is 10) of target, then just set it
    if ( NewMotorSpeed - 10 <= TargetMotorSpeed )
    {
      NewMotorSpeed = TargetMotorSpeed;
    }

    TimeLastMotorSpeedChanged = CurrentTime;
    CurrentMotorSpeed = NewMotorSpeed;
  }
}

// We need to set the Target Motor Speed here.
void ProcessSpeedControl()
{
  static byte LastSetMaxSpeed = 100;

  if ( CommandRev == COMMAND_REV_HALF ) SetMaxSpeed = MotorSpeedHalf;
  if ( CommandRev == COMMAND_REV_FULL ) SetMaxSpeed = MotorSpeedFull;
  if ( CommandRev == COMMAND_REV_IDLE ) SetMaxSpeed = MotorSpeedIdle;
  if ( CommandRev == COMMAND_REV_NONE ) SetMaxSpeed = 0;

  if ( LastSetMaxSpeed == SetMaxSpeed ) return; // Speed hasn't changed

  if ( CommandRev > COMMAND_REV_NONE )
  {
    SetMaxSpeed = constrain( SetMaxSpeed, 30, 100 ); // Constrain between 30% and 100%
  }

  TargetMotorSpeed = map( SetMaxSpeed, 0, 100, MinMotorSpeed, MaxMotorSpeed );  // Find out our new target speed.

  LastSetMaxSpeed = SetMaxSpeed;

  // Serial.print( F("New max speed % = ") );
  // Serial.println( SetMaxSpeed );

  //Serial.print( F("New target speed = ") );
  //Serial.println( TargetMotorSpeed );
}

void ProcessFiring()
{
  if ( !(SystemMode == MODE_NORMAL) ) // Finish off the stroke unless in running or in ROF config mode
  {
    ShotsToFire = 1;
    CurrentFireMode = FIRE_MODE_LASTSHOT;
    Pusher.SetBridgeSpeed( GetPusherSpeed( ROF_LASTSHOT ) );
    return;
  }

  // Requesting Shot while we were doing nothing special
  if ( RequestShot )
  {
    Serial.println( F("Shot Out") );
    switch ( CurrentFireMode )
    {
      case FIRE_MODE_SINGLE:
        ShotsToFire = 1; // Add another shot to the queue
        Pusher.SetBridgeSpeed( GetPusherSpeed( ROF_SINGLE ) );
        break;
      case FIRE_MODE_AUTO:
        ShotsToFire = 9999; // Set to something unreasonably high
        Pusher.SetBridgeSpeed( GetPusherSpeed( PusherSpeedIndex ) );
        break;
      case FIRE_MODE_LASTSHOT:
        ShotsToFire = 1;
        Pusher.SetBridgeSpeed( GetPusherSpeed( ROF_LASTSHOT ) );
        RequestShot = false;
        break;
      case FIRE_MODE_RETRACT:
        ShotsToFire = 0;
        Pusher.SetBridgeSpeed( GetPusherSpeed( ROF_RETRACT ) );
        break;
    }
    Pusher.StartBridge();
  }

  if ( ShotsToFire <= 0 && !(CurrentFireMode == FIRE_MODE_IDLE) && CycleControlPressed)
  {
    Pusher.StopBridge();
    CurrentFireMode = FIRE_MODE_IDLE;
  }
}
