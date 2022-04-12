#include <Bounce2.h>
#include <NarfduinoBattery.h>


// Pin Definitions
#define PIN_TRIGGER_HALF 8
#define PIN_TRIGGER_FULL 7
#define PIN_TRIGGER_REV 13
#define PIN_CYCLE_CONTROL 4
#define PIN_MOTOR_A 9
#define PIN_MOTOR_B 10
#define PIN_PUSHER_RUN 5
#define PIN_PUSHER_STOP 15
#define PIN_SPEEDPOT A2

// Configuration Options
#define MOTOR_KV 3000
#define MOTOR_CQB_RPM_SPEED 37800 // RPM calculated by 37800 / (Battery S x 4.2 x motor kv) for percentage of full power that we want the motors near in CQB mode
byte MotorSpeedFull = 100; // For full-pull
byte MotorSpeedHalf = 30; // For half-pull. This will be calculated into CQB percentage later

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
#define FIRE_MODE_SINGLE 0
#define FIRE_MODE_AUTO 1
#define FIRE_MODE_AUTO_LASTSHOT 2
#define FIRE_MODE_IDLE 3
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
bool AutoFire = false;
int AutoFireMotorSpeed = 0;

// Motor Controls
#define MOTOR_SPINUP_LAG 150 // How long we give the motors before we know that have spun up.
#define MOTOR_SPINDOWN_2S 3000
#define MOTOR_SPINDOWN_3S 4000
#define MOTOR_SPINDOWN_4S 6000
#define MOTOR_SPINUP_2S 0
#define MOTOR_SPINUP_3S 0
#define MOTOR_SPINUP_4S 0
#define MOTOR_MAX_SPEED 2000
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
unsigned long TimeSinceCQBRevRequest = 0;
#define COMMAND_REV_NONE 0
#define COMMAND_REV_HALF 1
#define COMMAND_REV_FULL 2
byte CommandRev = COMMAND_REV_NONE;
byte PrevCommandRev = COMMAND_REV_NONE;
bool AutoRev = false; // True when the computer is managing the rev process.


// Inputs
#define DebounceWindow 5 // Debounce Window = 5ms
#define POT_READ_INTERVAL 100
Bounce FireHalfTriggerBounce = Bounce();
Bounce FireFullTriggerBounce = Bounce();
Bounce RevTriggerBounce = Bounce();
Bounce CycleControlBounce = Bounce();


// Battery Monitoring
NarfduinoBattery Battery = NarfduinoBattery();


// Pusher controls
bool PusherStopping = false;
bool PusherRetracting = false;
bool PusherPWMFETOn = false; // Keep track of the PWM FET
bool PusherBrakeFETOn = false; // Keep track of the brake fet
bool PusherRequest = false; // False = stop pusher, true = run pusher
#define PusherStop 0     // Pusher brake is on, PWM is off
#define PusherTransition 1  // Pusher brake is off, PWM is off, waiting for fet caps to discharge
#define PusherRun 2         // Pusher brake is off, PWM is on
byte CurrentPusherStatus = PusherTransition; // Start in transition mode for boot.
unsigned long PusherTransitionStart = 0; // Time transition started
#define PusherOnTransitionTime 10 // 100ms transition time 
#define PusherOffTransitionTime 2 // 100ms transition time 
unsigned long TimeLastPusherResetOrActivated = 0; // We are keeping track when the pusher was last reset for anti-jam purposes.
#define PusherMaxCycleTime 2000   // Max number of time between cycles before we recognise a jam
bool JamDetected = false;
bool RunningFiringSequence = false;
int PusherSpeedPWM[] = {40, 65, 100, 40, 50, 50, 
                        33, 55, 88, 75, 50, 50,
                        30, 50, 75, 66, 50,  50}; // 2s: Slow, Med, High, Single, last shot, retract; 3S; 4S
#define ROF_SLOW 0
#define ROF_MEDIUM 1
#define ROF_HIGH 2
#define ROF_SINGLE 3
#define ROF_LASTSHOT 4
#define ROF_RETRACT 5
byte PusherSpeedIndex = ROF_HIGH; // 0 = Slow, 1 = Med, 2 = High (Add base 3 for 3s)
byte PusherROF = 0; // ROF Percentage
byte GetPusherSpeed( byte PSI ) {return PusherSpeedPWM[ PSI + (Battery.GetBatteryS() - 2) ];}  // Mini function to determine pusher speed based on battery sizr


// Updates the PWM Timers
void UpdatePWM( int NewSpeed )
{
  NewSpeed = (NewSpeed * 2) + 2; // Adjust for the prescalar
  OCR1A = NewSpeed;
  OCR1B = NewSpeed;
}


/*
 * This is a boot time init sub to calcualte the Acceleration and 
 * deceleration ramp rates of the motors.
 */
void CalculateRampRates()
{
  long SpeedRange = (long)(MaxMotorSpeed - MinMotorSpeed) * 1000; // Multiply by 1000 to give some resolution due to integer calculations
  if( AccelerateTime == 0 )
  {
    MotorRampUpPerMS = SpeedRange;  // For instant acceleration
  }
  else
  {
    MotorRampUpPerMS = SpeedRange / AccelerateTime;  // Use when Accelerating
  }

  if( DecelerateTime == 0 )
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
  RevTriggerBounce.attach( PIN_TRIGGER_REV );
  RevTriggerBounce.interval( DebounceWindow );

  pinMode(PIN_TRIGGER_HALF, INPUT_PULLUP);
  FireHalfTriggerBounce.attach( PIN_TRIGGER_HALF );
  FireHalfTriggerBounce.interval( DebounceWindow );
  
  pinMode(PIN_TRIGGER_FULL, INPUT_PULLUP);
  FireFullTriggerBounce.attach( PIN_TRIGGER_FULL );
  FireFullTriggerBounce.interval( DebounceWindow );

  pinMode(PIN_CYCLE_CONTROL, INPUT_PULLUP);
  CycleControlBounce.attach( PIN_CYCLE_CONTROL );
  CycleControlBounce.interval( DebounceWindow );

  pinMode( PIN_SPEEDPOT, INPUT );

  Serial.println( F("Debouncing Configured") );

  // Setup Motor Outputs
  Serial.println( F("Configuring PWM Ports") );
  pinMode( PIN_MOTOR_A, OUTPUT );
  pinMode( PIN_MOTOR_B, OUTPUT );
  digitalWrite( PIN_MOTOR_A, LOW );
  digitalWrite( PIN_MOTOR_B, LOW );

  // Start the 16 bit PWM timer, and arm the ESC
  TCCR1A = 0;
  TCCR1A = (1 << WGM11) | (1 << COM1A1) | (1 << COM1B1);
  TCCR1B = 0;
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
  ICR1 = 40000;
  UpdatePWM( 1000 );

  // Setup Pusher Outputs
  Serial.println( F("Configuring Pusher FET") );
  pinMode( PIN_PUSHER_RUN, OUTPUT );
  digitalWrite( PIN_PUSHER_RUN, LOW );  
  pinMode( PIN_PUSHER_STOP, OUTPUT );
  digitalWrite( PIN_PUSHER_STOP, LOW ); 

  // Set up battery
  Serial.println( F("Setting up Battery") );
  Battery.Init();
  Battery.SetupSelectBattery();
  byte BatteryS = Battery.GetBatteryS();
  Serial.print( F("Battery Connected S = ") );
  Serial.println( Battery.GetBatteryS() );

  // Configure Motor Speeds
  switch( BatteryS )
  {
    case 2:
      DecelerateTime = MOTOR_SPINDOWN_2S;
      AccelerateTime = MOTOR_SPINUP_2S;
      break;
    case 3:
      DecelerateTime = MOTOR_SPINDOWN_3S;
      AccelerateTime = MOTOR_SPINUP_3S;
      break;
    default: // 4s
      DecelerateTime = MOTOR_SPINDOWN_4S;
      AccelerateTime = MOTOR_SPINUP_4S;
      break;
  }

  SystemMode = MODE_NORMAL;
  CalculateRampRates();

  // Set CQB Motor Speed
  byte MotorkV = MOTOR_KV;
  byte MotorHalfRPM = MOTOR_CQB_RPM_SPEED;
  MotorSpeedHalf = 100 * (MotorHalfRPM / ((BatteryS * 4.2) * MotorkV));
  if( MotorSpeedHalf >= 100) MotorSpeedHalf = 100;

  // Now wait until the trigger is high
  Serial.println( F("Waiting for trigger safety") );
  FireFullTriggerBounce.update();
  FireHalfTriggerBounce.update();
  while( FireFullTriggerBounce.read() == LOW && FireHalfTriggerBounce.read() == LOW )
  {
    delay(10);
    FireFullTriggerBounce.update();
    FireHalfTriggerBounce.update();
  }
  delay(10);

  Serial.println( F("Booted") );   
}

void loop() {
  // put your main code here, to run repeatedly:

  ProcessButtons(); // Get User and Sensor input
  Battery.ProcessBatteryMonitor(); // Check battery voltage
  ProcessSystemMode(); // Find out what the system should be doing
  
  ProcessRevCommand(); // Handle motor intentions
  
  // Detected a change to the command. Reset the last speed change timer.
  if( PrevCommandRev != CommandRev )
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
  ProcessPusherReturn();
  ProcessPusher();  

}

/*
 * Process input from Buttons and Sensors.
 */
void ProcessButtons()
{
  bool CQBMode = false;
  RevTriggerBounce.update(); // Update the pin bounce state
  RevTriggerPressed = !(RevTriggerBounce.read());

  CycleControlBounce.update(); // Update the pin bounce state
  CycleControlPressed = !(CycleControlBounce.read());

  FireFullTriggerBounce.update(); // Update the pin bounce state
  FireFullTriggerPressed = !(FireFullTriggerBounce.read());

  FireHalfTriggerBounce.update(); // Update the pin bounce state
  FireHalfTriggerPressed = !(FireHalfTriggerBounce.read());
  
  if( !RevTriggerPressed && FireFullTriggerBounce.fell() )
  {
    RequestShot = true; // manual shot request
  }
  else if( RevTriggerPressed && FireHalfTriggerBounce.rose() )
  {
    if( (millis() < TimeSinceCQBRevRequest) && (TimeSinceCQBRevRequest - millis() >= 90) ) // Sufficient time has passed for flywheels to rev for prefiring
    {
      RequestShot = true; // Prefire semi shot request on CQB mode.
      CQBMode = true;
    }
  }
  else if( RevTriggerPressed && FireFullTriggerPressed && CQBMode == true)
  {
    RequestShot = true; // fire request 
  }

  if( (!RevTriggerPressed && FireFullTriggerBounce.rose()) || (RevTriggerPressed && (FireHalfTriggerBounce.rose() || FireFullTriggerBounce.rose())) )
  {
    RequestAutoStop = true; // Programatically keep track of the request to stop the pusher
    if (FireHalfTriggerBounce.rose() && CQBMode == true)
    {
      CQBMode = false;
    }
  }

  // Determine the current firing mode
  RevTriggerBounce.update();
  FireHalfTriggerBounce.update();
  FireFullTriggerBounce.update();
  
  if( !AutoFire )
  {
    if( RevTriggerBounce.read() == LOW && FireFullTriggerBounce.read() == LOW && CurrentFireMode != FIRE_MODE_AUTO_LASTSHOT )
      CurrentFireMode = FIRE_MODE_AUTO;
    else if( (RevTriggerBounce.read() == HIGH && FireHalfTriggerBounce.read() == LOW && FireFullTriggerBounce.read() == HIGH) || (RevTriggerBounce.read() == HIGH && FireFullTriggerBounce.read() == LOW) )
      CurrentFireMode = FIRE_MODE_SINGLE;
  }
}


// We are toggling between different system states here..
void ProcessSystemMode()
{
  static byte LastSystemMode = MODE_NORMAL;

  if( Battery.IsBatteryFlat() ) // Battery low
  {
    SystemMode = MODE_LOWBATTERY;
  }

  else if( JamDetected ) // Jam is detected
  {
    SystemMode = MODE_PUSHER_JAM;
    
    if( LastSystemMode != MODE_PUSHER_JAM )
    {
      SystemMode = MODE_PUSHER_JAM;

      ShotsToFire = 0;
      Serial.println( F("Halt pusher") );
      PusherStopping = true;
      StopPusher(); 

      ExecuteFiring = false;
      if( AutoFire )
      {
        AutoFire = false;
        RequestShot = false;
        AutoRev = false;
        CommandRev = COMMAND_REV_NONE;
        ProcessingFireMode = FIRE_MODE_IDLE;
      }
      else
      {
        CommandRev = COMMAND_REV_NONE;
        ProcessingFireMode = FIRE_MODE_IDLE; 
        RequestShot = false;      
      }
      
      Serial.println( F("Pusher Jam Detected") );  
      
    }
    
  }
  
  else 
  {
    SystemMode = MODE_NORMAL;
  }

  if( LastSystemMode != SystemMode )
  {
    Serial.print( F("New System Mode = ") );
    Serial.println( SystemMode );
    LastSystemMode = SystemMode;
  }
}


/*
 * Process the manual commands leading to motor reving
 * 
 * Logic:
 * If AutoRev is being performed, disconnect it when the half trigger is pulled.
 * We are looking for the following events: 
 * If the Half Trigger is pressed, Rev to Speed A
 * If the Rev Trigger is pressed, and the Half Trigger is also pressed, Rev to Speed B
 * If the Rev Trigger is pressed, but the Half Trigger is not, then ignore the command.
 * 
 */
void ProcessRevCommand()
{

  if( !(SystemMode == MODE_NORMAL) ) // Spin the motors down when something out of the ordinary happens.
  {
     CommandRev = COMMAND_REV_NONE;
     AutoRev = false;
     return;
  }

  if( (prevRequestRev != RequestRev) && (SystemMode == MODE_NORMAL) )
  {
    // Human has taken control - disengage autopilot but only when not in config mode
    prevRequestRev = RequestRev;
    AutoRev = false;
  }

  if( !AutoRev )
  {
    if( !RevTriggerPressed && FireHalfTriggerPressed )
    {
      CommandRev = COMMAND_REV_FULL;
    }
    else if( RevTriggerPressed && FireHalfTriggerBounce.fell() )
    {
      TimeSinceCQBRevRequest = millis();
      CommandRev = COMMAND_REV_FULL;
    }
    else
    {
      CommandRev = COMMAND_REV_NONE;
    }

    if( (RevTriggerPressed && FireHalfTriggerPressed) && ((millis() - TimeSinceCQBRevRequest) >= 90) )
    {
      CommandRev = COMMAND_REV_HALF;
    }
  }
  // Else the computer is controlling, and the current rev trigger state is ignored. Autopilot will adjust CommandRev
  
}

// Update the motors with the new speed
void ProcessMainMotors()
{
  static int PreviousMotorSpeed = MinMotorSpeed;

  if( PreviousMotorSpeed != CurrentMotorSpeed ) 
  { 
    // Debugging output
    //Serial.println(CurrentMotorSpeed);

    // Use this for Servo Library
    if( CurrentMotorSpeed > MOTOR_MAX_SPEED )
      UpdatePWM( MOTOR_MAX_SPEED );
    else
      UpdatePWM( CurrentMotorSpeed );

    PreviousMotorSpeed = CurrentMotorSpeed;
  }
}

/*
 * Calculate the desired motor speed
 */
void ProcessMotorSpeed()
{
  // Don't do anything if the motor is already running at the desired speed.
  if( CurrentMotorSpeed == TargetMotorSpeed )
  {
    return;
  }

  unsigned long CurrentTime = millis(); // Need a base time to calcualte from
  unsigned long MSElapsed = CurrentTime - TimeLastMotorSpeedChanged;
  if( MSElapsed == 0 ) // No meaningful time has elapsed, so speed will not change
  {
    return;
  }
  if( CurrentMotorSpeed < TargetMotorSpeed )
  {
    long SpeedDelta = (MSElapsed * MotorRampUpPerMS / 1000);
    if( SpeedDelta < 1 ) return; // Not enough cycles have passed to make an appreciable difference to speed.    
    int NewMotorSpeed = CurrentMotorSpeed + SpeedDelta; // Calclate the new motor speed..  

    // If it's within 1% (which is 10) of target, then just set it
    if( NewMotorSpeed + 10 >= TargetMotorSpeed )
    {
      NewMotorSpeed = TargetMotorSpeed;
    }

    TimeLastMotorSpeedChanged = CurrentTime;
    CurrentMotorSpeed = NewMotorSpeed;
  }
  if( CurrentMotorSpeed > TargetMotorSpeed )
  {
    //Serial.println( MSElapsed );
    long SpeedDelta = (MSElapsed * MotorRampDownPerMS / 1000);
    if( SpeedDelta < 1 ) return; // Not enough cycles have passed to make an appreciable difference to speed.
    int NewMotorSpeed = CurrentMotorSpeed - SpeedDelta; // Calclate the new motor speed..

    // If it's within 1% (which is 10) of target, then just set it
    if( NewMotorSpeed - 10 <= TargetMotorSpeed )
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

  if( CommandRev == COMMAND_REV_HALF ) SetMaxSpeed = MotorSpeedHalf;
  if( CommandRev == COMMAND_REV_FULL ) SetMaxSpeed = MotorSpeedFull;
  if( CommandRev == COMMAND_REV_NONE ) SetMaxSpeed = 0;

  if( LastSetMaxSpeed == SetMaxSpeed ) return; // Speed hasn't changed

  if( CommandRev > COMMAND_REV_NONE ) 
  {
    SetMaxSpeed = constrain( SetMaxSpeed, 30, 100 ); // Constrain between 30% and 100%
  }
  
  TargetMotorSpeed = map( SetMaxSpeed, 0, 100, MinMotorSpeed, MaxMotorSpeed );  // Find out our new target speed.

  LastSetMaxSpeed = SetMaxSpeed;

  Serial.print( F("New max speed % = ") );
  Serial.println( SetMaxSpeed );

  Serial.print( F("New target speed = ") );
  Serial.println( TargetMotorSpeed );
}

void ProcessFiring()
{
  if( !(SystemMode == MODE_NORMAL) ) // Finish off the stroke unless in running or in ROF config mode
  {
    ShotsToFire = 0;
    if( ProcessingFireMode == FIRE_MODE_AUTO_LASTSHOT )
      ProcessingFireMode = FIRE_MODE_IDLE;
    return;
  }

  static unsigned long InitiatedAutoFire = 0;
  if( AutoFire )
  {
    if( (InitiatedAutoFire == 0) && (CommandRev == COMMAND_REV_NONE) ) // Started auto fire process. Start spinning the motors
    {
      InitiatedAutoFire = millis();
      AutoRev = true;
      CommandRev = AutoFireMotorSpeed;
      return;
    }
    else if( InitiatedAutoFire == 0 )
    {
      InitiatedAutoFire = millis();
      return;      
    }
    if( (millis() - InitiatedAutoFire) < (MOTOR_SPINUP_LAG) ) // Wait for the lag in non-cqb mode
    {
      return;
    }
    RequestShot = true;
  }
  else
  {
    InitiatedAutoFire = 0;
  }
    

  if( (CommandRev == COMMAND_REV_NONE) && (SystemMode == MODE_NORMAL) && RequestShot ) // Trigger was pushed. 
  {
    AutoFire = true; // Actually, do nothing now except for indicate that we need to be 
    AutoFireMotorSpeed = COMMAND_REV_FULL;
    InitiatedAutoFire = 0;
    return;
  }

  // Requesting Shot while we were doing nothing special
  if( RequestShot && (ProcessingFireMode == FIRE_MODE_IDLE) )
  {
    ProcessingFireMode = CurrentFireMode;
    switch( ProcessingFireMode )
    {
      case FIRE_MODE_SINGLE:
        ShotsToFire = 1; // Add another shot to the queue
        PusherROF = GetPusherSpeed( ROF_SINGLE );
        ExecuteFiring = true;
        break;        
      case FIRE_MODE_AUTO:
        ShotsToFire = 9999; // Set to something unreasonably high
        PusherROF = GetPusherSpeed( PusherSpeedIndex );
        ExecuteFiring = true;
        break;        
    }

    if( ShotsToFire > 0 )
    {
      StartPusher();
    }
  }
  else if( RequestAutoStop && (ProcessingFireMode == FIRE_MODE_AUTO) ) // Requesting Stop while firing in Full Auto 
  {
    ProcessingFireMode = FIRE_MODE_AUTO_LASTSHOT;
    PusherROF = GetPusherSpeed( ROF_LASTSHOT );
    ExecuteFiring = true;
    ShotsToFire = 0;
  }
}

// Use this to manage requests to handle the pusher state.
void ProcessPusher()
{
  // Logic:
  // 1 - Do we need to change? If so, initiate transition
  // 2 - Are we in transition? If so, write digital 0 to both fets. Keep doing that
  // 3 - Are we out of transition? If so, 
  // 4.a  - Are we wanting to run? Are we already running or just out of transition?
  //        If so, turn on pusher fet, ensure brake fet is off
  //        Else turn fets off, and start transition timer
  // 4.b  - Are we wanting to stop? Are we already stoppedor just out of transition?
  //        If so, turn off pusher fet, and turn on brake fet
  //        Else turn fets off and start transition timer


  static bool LastPusherRequest = true;
  static int SelectedTransitionTime = 100;

  // Step 1 - initiate transition
  if( LastPusherRequest != PusherRequest )
  {
    CurrentPusherStatus = PusherTransition;
    if( PusherRequest )
    {
      SelectedTransitionTime = PusherOnTransitionTime;
    }
    else
    {
      SelectedTransitionTime = PusherOffTransitionTime;
    }
    PusherTransitionStart = millis();
    LastPusherRequest = PusherRequest;
    Serial.println( "Pusher FET now in Transition" );
    return;
  }

  // Step 2 - wait for transition to complete
  if( CurrentPusherStatus == PusherTransition )
  {
    digitalWrite( PIN_PUSHER_RUN, LOW );
    PusherPWMFETOn = false;
    digitalWrite( PIN_PUSHER_STOP, LOW );
    PusherBrakeFETOn = false;
    if( millis() - PusherTransitionStart <= SelectedTransitionTime )
      return;

    // We are now out of transition
    if( PusherRequest )
    {
      CurrentPusherStatus = PusherRun;
      Serial.println( "Pusher FET now in RUN" );
    }
    else
    {
      CurrentPusherStatus = PusherStop;
      Serial.println( "Pusher FET now in STOP" );
    }
    
    return;
  }
  
  if( CurrentPusherStatus == PusherRun )
  {
    // Check to make sure brake fet is not on
    if( PusherBrakeFETOn )
    {
      Serial.println( F( "** ERROR ** Pusher Run Status with Brake FET on!!!" ) );
      return;
    }
    if( JamDetected )
    {
      digitalWrite( PIN_PUSHER_RUN, LOW );
      return;
    }

    if( (millis() - TimeLastPusherResetOrActivated) > PusherMaxCycleTime )
    {
      // Jam detected
      digitalWrite( PIN_PUSHER_RUN, LOW );
      JamDetected = true;
      return;
    }
    else
    {
      JamDetected = false;
    }

    if( PusherROF == 100 )
    {
      // Digital write HIGH for 100%
      digitalWrite( PIN_PUSHER_RUN, HIGH );
      //Serial.println( 255 );
    }
    else if( PusherROF == 0 )
    {
      // Just in case we want 0%
      digitalWrite( PIN_PUSHER_RUN, LOW );
      //Serial.println( 0 );
    }
    else
    {
      // PWM Write
      int PWM = map( PusherROF, 0, 100, 0, 255 );
      analogWrite( PIN_PUSHER_RUN, PWM );
      //Serial.println( PWM );
    }
    PusherPWMFETOn = true;
    
    digitalWrite( PIN_PUSHER_STOP, LOW );
    PusherBrakeFETOn = false;

    return;
  }

  if( CurrentPusherStatus == PusherStop )
  {
    // Check to make sure motor fet is not on
    if( PusherPWMFETOn )
    {
      Serial.println( F( "** ERROR ** Pusher Stop Status with PWM FET on!!!" ) );
      return;
    }

    digitalWrite( PIN_PUSHER_RUN, LOW );
    PusherPWMFETOn = false;
    
    digitalWrite( PIN_PUSHER_STOP, HIGH );
    PusherBrakeFETOn = true;

    return;
  }
}

void StartPusher()
{  
  PusherStopping = false;
  PusherRequest = true;
  TimeLastPusherResetOrActivated = millis();
}

void StopPusher()
{
  Serial.println( F("Pusher Stopped!!") );
  JamDetected = false;
  PusherRequest = false;
}

/*
 * Process When the Pusher Returns Home
 */
void ProcessPusherReturn()
{
  static bool LastCycleControlPressed = true;

  if( CycleControlBounce.fell() )
  {
    if( CommandRev == COMMAND_REV_NONE )
    {
      ShotsToFire = 0;
      Serial.println( "EMERGENCY HALT" );
    } 
    if( ShotsToFire > 1 )
    {
      ShotsToFire --; // Decrease the number of darts to fire.
      TimeLastPusherResetOrActivated = millis();
      Serial.print( F("Darts remaining: ") );
      Serial.println( ShotsToFire );
    }
    else
    {
      ShotsToFire = 0;
      if( (ShotsToFire <= 1) && (ProcessingFireMode != FIRE_MODE_SINGLE ) )
      {
        Serial.println( "Slow down pusher" );
        PusherROF = GetPusherSpeed( ROF_LASTSHOT );      
      }
      Serial.println( F("Halt pusher") );
      FiringLastShot = false;
      PusherStopping = true;
      StopPusher(); 

      ExecuteFiring = false;
      if( AutoFire )
      {
        AutoFire = false;
        RequestShot = false;
        AutoRev = false;
        CommandRev = COMMAND_REV_NONE;
        ProcessingFireMode = FIRE_MODE_IDLE;
      }
      else
      {
        ProcessingFireMode = FIRE_MODE_IDLE; 
        RequestShot = false;      
      }
    }
  }
  if( CycleControlBounce.rose() )
  {
    if( (ShotsToFire <= 1) && (ExecuteFiring) )
    {
      FiringLastShot = true;
      PusherROF = GetPusherSpeed( ROF_LASTSHOT );
      Serial.println( "Running last shot now!" );
    }
    else
    {
      Serial.println( "Not Last!" );
      FiringLastShot = false;
    }
  }
}
