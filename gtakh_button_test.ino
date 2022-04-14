
#include <Bounce2.h>

// Pin Definitions
#define PIN_TRIGGER_HALF 8
#define PIN_TRIGGER_FULL 7
#define PIN_TRIGGER_REV 13
#define PIN_CYCLE_CONTROL 4

// Inputs
#define DebounceWindow 10 // Debounce Window = 5ms
Bounce FireHalfTriggerBounce = Bounce();
Bounce FireFullTriggerBounce = Bounce();
Bounce RevTriggerBounce = Bounce();
Bounce CycleControlBounce = Bounce();

// Physical Switch Status
bool RevTriggerPressed = false; // Rev Trigger is Depressed
bool FireHalfTriggerPressed = false; // Front trigger is Depressed
bool FireFullTriggerPressed = false; // Rear Trigger is Depressed
bool CycleControlPressed = false; // Cycle Control Switch is Depressed

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
}

void loop() {
  // put your main code here, to run repeatedly:

  RevTriggerBounce.update(); // Update the pin bounce state
  RevTriggerPressed = !(RevTriggerBounce.read());

  CycleControlBounce.update(); // Update the pin bounce state
  CycleControlPressed = !(CycleControlBounce.read());

  FireFullTriggerBounce.update(); // Update the pin bounce state
  FireFullTriggerPressed = !(FireFullTriggerBounce.read());

  FireHalfTriggerBounce.update(); // Update the pin bounce state
  FireHalfTriggerPressed = !(FireHalfTriggerBounce.read());

  if( RevTriggerBounce.fell() )
  {
    Serial.println( "Rev Trigger input fall" );
  }
  else if( RevTriggerBounce.rose() )
  {
    Serial.println( "Rev Trigger input rose" );
  }
  else if( RevTriggerPressed )
  {
    Serial.println( "Rev Trigger Pressed" );
  }

  if( CycleControlBounce.fell() )
  {
    Serial.println( "Cycle Control input fall" );
  }
  else if( CycleControlBounce.rose() )
  {
    Serial.println( "Cycle Control input rose" );
  }
  else if( CycleControlPressed )
  {
    Serial.println( "Cycle Control Pressed" );
  }

  if( FireHalfTriggerBounce.fell() )
  {
    Serial.println( "Front Trigger input fall" );
  }
  else if( FireHalfTriggerBounce.rose() )
  {
    Serial.println( "Front Trigger input rose" );
  }
  else if( FireHalfTriggerPressed )
  {
    Serial.println( "Front Trigger Pressed" );
  }

  if( FireFullTriggerBounce.fell() )
  {
    Serial.println( "Rear Trigger input fall" );
  }
  else if( FireFullTriggerBounce.rose() )
  {
    Serial.println( "Rear Trigger input rose" );
  }
  else if( FireFullTriggerPressed )
  {
    Serial.println( "Rear Trigger Pressed" );
  }
}
