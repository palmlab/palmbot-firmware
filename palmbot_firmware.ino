#include <pbPort.h>
#include <pbShield.h>
#include <pbLineFollower.h>
#include <pbUltrasonic.h>
#include <pbServoMotor.h>
#include <CmdHandler.h>

#define FW_VERSION 1.51

pbShield palmbot;
//----------------- Sensors ---------------
pbUltrasonic ultrasonicSensor = pbUltrasonic(palmbot.Port3);
pbLineFollower lineFollower = pbLineFollower(palmbot.Port1);


pbServoMotor servoMotor;

//--------------- Variables ---------------
CmdHandler serialHandler;
pbPort ports[] = {palmbot.Port0, palmbot.Port1, palmbot.Port2, palmbot.Port3, palmbot.Port4 }; // Port0 is internally assiugned to on-board servo port

// Analog A5 is no longer connecet to Bluteooth_EN,
// but it enables the RX of the bluteetoh module through
// internal circuit on the board.
int analogs[8]={A0,A1,A2,A3,A4,A5,A6,A7};



//--------------- Welocme Buzzer song -----
void buzzerWelcome(){
  int melody[] = {
    262, 196, 196, 220, 196, 0, 247, 262, 262
  };

  int noteDurations[] = {
    4, 8, 8, 4, 4, 4, 4, 4, 8
  };

  for(int i=0; i<9; i++){
    int nDuration = 1000 / noteDurations[i];
    palmbot.PlayTone(melody[i], nDuration);

    int pauseNote = nDuration * 1.30;
    delay(pauseNote);
  }
}

//---------------- Handler ----------------
bool OnRunRequest(CmdRequest* req)
{
  switch (req->device)
  {
    case MOTOR_CMD:
      {
	      if(req->motor.port==0) { //
		      // left motor
		      palmbot.LMotor.Run(req->motor.speed>0?FORWARD_DIRECTION:BACKWARD_DIRECTION, abs(req->motor.speed));
	      } else if (req->motor.port==1) {
		      // right motor
		      palmbot.RMotor.Run(req->motor.speed>0?FORWARD_DIRECTION:BACKWARD_DIRECTION, abs(req->motor.speed));
	      }
      }
      break;

     case JOYSTICK_CMD:
      {
        palmbot.LMotor.Run(req->joystick.lspeed>0?FORWARD_DIRECTION:BACKWARD_DIRECTION, abs(req->joystick.lspeed));
        palmbot.RMotor.Run(req->joystick.rspeed>0?FORWARD_DIRECTION:BACKWARD_DIRECTION, abs(req->joystick.rspeed));
      }
      break;

     case SERVO_CMD:
      {
       // servo port expected to 1, 2, 3, 4 (regular ports on board with slot 1 or 2) OR 0 (dedicated port on board with slot1=slot2)
       servoMotor.Set(ports[req->servo.port], req->servo.slot, req->servo.angle);
       palmbot.Buzzer.Tone(150, 10);
      }
      break;    

    case LED_CMD:
      palmbot.TurnLED(req->led.on_off);
      break;

    case LIGHT_CMD:
      //TODO
      break;

    case DIGITAL_CMD:
      {
        pinMode(req->digital.pin,OUTPUT);
        digitalWrite(req->digital.pin, req->digital.value);
      }
      break;

    case PWM_CMD:
      {
        pinMode(req->pwm.pin,OUTPUT);
        analogWrite(req->pwm.pin, req->pwm.value);
      }
      break;

    case TONE_CMD:
      {
        palmbot.Buzzer.Tone(req->tone.freq, req->tone.duration);
      }
      break;
      
  }
  return true;
}

bool OnGetRequest(CmdRequest* req, CmdResponse* resp)
{
  switch (req->device)
  {
    case BUTTON_CMD:
      {
        resp->_float = req->button.status ^ palmbot.ButtonPressed();
        resp->type = FLOAT_DATATYPE;
      }
      break;
      
    case ULTRASONIC_CMD:
      {
        pbUltrasonic us = pbUltrasonic(ports[req->ultrasonic.port]);
        resp->_float = us.Measure();
        resp->type = FLOAT_DATATYPE;
      }
      break;

    case TEMPERATURE_CMD:
      //TODO
      break;

    case  LIGHT_CMD:
    {
      
      resp->_float = palmbot.LightSense(req->light.port);
      resp->type = FLOAT_DATATYPE;
    }
    break;
    
   case  SOUND_CMD:
   case  POTENTIONMETER_CMD:
    {
      resp->_float = ports[req->light.port].Pin2.AnalogeRead();
      resp->type = FLOAT_DATATYPE;
    }
    break;

   case JOYSTICK_CMD:
    //TODO
    break;

   case LINEFOLLOWER_CMD:
    {
     pbLineFollower lf = pbLineFollower(ports[req->linefollower.port]); 
     resp->_float = lf.ReadSensor1()*2+lf.ReadSensor2();
     resp->type = FLOAT_DATATYPE;
    }
    break;

   case DIGITAL_CMD:
    {
     pinMode(req->digital.pin,INPUT);
     resp->_float = digitalRead(req->digital.pin);
     resp->type = FLOAT_DATATYPE;
    }
    break;

   case ANALOG_CMD:
    {
     pinMode(analogs[req->analog.pin],INPUT);
     resp->_float = analogRead(analogs[req->analog.pin]);
     resp->type = FLOAT_DATATYPE;
    }
    break;
    
  }
  return true;
}

//-------------- Main Code --------------
 
void setup() { 

  // Enable Bluetooth through A5 n internal cicuit
  pinMode(A5, OUTPUT);
  digitalWrite(A5, HIGH);
  
  // put your setup code here, to run once:
  Serial.begin(115200);
  serialHandler.OnRun = OnRunRequest;
  serialHandler.OnGet = OnGetRequest;

  servoMotor.Set(palmbot.Port4, 0, 90);
  delay(500); // delay required
  servoMotor.Set(palmbot.Port0, 0, 90);

  buzzerWelcome();

  Serial.print("PalmBot Factory Firmware version ");
  Serial.print(FW_VERSION);
  Serial.write("\n");
}

uint8_t x = 0;
void loop() {
 serialHandler.Handle();

}
