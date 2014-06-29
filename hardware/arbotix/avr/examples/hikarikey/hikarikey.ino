/* ========================================================================== */
/*                          Commander/HikariKey/ServoControl
/*  This code is a good starting place for using the
/*  keyboard of a computer and the commander to control servos using the arbotix.
/*  Send data in the form of # (servo ID 2 digits) , (goal position 4 digits) CR
/*  You can also send it letters from the keyboard and have it play
/*  sequences based on which letters you send it.
/*  If you use this you want to change the "hikarikey04b.h to whatever
/*  your poses are called.  You also want to change the if-else's to
/*  things that make sense to you.
/*
/*  This code was written by Darkback2 with more help than not by
/*  Chris Novak
/*  lnxfergy (Michael Ferguson, inventor of the arbotix)
/*  Upgrayd
/*  Jess
/*  tician
/*  and the rest of the Trossen Robotics Forum.
/* ========================================================================== */

#include <ax12.h>
#include <SharpIR.h>
#include <BioloidController.h>
#include <Commander.h>
#include "Hikarikey04b.h"  // exported from PyPose
BioloidController bioloid = BioloidController(1000000);
Commander command = Commander();
// add sharp sensor
SharpIR myIR = SharpIR(GP2Y0A02YK,0);  // Really pin 0?  Is that not an LED output?
//SharpIR face01 = SharpIR(GP2Y0A02YK,3);
//SharpIR face02 = SharpIR(GP2Y0A02YK,4);

// make variables
int mode;

int panh;
int panadd;
int pany;
int panyadd;

int armx;
int armxadd;
int army;
int armyadd;
int armz;
int armzadd;
int gripper;
int gripperadd;

int irValue;
int faceDown;
int faceUp;
int screwUp;
int gripperTemp;


#define checkFaceDown()	analogRead(3)
#define checkFaceUp()	analogRead(4)
void updateSensors()
{
// read the IR sensor
    irValue = myIR.getData();
// read the tilt sensor (analog?)
    faceDown = checkFaceDown();
// read the other tilt sensor (analog?)
    faceUp = checkFaceUp();
// read the temp of the gripper servo
    gripperTemp = GetTemperature(28);
}
void printSensors()
{
    updateSensors();
// print the IR sensor
    Serial.println(irValue);
// print the tilt sensor
    Serial.println(faceDown);
// print the other tilt sensor
    Serial.println(faceUp);
// print the temp of the gripper servo
    Serial.println(gripperTemp);
}

void setLEDs(int num)  // Sets LED states to 'num'
{
	// Set LEDs as if they were bits in a 3-bit number
    digitalWrite(0, (num&(1<<0)));
    digitalWrite(1, (num&(1<<1)));
    digitalWrite(2, (num&(1<<2)));
}
void flashLEDs(int num)  // Flashes and finally sets LED states to 'num'
{
    setLEDs(0);
    delay(50);
    setLEDs(num);
    delay(50);
    setLEDs(0);
    delay(50);
    setLEDs(num);
}
void flashLEDs()  // Flashes LEDs using current state (does not modify)
{
    int currentState = 0;
    currentState += (digitalRead(0)<<0);
    currentState += (digitalRead(1)<<1);
    currentState += (digitalRead(2)<<2);

    flashLEDs(currentState);
}

void makeActuatorSoft(int servoNum)  // Drop torque and increase compliance
{
// decrease the torque limit
    ax12SetRegister2(servoNum, AX_TORQUE_LIMIT_L, 200);

// increase the compliance slopes and margins
    ax12SetRegister(servoNum, AX_CW_COMPLIANCE_MARGIN, 30);
    ax12SetRegister(servoNum, AX_CCW_COMPLIANCE_MARGIN, 30);
    ax12SetRegister(servoNum, AX_CW_COMPLIANCE_SLOPE, 128);
    ax12SetRegister(servoNum, AX_CCW_COMPLIANCE_SLOPE, 128);
}
void makeActuatorHard(int servoNum)  // Restore torque and compliance to default
{
// increase the torque limit
    ax12SetRegister2(servoNum, AX_TORQUE_LIMIT_L, 1023);
        // assumes AX_MAX_TORQUE_L/H is 1023

// restore the compliance slopes and margins to defaults
    ax12SetRegister(servoNum, AX_CW_COMPLIANCE_MARGIN, 1);
    ax12SetRegister(servoNum, AX_CCW_COMPLIANCE_MARGIN, 1);
    ax12SetRegister(servoNum, AX_CW_COMPLIANCE_SLOPE, 32);
    ax12SetRegister(servoNum, AX_CCW_COMPLIANCE_SLOPE, 32);
}

//----------------------Start Void setup----------------------
void setup(){
    //open the serial port
    command.begin(38400);

    delay(2000); // recommended pause

	// Set 0, 1, and 2 as outputs for LEDs
    pinMode (0,OUTPUT);
    pinMode (1,OUTPUT);
    pinMode (2,OUTPUT);
	// Set 3 and 4 as inputs for tilt sensors
    pinMode(3, INPUT);
    pinMode(4, INPUT);


    // set defaults of global variables and take initial readings
    mode = 0;

    panh = 512;
    panadd = 0;
    pany = 512;
    panyadd = 0;

    //These are the arm movement variables
    armx = 512;
    armxadd = 0;
    army = 512;
    armyadd = 0;
    armz = 512;
    armzadd = 0;
    gripper = 712;
    gripperadd = 0;

    updateSensors();

//--------- arbotix only: scan for RX servos to operate with RX-bridge ---------
    delay(1000);
    commsTypeScan();

//--------------------crouch down slowly----------------------------------------
    bioloid.loadPose(rest01);
    bioloid.readPose();
    bioloid.interpolateSetup(2000);
    while(bioloid.interpolating > 0){
        bioloid.interpolateStep();
        delay(3);
    }

//---------------set the compliance of all leg servos---------------------------
    for (int i = 1; i < 16; i++) {
        ax12SetRegister(i, AX_CW_COMPLIANCE_MARGIN, 3);
        delay(3);
        ax12SetRegister(i, AX_CCW_COMPLIANCE_MARGIN, 3);
        delay(3);
    }
    for (int i = 17; i < 24; i++) {
        ax12SetRegister(i, AX_CW_COMPLIANCE_MARGIN, 30);
        delay(3);
        ax12SetRegister(i, AX_CCW_COMPLIANCE_MARGIN, 30);
        delay(3);
    }
//-------------------end new stuff from jess------------------------------------
}
//-------------------end setup loop --------------------------------------------

//--------------------start void loop-------------------------------------------
void loop(){

    updateSensors();
    // you keep calling/printing faceDown, faceUp, and others variables
    //  without updating them anywhere in the loop!

//******************************************************************************
//******************************************************************************
// keyboard mode
    if(mode == 5){
        // Set LEDs to display binary '5'
        flashLEDs(5);

//--------------------takes commands from the keyboard--------------------------
// Servo command format: #nnpppp
//  (hash/pound delimiter)(two digit servo number)(four digit goal position)
  // input must conform exactly to this format as parsing code is very fragile
// Other commands: c
//  (one ASCII character)

        char cmd[8];
        int startByte = 0;
// if there are bytes waiting on the serial port
        if(Serial.available() > 0) {
            // read the incoming byte
            startByte = Serial.read();
            // toggle LED so we know johnny5 is alive
//            digitalWrite(0,HIGH-digitalRead(0));
            flashLEDs();
        } // end Serial.available()

        if(startByte == '#') {
            // get the command from the serial port
            while(Serial.available() < 8); // wait for data to arrive, do nothing
            for(int i=0;i<8;i++){
                cmd[i] = Serial.read();
            } //this creates an array that takes in the data from the serial port
            // convert command to two numbers
            int servoNum = 0;
            int servoPos = 0;
            servoNum = (cmd[0]-48)*10 + (cmd[1]-48);
            //builds the 2 digit servo Id number
            servoPos = (cmd[3]-48)*1000 + (cmd[4]-48)*100 + (cmd[5]-48)*10 + (cmd[6]-48);
            //builds the 4 digit goal position
            SetPosition(servoNum,servoPos);
            //sets servo position using 2 numbers one for servo, other for position.
            // debug
            Serial.print("cmd array:");
            for(int i=0;i<8;i++){
                Serial.print(cmd[i]);
            }
            Serial.println("----");
            Serial.print("servoNum:");
            Serial.println(servoNum);
            Serial.print("servoPos:");
            Serial.println(servoPos);
        }// end if(startByte == '#')

        else {
            //process commands other than servo positions
            // only process if we aren't currently doing a sequence
            if(bioloid.playing == 0){
                if(startByte == 'r'){ //if I press r relax the legs
                    Relax(5);
                    Relax(7);
                    Relax(9);
                    Relax(11);
                    Relax(6);
                    Relax(8);
                    Relax(10);
                    Relax(12);
                }
                //commands from keyboard
                else if(startByte == 'w'){ //if I press w walk forward
                    bioloid.playSeq(simplewalk01);
                }else if(startByte == 'W'){ //if I press W walk forward slowly
                    bioloid.playSeq(Msimplewalk01tr);
                }else if(startByte == 'X'){ //if I press X walk backword slowly
                    bioloid.playSeq(Rest01);
                }else if(startByte == 's'){ //if I press s go to base fire pose
                    bioloid.playSeq(Mechfire);
                }else if(startByte == 'S'){ //if I press S big jump
                    bioloid.playSeq(Jump01);
                }else if(startByte == 'x'){ //if I press x defensive pose
                    bioloid.playSeq(Msimpleback02);
                }else if(startByte == 'g'){ //if I press g get up facedown
                    bioloid.playSeq(Facedown01);
                }else if(startByte == 'G'){ //if I press G get up facedown
                    bioloid.playSeq(Facedown02);
                }else if(startByte == 'b'){ //if I press b get up from back
                    bioloid.playSeq(Backup01);
                }else if(startByte == 'B'){ //if I press B finish getting up
                    bioloid.playSeq(Backup02);
                }else if(startByte == 'q'){ //if I press q turn left
                    bioloid.playSeq(SWmTurnLeft01);
                }else if(startByte == 'a'){ //if I press a sidestep left
                    bioloid.playSeq(MSl01);
                }else if(startByte == 'e'){ //if I press e turn left
                    bioloid.playSeq(SWmTurnRight01);
                }else if(startByte == 'd'){ //if I press d sidestep right
                    bioloid.playSeq(MSr01);
                }else if(startByte == 'j'){ //if I press j attack left
                    bioloid.playSeq(LeftAttack01);
                }else if(startByte == 'l'){ //if I press l attack right
                    bioloid.playSeq(RightAttack01);
                }else if(startByte == 'k'){ //if I press k right push front
                    bioloid.playSeq(AttackFront01);
                }else if(startByte == 'K'){ //if I press k Taunt
                    bioloid.playSeq(Taunt01);
                }else if(startByte == 'i'){ //if I press i gunplay
                    bioloid.playSeq(Msimplewalk01au);
                }else if(startByte == 'u'){ //if I press u Bad Robot
                    bioloid.playSeq(BadRobot01);
                }else if(startByte == 'o'){ //if I press o good robot
                    bioloid.playSeq(GoodRobot01);
                }else if(startByte == 'z'){ //if I press z rest
                    bioloid.playSeq(Defence01);
                }else if(startByte == 'n'){ //if I press n get into dropped push mode
                    bioloid.playSeq(DownPush01);
                }else if(startByte == 'N'){ //if I press n Downpush 02
                    bioloid.playSeq(DownPush02);
                }else if(startByte == 'm'){ //if I press n rest attack
                    bioloid.playSeq(RestAttack01);
                }else if(startByte == '<'){ //if I press Left Kick
                    bioloid.playSeq(LeftKick01);
                }else if(startByte == '>'){ //if I press Right Kick
                    bioloid.playSeq(RightKick01);
                }else if(startByte == '"'){ //if I press ` set mode to 0
                    mode += 1;
                }
                //special commands from keyboard that sets specific servos to shooting.

                // Cut down on code repetition.  copy-pasta breeds errors.
                printSensors();  // Calls updateSensors() again, internally

                //end of keyboard stuff.
            }
        }
    }

//******************************************************************************
//******************************************************************************
    if (mode != 5){
        // check for new messages from Commander
        if(command.ReadMsgs() > 0){
            // toggle LED so we know johnny5 is alive
//            digitalWrite(0,HIGH-digitalRead(0));
            // exact mode unknown, set all 3 LEDs to identify it as commander control
            flashLEDs(7);

            // only process commands if we aren't currently doing a sequence
            if(bioloid.playing == 0){       
//-----------------change modes by pressing button R1---------------------------
                if(command.buttons&BUT_R1) {
                // slowjump as visual indicator
                    mode += 1;
//------------------------resets mode to 0--------------------------------------
                    if(mode >= 6){
                        mode = 0;
                        bioloid.playSeq(Defence01);
                    }
                    bioloid.playSeq(slowjump);
                }

//******************************************************************************
//******************************************************************************
//---------------------------Initial mode-(kung fu)-----------------------------
                if(mode==0){
                    // Set LEDs to display binary '6' since '0' is not helpful
                    flashLEDs(6);

                    //use Button1 to change mode.
                    if(command.walkV > 80){
                        // walk forward
                        bioloid.playSeq(Msimplewalk01tr);
//                        Serial.println(faceDown, DEC);
//                        Serial.println(faceUp, DEC);
                    }else if(command.walkV < -50){
                        // walk defence pose
                        bioloid.playSeq(Msimpleback02);
//                        Serial.println(faceDown, DEC);
//                        Serial.println(faceUp, DEC);
                    }else if(command.walkH > 50){
                        // side step right
                        bioloid.playSeq(MSr01);
//                        Serial.println(faceDown, DEC);
//                        Serial.println(faceUp, DEC);
                    }else if(command.walkH < -50){
                        // side step left
                        bioloid.playSeq(MSl01);
//                        Serial.println(faceDown, DEC);
//                        Serial.println(faceUp, DEC);
                    }else if(command.lookH > 50){
                        // new turn left using twisty foot thing
                        bioloid.playSeq(SWmTurnLeft01);
//                        Serial.println(faceDown, DEC);
//                        Serial.println(faceUp, DEC);
                    }else if(command.lookH < -50){
                        // turn right
                        bioloid.playSeq(SWmTurnRight01);
//                        Serial.println(faceDown, DEC);
//                        Serial.println(faceUp, DEC);
                    }else if(command.lookV < -50){
                        // crouch
                        bioloid.playSeq(Defence01);
//                        Serial.println(faceDown, DEC);
//                        Serial.println(faceUp, DEC);
                    }
//--------------------Here go the button commands-------------------------------
/// Kinda guessing these should be mostly '}else if() {' ?
/// Do we really want all of these possibly playing in this sequence from a
///  single Commander packet?
                    if(command.buttons&BUT_R2) {
                        //paused side attack
                        bioloid.playSeq(FrontAttack01);
                    }if(command.buttons&BUT_R3) {
                        //return to neutral position
                        bioloid.playSeq(UpSlow01);
                        SetPosition(28, 712);
                    }if(command.buttons&BUT_L6) {
                        //attackfront
                        bioloid.playSeq(AttackFront01);
                    }if(command.buttons&BUT_L5) {
                        //drop down and push
                        bioloid.playSeq(DownPush02);
                    }if(command.buttons&BUT_L4) {
                        //reach back...back push?
                        bioloid.playSeq(Backup01);
                    }if(command.buttons&BUT_RT) {
                        //Right attack
                        bioloid.playSeq(RightAttack01);
                    }if(command.buttons&BUT_LT) {
                        //Left Attack
                        SetPosition(28, 524);
                        delay(200);
                        bioloid.playSeq(LeftAttack01);
                        delay(400);
                        SetPosition(28, 712);
                    }

                    // Is anybody actually listening here? A physical Commander
                    //  with default firmware does not listen to anything sent
                    //  to it over Xbee.  Maybe a fit-pc2i listening in or emulating?
                    printSensors();
                }
//-----------------------------end kungfu mode----------------------------------


//******************************************************************************
//******************************************************************************
//------------------------------get up mode-------------------------------------
                if(mode == 1){
                    // Set LEDs to display binary '1'
                    flashLEDs(1);

                    //get up mode
                    if(command.walkV > 80){
                        // walk forward
                        bioloid.playSeq(simplewalk01);
//                        Serial.println(faceDown, DEC);
//                        Serial.println(faceUp, DEC);
                    }else if(command.walkV < -50){
                        // walk backwards
                        bioloid.playSeq(simpleback02);
//                        Serial.println(faceDown, DEC);
//                        Serial.println(faceUp, DEC);
                    }else if(command.walkH > 50){
                        // side step right
                        bioloid.playSeq(Sr01);
//                        Serial.println(faceDown, DEC);
//                        Serial.println(faceUp, DEC);
                    }else if(command.walkH < -50){
                        // side step left
                        bioloid.playSeq(Sl01);
//                        Serial.println(faceDown, DEC);
//                        Serial.println(faceUp, DEC);
                    }else if(command.lookH > 50){
                        // turn left
                        bioloid.playSeq(SWmTurnLeft01);
//                        Serial.println(faceDown, DEC);
//                        Serial.println(faceUp, DEC);
                    }else if(command.lookH < -50){
                        // turn right
                        bioloid.playSeq(SWmTurnRight01);
//                        Serial.println(faceDown, DEC);
//                        Serial.println(faceUp, DEC);
                    }else if(command.lookV < -50){
                        // crouch
                        bioloid.playSeq(Defence01);
//                        Serial.println(faceDown, DEC);
//                        Serial.println(faceUp, DEC);
                    }
//----------------------Here go the button commands-----------------------------
/// Kinda guessing these should be mostly '}else if() {' ?
/// Do we really want all of these possibly playing in this sequence from a
///  single Commander packet?
                    if(command.buttons&BUT_R2) {
                        //Start getting up from face down
                        bioloid.playSeq(Facedown01);
                    }if(command.buttons&BUT_R3) {
                        //Finish getting up from face down
                        bioloid.playSeq(Facedown02);
                    }if(command.buttons&BUT_L5) {
                        //Start getting up from back down
                        bioloid.playSeq(Backup01);
                    }if(command.buttons&BUT_L4) {
                        //Finish getting up from back down
                        bioloid.playSeq(Backup02);
                    }if(command.buttons&BUT_L6) {
                        //Neutral position
                        mode -= 1;
                    }if(command.buttons&BUT_RT) {
                        //Right attack
                        bioloid.playSeq(RightAttack01);
                    }if(command.buttons&BUT_LT) {
                        //Left Attack
                        SetPosition(28, 524);
                        bioloid.playSeq(LeftAttack01);
                        delay (400);
                        SetPosition(28, 712);
                    }

                    // Is anybody actually listening here? A physical Commander
                    //  with default firmware does not listen to anything sent
                    //  to it over Xbee.  Maybe a fit-pc2i listening in or emulating?
                    printSensors();
                }
//----------------------------end get up mode-----------------------------------


//******************************************************************************
//******************************************************************************
//------------------------cheesy moves mode-(soccer)----------------------------
                if(mode == 2){
                    // Set LEDs to display binary '2'
                    flashLEDs(2);

                    //special moves
                    if(command.walkV > 80){
                        // walk forward
                        bioloid.playSeq(simplewalk01);
                        SetPosition(28, 712);
//                        Serial.println(faceDown, DEC);
//                        Serial.println(faceUp, DEC);
                    }else if(command.walkV < -50){
                        // walk backwards
                        bioloid.playSeq(Msimpleback02);
//                        Serial.println(faceDown, DEC);
//                        Serial.println(faceUp, DEC);
                    }else if(command.walkH > 50){
                        // side step right
                        bioloid.playSeq(MSr01);
//                        Serial.println(faceDown, DEC);
//                        Serial.println(faceUp, DEC);
                    }else if(command.walkH < -50){
                        // side step left
                        bioloid.playSeq(MSl01);
//                        Serial.println(faceDown, DEC);
//                        Serial.println(faceUp, DEC);
                    }else if(command.lookH > 50){
                        // turn left
                        bioloid.playSeq(SWmTurnLeft01);
//                        Serial.println(faceDown, DEC);
//                        Serial.println(faceUp, DEC);
                    }else if(command.lookH < -50){
                        // turn right
                        bioloid.playSeq(SWmTurnRight01);
//                        Serial.println(faceDown, DEC);
//                        Serial.println(faceUp, DEC);
                    }else if(command.lookV < -50){
                        // crouch
                        bioloid.playSeq(Defence01);
//                        Serial.println(faceDown, DEC);
//                        Serial.println(faceUp, DEC);
                    }
//--------------------Here go the button commands-------------------------------
/// Kinda guessing these should be mostly '}else if() {' ?
/// Do we really want all of these possibly playing in this sequence from a
///  single Commander packet?
                    if(command.buttons&BUT_R2) {
                        //jump
                        bioloid.playSeq(Jump01);
                    }if(command.buttons&BUT_R3) {
                        //gunplay
                        bioloid.playSeq(Taunt01);
                    }if(command.buttons&BUT_L6) {
                        //Collapse
                        SetPosition(28, 512);
                        delay (200);
                        bioloid.playSeq(Cruel01);
                        delay (800);
                        SetPosition(28, 512);
                    }if(command.buttons&BUT_L5) {
                        //Finish getting up from face down
                        SetPosition(28, 524);
                        SetPosition(29, 612);
                        delay (200);
                        bioloid.playSeq(GoodRobot01);
                        delay (800);
                        SetPosition(28, 712);
                        SetPosition(29, 512);
                    }if(command.buttons&BUT_L4) {
                        //Start getting up
                        SetPosition(29, 412);
                        delay (200);
                        bioloid.playSeq(BadRobot01);
                        delay (700);
                        SetPosition(29, 512);
                    }if(command.buttons&BUT_RT) {
                        //right kick
                        bioloid.playSeq(RightKick01);
                    }if(command.buttons&BUT_LT) {
                        //Left kick
                        bioloid.playSeq(LeftKick01);
                    }

                    // Is anybody actually listening here? A physical Commander
                    //  with default firmware does not listen to anything sent
                    //  to it over Xbee.  Maybe a fit-pc2i listening in or emulating?
                    printSensors();
                }
//--------------------------end cheesy moves mode ------------------------------


//******************************************************************************
//******************************************************************************
//-------------------------mech warfare mode -----------------------------------
                if(mode == 3){
                    // Set LEDs to display binary '3'
                    flashLEDs(3);

                    //int panh = 512;
                    panadd = (-float(command.lookH)/20);
                    panyadd = (5);
                //mech warfare mode
                    if(command.walkV > 80){
                        // walk forward
                        bioloid.playSeq(Msimplewalk01);
//                        Serial.println(faceDown, DEC);
//                        Serial.println(faceUp, DEC);
                    }else if(command.walkV < -50){
                        // rest pose
                        bioloid.playSeq(Rest01);
//                        Serial.println(faceDown, DEC);
//                        Serial.println(faceUp, DEC);
                    }else if(command.walkH > 50){
                        // turn right
                        bioloid.playSeq(SWmTurnLeft01);
//                        Serial.println(faceDown, DEC);
//                        Serial.println(faceUp, DEC);
                    }else if(command.walkH < -50){
                        // turn right
                        bioloid.playSeq(SWmTurnRight01);
//                        Serial.println(faceDown, DEC);
//                        Serial.println(faceUp, DEC);
                    }

//----------------------------Head psn code ------------------------------------
                    else if (command.lookH > 25){
                        panh +=  (2 * panadd);
                        SetPosition(27, (((panh/1024.00) * 512.00)+256));
                    }else if (command.lookH < -25){
                        panh += (2 * panadd);
                        SetPosition(27, (((panh/1024.00) * 512.00)+256));
                    }
//------------------------------Gun/head Tilt code ----------------------------------
                    else if (command.lookV > 25){
                        pany +=  (2 * panyadd);
                        SetPosition(18, (((((pany/1024.00) * 312.00)+ 312)- 1024) * -1));
                        SetPosition(22, 512);
                        //trying to get servo 18 to move in the opposite direction
                        SetPosition(17, (((pany/1024.00) * 312.00)+ 312));
                        SetPosition(21, 512);
                        SetPosition(29, (((pany/1024.00) * 312.00)+ 312));
                    }else if (command.lookV < -25){
                        pany -= (2 * panyadd);
                        SetPosition(17, (((pany/ 1024.00) * 312.00)+ 312));
                        SetPosition(18, (((((pany/1024.00) * 312.00)+ 312)- 1024) * -1));
                        SetPosition(29, (((pany/1024.00) * 312.00)+ 312));
                    }
//------------------------Here go the button commands---------------------------
/// Kinda guessing these should be mostly '}else if() {' ?
/// Do we really want all of these possibly playing in this sequence from a
///  single Commander packet?
                    if(command.buttons&BUT_RT) {
                        //shoot right gun
                        ax12SetRegister2(31, AX_GOAL_SPEED_L,312);
                        delay (600);
                        ax12SetRegister2(31,AX_GOAL_SPEED_L,0);
                    }if(command.buttons&BUT_LT) {
                        //shoot left gun
                        ax12SetRegister2(30, AX_GOAL_SPEED_L,312);
                        delay (600);
                        ax12SetRegister2(30, AX_GOAL_SPEED_L,0);
                    }if(command.buttons&BUT_R3) {
                        //shoot right cannon
                        ax12SetRegister2(40, AX_GOAL_SPEED_L,362);
                        delay (1000);
                        ax12SetRegister2(40, AX_GOAL_SPEED_L,0);
                    }if(command.buttons&BUT_L4) {
                        //shoot left cannon
                        ax12SetRegister2(41, AX_GOAL_SPEED_L,362);
                        delay (1000);
                        ax12SetRegister2(41, AX_GOAL_SPEED_L,0);
                    }if(command.buttons&BUT_R2) {
                        //rest01
                        SetPosition(28, 524);
                        delay (400);
                        bioloid.playSeq(Cruel01);
                        delay (1000);
                        SetPosition(28, 712);
                    }if(command.buttons&BUT_L5) {
                        //Side step left
                        bioloid.playSeq(Mechfire);
                    }if(command.buttons&BUT_L6) {
                        //reset to mode 0
                        mode += 3;
                    }

                    // Is anybody actually listening here? A physical Commander
                    //  with default firmware does not listen to anything sent
                    //  to it over Xbee.  Maybe a fit-pc2i listening in or emulating?
                    printSensors();
                }
//--------------------------end mech warfare mode ------------------------------


//******************************************************************************
//******************************************************************************
//------------------------- arm control mode -----------------------------------
                if(mode == 4){
                    // Set LEDs to display binary '4'
                    flashLEDs(4);

                  //this sets the initial value of the variables
                    panadd = (-float(command.lookH)/20);
                    armxadd = (-float(command.lookH)/20);
                    panyadd = (5);
                    armyadd = (5);
                    armzadd = (5);
                    gripperadd = (5);
                    SetPosition(17, 262);
                    SetPosition(19, 262);
                    SetPosition(21, 762);
                    SetPosition(23, 562);
                  // This should drastically reduce the torque for the gripper servo
                  //  It should work now that it is correct command and going 
                //  to the correct register table address
                //    ax12SetRegister2(28, AX_TORQUE_LIMIT_L, 200);
                  // This sets the compliance really high for the gripper servo
                  //  Only sets one direction
                //    ax12SetRegister(28, 0x1B, 100);
                    makeActuatorSoft(28);

                    if(command.walkV > 80){
                        // walk forward
                        bioloid.playSeq(Msimplewalk01);
//                        Serial.println(faceDown, DEC);
//                        Serial.println(faceUp, DEC);
                    }else if(command.walkV < -50){
                        // rest pose
                        bioloid.playSeq(Defence01);
//                        Serial.println(faceDown, DEC);
//                        Serial.println(faceUp, DEC);
                    }else if(command.walkH > 50){
                        // turn right
                        bioloid.playSeq(SWmTurnLeft01);
//                        Serial.println(faceDown, DEC);
//                        Serial.println(faceUp, DEC);
                    }else if(command.walkH < -50){
                        // turn right
                        bioloid.playSeq(SWmTurnRight01);
//                        Serial.println(faceDown, DEC);
//                        Serial.println(faceUp, DEC);

//----------------------------ArmXCode ------------------------------------
                    }else if (command.lookH > 25){
                        armx +=  (2 * armxadd);
                        SetPosition(27, (((armx/1024.00) * 400.00)+312));
                    }else if (command.lookH < -25){
                        armx += (2 * armxadd);
                        SetPosition(27, (((armx/1024.00) * 400.00)+312));
                    }
//------------------------------armYcode ----------------------------------
                    else if (command.lookV < -25){
                      //move the gripper forward
                        pany -= (1.75 *panyadd);
                        army +=  (2 * armyadd);
                        SetPosition(18, ((((army/1024.00) * 612.00)+ 212)));
                        SetPosition(22, (((((army/1024.00) * 612.00)+ 212.00) - 1024.00) * -1));
                        SetPosition(29, (((pany/1024.00) * 262.00)+ 412));
                    }else if (command.lookV > 25){
                      //move the gripper backwards
                        pany += (1.75 * panyadd);
                        army -= (2 * armyadd);
                        SetPosition(18, ((((army/1024.00) * 612.00)+ 212)));
                        SetPosition(22, (((((army/1024.00) * 612.00)+ 212.00) - 1024.00) * -1));
                        SetPosition(29, (((pany/1024.00) * 262.00)+ 412));
                    }
//------------------------Here go the button commands---------------------------
/// Kinda guessing these should be mostly '}else if() {' ?
/// Do we really want all of these possibly playing in this sequence from a
///  single Commander packet?
                    if(command.buttons&BUT_RT) {
                       //close the gripper
                       //this needs a bit of work.
                       //the current goal is to monitor the temperature upon closing.
                       //if the temperature rises the gripper should relax a bit.
                       //setting the compliance relatively high and limiting the torque
                       //of the gripper seams to have helped.
                       gripper += (2 * gripperadd);
                       SetPosition(28, ((((gripper/1024.00) * 262.00)+ 512)));
          //             ax12GetRegister (28,0x2B,1);
          // What we doing getting, but not storing/printing/using?
                    }if(command.buttons&BUT_LT) {
                        //open the gripper
                        gripper -= (2 * gripperadd);
                        SetPosition(28, ((((gripper/1024.00) * 262.00)+ 512)));
          //              ax12GetRegister (28,0x2B,1);
          // What we doing getting, but not storing/printing/using?
                    }if(command.buttons&BUT_R3) {
                         //lower the arm
                        armz -= (2 * armzadd);
                        pany += (2 *panyadd);
                        SetPosition(18, ((((armz/1024.00) * 612.00)+ 212)));
                        SetPosition(29, (((pany/1024.00) * 262.00)+ 412));
                    }if(command.buttons&BUT_L4) {
                        //raise the arm
                        armz += (2 * armzadd);
                        pany -= (2 *panyadd);
                        SetPosition(18, ((((armz/1024.00) * 612.00)+ 212)));
                        SetPosition(29, (((pany/1024.00) * 262.00)+ 412));
                    }if(command.buttons&BUT_R2) {
                        bioloid.playSeq(Rest01);
                    }if(command.buttons&BUT_L5) {
                        //stand up
                        bioloid.playSeq(UpSlow01);
                    }if(command.buttons&BUT_L6) {
                        //reset to mode 0
                        mode += 4;
                    }
                    
                    // Is anybody actually listening here? A physical Commander
                    //  with default firmware does not listen to anything sent
                    //  to it over Xbee.  Maybe a fit-pc2i listening in or emulating?
                    printSensors();  // calls updateSensors() again, internally

                    // Temperature Safety Checks/Corrections on gripper servo
                    // updateSensors();
                      // already called in printSensors() and at start of loop()
                    if (gripperTemp >= 50)
                    {
                        gripper -= (2 * gripperadd);
                        SetPosition(28, ((((gripper/1024.00) * 262.00)+ 512)));
                    }
                }
            }
        }
    }

    if(bioloid.playing > 0){
        bioloid.play(); // continue sequence
    }
} //end program
