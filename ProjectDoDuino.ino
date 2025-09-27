/*********************************************************************************************************
------------------------------------Important Information-------------------------------------------------
**  ProtocolProcess() is used to have the Dobot execute commands. (23:39 - 27/09/2025)
**
**
**
**
**
**
----------------------------------------------------------------------------------------------------------
*********************************************************************************************************/

#include "stdio.h"
#include "Protocol.h"
#include "command.h"
#include "FlexiTimer2.h"

//Set Serial TX&RX Buffer Size
#define SERIAL_TX_BUFFER_SIZE 64
#define SERIAL_RX_BUFFER_SIZE 256

//#define JOG_STICK 
/*********************************************************************************************************
** Global parameters
*********************************************************************************************************/
EndEffectorParams gEndEffectorParams;

JOGJointParams  gJOGJointParams;
JOGCoordinateParams gJOGCoordinateParams;
JOGCommonParams gJOGCommonParams;
JOGCmd          gJOGCmd;

PTPCoordinateParams gPTPCoordinateParams;
PTPCommonParams gPTPCommonParams;
PTPCmd          gPTPCmd;

uint64_t gQueuedCmdIndex;

/*********************************************************************************************************
** Global variables // made by lieuwe
*********************************************************************************************************/
//start position of the dobot, based on cartesian coordinates
float startX = 0.00;       
float startY = 0.00;       
float startZ = 50.00;       
float startR = 0.00;
//current position of the dobot, updated on chance, based on cartesian coordinates
float currentX = startX;
float currentY = startY;
float currentZ = startZ;
float currentR = startR;
//if a suction cup is installed on the dobot then the variable is true, if not then the variable is false
bool suctionCup = false;
bool suctionCurrentlyOn = false;
/*********************************************************************************************************
** Function name:       setup
** Descriptions:        Initializes Serial
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
** Developer:           Dobot Labs
*********************************************************************************************************/
void setup() {
    Serial.begin(115200);
    Serial1.begin(115200); 
    printf_begin();
    Serial.println(" ");
    Serial.println("===========Serial comminucation established=================");
    Serial.println(" ");
    //Set Timer Interrupt
    FlexiTimer2::set(100,Serialread); 
    FlexiTimer2::start();
}

/*********************************************************************************************************
** Function name:       Serialread
** Descriptions:        import data to rxbuffer
** Input parametersnone:
** Output parameters:   
** Returned value:
** Developer:           Dobot Labs      
*********************************************************************************************************/
void Serialread()
{
  while(Serial1.available()) {
        uint8_t data = Serial1.read();
        if (RingBufferIsFull(&gSerialProtocolHandler.rxRawByteQueue) == false) {
            RingBufferEnqueue(&gSerialProtocolHandler.rxRawByteQueue, &data);
        }
  }
}
/*********************************************************************************************************
** Function name:       Serial_putc
** Descriptions:        Remap Serial to Printf
** Input parametersnone:
** Output parameters:   
** Returned value:  
** Developer:           Dobot Labs    
*********************************************************************************************************/
int Serial_putc( char c, struct __file * )
{
    Serial.write( c );
    return c;
}

/*********************************************************************************************************
** Function name:       printf_begin
** Descriptions:        Initializes Printf
** Input parameters:    
** Output parameters:
** Returned value:      
** Developer:           Dobot Labs
*********************************************************************************************************/
void printf_begin(void)
{
    fdevopen( &Serial_putc, 0 );
}
/*********************************************************************************************************
** Function name:       moveDobotToPos
** Descriptions:        Move the Dobot arm to a set position
** Input parameters:    float x, float y, float z, float r
** Output parameters:   none
** Returned value:      none
** Developer:           Lieuwe Baron
*********************************************************************************************************/
void moveDobotToPos(float x, float y, float z, float r) {
    gPTPCmd.x = x;
    gPTPCmd.y = y;
    gPTPCmd.z = z;
    gPTPCmd.r = r;
    currentX = x;
    currentY = y;
    currentZ = z;
    currentR = r;
    SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);
    ProtocolProcess();
    delay(500);
}
/*********************************************************************************************************
** Function name:       moveDobotByIncrement
** Descriptions:        Move the Dobot arm 
** Input parameters:    float x, float y, float z, float r
** Output parameters:   none
** Returned value:      none
** Developer:           Lieuwe Baron
*********************************************************************************************************/
void moveDobotByIncrement(float x, float y, float z, float r) {
    gPTPCmd.x += x;
    gPTPCmd.y += y;
    gPTPCmd.z += z;
    gPTPCmd.r += r;
    currentX += x;
    currentY += y;
    currentZ += z;
    currentR += r;
    SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);
    ProtocolProcess();
    delay(500);
}
/*********************************************************************************************************
** Function name:       InitRAM
** Descriptions:        Enables/Disables the suction cup of the Dobot arm
** Input parameters:    bool suctionEnable
** Output parameters:   none
** Returned value:      none
** Developer:           Lieuwe Baron
*********************************************************************************************************/
void suctionCupEnable(bool suctionEnable) {
    if(suctionEnable == true & suctionCurrentlyOn == false) {
        SetEndEffectorSuctionCup(true, true, &gQueuedCmdIndex);
        suctionCurrentlyOn = suctionEnable;
    }
    else {
        if (suctionEnable == false & suctionCurrentlyOn == true){
            SetEndEffectorSuctionCup(false, true, &gQueuedCmdIndex);
            suctionCurrentlyOn = suctionEnable;
        }
    }
    ProtocolProcess();
}
/*********************************************************************************************************
** Function name:       InitRAM
** Descriptions:        Initializes a global variable
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
** Developer:           Dobot Labs/Lieuwe Baron
*********************************************************************************************************/
void InitRAM(void)
{
    //Set JOG Model
    gJOGJointParams.velocity[0] = 100;
    gJOGJointParams.velocity[1] = 100;
    gJOGJointParams.velocity[2] = 100;
    gJOGJointParams.velocity[3] = 100;
    gJOGJointParams.acceleration[0] = 80;
    gJOGJointParams.acceleration[1] = 80;
    gJOGJointParams.acceleration[2] = 80;
    gJOGJointParams.acceleration[3] = 80;

    gJOGCoordinateParams.velocity[0] = 100;
    gJOGCoordinateParams.velocity[1] = 100;
    gJOGCoordinateParams.velocity[2] = 100;
    gJOGCoordinateParams.velocity[3] = 100;
    gJOGCoordinateParams.acceleration[0] = 80;
    gJOGCoordinateParams.acceleration[1] = 80;
    gJOGCoordinateParams.acceleration[2] = 80;
    gJOGCoordinateParams.acceleration[3] = 80;

    gJOGCommonParams.velocityRatio = 50;
    gJOGCommonParams.accelerationRatio = 50;
   
    gJOGCmd.cmd = AP_DOWN;
    gJOGCmd.isJoint = JOINT_MODEL;

    

    //Set PTP Model
    gPTPCoordinateParams.xyzVelocity = 100;
    gPTPCoordinateParams.rVelocity = 100;
    gPTPCoordinateParams.xyzAcceleration = 80;
    gPTPCoordinateParams.rAcceleration = 80;

    gPTPCommonParams.velocityRatio = 50;
    gPTPCommonParams.accelerationRatio = 50;

    gPTPCmd.ptpMode = MOVL_XYZ;
    gQueuedCmdIndex = 0;
    moveDobotToPos(startX, startY, startZ, startR);
    ProtocolProcess();

    
}

/*********************************************************************************************************
** Function name:       loop
** Descriptions:        Program entry
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/

void loop() 
{
    InitRAM();

    ProtocolInit();
    
    SetJOGJointParams(&gJOGJointParams, true, &gQueuedCmdIndex);
    
    SetJOGCoordinateParams(&gJOGCoordinateParams, true, &gQueuedCmdIndex);
    
    SetJOGCommonParams(&gJOGCommonParams, true, &gQueuedCmdIndex);

    //SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);
    ProtocolProcess(); 
    // start infinite loop
    for(; ;) {
        //Serial.println("looping");
        delay(1000);
    }
}   

