/*********************************************************************************************************
------------------------------------Important Information-------------------------------------------------
**  ProtocolProcess() is used to have the Dobot execute commands. (23:39 - 27/09/2025)
**  Serial.println(printf("Address of value: %p\n", (void*)ptr)); to print the adress of a pointer in the Serial Monitor
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
EndEffectorParams   gEndEffectorParams;

JOGJointParams      gJOGJointParams;
JOGCoordinateParams gJOGCoordinateParams;
JOGCommonParams     gJOGCommonParams;
JOGCmd              gJOGCmd;

PTPCoordinateParams gPTPCoordinateParams;
PTPCommonParams     gPTPCommonParams;
PTPCommonParams     speed;
PTPCmd              gPTPCmd;    

uint64_t gQueuedCmdIndex;

/*********************************************************************************************************
** Global variables // made by lieuwe
*********************************************************************************************************/
Pose robotPose;
int dobotMode = 1;
int currentMillis = 0;
int dynamicDelayMillis = 0;
int staticDelayMillis = 0;
int queueSize = 5;
//start position of the dobot, based on cartesian coordinates
float startX = 0.00;       
float startY = -200.00;       
float startZ = 0.00;       
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
    Serial.println("===========Serial communication established=================");
    Serial.println(" ");
    //Set Timer Interrupt
    FlexiTimer2::set(100,Serialread); 
    FlexiTimer2::start();
}

void display_freeRam() {
  Serial.print(F("- SRAM left: ")); Serial.println(freeRam());
}

int freeRam() {
  extern int __heap_start,*__brkval;
  int v;
  return (int)&v - (__brkval == 0  
    ? (int)&__heap_start : (int) __brkval);  
}
/*********************************************************************************************************
** Function name:       Serialread
** Descriptions:        import data to rxbuffer
** Input parameters     none:
** Output parameters:   
** Returned value:
** Developer:           Dobot Labs      
*********************************************************************************************************/
void Serialread() {
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
int Serial_putc( char c, struct __file * ) {
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
void printf_begin(void) {
    fdevopen( &Serial_putc, 0 );
}
/*********************************************************************************************************
** Class name:          node
** Descriptions:        Node for the dynamic queue 
** Input parameters:    
** Output parameters:
** Returned value:      
** Developer:           Lieuwe Baron
*********************************************************************************************************/
class node {
    public:
        int command;
        int pArraySize;
        int pArray[4] = {9999,9999,9999,9999};
        //constructs the head
        node() {
            command = 9999;
            pArraySize = 4;
        }     
};
/*********************************************************************************************************
** Class name:          queue
** Descriptions:        Queue that has a fixed size
** Class Functions:
**                      Function 1:
**                           Function name:         addToQueue
**                            Descriptions:         adds command and parameter data to the queue, if queue is full replace the oldest existing data
**                            Input parameters:     int command, int p[]
**                            Output parameters:    none
**                            Returned value:       none
**                      Function 2:
**                           Function name:         removeFromQueue
**                            Descriptions:         removes oldest item from the queue
**                            Input parameters:     int index
**                            Output parameters:    none
**                            Returned value:       none     
**
**                      Function 3:
**                           Function name:         getNextInQueueAndRemoveIt
**                            Descriptions:         gets the value of the oldest item in the queue and removes it from the queue
**                            Input parameters:     none
**                            Output parameters:    none
**                            Returned value:       int *nextInQueueValueArray
**
**                      Function 4:
**                           Function name:         printQueue
**                            Descriptions:         prints the queue
**                            Input parameters:     none
**                            Output parameters:    none
**                            Returned value:       none 
** 
**                      
** Developer:           Lieuwe Baron
*********************************************************************************************************/
class cmdQueue {
    //index that decides at which location a new item has to be added to the queue
    int addIndex;
    //index that decides at which location a new item has to be removed from the queue
    int removeIndex; 
    //length of the queue
    int maxLength = 5;
    //current length of the queue
    int currentLength;
    //pointer to the location of the first item in the queue
    node queue[5];
    public:
        //constuctor for queue
        cmdQueue() {
            addIndex = 1;
        }

        void addToQueue(int command, int p[]) {
            for(int i = (maxLength - 1); i >= 0; i--) {
                if(i != 0) {
                    queue[i].command = queue[i-1].command;
                    for(int j = 0; j <= 3; j++) {

                            queue[i].pArray[j] = queue[i-1].pArray[j];
                    } 
                } else {
                    queue[i].command = command;
                    for(int j = 0; j <= 3; j++) {
                        queue[i].pArray[j] = p[j];
                    }
                }
            }
        }

        void removeFromQueue(int index) {
             queue[index].command = 9999;
             for(int j = 0; j <= 3; j++) {
                        queue[index].pArray[j] = 9999;
                    }
        }



        int * getNextInQueueAndRemoveIt() {
            int *nextInQueueValueArray = new int[5];
            for(int i = 0; i <= 5; i++) {
                nextInQueueValueArray[i] = 9999;
            }
            for(int i = (maxLength - 1); i >= 0; i--) {
                if(queue[i].command != 9999) {
                    nextInQueueValueArray[0] = queue[i].command;
                    for(int j = 1; j <= 4; j++) {
                        nextInQueueValueArray[j] = queue[i].pArray[j-1];
                    }
                    removeFromQueue(i);
                    break;
                }
            } 
            return nextInQueueValueArray;
        }

    

        void printQueue() {
            Serial.println("=================================");
            Serial.println("Printing the Static Queue");
            for(int i = 0; i < maxLength; i++) {
                Serial.print("index "); Serial.print(i); Serial.print(": "); Serial.print("Command: "); Serial.print(queue[i].command); Serial.print("; parameters: "); 
                Serial.print(queue[i].pArray[0]); Serial.print(", "); Serial.print(queue[i].pArray[1]); Serial.print(", "); Serial.print(queue[i].pArray[2]); Serial.print(", "); Serial.print(queue[i].pArray[3]); Serial.println(";");
            }
            Serial.println("=================================");
        }
};
/*********************************************************************************************************
** Function name:       dynamicQueueExecutor
** Descriptions:        executes the command from the static queue associated with the inputted id and sets the delay until the next command
** Input parameters:    int firstInQueue
** Output parameters:   none
** Returned value:      none
** Developer:           Lieuwe Baron
*********************************************************************************************************/
void queueExecutor(int firstInQueueID) {
    //only execute switch statement if the delay between commands is over
    if(currentMillis >= dynamicDelayMillis) {
        //this switch statement ensures that only one item from the queue can be handled in each iteration of the loop
        switch(firstInQueueID) {
            //9999: dynamic queue is empty, break for new loop
            case 9999:
                break;
            case 1:
                Serial.println("EMPTY");
                break;
            case 2:
                Serial.println("EMPTY");
                break;
            case 3:
                Serial.println("EMPTY");
                break;
            case 4:
                Serial.println("EMPTY");
                break;
            case 5:
                Serial.println("EMPTY");
                break;
            case 6:
                Serial.println("EMPTY");
                break;
            case 7:
                Serial.println("EMPTY");
                break;
            case 8:
                Serial.println("EMPTY");
                break;
            case 9:
                Serial.println("EMPTY");
                break;
            case 10:
               Serial.println("EMPTY");
                break;
        }
    }
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
    Serial.print(gPTPCmd.x);
    SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);
    ProtocolProcess();
    Serial.println("function moveDobotToPos() called");
    Serial.print("move to x:"); Serial.print(gPTPCmd.x); Serial.print(" y:"); Serial.print(gPTPCmd.y); Serial.print(" z:"); Serial.println(gPTPCmd.r);
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
    Serial.println("function moveDobotByIncrement() called");
    Serial.print("move to x:"); Serial.print(gPTPCmd.x); Serial.print(" y:"); Serial.print(gPTPCmd.y); Serial.print(" z:"); Serial.println(gPTPCmd.r);
    delay(500);
}
/*********************************************************************************************************
** Function name:       saveRoute
** Descriptions:        saves a user created route
** Input parameters:    none
** Output parameters:   savedRoute
** Returned value:      none
** Developer:           Lieuwe Baron
*********************************************************************************************************/
void saveRoute() {

}
/*********************************************************************************************************
** Function name:       replayRoute
** Descriptions:        replays a user created route
** Input parameters:    route
** Output parameters:   none
** Returned value:      none
** Developer:           Lieuwe Baron
*********************************************************************************************************/
void replayRoute() {

}
/*********************************************************************************************************
** Function name:       suctionCupEnable
** Descriptions:        Enables/Disables the suction cup of the Dobot arm
** Input parameters:    bool suctionEnable
** Output parameters:   none
** Returned value:      none
** Developer:           Lieuwe Baron
*********************************************************************************************************/
void suctionCupEnable(bool suctionEnable) {
    if(suctionEnable == true) {
        SetEndEffectorSuctionCup(true, true, &gQueuedCmdIndex);
        suctionCurrentlyOn = suctionEnable;
    }
    else {
        if (suctionEnable == false){
            SetEndEffectorSuctionCup(false, true, &gQueuedCmdIndex);
            suctionCurrentlyOn = suctionEnable; 
        }
    }
    ProtocolProcess();
    Serial.println("function suctionCupEnable() called");
    delay(10000);
    
}
/*********************************************************************************************************
** Function name:       InitRAM
** Descriptions:        Initializes a global variable
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
** Developer:           Dobot Labs/Lieuwe Baron
*********************************************************************************************************/
void InitRAM(void) {
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
    
    ProtocolProcess();
    
}

/*********************************************************************************************************
** Function name:       loop
** Descriptions:        loops routine of the dobot
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
** Developer            Lieuwe Baron
*********************************************************************************************************/

void loop()  {
    cmdQueue cmdsQueue;
    Serial.println("initialize loop");
    int params[4] = {1760,290,3650, 9999}; 
    int params2[4] = {100, 200, 9999, 9999}; 
    int params3[4] = {1000,2000,3000, 4000}; 
    int params4[4] = {8675, 34, 645, 9999}; 
    int params5[4] = {68,564,6548, 4333}; 
    cmdsQueue.addToQueue(8, params);
    cmdsQueue.addToQueue(54, params2);
    cmdsQueue.addToQueue(3, params3);
    cmdsQueue.addToQueue(2, params4);
    cmdsQueue.addToQueue(5, params5);
    cmdsQueue.printQueue();
    int *output = cmdsQueue.getNextInQueueAndRemoveIt();
    Serial.print("next in array: "); Serial.print("Command: "); Serial.print(output[0]); Serial.print(" Parameters: "); 
    for(int i = 1; i <= 5; i++) {
        Serial.print(output[i]); Serial.print(", ");  
    }
    cmdsQueue.printQueue();
    //cmdsQueue.addToQueue(3, params, 4);
    //cmdsQueue.addToQueue(3, params, 4);
    //cmdsQueue.printQueue();
    //display_freeRam();
    //dynamicParameterArray pArr(87,4,7,10);
    //pArr.printDynamicParameterArray();
    //pArr.deleteDynamicParameterArray();
    //InitRAM();
    //ProtocolInit();
    //SetJOGJointParams(&gJOGJointParams, true, &gQueuedCmdIndex);
    //SetJOGCoordinateParams(&gJOGCoordinateParams, true, &gQueuedCmdIndex);
    //SetJOGCommonParams(&gJOGCommonParams, true, &gQueuedCmdIndex);
    //delay to give the dobot time to start up
    //delay(5000);
    //SetEndEffectorSuctionCup(true, true, &gQueuedCmdIndex);
    //moveDobotToPos(startX, startY, startZ, startR);
    //ProtocolProcess(); 

    // start infinite loop
    for(; ;) {
        delay(1000);
        //set currentMillis to millis(), which is the time the board has been running in ms
        //this is how we keep track of time and delays
        currentMillis = millis();
        //handler for inputs, puts the identifier of the command for the dobot associated with a specified button to the queue
    }
}

