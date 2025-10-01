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
//queue variables
int queueSize = 5;
int queue[5] = {23,54,67,27,98};
int* queuePTR = queue;
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
** Class name:          node
** Descriptions:        Node for the dynamic queue (linkedlist)
** Input parameters:    
** Output parameters:
** Returned value:      
** Developer:           Lieuwe Baron
*********************************************************************************************************/
class node {
    int data;
    node* nextNodePTR;

    public:
    //constructor for the node
        Node(int data) {
            this->data = data;
            this->next = NULL;
        }
}
/*********************************************************************************************************
** Class name:          dynamicQueue
** Descriptions:        Queue that can expand and contract, just a linkedlist with a fancy name
** Functions            
** 
**       
** Developer:           Lieuwe Baron
*********************************************************************************************************/
class dynamicQueue {
    node *head;

    public:
        //constructor for the dynamic queue
        Linkedlist() {
            node *headNode = new Node(9999);
            head = headNode;
        }   
        
        void expandDynamicQueue(node* head, int data) {
            // Create the new Node
            node *newNode = new Node(data);
            if (head == NULL) {
                Serial.println("error, deleted head somehow");
                return;
            } else {
                Node *lastTraversed = head;
                // Traverse the list
                while (lastTraversed != NULL) {
                lastTraversed = lastTraversed->nextNodePTR;
            }
            lastTraversed->next = newNode;

            return head;
        }

        void shrinkDynamicQueue
}
/*********************************************************************************************************
** Function name:       expandQueue
** Descriptions:        Expand the queue of dobot commands
** Input parameters:    int queuedItem, int size, int* oldQueuePTR
** Output parameters:   none
** Returned value:      int* newQueuePTR
** Developer:           Lieuwe Baron
*********************************************************************************************************/
//int expandDynamicQueue(int queuedItem) {

}
/*int expandQueue(int queuedItem) {
    //queueSize += 1;
    int newQueue[queueSize] = {0};
    //newQueue[0] = queuedItem;
    //move over the items in the old queue to the new queue
    for(int i = 0; i < queueSize; i++) {
        newQueue[i] = *(queuePTR + i);
        //newQueue[i+1] = *(oldQueuePTR + i);
    }
    int* newQueuePTR = newQueue;
    //deallocate memory of the old queue using the pointer of the old queue
    for(int i = queueSize; i > 0; i--) {
        delete (queuePTR + i);
    }
    for(int i = 0; i < queueSize; i++) {
        Serial.println(*(newQueuePTR + i));
    }
    //chance the old queuePTR into the new one
    queuePTR = newQueuePTR;
}*/
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
    //delay to give the dobot time to start up
    delay(5000);
    //SetEndEffectorSuctionCup(true, true, &gQueuedCmdIndex);
    //moveDobotToPos(startX, startY, startZ, startR);
    ProtocolProcess(); 
    // start infinite loop

    for(; ;) {
        Serial.println("===========================");
        //int firstInQueue = cmdQueue[0];
        
        expandQueue(queueSize);
        delay(50);
        Serial.println("---------------------------");
        delay(50);
        for(int i = 0; i < queueSize; i++) {
            Serial.println(*(queuePTR + i));
        }
        Serial.println("===========================");
        /*switch(firstInQueue) {
            case 0:
                Serial.println("EMPTY");
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
        }*/
        delay(1000);
        //ClearAllAlarmsState(true);
        //SetEndEffectorSuctionCup(true, true, &gQueuedCmdIndex);
        //SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);
        ProtocolProcess();
    }
}   

