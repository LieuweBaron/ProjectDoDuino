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
int dobotMode = 1;
int currentMillis = 0;
int dynamicDelayMillis = 0;
int staticDelayMillis = 0;
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
** Descriptions:        Node for the dynamic queue (linkedlist)
** Input parameters:    
** Output parameters:
** Returned value:      
** Developer:           Lieuwe Baron
*********************************************************************************************************/
class node {
    public:
        int data;
        node *next;

        // constructor for node
        node(int data) {
            data = data;
            next = NULL;
        }
};
/*********************************************************************************************************
** Class name:          dynamicQueue
** Descriptions:        Queue that can expand and contract, just a linkedlist with a fancy name
** Class Functions:
**                      Function 1:
**                           Function name:         addToDynamicQueue
**                            Descriptions:         adds data to the dynamic queue
**                            Input parameters:     int data
**                            Output parameters:    none
**                            Returned value:       none
**                      Function 2:
**                           Function name:         removeFromDynamicQueue
**                            Descriptions:         removes item first added item to the dynamic queue from the dynamic queue
**                            Input parameters:     none
**                            Output parameters:    none
**                            Returned value:       none     
** 
**                      Function 3:
**                           Function name:         getNextInDynamicQueue
**                            Descriptions:         gets the value second item in the dynamic queue, if there is no second item, return the head
**                            Input parameters:     none
**                            Output parameters:    none
**                            Returned value:       int nextInQueueValue
**
**                      Function 4:
**                           Function name:         getNextInDynamicQueueAndRemoveIt
**                            Descriptions:         gets and removes the value second item in the dynamic queue, if there is no second item, return the head
**                            Input parameters:     none
**                            Output parameters:    none
**                            Returned value:       none 
**
**                      Function 5:
**                           Function name:         printDynamicQueue
**                            Descriptions:         prints the dynamic queue
**                            Input parameters:     none
**                            Output parameters:    none
**                            Returned value:       none 
**       
** Developer:           Lieuwe Baron 
*********************************************************************************************************/
class dynamicQueue {
    node *head;

    public:
    //constructor for dynamicQueue
        dynamicQueue() {
            head = new node(9999);
        }

        void addToDynamicQueue(int data) {
            // Create the new Node
            node *newNode = new node(data);
            //saves the node the program was last on
            node *lastTraversed = head;
            //traverse the queue
            while (lastTraversed->next != NULL) {
                lastTraversed = lastTraversed->next;
            }
            //add the next node to the back of the queue
            lastTraversed->next = newNode;
        }

        void removeFromDynamicQueue() {
            node* lastTraversed = head;
            //if head is the last item in the queue then do nothing, else remove the seconditem from the queue
            if(head->next == NULL) {
            } 
            else {
                node *secondItem = head->next;
                lastTraversed->next = secondItem->next;
                delete secondItem;
            }
        }

        int getNextInDynamicQueue() {
            //if head is the last item in the queue return its value, if it is not return the value of the second item in the queue
            if(head->next == NULL) {
                return head->data;
            } else {
                node *secondItem = head->next;
                return secondItem->data;
            }
        }

        int getNextInDynamicQueueAndRemoveIt() {
            int nextInQueue = this->getNextInDynamicQueue();
            this->removeFromDynamicQueue();
            return nextInQueue;
        }

        void printDynamicQueue() {
            int count = 0;
            node *lastTraversed = head;
            // Traverse the list
            Serial.println("=================================");
            Serial.println("Printing the Dynamic Queue");
            while (lastTraversed != NULL) {
                count++;
                Serial.print("item #"); Serial.print(count); Serial.print(" ");Serial.println(lastTraversed->data);
                lastTraversed = lastTraversed->next;
            }
            Serial.println("=================================");
        }
};
/*********************************************************************************************************
** Class name:          staticQueue
** Descriptions:        Queue that has a fixed size and loops around
** Class Functions:
**                      Function 1:
**                           Function name:         addToStaticQueue
**                            Descriptions:         adds data to the static queue, if queue is full replace the oldest existing data
**                            Input parameters:     int data
**                            Output parameters:    none
**                            Returned value:       none
**                      Function 2:
**                           Function name:         removeFromStaticQueue
**                            Descriptions:         removes oldest item from the static queue
**                            Input parameters:     none
**                            Output parameters:    none
**                            Returned value:       none     
** 
**                      Function 3:
**                           Function name:         getNextInStaticQueue
**                            Descriptions:         gets the value of the oldest item in the static queue
**                            Input parameters:     none
**                            Output parameters:    none
**                            Returned value:       int nextInQueueValue
**
**                      Function 4:
**                           Function name:         getNextInStaticQueueAndRemoveIt
**                            Descriptions:         gets the value of the oldest item in the static queue and removes it from the static queue
**                            Input parameters:     none
**                            Output parameters:    none
**                            Returned value:       int nextInQueueValue
**
**                      Function 5:
**                           Function name:         recalculateRemoveIndex
**                            Descriptions:         recalculates the next item that needs to be removed
**                            Input parameters:     none
**                            Output parameters:    none
**                            Returned value:       none
**
**                      Function 6:
**                           Function name:         printStaticQueue
**                            Descriptions:         prints the static queue
**                            Input parameters:     none
**                            Output parameters:    none
**                            Returned value:       none 
** 
**                      
** Developer:           Lieuwe Baron
*********************************************************************************************************/
class staticQueue {
    //index that decides at which location a new item has to be added to the static queue
    int addIndex;
    //index that decides at which location a new item has to be removed from the static queue
    int removeIndex; 
    //length of the static queue
    int maxLength;
    //curretn length of the static queue
    int currentLength;
    //pointer to the location of the first item in the queue
    int* queue;


    public:
        //constuctor for staticQueue
        staticQueue(int newStaticQueueMaxLength) {
            addIndex = 1;
            maxLength = newStaticQueueMaxLength;
            queue = new int[newStaticQueueMaxLength];
            //fill the array with NULLs
            for(int i = 0; i < newStaticQueueMaxLength; i++) {
                queue[i] = NULL;
            }
        }

        void addToStaticQueue(int data) {
            //move over all the items in the queue to the right
            for(int i = (maxLength - 1); i >= 0; i--) {
                if(i != 0) {
                    queue[i] = queue[(i-1)];
                } else {
                    queue[i] = data;
                }
            }
        }

        void removeFromStaticQueue(int index) {
             queue[index] = NULL;
        }

        int getNextInStaticQueue() {
            //set return value to 9999, will change if the static queue has an item in it that is not null
            //if the static queue does not have an item in it that is not null return 9999 to indicate that the static queue is empty 
            int nextInQueueValue = 9999;
            for(int i = (maxLength - 1); i >= 0; i--) {
                if(queue[i] != NULL) {
                    nextInQueueValue = queue[i];
                    break;
                }
            } 
            return nextInQueueValue;
        }

        int getNextInStaticQueueAndRemoveIt() {
            int nextInQueueValue = 9999;
            for(int i = (maxLength - 1); i >= 0; i--) {
                if(queue[i] != NULL) {
                    nextInQueueValue = queue[i];
                    removeFromStaticQueue(i);
                    break;
                }
            } 
            return nextInQueueValue;
        }

    
        
        void printStaticQueue() {
            Serial.println("=================================");
            Serial.println("Printing the Static Queue");
            for(int i = 0; i < maxLength; i++) {
                //if(queue[i] != NULL) {
                    Serial.print("index "); Serial.print(i); Serial.print(": "); Serial.println(queue[i]);
                //}
            }
            Serial.println("=================================");
        }
};
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

void dynamicQueueExecutor(int firstInQueue) {
    //only execute switch statement ifd the delay between commands is over
    if(currentMillis >= dynamicDelayMillis) {
        //this switch statement ensures that only one item from the queue can be handled in each iteration of the loop
        switch(firstInQueue) {
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

void staticQueueExecutor(int firstInQueue) {
//only execute switch statement ifd the delay between commands is over
    if(currentMillis >= dynamicDelayMillis) {
        //this switch statement ensures that only one item from the queue can be handled in each iteration of the loop
        switch(firstInQueue) {
            //9999: dynamic queue is empty, break for new loop
            case 9999:
                break;
            case 1:
                Serial.println("EMPTY");
                dynamicDelayMillis += 0;
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
** Function name:       loop
** Descriptions:        loops routine of the dobot
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
** Developer            Lieuwe Baron
*********************************************************************************************************/

void loop()  {
    dynamicQueue dQueue;
    staticQueue sQueue(5);
    Serial.println("initialize loop");
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
        //set currentMillis to millis(), which is the time the board has been running in ms
        //this is how we keep track of time and delays
        currentMillis = millis();
        //handler for inputs, puts the identifier of the command for the dobot associated with a specified button to the queue
        if(dobotMode == 1) {
            if(1 == 1) {
            
            } else if(1 == 1) {

            }
        } else if(dobotMode == 2) {
            if(1 == 1) {
            
            } else if(1 == 1) {

            }
        }
        //if the mode is for the dynamic queue then:
        if(dobotMode == 1) {
            int firstInQueue = dQueue.getNextInDynamicQueueAndRemoveIt();
            dynamicQueueExecutor(firstInQueue);
        //if the mode is for the static queue then:
        } else if(dobotMode == 2) {
            int firstInQueue = sQueue.getNextInStaticQueueAndRemoveIt();
            staticQueueExecutor(firstInQueue);
        }
    }
}

