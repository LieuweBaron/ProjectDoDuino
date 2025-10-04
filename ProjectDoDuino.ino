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
** Class name:          DynamicParameterArray
** Descriptions:        Node for the dynamic queue (linkedlist)
** Input parameters:    
** Output parameters:
** Returned value:      
** Developer:           Lieuwe Baron
*********************************************************************************************************/
class parameterArray {
    int *pArray;
    int length;
    public:
        parameterArray(int par1) {
            pArray = new int[1];
            pArray[0] = par1;

            length = 1;
        }

        parameterArray(int par1, int par2) {
            pArray = new int[2];
            pArray[0] = par1;
            pArray[1] = par2;
            length = 2;
        }

        parameterArray(int par1, int par2, int par3) {
            pArray = new int[3];
            pArray[0] = par1;
            pArray[1] = par2;
            pArray[2] = par3;
            length = 3;
        }

        parameterArray(int par1, int par2, int par3, int par4) {
            pArray = new int[4];
            pArray[0] = par1;
            pArray[1] = par2;
            pArray[2] = par3;
            pArray[3] = par4;
            length = 4;
        }

        parameterArray(int par1, int par2, int par3, int par4, int par5) {
            pArray = new int[5];
            pArray[0] = par1;
            pArray[1] = par2;
            pArray[2] = par3;
            pArray[3] = par4;
            pArray[4] = par5;
            length = 5;
        }

        void printDynamicParameterArray() {
            for(int i = 0; i <= (length - 1); i++) {
                Serial.print("parameter array index "); Serial.print(i); Serial.print(": "); Serial.println(*(pArray + i));
            }
        }

        void deleteDynamicParameterArray() {
            //display_freeRam();
            delete[] pArray;
            //display_freeRam();
        }
};
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
        int data;
        node *next;
        int pArraySize;
        int *pArray;
        //constructs the head
        node(int data) {
            data = data;
            next = NULL;
            pArraySize = NULL;
            pArray = NULL;
        }

        node(int data, int par1) {
            data = data;
            next = NULL;
            pArraySize = 1;
            pArray = new int[1];
            pArray[0] = par1;
        }

        node(int data, int par1, int par2) {
            data = data;
            next = NULL;
            pArraySize = 2;
            pArray = new int[2];
            pArray[0] = par1;
            pArray[1] = par2;
        }

        node(int data, int par1, int par2, int par3) {
            data = data;
            next = NULL;
            pArraySize = 3;
            pArray = new int[3];
            pArray[0] = par1;
            pArray[1] = par2;
            pArray[2] = par3;
        }

        node(int data, int par1, int par2, int par3, int par4) {
            data = data;
            next = NULL;
            pArraySize = 4;
            pArray = new int[4];
            pArray[0] = par1;
            pArray[1] = par2;
            pArray[2] = par3;
            pArray[3] = par4;
        }

        node(int data, int par1, int par2, int par3, int par4, int par5) {
            data = data;
            next = NULL;
            pArraySize = 5;
            pArray = new int[5];
            pArray[0] = par1;
            pArray[1] = par2;
            pArray[2] = par3;
            pArray[3] = par4;
            pArray[4] = par5;
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

        void addToDynamicQueue(int command, int p[], int pSize) {
            node *newNode;
            //not elegant, but it works
            if(pSize == 1) {
                newNode = new node(command, p[0]);
            } else if(pSize == 2) {
                newNode = new node(command, p[0], p[1]);
            } else if(pSize == 3) {
                newNode = new node(command, p[0], p[1], p[2]);
            } else if(pSize == 4) {
                newNode = new node(command, p[0], p[1], p[2], p[3]);
            } else if(pSize == 5) {
                newNode = new node(command, p[0], p[1], p[2], p[3], p[4]);
            }
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
** Function name:       dynamicQueueExecutor
** Descriptions:        executes the command from the static queue associated with the inputted id and sets the delay until the next command
** Input parameters:    int firstInQueue
** Output parameters:   none
** Returned value:      none
** Developer:           Lieuwe Baron
*********************************************************************************************************/
void dynamicQueueExecutor(int firstInQueueID) {
    //only execute switch statement ifd the delay between commands is over
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
** Function name:       staticQueueExecutor
** Descriptions:        executes the command from the static queue associated with the inputted id and sets the delay until the next command
** Input parameters:    int firstInQueue
** Output parameters:   none
** Returned value:      none
** Developer:           Lieuwe Baron
*********************************************************************************************************/
void staticQueueExecutor(int firstInQueueID) {
//only execute switch statement ifd the delay between commands is over
    if(currentMillis >= dynamicDelayMillis) {
        //this switch statement ensures that only one item from the queue can be handled in each iteration of the loop
        switch(firstInQueueID) {
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
    dynamicQueue dQueue;
    staticQueue sQueue(5);
    Serial.println("initialize loop");
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
        if(dobotMode == 1) {
            if(1 == 1) {
            
            } 
            if(1 == 1) {

            }
        } else if(dobotMode == 2) {
            if(1 == 1) {
            
            } 
            if(1 == 1) {

            }
        }
        //if the mode is for the dynamic queue then:
        if(dobotMode == 1) {
            int firstInQueue = dQueue.getNextInDynamicQueueAndRemoveIt();
            //dynamicQueueExecutor(firstInQueue);
        //if the mode is for the static queue then:
        } else if(dobotMode == 2) {
            int firstInQueue = sQueue.getNextInStaticQueueAndRemoveIt();
            //staticQueueExecutor(firstInQueue);
        }
    }
}

