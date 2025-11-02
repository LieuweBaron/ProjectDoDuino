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
#include <Nextion.h>
#include <EEPROM.h>
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

JOGJointParams gJOGJointParams;
JOGCoordinateParams gJOGCoordinateParams;
JOGCommonParams gJOGCommonParams;
JOGCmd gJOGCmd;

PTPCoordinateParams gPTPCoordinateParams;
PTPCommonParams gPTPCommonParams;
PTPCommonParams speed;
PTPCmd gPTPCmd;

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
bool suctionCup = true;
bool suctionCurrentlyOn = false;

float moveIncrement = 0.0;
int currentPage = 1;
//dual state button states
int bt100State = -1;
int bt101State = -1;
int bt102State = -1;
int bt103State = -1;
int bt104State = -1;
int bt300State = -1;
int bt301State = -1;
int bt302State = -1;
int bt303State = -1;
int bt304State = -1;
int bt305State = -1;
int bt306State = -1;
int bt307State = -1;
int bt308State = -1;
int bt309State = -1;
int bt310State = -1;
int bt311State = -1;




//pages on nextion screen
NexPage page1 = NexPage(0, 0, "page0");  //page 0, reffered to as page 1, Controls page
NexPage page2 = NexPage(1, 0, "page1");  //page 1, reffered to as page 2, Routes page
NexPage page3 = NexPage(2, 0, "page2");  //page 2, reffered to as page 3, Use Routes page
NexPage page4 = NexPage(3, 0, "page3");  //page 3, reffered to as page 4, Queue page
//buttons on nextion screen
NexButton b100 = NexButton(0, 18, "b100");  //button that moves X in the + direction, page 1
NexButton b101 = NexButton(0, 19, "b101");  //button that moves X in the - direction, page 1
NexButton b102 = NexButton(0, 20, "b102");  //button that moves Y in the + direction, page 1
NexButton b103 = NexButton(0, 21, "b103");  //button that moves Y in the - direction, page 1
NexButton b104 = NexButton(0, 22, "b104");  //button that moves Z in the + direction, page 1
NexButton b105 = NexButton(0, 23, "b105");  //button that moves Z in the - direction, page 1

NexButton b200 = NexButton(1, 24, "b200");  //button that adds new point to trajectory, page 2
NexButton b201 = NexButton(1, 4, "b201");   //button that saves trajectory to route 1, page 2
NexButton b202 = NexButton(1, 5, "b202");   //button that saves trajectory to route 2, page 2
NexButton b203 = NexButton(1, 6, "b203");   //button that saves trajectory to route 3, page 2
NexButton b204 = NexButton(1, 7, "b204");   //button that saves trajectory to route 4, page 2
NexButton b205 = NexButton(1, 25, "b205");  //button that removes point 2 in the trajectory, page 2
NexButton b206 = NexButton(1, 22, "b206");  //button that removes point 3 in the trajectory, page 2
NexButton b207 = NexButton(1, 23, "b207");  //button that removes point 4 in the trajectory, page 2
NexButton b208 = NexButton(1, 26, "b208");  //button that removes point 5 in the trajectory, page 2
NexButton b209 = NexButton(1, 27, "b209");  //button that removes point 6 in the trajectory, page 2
NexButton b210 = NexButton(1, 28, "b210");  //button that removes point 7 in the trajectory, page 2

NexButton b400 = NexButton(3, 11, "b400");  //button that removes element 0 out of the queue, page 4
NexButton b401 = NexButton(3, 12, "b401");  //button that removes element 1 out of the queue, page 4
NexButton b402 = NexButton(3, 13, "b402");  //button that removes element 2 out of the queue, page 4
NexButton b403 = NexButton(3, 14, "b403");  //button that removes element 3 out of the queue, page 4
NexButton b404 = NexButton(3, 15, "b404");  //button that removes element 4 out of the queue, page 4
NexButton b405 = NexButton(3, 16, "b405");  //button that removes element 5 out of the queue, page 4
NexButton b406 = NexButton(3, 17, "b406");  //button that removes element 6 out of the queue, page 4
NexButton b407 = NexButton(3, 18, "b407");  //button that removes element 7 out of the queue, page 4
NexButton b408 = NexButton(3, 19, "b408");  //button that removes element 8 out of the queue, page 4
NexButton b409 = NexButton(3, 20, "b409");  //button that removes element 9 out of the queue, page 4

//dual-state buttons on nextion screen
NexDSButton bt100 = NexDSButton(0, 24, "bt100");  //enable or disable suction cup
NexDSButton bt101 = NexDSButton(0, 10, "bt101");  //increment of movement: 0.1, page 1
NexDSButton bt102 = NexDSButton(0, 11, "bt102");  //increment of movement: 1, page 1
NexDSButton bt103 = NexDSButton(0, 12, "bt103");  //increment of movement: 10, page 1
NexDSButton bt104 = NexDSButton(0, 13, "bt104");  //increment of movement: 50, page 1

NexDSButton bt300 = NexDSButton(2, 5, "bt300");   //button that activates route 1, page 3
NexDSButton bt301 = NexDSButton(2, 6, "bt301");   //button that activates route 2, page 3
NexDSButton bt302 = NexDSButton(2, 7, "bt302");   //button that activates route 3, page 3
NexDSButton bt303 = NexDSButton(2, 8, "bt303");   //button that activates route 4, page 3
NexDSButton bt304 = NexDSButton(2, 9, "bt304");   //button that activates detect sensor mode on route 1, page 3
NexDSButton bt305 = NexDSButton(2, 12, "bt305");  //button that activates detect sensor mode on route 2, page 3
NexDSButton bt306 = NexDSButton(2, 14, "bt306");  //button that activates detect sensor mode on route 3, page 3
NexDSButton bt307 = NexDSButton(2, 16, "bt307");  //button that activates detect sensor mode on route 4, page 3
NexDSButton bt308 = NexDSButton(2, 10, "bt308");  //button that activates repeat mode on route 1, page 3
NexDSButton bt309 = NexDSButton(2, 11, "bt309");  //button that activates repeat mode on route 2, page 3
NexDSButton bt310 = NexDSButton(2, 13, "bt310");  //button that activates repeat mode on route 3, page 3
NexDSButton bt311 = NexDSButton(2, 15, "bt311");  //button that activates repeat mode on route 4, page 3
//text fields on nextion screen
NexText t100 = NexText(0, 1, "t100");  //current X position, page 1
NexText t101 = NexText(0, 5, "t101");  //upper bound of the current X limit, page 1
NexText t102 = NexText(0, 6, "t102");  //lower bount of the current X limit, page 1
NexText t103 = NexText(0, 2, "t103");  //current Y position, page 1
NexText t104 = NexText(0, 3, "t104");  //upper bound of the current Y limit, page 1
NexText t105 = NexText(0, 7, "t105");  //lower bount of the current Y limit, page 1
NexText t106 = NexText(0, 4, "t106");  //current Z position, page 1
NexText t107 = NexText(0, 8, "t107");  //upper bound of the current Z limit, page 1
NexText t108 = NexText(0, 9, "t108");  //lower bount of the current Z limit, page 1

NexText t200 = NexText(1, 8, "t200");   //current X, page 2
NexText t201 = NexText(1, 10, "t201");  //current Y, page 2
NexText t202 = NexText(1, 9, "t202");   //current Z, page 2
NexText t203 = NexText(1, 11, "t203");  //current SC (suction cup state), page 2
NexText t205 = NexText(1, 14, "t205");  //point 1 in trajectory (start point), page 2
NexText t206 = NexText(1, 15, "t206");  //point 2 in trajectory, page 2
NexText t207 = NexText(1, 16, "t207");  //point 3 in trajectory, page 2
NexText t208 = NexText(1, 17, "t208");  //point 4 in trajectory, page 2
NexText t209 = NexText(1, 18, "t209");  //point 5 in trajectory, page 2
NexText t210 = NexText(1, 19, "t210");  //point 6 in trajectory, page 2
NexText t211 = NexText(1, 20, "t211");  //point 7 in trajectory, page 2
NexText t212 = NexText(1, 21, "t212");  //point 8 in trajectory (end point), page 2

NexText t400 = NexText(3, 1, "t400");   //queue element 0, page 4
NexText t401 = NexText(3, 2, "t401");   //queue element 1, page 4
NexText t402 = NexText(3, 3, "t402");   //queue element 2, page 4
NexText t403 = NexText(3, 4, "t403");   //queue element 3, page 4
NexText t404 = NexText(3, 5, "t404");   //queue element 4, page 4
NexText t405 = NexText(3, 6, "t405");   //queue element 5, page 4
NexText t406 = NexText(3, 7, "t406");   //queue element 6, page 4
NexText t407 = NexText(3, 8, "t407");   //queue element 7, page 4
NexText t408 = NexText(3, 9, "t408");   //queue element 8, page 4
NexText t409 = NexText(3, 10, "t409");  //queue element 9, page 4

NexTouch *nex_listen_list[] = {
    &page1,
    &page2,
    &page3,
    &page4,

    &b100,
    &b101,
    &b102,
    &b103,
    &b104,
    &b105,
    &b200,
    &b201,
    &b202,
    &b203,
    &b204,
    &b205,
    &b206,
    &b207,
    &b208,
    &b209,
    &b210,
    &b400,
    &b401,
    &b402,
    &b403,
    &b404,
    &b405,
    &b406,
    &b407,
    &b408,
    &b409,

    &bt100,
    &bt101,
    &bt102,
    &bt103,
    &bt104,
    &bt300,
    &bt301,
    &bt302,
    &bt303,
    &bt304,
    &bt305,
    &bt306,
    &bt307,
    &bt308,
    &bt309,
    &bt310,
    &bt311,

    NULL
};

//page change event handlers
void page1PushEventHandler(void *ptr)
{
    Serial.println("Page 1");
    currentPage = 1;
}

void page2PushEventHandler(void *ptr)
{
    Serial.println("Page 2");
    currentPage = 2;
}
void page3PushEventHandler(void *ptr)
{
    Serial.println("Page 3");
    currentPage = 3;
}

void page4PushEventHandler(void *ptr)
{
    Serial.println("Page 4");
    currentPage = 4;
}

//button event handlers
void b100PopEventHandler(void *ptr)
{
    Serial.println("button b100 (move Dobot in +X Direction | [+X]) pressed");
}

void b101PopEventHandler(void *ptr)
{
    Serial.println("button b101 (move Dobot in -X Direction | [-X]) pressed");
}

void b102PopEventHandler(void *ptr)
{
    Serial.println("button b102 (move Dobot in +Y Direction | [+Y]) pressed");
}

void b103PopEventHandler(void *ptr)
{
    Serial.println("button b103 (move Dobot in -Y Direction | [+Y]) pressed");
}

void b104PopEventHandler(void *ptr)
{
    Serial.println("button b104 (move Dobot in +Z Direction | [+Z]) pressed");
}
void b105PopEventHandler(void *ptr)
{
    Serial.println("button b105 (move Dobot in -Z Direction | [+Z]) pressed");
}

void b200PopEventHandler(void *ptr)
{
    Serial.println("button b200 (add point to trajectory | [Add Point To Trajecory]) pressed");
}

void b201PopEventHandler(void *ptr)
{
    Serial.println("button b201 (save trajectory to route 1 | [Save Route 1]) pressed");
}

void b202PopEventHandler(void *ptr)
{
    Serial.println("button b202 (save trajectory to route 2 | [Save Route 2]) pressed");
}

void b203PopEventHandler(void *ptr)
{
    Serial.println("button b203 (save trajectory to route 3 | [Save Route 3]) pressed");
}

void b204PopEventHandler(void *ptr)
{
    Serial.println("button b204 (save trajectory to route 4 | [Save Route 4]) pressed");
}

void b205PopEventHandler(void *ptr)
{
    Serial.println("button b205 (remove point 2 in trajectory | [X]) pressed");
}

void b206PopEventHandler(void *ptr)
{
    Serial.println("button b206 (remove point 3 in trajectory | [X]) pressed");
}

void b207PopEventHandler(void *ptr)
{
    Serial.println("button b207 (remove point 4 in trajectory | [X]) pressed");
}

void b208PopEventHandler(void *ptr)
{
    Serial.println("button b208 (remove point 5 in trajectory | [X]) pressed");
}

void b209PopEventHandler(void *ptr)
{
    Serial.println("button b209 (remove point 6 in trajectory | [X]) pressed");
}

void b210PopEventHandler(void *ptr)
{
    Serial.println("button b210 (remove point 7 in trajectory | [X]) pressed");
}

void b400PopEventHandler(void *ptr)
{
    Serial.println("button b400 (remove element 0 out of queue | [X]) pressed");
}

void b401PopEventHandler(void *ptr)
{
    Serial.println("button b401 (remove element 1 out of queue | [X]) pressed");
}

void b402PopEventHandler(void *ptr)
{
    Serial.println("button b402 (remove element 2 out of queue | [X]) pressed");
}

void b403PopEventHandler(void *ptr)
{
    Serial.println("button b403 (remove element 3 out of queue | [X]) pressed");
}

void b404PopEventHandler(void *ptr)
{
    Serial.println("button b404 (remove element 4 out of queue | [X]) pressed");
}

void b405PopEventHandler(void *ptr)
{
    Serial.println("button b405 (remove element 5 out of queue | [X]) pressed");
}

void b406PopEventHandler(void *ptr)
{
    Serial.println("button b406 (remove element 6 out of queue | [X]) pressed");
}

void b407PopEventHandler(void *ptr)
{
    Serial.println("button b407 (remove element 7 out of queue | [X]) pressed");
}

void b408PopEventHandler(void *ptr)
{
    Serial.println("button b408 (remove element 8 out of queue | [X]) pressed");
}

void b409PopEventHandler(void *ptr)
{
    Serial.println("button b409 (remove element 9 out of queue | [X]) pressed");
}

//dual-state buttons event handlers

//output port not working correctly
void bt100PopEventHandler(void *ptr)
{
    //moveDobotToPos(30,30,30,0);
    Serial.println("button bt100 (enable/disable suction cup | [Suction Cup]) pressed");
    if (bt100State == 1)
    {  //previously enabled
        bt100State = 0;
        //suctionCupEnable(false);
    }
    else if (bt100State == 0)
    {  //previously disabled
        bt100State = 1;
        //suctionCupEnable(true);
    }
    else if (bt100State == -1)
    {  //not clicked before
        bt100State = 1;
        //suctionCupEnable(true);
    }
}

void bt101PopEventHandler(void *ptr)
{
    Serial.println("button bt101 (movement increment 0.1 | [0.1]) pressed");
    moveIncrement = 0.1;
    if (bt101State == 0)
    {  //previously enabled
        bt101State = 1;
    }
    else if (bt101State == 1)
    {  //previously disabled
        bt101State = 0;
    }
    else if (bt101State == -1)
    {  //not clicked before
        bt101State = 1;
    }
}

void bt102PopEventHandler(void *ptr)
{
    Serial.println("button bt102 (movement increment 1 | [1]) pressed");
    moveIncrement = 1.0;
    if (bt102State == 0)
    {  //previously enabled
        bt102State = 1;
    }
    else if (bt102State == 1)
    {  //previously disabled
        bt102State = 0;
    }
    else if (bt102State == -1)
    {  //not clicked before
        bt102State = 1;
    }
}

void bt103PopEventHandler(void *ptr)
{
    Serial.println("button bt103 (movement increment 10 | [10]) pressed");
    moveIncrement = 10.0;
    if (bt103State == 0)
    {  //previously enabled
        bt103State = 1;
    }
    else if (bt103State == 1)
    {  //previously disabled
        bt103State = 0;
    }
    else if (bt103State == -1)
    {  //not clicked before
        bt103State = 1;
    }
}

void bt104PopEventHandler(void *ptr)
{
    Serial.println("button bt104 (movement increment 50 | [50]) pressed");
    moveIncrement = 50.0;
    if (bt104State == 0)
    {  //previously enabled
        bt104State = 1;
    }
    else if (bt104State == 1)
    {  //previously disabled
        bt104State = 0;
    }
    else if (bt104State == -1)
    {  //not clicked before
        bt104State = 1;
    }
}

void bt300PopEventHandler(void *ptr)
{
    Serial.println("button bt300 (activates route 1| [Activate Route 1]) pressed");
    if (bt301State == 0)
    {  //previously enabled
        bt301State = 1;
    }
    else if (bt301State == 1)
    {  //previously disabled
        bt301State = 0;
    }
    else if (bt301State == -1)
    {  //not clicked before
        bt301State = 1;
    }
}

void bt301PopEventHandler(void *ptr)
{
    Serial.println("button bt301 (activates route 2| [Activate Route 2]) pressed");
    if (bt302State == 0)
    {  //previously enabled
        bt302State = 1;
    }
    else if (bt302State == 1)
    {  //previously disabled
        bt302State = 0;
    }
    else if (bt302State == -1)
    {  //not clicked before
        bt302State = 1;
    }
}

void bt302PopEventHandler(void *ptr)
{
    Serial.println("button bt302 (activates route 3| [Activate Route 3]) pressed");
    if (bt303State == 0)
    {  //previously enabled
        bt303State = 1;
    }
    else if (bt303State == 1)
    {  //previously disabled
        bt303State = 0;
    }
    else if (bt303State == -1)
    {  //not clicked before
        bt303State = 1;
    }
}

void bt303PopEventHandler(void *ptr)
{
    Serial.println("button bt303 (activates route 4| [Activate Route 3]) pressed");
    if (bt304State == 0)
    {  //previously enabled
        bt304State = 1;
    }
    else if (bt304State == 1)
    {  //previously disabled
        bt304State = 0;
    }
    else if (bt304State == -1)
    {  //not clicked before
        bt304State = 1;
    }
}

void bt304PopEventHandler(void *ptr)
{
    Serial.println("button bt304 (activates sensor detect mode on route 1| [Activate On Sensor Detect]) pressed");
    if (bt305State == 0)
    {  //previously enabled
        bt305State = 1;
    }
    else if (bt305State == 1)
    {  //previously disabled
        bt305State = 0;
    }
    else if (bt305State == -1)
    {  //not clicked before
        bt305State = 1;
    }
}

void bt305PopEventHandler(void *ptr)
{
    Serial.println("button bt305 (activates sensor detect mode on route 2| [Activate On Sensor Detect]) pressed");
    if (bt306State == 0)
    {  //previously enabled
        bt306State = 1;
    }
    else if (bt306State == 1)
    {  //previously disabled
        bt306State = 0;
    }
    else if (bt306State == -1)
    {  //not clicked before
        bt306State = 1;
    }
}

void bt306PopEventHandler(void *ptr)
{
    Serial.println("button bt306 (activates sensor detect mode on route 3| [Activate On Sensor Detect]) pressed");
    if (bt306State == 0)
    {  //previously enabled
        bt306State = 1;
    }
    else if (bt306State == 1)
    {  //previously disabled
        bt306State = 0;
    }
    else if (bt306State == -1)
    {  //not clicked before
        bt306State = 1;
    }
}

void bt307PopEventHandler(void *ptr)
{
    Serial.println("button bt307 (activates sensor detect mode on route 4| [Activate On Sensor Detect]) pressed");
    if (bt307State == 0)
    {  //previously enabled
        bt307State = 1;
    }
    else if (bt307State == 1)
    {  //previously disabled
        bt307State = 0;
    }
    else if (bt307State == -1)
    {  //not clicked before
        bt307State = 1;
    }
}

void bt308PopEventHandler(void *ptr)
{
    Serial.println("button bt308 (activates repeat mode on route 1| [Activate On Repeat]) pressed");
    if (bt308State == 0)
    {  //previously enabled
        bt308State = 1;
    }
    else if (bt308State == 1)
    {  //previously disabled
        bt308State = 0;
    }
    else if (bt308State == -1)
    {  //not clicked before
        bt308 State = 1;
    }
}

void bt309PopEventHandler(void *ptr)
{
    Serial.println("button bt309 (activates repeat mode on route 2| [Activate On Repeat]) pressed");
    if (bt309State == 0)
    {  //previously enabled
        bt309State = 1;
    }
    else if (bt309State == 1)
    {  //previously disabled
        bt309State = 0;
    }
    else if (bt309State == -1)
    {  //not clicked before
        bt309State = 1;
    }
}

void bt310PopEventHandler(void *ptr)
{
    Serial.println("button bt310 (activates repeat mode on route 3| [Activate On Repeat]) pressed");
    if (bt310State == 0)
    {  //previously enabled
        bt310State = 1;
    }
    else if (bt310State == 1)
    {  //previously disabled
        bt310State = 0;
    }
    else if (bt310State == -1)
    {  //not clicked before
        bt310State = 1;
    }
}

void bt311PopEventHandler(void *ptr)
{
    Serial.println("button bt311 (activates repeat mode on route 4| [Activate On Repeat]) pressed");
    if (bt311State == 0)
    {  //previously enabled
        bt311State = 1;
    }
    else if (bt311State == 1)
    {  //previously disabled
        bt311State = 0;
    }
    else if (bt311State == -1)
    {  //not clicked before
        bt311State = 1;
    }
}

/*********************************************************************************************************
** Function name:       setup
** Descriptions:        Initializes Serial
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
** Developer:           Dobot Labs
*********************************************************************************************************/
void setup()
{
    Serial.begin(115200);
    Serial1.begin(115200);
    Serial2.begin(9600);

    page1.attachPush(page1PushEventHandler);
    page2.attachPush(page2PushEventHandler);
    page3.attachPush(page3PushEventHandler);
    page4.attachPush(page4PushEventHandler);

    b100.attachPop(b100PopEventHandler, &b100);
    b101.attachPop(b101PopEventHandler, &b101);
    b102.attachPop(b102PopEventHandler, &b102);
    b103.attachPop(b103PopEventHandler, &b103);
    b104.attachPop(b104PopEventHandler, &b104);
    b105.attachPop(b105PopEventHandler, &b105);
    b200.attachPop(b200PopEventHandler, &b200);
    b201.attachPop(b201PopEventHandler, &b201);
    b202.attachPop(b202PopEventHandler, &b202);
    b203.attachPop(b203PopEventHandler, &b203);
    b204.attachPop(b204PopEventHandler, &b204);
    b205.attachPop(b205PopEventHandler, &b205);
    b206.attachPop(b206PopEventHandler, &b206);
    b207.attachPop(b207PopEventHandler, &b207);
    b208.attachPop(b208PopEventHandler, &b208);
    b209.attachPop(b209PopEventHandler, &b209);
    b210.attachPop(b210PopEventHandler, &b210);
    b400.attachPop(b400PopEventHandler, &b400);
    b401.attachPop(b401PopEventHandler, &b401);
    b402.attachPop(b402PopEventHandler, &b402);
    b403.attachPop(b403PopEventHandler, &b403);
    b404.attachPop(b404PopEventHandler, &b404);
    b405.attachPop(b405PopEventHandler, &b405);
    b406.attachPop(b406PopEventHandler, &b406);
    b407.attachPop(b407PopEventHandler, &b407);
    b408.attachPop(b408PopEventHandler, &b408);
    b409.attachPop(b409PopEventHandler, &b409);

    bt100.attachPop(bt100PopEventHandler, &bt100);
    bt101.attachPop(bt101PopEventHandler, &bt101);
    bt102.attachPop(bt102PopEventHandler, &bt102);
    bt103.attachPop(bt103PopEventHandler, &bt103);
    bt104.attachPop(bt104PopEventHandler, &bt104);
    bt300.attachPop(bt300PopEventHandler, &bt300);
    bt301.attachPop(bt301PopEventHandler, &bt301);
    bt302.attachPop(bt302PopEventHandler, &bt302);
    bt303.attachPop(bt303PopEventHandler, &bt303);
    bt304.attachPop(bt304PopEventHandler, &bt304);
    bt305.attachPop(bt305PopEventHandler, &bt305);
    bt306.attachPop(bt306PopEventHandler, &bt306);
    bt307.attachPop(bt307PopEventHandler, &bt307);
    bt308.attachPop(bt308PopEventHandler, &bt308);
    bt309.attachPop(bt309PopEventHandler, &bt309);
    bt310.attachPop(bt310PopEventHandler, &bt310);
    bt311.attachPop(bt311PopEventHandler, &bt311);

    printf_begin();
    Serial.println(" ");
    Serial.println("===========Serial communication established=================");
    Serial.println(" ");
    //Set Timer Interrupt
    FlexiTimer2::set(100, Serialread);
    FlexiTimer2::start();
}

void display_freeRam()
{
    Serial.print(F("- SRAM left: "));
    Serial.println(freeRam());
}

int freeRam()
{
    extern int __heap_start, *__brkval;
    int v;
    return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}
/*********************************************************************************************************
** Function name:       Serialread
** Descriptions:        import data to rxbuffer
** Input parameters     none:
** Output parameters:   
** Returned value:
** Developer:           Dobot Labs      
*********************************************************************************************************/
void Serialread()
{
    while (Serial1.available())
    {
        uint8_t data = Serial1.read();
        if (RingBufferIsFull(&gSerialProtocolHandler.rxRawByteQueue) == false)
        {
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
int Serial_putc(char c, struct __file *)
{
    Serial.write(c);
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
    fdevopen(&Serial_putc, 0);
}
/*********************************************************************************************************
** Class name:          node
** Descriptions:        Node for the dynamic queue 
** Input parameters:    
** Output parameters:
** Returned value:      
** Developer:           Lieuwe Baron
*********************************************************************************************************/
class node
{
  public:
    int command;
    int pArraySize;
    int pArray[4] = { 9999, 9999, 9999, 9999 };
    //constructs the head
    node()
    {
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
class cmdQueue
{
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
    cmdQueue()
    {
        addIndex = 1;
    }

    void addToQueue(int command, int p[])
    {
        if (queue[maxLength - 1].command == 9999)
        {
            for (int i = (maxLength - 1); i >= 0; i--)
            {
                if (i != 0)
                {
                    queue[i].command = queue[i - 1].command;
                    for (int j = 0; j <= 3; j++)
                    {
                        queue[i].pArray[j] = queue[i - 1].pArray[j];
                    }
                }
                else
                {
                    queue[i].command = command;
                    for (int j = 0; j <= 3; j++)
                    {
                        queue[i].pArray[j] = p[j];
                    }
                }
            }
        }
    }

    void removeFromQueue(int index)
    {
        queue[index].command = 9999;
        for (int j = 0; j <= 3; j++)
        {
            queue[index].pArray[j] = 9999;
        }
    }

    int *getNextInQueueAndRemoveIt()
    {
        int *nextInQueueValueArray = new int[5];
        for (int i = 0; i <= 5; i++)
        {
            nextInQueueValueArray[i] = 9999;
        }
        for (int i = (maxLength - 1); i >= 0; i--)
        {
            if (queue[i].command != 9999)
            {
                nextInQueueValueArray[0] = queue[i].command;
                for (int j = 1; j <= 4; j++)
                {
                    nextInQueueValueArray[j] = queue[i].pArray[j - 1];
                }
                removeFromQueue(i);
                break;
            }
        }
        return nextInQueueValueArray;
    }

    void printQueue()
    {
        Serial.println("=================================");
        Serial.println("Printing the Command Queue");
        for (int i = 0; i < maxLength; i++)
        {
            Serial.print("index ");
            Serial.print(i);
            Serial.print(": ");
            Serial.print("Command: ");
            Serial.print(queue[i].command);
            Serial.print("; parameters: ");
            Serial.print(queue[i].pArray[0]);
            Serial.print(", ");
            Serial.print(queue[i].pArray[1]);
            Serial.print(", ");
            Serial.print(queue[i].pArray[2]);
            Serial.print(", ");
            Serial.print(queue[i].pArray[3]);
            Serial.println(";");
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
void queueExecutor(int firstInQueueID)
{
    //only execute switch statement if the delay between commands is over
    if (currentMillis >= dynamicDelayMillis)
    {
        //this switch statement ensures that only one item from the queue can be handled in each iteration of the loop
        switch (firstInQueueID)
        {
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

void moveDobotToPos(float x, float y, float z, float r)
{
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
    Serial.print("move to x:");
    Serial.print(gPTPCmd.x);
    Serial.print(" y:");
    Serial.print(gPTPCmd.y);
    Serial.print(" z:");
    Serial.println(gPTPCmd.r);
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
void moveDobotByIncrement(float x, float y, float z, float r)
{
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
    Serial.print("move to x:");
    Serial.print(gPTPCmd.x);
    Serial.print(" y:");
    Serial.print(gPTPCmd.y);
    Serial.print(" z:");
    Serial.println(gPTPCmd.r);
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
void saveRoute()
{
}
/*********************************************************************************************************
** Function name:       replayRoute
** Descriptions:        replays a user created route
** Input parameters:    route
** Output parameters:   none
** Returned value:      none
** Developer:           Lieuwe Baron
*********************************************************************************************************/
void replayRoute()
{
}
/*********************************************************************************************************
** Function name:       suctionCupEnable
** Descriptions:        Enables/Disables the suction cup of the Dobot arm
** Input parameters:    bool suctionEnable
** Output parameters:   none
** Returned value:      none
** Developer:           Lieuwe Baron
*********************************************************************************************************/
void suctionCupEnable(bool suctionEnable)
{
    if (suctionEnable == true)
    {
        SetEndEffectorSuctionCup(true, true, &gQueuedCmdIndex);
        suctionCurrentlyOn = suctionEnable;
    }
    else
    {
        if (suctionEnable == false)
        {
            SetEndEffectorSuctionCup(false, true, &gQueuedCmdIndex);
            suctionCurrentlyOn = suctionEnable;
        }
    }
    ProtocolProcess();
    Serial.println("function suctionCupEnable() called");
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



    gPTPCoordinateParams.xyzVelocity = 100;
    gPTPCoordinateParams.rVelocity = 100;
    gPTPCoordinateParams.xyzAcceleration = 80;
    gPTPCoordinateParams.rAcceleration = 80;

    gPTPCommonParams.velocityRatio = 50;
    gPTPCommonParams.accelerationRatio = 50;

    gPTPCmd.ptpMode = MOVL_XYZ;
    gPTPCmd.x = 0;
    gPTPCmd.y = 0;
    gPTPCmd.z = 0;
    gPTPCmd.r = 0;

    gQueuedCmdIndex = 0;
}

/*********************************************************************************************************
** Function name:       loop
** Descriptions:        loops routine of the dobot
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
** Developer            Lieuwe Baron
*********************************************************************************************************/

void loop()
{
    InitRAM();

    ProtocolInit();
    SetJOGJointParams(&gJOGJointParams, true, &gQueuedCmdIndex);
    SetJOGCoordinateParams(&gJOGCoordinateParams, true, &gQueuedCmdIndex);
    SetJOGCommonParams(&gJOGCommonParams, true, &gQueuedCmdIndex);
    cmdQueue cmdsQueue;
    Serial.println("initialize loop");
    /*int params[4] = { 1760, 290, 3650, 9999 };
  int params2[4] = { 100, 200, 9999, 9999 };
  int params3[4] = { 1000, 2000, 3000, 4000 };
  int params4[4] = { 8675, 34, 645, 9999 };
  int params5[4] = { 68, 564, 6548, 4333 };
  cmdsQueue.addToQueue(8, params);
  cmdsQueue.addToQueue(54, params2);
  cmdsQueue.addToQueue(3, params3);
  cmdsQueue.addToQueue(2, params4);
  cmdsQueue.addToQueue(5, params5);
  cmdsQueue.printQueue();
  cmdsQueue.addToQueue(5, params5);
  cmdsQueue.addToQueue(98, params3);
  cmdsQueue.addToQueue(6576, params3);
  cmdsQueue.addToQueue(786, params3);
  cmdsQueue.printQueue();
  int *output = cmdsQueue.getNextInQueueAndRemoveIt();
  //int *output2 = cmdsQueue.getNextInQueueAndRemoveIt();
  //int *output3 = cmdsQueue.getNextInQueueAndRemoveIt();
  //int *output4 = cmdsQueue.getNextInQueueAndRemoveIt();
  Serial.print("next in array: ");
  Serial.print("Command: ");
  Serial.print(output[0]);
  Serial.print(" Parameters: ");
  for (int i = 1; i <= 5; i++) {
    Serial.print(output[i]);
    Serial.print(", ");
  }
  cmdsQueue.printQueue();*/

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
    for (;;)
    {
        nexLoop(nex_listen_list);
        //set currentMillis to millis(), which is the time the board has been running in ms
        //this is how we keep track of time and delays
        currentMillis = millis();
        //handler for inputs, puts the identifier of the command for the dobot associated with a specified button to the queue
    }
}
