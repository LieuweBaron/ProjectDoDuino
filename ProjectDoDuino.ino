#include "stdio.h"
#include "Protocol.h"
#include "command.h"
#include "FlexiTimer2.h"
#include <Nextion.h>
#include <EEPROM.h>

//Set Serial TX&RX Buffer Size
#define SERIAL_TX_BUFFER_SIZE 64
#define SERIAL_RX_BUFFER_SIZE 256

EndEffectorParams gEndEffectorParams;

JOGJointParams gJOGJointParams;
JOGCoordinateParams gJOGCoordinateParams;
JOGCommonParams gJOGCommonParams;
JOGCmd gJOGCmd;

PTPCoordinateParams gPTPCoordinateParams;
PTPCommonParams gPTPCommonParams;
PTPCmd gPTPCmd;

uint64_t gQueuedCmdIndex;
bool homed = 0;
bool sensorState = false;
int suction = 0;
int sensorInUse = 0;
int loopInUse = 0;
int currentRoute = 0;
float moveIncrement = 0;
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

String displayText = "...";
//pages on nextion screen
NexPage page1 = NexPage(0, 0, "page1");  //page 0, reffered to as page 1, Controls page
NexPage page2 = NexPage(1, 0, "page2");  //page 1, reffered to as page 2, Routes page
NexPage page3 = NexPage(2, 0, "page3");  //page 2, reffered to as page 3, Use Routes page
NexPage page4 = NexPage(3, 0, "page4");  //page 3, reffered to as page 4, Queue page
NexPage page5 = NexPage(4, 0, "page5");  //page 0, reffered to as page 5, Loading page
NexPage page6 = NexPage(5, 0, "page6");  //page 0, reffered to as page 6, Homing page

//buttons on nextion screen
NexButton b100 = NexButton(0, 17, "b100");  //button that moves X in the + direction, page 1
NexButton b101 = NexButton(0, 18, "b101");  //button that moves X in the - direction, page 1
NexButton b102 = NexButton(0, 19, "b102");  //button that moves Y in the + direction, page 1
NexButton b103 = NexButton(0, 20, "b103");  //button that moves Y in the - direction, page 1
NexButton b104 = NexButton(0, 21, "b104");  //button that moves Z in the + direction, page 1
NexButton b105 = NexButton(0, 22, "b105");  //button that moves Z in the - direction, page 1

NexButton b200 = NexButton(1, 21, "b200");  //button that adds new point to trajectory, page 2
NexButton b201 = NexButton(1, 4, "b201");   //button that saves trajectory to route 1, page 2
NexButton b202 = NexButton(1, 5, "b202");   //button that saves trajectory to route 2, page 2
NexButton b203 = NexButton(1, 6, "b203");   //button that saves trajectory to route 3, page 2
NexButton b204 = NexButton(1, 7, "b204");   //button that saves trajectory to route 4, page 2
NexButton b205 = NexButton(1, 22, "b205");  //button that removes point in the trajectory, page 2

NexButton b400 = NexButton(3, 2, "b400");  //button that stops route from executing, page 4

NexButton b600 = NexButton(5, 2, "b600");  //button that homing has been completed by the user, page 6

//dual-state buttons on nextion screen
NexDSButton bt100 = NexDSButton(0, 23, "bt100");  //enable or disable suction cup
NexDSButton bt101 = NexDSButton(0, 10, "bt101");  //increment of movement: 0.1, page 1
NexDSButton bt102 = NexDSButton(0, 11, "bt102");  //increment of movement: 1, page 1
NexDSButton bt103 = NexDSButton(0, 12, "bt103");  //increment of movement: 10, page 1
NexDSButton bt104 = NexDSButton(0, 13, "bt104");  //increment of movement: 50, page 1

NexDSButton bt300 = NexDSButton(2, 4, "bt300");   //button that activates route 1, page 3
NexDSButton bt301 = NexDSButton(2, 5, "bt301");   //button that activates route 2, page 3
NexDSButton bt302 = NexDSButton(2, 6, "bt302");   //button that activates route 3, page 3
NexDSButton bt303 = NexDSButton(2, 7, "bt303");   //button that activates route 4, page 3
NexDSButton bt304 = NexDSButton(2, 8, "bt304");   //button that activates detect sensor mode on route 1, page 3
NexDSButton bt305 = NexDSButton(2, 11, "bt305");  //button that activates detect sensor mode on route 2, page 3
NexDSButton bt306 = NexDSButton(2, 13, "bt306");  //button that activates detect sensor mode on route 3, page 3
NexDSButton bt307 = NexDSButton(2, 15, "bt307");  //button that activates detect sensor mode on route 4, page 3
NexDSButton bt308 = NexDSButton(2, 9, "bt308");   //button that activates repeat mode on route 1, page 3
NexDSButton bt309 = NexDSButton(2, 10, "bt309");  //button that activates repeat mode on route 2, page 3
NexDSButton bt310 = NexDSButton(2, 12, "bt310");  //button that activates repeat mode on route 3, page 3
NexDSButton bt311 = NexDSButton(2, 14, "bt311");  //button that activates repeat mode on route 4, page 3
//text fields on nextion screen
NexText t100 = NexText(0, 1, "t100");   //current X position, page 1
NexText t103 = NexText(0, 2, "t103");   //current Y position, page 1
NexText t106 = NexText(0, 3, "t106");   //current Z position, page 1
NexText t107 = NexText(0, 18, "t107");  //current Z position, page 1

NexText t200 = NexText(1, 8, "t200");   //current X, page 2
NexText t201 = NexText(1, 10, "t201");  //current Y, page 2
NexText t202 = NexText(1, 9, "t202");   //current Z, page 2
NexText t203 = NexText(1, 11, "t203");  //current SC (suction cup state), page 2
NexText t205 = NexText(1, 13, "t205");  //point 1 in trajectory (start point), page 2
NexText t206 = NexText(1, 14, "t206");  //point 2 in trajectory, page 2
NexText t207 = NexText(1, 15, "t207");  //point 3 in trajectory, page 2
NexText t208 = NexText(1, 16, "t208");  //point 4 in trajectory, page 2
NexText t209 = NexText(1, 17, "t209");  //point 5 in trajectory, page 2
NexText t210 = NexText(1, 18, "t210");  //point 6 in trajectory, page 2
NexText t211 = NexText(1, 19, "t211");  //point 7 in trajectory, page 2
NexText t212 = NexText(1, 20, "t212");  //point 8 in trajectory (end point), page 2

NexText t400 = NexText(3, 1, "t400");  //route playing

NexTouch *nex_listen_list[] = {
  &page1,
  &page2,
  &page3,
  &page4,
  &page5,
  &page6,

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
  &b400,
  &b600,

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

struct params {
  int command = 9999;
  int param1 = 9999;
  int param2 = 9999;
  int param3 = 9999;
};

class cmdQueue {
  //index that decides at which location a new item has to be added to the queue
  int addIndex;
  //index that decides at which location a new item has to be removed from the queue
  int removeIndex;
  //length of the queue
  int maxLength = 20;
  //current length of the queue
  int currentLength;
  //pointer to the location of the first item in the queue
  params queue[20];

public:
  //constuctor for queue
  cmdQueue() {
    addIndex = 1;
  }

  void addToQueue(params newParameters) {
    if (queue[maxLength - 1].command == 9999) {
      for (int i = (maxLength - 1); i >= 0; i--) {
        if (i != 0) {
          queue[i].command = queue[i - 1].command;
          queue[i].param1 = queue[i - 1].param1;
          queue[i].param2 = queue[i - 1].param2;
          queue[i].param3 = queue[i - 1].param3;

        } else {
          queue[i] = newParameters;
        }
      }
    }
  }

  void removeFromQueue(int index) {
    queue[index].command = 9999;
    queue[index].param1 = 9999;
    queue[index].param2 = 9999;
    queue[index].param3 = 9999;
  }

  void clearQueue() {
    for (int i = 0; i <= maxLength; i++) {
      queue[i].command = 9999;
      queue[i].param1 = 9999;
      queue[i].param2 = 9999;
      queue[i].param3 = 9999;
    }
  }

  int getNextInQueueIndex() {
    int nextIndex = 9999;
    for (int i = (maxLength - 1); i >= 0; i--) {
      if (queue[i].command != 9999) {
        nextIndex = i;
        break;
      }
    }
    return nextIndex;
  }


  params getNextInQueueValues() {
    params parameters;
    for (int i = (maxLength - 1); i >= 0; i--) {
      if (queue[i].command != 9999) {
        parameters = queue[i];
        break;
      }
    }
    return parameters;
  }

  params getQueueValuesOfIndex(int index) {
    return queue[index];
  }

  void compressQueue() {
    int emptySlots[10] = { 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999 };
    int totalEmpty = 0;
    //find the indexes at which a slot is empty (9999)
    for (int queueIndex = 0; queueIndex < (maxLength - 1); queueIndex++) {
      if (queue[queueIndex].command == 9999) {
        for (int emptySlotsIndex = 0; emptySlotsIndex < 9; emptySlotsIndex++) {
          if (emptySlots[emptySlotsIndex] == 9999) {
            emptySlots[emptySlotsIndex] = queueIndex;
            totalEmpty++;
            break;
          }
        }
      }
    }
    //move empty slots to the back of the queue, starting with the highest index empty slot
    //this is achieved by swapping with the next item in the queue until either the end or another empty slot (9999) is found
    for (int emptySlotsIndex = totalEmpty - 1; emptySlotsIndex >= 0; emptySlotsIndex--) {
      int currentQueueIndex = emptySlots[emptySlotsIndex];
      while (!(currentQueueIndex >= maxLength - 1) && !(queue[currentQueueIndex + 1].command == 9999)) {
        queue[currentQueueIndex] = queue[currentQueueIndex + 1];
        queue[currentQueueIndex + 1].command = 9999;
        queue[currentQueueIndex + 1].param1 = 9999;
        queue[currentQueueIndex + 1].param2 = 9999;
        queue[currentQueueIndex + 1].param3 = 9999;
        currentQueueIndex++;
      }
    }
  }

  void printQueue() {
    Serial.println("=================================");
    Serial.println("Printing the Command Queue");
    for (int i = 0; i < maxLength; i++) {
      Serial.print("index ");
      Serial.print(i);
      Serial.print(": ");
      Serial.print("Command: ");
      Serial.print(queue[i].command);
      Serial.print("; parameters: ");
      Serial.print(queue[i].param1);
      Serial.print(", ");
      Serial.print(queue[i].param2);
      Serial.print(", ");
      Serial.print(queue[i].param3);
      Serial.println(";");
    }
    Serial.println("=================================");
  }
};

cmdQueue queue;

struct point {
  int x = 9999;
  int y = 9999;
  int z = 9999;
  int suction = 0;
};

class savedRoute {
  point route[8];
  int routeLength = 8;
  int addIndex = 0;
public:
  savedRoute() {
    point defaultPos;
    defaultPos.x = 122;
    defaultPos.y = -2;
    defaultPos.z = -42;
    defaultPos.suction = 0;
    route[0] = defaultPos;
    route[7] = defaultPos;
    addIndex = 1;
  }

  void addPoint(int x, int y, int z, int suction) {
    if (!(addIndex >= 7)) {
      route[addIndex].x = x;
      route[addIndex].y = y;
      route[addIndex].z = z;
      route[addIndex].suction = suction;
      Serial.print("addIndex: ");
      Serial.println(addIndex);
      addIndex += 1;
    } else {
      Serial.println("route full");
    }
  }

  void removeLastAddedPoint() {
    addIndex -= 1;
    route[addIndex].x = 9999;
    route[addIndex].y = 9999;
    route[addIndex].z = 9999;
    route[addIndex].suction = 0;
  }

  String getStringifiedPoint(int index) {
    point sPoint = route[index];
    String stringifiedPoint = String((sPoint.x), 1) + "|" + String((sPoint.y), 1) + "|" + String((sPoint.z), 1) + "|" + String((sPoint.suction), 1);
    return stringifiedPoint; 
  }

  //push route to queue
  void executeRoute() {
    for (int i = 0; i <= 7; i++) {
      params parameters1;
      params parameters2;
      parameters1.command = 3001;
      parameters1.param1 = route[i].x;
      parameters1.param2 = route[i].y;
      parameters1.param3 = route[i].z;

      parameters2.command = 3002;
      parameters2.param1 = route[i].suction;

      queue.addToQueue(parameters1);
      queue.addToQueue(parameters2);
    }
    if (loopInUse == 1) {
      //push command to execute again to queue
    }
  }

  void saveRouteToEEPROM(int routeNumber) {
    if (routeNumber == 1) {
      int index = 1;
      for (int i = 0; i <= 120; i += 20) {
        point readPoint;
        EEPROM.get(i, readPoint);
        route[index] = readPoint;
        index++;
      }
    } else if (routeNumber == 2) {
      int index = 1;
      for (int i = 140; i <= 260; i += 20) {
        point readPoint;
        EEPROM.get(i, readPoint);
        route[index] = readPoint;
        index++;
      }
    } else if (routeNumber == 3) {
      int index = 1;
      for (int i = 280; i <= 400; i += 20) {
        point readPoint;
        EEPROM.get(i, readPoint);
        route[index] = readPoint;
        index++;
      }
    } else if (routeNumber == 4) {
      int index = 1;
      for (int i = 420; i <= 540; i += 20) {
        point readPoint;
        EEPROM.get(i, readPoint);
        route[index] = readPoint;
        index++;
      }
    }
  }

  void getRouteFromEEPROM(int routeNumber) {
    if (routeNumber == 1) {
      int index = 1;
      for (int i = 0; i <= 120; i += 20) {
        EEPROM.put(i, route[index]);
        index++;
      }
    } else if (routeNumber == 2) {
      int index = 1;
      for (int i = 140; i <= 260; i += 20) {
        EEPROM.put(i, route[index]);
        index++;
      }
    } else if (routeNumber == 3) {
      int index = 1;
      for (int i = 280; i <= 400; i += 20) {
        EEPROM.put(i, route[index]);
        index++;
      }
    } else if (routeNumber == 4) {
      int index = 1;
      for (int i = 420; i <= 540; i += 20) {
        EEPROM.put(i, route[index]);
        index++;
      }
    }
  }

  void printRoute() {
    Serial.println("=========================================");
    for (int i = 0; i <= 7; i++) {
      Serial.print("point ");
      Serial.print(i);
      Serial.print(": x: ");
      Serial.print(route[i].x);
      Serial.print(", y: ");
      Serial.print(route[i].y);
      Serial.print(", z: ");
      Serial.print(route[i].z);
      Serial.print(", suction: ");
      Serial.print(route[i].suction);
      Serial.println(";");
    }
    Serial.println("=========================================");
  }
};

savedRoute tempRoute;
savedRoute route1;
savedRoute route2;
savedRoute route3;
savedRoute route4;

void suck(bool suckIt) {
  if (suckIt == true) {
    digitalWrite(30, HIGH);
  } else if (suckIt == false) {
    digitalWrite(30, LOW);
  }
}

bool detectSensor() {
  int sensorDetects = digitalRead(33);
  if (sensorDetects == 1) {
    return true;
  } else if (sensorDetects == 0) {
    return false;
  }
}

//page change event handlers
void page1PushEventHandler(void *ptr) {
  //Serial.println("Page 1");
  currentPage = 1;
  updateScreen();
}

void page2PushEventHandler(void *ptr) {
  //Serial.println("Page 2");
  currentPage = 2;
  updateScreen();
}
void page3PushEventHandler(void *ptr) {
  //Serial.println("Page 3");
  currentPage = 3;
  updateScreen();
}

void page4PushEventHandler(void *ptr) {
  //Serial.println("Page 4");
  currentPage = 4;
  updateScreen();
}

void page5PushEventHandler(void *ptr) {
  //Serial.println("Page 5");
  currentPage = 5;
}

void page6pushEventHandler(void *ptr) {
  //Serial.println("Page 6");
  currentPage = 6;
}

/*
commands:
-1001: move dobot in positive direction
-1002: move dobot in negative direction
-1003: enable/disable suction cup
*/

//button event handlers
void b100PopEventHandler(void *ptr) {
  //Serial.println("button b100 (move Dobot in +X Direction | [+X]) pressed");
  params newParams;
  newParams.command = 1001;
  newParams.param1 = moveIncrement;
  newParams.param2 = 0;
  newParams.param3 = 0;
  queue.addToQueue(newParams);
  //queue.printQueue();
}

void b101PopEventHandler(void *ptr) {
  //Serial.println("button b101 (move Dobot in -X Direction | [-X]) pressed");
  params newParams;
  newParams.command = 1002;
  newParams.param1 = moveIncrement;
  newParams.param2 = 0;
  newParams.param3 = 0;
  queue.addToQueue(newParams);
  //queue.printQueue();
}

void b102PopEventHandler(void *ptr) {
  //Serial.println("button b102 (move Dobot in +Y Direction | [+Y]) pressed");
  params newParams;
  newParams.command = 1001;
  newParams.param1 = 0;
  newParams.param2 = moveIncrement;
  newParams.param3 = 0;
  queue.addToQueue(newParams);
  //queue.printQueue();
}

void b103PopEventHandler(void *ptr) {
  //Serial.println("button b103 (move Dobot in -Y Direction | [-Y]) pressed");
  params newParams;
  newParams.command = 1002;
  newParams.param1 = 0;
  newParams.param2 = moveIncrement;
  newParams.param3 = 0;
  queue.addToQueue(newParams);
  //queue.printQueue();
}

void b104PopEventHandler(void *ptr) {
  //Serial.println("button b104 (move Dobot in +Z Direction | [+Z]) pressed");
  params newParams;
  newParams.command = 1001;
  newParams.param1 = 0;
  newParams.param2 = 0;
  newParams.param3 = moveIncrement;
  queue.addToQueue(newParams);
  //queue.printQueue();
}
void b105PopEventHandler(void *ptr) {
  //Serial.println("button b105 (move Dobot in -Z Direction | [-Z]) pressed");
  params newParams;
  newParams.command = 1002;
  newParams.param1 = 0;
  newParams.param2 = 0;
  newParams.param3 = moveIncrement;
  queue.addToQueue(newParams);
  //queue.printQueue();
}

void b200PopEventHandler(void *ptr) {
  //Serial.println("button b200 (add point to trajectory | [Add Point To Trajecory]) pressed");
  tempRoute.addPoint(gPTPCmd.x, gPTPCmd.y, gPTPCmd.z, suction);
  tempRoute.printRoute();
}

void b201PopEventHandler(void *ptr) {
  //Serial.println("button b201 (save trajectory to route 1 | [Save Route 1]) pressed");
  tempRoute.saveRouteToEEPROM(1);
}

void b202PopEventHandler(void *ptr) {
  //Serial.println("button b202 (save trajectory to route 2 | [Save Route 2]) pressed");
  tempRoute.saveRouteToEEPROM(2);
}

void b203PopEventHandler(void *ptr) {
  //Serial.println("button b203 (save trajectory to route 3 | [Save Route 3]) pressed");
  tempRoute.saveRouteToEEPROM(3);
}

void b204PopEventHandler(void *ptr) {
  //Serial.println("button b204 (save trajectory to route 4 | [Save Route 4]) pressed");
  tempRoute.saveRouteToEEPROM(4);
}

void b205PopEventHandler(void *ptr) {
  //Serial.println("button b205 (remove point  in trajectory | [Remove point from trajectory]) pressed");
  tempRoute.removeLastAddedPoint();
}

void b400PopEventHandler(void *ptr) {
  page1.show();
  //Serial.println("stop route");
}

void b600PopEventHandler(void *ptr) {
  //Serial.println("button b600 pressed");
  params newParams;
  newParams.command = 1002;
  newParams.param1 = 67;
  newParams.param2 = 2;
  newParams.param3 = 42;
  queue.addToQueue(newParams);
  //currX = 189;
  //currY = 0;
  //currZ = 0;
  //int params[4] = { -67, -2, -42, 500 };  //{ 181.8, -3, -41.4, 500 };
  //Serial.println(gPTPCmd.x);
  //Serial.println(gPTPCmd.y);
  //Serial.println(gPTPCmd.z);

  homed = 1;
}

//dual-state buttons event handlers
//pressing this button determines whether the suction cup is on or off
void bt100PopEventHandler(void *ptr) {
  uint32_t dual_state;
  bt100.getValue(&dual_state);
  if (dual_state) {
    params newParams;
    newParams.command = 1003;
    newParams.param1 = 1;
    newParams.param2 = 0;
    newParams.param3 = 0;
    queue.addToQueue(newParams);
    suction = 1;
    Serial.println("on");

  } else {
    params newParams;
    newParams.command = 1003;
    newParams.param1 = 0;
    newParams.param2 = 0;
    newParams.param3 = 0;
    queue.addToQueue(newParams);
    suction = 0;
    Serial.println("off");
  }
  //Serial.println("button bt100 (enable/disable suction cup | [Suction Cup]) pressed");
}

void bt101PopEventHandler(void *ptr) {
  //Serial.println("button bt101 (movement increment 0.1 | [0.1]) pressed");
  moveIncrement = 0.1;
  uint32_t dual_state;
  bt101.getValue(&dual_state);
  if (dual_state) {
    moveIncrement = 1;
  } else {
    moveIncrement = 0;
  }
}

void bt102PopEventHandler(void *ptr) {
  //Serial.println("button bt102 (movement increment 1 | [1]) pressed");
  moveIncrement = 1;
  uint32_t dual_state;
  bt102.getValue(&dual_state);
  if (dual_state) {
    moveIncrement = 10;
  } else {
    moveIncrement = 0;
  }
}

void bt103PopEventHandler(void *ptr) {
  //Serial.println("button bt103 (movement increment 10 | [10]) pressed");
  uint32_t dual_state;
  bt103.getValue(&dual_state);
  if (dual_state) {
    moveIncrement = 50;
  } else {
    moveIncrement = 0;
  }
}

void bt104PopEventHandler(void *ptr) {
  //Serial.println("button bt104 (movement increment 50 | [50]) pressed");
  uint32_t dual_state;
  bt104.getValue(&dual_state);
  if (dual_state) {
    moveIncrement = 100;
  } else {
    moveIncrement = 0;
  }
}

void bt300PopEventHandler(void *ptr) {
  //Serial.println("button bt300 (activates route 1| [Activate Route 1]) pressed");
  uint32_t dual_state;
  bt300.getValue(&dual_state);
  if (dual_state) {
    currentRoute = 1;
  } else {
    currentRoute = 0;
  }
}

void bt301PopEventHandler(void *ptr) {
  //Serial.println("button bt301 (activates route 2| [Activate Route 2]) pressed");
  uint32_t dual_state;
  bt301.getValue(&dual_state);
  if (dual_state) {
    currentRoute = 2;
  } else {
    currentRoute = 0;
  }
}

void bt302PopEventHandler(void *ptr) {
  //Serial.println("button bt302 (activates route 3| [Activate Route 3]) pressed");
  uint32_t dual_state;
  bt302.getValue(&dual_state);
  if (dual_state) {
    currentRoute = 3;
  } else {
    currentRoute = 0;
  }
}

void bt303PopEventHandler(void *ptr) {
  //Serial.println("button bt303 (activates route 4| [Activate Route 3]) pressed");
  //temperary enables/disables suction cup on loop;
  uint32_t dual_state;
  int count = 0;
  bt303.getValue(&dual_state);
  if (dual_state) {
    currentRoute = 4;
  } else {
    currentRoute = 0;
  }
}

void bt304PopEventHandler(void *ptr) {
  //Serial.println("button bt304 (activates sensor detect mode on route 1| [Activate On Sensor Detect]) pressed");
  uint32_t dual_state;
  bt304.getValue(&dual_state);
  if (dual_state) {
    sensorInUse = 1;
    loopInUse = 0;
  } else {
    sensorInUse = 0;
  }
}

void bt305PopEventHandler(void *ptr) {
  //Serial.println("button bt305 (activates sensor detect mode on route 2| [Activate On Sensor Detect]) pressed");
  uint32_t dual_state;
  bt305.getValue(&dual_state);
  if (dual_state) {
    sensorInUse = 1;
    loopInUse = 0;
  } else {
    sensorInUse = 0;
  }
}

void bt306PopEventHandler(void *ptr) {
  //Serial.println("button bt306 (activates sensor detect mode on route 3| [Activate On Sensor Detect]) pressed");
  uint32_t dual_state;
  bt306.getValue(&dual_state);
  if (dual_state) {
    sensorInUse = 1;
    loopInUse = 0;
  } else {
    sensorInUse = 0;
  }
}

void bt307PopEventHandler(void *ptr) {
  //Serial.println("button bt307 (activates sensor detect mode on route 4| [Activate On Sensor Detect]) pressed");
  uint32_t dual_state;
  bt307.getValue(&dual_state);
  if (dual_state) {
    sensorInUse = 1;
    loopInUse = 0;
  } else {
    sensorInUse = 0;
  }
}

void bt308PopEventHandler(void *ptr) {
  //Serial.println("button bt308 (activates repeat mode on route 1| [Activate On Repeat]) pressed");
  uint32_t dual_state;
  bt308.getValue(&dual_state);
  if (dual_state) {
    loopInUse = 1;
    sensorInUse = 0;
  } else {
    loopInUse = 0;
  }
}

void bt309PopEventHandler(void *ptr) {
  //Serial.println("button bt309 (activates repeat mode on route 2| [Activate On Repeat]) pressed");
  uint32_t dual_state;
  bt309.getValue(&dual_state);
  if (dual_state) {
    loopInUse = 1;
    sensorInUse = 0;
  } else {
    loopInUse = 0;
  }
}

void bt310PopEventHandler(void *ptr) {
  //Serial.println("button bt310 (activates repeat mode on route 3| [Activate On Repeat]) pressed");
  uint32_t dual_state;
  bt310.getValue(&dual_state);
  if (dual_state) {
    loopInUse = 1;
    sensorInUse = 0;
  } else {
    loopInUse = 0;
  }
}

void bt311PopEventHandler(void *ptr) {
  //Serial.println("button bt311 (activates repeat mode on route 4| [Activate On Repeat]) pressed");
  uint32_t dual_state;
  bt311.getValue(&dual_state);
  if (dual_state) {
    loopInUse = 1;
    sensorInUse = 0;
  } else {
    loopInUse = 0;
  }
}

int bounds[32][3] = {
  { 170, 140, 170 },
  { 160, 140, 200 },
  { 150, 140, 210 },
  { 140, 140, 220 },
  { 130, 140, 230 },
  { 120, 140, 240 },
  { 110, 140, 240 },
  { 100, 140, 250 },
  { 90, 130, 250 },
  { 80, 130, 250 },
  { 70, 130, 250 },
  { 60, 120, 260 },
  { 50, 110, 260 },
  { 40, 110, 260 },
  { 30, 100, 260 },
  { 20, 80, 260 },
  { 10, 70, 260 },
  { 0, 50, 260 },
  { -10, 60, 260 },
  { -20, 70, 260 },
  { -30, 90, 260 },
  { -40, 100, 260 },
  { -50, 110, 260 },
  { -60, 120, 260 },
  { -70, 130, 25 },
  { -80, 130, 250 },
  { -90, 140, 250 },
  { -100, 140, 240 },
  { -110, 150, 230 },
  { -120, 150, 210 },
  { -130, 150, 200 },
  { -140, 150, 170 },
};

bool outOfBounds(int x, int y, int z) {
  if (z >= 0) {
    z = floor(z / 10) * 10;
  } else if (z < 0) {
    z = ceil(z / 10) * 10;
  }
  float baseAngle = atan2(y, x) * (180.0 / 3.1415);
  if (!(baseAngle <= 90 && baseAngle >= -90)) {
    Serial.println(", out of bounds");
    return false;
  }
  if (z > 170 || z < -140) {
    Serial.println("out of bounds");
    return false;
  }
  double length = sqrt(pow(x, 2) + pow(y, 2));
  for (int i = 0; i <= 31; i++) {
    if (bounds[i][0] == z) {
      if (length >= bounds[i][1] && length <= bounds[i][2]) {
        Serial.println("in bounds");
        return true;
      } else {
        Serial.println("out of bounds");
        return false;
      }
    }
  }
}

void updateScreen() {
  if (currentPage == 1) {
    displayText = String((gPTPCmd.x), 1);
    Serial.println(displayText.c_str());
    t100.setText(displayText.c_str());
    displayText = String((gPTPCmd.y), 1);
    Serial.println(displayText.c_str());
    t103.setText(displayText.c_str());
    displayText = String((gPTPCmd.z), 1);
    Serial.println(displayText.c_str());
    t106.setText(displayText.c_str());
    return;
  } else if (currentPage == 2) {
    displayText = "X: " + String((gPTPCmd.x), 1);
    t200.setText(displayText.c_str());
    displayText = "Y: " + String((gPTPCmd.y), 1);
    t201.setText(displayText.c_str());
    displayText = "Z: " + String((gPTPCmd.z), 1);
    t202.setText(displayText.c_str());
    switch(suction) {
      case 0:
        displayText = "SC: False";
        break;
      case 1:
        displayText = "SC: True";
        break;
    }
    t203.setText(displayText.c_str());
    displayText = tempRoute.getStringifiedPoint(1);
    t206.setText(displayText.c_str());
    displayText = tempRoute.getStringifiedPoint(2);
    t207.setText(displayText.c_str());
    displayText = tempRoute.getStringifiedPoint(3);
    t208.setText(displayText.c_str());
    displayText = tempRoute.getStringifiedPoint(4);
    t209.setText(displayText.c_str());
    displayText = tempRoute.getStringifiedPoint(5);
    t210.setText(displayText.c_str());
    displayText = tempRoute.getStringifiedPoint(6);
    t211.setText(displayText.c_str());
    return;
  } else if (currentPage == 4) {
    switch(currentRoute) {
      case 1:
        t400.setText("Executing Route: 1");
        break;
      case 2:
        t400.setText("Executing Route: 2");
        break;
      case 3:
        t400.setText("Executing Route: 3");
        break;
      case 4:
        t400.setText("Executing Route: 4");
        break;
    }
    return;
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(9600);
  printf_begin();
  FlexiTimer2::set(100, Serialread);
  FlexiTimer2::start();
  page1.attachPush(page1PushEventHandler);
  page2.attachPush(page2PushEventHandler);
  page3.attachPush(page3PushEventHandler);
  page4.attachPush(page4PushEventHandler);
  page5.attachPush(page5PushEventHandler);
  page6.attachPush(page5PushEventHandler);

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
  b400.attachPop(b400PopEventHandler, &b400);
  b600.attachPop(b600PopEventHandler, &b600);

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
  page5.show();

  pinMode(30, OUTPUT);  // relais output for sucking
  pinMode(32, OUTPUT);  // power for sensor
  pinMode(33, INPUT);   // input for sensor
  digitalWrite(32, HIGH);
}

void Serialread() {
  while (Serial1.available()) {
    uint8_t data = Serial1.read();
    if (RingBufferIsFull(&gSerialProtocolHandler.rxRawByteQueue) == false) {
      RingBufferEnqueue(&gSerialProtocolHandler.rxRawByteQueue, &data);
    }
  }
}

int Serial_putc(char c, struct __file *) {
  Serial.write(c);
  return c;
}

void printf_begin(void) {
  fdevopen(&Serial_putc, 0);
}

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
  gPTPCmd.x = 189;
  gPTPCmd.y = 0;
  gPTPCmd.z = 0;
  gPTPCmd.r = 0;

  gQueuedCmdIndex = 0;
}

void loop() {
  InitRAM();
  ProtocolInit();
  SetJOGJointParams(&gJOGJointParams, true, &gQueuedCmdIndex);
  SetJOGCoordinateParams(&gJOGCoordinateParams, true, &gQueuedCmdIndex);
  SetJOGCommonParams(&gJOGCommonParams, true, &gQueuedCmdIndex);
  int count = 0;
  delay(5000);
  SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);
  ProtocolProcess();
  delay(1000);
  page6.show();

  while (homed != 1) {
    nexLoop(nex_listen_list);
    delay(10);
  }

  printf("\r\n======Enter application======\r\n");
  page1.show();
  for (;;) {
    nexLoop(nex_listen_list);
    int nextCommandIndex = queue.getNextInQueueIndex();
    params nextCommandParams;
    nextCommandParams = queue.getNextInQueueValues();
    int nextCommand = nextCommandParams.command;
    if (currentRoute != 0 && sensorInUse == 1) {
      if (currentPage != 4) {
        page4.show();
      }
      bool sensorOn = detectSensor();
      if (sensorOn == true) {
        params routeToActivate;
        routeToActivate.command = 3004;
        routeToActivate.param1 = currentRoute;
        queue.addToQueue(routeToActivate);
      }
    }
    switch (nextCommand) {
      //command is empty
      case 9999:
        break;
      //this is the command to move the dobot in the positive direction, parameters determine to where
      case 1001:
        if (outOfBounds(gPTPCmd.x + nextCommandParams.param1, gPTPCmd.y + nextCommandParams.param2, gPTPCmd.z + nextCommandParams.param3) == false) {
          queue.removeFromQueue(nextCommandIndex);
          Serial.println("out of bounds");
          break;
        } else {
          gPTPCmd.x += nextCommandParams.param1;
          gPTPCmd.y += nextCommandParams.param2;
          gPTPCmd.z += nextCommandParams.param3;
          gPTPCmd.r += 0;
          SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);
          queue.removeFromQueue(nextCommandIndex);
          if (currentPage == 1 || currentPage == 2 || currentPage == 4) {
            updateScreen();
          }
          ProtocolProcess();
          delay(1000);
          break;
        }
      //this is the command to move the dobot in the negative direction, parameters determine to where
      case 1002:
        if (outOfBounds(gPTPCmd.x - nextCommandParams.param1, gPTPCmd.y - nextCommandParams.param2, gPTPCmd.z - nextCommandParams.param3) == false) {
          queue.removeFromQueue(nextCommandIndex);
          Serial.println("main loop out of bounds");
          break;
        } else {
          gPTPCmd.x -= nextCommandParams.param1;
          gPTPCmd.y -= nextCommandParams.param2;
          gPTPCmd.z -= nextCommandParams.param3;
          gPTPCmd.r -= 0;
          SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);
          queue.removeFromQueue(nextCommandIndex);
          displayText = String((gPTPCmd.x), 1);
          t100.setText(displayText.c_str());
          displayText = String((gPTPCmd.y), 1);
          t103.setText(displayText.c_str());
          displayText = String((gPTPCmd.z), 1);
          t106.setText(displayText.c_str());
          if (currentPage == 1 || currentPage == 2 || currentPage == 4) {
            updateScreen();
          }
          ProtocolProcess();
          delay(1000);
          break;
        }
      //this command enables or disables the suction cup, dependant on the parameters
      case 1003:
        if (nextCommandParams.param1 == 1) {
          SetEndEffectorSuctionCup(true, true, &gQueuedCmdIndex);
          queue.removeFromQueue(nextCommandIndex);
        } else if (nextCommandParams.param1 == 0) {
          SetEndEffectorSuctionCup(false, true, &gQueuedCmdIndex);
          queue.removeFromQueue(nextCommandIndex);
        }
        if (currentPage == 2 || currentPage == 4) {
          updateScreen();
        }
        delay(1000);
        ProtocolProcess();
        break;

      case 3001:
        gPTPCmd.x = nextCommandParams.param1;
        gPTPCmd.y = nextCommandParams.param2;
        gPTPCmd.z = nextCommandParams.param3;
        SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);
        queue.removeFromQueue(nextCommandIndex);
        ProtocolProcess();
        delay(1000);
        break;
      case 3002:
        if (nextCommandParams.param1 == 1) {
          suck(true);
        } else if (nextCommandParams.param1 == 1) {
          suck(false);
        }
        ProtocolProcess();
        delay(1000);
        break;
      case 3003:
        switch (currentRoute) {
          case 1:
            route1.getRouteFromEEPROM(1);
            route1.executeRoute();
            break;
          case 2:
            route1.getRouteFromEEPROM(2);
            route1.executeRoute();
            break;
          case 3:
            route1.getRouteFromEEPROM(3);
            route1.executeRoute();
            break;
          case 4:
            route1.getRouteFromEEPROM(4);
            route1.executeRoute();
            break;
        }
        break;
    }
    delay(201);
  }
}
