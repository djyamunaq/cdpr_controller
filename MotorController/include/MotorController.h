#ifndef _MOTORCONTROLLER_
#define _MOTORCONTROLLER_
    #include <iostream>
    #include <map>
    #include <unistd.h>
    #include <string.h>
    #include "Definitions.h"
#endif

#define RESET       "\033[0m"
#define BLACK       "\033[30m"             /* Black */
#define RED         "\033[31m"             /* Red */
#define GREEN       "\033[32m"             /* Green */
#define YELLOW      "\033[33m"             /* Yellow */
#define BLUE        "\033[34m"             /* Blue */
#define MAGENTA     "\033[35m"             /* Magenta */
#define CYAN        "\033[36m"             /* Cyan */
#define WHITE       "\033[37m"             /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */

#define MAXON_RS232     "MAXON_RS232"
#define MAXON_SERIAL_V2 "MAXON SERIAL V2"
#define CANopen         "CANopen"

#define RELATIVE    0
#define ABSOLUTE    1
#define WAIT        0
#define IMMEDIATE   1
#define FALSE       0
#define TRUE        1

using std::cout;
using std::cin;
using std::endl;
using std::string;
using std::map;
using std::to_string;

typedef void* HANDLE;

namespace epos {
    typedef struct t_movement_profile {
        unsigned int velocity;
        unsigned int acceleration;
        unsigned int deceleration;
    } MovementProfile;

    class MotorController {
        public:
            MotorController();
            ~MotorController();
            /* Connect to Epos2 Card */
            void connect(string deviceName, string protocolName, string interfaceName, string portName, unsigned int baudrate);
            void clearFault(unsigned int nodeId);
            void setEnable(unsigned int nodeId);
            void checkMotorState(unsigned int nodeId);
            void setMaxAcc(unsigned int nodeId, unsigned int acc);
            void activateProfilePositionMode(unsigned int nodeId);
            void activatePositionMode(unsigned int nodeId);
            long getPos(unsigned int nodeId);
            void setPos(unsigned int nodeId, long pos);
            void startMovement(unsigned int nodeId, unsigned int pos, unsigned int relAbs, unsigned int immWait);
            void stopMovement(unsigned int nodeId);
            void setMovementProfile(unsigned int nodeId, MovementProfile movementProfile);
            MovementProfile getMovementProfile(unsigned int nodeId);
            void printMovementProfile(unsigned int nodeId);
        private:
            void LogError(string functionName, unsigned int errorCode);
            void LogInfo(string info);
            void LogDebug(string debugInfo);
            HANDLE deviceHandle{0};
            unsigned int baudrate;
            unsigned int errorCode{0};
            map<unsigned int, MovementProfile> movementProfiles = map<unsigned int, MovementProfile>();
    };
};
