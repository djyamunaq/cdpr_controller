#include "MotorController.h"

void epos::MotorController::LogError(string info, unsigned int errorCode) {
    char* errorInfo = new char[1000];
    if(errorCode == -1) {
        cout << "\033[1;31m[" << std::hex << this->errorCode << std::dec << "] " << info << "\033[0m\n";
    } else if(VCS_GetErrorInfo(this->errorCode, errorInfo, 1000) == 0) {
        cout << BOLDRED << "Unknown Error" << RESET << endl;
    } else {
        cout << "\033[1;31m[" << std::hex << this->errorCode << std::dec << "] " << info << ": " << errorInfo << "\033[0m\n";
    }
}

void epos::MotorController::LogInfo(string info) {
    cout << BOLDBLUE << info << RESET << endl;
}

void epos::MotorController::LogDebug(string debugInfo) {
    cout << YELLOW << debugInfo << RESET << endl;
}

epos::MotorController::MotorController() {
}

epos::MotorController::~MotorController() {
}

void epos::MotorController::setMovementProfile(unsigned int nodeId, MovementProfile movementProfile) {
    this->movementProfiles.insert({nodeId, movementProfile});
}

epos::MovementProfile epos::MotorController::getMovementProfile(unsigned int nodeId) {
    if(this->movementProfiles.find(nodeId) == this->movementProfiles.end()) {
        unsigned int velocity, acceleration, deceleration;

        if(VCS_GetPositionProfile(this->deviceHandle, nodeId, &velocity, &acceleration, &deceleration, &(this->errorCode)) == 0) {
            LogError("getMovementProfile", this->errorCode);
        }

        MovementProfile movementProfile;
        movementProfile.velocity = velocity;        
        movementProfile.acceleration = acceleration;
        movementProfile.deceleration = deceleration;

        this->movementProfiles.insert({nodeId, movementProfile});

        return movementProfile;
    }

    return this->movementProfiles.at(nodeId);
}

void epos::MotorController::printMovementProfile(unsigned int nodeId) {

    MovementProfile movementProfile = this->getMovementProfile(nodeId);
    
    string msg = "Movement Profile -> Node " + to_string(nodeId);
    LogInfo(msg);

    msg = "\tVelocity: " + to_string(movementProfile.velocity);
    LogInfo(msg);

    msg = "\tAcceleration: " + to_string(movementProfile.acceleration);
    LogInfo(msg);

    msg = "\tDeceleration: " + to_string(movementProfile.deceleration);
    LogInfo(msg);
}

void epos::MotorController::connect(string deviceName, string protocolName, string interfaceName, string portName, unsigned int baudrate) {
    this->baudrate = baudrate;

    char* c_deviceName = new char[255];
    strcpy(c_deviceName, deviceName.c_str());

    char* c_protocolName = new char[255];
    strcpy(c_protocolName, protocolName.c_str());

    char* c_interfaceName = new char[255];
    strcpy(c_interfaceName, interfaceName.c_str());

    char* c_portName = new char[255];
    strcpy(c_portName, portName.c_str());

    LogInfo("Connecting...");

    /* Open the port to send and receive commands -> successful if != 0 */
	this->deviceHandle = VCS_OpenDevice(c_deviceName, c_protocolName, c_interfaceName, c_portName, &(this->errorCode));

    delete c_deviceName, c_protocolName, c_interfaceName, c_portName;

    if(this->deviceHandle == 0 || this->errorCode != 0) {
        LogError("Connection Error: ", this->errorCode);
        exit(1);
    }

    LogInfo("Successful connected");
}

void epos::MotorController::clearFault(unsigned int nodeId) {
    if(VCS_ClearFault(this->deviceHandle, nodeId, &(this->errorCode)) == 0) {
        LogError("VCS_ClearFault", this->errorCode);
        exit(1);
    }
}

void epos::MotorController::setEnable(unsigned int nodeId) {
    if(VCS_SetEnableState(this->deviceHandle, nodeId, &(this->errorCode)) == 0) {
        cout << "OPA" << endl;
        LogError("VCS_SetEnableState", this->errorCode);
        exit(1);
    }
}


void epos::MotorController::checkMotorState(unsigned int nodeId) {
    unsigned int errorCode, lBaudrate, lTimeout;

    if(VCS_GetProtocolStackSettings(this->deviceHandle, &lBaudrate, &lTimeout, &(this->errorCode)) == 0) {
        LogError("VCS_GetProtocolStackSettings", this->errorCode);
        exit(1);
    }

    if(VCS_SetProtocolStackSettings(this->deviceHandle, this->baudrate, lTimeout, &(this->errorCode)) == 0) {
        LogError("VCS_SetProtocolStackSettings", this->errorCode);
        exit(1);
    }

    int isFault;

    if(VCS_GetFaultState(this->deviceHandle, nodeId, &isFault, &(this->errorCode) ) == 0) {
		LogError("VCS_GetFaultState", this->errorCode);
	}

    if(isFault) {
        string info = "clear fault, node = '" + nodeId;
        LogInfo(info);

        if(VCS_ClearFault(this->deviceHandle, nodeId, &(this->errorCode)) == 0) {
            LogError("VCS_ClearFault", this->errorCode);
            exit(1);
        }
    }
}

void epos::MotorController::activatePositionMode(unsigned int nodeId) {
    if(VCS_ActivatePositionMode(this->deviceHandle, nodeId, &(this->errorCode)) == 0) {
        this->LogError("startMovement->VCS_SetPositionProfile", this->errorCode);
        exit(1);
    }
}

void epos::MotorController::setMaxAcc(unsigned int nodeId, unsigned int acc) {
    if(VCS_SetMaxAcceleration(this->deviceHandle, nodeId, acc, &(this->errorCode)) == 0) {
        this->LogError("startMovement->VCS_SetPositionProfile", this->errorCode);
        exit(1);
    }
}

long epos::MotorController::getPos(unsigned int nodeId) {
    long pos;
    if(VCS_GetPositionMust(this->deviceHandle, nodeId, &pos, &(this->errorCode)) == 0) {
        this->LogError("startMovement->VCS_SetPositionProfile", this->errorCode);
        exit(1);
    }

    return pos;
}

void epos::MotorController::setPos(unsigned int nodeId, long pos) {
    // checkMotorState(nodeId);

    if(VCS_SetPositionMust(this->deviceHandle, nodeId, pos, &(this->errorCode)) == 0) {
        this->LogError("startMovement->VCS_SetPositionProfile", this->errorCode);
        exit(1);
    }
}

void epos::MotorController::activateProfilePositionMode(unsigned int nodeId) {
    if(VCS_ActivateProfilePositionMode(this->deviceHandle, nodeId, &(this->errorCode)) == 0) {
        this->LogError("startMovement->VCS_SetPositionProfile", this->errorCode);
        exit(1);
    }
}

void epos::MotorController::startMovement(unsigned int nodeId, unsigned int pos, unsigned int relAbs, unsigned int immWait) {
    int isFault;


    /* Check if key exists in movement profile map */
    MovementProfile movementProfile = this->getMovementProfile(nodeId);
    
    if(VCS_SetPositionProfile(this->deviceHandle, nodeId, movementProfile.velocity, movementProfile.acceleration, movementProfile.deceleration, &(this->errorCode)) == 0) {
        this->LogError("startMovement->VCS_SetPositionProfile", this->errorCode);
        exit(1);
    }

    if(!VCS_MoveToPosition(this->deviceHandle, nodeId, pos, relAbs, immWait, &(this->errorCode))) {
        string msg = "Error while moving to position " + to_string(pos);
        LogError(msg, -1);
        exit(1);
    }

    sleep(1);
}

void epos::MotorController::stopMovement(unsigned int nodeId) {
    if(VCS_HaltPositionMovement(this->deviceHandle, nodeId, &(this->errorCode)) == 0) {
        LogError("stopMovement", this->errorCode);
        exit(1);
    }
}