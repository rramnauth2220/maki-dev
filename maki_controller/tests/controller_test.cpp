#include "../MAKIv14.h"
#define Serial cout

#include <sys/time.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <stdio.h>
using namespace std;

typedef unsigned char byte;

int lastErrorCheckMillis = 0;
int millis() {
    struct timeval now;
    if (gettimeofday(&now, NULL)) {
        // error
        return -1;
    }
    return now.tv_sec * 1000 + now.tv_usec / 1000;
}

char receiveBuffer[100];
unsigned int receiveBufferI = 0;
byte lastErrors[SERVO_COUNT];

// Gets the ID of the servo from the abbreviation made by the first two characters of message
int getIDFromAbbreviation(const char* message, const char* nameAbbreviationsString) {
    char abbreviation[3];
    strncpy(abbreviation, message, 2); // So we can have a 2-char null-terminated string
    abbreviation[2] = 0;
    const char* foundAbbreviation = strstr(nameAbbreviationsString, abbreviation); // gives pointer within nameAbbreviationsString
    if (foundAbbreviation) {
        return (foundAbbreviation - nameAbbreviationsString) / 3 + 1;
    }
    return -1;
}

// Gets the ID of the servo from the abbreviation made by the first two characters of message
int getServoID(const char* message) {
    return getIDFromAbbreviation(message, SERVO_NAMES_STR);
}

// Gets the ID of the feedback type from the abbreviation made by the first two characters of message
int getFeedbackType(const char* message) {
    return getIDFromAbbreviation(message, FEEDBACK_NAMES_STR);
}

// Gets the ID of the parameter set type from the abbreviation made by the first two characters of message
int getSetType(const char* message) {
    return getIDFromAbbreviation(message, SET_NAMES_STR);
}

void reportOn(int feedbackType, const char* feedbackAbbreviation) {
    Serial << feedbackAbbreviation[0] << feedbackAbbreviation[1]; // The feedback type 
    for (byte i = 0; i < SERVO_COUNT; i++) {
        Serial << i * 10;
        if (i < SERVO_COUNT - 1) {
            Serial << ':';
        }
    }
    Serial << ";\n";
}

void parseFeedback() {
    int feedbackType = getFeedbackType(receiveBuffer + 1);
    if (feedbackType == -1) {
        Serial << "Error: Feedback command " << receiveBuffer[1] <<  receiveBuffer[2] << " malformed\n";
        return;
    }
    reportOn(feedbackType, receiveBuffer + 1);
}

void parseSetCommand() {
    // Are we adjusting goal speed for position commands?
    const char* movementTimePtr = strstr(receiveBuffer, MOVEMENT_TIME_STR);
    int movementTimeMs = 0; // milliseconds to spend on all movements in this command, 0 meaning don't regulate this
    if (movementTimePtr) {
        movementTimeMs = atoi(movementTimePtr + sizeof(MOVEMENT_TIME_STR));
        // atoi returns 0 on failure, which we will use again to mean not to use this feature.
        if (movementTimeMs == 0) {
            Serial << "Warning: malformed specification of time to be moving. Ignoring this parameter: " << movementTimePtr << endl;
        }
    }
    
    int lastTargetServo = -1;
    
    const char* command = receiveBuffer;
    while (command[0]) {
        // Reached the end of the commands.
        if (strncmp(command, MOVEMENT_TIME_STR, sizeof(MOVEMENT_TIME_STR) - 1) == 0) {
            return;
        }
        
        int targetServo = getServoID(command);
        if (targetServo != -1) {
            lastTargetServo = targetServo;
            command += 2;
        } else {
            if (lastTargetServo != -1) {
                targetServo = lastTargetServo;
            } else {
                Serial << "Error: Target servo '" << command[0] << command[1] << "' malformed\n";
                return;
            }
        }
        
        int setType = getSetType(command);
        if (setType == -1) {
            Serial << "Error: Parameter to set '" << command[0] << command[1] << "' malformed\n";
            return;
        }
        command += 2;
        
        // Also get a pointer to the end of the number as well as the value of the number
        char* afterNumberPtr;
        int value = strtol(command, &afterNumberPtr, 10);
        
        switch (setType) {
            case SET_POS_ID:
                if (movementTimeMs > 0) {
                    // The unit for speed: ~= .111rpm ~= 2/3 degrees per 1000ms
                    // the unit for position is .29 degrees
                    // so speed is distance * 0.29f / (2 / 3) / (movementTimesMs / 1000)
                    int currentPos = 0;
                    int distance = abs(currentPos - value);
                    // we add movementTimeMs / 2 for rounding
                    int speed = (distance * 435 + movementTimeMs / 2) / movementTimeMs;
                    Serial << "Setting " << " position to " << value << " and speed to " << speed << " (current position = 0) " << endl;
                } else {
                    Serial << "Setting position to " << value << endl;
                }
                break;
            case SET_SPEED_ID:
                Serial << "Setting speed to " << value << endl;
                break;
            case SET_MAX_TORQUE_ID:
                Serial << "Setting max torque to " << value << endl;
                break;
            case SET_TORQUE_LIM_ID:
                Serial << "Setting torque limit to " << value << endl;
                break;
            case SET_TORQUE_ENABLE_ID:
                Serial << "Setting torque enable to " << (bool)value << endl;
                break;
        }
        
        command = afterNumberPtr;
    }
}

void parseCommand() {
    if (receiveBuffer[0] == 'F') {
        parseFeedback();
    } else {
        parseSetCommand();
    }
}

int main() {
    lastErrorCheckMillis = millis();
    
    while (true) {
        // Periodically report on errors
        int now = millis();
        if (now - lastErrorCheckMillis >= ERROR_CHECK_MS) {
            reportOn(ERROR_ID, "ER");
            lastErrorCheckMillis = now;
        }
    
        char newChar = fgetc(stdin);
        if (feof(stdin)) { // CTRL-D clean exit
            break;
        }
        
        // Reset on white-space
        if (newChar == ' ' || newChar == '\n' || newChar == '\r') {
            receiveBufferI = 0;
        } else if (newChar == 'Z') {
            // Command fully received (note we don't add Z to the receiveBuffer)
            // make sure to null-terminated
            receiveBuffer[receiveBufferI] = 0;
            parseCommand();
            receiveBufferI = 0;
        } else {
            if (receiveBufferI + 1 >= sizeof(receiveBuffer)) {
                Serial << "Error: Buffer overflow! Throwing away: " << receiveBuffer << endl;
                receiveBufferI = 0;
            }
            receiveBuffer[receiveBufferI++] = newChar;
        }
    }
    return 0;
}
