#pragma once
#include "ofMain.h"

class HandOnEvent : public ofEventArgs {
    
public:
    
    unsigned int handLabel;
    ofVec2f center;
    ofVec3f worldCoordinates;
    
    HandOnEvent() {
        handLabel = 0;
        center.x = 0;
        center.y = 0;
        worldCoordinates.x = 0;
        worldCoordinates.y = 0;
        worldCoordinates.z = 0;
    }
    
    static ofEvent <HandOnEvent> events;
};

class HandOutEvent : public ofEventArgs {
    
public:
    
    unsigned int handLabel;
    
    HandOutEvent() {
        handLabel = 0;
    }
    
    static ofEvent <HandOutEvent> events;
};

