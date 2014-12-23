#pragma once
#include "ofMain.h"

class HandOnEvent : public ofEventArgs {
    
public:
    
    unsigned int handLabel;
    vector<ofVec3f> quad;
    
    HandOnEvent() {
        handLabel = 0;
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

