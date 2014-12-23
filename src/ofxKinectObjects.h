//
//  ofxKinectObjects.h
//  kinectObjects
//
//  Created by Álvaro Sarasúa Berodia on 17/12/14.
//
//

#ifndef __ofxKinectObjects__
#define __ofxKinectObjects__

#include "ofxKinect.h"
#include "ofxCv.h"
#include "ofxKinectObjectsEvents.h"


namespace ofxKinectObjects {
    
    class FloorObject{
    protected:
        bool quadIntersects(vector<ofVec3f> quad);
        
        //cv::Point2f center;
        float area_;
//        ofVec2f center_;
        ofVec3f worldCoordinates_;
        vector<ofPoint> quad_;
        unsigned int category_, touchedBy_;
        bool updated_;
    public:
        FloorObject(){};
        FloorObject(float area, ofVec3f worldCoordinates, vector<ofVec3f> quad);
        
        //Hand events handlers
        void handOn(HandOnEvent &e);
        void handOut(HandOutEvent &e);
        
        void setArea (float area);
        float getArea();
        
        void setQuad(vector<ofPoint> quad);
        
        void setWorldCoordinates(ofVec3f worldCoordinates);
        ofVec3f getWorldCoordinates();
        
        void setCategory (unsigned int category);
        unsigned int getCategory();
        
        void touch (unsigned int handLabel);
        bool isTouched ();
    };

    class ObjectTracker{
    public:
        ObjectTracker();
        void setup();
        void update();
        void updateParameters(ofVec2f floorThreshold, ofVec2f handsThreshold, ofVec2f objectsBlobSize, ofVec2f handsBlobSize, bool drawDetectors);
        void drawObjectDetector(int x, int y, int w, int h);
        void drawHandsDetector(int x, int y, int w, int h);
        void drawInput(int x, int y, int w, int h);
        void setKinectAngle(int angle);
        void startBgCalibration();
        bool isBgCalibrated();
        void exit();
        
    private:
        bool mousePressed(ofMouseEventArgs& mouse);
        float distanceToBackground (int kinectX, int kinectY);
        void selectCategory (unsigned int _label);
        
        //Kinect
        ofxKinect kinect;
        int kinectX, kinectY, kinectW, kinectH;
    #ifdef USE_TWO_KINECTS
        ofxKinect kinect2;
    #endif
        
        //CV images & contour finders
        ofImage objectsImage;
        ofImage handsImage;
        ofxCv::ContourFinder objectsFinder;
        ofxCv::ContourFinder handsFinder;
        
        //Parameters
        ofVec2f floorThreshold_;
        ofVec2f handsThreshold_;
        ofVec2f objectsBlobSize_;
        ofVec2f handsBlobSize_;
        bool drawDetectors_;
        
        //Objects
        map<unsigned int, FloorObject*> objects;
        
        //Background
        bool bCalibratingBackground;
        vector<ofVec3f> backgroundPoints;
        ofVec3f background_n;
        ofVec3f background_v0;        
    };
    
    vector<ofPoint> ofxCvPointQuadToOfPointQuad (vector<cv::Point> cvPointQuad);
    
} //namespace ofxKinectObjects

#endif /* defined(__kinectObjects__ofxKinectObjects__) */
