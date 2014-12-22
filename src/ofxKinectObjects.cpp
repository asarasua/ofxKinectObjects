//
//  ObjectTracker.cpp
//  kinectObjects
//
//  Created by Álvaro Sarasúa Berodia on 17/12/14.
//
//

#include "ofxKinectObjects.h"

namespace ofxKinectObjects {
    /***
     OBJECTS
     **___________________________________*/
    FloorObject::FloorObject(float area, ofVec3f worldCoordinates){
        area_ = area;
        touchedBy_ = 0;
        //touched_ = false;
        worldCoordinates_ = worldCoordinates;
        ofAddListener(HandOnEvent::events, this, &FloorObject::handOn);
        ofAddListener(HandOutEvent::events, this, &FloorObject::handOut);
    }
    
    float FloorObject::getArea(){
        return area_;
    }
    
    void FloorObject::setArea(float area){
        area_ = area;
    }
    
    void FloorObject::setWorldCoordinates(ofVec3f worldCoordinates){
        worldCoordinates_ = worldCoordinates;
    }
    
    ofVec3f FloorObject::getWorldCoordinates(){
        return worldCoordinates_;
    }
    
    unsigned int FloorObject::getCategory(){
        return category_;
    }
    
    void FloorObject::setCategory(unsigned int category){
        category_ = category;
    }
    
//    void FloorObject::setTouched (bool touched){
//        touched_ = touched;
//    }
    
    void FloorObject::touch(unsigned int handLabel){
        touchedBy_ = handLabel;
    }
    
    bool FloorObject::isTouched (){
        return (touchedBy_ != 0);
    }
    
    void FloorObject::handOn(HandOnEvent &e){
        // NOT TOUCHED
        //    - distance < lim --> touch
        //    - distance > lim --> nothing
        
        // TOUCHED
        //    - other hand --> nothing
        //    - same hand & distance > lim --> untouch
        
        if (touchedBy_ == 0 && (e.worldCoordinates.distance(worldCoordinates_) < 100)) {
            touch(e.handLabel);
        } else if (touchedBy_ == e.handLabel && (e.worldCoordinates.distance(worldCoordinates_) >= 100)){
            touch(0);
        }
    }
    
    void FloorObject::handOut(HandOutEvent &e){
        if (e.handLabel == touchedBy_) {
            touch(0);
        }
    }
    
    /***
     OBJECT TRACKER
     **___________________________________*/
    
    ObjectTracker::ObjectTracker(){
        
    }

    void ObjectTracker::setup(){
        //add listener to mousePressed event
        ofAddListener(ofEvents().mousePressed, this, &ObjectTracker::mousePressed);
        
        // enable depth->video image calibration
        kinect.setRegistration(true);
        
        //kinect.init();
        //kinect.init(true); // shows infrared instead of RGB video image
        kinect.init(false, false); // disable video image (faster fps)
        
        kinect.open();		// opens first available kinect
        //kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
        //kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
        
        // print the intrinsic IR sensor values
        if(kinect.isConnected()) {
            ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
            ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
            ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
            ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
        }
        
    #ifdef USE_TWO_KINECTS
        kinect2.init();
        kinect2.open();
    #endif

        objectsImage.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);
        handsImage.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);
                
        ofSetFrameRate(60);
        
        kinectX = kinectY = kinectW = kinectH = 0;
        
        // zero the tilt on startup
        kinect.setCameraTiltAngle(0);
        
        bCalibratingBackground = false;
        
    }

    void ObjectTracker::update(){
        kinect.update();
        
        // there is a new frame and we are connected
        if(kinect.isFrameNew()) {
            
            // load grayscale depth image from the kinect source
            //grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
            
            // or we do it ourselves - show people how they can work with the pixels
            ofPixelsRef objPixels = objectsImage.getPixelsRef();
            ofPixelsRef handsPixels = handsImage.getPixelsRef();
            
            for(int i = 0; i < kinect.getWidth(); i++) {
                for (int j = 0; j < kinect.getHeight(); j++) {
                    //OBJECTS ON THE FLOOR
                    if (ofInRange(distanceToBackground(i, j), floorThreshold_.x, floorThreshold_.y)){
                        objPixels.setColor(i, j, 255);
                        handsPixels.setColor(i, j, 0);
                        //HANDS ON OBJECTS
                    } else if (ofInRange(distanceToBackground(i, j), handsThreshold_.x, handsThreshold_.y)){
                        objPixels.setColor(i, j, 0);
                        handsPixels.setColor(i, j, 255);
                    } else {
                        objPixels.setColor(i, j, 0);
                        handsPixels.setColor(i, j, 0);
                    }
                }
            }
            
            // update the cv images
            objectsImage.setFromPixels(objPixels);
            objectsImage.update();
            handsImage.setFromPixels(handsPixels);
            handsImage.update();
            
            // find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
            // also, find holes is set to true so we will get interior contours as well....
            //contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);
            objectsFinder.setMinArea(objectsBlobSize_.x);
            objectsFinder.setMaxArea(objectsBlobSize_.y);
            objectsFinder.findContours(objectsImage);
            
            handsFinder.setMinArea(handsBlobSize_.x);
            handsFinder.setMaxArea(handsBlobSize_.y);
            handsFinder.findContours(handsImage);
            
            //ObjectTracker().update(objectsFinder, handsFinder, kinect);
        }
        
    #ifdef USE_TWO_KINECTS
        kinect2.update();
    #endif
        
        //TODO is this efficient / smart???
        vector<unsigned int> objectIds;
        
        for (int i = 0; i < objectsFinder.size(); ++i) {
            objectIds.push_back(objectsFinder.getLabel(i));
            
            vector<cv::Point> quads = objectsFinder.getFitQuad(i);
            float currentArea = kinect.getWorldCoordinateAt(quads[0].x, quads[0].y).distance(kinect.getWorldCoordinateAt(quads[1].x, quads[1].y))
            + kinect.getWorldCoordinateAt(quads[1].x, quads[1].y).distance(kinect.getWorldCoordinateAt(quads[2].x, quads[2].y));
            
            //Object already exists
            if (objects.find(objectsFinder.getLabel(i)) != objects.end()) {
                //update object
                //update area if bigger
                if (currentArea > objects[objectsFinder.getLabel(i)]->getArea()) {
                    objects[objectsFinder.getLabel(i)]->setArea(currentArea);
                }
                objects[objectsFinder.getLabel(i)]->setWorldCoordinates(kinect.getWorldCoordinateAt(objectsFinder.getCentroid(i).x, objectsFinder.getCentroid(i).y));
            }
            //object doesn't exist
            else {
                //insert new object
                //FloorObject newObject(currentArea, kinect.getWorldCoordinateAt(objectsFinder.getCentroid(i).x, objectsFinder.getCentroid(i).y));
                FloorObject* newObject = new FloorObject(currentArea, kinect.getWorldCoordinateAt(objectsFinder.getCentroid(i).x, objectsFinder.getCentroid(i).y));
                objects[objectsFinder.getLabel(i)] = newObject;
            }
            
            //Choose category
            selectCategory(objectsFinder.getLabel(i));
        }
        
        //Eliminate unpresent objects
        map<unsigned int, FloorObject*>::iterator it = objects.begin();
        while (it != objects.end()) {
            if (find(objectIds.begin(), objectIds.end(), it->first) == objectIds.end() ) {
                objects.erase(it++);
            } else {
                ++it;
            }
        }
        
        for (int i = 0; i < handsFinder.size(); ++i) {
            static HandOnEvent newEvent;
            newEvent.handLabel = handsFinder.getLabel(i);
            newEvent.center.x = handsFinder.getCentroid(i).x;
            newEvent.center.y = handsFinder.getCentroid(i).y;
            newEvent.worldCoordinates = kinect.getWorldCoordinateAt(handsFinder.getCentroid(i).x, handsFinder.getCentroid(i).y);
            ofNotifyEvent(HandOnEvent::events, newEvent);
        }
        
        for (int i = 0; i < handsFinder.getTracker().getDeadLabels().size(); i++){
            static HandOutEvent newEvent;
            newEvent.handLabel = handsFinder.getTracker().getDeadLabels()[i];
            ofNotifyEvent(HandOutEvent::events, newEvent);
        }
    }


   /* void ObjectTracker::draw()
    {

        // draw instructions
        ofSetColor(255, 0, 0);
        stringstream reportStream;
        
        if (bCalibratingBackground){
            reportStream << "Choose 3 corners!";
        }
        else {
            if(kinect.hasAccelControl()) {
                reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
                << ofToString(kinect.getMksAccel().y, 2) << " / "
                << ofToString(kinect.getMksAccel().z, 2) << endl;
            } else {
                reportStream << "Note: this is a newer Xbox Kinect or Kinect For Windows device," << endl
                << "motor / led / accel controls are not currently supported" << endl << endl;
            }
            reportStream << "press b to calibrate background" << endl
            << "fps: " << ofGetFrameRate() << endl
            << "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl;
        }
        
        ofDrawBitmapString(reportStream.str(), 20, 652);
    }*/
    
    //--------------------------------------------------------------
    void ObjectTracker::drawObjectDetector(int x, int y, int w, int h){
        //OBJECTS
        //Image
        ofSetColor(0, 255, 0);
        objectsImage.draw(x, y, w, h);
        
        //Detector
        ofPushMatrix();
        ofSetLineWidth(2);
        ofSetColor(0, 0, 255);
        ofTranslate(x, y);
        ofScale(w/float(kinect.width), h/float(kinect.height));
        objectsFinder.draw();
        ofPopMatrix();
        
        for (int i = 0; i < objectsFinder.size(); ++i) {
            vector<cv::Point> quads = objectsFinder.getFitQuad(i);
            
            float realArea = kinect.getWorldCoordinateAt(quads[0].x, quads[0].y).distance(kinect.getWorldCoordinateAt(quads[1].x, quads[1].y))
            + kinect.getWorldCoordinateAt(quads[1].x, quads[1].y).distance(kinect.getWorldCoordinateAt(quads[2].x, quads[2].y));
            
            float x_obj = ofMap(objectsFinder.getCentroid(i).x, 0, objectsImage.width, x, x+w);
            float y_obj = ofMap(objectsFinder.getCentroid(i).y, 0, objectsImage.height, y, y+h);
            
            if (objects[objectsFinder.getLabel(i)]->isTouched()) {
                ofSetColor(ofColor::yellow);
                ofCircle(x_obj, y_obj, 4);
                ofSetColor(0, 0, 255);
            }
            ofDrawBitmapString(ofToString(realArea), x_obj, y_obj);
        }
    }
    
    //--------------------------------------------------------------
    void ObjectTracker::drawHandsDetector(int x, int y, int w, int h){
        //HANDS
        //Image
        ofSetColor(255, 0, 0);
        handsImage.draw(x, y, w, h);
        
        //Detector
        ofPushMatrix();
        ofSetLineWidth(2);
        ofSetColor(0, 0, 255);
        ofTranslate(x, y);
        ofScale(w/float(kinect.width), h/float(kinect.height));
        handsFinder.draw();
        ofPopMatrix();
        
        for (int i = 0; i < handsFinder.size(); ++i) {
            vector<cv::Point> quads = handsFinder.getFitQuad(i);
            
            float realArea = kinect.getWorldCoordinateAt(quads[0].x, quads[0].y).distance(kinect.getWorldCoordinateAt(quads[1].x, quads[1].y))
            + kinect.getWorldCoordinateAt(quads[1].x, quads[1].y).distance(kinect.getWorldCoordinateAt(quads[2].x, quads[2].y));
            
            float x_hand = ofMap(handsFinder.getCentroid(i).x, 0, handsImage.width, x, x+w);
            float y_hand = ofMap(handsFinder.getCentroid(i).y, 0, handsImage.width, y, y+h);
            
            ofDrawBitmapString(ofToString(realArea), x_hand, y_hand);
        }
        ofSetColor(255, 255, 255);
    }
    
    //--------------------------------------------------------------
    void ObjectTracker::drawInput(int x, int y, int w, int h){
        // draw from the live kinect
        kinect.drawDepth(x, y, w, h);
        kinectX = x;
        kinectY = y;
        kinectW = w;
        kinectH = h;
        int kinectMouseX = ofMap(ofGetMouseX(), x, x+w, 0, kinect.getWidth());
        int kinectMouseY = ofMap(ofGetMouseY(), y, y+h, 0, kinect.getHeight());
        ofSetColor(255, 0, 0);
        
        //Info over mouse
        if (isBgCalibrated() && ofGetMouseX() > x && ofGetMouseX() < x+w && ofGetMouseY() > y && ofGetMouseY() < y+h){
            ofDrawBitmapString(ofToString(distanceToBackground(kinectMouseX, kinectMouseY)), ofGetMouseX(), ofGetMouseY());
        }
        else if (ofGetMouseX() > x && ofGetMouseX() < x+w && ofGetMouseY() > y && ofGetMouseY() < y+h){
            ofDrawBitmapString(ofToString(kinect.getWorldCoordinateAt(kinectMouseX, kinectMouseY)), ofGetMouseX(), ofGetMouseY());
        }
        ofSetColor(255, 255, 255);
        
#ifdef USE_TWO_KINECTS
        kinect2.draw(420, 320, 400, 300);
#endif
    }
    
    bool ObjectTracker::isBgCalibrated(){
        return backgroundPoints.size() == 3;
    }
    
    //--------------------------------------------------------------
    void ObjectTracker::exit(){
        kinect.setCameraTiltAngle(0); // zero the tilt on exit
        kinect.close();
        
    #ifdef USE_TWO_KINECTS
        kinect2.close();
    #endif
    }

    //--------------------------------------------------------------
    float ObjectTracker::distanceToBackground(int kinectMouseX, int kinectMouseY){
        ofVec3f p = kinect.getWorldCoordinateAt(kinectMouseX, kinectMouseY);
        
        return abs(background_n.dot(p-background_v0));
    }
    
    //--------------------------------------------------------------
    void ObjectTracker::setKinectAngle(int angle){
        if(kinect.hasCamTiltControl()) {
            kinect.setCameraTiltAngle(angle);
        }
    }
    
    //--------------------------------------------------------------
    void ObjectTracker::updateParameters(ofVec2f floorThreshold, ofVec2f handsThreshold, ofVec2f objectsBlobSize, ofVec2f handsBlobSize, bool drawDetectors){
        floorThreshold_ = floorThreshold;
        handsThreshold_ = handsThreshold;
        objectsBlobSize_ = objectsBlobSize;
        handsBlobSize_ = handsBlobSize;
        drawDetectors_ = drawDetectors;
    }
    
    //--------------------------------------------------------------
    void ObjectTracker::selectCategory(unsigned int _label){
        //TODO implement categories logic
        if (objects[_label]->getArea() < 50) {
            objects[_label]->setCategory(1);
        } else {
            objects[_label]->setCategory(2);
        }
    }
    
    //--------------------------------------------------------------
    void ObjectTracker::startBgCalibration(){        
        bCalibratingBackground = !bCalibratingBackground;
        if (bCalibratingBackground){
            backgroundPoints.clear();
        }
    }
    
    //--------------------------------------------------------------
    bool ObjectTracker::mousePressed(ofMouseEventArgs &mouse){
        if (bCalibratingBackground) {
            if (backgroundPoints.size() < 3) {
                int kinectMouseX = ofMap(ofGetMouseX(), kinectX, kinectX+kinectW, 0, kinect.getWidth());
                int kinectMouseY = ofMap(ofGetMouseY(), kinectY, kinectY+kinectH, 0, kinect.getHeight());
                backgroundPoints.push_back(kinect.getWorldCoordinateAt(kinectMouseX, kinectMouseY));
            }
            if (backgroundPoints.size() == 3){
                bCalibratingBackground = false;
                background_v0 = backgroundPoints[0];
                background_n = (backgroundPoints[1]-backgroundPoints[0]).getCrossed(backgroundPoints[2]-backgroundPoints[0]).limit(1.0);
            }
        }
    }
}