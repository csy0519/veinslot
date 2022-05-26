//
// Copyright (C) 2016 David Eckhoff <david.eckhoff@fau.de>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#include "MyVeinsApp.h"

Define_Module(MyVeinsApp);

void MyVeinsApp::initialize(int stage) {
    beaconRate = 1/interval;
    BaseWaveApplLayer::initialize(stage);
    if (stage == 0) {
        for(int i = 0;i<100;i++){
                //    beaconrate1[i] = 0;
              //      Idbeused[i] = 0;
                    THSbeaconrate[i] = 0;
                    THSposition[i] = Coord(0,0);
                    THSangle[i] = 0;
                    THSspeed[i] = 0;
                    THSacceleration[i] = 0;
                    THScp[i] = 0;
                    index[i] = 0;
                    for(int j=0;j<10;j++){
                        THSslotnum[j][i] = 0;
                    }
                }
        //Initializing members and pointers of your application goes here
        EV << "Initializing " << par("appName").stringValue() << std::endl;
        sendBeacon = new cMessage("send Beacon");
        judge = new cMessage("judge");
        sendUrgentMsg = new cMessage("send UrgentMsg");
    }
    else if (stage == 1) {
        intralBeaconInterval = SimTime(1) / intralBeaconRate;
        //Initializing members that require initialized other modules goes here
        SimTime frameStart = SimTime((double)((int)(simTime().raw()/SimTime(frameLength).raw()))/10);
        SimTime slotTime = frameLength / slot ;
        EV << "slotTime = " << slotTime << std::endl;
        SimTime sendTime = frameStart + this->getParentModule()->getIndex()*slotTime ; //distribute slot by id
        EV << "index = " << this->getParentModule()->getIndex()<< std::endl;
        scheduleAt(sendTime,judge);
        EV << "stage == 1 !!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;


    }
}

void MyVeinsApp::finish() {
    BaseWaveApplLayer::finish();
    //statistics recording goes here

}

void MyVeinsApp::onBSM(BasicSafetyMessage* bsm) {
    //Your application has received a beacon message from another car or RSU
    //code for handling the message goes here

}

void MyVeinsApp::onWSM(WaveShortMessage* wsm) {
    EV << "onWSM !!!!!!!!! " << simTime() << endl;
  //  MyVeinsApp::handleLowerfMsg();
    //Your application has received a data message from another car or RSU
    //code for handling the message goes here, see TraciDemo11p.cc for examples

}

void MyVeinsApp::onWSA(WaveServiceAdvertisment* wsa) {
    //Your application has received a service advertisement from another car or RSU
    //code for handling the message goes here, see TraciDemo11p.cc for examples

}

void MyVeinsApp::handleSelfMsg(cMessage* msg) {
    //this method is for self messages (mostly timers)
    //it is important to call the BaseWaveApplLayer function for BSM and WSM transmission
    if (msg == sendBeacon) {
        // get information about the vehicle via traci
        cModule* vehicle = getParentModule();
        Veins::TraCIMobility* traci = dynamic_cast<Veins::TraCIMobility*>(vehicle->getSubmodule("veinsmobility", 0));
        Veins::TraCICommandInterface::Vehicle* traciVehicle = traci->getVehicleCommandInterface();
      //  Plexe::VEHICLE_DATA data;
       // traciVehicle->getVehicleData(&data);
        THSposition[this->getParentModule()->getIndex()] = traci->getPositionAt(simTime());
        THSangle[this->getParentModule()->getIndex()] = traci->getAngleRad();
        THSspeed[this->getParentModule()->getIndex()] = traci->getSpeed();
        THSacceleration[this->getParentModule()->getIndex()] = acceleration;

       // EV << "id = " <<this->getParentModule()->getIndex()<<endl;
        Beacon* beacon = new Beacon();

        beacon->setName("beacon");
        beacon->setVehicleId(this->getParentModule()->getIndex());

        acceleration = ( traci->getSpeed() - speedBefore ) * 1;//renew acc every 1s
        speedBefore = traci->getSpeed();
        beacon->setA(acceleration);
        beacon->setV(traci->getSpeed());
        beacon->setX(traci->getPositionAt(simTime()).x);
        beacon->setY(traci->getPositionAt(simTime()).y);
        SimTime frameStart = SimTime((double)((int)(simTime().raw()/SimTime(frameLength).raw()))/10);
        int frmNum = ((int)(10*(frameStart.raw()/SimTime(1).raw())))%10;
        int sltNum = this->getParentModule()->getIndex();
        beacon->setSlotpos(Coord(frmNum,sltNum));
        EV << "current position = " <<traci->getPositionAt(simTime())<<endl;

        //beacon->setLength();
        beacon->setAng(traci->getAngleRad());
      //  beacon->setP(selfp);
       // beacon->setBr(beaconRate);
      //  EV << "beaconrate = " <<beacon->getBeaconrate(i)<<endl;    //get THSbeaconrate ???


        for (int i = 0;i < 100;i++){
            beacon->setBeaconrate(i,THSbeaconrate[i]);
            beacon->setCp(i, THScp[i]);
            beacon->setSpeed(i,THSspeed[i]);
            beacon->setAcceleration(i, THSacceleration[i]);
            beacon->setPosition(i, THSposition[i]);
            beacon->setAngle(i, THSangle[i]);
         //   EV << "speed = " <<beacon->getSpeed(i)<<endl;
         //   EV << "acceleration = " <<beacon->getAcceleration(i)<<endl;
         //   EV << "position = " <<beacon->getPosition(i)<<endl;
            EV << "angle = " <<beacon->getAngle(i)<<endl;

        }

        WaveShortMessage* WSM = new WaveShortMessage();
        WSM->encapsulate(beacon);
        populateWSM(WSM);
        send(WSM,lowerLayerOut);
        return;

    }
    else if(msg == judge){
      //  if (Idbeused[bc->getVehicleId()] = 0 ){
        // calculate p   selfp = p  beaconRate =

        double a1 = acceleration; //current a of current node
        double selfp[1000];
        for(int i=0;i<1000;i++){
            selfp[i] = 0;
        }
        for (int i = 0;i<=100;i++){
            double a = acmin +i*(acmax-acmin)/100;
            if(a>=acmin && a<=a1){   //int ()?????
                pa1 = 2*(a-acmin)/((a1-acmin)*(acmax-acmin));
            }
            else if (a>a1 && a<=acmax){
                pa1 = 2*(acmax-a)/((acmax-a1)*(acmax-acmin));
            }
           selfp[i] = pa1;
        }

        double maxp = 0;
    for (int k = 0;k < 100;k++){
        p = 0;
        if( THSposition[k] != Coord(0,0) && k != this->getParentModule()->getIndex()){
            double a2 = THSacceleration[k];  //current a of neighbor node

            for (int i = 0;i<=100;i++){
                double a = acmin +i*(acmax-acmin)/100;
                if(a>=acmin && a<=a2){   //int ()?????
                    pa2 = 2*(a-acmin)/((a2-acmin)*(acmax-acmin));
                }
                else if (a>a2 && a<=acmax){
                    pa2 = 2*(acmax-a)/((acmax-a2)*(acmax-acmin));
                }
                neighborp = pa2;
                for(int j=0;j<100;j++){
                    p = p + selfp[j]*neighborp*isCollision(k);

                }
            }
            if(p>maxp){
                maxp = p;
            }

            EV << "selfp = " <<selfp[1]<<endl;
            EV << "neighborp = " <<neighborp<<endl;
            EV << "isCol = " <<isCollision(k)<<endl;
           //cp[k] = p;



           EV << "maxp =  " << maxp << endl;
        }

    }
    beaconRate = ceil(9*maxp + 1);
    THScp[this->getParentModule()->getIndex()] = maxp;
    THSbeaconrate[this->getParentModule()->getIndex()] = beaconRate;

        for (int i = 0;i < 100;i++){
                     totalRate = totalRate + THSbeaconrate[i];
         }
           if(totalRate>slot*10){
               double renewTotalRate = 0;
               for(int i = 0;i < 100;i++){
                   THSbeaconrate[i] = 1;
               }
             //  for(k = 0;k < 100;k++){
               while(renewTotalRate < 1000){
                   int id = -1;
                   double maxp = FindMaxcp(maxp,id);
                   int renewId = id;
                   double newBr = ceil(1+9*maxp*maxp);  // renew beaconrate
                       // put renewed beaconrate in THSbeaconrate[]
                   THSbeaconrate[renewId] = newBr;
                   renewTotalRate = renewTotalRate + newBr;
                    //   if(beaconRate<THSbeaconrate[i]){
                      //     THSbeaconrate2[i] = beaconRate;
                     //  }
                     //  else THSbeaconrate2[i] = THSbeaconrate[i];
               }
               SimTime frameStart = SimTime((double)((int)(simTime().raw()/SimTime(frameLength).raw()))/10);
               SimTime slotTime = frameLength / slot ;
               SimTime sendTime = frameStart + 99*slotTime ;  // send urgent message at the end of each frame
               scheduleAt(sendTime,sendUrgentMsg);
           }
           if(needBeaconInThisFrame(lastBeaconTime , intralBeaconInterval)){

               scheduleAt(simTime(), sendBeacon);
               EV << "my next beacon time: " << simTime() << endl;
           }
           scheduleAt(simTime() + frameLength*10,judge);
           return;
          }
    else if(msg == sendUrgentMsg){
   //         Urgent* urgent = new Urgent();
   //         urgent->setName("urgent");
   //         for(i = 0;i < 100;i++){
   //         urgent->setNewBr(i,THSbeaconrate[i]);
  //              for (j = 0;j < 1/THSbeaconrate[i];j++){


 //       send(WSM,lowerLayerOut);
    }




        BaseWaveApplLayer::handleSelfMsg(msg);
   }
  //  else if(msg == calculate){
    //    if(totalRate > slot) {

    //    }
 //   }





void MyVeinsApp::handleLowerMsg(cMessage* msg) { //calculate cp, compare and change interval
    WaveShortMessage* WSM = check_and_cast<WaveShortMessage*>(msg);
    cPacket* enc = WSM->getEncapsulatedPacket();
    Beacon* bc = dynamic_cast<Beacon*>(enc);

    for (int i = 0;i < 100;i++){
        THSbeaconrate[i] = bc->getBeaconrate(i) ;
        THSspeed[i] = bc->getSpeed(i) ;
        THSacceleration[i] = bc->getAcceleration(i) ;
        THSposition[i] = bc->getPosition(i) ;
        THSangle[i] = bc->getAngle(i) ;
        THScp[i] = bc->getCp(i) ;

    //    EV << "received beaconrate = " <<bc->getBeaconrate(i)<<endl;    //get THSbeaconrate ???
    //    EV << "received speed = " <<bc->getSpeed(i)<<endl;
    //    EV << "received acceleration = " <<bc->getAcceleration(i)<<endl;
    //    EV << "received position = " <<bc->getPosition(i)<<endl;
    //    EV << "received angle = " <<bc->getAngle(i)<<endl;
    }
    for (int i = 0;i < (int)bc->getBeaconrate(bc->getVehicleId());i++ ){
        int k = i*(int)(10/bc->getBeaconrate(bc->getVehicleId())) ;
        if(bc->getSlotpos().x+k < 10){
        THSslotnum[(int)bc->getSlotpos().x+k][(int)bc->getSlotpos().y] = 1 ;
        }
    }
    THSbeaconrate[bc->getVehicleId()] = bc->getBeaconrate(bc->getVehicleId());
    THSspeed[bc->getVehicleId()] = bc->getSpeed(bc->getVehicleId()) ;
    THSacceleration[bc->getVehicleId()] = bc->getAcceleration(bc->getVehicleId()) ;
    THSposition[bc->getVehicleId()] = bc->getPosition(bc->getVehicleId()) ;
    THSangle[bc->getVehicleId()] = bc->getAngle(bc->getVehicleId()) ;

    // beaconrate is an array
 //   if(int(simtime()) == ){

   //     scheduleAt(simtime(),calculate);
 //   }

   // cp =
    EV << "Receive successfully !!!!!!!!!!!" << endl;

}

void MyVeinsApp::handlePositionUpdate(cObject* obj) {
    BaseWaveApplLayer::handlePositionUpdate(obj);
    //the vehicle has moved. Code that reacts to new positions goes here.
    //member variables such as currentPosition and currentSpeed are updated in the parent class

}


bool MyVeinsApp::needBeaconInThisFrame(SimTime lastBeaconTime , SimTime intralBeaconInterval){
    EV << "lastBeaconTime:" << lastBeaconTime << " intralBeaconInterval:" << intralBeaconInterval << endl;
    SimTime t = simTime();
    SimTime pointOne = SimTime((double)1/10);
    SimTime frameStart = SimTime((double)((int)(t.raw()/pointOne.raw()))/10);
    SimTime bt = lastBeaconTime + intralBeaconInterval;
    if(bt < frameStart + 0.1){
        EV << "beacon in this frame........." << endl;
        return true;
    }
    else{
        EV << "do not beacon in this frame." << endl;
        return false;

    }
}


bool MyVeinsApp::isCollision(int i){
    cModule* vehicle = getParentModule();
    Veins::TraCIMobility* traci = dynamic_cast<Veins::TraCIMobility*>(vehicle->getSubmodule("veinsmobility", 0));
    Veins::TraCICommandInterface::Vehicle* traciVehicle = traci->getVehicleCommandInterface();

    double v1 = traci->getSpeed();
    double a1 = acceleration;
    double x1 = traci->getPositionAt(simTime()).x;
    double y1 = traci->getPositionAt(simTime()).y;
    double angle1 = traci->getAngleRad();
    double angle3 = AngleToXY(angle1);
  //  for (int i = 0;i < 100;i++){
        double v2 = THSspeed[i];
        double a2 = THSacceleration[i];
        double x2 = THSposition[i].x;
        double y2 = THSposition[i].y;
        double angle2 = THSangle[i];
        double angle4 = AngleToXY(angle2);
        double d1 = v1*pct + a1*pct*pct/2;
        double d2 = v2*pct + a2*pct*pct/2;

        //EV << "vehicle" << this->getParentModule()->getIndex() << ":[" << x1 << "," << y1 << "]" << endl;
       //         EV << "vehicle" << i << ":[" << x2 << "," << y2 << "]" << endl;
         //       EV << "v1" << v1 << endl;
        //        EV << "a1" << a1 << endl;
       //         EV << "angle3" << angle3 << endl;
        //        EV << "angle4" << angle4 << endl;
       //         EV << "d1" << d1 << endl;
        //        EV << "d2" << d2 << endl;
       //         EV << "v2" << v2 << endl;
       //         EV << "a2" << a2 << endl;


         //       EV << "abs(x2-x1-L/2)" << abs(x2-x1-L/2) << endl;
          //      EV << "d1*cos(angle3)+d2*cos(angle4)" << d1*cos(angle3)+d2*cos(angle4) << endl;
          //      EV << "abs(x2-x1+L/2)" << abs(x2-x1+L/2) << endl;
          //      EV << "abs(y2-y1-W/2)" << abs(y2-y1-W/2) << endl;
          //      EV << "d1*sin(angle3)+d2*sin(angle4)" << d1*sin(angle3)+d2*sin(angle4) << endl;
          //      EV << "abs(y2-y1+W/2)" << abs(y2-y1+W/2) << endl;
        if(((d1*cos(angle3)+d2*cos(angle4))>=abs(x2-x1-L/2))&&((d1*cos(angle3)+d2*cos(angle4))<=abs(x2-x1+L/2))&&((d1*sin(angle3)+d2*sin(angle4))>=abs(y2-y1-W/2))&&((d1*sin(angle3)+d2*sin(angle4))<=abs(y2-y1+W/2))){
            return true;
        }
        else
            return false;


 //   }

}

double MyVeinsApp::AngleToXY(double angle){
    double angleToXY = 0;
    if((angle >= 0)&&(angle < PI/2)){
        angleToXY = angle;
    }
    else if((angle >= PI/2)&&(angle < PI )) angleToXY = PI - angle;
    else if((angle > -PI/2)&&(angle <= 0)) angleToXY = -angle;
    else angleToXY = PI + angle;
    return angleToXY ;
}


double MyVeinsApp::FindMaxcp(double maxcp,int& id){
  //  id = 0;
    maxcp = THScp[0];
    double lastMax = 2;
    for(int j = 1;j <= 99;j++){   //find max cp in THS
        if(THScp[j]<=lastMax&&!index[j]){
            if(THScp[j]>=maxcp){
                maxcp = THScp[j];
                index[j] = 1;
                id = j;
            }
        }
    }


    return maxcp;
    lastMax = maxcp;

}
