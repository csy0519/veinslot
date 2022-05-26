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

#ifndef __VEINS_MYVEINSAPP_H_
#define __VEINS_MYVEINSAPP_H_

#include <omnetpp.h>
#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"
#include "veins/modules/messages/Beacon_m.h"
#include "veins/base/utils/Coord.h"

using namespace omnetpp;


/**
 * @brief
 * This is a stub for a typical Veins application layer.
 * Most common functions are overloaded.
 * See MyVeinsApp.cc for hints
 *
 * @author David Eckhoff
 *
 */

class MyVeinsApp : public BaseWaveApplLayer {
    private:

    SimTime intralBeaconInterval = SimTime(0);
    SimTime lastBeaconTime;
    double intralBeaconRate = 10; //unit:Hz , max = 10;
    cMessage* sendBeacon;
    cMessage* judge;
    cMessage* sendUrgentMsg;
    double speedBefore = 0;
    double acceleration = 0;
    double beaconRate;
    double acmax = 2.1;
    double acmin = -9.55;
    double apmax = 0.7;
    double apmin = -2;
    double pa1 = 0;
    double pa2 = 0;
  //  double beaconrate1[100];
  //  double selfp = 0;
    double neighborp = 0;
    double p = 0;
    double cp[100]; //collision probability
    int slot = 100;
    int interval = 10; //frame interval
    SimTime frameLength = SimTime (0.1);
    double totalRate = 0;
  //  bool Idbeused[100];
    bool index[100];
    double THSbeaconrate[100];
    double THSbeaconrate2[100];   // renew THSbeaconrate
    double THSspeed[100];
    double THSacceleration[100];
    double THScp[100];
    Coord THSposition[100];
    double THSangle[100];
    double THSslotnum[10][100];
    int pct = 10;   //predicted collision time
    double L = 4;  //car length
    double W = 2;   //car width

    public:
        virtual void initialize(int stage);
        virtual void finish();

    protected:
        virtual void onBSM(BasicSafetyMessage* bsm);
        virtual void onWSM(WaveShortMessage* wsm);
        virtual void onWSA(WaveServiceAdvertisment* wsa);

        virtual void handleSelfMsg(cMessage* msg);
        virtual void handleLowerMsg(cMessage* msg);
        virtual void handlePositionUpdate(cObject* obj);
        virtual bool needBeaconInThisFrame(SimTime sim1 , SimTime sim2 );
        virtual bool isCollision(int i);
        double AngleToXY(double angle);
        double FindMaxcp(double maxcp,int& id);
    };

#endif
