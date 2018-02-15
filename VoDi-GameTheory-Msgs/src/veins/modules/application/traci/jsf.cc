#include "jsf.h"
#include <WaveShortMessage_m.h>
#include <beaconFat_m.h>
#include <iostream>

using Veins::TraCIMobility;
using Veins::TraCIMobilityAccess;
using Veins::AnnotationManagerAccess;

using std::map;
using std::vector;
using std::string;
const double timeSlot = 0.000013;

Define_Module(jsf)

void jsf::initialize(int stage)
{
    timeoutEvent = new cMessage("timeoutEvent");
    timeoutEvent2 = new cMessage("timeoutEvent2");

    BaseWaveApplLayer::initialize(stage);
    if (stage == 0) {

        // configuration variables found in omnetpp.ini
        neighborLifetimeThreshold = par("neighborLifetimeThreshold").doubleValue();
        indexOfAccidentNode = par("indexOfAccidentNode").longValue();
        distanceThreshold = par("distanceThreshold").doubleValue();
        randomWaitingTime= par("randomWaitingTime").doubleValue();
        // end



        arrivalSignal = registerSignal("arrival");
        arrivalSignal1 = registerSignal("arrival1");

        warningMsgCounterSignalRx = registerSignal("warningMsgCounterRx");
        beaconMsgCounterSignalRx = registerSignal("beaconMsgCounterRx");


        distanceMsgSignalRx = registerSignal("distanceMsgRx");
        numberOfNodesSignal = registerSignal("numberOfNodes");

        receivedData=receivedBeacons=0;

        lastDroveAt = simTime();
        sentMessage = false;


        mobility = TraCIMobilityAccess().get(getParentModule());
        traci = mobility->getCommandInterface();
        traciVehicle = mobility->getVehicleCommandInterface();
        annotations = AnnotationManagerAccess().getIfExists();
        ASSERT(annotations);


    /*************************/

        receivedBeacons = 0;
        receivedData = 0;
        contador=0;

        WATCH(receivedBeacons);
        WATCH(receivedData);
        WATCH(contador);

        listNst.push_back(0);
        listN0t.push_back(0);
        listChannelOccupancy.push_back(0);
        listTimeWindow.push_back(0);

        junctionIds = mobility->getCommandInterface()->getJunctionIds();

        for (list<string>::iterator i = junctionIds.begin(); i != junctionIds.end(); ++i) {
            string jId = *i;
            Coord jPos = mobility->getCommandInterface()->junction(jId).getPosition();
            junctionMap[jId] = jPos;
        }

        //


        sentMessage = false;
        lastDroveAt = simTime();



         // configurable variables in omnetpp.ini

         indexOfAccidentNode = par("indexOfAccidentNode").longValue();

         vectorKiUi[simTime()]=0;
         vectorKjUj[getParentModule()->getIndex()]=1;
         vectorMKjUjN[simTime()]=0;

         beaconingRate[simTime()]=1;



         //
    }


    /***************************/


}


jsf::jsf()
{
    timeoutEvent = nullptr;
    timeoutEvent2 = nullptr;

}


jsf::~jsf()
{
   // cancelAndDelete(timeoutEvent);
   // cancelAndDelete(timeoutEvent2);
}



void jsf::receiveSignal(cComponent *source, simsignal_t signalID, cComponent::cObject *obj)
{
    Enter_Method_Silent();
    if (signalID == mobilityStateChangedSignal) {
        handlePositionUpdate(obj);
    }
}


void jsf::onBeacon(WaveShortMessage *wsm1)
{
    /*SCF*/
    beaconFat* wsm = dynamic_cast<beaconFat*>(wsm1);
    ASSERT(wsm);



    //Load data from MAC and Physical Layer


    double dsr=(mobility->getPositionAt(simTime()).distance(wsm->getSourcePos()));
    double distanceFromPreviousNode=mobility->getPositionAt(simTime()).distance(wsm->getSenderPos());
    double distanceToIntersection=calcDistJoin(wsm);

    timeout2=(simtime_t)(timeSlot*distanceFromPreviousNode)+SimTime(randomWaitingTime, SIMTIME_US);

    IntVector indices1=wsm->getIndices();



    bool bEqual=false;

    EV <<"<=====El beacon contiene IDs con mensajes que el vehiculo conoce?====>"<<bEqual<<endl;


    // handle stats
    receivedBeacons++;
    emit(beaconMsgCounterSignalRx,receivedBeacons);

    // is it a new neighbor?
    bool isNewNeighbor = true;
    vector<uint> indices;





    for (uint i = 0; i < neighbors.size(); ++i) {
        WaveShortMessage* neighbor = neighbors[i];
        if (neighbor->getCarId() == wsm->getCarId()) {
         // isNewNeighbor = false;
            neighbors.erase(neighbors.begin()+i);
        }
        else {
            // check for removal
            if (simTime() - neighbor->getArrivalTime() > neighborLifetimeThreshold)
                indices.push_back(i);
        }
    }

    // if it is a new neighbor
   // if (isNewNeighbor && !bEqual) {
    if (!bEqual) {
        for (map<long,WaveShortMessages>::iterator i = receivedWarningMessageMap.begin(); i != receivedWarningMessageMap.end(); ++i)
        {
            WaveShortMessage* msg = i->second[0];
            ASSERT(msg);
            findHost()->getDisplayString().updateWith("r=16,purple");
            // disseminate warning message
            if (timeoutEvent2->isScheduled())
               cancelEvent(timeoutEvent2);
            else
                scheduleAt(simTime()+timeout2 ,timeoutEvent2);
        }

    }

  // add new neighbor to neighbors list
  neighbors.push_back(wsm->dup());




 // remove the old neighbors
 WaveShortMessages newNeighborList;
 for (uint i = 0; i < neighbors.size(); ++i) {
     bool keepNeighbor = true;
     for (uint j = 0; j < indices.size(); ++j) {
         if (i == indices[j])
             keepNeighbor = false;
     }
     if (keepNeighbor)
         newNeighborList.push_back(neighbors[i]);
 }
 neighbors = newNeighborList;









 /*******************************************/
    /*Beaconing*/
 /*******************************************/
    beaconingRate[simTime()]=1;

    if(neighbors.size()>0 && receivedWarningMessageMap.size()>0 && vehicleOnJunction())
        forwardMessage(1);

}




void jsf::onData(WaveShortMessage *wsm)
{


    // handle stats
    receivedData++;
    emit(warningMsgCounterSignalRx, receivedData);

    indicesWarnigMsgsApp.push_back (wsm->getSerial());
    idMsg=wsm->getTreeId();



    // prevent originating disseminator from participating in further dissemination attempts
    if (sentMessage)
        return;

    // check if new warning
    bool isNewWarning = true;
    for (map<long,WaveShortMessages>::iterator i = receivedWarningMessageMap.begin(); i != receivedWarningMessageMap.end(); ++i) {
        WaveShortMessage* msg = i->second[0];
        if (msg->getTreeId() == wsm->getTreeId())
            isNewWarning = false;
    }

    if (isNewWarning) {

        findHost()->getDisplayString().updateWith("r=16,green");
        // handle stats
        simtime_t delayFirstNewMessage = wsm->getArrivalTime()-wsm->getCreationTime();
        // send a signal
        // statistics recording
        emit(arrivalSignal, delayFirstNewMessage);
    }


    // statistics recording
    int hops=wsm->getHopCount();
    emit(arrivalSignal1, hops);

/***************************************************************************************************************************************/
/***************************************************************************************************************************************/


    double dsr=(mobility->getPositionAt(simTime()).distance(wsm->getSourcePos()));
    double d=distanceFactorCalc(dsr,calcDistJoin(wsm),distanceThreshold,vehicleOnJunction());
    double distanceFromPreviousNode=mobility->getPositionAt(simTime()).distance(wsm->getSenderPos());

    double difDistance=1;
    (distanceThreshold-distanceFromPreviousNode)>0 ? difDistance=abs(distanceThreshold-distanceFromPreviousNode):difDistance=1;


    // add warning message to received messages storage
    receivedWarningMessageMap[wsm->getTreeId()].push_back(wsm->dup());


    cout.precision(17);


    double probabilidadEnvio;
    EV<<"=Probabilidad de envio: "<<probabilidadEnvio<<endl;



    timeout=(simtime_t)(timeSlot*difDistance)+SimTime(randomWaitingTime, SIMTIME_US);


    if (isNewWarning && vehicleOnJunction()) {
        EV<<"ENVIANDO SELFMSG EN : "<<timeout<<endl;
            // add a random waiting period before proceeding. Please see:
            //     * onSelfMsg for continuation.
            //     * .randomBroadcastDelay configuration in omnetpp.ini

        if (uniform(0,1) < 1)
        {
            forwardMessage(1);
            return;
         }else{
             if (timeoutEvent->isScheduled())
                 cancelEvent(timeoutEvent);
             else
                 scheduleAt(simTime()+timeout ,timeoutEvent);
         }


    }else{


                // add a random waiting period before proceeding. Please see:
                //     * onSelfMsg for continuation.
                //     * .randomBroadcastDelay configuration in omnetpp.ini
               if (timeoutEvent->isScheduled()) {
                           // EV<<"CANCELANDO  SELFMSG1"<<endl;
                            std::cerr << "[INFO] RE-BROADCAST  CANCELED @simTime: " << simTime().str() << " in node: " << getParentModule()->getIndex() << endl;
                            cancelEvent(timeoutEvent);

               }
    }

}

void jsf::handlePositionUpdate(cComponent::cObject *obj)
{
    BaseWaveApplLayer::handlePositionUpdate(obj);

    // stopped for for at least 10s?
    if (mobility->getSpeed() < 1) {
        if (simTime() - lastDroveAt >= 10) {
            //&& indexOfAccidentNode == getParentModule()->getIndex()
            if (!sentMessage && indexOfAccidentNode == getParentModule()->getIndex()){
                findHost()->getDisplayString().updateWith("r=16,red");
                DBG << ">>>>>>>>>>>>>> NEW MESSAGE EMERGENCY GENERATED <<<<<<<<<<<<<"<<std::endl;
                std::cerr << "[INFO] ACCIDENT STARTED @simTime: " << simTime().str() << " for node: " << getParentModule()->getIndex() << endl;
                sendMessage(mobility->getRoadId());

            }
        }
    }
    else {
        lastDroveAt = simTime();
    }
}

void jsf::handleSelfMsg(cMessage *msg)
{
    // for "data" and "beacon" self messages
     if ((!strcmp(msg->getName(), "timeoutEvent"))) {
            // if(receivedWarningMessageMap[atol(msg->getName())].size()>1){
              //   std::cerr << "[INFO] RE-BROADCAST  CANCELED @simTime: " << simTime().str() << " in node: " << getParentModule()->getIndex() << endl;
               //  return;
           //  }

            // else
                 forwardMessage(1);
             return;


     }
     else if((!strcmp(msg->getName(), "timeoutEvent2"))){
         forwardMessage(1);
         return;
     }



     else{
         //BaseWaveApplLayer::handleSelfMsg(msg);
         switch (msg->getKind()) {
                 case SEND_BEACON_EVT: {
                     EV<<"LLEGO EL  SELFMSG1 PARA EL BEACON : "<<endl;//<<idMsg<<endl;

                     //TraCIMobility* mySpeed = FindModule<TraCIMobility*>::findSubModule(getParentModule());


                     speed=(mobility->getSpeed());
                     carId=(getParentModule()->getIndex());
                     angleRad=mobility->getAngleRad();
                     vecX= mobility->getCurrentPosition().x;
                     vecY= mobility->getCurrentPosition().y;
                     vecZ= mobility->getCurrentPosition().z;
                     Mac1609_4* myMacp = FindModule<Mac1609_4*>::findSubModule(getParentModule());
                     simtime_t tbt=myMacp->getTotalBusyTime();
                     simtime_t tI=(simTime()-tbt)/simTime();

                     EV<<"ENVIANDO BEACON DESDE HANDLE: "<<endl;//<<idMsg<<endl;
                     sendWSM(prepareWSM("beacon", beaconLengthBits, type_CCH, beaconPriority, 0, -1,carId, speed, angleRad, vecX, vecY, vecZ, tI));
                       //scheduleAt(simTime() + par("beaconInterval").doubleValue(), sendBeaconEvt);
                   /**************/
                     map<simtime_t, double>::iterator itrBeaconing;
                          // Display the last element in vectorKjUj.
                         itrBeaconing = beaconingRate.end();
                          --itrBeaconing;
                   /******************/
                     double intervalBeacon=itrBeaconing->second;

                    //EV<<"intervalBeaconWatch: "<<intervalBeaconWatch<<endl;//<<idMsg<<endl;
                     EV<<"par(maxSpeedUrban).doubleValue():"<<par("maxSpeedUrban").doubleValue()<<endl;
                    // EV<<"intervalBeaconWatch: "<<intervalBeaconWatch<<endl;//<<idMsg<<endl;
                     scheduleAt(simTime() +intervalBeacon, sendBeaconEvt);
                     break;
                 }
                 default: {
                     if (msg)
                         DBG << "APP: Error: Got Self Message of unknown kind! Name: " << msg->getName() << endl;
                     break;
                 }
             }
         return;
     }
}



void jsf::forwardMessage(double p)
{
/*
    // if the number of times a warning message is received exceeds the counterThreshold
    // configuration variable, do not rebroadcast.*/
    //if (uniform(0,1) > p)
    //if ((unsigned)counterWarningMessages  >=  (unsigned)counterThreshold)// && uniform(0,1) > p)
    if (uniform(0,1) > p)
        {

        if (timeoutEvent->isScheduled()) {
            std::cerr << "[INFO] RE-BROADCAST  CANCELED @simTime: " << simTime().str() << " in node: " << getParentModule()->getIndex() << endl;
            cancelEvent(timeoutEvent);
         }
           return;
          }


  EV<<"<< BROADCAST>>"<<endl;

    // Duplicate message and send the copy.
    WaveShortMessage *copy = receivedWarningMessageMap[idMsg][0]->dup();

    // Increment hop count.
    copy->setHopCount(copy->getHopCount()+1);

    // Update Position

    copy->setSenderPos(mobility->getPositionAt(simTime()));

//    copy->setVecX(mobility->getPositionAt(simTime()).x);
//    copy->setVecY(mobility->getPositionAt(simTime()).y);
//    copy->setVecZ(mobility->getPositionAt(simTime()).z);

    copy->setCarId(getParentModule()->getIndex());



    sendWSM(copy);


    /* //print fowardedMessages
    EV<<"====FORWARDED MESSAGES===="<<endl;
    printMessages(fowardedMessages);*/



    std::cerr << "[INFO] RE-BROADCAST  STARTED @simTime: " << simTime().str() << " from node: " << getParentModule()->getIndex() << endl;


    return;
}

void jsf::sendMessage(std::string blockedRoadId)
{
    sentMessage = true;

    t_channel channel = dataOnSch ? type_SCH : type_CCH;
    WaveShortMessage* wsm = prepareWSM("data", dataLengthBits, channel, dataPriority, -1,2);
    wsm->setWsmData(blockedRoadId.c_str());
    sendWSM(wsm);
}


WaveShortMessage*  jsf::prepareWSM(std::string name, int lengthBits, t_channel channel, int priority, int rcvId, int serial,
        int carId, double speed, double angleRad, double vecX, double vecY, double vecZ, simtime_t idleTime) {
    beaconFat* wsm =        new beaconFat(name.c_str());
    wsm->addBitLength(headerLength);
    wsm->addBitLength(lengthBits);

    switch (channel) {
        case type_SCH: wsm->setChannelNumber(Channels::SCH1); break; //will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
        case type_CCH: wsm->setChannelNumber(Channels::CCH); break;
    }
    wsm->setPsid(0);
    wsm->setPriority(priority);
    wsm->setWsmVersion(1);
    wsm->setTimestamp(simTime());
    wsm->setSenderAddress(myId);
    wsm->setRecipientAddress(rcvId);
    wsm->setSenderPos(curPosition);
    wsm->setSourcePos(curPosition);
    wsm->setSerial(serial);


    wsm->setSpeed(speed);
    wsm->setCarId(carId);
    wsm->setAngleRad(angleRad);
    wsm->setVecX(vecX);
    wsm->setVecY(vecY);
    wsm->setVecZ(vecZ);
    wsm->setTimeIdleChannel(idleTime);

    wsm->setIndices(indicesWarnigMsgs);

    if (name == "beacon") {
        DBG << "Creating Beacon with Priority " << priority << " at Applayer at " << wsm->getTimestamp() << std::endl;

    }
    if (name == "data") {
        DBG << "Creating Data with Priority " << priority << " at Applayer at " << wsm->getTimestamp() << std::endl;
        indicesWarnigMsgsApp.push_back (wsm->getSerial());
      //  EV<<"wsm->getMulKjUjPowN()"<<wsm->getMulKjUjPowN()<<endl;
    }

    return wsm;
}

double jsf:: kiUi(double d, double lqD ) {

    return (double)calUtility(d,lqD);

}

/***********************************************************************************************************************************************/
/*Methods for metrics*/
/***********************************************************************************************************************************************/

double jsf::getLinkQuality(double chq, double cp, double sq ){


    EV<<"getChannelQuality = "<<chq<<endl;
    EV<<"getCollisionProbability = "<<cp<<endl;


    EV<<"getSignalQuality = "<<sq<<endl;

    double lq=sq*0.5+(1-cp)*chq*(1-0.5);

    return lq;


}

double jsf::calAgeFactor(double timeElapsed)
{
    return pow(0.99,timeElapsed);
}


bool jsf::hostIsClosestToJunction(string junctionId)
{
    // check to see if this host is near an intersection

    Coord jPos = junctionMap[junctionId];

    double hDist = jPos.distance(mobility->getPositionAt(simTime()));

    for (uint i = 0; i < neighbors.size(); ++i) {
        WaveShortMessage* neighbor = neighbors[i];
        if (jPos.distance(neighbor->getSenderPos()) < hDist) {
            return false;
        }
    }
    return true;
}


bool jsf::vehicleOnJunction()
{
    bool onJunction=false;
    // check to see if this host is near an intersection

    for (map<string,Coord>::iterator i = junctionMap.begin(); i != junctionMap.end(); ++i) {
        string jId = i->first;
        Coord jPos = i->second;
        Coord hPos = mobility->getPositionAt(simTime());
        if (jPos.distance(hPos) < 10) {
            onJunction=true;
            return onJunction;
        }
    }

    return onJunction;
}

string jsf::getIdJunction()
{
    // check to see if this host is near an intersection

    for (map<string,Coord>::iterator i = junctionMap.begin(); i != junctionMap.end(); ++i) {
        string jId = i->first;
        Coord jPos = i->second;
        Coord hPos = mobility->getPositionAt(simTime());
        if (jPos.distance(hPos) < 10) {
            return jId;
        }
    }

    return string();
}



double jsf::distanceFactorCalc(double d_sr, double d_ri, double r_max, bool  r_vehicleOnIntersection)
{
    // Dsr is the relative distance between source s and receptor r vehicles
    // Dri is the relative distance between vehicle r and the next nearest intersection
    // Rmax is the maximum transmission range
    double d;
    if (!r_vehicleOnIntersection)
        if(d_sr>r_max)
            d=1;
        else
            d=d_sr/r_max;
    else if(d_ri<10)
        d=1;
    else
        d=1-(d_ri/(d_ri+1));
    return d;
}



double jsf::getChannelQuality() {
    DBG << "::::::::::::  Channel Quality : :::::::::::: "<<std::endl;
    Mac1609_4* myMacp = FindModule<Mac1609_4*>::findSubModule(getParentModule());
    assert(myMacp);

    double cq=0;
    double n0t=0;
    double nst=0;
    //number Of Overall Transmisions
    //n0t =double(myMacp->getTXRXLostPackets())+double(myMacp->getSNIRLostPackets())+double(myMacp->getSentPackets());
    n0t =double(myMacp->getTXRXLostPackets())+double(myMacp->getSentPackets());

    //number Of Successfull Transmisions
    nst=double(myMacp->getNumSuccessfullTx());



    EV<<"Current n0t= "<<n0t<<std::endl;
    EV<<"Current nst= "<<nst<<std::endl;
    EV<<"Current push_back(n0t)= "<<listN0t.back()<<std::endl;
    EV<<"Current push_back(nst)= "<<listNst.back()<<std::endl;

    if((n0t-listN0t.back())>0 && (nst-listNst.back())>0)
        cq=(nst-listNst.back())/(n0t-listN0t.back());
    else
        cq=1;

    listNst.push_back(nst);
    listN0t.push_back(n0t);

    EV<<"ChannelQuality := "<<cq<<std::endl;
    DBG << ":::::::::::: ::::::::::: :::::::::::: "<<std::endl;
    return cq;
}
double jsf::getSignalQuality(WaveShortMessage* wsm) {
    DBG << "::::::::::::  Signal Quality : :::::::::::: "<<std::endl;

    Mac1609_4* myMacp = FindModule<Mac1609_4*>::findSubModule(getParentModule());
    assert(myMacp);

    // received signal strength

    double rss=pow(10,((DeciderResult80211*)((PhyToMacControlInfo*)wsm->getControlInfo())->getDeciderResult())->getRecvPower_dBm()/10);
    double rss_db=((DeciderResult80211*)((PhyToMacControlInfo*)wsm->getControlInfo())->getDeciderResult())->getRecvPower_dBm();
    double rssMax=par("maxTXPower").doubleValue()/1000;
    double rssMax_db=10*log10(rssMax);
    double rssTh=pow(10,par("sensitivity").doubleValue()/10);
    double rssTh_db=par("sensitivity").doubleValue();
    EV<<"Current rss= "<<rss<<std::endl;
    EV<<"Current rss_db= "<<rss_db<<std::endl;

    EV<<"Current rssMax= "<<rssMax<<std::endl;
    EV<<"Current rssMax_db= "<<rssMax_db<<std::endl;

    EV<<"Current rssTh_db= "<<rssTh_db<<std::endl;
    EV<<"Current rssTh= "<<rssTh<<std::endl;

    double s,sq;
    //if(rss>=rssTh)
        s=min((rssTh_db-rss_db)/(rssTh_db-rssMax_db),1.0);
   // else
      //  s=0;

    double snr=10*log10(((DeciderResult80211*)((PhyToMacControlInfo*)wsm->getControlInfo())->getDeciderResult())->getSnr());

    //speed
    double mySpeed =mobility->getSpeed();
    double v=mySpeed/par("maxSpeedUrban").doubleValue();

    if(snr>0)
        sq=max(0.0,s*(1-1/snr)*(1-v));
    else
        sq=s;

    EV<<"Current v= "<<v<<std::endl;
    EV<<"Current snr= "<<snr<<std::endl;
    EV<<"Current s= "<<s<<std::endl;
    EV<<"Current sq= "<<sq<<std::endl;

    EV<<"Signal Quality := "<<sq<<std::endl;
    DBG << ":::::::::::: ::::::::::: :::::::::::: "<<std::endl;
   return sq;
}


double jsf::getCollisionProbability() {

    DBG << "::::::::::::  Collision Probability : :::::::::::: "<<std::endl;
    Mac1609_4* myMacp = FindModule<Mac1609_4*>::findSubModule(getParentModule());
    assert(myMacp);

    EV<<"Current TotalBusyTime= "<<myMacp->getTotalBusyTime()<<std::endl;
    EV<<"Current simTime()= "<<simTime()<<std::endl;
    simtime_t cp=0;

    //Channel Occupancy time
    simtime_t co =myMacp->getTotalBusyTime();

    //last time window
    simtime_t tw=simTime();

    EV<<"Channel Occupancy time= "<<co<<std::endl;
    EV<<"last time window= "<<tw<<std::endl;

    EV<<"Channel Occupancy time (anterior)= "<<listChannelOccupancy.back()<<std::endl;
    EV<<"last time window (anterior)= "<<listTimeWindow.back()<<std::endl;
    cp=0;
    (tw.dbl()-listTimeWindow.back().dbl())>0 ? cp=(co.dbl()-listChannelOccupancy.back().dbl())/(tw.dbl()-listTimeWindow.back().dbl()):cp=co.dbl()/tw.dbl();

    listChannelOccupancy.push_back(co);
    listTimeWindow.push_back(tw);
    EV<<"Collision Probability := "<<cp<<std::endl;
    DBG << ":::::::::::: ::::::::::: :::::::::::: "<<std::endl;
    return cp.dbl();
}

double jsf::calcAbe(WaveShortMessage* wsm, double cpb)
{

        TraCIMobility* mySpeed = FindModule<TraCIMobility*>::findSubModule(getParentModule());
        Mac1609_4* myMacp = FindModule<Mac1609_4*>::findSubModule(getParentModule());
        string myroad=mobility->getRoadId();
        DBG << "Route :" << myroad<<std::endl;
        list<string> junctionIds = mobility->getCommandInterface()->getJunctionIds();


        DBG << "CWMIN_11P :" <<CWMIN_11P<<std::endl;
        DBG << "CWMAX_11P :" << CWMAX_11P<<std::endl;


        double k=(((2 * (SLOTLENGTH_11P + SIFS_11P).dbl())+(double(myMacp->gethNumBackoff())*SLOTLENGTH_11P.dbl()))/par("beaconInterval").doubleValue());

        DBG << "k :" << k<<std::endl;
        // How to calc ABE

        double f_m_N_s=-7.47*10e-5 *par("headerLength").doubleValue()-8.98*10e-3*double(neighbors.size())-1.42*10e-3*double(mySpeed->getSpeed())+1.98;
        f_m_N_s>1 ? f_m_N_s=0:f_m_N_s;

        double ts=wsm->getTimeIdleChannel().dbl();
        double tr=(simTime().dbl()-myMacp->getTotalBusyTime().dbl())/simTime().dbl();
        double myBitrate=((DeciderResult80211*)((PhyToMacControlInfo*)wsm->getControlInfo())->getDeciderResult())->getBitrate();
        DBG << "ts :" << ts<<std::endl;
        DBG << "tr :" << tr<<std::endl;
        DBG << "Bitrate :" << myBitrate<<std::endl;
        DBG << "f_m_N_s :" << f_m_N_s<<std::endl;

        //double cpb=getCollisionProbability();
        DBG << "cpb :" << cpb<<std::endl;

        double  abe1=double((1-k)*(1-f_m_N_s)*ts*tr*myBitrate);
        double  abe2=double((1-k)*(1-cpb)*ts*tr*myBitrate);
        double  availability= abe2/myBitrate;
       DBG << "::::::::::::::::::::::::ABE1::::::::::::::::::::::::" << abe1<<std::endl;
       DBG << "::::::::::::::::::::::::ABE2::::::::::::::::::::::::" << abe2<<std::endl;
       DBG << "::::::::::::::::::::::::availability::::::::::::::::::::::::" << availability<<std::endl;
       DBG << ":::::::::::: ::::::::::: :::::::::::: "<<std::endl;
       return availability;
}

double jsf::calcDistJoin(WaveShortMessage* wsm)
{
    string jId = getIdJunction();
     DBG  << "JUNCTIONS" << std::endl;

     DBG  << jId << std::endl;


     for (list<string>::iterator i = junctionIds.begin(); i != junctionIds.end(); ++i) {
                    string jId = *i;
                    Coord jPos = mobility->getCommandInterface()->junction(jId).getPosition();
                    Coord hPos = mobility->getPositionAt(simTime());
                    junctionDistance[jId] = jPos.distance(hPos);
                }
     double distJ=getMin(junctionDistance);

     if (distJ>5){
             DBG  << "NO Junctions" << std::endl;
         }
     if (distJ<=5) {
             DBG  << "Junctions :" <<distJ<<" m."<<std::endl;
             DBG  << "*********************************" << std::endl;
         }

     return distJ;

}



double jsf::getMin(std::map<std::string, double> mymap)
{
  std::pair<std::string, double> min
      = *min_element(mymap.begin(), mymap.end(), CompareSecond());
  return min.second;
}


/**********************************************************************/
/* Beaconing */
/**********************************************************************/


double jsf::beaconingMain(WaveShortMessage* wsm, int neighbors, double distanceFromPreviousNode, double distanceToIntersection )
{
    double C=beaconingChannel(neighbors,0.75,wsm);
    double P=beaconingPriority(distanceFromPreviousNode,distanceToIntersection,wsm);
    double wI=0.75;
    double dos=2;
    double uno=1;
    double intervalBeacon=(uno-wI)*pow(P,dos)+(wI*pow(C,dos));
    EV<<"beaconingMain"<<endl;
    EV<<"C="<<C<<endl;
    EV<<"P="<<P<<endl;
    EV<<"intervalBeacon="<<intervalBeacon<<endl;
    double Imin=par("Imin").doubleValue();
    double Imax=par("Imax").doubleValue();
    double deltaI=(Imin+(Imax-Imin)*intervalBeacon);

    EV<<"Imin="<<Imin<<endl;
    EV<<"Imax="<<Imax<<endl;


    return deltaI;


}

double jsf::beaconingChannel(double neighbors, double wc, WaveShortMessage* wsm)
{
    double numberOfCollision=0;
    double k=1-(1/(1+numberOfCollision));
    double snr=10*log10(((DeciderResult80211*)((PhyToMacControlInfo*)wsm->getControlInfo())->getDeciderResult())->getSnr());
    double snrMx=50;
    double neighborMax=160;
    double uno=1;

    double n=min(uno,(pow(neighbors/neighborMax,2)));
    double cero=0;
    double s=max(cero,pow(snr/snrMx,2));


    EV<<"beaconingChannel"<<endl;
    EV<<"snr="<<snr<<endl;
    EV<<"n="<<n<<endl;
    EV<<"s="<<s<<endl;
    EV<<"neighbors="<<neighbors<<endl;


    return ((n+wc*(s+k)/2)/(1+wc));
}

double jsf::beaconingPriority( double dEv, double dInt, WaveShortMessage* wsm)
{
    double speed=mobility->getSpeed();
    (speed<=0)?speed=0.001:speed=mobility->getSpeed();
    double de=pow((dEv/speed)/par("Imax").doubleValue(),2);
    double uno=1;
    double De=min(de,uno);

    double dintersection=pow((dInt/speed)/par("Imax").doubleValue(),2);
    double Dint=min(dintersection,uno);

    double msgAge=(simTime().dbl()-wsm->getCreationTime().dbl());
    double A=min(msgAge,uno);
    double Dr=0;
    double B=1;

    EV<<"beaconingPriority"<<endl;
    EV<<"msgAge"<<msgAge<<endl;

    EV<<"distancia Intersecciones"<<dInt<<endl;
    EV<<"distancia del evento"<<dEv<<endl;
    EV<<"Speed"<<speed<<endl;

    EV<<"A="<<A<<endl;
    EV<<"De="<<De<<endl;
    EV<<"Dint="<<Dint<<endl;
    EV<<"Dr="<<Dr<<endl;
    EV<<"B="<<B<<endl;


    return (B*(A+De+Dint+Dr)/4);

}



/***********************************************************************************************************************************************/
/*Methods for Vodi*/
/***********************************************************************************************************************************************/


double jsf::calBeqAsimetricoMult(double ki, double ui, double kjuj, double n)
{
    return (ui/ki)*(pow(kjuj,1/(n-1)));
}

double jsf::calBeq(double k, double u, double r, double n){

    return pow((k/((u*(1+r*(n-1))))),(1/(n-1)));
}


double jsf::calUtility(double distanceFactor, double linkQualityFactor)
{
    return pow(10,10-(6*distanceFactor+4*linkQualityFactor));
}


long double jsf::vodM(vector<long double> u){


       double mul_of_elems=1;
       for(vector<long double>::iterator it = u.begin(); it != u.end(); ++it){
           mul_of_elems *= *it;
       }
       cout <<"mul_of_elements: "<<mul_of_elems<<endl;

       return mul_of_elems;

}


/***********************************************************************************************************************************************/
/*Methods for Forwarding Game*/
/***********************************************************************************************************************************************/

// function to reduce matrix to r.e.f.  Returns a value to
// indicate whether matrix is singular or not
int jsf::forwardElim(double** mat, int N, int M){

    for (int k=0; k<N; k++)
    {
        // Initialize maximum value and index for pivot
        int i_max = k;
        int v_max = mat[i_max][k];

        /* find greater amplitude for pivot if any */


     for (int i = k+1; i < N; i++)
            if (abs(mat[i][k]) > v_max)
                v_max = mat[i][k], i_max = i;

        /* if a prinicipal diagonal element  is zero,
         * it denotes that matrix is singular, and
         * will lead to a division-by-zero later. */


    if (!mat[k][i_max])
            return k; // Matrix is singular

        /* Swap the greatest value row with current row */


    if (i_max != k)
            swap_row(mat,N,M, k, i_max);


        for (int i=k+1; i<N; i++)
        {
            /* factor f to set current row kth elemnt to 0,
             * and subsequently remaining kth column to 0 */
            double f = mat[i][k]/mat[k][k];

            /* subtract fth multiple of corresponding kth
               row element*/
            for (int j=k+1; j<=N; j++)
                mat[i][j] -= mat[k][j]*f;

            /* filling lower triangular matrix with zeros*/
            mat[i][k] = 0;
        }

        //print(mat);        //for matrix state
    }
    //print(mat);            //for matrix state
    return -1;
}


vector<long double> jsf::backSub(double** mat, int N, int M){
    // function to calculate the values of the unknowns
    vector<long double> v;
    double x[N];  // An array to store solution

    /* Start calculating from last equation up to the
       first */


    for (int i = N-1; i >= 0; i--)
    {
        /* start with the RHS of the equation */


        x[i] = mat[i][N];

        /* Initialize j to i+1 since matrix is upper
           triangular*/


        for (int j=i+1; j<N; j++)
        {
            /* subtract all the lhs values
             * except the coefficient of the variable
             * whose value is being calculated */



            x[i] -= mat[i][j]*x[j];
        }

        /* divide the RHS by the coefficient of the
           unknown being calculated */


        x[i] = x[i]/mat[i][i];
    }

    printf("\nSolution for the system:\n");
    for (int i=0; i<N; i++){
        v.push_back(x[i]);
        printf("%lf\n", x[i]);
    }

    return v;

}


vector<long double>  jsf::gaussianElimination(double** mat, int N, int M){
    // function to get matrix content

    /* reduction into r.e.f. */


    int singular_flag = forwardElim(mat,N,M);

    /* if matrix is singular */


    if (singular_flag != -1)
    {
        printf("Singular Matrix.\n");

        /* if the RHS of equation corresponding to
           zero row  is 0, * system has infinitely
           many solutions, else inconsistent*/

        if (mat[singular_flag][N])
            printf("Inconsistent System.");
        else
            printf("May have infinitely many "
                   "solutions.");
        vector<long double> vE;

        return vE;
    }

    /* get solution to system and print it using
       backward substitution */
    vector<long double> v1 = backSub(mat,N,M);
    return v1;
}


void  jsf::swap_row(double** mat, int N, int M, int i, int j){
    // function for elemntary operation of swapping two rows
    //printf("Swapped rows %d and %d\n", i, j);

     for (int k=0; k<=N; k++)
     {
         double temp = mat[i][k];
         mat[i][k] = mat[j][k];
         mat[j][k] = temp;
     }

}


void jsf::print(double** mat, int N, int M){
    // function to print matrix content at any stage
    for (int i=0; i<N; i++, printf("\n"))
        for (int j=0; j<=N; j++)
            printf("%lf ", mat[i][j]);

    printf("\n");

}
