#ifndef ASYNC_BASE_H
#define ASYNC_BASE_H

class asyncBase {
public:
    asyncBase(){}
    virtual ~asyncBase(){}

    virtual int onConnect(){return -1;}
    virtual int onDisconnect(){return -1;}

    virtual int onClose(){return -1;}
    virtual int onData(){return -1;}
    virtual int onDrain(){return -1;}
    virtual int onEnd(){return -1;}
    virtual int onError(){return -1;}
    virtual int onPause(){return -1;}
    virtual int onOpen(){return -1;}
};


#endif