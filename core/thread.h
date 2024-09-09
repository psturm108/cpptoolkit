#ifndef THREADS_H
#define THREADS_H

#include "types.h"
#include "handle.h"
#include "stream.h"

class thread {
public:
    thread(){}
    virtual ~thread(){}

    virtual int begin(){
        pthread_create(&this->id,0,this->execute,this);
        return 0;
    }
    virtual void end(){
        pthread_kill(this->id,SIGKILL);
    }
    static void *execute(void *context){
        thread *pt = (thread*) context;
        if(pt->run() != 0){
            pt->end();
        }
    }
    virtual int run(){
        return 0;
    }

    pthread_t id;
};

#endif