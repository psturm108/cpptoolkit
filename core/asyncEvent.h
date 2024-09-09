#ifndef ASYNC_EVENT_H
#define ASYNC_EVENT_H

#include "types.h"
#include "stream.h"
#include "pipe.h"

class asyncEvent {
public:
    asyncEvent(){
        this->px = 0;
        this->str = 0;
        this->flags = 0;
        this->close = false;
    }
    virtual ~asyncEvent(){}

    u8 flags;
    bool close;

    stream *str;
    xpipe *px;
};

#endif