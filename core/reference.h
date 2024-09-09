#ifndef REFERENCE_H
#define REFERENCE_H

#include "types.h"

class referenceCounted {
private:
    int counter;

    virtual int grab(){}
    virtual int release(){

    }
public:
    referenceCounted(){}
    virtual ~referenceCounted(){}
};

#endif