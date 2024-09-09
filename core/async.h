#ifndef ASYNC_H
#define ASYNC_H

#include "types.h"
#include "handle.h"
#include "stream.h"
#include "pipe.h"
#include "array.h"
#include "list.h"

#include "asyncEvent.h"

class async : public handle {
public:
    async():
    handle(HT_ASYNC),
    streams(),
    pipes()
    {
        this->finished = false;
        this->counter = 0;
    }
    virtual ~async(){}

    virtual int add(stream *str);
    virtual int remove(stream *str);
    virtual int wait(s32 timeout);

    epoll_event events[MAX_EPOLL_EVENTS];

    int counter;
    bool finished;

    list<stream> streams;
    list<xpipe> pipes;
};
inline int async::add(stream *str){
    struct epoll_event event;
    if(str->type == HT_TIMER)
        event.events = EPOLLIN | EPOLLET;
    else if (str->type == HT_SERIAL)
        event.events = EPOLLIN | EPOLLOUT | EPOLLRDHUP | EPOLLPRI;
    else
        event.events = EPOLLIN | EPOLLOUT | EPOLLRDHUP | EPOLLET; // Can append "|EPOLLOUT" for write events as well
    event.data.ptr = (void*) str;

    //! | EPOLLET

    int retval = epoll_ctl(this->descriptor, EPOLL_CTL_ADD, str->descriptor, &event);
    ++this->counter;
    return retval;
}
inline int async::remove(stream *str){
    int retval = epoll_ctl(this->descriptor, EPOLL_CTL_DEL, str->descriptor, 0);
    --this->counter;
    return retval;
}
inline int async::wait(s32 timeout){
    //! Continue waiting for events
    int ready = epoll_wait(this->descriptor,this->events,MAX_EPOLL_EVENTS,timeout);
    //! Go through each descriptor with an event
    for(int i=0;i<ready;++i){
        //! Now to determine some event specifics
        epoll_event* pev = &this->events[i];

        //! The stream in question
        stream *pstream = (stream*) pev->data.ptr;
        unsigned int flags = 0;
        //! Fast code:
        flags |= (pev->events & EPOLLIN)? AIO_DATA:0;
        flags |= (pev->events & EPOLLOUT)? AIO_DRAIN:0;
        flags |= (pev->events & EPOLLRDHUP)? AIO_SHUTDOWN:0;
        flags |= (pev->events & EPOLLPRI)? AIO_EXCEPTION:0;
        flags |= (pev->events & EPOLLERR)? AIO_ERROR:0;
        flags |= (pev->events & EPOLLHUP)? AIO_HANGUP:0;

        //! We have assimilated the flags. Now to create an event object
        asyncEvent ev;

        ev.flags = flags;
        //! Handle the specific events
        if(flags & AIO_SHUTDOWN
        || flags & AIO_HANGUP)
        {
            //! Close the specific descriptor
            pstream->close();
            //! The descriptor is no longer valid
            pstream->onClose();
            //! Remove the stream descriptor from epoll
            this->remove(pstream);
            //! Delete the stream object
            delete pstream;
            //! Continue looping through objects
            continue;
        }
        if(flags & AIO_ERROR)
        {
            //! Report the error
            pstream->onError();
            //! Close the specific descriptor
            pstream->close();
            //! The descriptor is no longer valid
            pstream->onClose();
            //! Remove the stream descriptor from epoll
            this->remove(pstream);
            //! Delete the stream object
            delete pstream;
            //! Continue looping through objects
            continue;
        }
        if(flags & AIO_DATA){
            //! Data is available for reading

            //! Read into the input buffer
            int returnValue = pstream->readin();
        }
        if(flags & AIO_DRAIN){
            //! Space is available for writing

            //! Flush the output buffer
            int returnValue = pstream->flush();
        }


    } //! END LOOP
 
    if(this->counter<=0){
        this->finished = true;
    }
    return (this->finished == true)?1:0;
}

#endif