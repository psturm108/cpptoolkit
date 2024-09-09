#ifndef STREAM_H
#define STREAM_H

#include "types.h"
#include "buffer.h"
#include "handle.h"
#include "array.h"
#include "list.h"

#include "asyncBase.h"

class stream : public handle, public asyncBase {
private:
    stream *last;
    stream *next;
public:
    stream(handleType ht =HT_SERIAL):
    handle(ht),
    inputStream(),
    outputStream()
    {
        this->bufferSize = DEFAULT_BUFFER_SIZE;
        this->last = 0;
        this->next = 0;
    }
    virtual ~stream(){
        for(int i=0;i<this->inputStream.length;++i){
            delete this->inputStream.get(i);
        }
        for(int i=0;i<this->outputStream.length;++i){
            delete this->outputStream.get(i);
        }
    }

    virtual buffer<u8>* getInputBuffer(int index=0);
    virtual buffer<u8>* getOutputBuffer(int index=0);
    virtual buffer<u8>* createInputBuffer(int size=1024);
    virtual buffer<u8>* createOutputBuffer(int size=1024);
    virtual void destroyInputBuffer(int ix);
    virtual void destroyOutputBuffer(int ix);
    virtual buffer<u8>* lastInputBuffer();
    virtual buffer<u8>* lastOutputBuffer();

    //virtual int readin();
    virtual int writeout();
    virtual int flush();

    s32 bufferSize;
    array<buffer<u8>*> inputStream;
    array<buffer<u8>*> outputStream;

    template<class T>
    friend class list;
};
inline buffer<u8>* stream::getInputBuffer(int index){
    if(this->inputStream.length<=0) return 0;
    return this->inputStream.get(index);
}
inline buffer<u8>* stream::getOutputBuffer(int index){
    if(this->outputStream.length<=0) return 0;
    return this->outputStream.get(index);
}
inline buffer<u8>* stream::createInputBuffer(int size){
    buffer<u8> *newBuffer = new buffer<u8>(size);
    return this->inputStream.push_back(newBuffer);
}
inline buffer<u8>* stream::createOutputBuffer(int size){
    buffer<u8> *newBuffer = new buffer<u8>(size);
    return this->outputStream.push_back(newBuffer);
}
inline void stream::destroyInputBuffer(int ix){
    buffer<u8> *pb = this->inputStream.get(ix);
    this->inputStream.remove(ix);
    delete pb;
}
inline void stream::destroyOutputBuffer(int ix){
    buffer<u8> *pb = this->outputStream.get(ix);
    this->outputStream.remove(ix);
    delete pb;
}
inline buffer<u8>* stream::lastInputBuffer(){
    if(this->inputStream.length<=0) return 0;
    return this->inputStream.values[this->inputStream.length-1];
}
inline buffer<u8>* stream::lastOutputBuffer(){
    if(this->outputStream.length<=0) return 0;
    return this->outputStream.values[this->outputStream.length-1];
}
/*
inline int stream::readin(){

    //! Check how many bytes are available for reading
    //! Note: This might not be accurate in many cases
    //!       Not sure what those cases are
    int bytesAvailable = this->available();
    //! This might not work with ssl
    if(bytesAvailable<=0){
        //! Try to read some in anyway
        bytesAvailable=1024;
    }
    //! We will try only a certain amount times
    int tries = 0;

    //! Data is available for reading
    int bytesTransferred = 0;
    //! A flag so we know when to quit the loop
    bool finished = false;
    //! Continue filling buffers until all data is consumed
    while(!finished){
        //! Get a pointer to the last available input buffer
        buffer<u8> *destination = this->lastInputBuffer();
        //! If one does not exists or no space available, create one
        if(!destination || destination->remaining()==0){
            destination = this->createInputBuffer(bytesAvailable+4096);
            printf("No io buffer found. Creating\r\n");
        }
        //! Determine the amount of space available in buffer
        int spaceAvailable = destination->remaining();

        int bytesRead = this->read(&destination->back[0],spaceAvailable);
        if(bytesRead<0){
            ++tries;
            printf("Read error %i\r\n",bytesRead);
        } else if (bytesRead ==0){
            printf("Read in no bytes\r\n");
        } else {
            tries = 0;
            //! Update the destination buffer
            destination->back = &destination->back[bytesRead];
            destination->length += bytesRead;

            printf("Read in %i bytes\r\n",bytesRead);
        }

        bytesTransferred += bytesRead;
        if(tries>=3) finished = true;

    }

    return bytesTransferred;
}*/
inline int stream::writeout(){
    //! Check if there is data available to write
    int bytesAvailable = 0;
    //! Go over each buffer and determine the size
    for(int i=0;i<this->outputStream.length;++i){
        //! Get a pointer to the specific output buffer
        buffer<u8> *pb = this->getOutputBuffer(i);
        //! Increment the amount we can write
        bytesAvailable += pb->available();
    }

    //! We won't transfer all of it now if its giant
    int bytesToTransfer = _min<int>(bytesAvailable,(64*1024));
    //! Go over buffers until we have transferred all the data
    int bytesWritten = 0;
    do {
        //! Allocate a local buffer
        u8 transferBuffer[bytesToTransfer];
        //! Clear the buffer space
        memset(&transferBuffer[0],0,bytesToTransfer);

        //! Get a pointer to the first buffer
        buffer<u8> *source = this->getOutputBuffer(0);
        if(!source){
            //! Hmm..
            break;
        }
        //! Get the amount of bytes in the buffer
        int bytesInBuffer = source->available();
        //! Get the amount of bytes avail to transfer
        int bytesFromBuffer = _min<u8>(bytesInBuffer, bytesToTransfer-bytesWritten );
        //! Read in from the buffer
        int bytesReadFromBuffer = source->read(&transferBuffer[bytesWritten],bytesFromBuffer);
        //! Bytes written to port
        int bytesOut = 0;
        do {
            bytesOut+=this->write(&transferBuffer[bytesWritten+bytesOut],bytesReadFromBuffer-bytesOut);
        } while(bytesOut != bytesReadFromBuffer);
        //! HAAHAHA

    } while(bytesWritten != bytesToTransfer);

    int returnValue = this->onDrain();

    return returnValue;
}
inline int stream::flush(){
    return this->writeout();
}

#endif