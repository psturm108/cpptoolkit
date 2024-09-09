#ifndef BUFFER_H
#define BUFFER_H

#include "types.h"
#include "container.h"

template<class T>
class buffer : public container<T> {
public:
    buffer(int s=256,encodingType et = ET_BINARY):
    container<T>(s)
    {
        this->type = et;
        this->front = this->back = &this->values[0];
    }
    buffer(const buffer<T>& buff):
    container<T>(buff)
    {
        this->type = buff.type;
        this->front = buff.front;
        this->back = buff.back;
    }
    virtual ~buffer(){}

    virtual int write(T* buff,int len);
    virtual int read(T* buff,int len);
    virtual int available();
    virtual int remaining();
    virtual void clear();

    encodingType type;

    T* front;
    T* back;
};
template<class T>
inline int buffer<T>::write(T* buff,int len){
    //! Determine space in buffer
    int bytesAvail = this->size - this->length-1;
    //! Determine the amount of elements we can write
    int bytesCanWrite = _min<int>(len,bytesAvail);
    //! Make sure we have a pointer to writing pad
    if(!this->back) this->back = &this->values[0];
    //! Make sure we have a pointer to the front
    if(!this->front) this->front = &this->values[0];
    //! Copy over the data
    for(int i=0;i<bytesCanWrite;++i){
        this->back[i] = buff[i];
    }
    //! Increment the buffer length
    this->length+=bytesCanWrite;
    //! Move over the new 'back' pointer
    this->back = &this->back[bytesCanWrite];
    //! Return amount of bytes written
    return bytesCanWrite;
}
template<class T>
inline int buffer<T>::read(T *buff,int len){
    //! Determine the amount of elements we can read
    int bytesCanRead = _min<int>(len,this->length);
    //! Copy and zero over the data
    for(int i=0;i<bytesCanRead;++i){
        //! Copy
        buff[i] = this->values[i];
        //! Zero
        this->values[i] = 0;
    }

    //! Decrement the buffer length
    this->length-=bytesCanRead;
    //! Move over the new 'front' pointer
    this->front = &this->front[bytesCanRead];
    //! If the head has met the tail, rewind
    if(this->front == this->back){
        this->front = this->back = 0;
    }
    //! Return amount of bytes read
    return bytesCanRead;
}
template<class T>
inline int buffer<T>::available(){
    if(!this->front && !this->back)
        return 0;
    
    return this->back - this->front;
}
template<class T>
inline int buffer<T>::remaining(){
    if(!this->front || !this->back)
        return this->size;

    return this->size -((u64)this->back-(u64)this->front);
}
template<class T>
inline void buffer<T>::clear(){
    memset(&this->values[0],0,this->length);
    this->length = 0;
}

#endif