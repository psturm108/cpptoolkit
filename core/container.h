#ifndef CONTAINER_H
#define CONTAINER_H

#include "types.h"

template<class T>
class container {
public:
    //! Empty constructor
    container(int s = 8){
        this->alloc(s);
    }
    //! Copy constructor
    container(const container<T>& cont){
        this->alloc(cont.size);
        //! Copy over the values
        for(s32 i=0;i<cont.length;++i){
            this->values[i] = cont.values[i];
        }
    }
    virtual ~container(){
        this->dealloc();
    }

    virtual T get(int index);
    virtual T set(int index,T value);

    virtual void alloc(int s);
    virtual void realloc(int s);
    virtual void dealloc();

    virtual void reverse();

    s32 size;
    s32 length;
    T *values;
};
template<class T>
inline T container<T>::get(int index){
    if(index <0 || index >=this->length) return T(0);
    return this->values[index];
}
template<class T>
inline T container<T>::set(int index,T value){
    if(index<0 || index>=this->length) return T(0);
    return this->values[index] = value;
}
template<class T>
inline void container<T>::alloc(int s){
    this->values = new T [s];
    this->size = s;
    this->length = 0;
    memset(&this->values[0],0,s*sizeof(T));
}
template<class T>
inline void container<T>::realloc(int s){
    //! Allocate some new memory
    T *nv = new T [s];
    //! Determine how many values to copy
    s32 count = _min<s32>(s,this->length);
    //! Copy over the values
    for(s32 i=0;i<count;++i){
        nv[i] = this->values[i];
    }
    //! Delete the old memory
    delete [] this->values;
    //! Copy the memory pointer
    this->values = nv;
    //! Write the new size
    this->size = s;
}
template<class T>
inline void container<T>::dealloc(){
    delete [] this->values;
    this->length = 0;
    this->size = 0;
}
template<class T>
inline void container<T>::reverse(){
    T memory = 0;
    int len = this->length/2;
    if( (len&1) == 1 ) --len;

    for(int i=0;i< len;++i){
        memory = this->values[i];
        this->values[i] = this->values[this->length-i-1];
        this->values[this->length-i-1] = memory;
    }
}

#endif