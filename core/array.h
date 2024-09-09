#ifndef ARRAY_H
#define ARRAY_H

#include "types.h"
#include "container.h"

template<class T>
class array : public container<T> {
public:
    array(int s=8):
    container<T>(s)
    {}
    virtual ~array(){}

    
    virtual T push_back(const T& v);
    virtual T push_front(const T& v);
    virtual T pop_back();
    virtual T pop_front();
    virtual T remove(int index);
    virtual T insert(int index,const T& v);
};

template<class T>
inline T array<T>::push_back(const T& v){
    if(this->length>=this->size){
        //! No more space in array. Get more space
        this->realloc(this->size+8);
    }
    //! Copy the specific value
    this->values[this->length] = v;
    //! Increment the array length
    ++this->length;
    return v;
}
template<class T>
inline T array<T>::push_front(const T& v){
    if(this->length>=this->size){
        //! No more space in array. Get more space
        this->realloc(this->size+8);
    }
    //! Move back the values
    for(int i=this->length-1;i>=0;--i){
        this->values[i+1] = this->values[i];
    }
    //! Copy the specific value
    this->values[0] = v;
    //! Increment the array length
    ++this->length;
    return v;
}
template<class T>
inline T array<T>::pop_back(){
    //! If empty, return null
    if(this->length<=0) return T(0);
    //! Copy over the value
    T v = this->values[this->length-1];
    //! Clear the value in array
    this->values[this->length-1] = 0;
    //! Decrement array length
    --this->length;
    return v;
}
template<class T>
inline T array<T>::pop_front(){
    //! If empty, return null
    if(this->length<=0) return T(0);
    //! Copy over the value
    T v = this->values[0];
    //! Move back the values
    for(int i=0;i<this->length;++i){
        this->values[i] = this->values[i+1];
    }
    //! Decrement array length
    --this->length;
    return v;
}
template<class T>
inline T array<T>::remove(int index){
    //! If empty, return null
    if(this->length<index) return T(0);
    //! Copy over the value
    T v = this->values[index];
    //! Move back the values
    for(int i=index;i<this->length;++i){
        this->values[i] = this->values[i+1];
    }
    //! Decrement array length
    --this->length;
    return v;
}
template<class T>
inline T array<T>::insert(int index,const T& v){
    if(this->length>=this->size){
        //! No more space in array. Get more space
        this->realloc(this->size+8);
    }

    //! Move back the values
    for(int i=this->length;i>index;--i){
        this->values[i] = this->values[i-1];
    }
    //! Increment array length
    ++this->length;
    return v;
}




#endif