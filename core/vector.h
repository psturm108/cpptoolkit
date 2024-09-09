#ifndef VECTOR_H
#define VECTOR_H

#include "types.h"

#define EARTH_RADIUS 6378
#define TO_RAD (3.1415926536 / 180)

template<class T>
class vec2 {
public:
    vec2():
        x(0),
        y(0)
    {}
    vec2(T a):
        x(a),
        y(a)
    {}
    vec2(T a,T b):
        x(a),
        y(b)
    {}
    vec2(const vec2<T>& a){
        this->x = a.x;
        this->y = a.y;
    }

    virtual ~vec2(){}

    virtual bool operator<(const vec2<T>& v);
    virtual bool operator<=(const vec2<T>& v);
    virtual bool operator==(const vec2<T>& v);
    virtual bool operator!=(const vec2<T>& v);
    virtual bool operator>(const vec2<T>& v);
    virtual bool operator>=(const vec2<T>& v);

    virtual vec2<T>& operator=(const T& a);
    virtual vec2<T>& operator=(const vec2<T>& v);

    virtual vec2<T> operator+(const vec2<T>& v);
    virtual vec2<T> operator+(const T& v);
    virtual vec2<T>& operator+=(const vec2<T>& v);
    virtual vec2<T>& operator+=(const T& v);

    virtual vec2<T> operator*(const vec2<T>& v);
    virtual vec2<T> operator*(const T& v);
    virtual vec2<T>& operator*=(const vec2<T>& v);
    virtual vec2<T>& operator*=(const T& v);

    virtual vec2<T> operator-(const vec2<T>& v);
    virtual vec2<T> operator-(const T& v);
    virtual vec2<T>& operator-=(const vec2<T>& v);
    virtual vec2<T>& operator-=(const T& v);

    virtual vec2<T> operator/(const vec2<T>& v);
    virtual vec2<T> operator/(const T& v);
    virtual vec2<T>& operator/=(const vec2<T>& v);
    virtual vec2<T>& operator/=(const T& v);

    virtual vec2<T>& abs();
    virtual vec2<T>& min(const vec2<T>& a,const vec2<T>& b);
    virtual vec2<T>& max(const vec2<T>& a,const vec2<T>& b);
    virtual vec2<T>& ceil();
    virtual vec2<T>& floor();
    virtual vec2<T>& round();

    virtual T lengthSq();
    virtual T length();

    virtual T distanceSq(const vec2<T>& v);
    virtual T distance(const vec2<T>& v);

    virtual T manhattan(const vec2<T>& v);
    virtual T haversine(const vec2<T>& v);

    virtual vec2<T> lerp(const vec2<T>& t,T f);

    virtual vec2<T>& invert();
    //! Compute the dot product
    virtual T dot(const vec2<T>& v);
    //! Rotate around another point
    virtual vec2<T>& rotate(const vec2<T>& point,float angle);


    virtual void print(){
        printf("Vector: %f, %f\r\n",(float)this->x,(float)this->y);
    }

    union {
        struct {
            T x,y;
        };
        struct {
            T lat,lng;
        };
        struct {
            T width,height;
        };
    };
};
template<class T>
inline vec2<T>& vec2<T>::floor(){
    this->x = ::floor(this->x);
    this->y = ::floor(this->y);
    return *this;
}
template<class T>
inline vec2<T>& vec2<T>::ceil(){
    this->x = ::ceil(this->x);
    this->y = ::ceil(this->y);
    return *this;
}
template<class T>
inline vec2<T>& vec2<T>::round(){
    this->x = ::round(this->x);
    this->y = ::round(this->y);
    return *this;
}
template<class T>
inline bool vec2<T>::operator<(const vec2<T>& v){
    return (this->x < v.x && this->y< v.y);
}
template<class T>
inline bool vec2<T>::operator<=(const vec2<T>& v){
    return (this->x <= v.x && this->y<= v.y);
}
template<class T>
inline bool vec2<T>::operator==(const vec2<T>& v){
    return (this->x == v.x && this->y == v.y);
}
template<class T>
inline bool vec2<T>::operator!=(const vec2<T>& v){
    return (this->x != v.x || this->y != v.y);
}
template<class T>
inline bool vec2<T>::operator>(const vec2<T>& v){
    return (this->x > v.x && this->y > v.y);
}
template<class T>
inline bool vec2<T>::operator>=(const vec2<T>& v){
    return (this->x >= v.x && this->y >= v.y);
}

template<class T>
inline vec2<T> vec2<T>::lerp(const vec2<T>& t,T f){
    vec2<T> diff = (*this) - t;
    diff*=f;
    diff+=(*this);
    return diff;
}
template<class T>
inline vec2<T>& vec2<T>::operator=(const T& a){
    this->x = a;
    this->y = a;
    return *this;
}
template<class T>
inline vec2<T>& vec2<T>::operator=(const vec2<T>& v){
    this->x = v.x;
    this->y = v.y;
    return *this;
}

template<class T>
inline vec2<T> vec2<T>::operator+(const vec2<T>& v){
    vec2<T> vx(this->x+v.x,this->y+v.y);
    return vx;
}
template<class T>
inline vec2<T> vec2<T>::operator+(const T& v){
    vec2<T> vx(this->x+v,this->y+v);
    return vx;
}
template<class T>
inline vec2<T>& vec2<T>::operator+=(const vec2<T>& v){
    this->x += v.x;
    this->y += v.y;
    return *this;
}
template<class T>
inline vec2<T>& vec2<T>::operator+=(const T& v){
    this->x += v;
    this->y += v;
    return *this;
}
template<class T>
inline vec2<T> vec2<T>::operator-(const vec2<T>& v){
    vec2<T> vx(this->x-v.x,this->y-v.y);
    return vx;
}
template<class T>
inline vec2<T> vec2<T>::operator-(const T& v){
    vec2<T> vx(this->x-v,this->y-v);
    return vx;
}
template<class T>
inline vec2<T>& vec2<T>::operator-=(const vec2<T>& v){
    this->x -= v.x;
    this->y -= v.y;
    return *this;
}
template<class T>
inline vec2<T>& vec2<T>::operator-=(const T& v){
    this->x -= v;
    this->y -= v;
    return *this;
}

template<class T>
inline vec2<T> vec2<T>::operator*(const vec2<T>& v){
    vec2<T> vx(this->x*v.x,this->y*v.y);
    return vx;
}
template<class T>
inline vec2<T> vec2<T>::operator*(const T& v){
    vec2<T> vx(this->x*v,this->y*v);
    return vx;
}
template<class T>
inline vec2<T>& vec2<T>::operator*=(const vec2<T>& v){
    this->x *= v.x;
    this->y *= v.y;
    return *this;
}
template<class T>
inline vec2<T>& vec2<T>::operator*=(const T& v){
    this->x *= v;
    this->y *= v;
    return *this;
}

template<class T>
inline vec2<T> vec2<T>::operator/(const vec2<T>& v){
    vec2<T> vx(this->x/v.x,this->y/v.y);
    return vx;
}
template<class T>
inline vec2<T> vec2<T>::operator/(const T& v){
    vec2<T> vx(this->x/v,this->y/v);
    return vx;
}
template<class T>
inline vec2<T>& vec2<T>::operator/=(const vec2<T>& v){
    this->x /= v.x;
    this->y /= v.y;
    return *this;
}
template<class T>
inline vec2<T>& vec2<T>::operator/=(const T& v){
    this->x /= v;
    this->y /= v;
    return *this;
}

template<class T>
inline vec2<T>& vec2<T>::min(const vec2<T> &a, const vec2<T> &b){
    this->x = _min<T>(a.x,b.x);
    this->y = _min<T>(a.y,b.y);
    return *this;
}
template<class T>
inline vec2<T>& vec2<T>::max(const vec2<T> &a, const vec2<T> &b){
    this->x = _max<T>(a.x,b.x);
    this->y = _max<T>(a.y,b.y);
    return *this;
}
template<class T>
inline vec2<T>& vec2<T>::abs(){
    this->x = (T) fabs((f32)this->x);
    this->y = (T) fabs((f32)this->y);
    return *this;
}
template<class T>
inline T vec2<T>::lengthSq(){
    return (this->x*this->x)+(this->y*this->y);
}
template<class T>
inline T vec2<T>::length(){
    return sqrt(this->lengthSq());
}

template<class T>
inline T vec2<T>::distanceSq(const vec2<T>& v){
    vec2<T> tmp = (*this);
    tmp -= v;

    return tmp.lengthSq();
}
template<class T>
inline T vec2<T>::distance(const vec2<T>& v){
    return sqrt(this->distanceSq(v));
}
template<class T>
inline T vec2<T>::manhattan(const vec2<T>& v){
    vec2<T> d = (*this)-v;

    if(d.x<0)
        d.x = -d.x;

    if(d.y<0)
        d.y = -d.y;

    return d.x+d.y;
}



template<class T>
inline T vec2<T>::haversine(const vec2<T>& v){
    T dx, dy, dz;
    T vx = v.x;

    this->y -= v.y;
    this->y *= TO_RAD;
    this->x *= TO_RAD;

    vx *= TO_RAD;

    dz = sin(this->x) - sin(vx);
    dx = cos(this->y) * cos(this->x) - cos(vx);
    dy = sin(this->y) * cos(this->x);
    return asin(sqrt(dx * dx + dy * dy + dz * dz) / 2) * 2 * EARTH_RADIUS;
}
template<class T>
inline vec2<T>& vec2<T>::invert(){
    this->x = -this->x;
    this->y = -this->y;
    return *this;
}
template<class T>
inline T vec2<T>::dot(const vec2<T>& v){
    return (this->x * v.x)+(this->y*v.y);
}
template<class T>
inline vec2<T>& vec2<T>::rotate(const vec2<T>& point,float angle)
{
    T cs = cos(angle);
    T sn = sin(angle);

    T tx = this->x - point.x;
    T ty = this->y - point.y;
        
    T rx = tx * cs - ty * sn;
    T ry = tx * sn + ty * cs;
        
    this->x = rx + point.x;
    this->y = ry + point.y;
    return *this;
}


template<class T>
class vec3 {
public:
    vec3():
        x(0),
        y(0),
        z(0)
    {}
    vec3(T a):
        x(a),
        y(a),
        z(a)
    {}
    vec3(T a,T b,T c):
        x(a),
        y(b),
        z(c)
    {}
    vec3(const vec3<T>& a){
        this->x = a.x;
        this->y = a.y;
        this->z = a.z;
    }

    virtual ~vec3(){}

    virtual vec3<T>& operator=(const T& a);
    virtual vec3<T>& operator=(const vec3<T>& v);

    virtual vec3<T> operator+(const vec3<T>& v);
    virtual vec3<T> operator+(const T& v);
    virtual vec3<T>& operator+=(const vec3<T>& v);
    virtual vec3<T>& operator+=(const T& v);

    vec3<T> operator*(const vec3<T>& v);
    vec3<T> operator*(const T& v);
    vec3<T>& operator*=(const vec3<T>& v);
    vec3<T>& operator*=(const T& v);

    virtual vec3<T> operator-(const vec3<T>& v);
    virtual vec3<T> operator-(const T& v);
    virtual vec3<T>& operator-=(const vec3<T>& v);
    virtual vec3<T>& operator-=(const T& v);

    virtual vec3<T> operator/(const vec3<T>& v);
    virtual vec3<T> operator/(const T& v);
    virtual vec3<T>& operator/=(const vec3<T>& v);
    virtual vec3<T>& operator/=(const T& v);

    virtual vec3<T>& set(const T& a,const T& b,const T& c){
        this->x = a;
        this->y = b;
        this->z = c;
        return *this;
    }

    virtual vec3<T>& min(const vec3<T>& a,const vec3<T>& b);
    virtual vec3<T>& max(const vec3<T>& a,const vec3<T>& b);

    virtual T lengthSq();
    virtual T length();

    virtual T distanceSq(const vec3<T>& v);
    virtual T distance(const vec3<T>& v);

    virtual T normalize();

    virtual vec3<T> lerp(const vec3<T>& t,T f);
    //! Compute the dot product
    virtual T dot(const vec3<T>& v);
    //! Compute the cross product
    virtual vec3<T> cross(const vec3<T>& v);

    virtual void print(){
        printf("Vector: %f, %f, %f\r\n", (float)1.0f * this->x, (float)1.0f * this->y, (float)1.0f * this->z);
    }

    union {
        struct {
            T x,y,z;
        };
        struct {
            T r,g,b;
        };
        struct
        {
            T width, height, depth;
        };
        struct {
            T lat,lng,alt;
        };
    };
};
template<class T>
inline T vec3<T>::normalize(){
    T len = this->length();
    T invmag = 1.0f/len;

    this->x *= invmag;
    this->y *= invmag;
    this->z *= invmag;
    return len;
}
template<class T>
inline vec3<T> vec3<T>::lerp(const vec3<T>& t,T f){
    vec3<T> diff = (*this) - t;
    diff*=f;
    diff+=(*this);
    return diff;
}
template<class T>
inline vec3<T>& vec3<T>::operator=(const T& a){
    this->x = a;
    this->y = a;
    this->z = a;
    return *this;
}
template<class T>
inline vec3<T>& vec3<T>::operator=(const vec3<T>& v){
    this->x = v.x;
    this->y = v.y;
    this->z = v.z;
    return *this;
}

template<class T>
inline vec3<T> vec3<T>::operator+(const vec3<T>& v){
    vec3<T> vx(this->x+v.x,this->y+v.y,this->z+v.z);
    return vx;
}
template<class T>
inline vec3<T> vec3<T>::operator+(const T& v){
    vec3<T> vx(this->x+v,this->y+v,this->z+v);
    return vx;
}
template<class T>
inline vec3<T>& vec3<T>::operator+=(const vec3<T>& v){
    this->x += v.x;
    this->y += v.y;
    this->z += v.z;
    return *this;
}
template<class T>
inline vec3<T>& vec3<T>::operator+=(const T& v){
    this->x += v;
    this->y += v;
    this->z += v;
    return *this;
}

template<class T>
inline vec3<T> vec3<T>::operator-(const vec3<T>& v){
    vec3<T> vx(this->x-v.x,this->y-v.y,this->z -v.z);
    return vx;
}
template<class T>
inline vec3<T> vec3<T>::operator-(const T& v){
    vec3<T> vx(this->x-v,this->y-v,this->z -v);
    return vx;
}
template<class T>
inline vec3<T>& vec3<T>::operator-=(const vec3<T>& v){
    this->x -= v.x;
    this->y -= v.y;
    this->z -=v.z;
    return *this;
}
template<class T>
inline vec3<T>& vec3<T>::operator-=(const T& v){
    this->x -= v;
    this->y -= v;
    this->z -=v;
    return *this;
}

template<class T>
inline vec3<T> vec3<T>::operator*(const vec3<T>& v){
    vec3<T> vx(this->x*v.x,this->y*v.y,this->z*v.z);
    return vx;
}
template<class T>
inline vec3<T> vec3<T>::operator*(const T& v){
    vec3<T> vx(this->x*v,this->y*v,this->z*v);
    return vx;
}
template<class T>
inline vec3<T>& vec3<T>::operator*=(const vec3<T>& v){
    this->x *= v.x;
    this->y *= v.y;
    this->z *= v.z;
    return *this;
}
template<class T>
inline vec3<T>& vec3<T>::operator*=(const T& v){
    this->x *= v;
    this->y *= v;
    this->z *= v;
    return *this;
}

template<class T>
inline vec3<T> vec3<T>::operator/(const vec3<T>& v){
    vec3<T> vx(this->x/v.x,this->y/v.y,this->z/v.z);
    return vx;
}
template<class T>
inline vec3<T> vec3<T>::operator/(const T& v){
    vec3<T> vx(this->x/v,this->y/v,this->z/v);
    return vx;
}
template<class T>
inline vec3<T>& vec3<T>::operator/=(const vec3<T>& v){
    this->x /= v.x;
    this->y /= v.y;
    this->z /= v.z;
    return *this;
}
template<class T>
inline vec3<T>& vec3<T>::operator/=(const T& v){
    this->x /= v;
    this->y /= v;
    this->z /= v;
    return *this;
}

template<class T>
inline T vec3<T>::lengthSq(){
    return (this->x*this->x)+(this->y*this->y)+(this->z*this->z);
}
template<class T>
inline T vec3<T>::length(){
    return sqrt(this->lengthSq());
}

template<class T>
inline T vec3<T>::distanceSq(const vec3<T>& v){
    vec3<T> tmp = (*this);
    tmp -= v;

    return tmp.lengthSq();
}
template<class T>
inline T vec3<T>::distance(const vec3<T>& v){
    return sqrt(this->distanceSq(v));
}
template<class T>
inline T vec3<T>::dot(const vec3<T>& v){
    return (this->x*v.x)+(this->y*v.y)+(this->z*v.z);
}
template<class T>
inline vec3<T> vec3<T>::cross(const vec3<T>& v){
    vec3<T> ox(
        (this->y * v.z) - (this->z * v.y),
        (this->z * v.x) - (this->x * v.z),
        (this->x * v.y) - (this->y * v.x)
    );
    //! Return the new vector
    return ox;
}

template <class T>
inline vec3<T>& vec3<T>::min(const vec3<T> &a, const vec3<T> &b)
{
    this->x = _min<T>(a.x, b.x);
    this->y = _min<T>(a.y, b.y);
    this->z = _min<T>(a.z, b.z);
    return *this;
}
template <class T>
inline vec3<T>& vec3<T>::max(const vec3<T> &a, const vec3<T> &b)
{
    this->x = _max<T>(a.x, b.x);
    this->y = _max<T>(a.y, b.y);
    this->z = _max<T>(a.z, b.z);
    return *this;
}



template<class T>
class vec4 {
public:
    vec4():
        x(0),
        y(0),
        z(0),
        w(1)
    {}
    vec4(T a):
        x(a),
        y(a),
        z(a),
        w(a)
    {}
    vec4(T a,T b,T c,T d):
        x(a),
        y(b),
        z(c),
        w(d)
    {}
    vec4(const vec4<T>& a){
        this->x = a.x;
        this->y = a.y;
        this->z = a.z;
        this->w = a.w;
    }

    virtual ~vec4(){}

    virtual vec4<T>& operator=(const T& a);
    virtual vec4<T>& operator=(const vec4<T>& v);

    virtual vec4<T> operator+(const vec4<T>& v);
    virtual vec4<T> operator+(const T& v);
    virtual vec4<T>& operator+=(const vec4<T>& v);
    virtual vec4<T>& operator+=(const T& v);

    virtual vec4<T> operator*(const vec4<T>& v);
    virtual vec4<T> operator*(const T& v);
    virtual vec4<T>& operator*=(const vec4<T>& v);
    virtual vec4<T>& operator*=(const T& v);

    virtual vec4<T> operator-(const vec4<T>& v);
    virtual vec4<T> operator-(const T& v);
    virtual vec4<T>& operator-=(const vec4<T>& v);
    virtual vec4<T>& operator-=(const T& v);

    virtual vec4<T> operator/(const vec4<T>& v);
    virtual vec4<T> operator/(const T& v);
    virtual vec4<T>& operator/=(const vec4<T>& v);
    virtual vec4<T>& operator/=(const T& v);

    virtual vec4<T>& min(const vec4<T>& a,const vec4<T>& b);
    virtual vec4<T>& max(const vec4<T>& a,const vec4<T>& b);

    virtual T lengthSq();
    virtual T length();

    virtual T distanceSq(const vec4<T>& v);
    virtual T distance(const vec4<T>& v);

    virtual T normalize();

    virtual void print(){
        printf("Vector: %f, %f, %f, %f\r\n", (float)1.0f * this->x, (float)1.0f * this->y, (float)1.0f * this->z, (float)1.0f * this->w);
    }

    union {
        struct {
            T x,y,z,w;
        };
        struct {
            T r,g,b,a;
        };
        struct {
            T top,left,bottom,right;
        };
        struct {
            T values[4];
        };
    };
};
template<class T>
inline vec4<T>& vec4<T>::operator=(const T& a){
    this->x = a;
    this->y = a;
    this->z = a;
    this->w = a;
    return *this;
}
template<class T>
inline vec4<T>& vec4<T>::operator=(const vec4<T>& v){
    this->x = v.x;
    this->y = v.y;
    this->z = v.z;
    this->w = v.w;
    return *this;
}

template<class T>
inline vec4<T> vec4<T>::operator+(const vec4<T>& v){
    vec4<T> vx(this->x+v.x,this->y+v.y,this->z+v.z,this->w+v.w);
    return vx;
}
template<class T>
inline vec4<T> vec4<T>::operator+(const T& v){
    vec4<T> vx(this->x+v,this->y+v,this->z+v,this->w+v);
    return vx;
}
template<class T>
inline vec4<T>& vec4<T>::operator+=(const vec4<T>& v){
    this->x += v.x;
    this->y += v.y;
    this->z += v.z;
    this->w += v.w;
    return *this;
}
template<class T>
inline vec4<T>& vec4<T>::operator+=(const T& v){
    this->x += v;
    this->y += v;
    this->z += v;
    this->w += v;
    return *this;
}

template<class T>
inline vec4<T> vec4<T>::operator-(const vec4<T>& v){
    vec4<T> vx(this->x-v.x,this->y-v.y,this->z -v.z,this->w-v.w);
    return vx;
}
template<class T>
inline vec4<T> vec4<T>::operator-(const T& v){
    vec4<T> vx(this->x-v,this->y-v,this->z -v,this->w-v);
    return vx;
}
template<class T>
inline vec4<T>& vec4<T>::operator-=(const vec4<T>& v){
    this->x -= v.x;
    this->y -= v.y;
    this->z -=v.z;
    this->w -=v.w;
    return *this;
}
template<class T>
inline vec4<T>& vec4<T>::operator-=(const T& v){
    this->x -= v;
    this->y -= v;
    this->z -=v;
    this->w -=v;
    return *this;
}

template<class T>
inline vec4<T> vec4<T>::operator*(const vec4<T>& v){
    vec4<T> vx(this->x*v.x,this->y*v.y,this->z*v.z,this->w*v.w);
    return vx;
}
template<class T>
inline vec4<T> vec4<T>::operator*(const T& v){
    vec4<T> vx(this->x*v,this->y*v,this->z*v,this->w*v);
    return vx;
}
template<class T>
inline vec4<T>& vec4<T>::operator*=(const vec4<T>& v){
    this->x *= v.x;
    this->y *= v.y;
    this->z *= v.z;
    this->w *= v.w;
    return *this;
}
template<class T>
inline vec4<T>& vec4<T>::operator*=(const T& v){
    this->x *= v;
    this->y *= v;
    this->z *= v;
    this->w *= v;
    return *this;
}

template<class T>
inline vec4<T> vec4<T>::operator/(const vec4<T>& v){
    vec4<T> vx(this->x/v.x,this->y/v.y,this->z/v.z,this->w/v.w);
    return vx;
}
template<class T>
inline vec4<T> vec4<T>::operator/(const T& v){
    vec4<T> vx(this->x/v,this->y/v,this->z/v,this->w/v);
    return vx;
}
template<class T>
inline vec4<T>& vec4<T>::operator/=(const vec4<T>& v){
    this->x /= v.x;
    this->y /= v.y;
    this->z /= v.z;
    this->w /= v.w;
    return *this;
}
template<class T>
inline vec4<T>& vec4<T>::operator/=(const T& v){
    this->x /= v;
    this->y /= v;
    this->z /= v;
    this->w /= v;
    return *this;
}

template<class T>
inline T vec4<T>::lengthSq(){
    return (this->x*this->x)+(this->y*this->y)+(this->z*this->z)+(this->w*this->w);
}
template<class T>
inline T vec4<T>::length(){
    return sqrt(this->lengthSq());
}

template<class T>
inline T vec4<T>::distanceSq(const vec4<T>& v){
    vec4<T> tmp = (*this);
    tmp -= v;

    return tmp.lengthSq();
}
template<class T>
inline T vec4<T>::distance(const vec4<T>& v){
    return sqrt(this->distanceSq(v));
}
template <class T>
inline vec4<T>& vec4<T>::min(const vec4<T> &a, const vec4<T> &b)
{
    this->x = _min<T>(a.x, b.x);
    this->y = _min<T>(a.y, b.y);
    this->z = _min<T>(a.z, b.z);
    this->w = _min<T>(a.w, b.w);
    return *this;
}
template <class T>
inline vec4<T>& vec4<T>::max(const vec4<T> &a, const vec4<T> &b)
{
    this->x = _max<T>(a.x, b.x);
    this->y = _max<T>(a.y, b.y);
    this->z = _max<T>(a.z, b.z);
    this->w = _max<T>(a.w, b.w);
    return *this;
}
template<class T>
inline T vec4<T>::normalize(){
    T len = sqrt( (this->x*this->x) + (this->y*this->y) + (this->z*this->z) );
    T invmag = 1.0f/len;

    this->x *= invmag;
    this->y *= invmag;
    this->z *= invmag;
    return this->w = len;

}

typedef vec3<u8> rgb8;
typedef vec4<u8> rgba8;

typedef vec3<f32> rgb32;
typedef vec4<f32> rgba32;

typedef vec3<f64> rgb64;
typedef vec4<f64> rgba64;

#endif