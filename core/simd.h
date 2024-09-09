#ifndef SIMD_H
#define SIMD_H

#include "types.h"

#define SIMD_FLAG_AMD64             0x01
#define SIMD_FLAG_ARM               0x02
#define SIMD_FLAG_ARM64             0x04

#define SIMD_NEON                   0x10
#define SIMD_HELIUM                 0x20
#define SIMD_FMA                    0x40
#define SIMD_SSE                    0x80

//! Architecture specific includes

#if defined(__amd64__)
#include <x86intrin.h>
#include <immintrin.h>
#include <nmmintrin.h>
#include <pmmintrin.h>
#include <tmmintrin.h>
#include <xmmintrin.h>
#include <emmintrin.h>
#elif defined(__arm__)
#include <arm_neon.h>
#endif


double epsilon = 0.0000000001f;
u64 simdFlags = 0;

enum compareFunction : u8 {
    CMP_EQ_OQ,
    CMP_LT_OS,
    CMP_LE_OS,
    CMP_UNORD_Q, 
    CMP_NEQ_UQ,
    CMP_NLT_US,
    CMP_NLE_US,
    CMP_ORD_Q,
    CMP_EQ_UQ,
    CMP_NGE_US,
    CMP_NGT_US,
    CMP_FALSE_OQ,
    CMP_NEQ_OQ,
    CMP_GE_OS,
    CMP_GT_OS,
    CMP_TRUE_UQ,
    CMP_EQ_OS,
    CMP_LT_OQ,
    CMP_LE_OQ,
    CMP_UNORD_S,
    CMP_NEQ_US,
    CMP_NLT_UQ,
    CMP_NLE_UQ,
    CMP_ORD_S,
    CMP_EQ_US,
    CMP_NGE_UQ, 
    CMP_NGT_UQ,
    CMP_FALSE_OS, 
    CMP_NEQ_OS,
    CMP_GE_OQ,
    CMP_GT_OQ,
    CMP_TRUE_US
};

struct simdVector {
    union {
        __m256  fv;
        __m256i iv;
        __m256d dv;
        __m128  fxv[2];
        __m128i ixv[2];
        __m128d dxv[2];
        f32 vx[8];
        s32 ix[8];
        f64 dx[4];
        s16 half[16];
        u8 bytes[32];
        s8 str[32];
    };
};

class simd128 {
public:
    simd128(){}
    virtual ~simd128(){}

    //! 4x32b Float Arithmetic
    virtual simd128& fadd(const simd128& a,const simd128& b);
    virtual simd128& fsub(const simd128& a,const simd128& b);
    virtual simd128& fmul(const simd128& a,const simd128& b);
    virtual simd128& fma(const simd128& a,const simd128&b,const simd128& c);
    //! 4x32b Float Compare
    virtual simd128& fcmp(const simd128& a,const simd128&b);
    //! 8*16b Integer Arithmetic
    virtual simd128& iadd(const simd128& a,const simd128& b);
    virtual simd128& isub(const simd128& a,const simd128& b);
    virtual simd128& imul(const simd128& a,const simd128& b);
    //! 8*16b Integer Compare
    virtual simd128& icmp(const simd128& a,const simd128& b);

    //! 128b Logic
    virtual simd128& iand(const simd128& a,const simd128& b);
    virtual simd128& iandnot(const simd128& a,const simd128& b);
    virtual simd128& ior(const simd128& a,const simd128& b);
    virtual simd128& ixor(const simd128& a,const simd128& b);

    union {
        u8 bytes[16];
#if defined(__amd64__)
        __m128 fv;
        __m128i sv;
#elif defined(__arm__)
        float32x4_t fv;
        int16x8_t sv;
        uint32x4_t dwords;
        uint64x2_t qwords;
#endif

    };
};
inline simd128& simd128::fadd(const simd128& a,const simd128& b){
#if defined(__amd64__)
    this->fv = _mm_add_ps(a.fv,b.fv);
#elif defined(__arm__)
    this->fv = vaddq_f32(a.fv,b.fv);
#endif
    return *this;
}
inline simd128& simd128::fsub(const simd128& a,const simd128& b){
#if defined(__amd64__)
    this->fv = _mm_sub_ps(a.fv,b.fv);
#elif defined(__arm__)
    this->fv = vsubq_f32(a.fv,b.fv);
#endif
    return *this;
}
inline simd128& simd128::fmul(const simd128& a,const simd128& b){
#if defined(__amd64__)
    this->fv = _mm_mul_ps(a.fv,b.fv);
#elif defined(__arm__)
    this->fv = vmulq_f32(a.fv,b.fv);
#endif
    return *this;
}
inline simd128& simd128::fma(const simd128& a,const simd128&b,const simd128& c){
#if defined(__amd64__)
    //this->fv = _mm_fmadd_ps(a.fv,b.fv,c.fv);
#elif defined(__arm__)
    this->fv = vfmaq_f32(a.fv,b.fv,c.fv);
#endif
    return *this;
}
inline simd128& simd128::fcmp(const simd128& a,const simd128& b){
#if defined(__amd64__)
    this->fv = _mm_cmpeq_ps(a.fv,b.fv);
#elif defined(__arm__)
    this->words = vceqq_f32(a.fv,b.fv);
#endif
    return *this;
}

inline simd128& simd128::iadd(const simd128& a,const simd128& b){
#if defined(__amd64__)
    this->sv = _mm_add_epi16(a.sv,b.sv);
#elif defined(__arm__)
    this->sv = vaddq_s16(a.sv,b.sv);
#endif
    return *this;
}
inline simd128& simd128::isub(const simd128& a,const simd128& b){
#if defined(__amd64__)
    this->sv = _mm_sub_epi16(a.sv,b.sv);
#elif defined(__arm__)
    this->sv = vsubq_s16(a.sv,b.sv);
#endif
    return *this;
}
inline simd128& simd128::imul(const simd128& a,const simd128& b){
#if defined(__amd64__)
    this->sv = _mm_mullo_epi16(a.sv,b.sv);
#elif defined(__arm__)
    this->sv = vmulq_s16(a.sv,b.sv);
#endif
    return *this;
}
inline simd128& simd128::icmp(const simd128& a,const simd128& b){
#if defined(__amd64__)
    this->sv = _mm_cmpeq_epi16(a.sv,b.sv);
#elif defined(__arm__)
    this->sv = vceqq_s16(a.sv,b.sv);
#endif
    return *this;
}
inline simd128& simd128::iand(const simd128& a,const simd128& b){
#if defined(__amd64__)
    this->sv = _mm_and_si128(a.sv,b.sv);
#elif defined(__arm__)
    this->sv = vandq_u16(a.sv,b.sv);
#endif
    return *this;
}
inline simd128& simd128::iandnot(const simd128& a,const simd128& b){
#if defined(__amd64__)
    this->sv = _mm_andnot_si128(a.sv,b.sv);
#elif defined(__arm__)
    this->sv = vandq_u16(a.sv,b.sv);
#endif
    return *this;
}
inline simd128& simd128::ior(const simd128& a,const simd128& b){
#if defined(__amd64__)
    this->sv = _mm_or_si128(a.sv,b.sv);
#elif defined(__arm__)
    this->sv = vorrq_u16(a.sv,b.sv);
#endif
    return *this;
}
inline simd128& simd128::ixor(const simd128& a,const simd128& b){
#if defined(__amd64__)
    this->sv = _mm_xor_si128(a.sv,b.sv);
#elif defined(__arm__)
    this->qwords = vxarq_u64(a.qwords,b.qwords);
#endif
    return *this;
}

class simd256 {
public:
    simd256(){}
    simd256(f32 a){this->set(a);}
    simd256(s32 a){this->set(a);}
    simd256(s64 a,s64 b,s64 c,s64 d){this->set(a,b,c,d);}
    simd256(s32 a,s32 b,s32 c,s32 d,s32 e,s32 f,s32 g,s32 h){this->set(a,b,c,d,e,f,g,h);}
    simd256(f32 a,f32 b,f32 c,f32 d,f32 e,f32 f,f32 g,f32 h){this->set(a,b,c,d,e,f,g,h);}
    virtual ~simd256(){}

    virtual simd256& qset(s64 a);
    virtual simd256& bset(s8 a);
    virtual simd256& set(f32 a);
    virtual simd256& set(s32 a);
    virtual simd256& set(s64 a,s64 b,s64 c,s64 d);
    virtual simd256& set(s32 a,s32 b,s32 c,s32 d,s32 e,s32 f,s32 g,s32 h);
    virtual simd256& set(f32 a,f32 b,f32 c,f32 d,f32 e,f32 f,f32 g,f32 h);

    //! 4*64b Double Arithmetic
    virtual simd256& dadd(const simd256& a,const simd256& b);
    virtual simd256& dsub(const simd256& a,const simd256& b);
    virtual simd256& dmul(const simd256& a,const simd256& b);
    virtual simd256& dfma(const simd256& a,const simd256& b,const simd256& c);
    //! 4*64b Double Compare
    virtual simd256& dcmp(const simd256& a,const simd256& b);
    //! 8*32b Float Arithmetic
    virtual simd256& fadd(const simd256& a);
    virtual simd256& fadd(const simd256& a,const simd256& b);
    virtual simd256& fsub(const simd256& a);
    virtual simd256& fsub(const simd256& a,const simd256& b);
    virtual simd256& fmul(const simd256& a);
    virtual simd256& fmul(const simd256& a,const simd256& b);
    virtual simd256& fdiv(const simd256& a);
    virtual simd256& fdiv(const simd256& a,const simd256& b);

    virtual simd256& fma(const simd256& a,const simd256& b);
    virtual simd256& fma(const simd256& a,const simd256& b,const simd256& c);
    //! 8*32b Float Maths
    virtual simd256& fcos(const simd256& a);
    virtual simd256& sqrt(const simd256& a);
    virtual simd256& rsqrt(const simd256& a);
    virtual simd256& floor(const simd256& a);
    virtual simd256& ceil(const simd256& a);
    virtual simd256& round(const simd256& a);
    virtual simd256& hypot(const simd256& a,const simd256& b);
    virtual simd256& hypotsq(const simd256& a,const simd256& b);
    virtual simd256& length(const simd256& x,const simd256& y,const simd256& z);
    virtual simd256& lengthsq(const simd256& x,const simd256& y,const simd256& z);
    //! 8*32b Float Compare
    virtual simd256& feq(const simd256& a,const simd256& b);
    virtual simd256& fneq(const simd256& a,const simd256& b);
    virtual simd256& flt(const simd256& a,const simd256& b);
    virtual simd256& flte(const simd256& a,const simd256& b);
    virtual simd256& fgt(const simd256& a,const simd256& b);
    virtual simd256& fgte(const simd256& a,const simd256& b);
    //! 8*32b Integer Arithmetic
    virtual simd256& iadd(const simd256& a);
    virtual simd256& iadd(const simd256& a,const simd256& b);
    virtual simd256& isub(const simd256& a);
    virtual simd256& isub(const simd256& a,const simd256& b);
    virtual simd256& imul(const simd256& a);
    virtual simd256& imul(const simd256& a,const simd256& b);

    virtual simd256& isign(const simd256& a,const simd256& b);
    virtual simd256& iinc();
    virtual simd256& idec();
    //! 8*32b Integer Logical
    virtual simd256& iand(const simd256& a,const simd256& b);
    virtual simd256& iandnot(const simd256& a,const simd256& b);
    virtual simd256& inand(const simd256& a,const simd256& b);
    virtual simd256& ior(const simd256& a,const simd256& b);
    virtual simd256& ixor(const simd256& a,const simd256& b);
    virtual simd256& inot(const simd256& a);
    virtual simd256& inor(const simd256& a,const simd256 &b);
    virtual simd256& ixnor(const simd256& a,const simd256& b);
    virtual simd256& iorandnot(const simd256& a,const simd256& b,const simd256& c);
    //! 8*32b Integer Maths
    virtual simd256& imin(const simd256& a,const simd256& b);
    virtual simd256& imax(const simd256& a,const simd256& b);
    //! 8*32b Integer Shift
    virtual simd256& ishiftleft(const simd256& a,int amount);
    virtual simd256& ishiftright(const simd256& a,int amount);
    //! 8*32b Integer Permutation
    virtual simd256& ipermute(const simd256& a,const simd256& b);
    virtual simd256& ishuffle8(const simd256& a,const simd256& mask);
    virtual simd256& ishuffle32int(const simd256& a);
    virtual simd256& ishuffle32deint(const simd256& a);
    //! 8*32b Integer Compare
    virtual simd256& ieq(const simd256& a,const simd256& b);
    virtual simd256& ineq(const simd256& a,const simd256& b);
    virtual simd256& igt(const simd256& a,const simd256& b);
    virtual simd256& igte(const simd256& a,const simd256& b);
    virtual simd256& ilt(const simd256& a,const simd256& b);
    virtual simd256& ilte(const simd256& a,const simd256& b);
    //! 4*64b Integer Arithmetic
    virtual simd256& qadd(const simd256& a);
    virtual simd256& qadd(const simd256& a,const simd256& b);
    virtual simd256& qsub(const simd256& a);
    virtual simd256& qsub(const simd256& a,const simd256& b);
    //! 4*64b Integer Compare
    virtual simd256& qeq(const simd256& a,const simd256& b);
    virtual simd256& qneq(const simd256& a,const simd256& b);
    virtual simd256& qgt(const simd256& a,const simd256& b);
    virtual simd256& qgte(const simd256& a,const simd256& b);
    virtual simd256& qlt(const simd256& a,const simd256& b);
    virtual simd256& qlte(const simd256& a,const simd256& b);
    //! 32x8b Integer Compare
    virtual simd256& beq(const simd256& a,const simd256& b);
    virtual simd256& bneq(const simd256& a,const simd256& b);
    virtual simd256& bgt(const simd256& a,const simd256& b);
    virtual simd256& bgte(const simd256& a,const simd256& b);
    virtual simd256& blt(const simd256& a,const simd256& b);
    virtual simd256& blte(const simd256& a,const simd256& b);

    virtual void print(){
        printf("0)\t0x%08x\t1)\t0x%08x\t2)\t0x%08x\t3)\t0x%08x\n4)\t0x%08x\t5)\t0x%08x\t6)\t0x%08x\t7)\t0x%08x\n",
        this->up[0],
        this->up[1],
        this->up[2],
        this->up[3],
        this->up[4],
        this->up[5],
        this->up[6],
        this->up[7]);
    }

    union PACKED_STRUCT {
        __m256 fv;
        __m256i iv;
        __m256d dv;
        s32 up[8];
        s64 us[4];
        f32 fp[8];
        s8 str[32];
    };
};
inline simd256& simd256::qset(s64 a){
    this->iv = _mm256_set1_epi64x(a);
    return *this;
}
inline simd256& simd256::bset(s8 a){
    this->iv = _mm256_set1_epi8(a);
    return *this;
}
inline simd256& simd256::set(f32 a){
    this->fv = _mm256_set1_ps(a);
    return *this;
}
inline simd256& simd256::set(s32 a){
    this->iv = _mm256_set1_epi32(a);
    return *this;
}
inline simd256& simd256::set(s64 a,s64 b,s64 c,s64 d){
    this->iv = _mm256_set_epi64x(a,b,c,d);
    return *this;
}
inline simd256& simd256::set(s32 a,s32 b,s32 c,s32 d,s32 e,s32 f,s32 g,s32 h){
    this->iv = _mm256_set_epi32(a,b,c,d,e,f,g,h);
    return *this;
}
inline simd256& simd256::set(f32 a,f32 b,f32 c,f32 d,f32 e,f32 f,f32 g,f32 h){
    this->fv = _mm256_set_ps(a,b,c,d,e,f,g,h);
    return *this;
}
inline simd256& simd256::dadd(const simd256& a,const simd256& b){
    this->dv = _mm256_add_pd(a.dv,b.dv);
    return *this;
}
inline simd256& simd256::dsub(const simd256& a,const simd256& b){
    this->dv = _mm256_sub_pd(a.dv,b.dv);
    return *this;
}
inline simd256& simd256::dmul(const simd256& a,const simd256& b){
    this->dv = _mm256_mul_pd(a.dv,b.dv);
    return *this;
}
inline simd256& simd256::dfma(const simd256& a,const simd256& b,const simd256& c){
    this->dv = _mm256_fmadd_pd(a.dv,b.dv,c.dv);
    return *this;
}
inline simd256& simd256::dcmp(const simd256& a,const simd256& b){
    this->dv = _mm256_cmp_pd(a.dv,b.dv,0x00);
    return *this;
}
inline simd256& simd256::fadd(const simd256& a){
    this->fv = _mm256_add_ps(a.fv,this->fv);
    return *this;
}
inline simd256& simd256::fadd(const simd256& a,const simd256& b){
    this->fv = _mm256_add_ps(a.fv,b.fv);
    return *this;
}
inline simd256& simd256::fsub(const simd256& a){
    this->fv = _mm256_sub_ps(a.fv,this->fv);
    return *this;
}
inline simd256& simd256::fsub(const simd256& a,const simd256& b){
    this->fv = _mm256_sub_ps(a.fv,b.fv);
    return *this;
}
inline simd256& simd256::fmul(const simd256& a){
    this->fv = _mm256_mul_ps(a.fv,this->fv);
    return *this;
}
inline simd256& simd256::fmul(const simd256& a,const simd256& b){
    this->fv = _mm256_mul_ps(a.fv,b.fv);
    return *this;
}
inline simd256& simd256::fdiv(const simd256& a){
    this->fv = _mm256_div_ps(a.fv,this->fv);
    return *this;
}
inline simd256& simd256::fdiv(const simd256& a,const simd256& b){
    this->fv = _mm256_div_ps(a.fv,b.fv);
    return *this;
}
inline simd256& simd256::fma(const simd256& a,const simd256& b){
    this-> fv = _mm256_fmadd_ps(a.fv,b.fv,this->fv);
    return *this;
}
inline simd256& simd256::fma(const simd256& a,const simd256& b,const simd256& c){
    this-> fv = _mm256_fmadd_ps(a.fv,b.fv,c.fv);
    return *this;
}
inline simd256& simd256::fcos(const simd256& a){
    //! Use taylor series if required
    return *this;
}
inline simd256& simd256::sqrt(const simd256& a){
    this->fv = _mm256_sqrt_ps(a.fv);
    return *this;
}
inline simd256& simd256::rsqrt(const simd256& a){
    this->fv = _mm256_rsqrt_ps(a.fv);
    return *this;
}
inline simd256& simd256::floor(const simd256& a){
    this->fv = _mm256_floor_ps(a.fv);
    return *this;
}
inline simd256& simd256::ceil(const simd256& a){
    this->fv = _mm256_ceil_ps(a.fv);
    return *this;
}
inline simd256& simd256::round(const simd256& a){
    this->fv = _mm256_round_ps(a.fv,_MM_FROUND_TO_NEAREST_INT);
    return *this;
}
inline simd256& simd256::hypot(const simd256& a,const simd256& b){
    simd256 asq;
    simd256 bsq;
    simd256 comb;

    asq.fmul(a,a);
    bsq.fmul(b,b);

    comb.fadd(asq,bsq);
    this->fdiv(comb,comb);
    return *this;
}
inline simd256& simd256::hypotsq(const simd256& a,const simd256& b){
    simd256 asq;
    simd256 bsq;

    asq.fmul(a,a);
    bsq.fmul(b,b);

    this->fadd(asq,bsq);
    return *this;
}
inline simd256& simd256::length(const simd256& x,const simd256& y,const simd256& z){
    simd256 xsq;
    simd256 ysq;
    simd256 zsq;
    simd256 intr0;
    simd256 intr1;

    xsq.fmul(x,x);
    ysq.fmul(y,y);
    zsq.fmul(z,z);

    intr0.fadd(xsq,ysq);
    intr1.fadd(intr0,zsq);
    this->fdiv(intr1,intr1);
    return *this;
}
inline simd256& simd256::lengthsq(const simd256& x,const simd256& y,const simd256& z){
    simd256 xsq;
    simd256 ysq;
    simd256 zsq;
    simd256 intr;

    xsq.fmul(x,x);
    ysq.fmul(y,y);
    zsq.fmul(z,z);

    intr.fadd(xsq,ysq);
    this->fadd(intr,zsq);
    return *this;
}
inline simd256& simd256::feq(const simd256& a,const simd256& b){
    this->fv = _mm256_cmp_ps(a.fv,b.fv,_CMP_EQ_OQ);
    return *this;
}
inline simd256& simd256::fneq(const simd256& a,const simd256& b){
    this->fv = _mm256_cmp_ps(a.fv,b.fv,_CMP_NEQ_OQ);
    return *this;
}
inline simd256& simd256::flt(const simd256& a,const simd256& b){
    this->fv = _mm256_cmp_ps(a.fv,b.fv,_CMP_LT_OQ);
    return *this;
}
inline simd256& simd256::flte(const simd256& a,const simd256& b){
    this->fv = _mm256_cmp_ps(a.fv,b.fv,_CMP_LE_OQ);
    return *this;
}
inline simd256& simd256::fgt(const simd256& a,const simd256& b){
    this->fv = _mm256_cmp_ps(a.fv,b.fv,_CMP_GT_OQ);
    return *this;
}
inline simd256& simd256::fgte(const simd256& a,const simd256& b){
    this->fv = _mm256_cmp_ps(a.fv,b.fv,_CMP_GE_OQ);
    return *this;
}
inline simd256& simd256::iadd(const simd256& a,const simd256& b){
    this->iv = _mm256_add_epi32(a.iv,b.iv);
    return *this;
}
inline simd256& simd256::isub(const simd256& a,const simd256& b){
    this->iv = _mm256_sub_epi32(a.iv,b.iv);
    return *this;
}
inline simd256& simd256::imul(const simd256& a,const simd256& b){
    this->iv = _mm256_mullo_epi32(a.iv,b.iv);
    return *this;
}
inline simd256& simd256::iadd(const simd256& a){
    this->iv = _mm256_add_epi32(a.iv,this->iv);
    return *this;
}
inline simd256& simd256::isub(const simd256& a){
    this->iv = _mm256_sub_epi32(a.iv,this->iv);
    return *this;
}
inline simd256& simd256::imul(const simd256& a){
    this->iv = _mm256_mullo_epi32(a.iv,this->iv);
    return *this;
}
inline simd256& simd256::isign(const simd256& a,const simd256& b){
    this->iv = _mm256_sign_epi32(a.iv,b.iv);
    return *this;
}
inline simd256& simd256::iinc(){
    simd256 ones(1);
    this->iadd(*this,ones);
    return *this;
}
inline simd256& simd256::idec(){
    simd256 ones(1);
    this->isub(*this,ones);
    return *this;
}
inline simd256& simd256::iand(const simd256& a,const simd256& b){
    this->fv = _mm256_and_ps(a.fv,b.fv);
    return *this;
}
inline simd256& simd256::iandnot(const simd256& a,const simd256& b){
    this->fv = _mm256_andnot_ps(a.fv,b.fv);
    return *this;
}
inline simd256& simd256::ior(const simd256& a,const simd256& b){
    this->fv = _mm256_or_ps(a.fv,b.fv);
    return *this;
}
inline simd256& simd256::ixor(const simd256& a,const simd256& b){
    this->fv = _mm256_xor_ps(a.fv,b.fv);
    return *this;
}
inline simd256& simd256::inot(const simd256& a){
    this->iv = _mm256_xor_si256 (a.iv, _mm256_set1_epi32(0xffffffff));
    return *this;
}
inline simd256& simd256::inand(const simd256& a,const simd256& b){
    this->iand(a,b);
    this->inot(*this);
    return *this;
}
inline simd256& simd256::inor(const simd256& a,const simd256& b){
    this->ior(a,b);
    this->inot(*this);
    return *this;
}
inline simd256& simd256::ixnor(const simd256& a,const simd256& b){
    simd256 reg0;
    //reg0.inand(a,b);
    this->iorandnot(a,b,reg0);
    return *this;
}
inline simd256& simd256::iorandnot(const simd256& a,const simd256& b,const simd256& c){
    simd256 reg0;
    reg0.ior(a,b);
    //this->inand(reg0,c);
    return *this;
}
inline simd256& simd256::imin(const simd256& a,const simd256& b){
    this->iv = _mm256_min_epi32(a.iv,b.iv);
    return *this;
}
inline simd256& simd256::imax(const simd256& a,const simd256& b){
    this->iv = _mm256_max_epi32(a.iv,b.iv);
    return *this;
}
inline simd256& simd256::ishiftleft(const simd256& a,int amount){
    this->iv = _mm256_slli_epi32(a.iv,amount);
    return *this;
}
inline simd256& simd256::ishiftright(const simd256& a,int amount){
    this->iv = _mm256_srli_epi32(a.iv,amount);
    return *this;
}
inline simd256& simd256::ipermute(const simd256& a,const simd256& b){
    this->iv = _mm256_permutevar8x32_epi32(a.iv,b.iv);
    return *this;
}
inline simd256& simd256::ishuffle8(const simd256& a,const simd256& mask){
    this->iv = _mm256_shuffle_epi8(a.iv,mask.iv);
    return *this;
}
inline simd256& simd256::ishuffle32int(const simd256& a){
    this->iv = _mm256_shuffle_epi32(a.iv,0xAA);
    return *this;
}
inline simd256& simd256::ishuffle32deint(const simd256& a){
    this->iv = _mm256_shuffle_epi32(a.iv,0x55);
    return *this;
}
inline simd256& simd256::ieq(const simd256& a,const simd256& b){
    this->iv = _mm256_cmpeq_epi32(a.iv,b.iv);
    return *this;
}
inline simd256& simd256::ineq(const simd256& a,const simd256& b){
    this->ieq(a,b);
    this->inot(*this);
    return *this;
}
inline simd256& simd256::igt(const simd256& a,const simd256& b){
    this->iv = _mm256_cmpgt_epi32(a.iv,b.iv);
    return *this;
}
inline simd256& simd256::igte(const simd256& a,const simd256& b){
    simd256 reg0;
    simd256 reg1;

    reg0.ieq(a,b);
    reg1.igt(a,b);
    this->ior(reg0,reg1);

    return *this;
}
inline simd256& simd256::ilt(const simd256& a,const simd256& b){
    simd256 reg0(0);
    //! Check if a == b
    reg0.ieq(a,b);
    //! Check if a > b
    this->igt(a,b);
    //! Manipulate the two bit fields
    this->inor(reg0,*this);
    return *this;
}
inline simd256& simd256::ilte(const simd256& a,const simd256& b){
    this->igt(a,b);
    this->inot(*this);
    return *this;
}
inline simd256& simd256::qadd(const simd256& a){
    this->iv = _mm256_add_epi64(a.iv,this->iv);
    return *this;
}
inline simd256& simd256::qadd(const simd256& a,const simd256& b){
    this->iv = _mm256_add_epi64(a.iv,b.iv);
    return *this;
}
inline simd256& simd256::qsub(const simd256& a){
    this->iv = _mm256_sub_epi64(a.iv,this->iv);
    return *this;
}
inline simd256& simd256::qsub(const simd256& a,const simd256& b){
    this->iv = _mm256_sub_epi64(a.iv,b.iv);
    return *this;
}

inline simd256& simd256::qeq(const simd256& a,const simd256& b){
    this->iv = _mm256_cmpeq_epi64(a.iv,b.iv);
    return *this;
}
inline simd256& simd256::qneq(const simd256& a,const simd256& b){
    this->iv = _mm256_cmpeq_epi64(a.iv,b.iv);
    this->inot(*this);
    return *this;
}
inline simd256& simd256::qgt(const simd256& a,const simd256& b){
    this->iv = _mm256_cmpgt_epi64(a.iv,b.iv);
    return *this;
}
inline simd256& simd256::qgte(const simd256& a,const simd256& b){
    simd256 reg0;
    simd256 reg1;

    reg0.qeq(a,b);
    reg1.qgt(a,b);
    this->ior(reg0,reg1);
    return *this;
}
inline simd256& simd256::qlt(const simd256& a,const simd256& b){
    simd256 reg0(0);
    //! Check if a == b
    reg0.qeq(a,b);
    //! Check if a > b
    this->qgt(a,b);
    //! Manipulate the two bit fields
    this->inor(reg0,*this);
    return *this;
}
inline simd256& simd256::qlte(const simd256& a,const simd256& b){
    this->qgt(a,b);
    this->inot(*this);
    return *this;
}
inline simd256& simd256::beq(const simd256& a,const simd256& b){
    this->iv = _mm256_cmpeq_epi8(a.iv,b.iv);
    return *this;
}
inline simd256& simd256::bneq(const simd256& a,const simd256& b){
    this->iv = _mm256_cmpeq_epi8(a.iv,b.iv);
    this->inot(*this);
    return *this;
}
inline simd256& simd256::bgt(const simd256& a,const simd256& b){
    this->iv = _mm256_cmpgt_epi8(a.iv,b.iv);
    return *this;
}
inline simd256& simd256::bgte(const simd256& a,const simd256& b){
    simd256 reg0;
    simd256 reg1;

    reg0.beq(a,b);
    reg1.bgt(a,b);
    this->ior(reg0,reg1);
    return *this;
}
inline simd256& simd256::blt(const simd256& a,const simd256& b){
    simd256 reg0(0);
    //! Check if a == b
    reg0.beq(a,b);
    //! Check if a > b
    this->bgt(a,b);
    //! Manipulate the two bit fields
    this->inor(reg0,*this);
    return *this;
}
inline simd256& simd256::blte(const simd256& a,const simd256& b){
    this->bgt(a,b);
    this->inot(*this);
    return *this;
}
/*

class simd512 {
public:
    simd512(){}
    virtual ~simd512(){}

    //! 
    virtual simd512& fadd(const simd512& a,const simd512& b);
    virtual simd512& fsub(const simd512& a,const simd512& b);
    virtual simd512& fmul(const simd512& a,const simd512& b);
    virtual simd512& fma(const simd512& a,const simd512& b);

    union {
        __m512 fv;
        __m512i iv;
        __m512d dv;
        //!__m512h hv;
    };
};

*/

class crc32 {
public:
    crc32(){}
    virtual ~crc32(){}

    bool equal(const crc32& c);

    void update(u8 v);
    void update(u16 v);
    void update(u32 v);
    void update(u64 v);

    u32 checksum;
};
bool crc32::equal(const crc32& c){
    return (this->checksum == c.checksum)?true:false;
}

void crc32::update(u8 v){
#if defined(__amd64__)
    this->checksum = _mm_crc32_u8(this->checksum,v);
#elif defined(__arm__)
    this->checksum = __crc32b(this->checksum,v);
#endif
}
void crc32::update(u16 v){
#if defined(__amd64__)
    this->checksum = _mm_crc32_u16(this->checksum,v);
#elif defined(__arm__)
    this->checksum = __crc32h(this->checksum,v);
#endif
}
void crc32::update(u32 v){
#if defined(__amd64__)
    this->checksum = _mm_crc32_u32(this->checksum,v);
#elif defined(__arm__)
    this->checksum = __crc32w(this->checksum,v);
#endif
}
void crc32::update(u64 v){
#if defined(__amd64__)
    this->checksum = _mm_crc32_u64(this->checksum,v);
#elif defined(__arm__)
    this->checksum = __crc32d(this->checksum,v);
#endif
}

const simd256 zeroRegister((int)0x00000000);
const simd256 maxRegister((int)0xFFFFFFFF);
const simd256 oneRegister((int)0x00000001);
const simd256 oneRegister64(
    (s64)0x0000000000000001,
    (s64)0x0000000000000001,
    (s64)0x0000000000000001,
    (s64)0x0000000000000001
);

#endif