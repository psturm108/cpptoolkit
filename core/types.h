#ifndef CTK_TYPES_H
#define CTK_TYPES_H

#if defined(ENABLE_JSON)
	//! Include RapidJSON Library
	#include <rapidjson/document.h>
	#include <rapidjson/rapidjson.h>
#endif

#if defined(_WIN32)
	//! We are using Windows
	#define CTK_WIN

	#include <windows.h>

#elif defined(__linux__)
	//! We are using Linux
	#define CTK_NIX
	
	#include <arpa/inet.h>
	#include <dlfcn.h>
	#include <dirent.h>
	#include <errno.h>
	#include <fcntl.h>           /* Definition of AT_* constants */
	#include <ifaddrs.h>
	#include <inttypes.h>
	#include <linux/fs.h>
	#include <linux/hw_breakpoint.h>
	#include <linux/i2c.h>
	#include <linux/i2c-dev.h>
	#include <linux/input.h>
	#include <linux/pci.h>
	#include <linux/perf_event.h>
	#include <linux/spi/spidev.h>
	#include <linux/videodev2.h>
	#include <math.h>
	#include <net/if.h>
	#include <netinet/in.h> /* struct sockaddr_in, struct sockaddr */
	#include <netdb.h>
	#include <pthread.h>
	#include <resolv.h>
	#include <signal.h>
	#include <stdio.h> /* printf, sprintf */
	#include <stdlib.h> /* exit */
	#include <string.h> /* memcpy, memset */
	#include <sys/epoll.h>
	#include <sys/inotify.h>
	#include <sys/ioctl.h>
	#include <sys/mman.h>
	#include <sys/sendfile.h>
	#include <sys/socket.h> /* socket, connect */
	#include <sys/stat.h>
	#include <sys/statfs.h>
	#include <sys/statvfs.h>
	#include <sys/syscall.h>
	#include <sys/timerfd.h>
	#include <sys/types.h>
	#include <sys/utsname.h>
	#include <sys/vfs.h>
	#include <termios.h>
	#include <time.h>
	#include <unistd.h>
	#include <gpiod.h>

#else

#endif

//! Include POSIX headers
#include <stdio.h>
#include <stdlib.h>

//! Include C++ headers
#include <string>
#include <map>
#include <list>
#include <array>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <functional>
#include <cctype>
#include <locale>
#include <unordered_map>

#if defined(ENABLE_IMAGE)
	//! Include general libraries
	#include <png.h>
	#include <jpeglib.h>
#endif

#if defined(ENABLE_SSL)
	//! Include OpenSSL
	#include <openssl/bio.h>
	#include <openssl/ssl.h>
	#include <openssl/err.h>
	#include <openssl/pem.h>
	#include <openssl/x509.h>
	#include <openssl/x509_vfy.h>
	#include <openssl/sha.h>
	#include <openssl/evp.h>
	#include <openssl/rsa.h>
#endif

//! Include processor intrinsics
#if defined(__amd64__)
	#include <x86intrin.h>
	#include <immintrin.h>
	#include <nmmintrin.h>
	#include <pmmintrin.h>
	#include <tmmintrin.h>
	#include <xmmintrin.h>
	#include <emmintrin.h>
	#include <x86gprintrin.h>
#elif defined(__arm__)
	#include <arm_neon.h>
#endif

#define PACKED_STRUCT   __attribute__((packed))

#define MAX_EPOLL_EVENTS 1024

#define SPI_HANDLE_MODE             0xC0000000
#define SPI_HANDLE_SPEED            0x00FF0000

#define DEFAULT_BUFFER_SIZE         4096
#define EVENT_SIZE  (sizeof(struct inotify_event))
#define EVENT_BUFFER_SIZE     (1*(EVENT_SIZE + 16))

#define AIO_DATA        0x01
#define AIO_DRAIN       0x02
#define AIO_ERROR       0x04
#define AIO_HANGUP      0x08
#define AIO_SHUTDOWN    0x10
#define AIO_EXCEPTION   0x20


namespace ctk {
	typedef char s8;
	typedef short s16;
	typedef int s32;
	typedef long long int s64;

	typedef unsigned char u8;
	typedef unsigned short u16;
	typedef unsigned int u32;
	typedef unsigned long long int u64;

	typedef float f32;
	typedef double f64;
	typedef long double f80;

	const f80 pi = 3.1415926535897932384626433832795028841971693993751058209749445923078164062;

	template<class T>
	inline T deg2rad(T d){
	  return (pi/180)*d;
	}
	inline T rad2deg(T d){
	  return (180/pi)*d;
	}

	inline u32 bit_set(u32 number, u32 n) {
	    return number | ((u32)1 << n);
	}
	inline u32 bit_clear(u32 number, u32 n) {
	    return number & ~((u32)1 << n);
	}
	inline u32 bit_toggle(u32 number, u32 n) {
	    return number ^ ((u32)1 << n);
	}
	inline bool bit_check(u32 number, u32 n) {
	    return (number >> n) & (u32)1;
	}
	inline u32 bit_set_to(u32 number, u32 n, bool x) {
	    return (number & ~((u32)1 << n)) | ((u32)x << n);
	}
	template<class T>
	inline T _min(const T& a,const T& b){
	    return (a<b)?a:b;
	}
	template<class T>
	inline T _max(const T& a,const T& b){
	    return (a>b)?a:b;
	}
	template<class T>
	inline T _avg(T a,T b,T c,T d){
	    T in0 = a+b+c+d;
	    in0 = (in0 == 0)?0: (in0/4);
	    return in0;
	}
	inline f32 randf32(){
	    return (float)rand()/(float)(RAND_MAX/1.0f);
	}
	inline f64 randf64(){
	    return (double) rand()/(float)(RAND_MAX/1.0f);
	}
}

#endif
