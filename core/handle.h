#ifndef HANDLE_H
#define HANDLE_H

#include "types.h"
#include "buffer.h"

class handle {
public:
    handle(handleType ht=HT_SERIAL)
    {
        this->descriptor = 0;
        this->blocking = true;
        this->type = ht;
        this->pathname = 0;
    }
    virtual ~handle(){
        if(this->pathname)
            free(this->pathname);

        this->close();
    }

    virtual handle& operator=(const handle& h);

    virtual int setBlocking(bool block);

    virtual int address(u8 addr);

    virtual int open(const char *px,u32 flags=0);
    virtual void close();

    virtual int read(void *buf,int len);
    virtual int write(const void *buf,int len);
    virtual int transfer(void *in,const void *out,int len);
    virtual int listen(int port);
    virtual handle accept(struct sockaddr* addr,int *size);
    virtual int bind(const char *localAddress="127.0.0.1");
    virtual int connect(const char *remoteAddress,int port);
    virtual buffer<u8>* readin();

    virtual int available();
    virtual s64 size();

    virtual s64 preallocate(s64 s);

    //! For INotify
    virtual handle add_watch(u32 flags = (IN_CREATE,IN_DELETE,IN_MODIFY));
    virtual int rem_watch(const handle& h);

    char *pathname;
    bool blocking;

    handleType type;
    int descriptor;
};
inline buffer<u8>* handle::readin(){
    u32 fs = (u32) this->size();

    buffer<u8>* pbuff = new buffer<u8>(fs+1);

    int bytesRead = 0;

    do {
        int bytesIn = this->read(&pbuff->values[bytesRead],fs-bytesRead);

        bytesRead += bytesIn;
    } while(bytesRead != fs);

    return pbuff;
}
inline int handle::listen(int port){
    return ::listen(this->descriptor,port);
}
inline handle handle::accept(struct sockaddr* addr,int *size){
    handle hsock(HT_SOCKET);
    hsock.descriptor = ::accept(this->descriptor,addr,(socklen_t*)size);
    return hsock;
}
inline int handle::bind(const char *localAddress){
    struct sockaddr_in serverAddress;
    memset(&serverAddress,0,sizeof(serverAddress));

    serverAddress.sin_family = AF_INET;
    serverAddress.sin_addr.s_addr = inet_addr(localAddress);
    serverAddress.sin_port = htons(80);

    return ::bind(this->descriptor,(struct sockaddr*)&serverAddress,sizeof(serverAddress));
}
inline int handle::connect(const char *remoteAddress,int port){
    struct addrinfo hints, *it, *result;
    int retval = 0;

    memset (&hints, 0, sizeof (hints));
    hints.ai_family = PF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags |= AI_CANONNAME;

    retval = getaddrinfo (remoteAddress, "443", &hints, &result);
    if (retval != 0){
        printf("Could not get address info\r\n");
        return -1;
    }

    char buff[32] = {0,};

    for (it = result; it != NULL; it = it->ai_next) {
        int tempfd = socket(it->ai_family, it->ai_socktype, it->ai_protocol);
        if(tempfd <0)
            continue;


        if (::connect(tempfd, it->ai_addr, it->ai_addrlen)>=0) {
            this->descriptor = tempfd;
            freeaddrinfo(result);

            printf("Connected to host %s\r\n",remoteAddress);
            break;
        }
        ::close(tempfd);
    }
    printf("Finished connecting\r\n");

    //! Sweet!

    return 0;
}
inline handle& handle::operator=(const handle& h){
    this->pathname = strdup(h.pathname);
    this->blocking = h.blocking;
    this->type = h.type;
    this->descriptor = h.descriptor;
    return *this;
}
inline int handle::setBlocking(bool block){
    int flags = fcntl(this->descriptor, F_GETFL, 0);
    if (flags == -1) return -1;

    flags = block ? (flags & ~O_NONBLOCK) : (flags | O_NONBLOCK);
    if( flags == -1) return -1;

    if(fcntl(this->descriptor, F_SETFL, flags) == -1){
        return -1;
    }
    return this->blocking = block;
}
inline int handle::address(u8 addr){
    return ioctl(this->descriptor,I2C_SLAVE,addr);
}
inline int handle::open(const char *px,u32 flags){
    if(px)
        this->pathname = strdup(px);

    //! Open the specified descriptor
    switch(this->type){
        case HT_ASYNC: {
            this->descriptor = epoll_create(MAX_EPOLL_EVENTS);
        } break;
        case HT_SERIAL: {
            this->descriptor = ::open(px,O_RDWR | O_NOCTTY | O_NONBLOCK);
        } break;
        case HT_INOTIFY: {
            //! Create INotify instance
            this->descriptor = inotify_init();
        } break;
        case HT_SPI: {
            this->descriptor = ::open(px,O_RDWR);
        } break;
        case HT_WATCH: {
            return 0;
        } break;
        case HT_SOCKET: {
            this->descriptor = socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
        } break;
        default: {
            this->descriptor = ::open(px,O_RDWR | O_CREAT,S_IRWXO);
        } break;
    }

    //! Check for errors
    if(this->descriptor<0) return -1;

    //! Set the required modes on the descriptor
    switch(this->type){
        case HT_SERIAL: {
            //! Open as a serial port
            struct termios tty;

            if(tcgetattr(this->descriptor, &tty) != 0) {
                this->close();
                return -1;
            }

            cfsetispeed(&tty, 2000000);
            cfsetospeed(&tty, 2000000);

            tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
            tty.c_cflag &= ~CSIZE;
            tty.c_cflag |= CS8;         /* 8-bit characters */
            tty.c_cflag &= ~PARENB;     /* no parity bit */
            tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
            tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

            // Setup for non-canonical mode
            tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
            tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
            tty.c_oflag &= ~OPOST;

            // Fetch bytes as they become available
            tty.c_cc[VTIME] = 0;
            tty.c_cc[VMIN] = 1;

            // Save tty settings, also checking for error
            if (tcsetattr(this->descriptor, TCSANOW, &tty) != 0) {
                this->close();
                return -1;
            }
        } break;
        case HT_SPI: {
            u32 spiMode = (flags & SPI_HANDLE_MODE)>>24;
            u32 spiSpeed = (flags & SPI_HANDLE_SPEED)>>16;

            if(ioctl(this->descriptor,SPI_IOC_WR_MODE,&spiMode)!=0){
                //! Could not ioctl
                return -1;
            }

            int spiSpeedHz = 0;
            switch(spiSpeed){
                case 0: {
                    //! 1.125 Mhz
                    spiSpeedHz = 1125000;
                } break;
                case 1: {
                    //! 2.25 Mhz
                    spiSpeedHz = 2250000;
                } break;
                case 2: {
                    //! 4.5 Mhz
                    spiSpeedHz = 4500000;
                } break;
                case 3: {
                    //! 9 Mhz
                    spiSpeedHz = 9000000;
                } break;
                case 4: {
                    //! 18 Mhz
                    spiSpeedHz = 18000000;
                } break;
                case 5: {
                    spiSpeedHz = 36000000;
                } break;
            }

            if(ioctl(this->descriptor,SPI_IOC_WR_MAX_SPEED_HZ,&spiSpeedHz)!=0){
                //! Could not ioctl
                return -1;
            }
            if(ioctl(this->descriptor,SPI_IOC_RD_MAX_SPEED_HZ,&spiSpeedHz)!=0){
                //! Could not ioctl
                return -1;
            }
            int bits = 8;
            if(ioctl(this->descriptor,SPI_IOC_RD_BITS_PER_WORD,&bits) !=0){
                //! Could not ioctl
                return -1;
            }
        } break;
    }

    return 0;
}
inline void handle::close(){
    ::close(this->descriptor);
}
inline int handle::read(void *buf,int len){
    if(this->type == HT_SOCKET){
        return ::recv(this->descriptor,buf,len,0);
    }
    return ::read(this->descriptor,buf,len);
}
inline int handle::write(const void *buf,int len){
    if(this->type==HT_SOCKET){
        return ::send(this->descriptor,buf,len,0);
    }
    return ::write(this->descriptor,buf,len);    
}
inline int handle::transfer(void *in,const void *out,int len){
    //! This function is only valid for SPI devices
    if(this->type != HT_SPI) return -1;

    struct spi_ioc_transfer trx;
    memset(&trx,0,sizeof(trx));

    trx.tx_buf =(u64) out;
    trx.rx_buf =(u64) in;
    trx.len = len;
    
    trx.bits_per_word = 8;
    trx.delay_usecs =0;
    trx.speed_hz = 20000000;

    int num_tx = 1;
    if(ioctl(this->descriptor,SPI_IOC_MESSAGE(num_tx),&trx) !=0){
        //! Could not transfer
        return -1;
    }
    return len;
}
inline int handle::available(){
    int retval = 0;
    switch(this->type){
        default: {
            int result = ioctl(this->descriptor,FIONREAD,&retval);
        } break;
    }
    return retval;
}
inline s64 handle::size(){
    struct stat st;
    fstat(this->descriptor, &st);
    s64 s = st.st_size;
    return s;
}
inline s64 handle::preallocate(s64 s){
    return fallocate64(this->descriptor,FALLOC_FL_ZERO_RANGE,0,s);
}
inline handle handle::add_watch(u32 flags){
    handle hp(HT_WATCH);

    hp.descriptor = inotify_add_watch(this->descriptor,this->pathname,flags);
    return hp;
}
inline int handle::rem_watch(const handle& h){
    if(h.type != HT_WATCH) return -1;
    return inotify_rm_watch(this->descriptor,h.descriptor);
}


#endif