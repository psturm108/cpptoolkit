#ifndef PROFILE_H
#define PROFILE_H

#include "types.h"

timespec diff(timespec start, timespec end)
{
    timespec temp;
    if ((end.tv_nsec-start.tv_nsec)<0) {
        temp.tv_sec = end.tv_sec-start.tv_sec-1;
        temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
    } else {
        temp.tv_sec = end.tv_sec-start.tv_sec;
        temp.tv_nsec = end.tv_nsec-start.tv_nsec;
    }
    return temp;
}

static long perf_event_open(struct perf_event_attr *hw_event, pid_t pid,
                int cpu, int group_fd, unsigned long flags)
{
    int ret;

    ret = syscall(__NR_perf_event_open, hw_event, pid, cpu,
                    group_fd, flags);
    return ret;
}

class profile {
public:
    profile(const std::string& n):
    name(n)
    {}
    virtual ~profile(){
        close(this->descriptor);
    }

    virtual int begin(){
        //! Initialise perf_event structure
        memset(&pe, 0, sizeof(struct perf_event_attr));
        //! Monitoring hardware performance
        pe.type = PERF_TYPE_HARDWARE;
        //! For internal system
        pe.size = sizeof(struct perf_event_attr);
        //! We are counting the amount of instructions
        pe.config = PERF_COUNT_HW_INSTRUCTIONS;
        //! Don't start counting straight away
        pe.disabled = 1;
        //! Don't count kernel events
        pe.exclude_kernel = 1;
        //! Don't count hypervisor events.
        pe.exclude_hv = 1;

        this->descriptor = perf_event_open(&pe,0,-1,-1,0);
        if(this->descriptor <0){
            printf("Could not open performance monitor: %i (%s)\r\n",errno,strerror(errno));
            return -1;
        }
        //! Reset the instruction counter
        if(ioctl(this->descriptor, PERF_EVENT_IOC_RESET, 0)!=0){
            printf("Could not reset performance monitor: %i (%s)\r\n",errno,strerror(errno));
            return -1;
        }
        //! Start the instruction counter
        if(ioctl(this->descriptor, PERF_EVENT_IOC_ENABLE, 0)!=0){
            printf("Could not enable performance monitor: %i (%s)\r\n",errno,strerror(errno));
            return -1;
        }
        return 0;
    }
    virtual int enable(){
        return ioctl(this->descriptor, PERF_EVENT_IOC_ENABLE, 0);
    }
    virtual int disable(){
        return ioctl(this->descriptor, PERF_EVENT_IOC_DISABLE, 0);
    }
    virtual int end(){
        ioctl(this->descriptor, PERF_EVENT_IOC_DISABLE, 0);
        int retval = read(this->descriptor, &this->counter, sizeof(u64));
        return retval;
    }
    virtual int print(){
        printf("Task %s took %llu instructions\r\n",this->name.c_str(),this->counter);
        return 0;
    }

    std::string name;
    int descriptor;

    u64 counter;
    struct perf_event_attr pe;

    profile *last;
    profile *next;
};

#endif