#ifndef SHMTIMEPERIODICEXECUTIONCONTEXT_H
#define SHMTIMEPERIODICEXECUTIONCONTEXT_H

#include <rtm/RTC.h>
#include <rtm/Manager.h>

#include <rtm/PeriodicExecutionContext.h>


class ShmTimePeriodicExecutionContext : public virtual RTC::PeriodicExecutionContext
{
public:
    ShmTimePeriodicExecutionContext();
    virtual ~ShmTimePeriodicExecutionContext(void);
    virtual int svc(void);
};

extern "C"
{
  void ShmTimePeriodicExecutionContextInit(RTC::Manager* manager);
};

#endif
