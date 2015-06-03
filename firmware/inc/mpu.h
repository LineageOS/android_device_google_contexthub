#ifndef _MPU_H_
#define _MPU_H_

#ifdef __cplusplus
extern "C" {
#endif


/*
 * MPU is very specific to each platform, so here really we just
 * leave it to the platform code to do the right thing for all the
 * requisite areas and such. Thhe simplest valid MPU implementation
 * will do nothing. Clearly we should strive to do more than that
 */

void mpuStart(void);
void mpuAllowRamExecution(bool allowSvcExecute);         /* for Supervisor only, if possible */
void mpuAllowRomWrite(bool allowSvcWrite);     /* for Supervisor only, if possible */


#ifdef __cplusplus
}
#endif

#endif

