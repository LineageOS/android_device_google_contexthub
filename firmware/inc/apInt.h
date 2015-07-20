#ifndef _APINT_H_
#define _APINT_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void apIntInit();
void apIntSet(bool wakeup);
void apIntClear(bool wakeup);

#ifdef __cplusplus
}
#endif

#endif
