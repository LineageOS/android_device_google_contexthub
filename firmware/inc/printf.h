#ifndef _PRINTF_H_
#define _PRINTF_H_

#ifdef __cplusplus
}
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>

typedef bool (*printf_write_c)(void* userData, char c);		//callback can return false anytime to abort  printing immediately

uint32_t cvprintf(printf_write_c writeF, void* writeD, const char* fmtStr, va_list vl);

#ifdef __cplusplus
}
#endif

#endif

