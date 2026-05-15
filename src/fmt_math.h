#ifndef __FMT_MATH_H
#define __FMT_MATH_H

#if __arm__
#include "arm_math.h"
#define fmt_sin_f32(x) arm_sin_f32((x))
#define fmt_cos_f32(x) arm_cos_f32((x))

#elif __riscv
#include "riscv_math.h"
#define fmt_sin_f32(x) riscv_sin_f32((x))
#define fmt_cos_f32(x) riscv_cos_f32((x))

#else
#error Unsupported target
#endif

#endif // __FMT_MATH_H