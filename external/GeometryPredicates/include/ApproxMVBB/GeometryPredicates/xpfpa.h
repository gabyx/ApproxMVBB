#ifndef ApproxMVBB_GeometryPredicates_xpfpa_h
#define ApproxMVBB_GeometryPredicates_xpfpa_h

/* Cross Platform Floating Point Arithmetics

   This header file defines several platform-dependent macros that ensure
   equal and deterministic floating point behaviour across several platforms,
   compilers and architectures.

   The current macros are currently only used on x86 and x86_64 architectures,
   on every other architecture, these macros expand to NOPs. This assumes that
   other architectures do not have an internal precision and the operhand types
   define the computational precision of floating point operations. This
   assumption may be false, in that case, the author is interested in further
   details on the other platform.

   For further details, please visit:
   http://www.christian-seiler.de/projekte/fpmath/

   Author: Christian Seiler <webmaster@christian-seiler.de>
   Version: 20081026

   This file is released under public domain - or - in countries where this is
   not possible under the following license:

      Permission is hereby granted, free of charge, to any person obtaining a
      copy of this software, to deal in the software without restriction,
      including without limitation the rights to use, copy, modify, merge,
      publish, distribute, sublicense, and/or sell copies of the software,
      and to permit persons to whom the software is furnished to do so, subject
     to no condition whatsoever.

      This software is provided AS IS, without warranty of any kind, express or
      implied. */

#ifndef XPFPA_H
#    define XPFPA_H

/*
     Implementation notes:

     x86_64:
      - Since all x86_64 compilers use SSE by default, it is probably unecessary
        to use these macros there. We define them anyway since we are too lazy
        to differentiate the architecture. Also, the compiler option -mfpmath=i387
        justifies this decision.

     General:
      - It would be nice if one could detect whether SSE if used for math via some
        funky compiler defines and if so, make the macros go to NOPs. Any ideas
        on how to do that?

     MS Visual C:
      - Since MSVC users tipically don't use autoconf or CMake, we will detect
        MSVC via compile time define.
    */

// MSVC detection (MSVC people usually don't use autoconf)
#    ifdef _MSC_VER
#        if _MSC_VER >= 1500
// Visual C++ 2008 or higher, supports _controlfp_s
#            define HAVE__CONTROLFP_S
#        else
// Visual C++ (up to 2005), supports _controlfp
#            define HAVE__CONTROLFP
#        endif  // MSC_VER >= 1500 \
                // Tell MSVC optimizer that we access FP environment
#        pragma fenv_access(on)
#    endif  // _MSC_VER

// MSVC does NOT support precision control (_control_fp) stuff on x64 platforms!
// Define everything as NOP.
#    if defined(_MSC_VER) && defined(_WIN64)

#        define XPFPA_DECLARE()                /* NOP */
#        define XPFPA_SWITCH_DOUBLE()          /* NOP */
#        define XPFPA_SWITCH_SINGLE()          /* NOP */
#        define XPFPA_SWITCH_DOUBLE_EXTENDED() /* NOP */
#        define XPFPA_RESTORE()                /* NOP */
#        define XPFPA_RETURN_DOUBLE(val) return (val);
#        define XPFPA_RETURN_SINGLE(val) return (val);
#        define XPFPA_RETURN_DOUBLE_EXTENDED(val) return (val);

#    elif HAVE__CONTROLFP_S

// float.h defines _controlfp_s
#        include <float.h>

#        define XPFPA_DECLARE() static unsigned int _xpfpa_fpu_oldcw, _xpfpa_fpu_cw;

#        define XPFPA_SWITCH_DOUBLE()           \
            _controlfp_s(&_xpfpa_fpu_cw, 0, 0); \
            _xpfpa_fpu_oldcw = _xpfpa_fpu_cw;   \
            _controlfp_s(&_xpfpa_fpu_cw, _PC_53, _MCW_PC);
#        define XPFPA_SWITCH_SINGLE()           \
            _controlfp_s(&_xpfpa_fpu_cw, 0, 0); \
            _xpfpa_fpu_oldcw = _xpfpa_fpu_cw;   \
            _controlfp_s(&_xpfpa_fpu_cw, _PC_24, _MCW_PC);
// NOTE: This only sets internal precision. MSVC does NOT support double-
// extended precision!
#        define XPFPA_SWITCH_DOUBLE_EXTENDED()  \
            _controlfp_s(&_xpfpa_fpu_cw, 0, 0); \
            _xpfpa_fpu_oldcw = _xpfpa_fpu_cw;   \
            _controlfp_s(&_xpfpa_fpu_cw, _PC_64, _MCW_PC);
#        define XPFPA_RESTORE() _controlfp_s(&_xpfpa_fpu_cw, _xpfpa_fpu_oldcw, _MCW_PC);
// We do NOT use the volatile return trick since _controlfp_s is a function
// call and thus FP registers are saved in memory anyway. However, we do use
// a variable to ensure that the expression passed into val will be evaluated
// *before* switching back contexts.
#        define XPFPA_RETURN_DOUBLE(val)      \
            {                                 \
                double _xpfpa_result = (val); \
                XPFPA_RESTORE()               \
                return _xpfpa_result;         \
            }
#        define XPFPA_RETURN_SINGLE(val)     \
            {                                \
                float _xpfpa_result = (val); \
                XPFPA_RESTORE()              \
                return _xpfpa_result;        \
            }
// This won't work, but we add a macro for it anyway.
#        define XPFPA_RETURN_DOUBLE_EXTENDED(val)  \
            {                                      \
                long double _xpfpa_result = (val); \
                XPFPA_RESTORE()                    \
                return _xpfpa_result;              \
            }

#    elif defined(HAVE__CONTROLFP)

// float.h defines _controlfp
#        include <float.h>

#        define XPFPA_DECLARE() static unsigned int _xpfpa_fpu_oldcw;

#        define XPFPA_SWITCH_DOUBLE()            \
            _xpfpa_fpu_oldcw = _controlfp(0, 0); \
            _controlfp(_PC_53, _MCW_PC);
#        define XPFPA_SWITCH_SINGLE()            \
            _xpfpa_fpu_oldcw = _controlfp(0, 0); \
            _controlfp(_PC_24, _MCW_PC);
// NOTE: This will only work as expected on MinGW.
#        define XPFPA_SWITCH_DOUBLE_EXTENDED()   \
            _xpfpa_fpu_oldcw = _controlfp(0, 0); \
            _controlfp(_PC_64, _MCW_PC);
#        define XPFPA_RESTORE() _controlfp(_xpfpa_fpu_oldcw, _MCW_PC);
// We do NOT use the volatile return trick since _controlfp is a function
// call and thus FP registers are saved in memory anyway. However, we do use
// a variable to ensure that the expression passed into val will be evaluated
// *before* switching back contexts.
#        define XPFPA_RETURN_DOUBLE(val)      \
            {                                 \
                double _xpfpa_result = (val); \
                XPFPA_RESTORE()               \
                return _xpfpa_result;         \
            }
#        define XPFPA_RETURN_SINGLE(val)     \
            {                                \
                float _xpfpa_result = (val); \
                XPFPA_RESTORE()              \
                return _xpfpa_result;        \
            }
// This will only work on MinGW
#        define XPFPA_RETURN_DOUBLE_EXTENDED(val)  \
            {                                      \
                long double _xpfpa_result = (val); \
                XPFPA_RESTORE()                    \
                return _xpfpa_result;              \
            }

#    elif defined(HAVE__FPU_SETCW)  // glibc systems

// fpu_control.h defines _FPU_[GS]ETCW
#        include <fpu_control.h>

#        define XPFPA_DECLARE() static fpu_control_t _xpfpa_fpu_oldcw, _xpfpa_fpu_cw;

#        define XPFPA_SWITCH_DOUBLE()                                                         \
            _FPU_GETCW(_xpfpa_fpu_oldcw);                                                     \
            _xpfpa_fpu_cw = (_xpfpa_fpu_oldcw & ~_FPU_EXTENDED & ~_FPU_SINGLE) | _FPU_DOUBLE; \
            _FPU_SETCW(_xpfpa_fpu_cw);
#        define XPFPA_SWITCH_SINGLE()                                                         \
            _FPU_GETCW(_xpfpa_fpu_oldcw);                                                     \
            _xpfpa_fpu_cw = (_xpfpa_fpu_oldcw & ~_FPU_EXTENDED & ~_FPU_DOUBLE) | _FPU_SINGLE; \
            _FPU_SETCW(_xpfpa_fpu_cw);
#        define XPFPA_SWITCH_DOUBLE_EXTENDED()                                                \
            _FPU_GETCW(_xpfpa_fpu_oldcw);                                                     \
            _xpfpa_fpu_cw = (_xpfpa_fpu_oldcw & ~_FPU_SINGLE & ~_FPU_DOUBLE) | _FPU_EXTENDED; \
            _FPU_SETCW(_xpfpa_fpu_cw);
#        define XPFPA_RESTORE() _FPU_SETCW(_xpfpa_fpu_oldcw);
// We use a temporary volatile variable (in a new block) in order to ensure
// that the optimizer does not mis-optimize the instructions. Also, a volatile
// variable ensures truncation to correct precision.
#        define XPFPA_RETURN_DOUBLE(val)               \
            {                                          \
                volatile double _xpfpa_result = (val); \
                XPFPA_RESTORE()                        \
                return _xpfpa_result;                  \
            }
#        define XPFPA_RETURN_SINGLE(val)              \
            {                                         \
                volatile float _xpfpa_result = (val); \
                XPFPA_RESTORE()                       \
                return _xpfpa_result;                 \
            }
#        define XPFPA_RETURN_DOUBLE_EXTENDED(val)           \
            {                                               \
                volatile long double _xpfpa_result = (val); \
                XPFPA_RESTORE()                             \
                return _xpfpa_result;                       \
            }

#    elif defined(HAVE_FPSETPREC)  // FreeBSD

// fpu_control.h defines _FPU_[GS]ETCW
#        include <machine/ieeefp.h>

#        define XPFPA_DECLARE() static fp_prec_t _xpfpa_fpu_oldprec;

#        define XPFPA_SWITCH_DOUBLE()         \
            _xpfpa_fpu_oldprec = fpgetprec(); \
            fpsetprec(FP_PD);
#        define XPFPA_SWITCH_SINGLE()         \
            _xpfpa_fpu_oldprec = fpgetprec(); \
            fpsetprec(FP_PS);
#        define XPFPA_SWITCH_DOUBLE_EXTENDED() \
            _xpfpa_fpu_oldprec = fpgetprec();  \
            fpsetprec(FP_PE);
#        define XPFPA_RESTORE() fpsetprec(_xpfpa_fpu_oldprec);
// We use a temporary volatile variable (in a new block) in order to ensure
// that the optimizer does not mis-optimize the instructions. Also, a volatile
// variable ensures truncation to correct precision.
#        define XPFPA_RETURN_DOUBLE(val)               \
            {                                          \
                volatile double _xpfpa_result = (val); \
                XPFPA_RESTORE()                        \
                return _xpfpa_result;                  \
            }
#        define XPFPA_RETURN_SINGLE(val)              \
            {                                         \
                volatile float _xpfpa_result = (val); \
                XPFPA_RESTORE()                       \
                return _xpfpa_result;                 \
            }
#        define XPFPA_RETURN_DOUBLE_EXTENDED(val)           \
            {                                               \
                volatile long double _xpfpa_result = (val); \
                XPFPA_RESTORE()                             \
                return _xpfpa_result;                       \
            }

#    elif defined(HAVE_FPU_INLINE_ASM_X86)

/*
  Custom x86 inline assembler implementation.

  This implementation does not use predefined wrappers of the OS / compiler
  but rather uses x86/x87 inline assembler directly. Basic instructions:

  fnstcw - Store the FPU control word in a variable
  fldcw  - Load the FPU control word from a variable

  Bits (only bits 8 and 9 are relevant, bits 0 to 7 are for other things):
     0x0yy: Single precision
     0x1yy: Reserved
     0x2yy: Double precision
     0x3yy: Double-extended precision

  We use an unsigned int for the datatype. glibc sources add __mode__ (__HI__)
  attribute to it (HI stands for half-integer according to docs). It is unclear
  what the does exactly and how portable it is.

  The assembly syntax works with GNU CC, Intel CC and Sun CC.
*/

#        define XPFPA_DECLARE() static unsigned int _xpfpa_fpu_oldcw, _xpfpa_fpu_cw;

#        define XPFPA_SWITCH_DOUBLE()                            \
            __asm__ __volatile__("fnstcw %0"                     \
                                 : "=m"(*&_xpfpa_fpu_oldcw));    \
            _xpfpa_fpu_cw = (_xpfpa_fpu_oldcw & ~0x100) | 0x200; \
            __asm__ __volatile__("fldcw %0"                      \
                                 :                               \
                                 : "m"(*&_xpfpa_fpu_cw));
#        define XPFPA_SWITCH_SINGLE()                         \
            __asm__ __volatile__("fnstcw %0"                  \
                                 : "=m"(*&_xpfpa_fpu_oldcw)); \
            _xpfpa_fpu_cw = (_xpfpa_fpu_oldcw & ~0x300);      \
            __asm__ __volatile__("fldcw %0"                   \
                                 :                            \
                                 : "m"(*&_xpfpa_fpu_cw));
#        define XPFPA_SWITCH_DOUBLE_EXTENDED()                \
            __asm__ __volatile__("fnstcw %0"                  \
                                 : "=m"(*&_xpfpa_fpu_oldcw)); \
            _xpfpa_fpu_cw = _xpfpa_fpu_oldcw | 0x300;         \
            __asm__ __volatile__("fldcw %0"                   \
                                 :                            \
                                 : "m"(*&_xpfpa_fpu_cw));
#        define XPFPA_RESTORE() __asm__ __volatile__("fldcw %0" \
                                                     :          \
                                                     : "m"(*&_xpfpa_fpu_oldcw));
// We use a temporary volatile variable (in a new block) in order to ensure
// that the optimizer does not mis-optimize the instructions. Also, a volatile
// variable ensures truncation to correct precision.
#        define XPFPA_RETURN_DOUBLE(val)               \
            {                                          \
                volatile double _xpfpa_result = (val); \
                XPFPA_RESTORE()                        \
                return _xpfpa_result;                  \
            }
#        define XPFPA_RETURN_SINGLE(val)              \
            {                                         \
                volatile float _xpfpa_result = (val); \
                XPFPA_RESTORE()                       \
                return _xpfpa_result;                 \
            }
#        define XPFPA_RETURN_DOUBLE_EXTENDED(val)           \
            {                                               \
                volatile long double _xpfpa_result = (val); \
                XPFPA_RESTORE()                             \
                return _xpfpa_result;                       \
            }

#    else                                      // FPU CONTROL

/*
          This is either not an x87 FPU or the inline assembly syntax was not
          recognized. In any case, default to NOPs for the macros and hope the
          generated code will behave as planned.
        */
#        define XPFPA_DECLARE()                /* NOP */
#        define XPFPA_SWITCH_DOUBLE()          /* NOP */
#        define XPFPA_SWITCH_SINGLE()          /* NOP */
#        define XPFPA_SWITCH_DOUBLE_EXTENDED() /* NOP */
#        define XPFPA_RESTORE()                /* NOP */
#        define XPFPA_RETURN_DOUBLE(val) return (val);
#        define XPFPA_RETURN_SINGLE(val) return (val);
#        define XPFPA_RETURN_DOUBLE_EXTENDED(val) return (val);

#    endif  // FPU CONTROL

#endif  // XPFPA_H

#endif  // header guard
