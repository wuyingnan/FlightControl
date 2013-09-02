/* --COPYRIGHT--,TI 
*
* IMPORTANT - PLEASE READ THE FOLLOWING LICENSE AGREEMENT CAREFULLY, WHICH 
* IS A LEGALLY BINDING AGREEMENT. AFTER YOU READ IT, YOU WILL BE ASKED WHETHER
* YOU ACCEPT AND AGREE TO ITS TERMS. DO NOT CLICK "I HAVE READ AND AGREE" 
* UNLESS: (1) YOU WILL USE THE LICENSED MATERIALS FOR YOUR OWN BENEFIT AND 
* PERSONALLY ACCEPT, AGREE TO, AND INTEND TO BE BOUND BY THESE TERMS; OR (2)
* YOU ARE AUTHORIZED TO ACCEPT AND AGREE TO, AND INTEND TO BE BOUND BY, THESE
* ON BEHALF OF YOUR COMPANY. 
*
* For the full license refer to {installdir}/MSPMATHLIB_1_00_00_00_Manifest.html
*/
#ifndef _MSP430MATHH_
#define _MSP430MATHH_

#include "math.h"

/* Mathlib function prototypes */
extern float msp430_sinf(float input);
extern float msp430_cosf(float input);
extern float msp430_tanf(float input);
extern float msp430_asinf(float input);
extern float msp430_acosf(float input);
extern float msp430_atanf(float input);
extern float msp430_atan2f(float input1, float input2);
extern float msp430_logf(float input);
extern float msp430_expf(float input);
extern float msp430_sqrtf(float input);
extern float msp430_fmodf(float input1, float input2);
extern float msp430_reciprocalf(float input);

/* 
 *  Define MSP430MATHUNMAP in the compiler options if you do not want to map
 *  the Mathlib functions to the existing function names.
 */
#ifndef MSP430MATHUNMAP

#undef sinf
#define sinf(x) msp430_sinf(x)
#undef cosf
#define cosf(x) msp430_cosf(x)
#undef tanf
#define tanf(x) msp430_tanf(x)
#undef asinf
#define asinf(x) msp430_asinf(x)
#undef acosf
#define acosf(x) msp430_acosf(x)
#undef atanf
#define atanf(x) msp430_atanf(x)
#undef atan2f
#define atan2f(x,y) msp430_atan2f(x,y)
#undef logf
#define logf(x) msp430_logf(x)
#undef expf
#define expf(x) msp430_expf(x)
#undef sqrtf
#define sqrtf(x) msp430_sqrtf(x)
#undef fmodf
#define fmodf(x,y) msp430_fmodf(x,y)
#define reciprocalf(x) msp430_reciprocalf(x)

#endif

#endif
