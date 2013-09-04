#ifndef _FC_H_
#define _FC_H_

extern void FC_init();
extern void FC_control();
extern void FC_emergencyStop();
extern void FC_changePitchPID(signed long kp,signed long ki,signed long kd);

#endif