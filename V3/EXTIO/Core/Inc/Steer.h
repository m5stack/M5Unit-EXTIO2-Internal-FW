#ifndef __STEER_H
#define __STEER_H
#ifdef __cplusplus
 extern "C" {
#endif 


#define STEER_NUMBER 8

extern void Analog_PWM(void);
extern void SetSteerAngle(unsigned char pos, unsigned char angle);
extern void SetSteerPulse(unsigned char pos, unsigned short int pulse);
extern void SteerUpdate(void);
extern void SteerInit(void);
extern void SetPWMPulse(unsigned char pos, unsigned short int pulse);

#ifdef __cplusplus
}
#endif

#endif
