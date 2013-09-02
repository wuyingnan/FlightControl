#ifndef UCS_H_
#define UCS_H_
#define CPU_F ((double)16000000) 
#define delay_us(x) __delay_cycles((long)(CPU_F*(double)x/1000000.0)) 
#define delay_ms(x) __delay_cycles((long)(CPU_F*(double)x/1000.0)) 
extern void UCS_init(void);
extern void CLK_output(void);
#endif /* UCS_H_ */