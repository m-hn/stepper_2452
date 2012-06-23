/*
 * Drawing board looks like this-
 * (0,MAX_Y)                                (MAX_X,MAX_Y)
 * Motor +-----------------------------+ Motor
 * M1    |                             | M2
 *       |                             |
 *       |                             |
 *       |                             |
 *       |                             |
 *       +-----------------------------+
 * (0,0)                                (MAX_X,0)
 *
 * Motor config-
 * M1 enabled on P1.0 and M2 on P2.0
 * P1.1-P1.3 controls M1
 * P2.1-P2.3 controls M2
 */

#include <msp430g2452.h>

unsigned const char one_phase_seq[] = {BIT1, BIT2, BIT3, BIT4, BIT1, BIT2, BIT3, BIT4};	// Bits are lt. shifted 1 since first bit is enable bit which is set separately
unsigned const char two_phase_seq[] = {BIT1|BIT2, BIT2|BIT3, BIT3|BIT4, BIT4|BIT1, BIT1|BIT2, BIT2|BIT3, BIT3|BIT4, BIT4|BIT1};
unsigned const char interleave_seq[] = {BIT1, BIT1|BIT2, BIT2, BIT2|BIT3, BIT3, BIT3|BIT4, BIT4, BIT4|BIT1};

#define NULL				((void *)0)
#define MAX_SEQ_INDX		(7)		// size of _seq[] array -1
#define CW 					(1)
#define CCW 				(-1)
#define STEPS_PER_TURN		(48)
#define	MAX_RPM				(600)
#define STEP_INTERVAL		(2)	// 2 ms. 600*48 steps/ min; 480 steps/ sec for an interval between steps of ~2ms

#define MAX_X				(unsigned long)(215900)		// Standard letter size, in micro meters
#define MAX_Y				(unsigned long)(279400)
#define MAX_LEN				(unsigned long)(353096)		// Length of diagonal, pre-calculated
#define REEL_DIA			(unsigned long)(10000)		// Diameter of the spool that each motor controls
#define LEN_REELED_PER_REV	(unsigned long)(31415)		// pi * d
#define LEN_REELED_PER_STEP	(unsigned long)(654)		// (pi * d)/48

#define MAX_TIMER_INDX		(1)
#define TIMER_ONE_SHOT		(1)

/*
#define MAX_MEM_POOLS		4
#define MEM_POOL_SIZE		16		// bytes
*/


typedef enum{
	PORT_P1 = 1,
	PORT_P2
}OP_PORT;

typedef struct{
	int				curr_seq; 	// The current bit pattern to drive the motor
	unsigned char	*mode;		// one phase, two phase or interleaved
	OP_PORT			port;		// Output port that the motor is wired to
	unsigned long 	len;		// The length reeled in or out so far
} MOTOR_DESC;

typedef enum{
	REEL_IN 	=  1,
	REEL_OUT 	= -1
} REEL_DIR;

/*
typedef enum{
	NOT_IN_USE = 0,
	IN_USE
}MEM_STATUS;

typedef struct{
	MEM_STATUS		is_in_use;
	unsigned char	pool[MEM_POOL_SIZE];	// small pool for now
} MEM_POOL;
*/

typedef	void (*TIMER_CBK)(void *);

typedef struct{
	TIMER_CBK		p_fn;			// Function the timer will call back
	void 			*args;			// Arguments to pass back. Remembered context for calling function. Calling fn manages memory
	unsigned int	duration;		// requested duration, in multiples of 1ms
	int				curr_dur;		// For use by timer- Count down to 0 from requested duration
	int				type;			// TIMER_ONE_SHOT or # times the timer must fire
}TIMER_CONTEXT;


// just to test timer cbk works
inline void test_timer( void *arg);

int register_timer(
		TIMER_CBK 			p_cbk,	// time call back fn
		void 				*p_args,// Args to be passed back to the call back fn
		int 				dur,	// duration in multiples of 1ms
		int					type);	// # times to fire

inline void one_step_in(
		void 				*motor_desc);

inline void one_step_out(
		void 				*motor_desc);

inline void stop_motor(
		MOTOR_DESC			*m);

inline void reel_in_out(
		MOTOR_DESC 			*m,		// Which motor to act on
		unsigned long 		len, 	// Length to reel in/ out
		REEL_DIR 			dir);	// in or out

void goto_xy(
		MOTOR_DESC 			*m1,	// X motor
		MOTOR_DESC 			*m2, 	// Y motor
		unsigned long 		x, 		// x & Y coordinates
		unsigned long 		y);

unsigned char *malloc();

int free(unsigned char *m);

// Global variables
MOTOR_DESC	m1, m2;
// MEM_POOL 	g_mem_pool[4];

TIMER_CONTEXT timer_context[MAX_TIMER_INDX+1] = {{NULL, NULL, 0, 0, TIMER_ONE_SHOT}, {NULL, NULL, 0, 0, TIMER_ONE_SHOT}};	// Service max 2 requests for now- one for each motor

void main(void)
{
	WDTCTL = WDTPW + WDTHOLD;	// Stop WDT

	P1OUT = P2OUT = 0x0;	// All pulled low to start
	P1DIR = P2DIR = 0xff;	// Initialize P1, P2 to all outputs

	// Initialize motor descriptors- Assumes start position is (0, MAX_Y)
	m1.curr_seq = 0;
	m1.mode = (unsigned char *)one_phase_seq;
	m1.port = PORT_P1;
	m1.len = 0;

	m2.curr_seq = 0;
	m2.mode = (unsigned char *)one_phase_seq;
	m2.port = PORT_P2;
	m2.len = MAX_Y;

	P2OUT = BIT0+BIT1+BIT2;

	// register_timer(test_timer, NULL, 1, 5);
	goto_xy( &m1, &m2, 100000, 130000);

	// Setup Timer A - Last step
	TACCTL0 = CCIE;        			// CCR0 interrupt enabled
	TACCR0 = 32768;
	TACTL = TASSEL_2 + MC_2;        // SMCLK, contmode
	_BIS_SR(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
}

inline void one_step_in(void *p_v)
{
	MOTOR_DESC	*p_m = p_v;

	if(p_m->len <= MAX_LEN)		// Not already reeled out fully
	{
		switch(p_m->port) {
			case PORT_P1: P1OUT = BIT0 + p_m->mode[p_m->curr_seq]; break;
			case PORT_P2: P2OUT = BIT0 + p_m->mode[p_m->curr_seq]; break;
			default: break;
		}
		p_m->curr_seq--;
		if( p_m->curr_seq < 0 ) p_m->curr_seq = MAX_SEQ_INDX;
		p_m->len -= LEN_REELED_PER_STEP;
	}
	if(p_m->len > MAX_LEN) // Since this is unsigned, it will wrap around -ve values
		p_m->len= MAX_LEN;
}

inline void one_step_out(void *p_v)
{
	MOTOR_DESC	*p_m = p_v;

	if(p_m->len <= MAX_LEN)		// Not already reeled out fully
	{
		switch(p_m->port) {
			case PORT_P1: P1OUT = BIT0 + p_m->mode[p_m->curr_seq]; break;
			case PORT_P2: P2OUT = BIT0 + p_m->mode[p_m->curr_seq]; break;
			default: break;
		}
		p_m->curr_seq++;
		if( p_m->curr_seq > MAX_SEQ_INDX ) p_m->curr_seq = 0;
		p_m->len += LEN_REELED_PER_STEP;
	}
	if(p_m->len > MAX_LEN)
		p_m->len= MAX_LEN;
}

inline void stop_motor(MOTOR_DESC *m)
{
	switch(m->port) {
		case PORT_P1: P1OUT = 0x0; break;
		case PORT_P2: P2OUT = 0x0; break;
		default: break;
	}
}

void goto_xy(MOTOR_DESC *m1, MOTOR_DESC *m2, unsigned long x, unsigned long y)
{
	if(m1->len > x) reel_in_out(m1, (m1->len)-x, REEL_IN);
	else if(m1->len < x) reel_in_out(m1, x-(m1->len), REEL_OUT);

	if(m2->len > y) reel_in_out(m2, (m2->len)-y, REEL_IN);
	else if(m2->len < y) reel_in_out(m2, y-(m2->len), REEL_OUT);
}

inline void reel_in_out(MOTOR_DESC *m, unsigned long len, REEL_DIR dir)
{
	int			num_steps;
	num_steps = (int)(len/LEN_REELED_PER_STEP);

	switch(dir)
	{
		case REEL_IN: register_timer(one_step_in, m, STEP_INTERVAL, num_steps); break;
		case REEL_OUT: register_timer(one_step_out, m, STEP_INTERVAL, num_steps); break;
		default: break;
	}
}

// Timer A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
{
	int 	i;
	TIMER_CONTEXT	*p_t;

	for(i = MAX_TIMER_INDX; i >=0; i--)
	{
		p_t = &timer_context[i];

		if(NULL != p_t->p_fn)				// There is a registered timer
		{
			if(--(p_t->curr_dur) <= 0)		// It's time is up
			{
				p_t->p_fn(p_t->args);		// Invoke the call back
				if(TIMER_ONE_SHOT == p_t->type) // Deregister the timer. Done with this timer
				{
					p_t->p_fn = NULL;
					p_t->args = NULL;
					p_t->duration = 0;
				}
				else
				{
					p_t->curr_dur = p_t->duration;
					p_t->type--;
				}
			}
		}
	}

	TACCR0 += 32768;
}

int register_timer(TIMER_CBK p_cbk, void *p_args, int dur, int type)
{
	int 			i = 0;

	for(i=MAX_TIMER_INDX; i>=0; i--)
	{
		if(NULL == timer_context[i].p_fn) // Found a free slot
		{
			timer_context[i].p_fn = p_cbk;
			timer_context[i].args = p_args;
			timer_context[i].curr_dur = timer_context[i].duration = dur;
			timer_context[i].type = type;
			break;
		}
	}

	return i;
}

inline void test_timer(void *args)
{
	P1OUT ^= BIT0;
	P2OUT ^= BIT0;
	CCR0 += 50000;
}

/*
unsigned char *malloc( ) // int num_bytes - fixed size for now
{
	int i =0;

	while(IN_USE == g_mem_pool[i].is_in_use)
	{
		if(i > MAX_MEM_POOLS)
			break;
	}i++;
	if(i < MAX_MEM_POOLS)	// Found a free mem pool
	{
		i--;
		g_mem_pool[i].is_in_use = IN_USE;
		return g_mem_pool[i].pool;
	}
	else
		return NULL;
}

int free(unsigned char *m)
{
	int i = 0;

	while( (m != g_mem_pool[i].pool) && (i < MAX_MEM_POOLS) ) i++;
	if( i < MAX_MEM_POOLS)
	{
		g_mem_pool[i].is_in_use = NOT_IN_USE;
	}
	else
		i = -1;

	return i;
}
*/
