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

/*
 * Assuming letter sheet sized canvas of 21.59cm x 27.94cm
 * Assuming a spool diameter of 10mm
 * For each step, the plot point will move pi * d/ 48 = 6.45mm
 * # steps required to cover the 21.59cm in the X direction = 21.59/.654 = 330
 * # steps required to cover the 27.94cm in the Y direction = 27.94/.654 = 427
 * #steps in the diagonal = sqrt(x^2+y^2)/ 654 = 540
 */
#define MAX_X_STEPS			(330)
#define MAX_Y_STEPS			(427)
#define MAX_STEPS			(540)

#define DELAY_1MS			(0x3fff)
/*
#define MAX_TIMERS			(25)	// May be as many events pending as # of steps
#define TIMER_ONE_SHOT		(1)
*/
#define TIMER_COUNTER_MAX	(11000)	// SMCLK is 1100KHZ. this count gives a 10ms time

/*
#define MAX_MEM_POOLS		4
#define MEM_POOL_SIZE		16		// bytes
*/

typedef enum{
	PORT_P1 = 1,
	PORT_P2
}OP_PORT;

typedef enum{
	REEL_IN = -1,
	REEL_OUT = 1
} REEL_DIR;

typedef struct{
	int				curr_seq; 	// The current bit pattern to drive the motor
	unsigned char	*mode;		// one phase, two phase or interleaved
	OP_PORT			port;		// Output port that the motor is wired to
	REEL_DIR		d;			// Current reel in/ out direction
	unsigned int	curr_step_pos;		// Current X/Y position, in # of steps from origin
} MOTOR_DESC;

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

/*
typedef	void (*TIMER_CBK)(void *);

typedef struct{
	TIMER_CBK		p_fn;			// Function the timer will call back
	void 			*args;			// Arguments to pass back. Remembered context for calling function. Calling fn manages memory
}TIMER_CONTEXT;


// just to test timer cbk works
inline void test_timer( void *arg);

int register_timer(
		TIMER_CBK 			p_cbk,	// time call back fn
		void 				*p_args);// Args to be passed back to the call back fn
*/
void one_step(
		void				*motor_desc);	// Callback fn from timer to step motor 1

void stop_motor(
		MOTOR_DESC			*m);

void lineto_xy(
		MOTOR_DESC 			*m1,	// X motor
		MOTOR_DESC 			*m2, 	// Y motor
		unsigned int 		x, 		// Where to go- Absolute x & Y coordinates in #steps
		unsigned int		y);

void my_delay(
		int					delay);	// Delay in multiples of 10ms

/*unsigned char *malloc();

int free(unsigned char *m);
*/

// Global variables
MOTOR_DESC	m1, m2;
// MEM_POOL 	g_mem_pool[4];
//debug
int g_d_onestep, g_d_p1, g_d_p2, g_timer_full_err;
volatile unsigned int g_timer_fired;
/*
TIMER_CONTEXT timer_context[MAX_TIMERS];
int g_timer_head, g_timer_tail;				// Head and tail of timer Q
int	g_num_timers;
*/
void main(void)
{
	WDTCTL = WDTPW + WDTHOLD;	// Stop WDT

	P1OUT = P2OUT = 0x0;	// All pulled low to start
	P1DIR = P2DIR = 0xff;	// Initialize P1, P2 to all outputs

	// Initialize motor descriptors- Assumes start position is (0, MAX_Y)
	m1.curr_seq = 0;
	m1.mode = (unsigned char *)one_phase_seq;
	m1.port = PORT_P1;
	m1.curr_step_pos = 0;
	m1.d = REEL_OUT;

	m2.curr_seq = 0;
	m2.mode = (unsigned char *)one_phase_seq;
	m2.port = PORT_P2;
	m2.curr_step_pos = 0;
	m2.d = REEL_OUT;


	// Initialize timer variables
	g_timer_fired = 0;
	/*
	g_timer_head = g_num_timers = g_timer_tail = 0;
	*/
	// debug
	g_d_onestep =g_d_p1 = g_d_p2 = 0;
	//g_timer_full_err = 0;

	// register_timer(test_timer, NULL, 1, 5);

	// Setup Timer A - Last step
	TACCTL0 = CCIE;        			// CCR0 interrupt enabled
	TACCR0 = TIMER_COUNTER_MAX;
	TACTL = TASSEL_2 + MC_1;        // SMCLK, contmode
	//_BIS_SR(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
	__enable_interrupt();			// Enable interrupts globally

	lineto_xy( &m1, &m2, 20, 100);
	my_delay(1000);
	lineto_xy( &m1, &m2, 0, 10);
}

void one_step(void *p_v)
{
	MOTOR_DESC	*p_m;

	p_m = (MOTOR_DESC *)p_v;
	g_d_onestep ++;

	if(p_m->curr_step_pos < MAX_STEPS)		// Not already reeled out fully
	{
		switch(p_m->port) {
			case PORT_P1: P1OUT = BIT0 + p_m->mode[p_m->curr_seq]; g_d_p1++; break;
			case PORT_P2: P2OUT = BIT0 + p_m->mode[p_m->curr_seq]; g_d_p2++; break;
			default: break;
		}
		p_m->curr_seq += p_m->d;	// Evaluates to ++ or --

		if( p_m->curr_seq < 0 ) p_m->curr_seq = MAX_SEQ_INDX;
		else if( p_m->curr_seq > MAX_SEQ_INDX ) p_m->curr_seq = 0;
	}
	my_delay(10);
}

void stop_motor(MOTOR_DESC *m)
{
	switch(m->port) {
		case PORT_P1: P1OUT = 0x0; break;
		case PORT_P2: P2OUT = 0x0; break;
		default: break;
	}
}

void lineto_xy(MOTOR_DESC *m1, MOTOR_DESC *m2, unsigned int x2, unsigned int y2)
{
	int xinc, yinc, xerr2, yerr2, dx, dy, dx2, dy2;
	unsigned int *x, *y;

	x = &(m1->curr_step_pos);
	y = &(m2->curr_step_pos);

	xinc = yinc = 1;;

	dx = x2 - *x;
	m1->d = m2-> d = REEL_OUT;
	if( dx < 0 )
	{
		xinc = -1;
		m1->d = REEL_IN;
		dx = -dx;
	}
	dx2 = dx * 2;

	dy = y2 - *y;
	if( dy < 0 )
	{
		yinc = -1;
		m2->d = REEL_IN;
		dy = -dy;
	}
	dy2 = dy * 2;

	if( (0 != dx) | (0 != dy) )
	{
		if( dy <= dx )		// Slope <= 1
		{
			xerr2 = 0;
			do{
				*x += xinc;
				xerr2 += dy2;
				one_step(m1);
				if( xerr2 > dx )
				{
					*y += yinc;
					one_step(m2);
					xerr2 -= dx2;
				}
			} while(*x != x2);
		}
		else
		{
			yerr2 = 0;
			do {
				*y += yinc;
				one_step(m2);
				yerr2 += dx2;
				if( yerr2 > dy )
				{
					*x += xinc;
					one_step(m1);
					xerr2 -= dy2;
				}
			} while(*y != y2);
		}
	}
}

void my_delay(int d)
{
	int i;

	for(i=d; i>0; i--)
		while(!g_timer_fired);

	TACCTL0 &= ~CCIE;	// Clear timer interrupt
	g_timer_fired = 0;
	TACCTL0 = CCIE;		// Re-enable timer interrupt
}
/*
void reel_in_out(MOTOR_DESC *m, unsigned long len)
{
	int			num_steps;
	num_steps = (int)(len/LEN_REELED_PER_STEP);

	register_timer(one_step, m, STEP_INTERVAL, num_steps);
}
*/

// Timer A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
{
	g_timer_fired = 1;
}
/*
int register_timer(TIMER_CBK p_cbk, void *p_args)
{
	if(g_num_timers < MAX_TIMERS)
	{
		timer_context[g_timer_tail].p_fn = p_cbk; 		// Tail always points to the next empty slot
		timer_context[g_timer_tail].args = p_args;
		g_timer_tail++; g_num_timers++;
		if(g_timer_tail >= MAX_TIMERS) g_timer_tail = 0;
	}
	else
		g_timer_full_err++;

	return g_timer_tail;
}

inline void test_timer(void *args)
{
	P1OUT ^= BIT0;
	P2OUT ^= BIT0;
	CCR0 += 50000;
}
*/
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
