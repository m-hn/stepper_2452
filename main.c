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
 * When pen is at (0,0), M1 will be REEL_OUT to MAX_Y and M2 to MAX (along diagonal)
 */

#include <msp430g2452.h>

#define ABS(x)	((x)<0?(-x):(x))

unsigned const char one_phase_seq[] = {BIT1, BIT2, BIT3, BIT4, BIT1, BIT2, BIT3, BIT4};	// Bits are lt. shifted 1 since first bit is enable bit which is set separately
unsigned const char two_phase_seq[] = {BIT1|BIT2, BIT2|BIT3, BIT3|BIT4, BIT4|BIT1, BIT1|BIT2, BIT2|BIT3, BIT3|BIT4, BIT4|BIT1};
unsigned const char interleave_seq[] = {BIT1, BIT1|BIT2, BIT2, BIT2|BIT3, BIT3, BIT3|BIT4, BIT4, BIT4|BIT1};
#define MAX_SEQ_INDX		(7)		// size of _seq[] array -1

#define NULL				((void *)0)
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
 * Defining X & Y limits in # of steps allows working with smaller numbers
 */
#define MAX_X_STEPS			(330)
#define MAX_Y_STEPS			(427)
#define MAX_STEPS			(540)

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
	unsigned int	curr_step_len;		// Current X/Y position, in # of steps
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
		MOTOR_DESC			*motor_desc);

void stop_motor(
		MOTOR_DESC			*m);

void lineto_xy(
		MOTOR_DESC 			*m1,	// X motor
		MOTOR_DESC 			*m2, 	// Y motor
		unsigned int 		x, 		// Where to go- Absolute x & Y coordinates in #steps
		unsigned int		y);

void my_delay(
		int					delay);	// Delay in multiples of 10ms

unsigned int my_sqrt(
		unsigned long		x);		// My integer square root implementation

/*unsigned char *malloc();

int free(unsigned char *m);
*/

// Global variables
MOTOR_DESC	m1, m2;
unsigned int g_curr_x, g_curr_y;

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

	// Initialize motor descriptors- Assumes start position is at origin
	// Since origin is lower left corner and motors are mounted on top left & right corners,
	// the starting position of (m1, m2) is (MAX_Y_STEPS, MAX_STEPS {Along diagonal})
	m1.curr_seq = 0;
	m1.mode = (unsigned char *)one_phase_seq;
	m1.port = PORT_P1;
	m1.curr_step_len = MAX_Y_STEPS;
	g_curr_x = 0;
	m1.d = REEL_OUT;

	m2.curr_seq = 0;
	m2.mode = (unsigned char *)one_phase_seq;
	m2.port = PORT_P2;
	m2.curr_step_len = MAX_STEPS;
	g_curr_y = 0;
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

	lineto_xy( &m1, &m2, 0, 0);
	my_delay(1000);
	lineto_xy( &m1, &m2, MAX_X_STEPS, 0);

	lineto_xy( &m1, &m2, MAX_X_STEPS, MAX_Y_STEPS);

	lineto_xy( &m1, &m2, 0, MAX_Y_STEPS);
}

void one_step(MOTOR_DESC *p_m)
{
	g_d_onestep ++;

	if(p_m->curr_step_len < MAX_STEPS)		// Not already reeled out fully
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
	unsigned int x1, y1, lx, ly;
	int i,j,k,l;
	long tmp;

	x1 = g_curr_x;
	y1 = g_curr_y;

	i = x1+x2; j = x1-x2;
	k = y1+y2; l = y1-y2;

	tmp = i*j+k*l-2*MAX_Y_STEPS*l;
	lx = (unsigned int)my_sqrt(ABS(tmp));
	ly = (unsigned int)my_sqrt(ABS(tmp-2*MAX_X_STEPS*j));


	if(j < 0)	// Reel out since the destination point is lower than the starting X coord
	{
		m1->d = REEL_OUT;
		m1->curr_step_len += lx;
	}
	else
	{
		m1->d = REEL_IN;
		m1->curr_step_len -= lx;
	}
	if(l < 0)	// Reel out since the destination point is lower than the starting Y coord
	{
		m2->d = REEL_OUT;
		m2->curr_step_len += ly;
	}
	else
	{
		m2->d = REEL_IN;
		m2->curr_step_len -= ly;
	}
	while(lx>0)
	{
		one_step(m1);
		lx--;
		if(ly >0)
		{
			one_step(m2);
			ly--;
		}
	}

	g_curr_x = x2; g_curr_y = y2;
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

/*
 * Assumes int is 16 bits and char is 8 bits
 */
unsigned int my_sqrt(unsigned long x) -- Fix this
{
	unsigned int j = 0x8000, l = 0x8000, m;
	unsigned long k;

	do {
		k = j*j;
		l >>=1;
		if( k == x)		// Found a perfect sqrt
			break;
		else if(k > x)	// Over shot. Need to undo the previous bit op
		{
			m = ~(l<<1); // Mask out all the bits except for the last op
			j &= m;
			j +=l;
		}
		else
		{
			j += l;
		}
	}while(l>1);

	return j;
}
