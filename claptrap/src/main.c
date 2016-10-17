/*
 * main.c
 *
 * NOTE
 * this program is designed to run on the atmega324p microcontroller
 * usage with any other model may not function properly.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>


// pins for controlling external devices
#define CTRL_DDR	DDRD
#define CTRL_PORT 	PORTD
#define CTRL_PIN	PIND

#define CTRL_AUDIO_ACT	PIND6

#define CTRL_BLINK_TOG	PIND7

// pins for sending data to audio device
#define AUDIO_DDR	DDRB
#define AUDIO_PORT	PORTB
#define AUDIO_PIN	PINB
#define AUDIO_PINCOUNT	8
#define AUDIO_MAX_DELAY	60

// amount of time the eye LED should be on/off
#define BLINK_MAX_OPEN	15000
#define BLINK_MAX_SHUT	300

// minimum and maximum distance a detected object can
#define SCAN_RANGE_NEAR	10
#define SCAN_RANGE_WARN	5

// pins for sending data to the wheel motors
#define MOTOR_DDR	DDRD
#define MOTOR_PORT	PORTD
#define MOTOR_PIN	PORTD
#define MOTOR_L_BACK	PIND5
#define MOTOR_L_FORWARD	PIND4
#define MOTOR_R_BACK	PIND3
#define MOTOR_R_FORWARD	PIND2

// different bitwise configurations for the motor
#define MOTOR_HALT	~(1 << MOTOR_L_BACK | 1 << MOTOR_L_FORWARD | 1 << MOTOR_R_BACK | 1 << MOTOR_R_FORWARD)
#define MOTOR_FORWARD	(1 << MOTOR_L_FORWARD | 1 << MOTOR_R_FORWARD)
#define MOTOR_BACKWARD	(1 << MOTOR_L_BACK | 1 << MOTOR_R_BACK)
#define MOTOR_LEFT	(1 << MOTOR_L_BACK | 1 << MOTOR_R_FORWARD)
#define MOTOR_RIGHT	(1 << MOTOR_R_BACK | 1 << MOTOR_L_FORWARD)

// amount of time it takes for the motor to turn
#define MOTOR_TURN_TIME	4000

// pseudonymes for integers 0 and 1 for marking an obstacle
#define PATH_CLEAR	0
#define PATH_OBSTICLE	1

// function prototypes

// initialize everything
void init();

// initialize the audio device
void init_audio();

// configure pins to control eye LED
void init_blink();

// initialize the scanner device
void init_scanner();

// initialize the motors
void init_motors();

// triggers a random audio clip to play
void trig_audio();

/*
 * turns the eye off for a random amount of time (BLINK_MAX_SHUT is max)
 * and then turns it back on
 */
void blink();

// read the scanner to detect obsticles
void scan();

// handles scanner data and enacts appropriate action if obsticles are found
void handle_scan();

// stops the motors from spinning
void halt_motors();

// sets the motor movement to the given configuration
void set_motors(unsigned int);
void notify_path(unsigned int);

// globals
unsigned int AudioClockCount = 0;
unsigned int Scan = 1;

// Main
int main()
{
	// enable interrupts
	sei();

	// initialize the device and all subsystems
	init();
	init_audio();
	init_scanner();
	init_motors();
	init_blink();

	// begin audio playback
	_delay_ms(1000);
	trig_audio();

	// main program loop
	while (1)
	{
		// blink
		blink();
		if (Scan)
		{
			// detect obsticles
			scan();
		}
		// regulate clock
		_delay_ms(1);
	}
	return 0;
}


// audio interrupts
ISR(PCINT3_vect)
{
	/*
	 * if the audio clip finished, schedule another one to start after a
	 * random amount of time
	 */
	if (bit_is_clear(CTRL_PIN, CTRL_AUDIO_ACT))
	{
		AUDIO_PORT = 0b11111111;
	}
	else
	{
		// configure the timer for a certain amount of seconds
		TCCR2B |= 1 << CS22 | 1 << CS21 | 1 << CS20;
		AudioClockCount = rand() % AUDIO_MAX_DELAY;
	}
}

ISR(TIMER2_COMPA_vect)
{
	// timer interrupt triggers audio after a delay
	if (AudioClockCount == 0)
	{
		TCCR2B = 0;
		trig_audio();
	}
	else
	{
		AudioClockCount--;
	}
}

// scan results interrupt
ISR(ADC_vect)
{
	// handle scan if object was detected
	handle_scan();
}

// motor turning callback
ISR(TIMER1_COMPA_vect)
{
	Scan = 1;
}

void init()
{
	// reset control ports
	CTRL_DDR = 0;
	CTRL_PORT = 0;
}


// Audio Functions

void init_audio()
{
	// configure audio delay timer and interrupts
	TCCR2A |= 1 << WGM21;
	 // if using atmega324, this flag is in regester TIMSK1
	TIMSK2 |= 1 << OCIE2A;
	OCR2A = 255;

	// configure audio control pins
	AUDIO_DDR = 0b11111111;
	AUDIO_PORT = 0b11111111;

	PCICR |= 1 << PCIE3;
	PCMSK3 |= 1 << PCINT30;
}

void trig_audio()
{
	static unsigned int Pin = 6;
	int pb;
	AUDIO_PORT = ~(1 << Pin);
	srand(rand());
	do
	{
		pb = rand() % AUDIO_PINCOUNT;
	} while (pb == Pin);
	Pin = pb;
}

void init_blink()
{
	// set the blink control pin to output
	CTRL_DDR |= 1 << CTRL_BLINK_TOG;
}

void blink()
{
	static unsigned int delay = 0;
	static unsigned char state = 0;
	if (delay == 0)
	{
		switch(state)
		{
		case 0:
			delay = rand() % BLINK_MAX_OPEN;
			break;
		case 1:
			delay = rand() % BLINK_MAX_SHUT;
			break;
		default:
			break;
		}
		state ^= 1;
		CTRL_PORT ^= 1 << CTRL_BLINK_TOG;
	}
	else
	{
		delay--;
	}
}

void init_scanner()
{
	// configure the analog to digital converter to read scanner input
	ADCSRA |= 1 << ADPS2;
	ADMUX |= 1 << ADLAR;
	ADMUX |= 1 << REFS0;
	ADCSRA |= 1 << ADIE;
	ADCSRA |= 1 << ADEN;

	scan();
}

void handle_scan()
{
	int adc_val = ADCH;
	// notify if the voltage signifies a close enough object
	notify_path(adc_val <= 7);
}

void scan()
{
	ADCSRA |= 1 << ADSC;
}

void init_motors()
{
	// configure the proper control pins for the motors
	int mask = 1 << MOTOR_L_BACK | 1 << MOTOR_L_FORWARD;
	mask |= 1 << MOTOR_R_BACK | 1 << MOTOR_R_FORWARD;
	MOTOR_DDR |= mask;
	MOTOR_PORT |= mask;

	// configure the motor timer for when to switch directions
	TCCR1B |= 1 << WGM12;
	TIMSK1 |= 1 << OCIE1A;
	OCR1A = MOTOR_TURN_TIME;
}

void halt_motors()
{
	// set the motors to stop
	static const unsigned int mask = ~(1 << MOTOR_L_BACK |
			1 << MOTOR_L_FORWARD |
			1 << MOTOR_R_BACK |
			1 << MOTOR_R_FORWARD);
	MOTOR_PORT &= mask;
}

void set_motors(unsigned int mask)
{
	// first stop the motor
	halt_motors();
	// then start it in the new direction
	MOTOR_PORT |= mask;
}

void notify_path(unsigned int path)
{
	static int dir = 0;
	switch(path)
	{
	case PATH_OBSTICLE:
		// if an obsticle is found, turn in a random direction
		if (dir == 0)
		{
			dir = rand() % 2;
			switch(dir)
			{
			case 0:
				dir = MOTOR_LEFT;
				break;
			default:
				dir = MOTOR_RIGHT;
				break;
			}
		}
		Scan = 0;
		TCCR1B |= 1 << CS10 | 1 << CS11;
		set_motors(dir);
		break;
	case PATH_CLEAR:
		// if no obsticle is found, go forward
		dir = 0;
		TCCR1B &= ~(1 << CS10 | 1 << CS11);
		set_motors(MOTOR_FORWARD);
		break;
	default:
		break;
	}
}
