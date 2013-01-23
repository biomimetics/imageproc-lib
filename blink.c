/* blink.c 
Ron Fearing Dec. 19, 2012
make LEDs blink even if interrupts are over riding LED
setting */
// leave LEDs off when done 

#include "led.h"
#include "stopwatch.h"

// n number of blinks, k number of milliseconds
void blink_leds(int n, int k)
{ int i,j;
	for (i = 0; i < n; i++)
	{ for (j = 0; j < 10*k; j++)
	   { LED_RED = 0;
	     LED_GREEN = 1;
	     LED_BLUE = 0;
		swatchDelayUs(100); /* wait 100 us */
	   }
	   for (j = 0; j < 10*k; j++)
	  { LED_RED = 1;
	     LED_GREEN = 0;
	     LED_BLUE = 1;
		swatchDelayUs(100); /* wait 100 us */
	   }
 	   LED_RED = 0;
	   LED_GREEN = 0;
	   LED_BLUE = 0;
    }
}
