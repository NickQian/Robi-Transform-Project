#include <fcntl.h>
#include <iostream>
#include <stdlib.h>         //malloc size_t
#include <time.h>
#include <stdint.h>


#include "peri.h"
#include "ultrasonic.h"

struct timer_list timer_usc;   //ultrasonic;

int request_irq()

void free_irq()


unsigned int distmu(){        // distance measure use ultrasonic

    unsigned int dist_result = 0;
   
    gpio_write(PIN_ULTRASNC_TRIG, 1);        // set trig


if ( cpu_gpio_eds(PIN_ULTRASNC_ECHO) ){

   cpu_gpio_set_eds(PIN_ULTRASNC_ECHO);


return dist_result;

}
