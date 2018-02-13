/*  c driver for optical encoder of the balance car motors(GPIO interrupt)
    ----
    Licensed under BSD license.
    by Nick Qian
    ----
    0.1 - 2017.4.27  init version for Raspberry 2B(bcm2836) by Nick Qian
    ----
    output: speed of the motor
    input: GPIO pin (work as interrupt)
*/

#include <stdint.h>
#include <sstream>

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/gpio.h>         // include/asm-generic/gpio.h
#include <linux/interrupt.h>
//#include <linux/irq.h>

#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"

#include "sensorprocess/BMotorSensMsg.h"


static short int opt_enc1A_irq =0;
static short int opt_enc2A_irq =0;
#ifdef USE_QUADRATURE_ENCODER
static short int opt_enc1B_irq =0;
static short int opt_enc2B_irq =0; 
#endif


static ros::NodeHandle nh;
static ros::Publisher optEnc_pub = nh.advertise<sensorprocess::BMotorSensMsg>("tpc_optEnc", 1000);
//static sensorprocess::BMotorSensMSg msg;

//------------------------------ Speed data ---------------------------------

struct motor_speed{
    struct timeval last_timestamp;
    int32_t pulseL, pulseR;
    int32_t speedL, speedR;
    spinlock_t spinlock;
};

static volatile motor_speed bmotor_spd;             //banlancer motor speed

//------------------------- Input Pin Data process ------------------------------
static unsigned char previous_seq_L, previous_seq_R;
static unsigned char delta_L, delta_R;


/**********************************************************************************
/* Ros Pub
/**********************************************************************************/

int ros_pub_sens_optEnc(sensorprocess::BMotorSensMSg msg){
    //std_msg::String msg;

    //std::stringstream ss;
    optEnc_pub.publish(msg);
}

sensorprocess::BMotorSensMSg measure_speed_once(){
    bmotor_spd.speedL = motor_spd.pulseL / PULESES_PER_TURN; 
    bmotor_spd.speedR = motor_spd.pulseR / PULESES_PER_TURN;

    sensorprocess::BMotorSensMSg msg;  //= new sensorprocess::BMotorSensMSg();
    msg.header.stamp = ros::Time.now();
    msg.header.frame_id = "bmotor_speed";
    msg.speedL = bmotor_spd.speedL;
    msg.speedR = bmotor_spd.speedR;

    // clear the pulse count
    bmotor_spd.pulseL = 0;
    bmotor_spd.pulseR = 0;

    return msg;
}


void count_L(){
    bmotor_spd.pulseL++;
}


void count_R(){
    bmotor_spd.pulseR++;
}

/* delta:
*    1: no change
*    2: 1 step clockwise
*    3: 2 step clockwise or counter-clockwise
*    4: 1 step counter clockwise
*/

uint8_t calcu_delta_A(a_state, b_state){

    uint8_t current_seq;

    current_seq = (a_state ^ b_state) | b_state << 1;
    delta_A = (current_seq - previous_seq_A) % 4;

}

uint8_t calcu_delta(a_state, b_state){
    uint8_t delta;
    uint8_t current_seq;

    current_seq = (a_state ^ b_state) | b_state << 1;
    delta = (current_seq - previous_seq_B) % 4;

    return delta;
}


/*********************************************************************************
/*     IRQ
/*********************************************************************************/

//static irqreturn_t irq_handle_optEnc_1A(int irq, void *dev_id, struct pt_regs *regs){
static irqreturn_t irq_handle_optEnc_1A(int irq, void *dev_id){

    unsigned long flags;

    local_irq_save(flags);
    printf("Info: Interrupt [%d] Triggered in <optEnc_1A_isr> for device %s.\n", irq, (char *) dev_id);

    /////// blabla /////////
    count_L();


    /////// end blabla /////

    local_irq_restore(flags);
    return IRQ_HANDLED;

}


static irqreturn_t irq_handle_optEnc_2A(int irq, void *dev_id, struct pt_regs *regs){

    unsigned long flags;

    local_irq_save(flags);
    printk("Info: Interrupt [%d] Triggered in <optEnc_2A_isr> for device %s.\n", irq, (char *) dev_id);

    /////// blabla /////////
    count_R();


    /////// end blabla /////

    local_irq_restore(flags);
    return IRQ_HANDLED;

}

/***************************************************************************
/* Module Init / Cleanup
/****************************************************************************/

// void __init r_init
int r_init(void){
    // set GPIO ass input
    gpio_funcSel(PIN_OPT_ENCODER_1A,  GPIO_FSEL_INPUT);
    gpio_funcSel(PIN_OPT_ENCODER_1B,  GPIO_FSEL_INPUT);
    gpio_funcSel(PIN_OPT_ENCODER_2A,  GPIO_FSEL_INPUT);
    gpio_funcSel(PIN_OPT_ENCODER_2B,  GPIO_FSEL_INPUT);


    //--------- Ros topic node init --------
    int argc = 0;
    char** argv = {};
    ros::init(argc, argv, "node_optEnc");
    //ros::NodeHandle n;
    ros::Rate loop_rate(50);     // 100Hz, 10ms max, 20ms = 2 pulse

    //------------- Irq register -----------
    //--- (1) check valid
    if (!gpio_is_valid(PIN_OPT_ENCODER_1A)){
        printk("Error: GPIO pin <PIN_OPT_ENCODER_1A> Invalid. \n")
    }

    if (!gpio_is_valid(PIN_OPT_ENCODER_2A)){
        printk("Error: GPIO pin <PIN_OPT_ENCODER_2A> Invalid. \n")

    //--- (2) request gpio PINs & set as input.   cat /sys/kernel/debug/gpio
    if (gpio_request(PIN_OPT_ENCODER_1A,  DEV_NAME_PHOENC_1A) >= 0){ //(unsigned gpio, const char *label)  return 0 if fine
        gpio_direction_input(PIN_OPT_ENCODER_1A);
     }
    else{
        printk("Error: GPIO <PIN_OPT_ENCODER_1A> request failed. \n")
    }

    if (gpio_request(PIN_OPT_ENCODER_2A,  DEV_NAME_PHOENC_2A) >= 0){
        gpio_direction_input(PIN_OPT_ENCODER_2A);
    }
    else{
        printk("Error: GPIO <PIN_OPT_ENCODER_2A> request failed. \n")
    }

    //--- (3) request irq
    if ( (opt_enc1A_irq = gpio_to_irq(PIN_OPT_ENCODER_1A) ) <0){
        printk(KERN_ERR "Unable to request IRQ <OPT_ENCODER_1A>" );
        return -1;
    }

    if ( (opt_enc2A_irq = gpio_to_irq(PIN_OPT_ENCODER_2A) ) <0){
        printk(KERN_ERR "Unable to request IRQ <OPT_ENCODER_2A>" );
        return -1;
    }

    //--- (4) link handler -----
    if (request_irq(opt_enc1A_irq,                         // [unsigned int irq]
                    irq_handle_optEnc_1A,                  // [irqreturn_t (*handler)]
                    IRQF_TRIGGER_RISING,                   // [unsigned long irqflags] interrupt mode flag
                    DEV_NAME_PHOENC_1A,                    // [const char *devname] see: /proc/interrupts
                    DEV_ID_PHOENC_1A)) <  0){              // [void *dev_id] shared interrupt lines
        printk("Error: Irq Request failure<request_irq:opt_enc1A_irq>\n");
        return -1;
    }

    if (request_irq(opt_enc2A_irq,                         // [unsigned int irq]
                    irq_handle_optEnc_2A,                  // [irqreturn_t (*handler)]
                    IRQF_TRIGGER_RISING,                   // [unsigned long irqflags] interrupt mode flag
                    DEV_NAME_PHOENC_2A,                    // [const char *devname] see: /proc/interrupts
                    DEV_ID_PHOENC_2A)) <  0){              // [void *dev_id] shared interrupt lines
        printk("Error: Irq Request failure<request_irq:opt_enc1A_irq>\n");
        return -1;
    }

   //----- measure & publish ---------
    while(ros::ok() ){

        sensorprocess::BMotorSensMSg msg = measure_speed_once();

        ros_pub_sens_optEnc(msg);

        ros::spinOnce();

        loop_rate.sleep();

    };


    return 0;

}


//void __exit(r_int_cleanup)
void r_int_cleanup(){

    free_irq(opt_enc1A_irq, DEV_ID_PHOENC_1A);   // (unsigned int irq, void *dev_id)
    free_irq(opt_enc2A_irq, DEV_ID_PHOENC_2A);

    #ifdef USE_QUADRATURE_ENCODER
    free_irq(opt_enc1B_irq, DEV_ID_PHOENC_2A);
    free_irq(opt_enc2B_irq, DEV_ID_PHOENC_2B);
    #endif

    //gpio_free(unsigned gpio);


    // say bye
    printk(KERN_NOTICE "Goodbye, Opt_encoder! \n");

}



/******************************************************************/

module_init(r_init);
module_exit(r_int_cleanup);


/******************************************************************
/* Module licensing/description
/******************************************************************/
MODULE_LICENSE("GPL")
MODULE_AUTHOR("The Rtp project author")
MODULE_DESCRIPTION("Rtp project Optical encoder's 4 GPIO input driver")


