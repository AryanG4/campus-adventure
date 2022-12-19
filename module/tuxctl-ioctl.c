/* tuxctl-ioctl.c
 *
 * Driver (skeleton) for the mp2 tuxcontrollers for ECE391 at UIUC.
 *
 * Mark Murphy 2006
 * Andrew Ofisher 2007
 * Steve Lumetta 12-13 Sep 2009
 * Puskar Naha 2013
 */

#include <asm/current.h>
#include <asm/uaccess.h>

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/file.h>
#include <linux/miscdevice.h>
#include <linux/kdev_t.h>
#include <linux/tty.h>
#include <linux/spinlock.h>

#include "tuxctl-ld.h"
#include "tuxctl-ioctl.h"
#include "mtcp.h"

#define debug(str, ...) \
	printk(KERN_DEBUG "%s: " str, __FUNCTION__, ## __VA_ARGS__)


/************************** FLAGS **************************/

static unsigned int ACK_FLAG = 1; // 1 if waiting for ACK, 0 otherwise
//volatile int CLK_FLAG = 0; // 1 if in clock mode
//volatile int USR_FLAG = 0; // 1 if set led in usr mode

/************************** GLOBALS **************************/

static unsigned long led_state; // current state of the LEDs
static unsigned char button_packet[3]; // gets button status packet
static unsigned char hex_array[16] = {0xE7, 0x06, 0xCB, 0x8F, 0x2E, 0xAD, 
	0xED, 0x86, 0xEF, 0xAF, 0xEE, 0x6D, 0xE1, 0x4F, 0xE9, 0xE8}; // 0 - 9 and A - F in seven segment display, b and d are lowercase to avoid confusion


/*************************Function Headers*********************/
static void tux_init(struct tty_struct* tty);
static void tux_set_led(struct tty_struct* tty, unsigned long arg);
static int tux_buttons(struct tty_struct* tty, unsigned long arg);
static uint8_t get_led(char hex_index, char current_dp);
static spinlock_t button_lock = SPIN_LOCK_UNLOCKED;
/************************ Protocol Implementation *************************/

// input - tty stucy
// output - return 0
// side effect -  initiliazes driver variables
void tux_init(struct tty_struct* tty){
	unsigned char init[2]; // make buffers
	init[0] = MTCP_LED_USR; //// set led to usr mode
	init[1] = MTCP_BIOC_ON; // get ack from driver
	tuxctl_ldisc_put(tty, init, 2); // pass data to tux bus
	
	
	ACK_FLAG = 0;  // tell the interrupt handler to wait for acknowledge
	led_state = 0xFFFF0000; // set all LEDs to 0, clears the leds
	tux_set_led(tty, led_state);
	return;
}

// input - hex value from mask and decimal point flag
// output - returns seven-segment hex
// side effect -  
uint8_t get_led(char hex_index, char current_dp){
	uint8_t displayed_led_val; // assign display to a return value
      switch (hex_index){
          case 0x0: //HEX 0X0
              displayed_led_val = hex_array[0]; //7segment display
              break;
          case 0x1: //HEX 0X1
              displayed_led_val = hex_array[1]; //7segment display
              break;
          case 0x2: //HEX 0X2
              displayed_led_val = hex_array[2]; //7segment display
              break;
          case 0x3: //HEX 0X3
              displayed_led_val = hex_array[3]; //7segment display
              break;
          case 0x4: //HEX 0X4
              displayed_led_val = hex_array[4]; //7segment display
              break;
          case 0x5: //HEX 0X5
              displayed_led_val = hex_array[5]; //7segment display
              break;
          case 0x6: //HEX 0X6
              displayed_led_val = hex_array[6]; //7segment display
              break;
          case 0x7: //HEX 0X7
              displayed_led_val = hex_array[7]; //7segment display
              break;
          case 0x8: //HEX 0X8
              displayed_led_val = hex_array[8]; //7segment display
              break;
          case 0x9: //HEX 0X9
              displayed_led_val = hex_array[9]; //7segment display
              break;
          case 0xA: //HEX 0XA
              displayed_led_val = hex_array[10]; //7segment display
              break;
          case 0xB: //HEX 0XB
              displayed_led_val = hex_array[11]; // b:0x6D B:0xEF
              break;
          case 0xC: //HEX 0XC
              displayed_led_val = hex_array[12]; //7segment display
              break;
          case 0xD: //HEX 0XD
              displayed_led_val = hex_array[13]; // d:0x4F D:0xE7
              break;
          case 0xE: //HEX 0XE
              displayed_led_val = hex_array[14]; //7segment display
              break;
          case 0xF: //HEX 0XF
              displayed_led_val = hex_array[15]; //7segment display
              break;
          default:  //default case
              return 0;
      }
      if(current_dp){
          displayed_led_val += 0x10; // light up dp by adding 0x10 to hex display
      }
      return displayed_led_val;
}


// input - tty stuct, led value
// output - return 0
// side effect -  converts input into led value on the 7 segment
void tux_set_led(struct tty_struct* tty, unsigned long arg){
	uint8_t led_buffer[6]; //initialize buffer
	char led_set, dp_set; //initialize variables
	int i;
	unsigned long temp_arg;
	int offset = 2;
	
	unsigned long bitmask = 0x000F; //mask for last 4 bits of arg
	char led_mask = 0x01; //mask for last bit of arg
	//unsigned long on_mask = 0x0F; //mask to check which led is on

	temp_arg = arg; //make temp arg since we will be shifting it
	

	led_buffer[0] = MTCP_LED_SET; //set led with usr set values
	led_set = (arg >> 16) & 0x0F; //right shift 16 bits to get which leds are on
	dp_set = (arg >> 24) & 0x0F; //right shift 24 bits to get which decimal point is set
	led_buffer[1] = led_set; // but the led to set in buffer

	//set the 4 leds
	
	
	for(i = 0; i < 4; i++){
		char current_led = led_set & led_mask; //get the current led
		if(current_led == led_mask){
			char current_dp = dp_set & led_mask; //get the current decimal point
			char hex_index = (char)(temp_arg & bitmask); //get the hex value of the led
			//int num_index = (int)strol(hex_index, NULL, 0)
			led_buffer[offset] = get_led(hex_index, current_dp); //get the 7 segment value
			offset++; //increment offset
			// if(current_dp){
			// 	led_buffer[i+2] += 0x10;	
			// }
		}

		led_mask <<= 1; //shift mask to next bit
		temp_arg >>= 4; //shift arg to next 4 bits to get next hex value
	}
	if(ACK_FLAG == 0) return; // check if waiting for other functions
	ACK_FLAG = 0; // tell other functions to wait
	led_state = arg; //save current value of led 
	tuxctl_ldisc_put(tty, led_buffer, offset);
	return;
}


int tux_buttons(struct tty_struct* tty, unsigned long arg){
	unsigned char button_buffer[1]; //make buffer
	unsigned long *button_ptr;
	//int button_mask = 0x00000000;
	button_ptr = (unsigned long *)arg;
	if(!button_ptr) return -EINVAL; //check if arg is valid

	spin_lock_irq(&button_lock); //lock button

	
	button_buffer[0] = (button_packet[1] & 0x0F); //get button value
	button_buffer[0] = (button_buffer[0] | ((button_packet[2]<<4) & 0xF0)); 
	copy_to_user(button_ptr, button_buffer, 1); //send button value to user space
	spin_unlock_irq(&button_lock); //unlock button

	return 0;
}
/* tuxctl_handle_packet()
 * IMPORTANT : Read the header for tuxctl_ldisc_data_callback() in 
 * tuxctl-ld.c. It calls this function, so all warnings there apply 
 * here as well.
 */
void tuxctl_handle_packet (struct tty_struct* tty, unsigned char* packet)
{
    unsigned a, b, c;
	unsigned char ret_op[2]; //buffer for reset

    a = packet[0]; /* Avoid printk() sign extending the 8-bit */
    b = packet[1]; /* values when printing them. */
    c = packet[2];

	switch (a) {
      case MTCP_ERROR:
          return;
      case MTCP_ACK:
          ACK_FLAG = 1; // tell other functions to continue
          return;
      case MTCP_RESET:
        	 //reset, copied from init 
			ret_op[0] = MTCP_LED_USR;     //return to user mode
			ret_op[1] = MTCP_BIOC_ON;     //enable button interrupts
			tuxctl_ldisc_put(tty, ret_op, 2); //send to tux
			tux_set_led(tty, led_state);  //set led to previous state
          return;
      case MTCP_BIOC_EVENT:
          button_packet[1] = b; //save button packet
          button_packet[2] = c;
          return;
      default:
          return;
    }
    /*printk("packet : %x %x %x\n", a, b, c); */
}

/******** IMPORTANT NOTE: READ THIS BEFORE IMPLEMENTING THE IOCTLS ************
 *                                                                            *
 * The ioctls should not spend any time waiting for responses to the commands *
 * they send to the controller. The data is sent over the serial line at      *
 * 9600 BAUD. At this rate, a byte takes approximately 1 millisecond to       *
 * transmit; this means that there will be about 9 milliseconds between       *
 * the time you request that the low-level serial driver send the             *
 * 6-byte SET_LEDS packet and the time the 3-byte ACK packet finishes         *
 * arriving. This is far too long a time for a system call to take. The       *
 * ioctls should return immediately with success if their parameters are      *
 * valid.                                                                     *
 *                                                                            *
 ******************************************************************************/
int 
tuxctl_ioctl (struct tty_struct* tty, struct file* file, 
	      unsigned cmd, unsigned long arg)
{
	
	//unsigned char led_set[1];
	 
    switch (cmd) {
		case TUX_INIT:
			tux_init(tty);
			return 0;
		
		case TUX_BUTTONS:

			return tux_buttons(tty, arg);

		case TUX_SET_LED:
			
			// led_set[0] = MTCP_LED_USR; // set led to usr mode
			// tuxctl_ldisc_put(tty, led_set, 1); // pass data to tux bus
			tux_set_led(tty, arg);
			return 0;

		case TUX_LED_ACK:
		case TUX_LED_REQUEST:
		case TUX_READ_LED:
		default:
			return -EINVAL;
    }
}




