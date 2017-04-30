/**
 * \file    gloveDriver.c
 * \brief   This module contains an implementation for the glove
 *          driver
 * \author  Josh Wildey
 * \author  Ian Hogan
 * 
 * Course   ENG EC535
 *          Final Project
 * This file implements a kernel module driver that interfaces to
 * digital IO using GPIO pins.  The GPIO are an accumulations of 
 * buttons, joystick and a tilt sensor.  This kernel driver reads
 * the DIO and sets a state that can be read from a user space
 * program.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h> /* printk() */
#include <linux/slab.h> /* kmallac() */
#include <linux/fs.h> /* everything... */
#include <linux/errno.h> /* error codes */
#include <linux/types.h> /* size_t */
#include <linux/proc_fs.h>
#include <linux/fcntl.h> /* 0_ACCMODE */
#include <linux/jiffies.h> /* jiffies */
#include <asm/system.h> /* cli(), *_flags */
#include <asm/uaccess.h> /* copy_from/to_user */
#include <linux/timer.h> /* kernel timer */
#include <linux/list.h>
#include <asm/string.h>
#include <linux/signal.h>
#include <asm/arch/hardware.h> /* GPIO access */
#include <asm/arch/gpio.h>
#include <asm/arch/pxa-regs.h>

// GPIO Pin Definitions
#define GPIO_JOYSTICK_YL 17
#define GPIO_JOYSTICK_YR 101
#define GPIO_JOYSTICK_XU 28
#define GPIO_JOYSTICK_XD 29
#define GPIO_JOYSTICK_B 30
#define GPIO_TILT 31

#define GPIO_BTN 16

// GPIO Pin Names
#define GPIO_JOYSTICK_YL_NAME "JOYSTICK_YL"
#define GPIO_JOYSTICK_YR_NAME "JOYSTICK_YR"
#define GPIO_JOYSTICK_XU_NAME "JOYSTICK_XU"
#define GPIO_JOYSTICK_XD_NAME "JOYSTICK_XD"
#define GPIO_JOYSTICK_B_NAME "JOYSTICK_BTN"
#define GPIO_TILT_NAME "TILT"
#define GPIO_BTN_NAME "BTN"

// GPIO High/Low Definitions
#define GPIO_HIGH 1
#define GPIO_LOW  0

// Max read length
#define MAX_MSG_LEN 8


/************************************
 * Set Module Info
 ************************************/
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("GLOVE CONTROLLER");
MODULE_AUTHOR("Josh Wildey, Ian Hogan");

/************************************
 * Declaration of mygpio.c functions
 ************************************/
static int glove_open(struct inode *inode, struct file *filp);
static int glove_release(struct inode *inode, struct file *filp);
static ssize_t glove_read(struct file *filp,
 char *buf, size_t count,
 loff_t *f_pos);
static ssize_t glove_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos);
static void glove_exit(void);
static int glove_init(void);

/************************************
 * Structure Definitions
 ************************************/
// Structure that declares the usual file access functions
struct file_operations glove_fops = {
    read: glove_read,
    write: glove_write,
    open: glove_open,
    release: glove_release
};

/************************************
 * Global Variables
 ************************************/
// Major number
static int glove_major = 61;

// Counter Period (ms)
static int ctrPer = 500;

// Robotic Arm Global Variable to be sent over BT
static int js_x = 0;   // joystick X direction 0-none, 1-up, 2-down
static int js_y = 0;   // joystick Y direction 0-none, 1-left, 2-right
static int tilt_z = 0; // indicates that tilt is active 
                       // (move in z direction, 0-none, 1-forw, 1-back)
static int gripper_state = 0;     // indicates whether to open or close the gripper (0-open, 1-close)

// Recurring Timer
static struct timer_list gTimer;

// Message
static char *msgBuffer;
static int msgBufferLen = MAX_MSG_LEN;

/************************************
 * Declaration of the init and exit 
 * functions
 ************************************/
// Set intialization and exit functions
module_init(glove_init);
module_exit(glove_exit);


/** 
 * \brief Callback function when timer expires
 *
 * This is the function that is called by the timer.
 * When expired, this function will update the state
 * of the DIO to be read from the user level program.
 *
 * \param data - unused
 */
static void timerCallbackFcn(unsigned long data)
{
    // Variables
    int joystick_xu = 0;    
    int joystick_xd = 0;
    int joystick_yl = 0;    
    int joystick_yr = 0;

    int tilt = 0;         
    int gripper = 0;          
    int tilt_state = 0;

    // Read GPIO Values
    joystick_xu = pxa_gpio_get_value(GPIO_JOYSTICK_XU);
    joystick_xd = pxa_gpio_get_value(GPIO_JOYSTICK_XD);
    joystick_yl = pxa_gpio_get_value(GPIO_JOYSTICK_YL);
    joystick_yr = pxa_gpio_get_value(GPIO_JOYSTICK_YR);
    tilt = pxa_gpio_get_value(GPIO_TILT);
    gripper = pxa_gpio_get_value(GPIO_JOYSTICK_B);

    tilt_state = pxa_gpio_get_value(GPIO_BTN);

    // DEBUG
    printk(KERN_INFO "Read: Xu: %d Xd: %d Yl: %d Yr: %d T: %d Ts: %d G: %d\n", joystick_xu,
                                                                               joystick_xd,
                                                                               joystick_yl,
                                                                               joystick_yr,
                                                                               tilt,
                                                                               tilt_state,
                                                                               gripper);
    // END DEBUG

    // X Direction State
    if (joystick_xu > 0 )
    {
        js_x = 1;    // set x direction up
    } else if (joystick_xd ==  0) { // reverse logic due to hardware
        js_x = 2;    // set x direction down
    } else {
        js_x = 0;    // do nothing
    }

    // Y Direction State   
    if (joystick_yl > 0 )
    {
        js_y = 1;    // set y direction left
    } else if (joystick_yr > 0) {
        js_y = 2;    // set y direction right
    } else {
        js_y = 0;    // do nothing
    }

    // Z Direction State
    if (tilt != 0 )
    {
        if (tilt_state == 0) {
            tilt_z = 2;    // set z backwards
        } else {
            tilt_z = 1;    // set z forwards
        }
    } else {
        tilt_z = 0;    // do nothing
    }    

    // Gripper 
    if (gripper > 0) {
        gripper_state = 1;    // close gripper
    } else {
        gripper_state = 0;    // open gripper
    }

    // DEBUG
    printk(KERN_INFO "X: %d Y: %d Z: %d Grip: %d\n", js_x, js_y, tilt_z, gripper_state);
    // END DEBUG

    // Reset Timer
    mod_timer(&gTimer, jiffies + msecs_to_jiffies(ctrPer));
}


/**
 *  \brief Initialize glove driver module
 *
 *  The kernel space function which corresponds to inserting
 *  the module to the kernel.
 *
 *  \return 0 for success, otherwise error
 */
static int glove_init(void)
{
    // Variables
    int result;

    // Registering device
    result = register_chrdev(glove_major, "gloveDriver", &glove_fops);
    if (result < 0)
    {
        printk(KERN_ALERT "gloveDriver: cannot obtain major number %d\n", glove_major);
        return result;
    }

    // Allocate space for message buffer
    msgBuffer = kmalloc(MAX_MSG_LEN, GFP_KERNEL);
    if (!msgBuffer)
    {
        printk("gloveDriver: cannot allocate space for message buffer\n");
        return -ENOMEM;
    }

    // Request GPIO pins
    gpio_request(GPIO_JOYSTICK_XU, GPIO_JOYSTICK_XU_NAME);
    gpio_request(GPIO_JOYSTICK_XD, GPIO_JOYSTICK_XD_NAME);
    gpio_request(GPIO_JOYSTICK_YL, GPIO_JOYSTICK_YL_NAME);
    gpio_request(GPIO_JOYSTICK_YR, GPIO_JOYSTICK_YR_NAME);
    gpio_request(GPIO_JOYSTICK_B, GPIO_JOYSTICK_B_NAME);
    gpio_request(GPIO_TILT, GPIO_TILT_NAME);
    gpio_request(GPIO_BTN, GPIO_BTN_NAME);

    // Set GPIO pin direction
    gpio_direction_input(GPIO_JOYSTICK_XU);
    gpio_direction_input(GPIO_JOYSTICK_XD);
    gpio_direction_input(GPIO_JOYSTICK_YL);
    gpio_direction_input(GPIO_JOYSTICK_YR);
    gpio_direction_input(GPIO_JOYSTICK_B);

    gpio_direction_input(GPIO_TILT);
    gpio_direction_input(GPIO_BTN);

    // Setup Timer
    setup_timer(&gTimer, timerCallbackFcn, 0);
    mod_timer(&gTimer, jiffies + msecs_to_jiffies(ctrPer));

    printk(KERN_INFO "gloveDriver: module loaded.\n");

    return 0;
}


/**
 *  \brief De-intializes glove driver module
 *
 *  The kernal space function which corresponds to removing
 *  the module from the kernel.
 *
 */
static void glove_exit(void)
{
    // Freeing the major number
    unregister_chrdev(glove_major, "gloveDriver");

    // Free up GPIOs
    gpio_free(GPIO_JOYSTICK_XU);
    gpio_free(GPIO_JOYSTICK_XD);
    gpio_free(GPIO_JOYSTICK_YL);
    gpio_free(GPIO_JOYSTICK_YR);
    gpio_free(GPIO_JOYSTICK_B);
    gpio_free(GPIO_TILT);
    gpio_free(GPIO_BTN);

    // Delete Timer
    del_timer(&gTimer);
    
    // Free up buffer memory
    if (msgBuffer)
    {
        kfree(msgBuffer);
    }

    printk(KERN_INFO "gloveDriver: module unloaded.\n");
}


/**
 *  \brief Open the glove device as a file
 *
 *  The Kernel Space function, which corresponds to opening the
 *  /dev/glove file in user space. This function is not
 *  required to do anything for this driver
 *
 *  \param *inode - info structure with major/minor numbers
 *  \param *filp  - info structure with available file operations
 *  \return 0 for success
 */
static int glove_open(struct inode *inode, struct file *filp)
{
    // Nothing to do here
    return 0;
}


/**
 *  \brief Release the glove device as a file
 *
 *  The Kernel Space function which corresponds to closing the
 *  /dev/glove file in user space. This function is not
 *  required to do anything for this driver
 *
 *  \param *inode - info structure with major/minor numbers
 *  \param *filp  - info structure with available file operations
 *  \return 0 for success
 */
static int glove_release(struct inode *inode, struct file *filp)
{
    // Nothing to do here
    return 0;
}


/**
 * \brief Read the glove device as a file
 *
 * The kernel space function which corresponds to reading the
 * /dev/glove file in user space
 *
 * \param *filp  - info structure with available file ops
 * \param *buf   - output string
 * \param count  - number of chars/bytes to read
 * \param *f_pos - position in file
 * \return number of bytes read
 */
static ssize_t glove_read(struct file *filp, char *buf,
                          size_t count, loff_t *f_pos)
{
    // Clear the buffer
    memset(msgBuffer, 0, MAX_MSG_LEN);

    // Check if reached max length for message
    if (*f_pos >= msgBufferLen)
    {
        return 0;
    }
    
    // Build string
    sprintf(msgBuffer, "%d %d %d %d\n", js_x, js_y, tilt_z, gripper_state);
    msgBufferLen = strlen(msgBuffer);

    // Copy to user space
    if (copy_to_user(buf, msgBuffer, msgBufferLen))
    {
        printk(KERN_ALERT "gloveDriver: Fault in copying to user space\n");
        return -EFAULT;
    }

    // Update File position
    *f_pos += msgBufferLen;

    // Return length of string
    return msgBufferLen;
}


/**
 *  \brief Write to the glove device as a file
 *
 *  The kernel space function which corresponds to writing to the
 *  /dev/glove file in user space.
 *
 *  \param *filp  - info structure with available file operations
 *  \param *buf   - input string from user space
 *  \param count  - num of chars/bytes to write
 *  \param *f_pos - position of file
 *  \return number of bytes written
 */
static ssize_t glove_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
    // Nothing to do here
    return 0;
}
