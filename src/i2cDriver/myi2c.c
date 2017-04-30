/**
 * \file    myi2c.c
 * \brief   This module contains an i2c driver
 * \author  Josh Wildey
 * \author  Ian Hogan
 * 
 * Course   ENG EC535
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
#include <asm/arch/i2c.h>
#include <linux/delay.h>

#include "myi2c.h"

// GPIO Pin Definitions
#define X_SCL (117 | GPIO_ALT_FN_1_IN)
#define X_SDA (118 | GPIO_ALT_FN_1_IN)

// I2C Definitions
#define R_nW_READ  0x01
#define R_nW_WRITE 0x00

// Max read/write lengths
#define MAX_WRT_LEN 5
#define MAX_MSG_LEN 128

/************************************
 * Set Module Info
 ************************************/
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("I2C that actually works");
MODULE_AUTHOR("Josh Wildey, Ian Hogan");

/************************************
 * Declaration of myi2c.c functions
 ************************************/
static int myi2c_open(struct inode *inode, struct file *filp);
static int myi2c_release(struct inode *inode, struct file *filp);
static int myi2c_ioctl(struct inode *inode, struct file *file,
		               unsigned int cmd, unsigned long arg);
static ssize_t myi2c_read(struct file *filp,
                            char *buf,
                            size_t count,
                            loff_t *f_pos);
static ssize_t myi2c_write(struct file *filp,
		                     const char *buf,
                             size_t count,
                             loff_t *f_pos);
static void myi2c_exit(void);
static int myi2c_init(void);

/************************************
 * Structure Definitions
 ************************************/
// Structure that declares the usual file access functions
struct file_operations myi2c_fops = {
    read: myi2c_read,
	write: myi2c_write,
	open: myi2c_open,
	release: myi2c_release,
    ioctl: myi2c_ioctl
};

/************************************
 * Global Variables
 ************************************/
// Major number
static int myi2c_major = 61;

// Buffer and Length for read
static char *msgBuffer;
static int msgBufferLen;

/************************************
 * Declaration of the init and exit 
 * functions
 ************************************/
// Set intialization and exit functions
module_init(myi2c_init);
module_exit(myi2c_exit);

int i2c_write_1_byte(uint8_t addr, uint8_t value)
{
    // Step 1 Page 579 of PXA27x manual Section 9.4.9.2
    printk(KERN_INFO "%s: step 1", __FUNCTION__);
    IDBR = (addr << 1) | R_nW_WRITE;

    // Step 2
    printk(KERN_INFO "%s: step 2", __FUNCTION__);
    ICR |= ICR_START;  // set start
    ICR &= ~ICR_STOP;  // clear stop
    ICR &= ~ICR_ALDIE; // clear arbitrations interrupt
    ICR |= ICR_TB;     // set transfer byte bit

    // Step 3
    printk(KERN_INFO "%s: step 3", __FUNCTION__);
    while (~ (ISR & ISR_ITE)) // wait for transmit empty interrupt
    {
        //usleep(1);
    }

    // Step 4
    printk(KERN_INFO "%s: step 4", __FUNCTION__);
    ISR &= ~ISR_ITE;  // clear interrupt
    
    // Step 5
    printk(KERN_INFO "%s: step 5", __FUNCTION__);
    if ( ISR & ISR_ALD)  // write 0b1 to ISR[ALD] bit if set
    {
        ISR |= ISR_ALD;
    }

    // Step 6
    printk(KERN_INFO "%s: step 6", __FUNCTION__);
    IDBR = value; // load value to write into buffer

    // Step 7
    printk(KERN_INFO "%s: step 7", __FUNCTION__);
    ICR &= ~ICR_START; // clear start
    ICR |= ICR_STOP;   // set stop
    ICR |= ICR_ALDIE;  // set arbitrations interrupt
    ICR |= ICR_TB;     // set transfer byte bit

    // Step 8
    printk(KERN_INFO "%s: step 8", __FUNCTION__);
    while (~ (ISR & ISR_ITE)) // wait for transmit empty interrupt
    {
        //usleep(1);
    }

    // Step 9
    printk(KERN_INFO "%s: step 9", __FUNCTION__);
    ISR &= ~ISR_ITE;  // clear interrupt

    // Step 10
    printk(KERN_INFO "%s: step 10", __FUNCTION__);
    ICR &= ~ICR_STOP;   // clear stop

    return 0;
}

int i2c_read_1_byte(uint8_t addr, uint8_t *result)
{
    ISR &= ~ISR_ITE;  // clear interrupt
    printk(KERN_INFO "ISR %04x\n", ISR);
    printk(KERN_INFO "ICR %04x\n", ICR);
    // Step 1 Page 580 of PXA27x manual Section 9.4.9.3
    printk(KERN_INFO "===%s: step 1\n", __FUNCTION__);
    IDBR = (addr << 1) | R_nW_READ;
    printk(KERN_INFO "IDBR: %04x\n", IDBR);

    // Step 2
    printk(KERN_INFO "===%s: step 2\n", __FUNCTION__);
    ICR |= ICR_START;  // set start
    printk(KERN_INFO "ICR: %04x\n", ICR);
    ICR &= ~ICR_STOP;  // clear stop
    printk(KERN_INFO "ICR: %04x\n", ICR);
    ICR &= ~ICR_ALDIE; // clear arbitrations interrupt
    printk(KERN_INFO "ICR: %04x\n", ICR);
    ICR |= ICR_TB;     // set transfer byte bit
    printk(KERN_INFO "ICR: %04x\n", ICR);

    // Step 3
    printk(KERN_INFO "===%s: step 3\n", __FUNCTION__);
    printk(KERN_INFO "myi2c: ISR %04x\n", ISR);
    while ((ISR & ISR_ITE) != 0x40) // wait for transmit empty interrupt
    {
        mdelay(5);
    }
    printk(KERN_INFO "IDBR: %04x\n", IDBR);

    // Step 4
    printk(KERN_INFO "===%s: step 4\n", __FUNCTION__);
    ISR &= ~ISR_ITE;  // clear interrupt
    printk(KERN_INFO "IDBR: %04x\n", IDBR);

    // Step 5
    printk(KERN_INFO "===%s: step 5\n", __FUNCTION__);
    ICR &= ~ICR_START; // clear start
    ICR |= ICR_STOP;   // set stop
    ICR |= ICR_ALDIE;  // set arbitrations interrupt
    ICR |= ICR_ACKNAK; // set ack/nak status
    ICR |= ICR_TB;     // set transfer byte bit
    printk(KERN_INFO "IDBR: %04x\n", IDBR);

    // Step 6
    printk(KERN_INFO "===%s: step 6\n", __FUNCTION__);
    //while ((ISR & ISR_IRF) != 0x80) // wait for receive full interrupt
    printk(KERN_INFO "myi2c: ISR %04x\n", ISR);
    while ((ISR & ISR_ITE) != 0x40) // wait for transmit empty interrupt
    {
        printk(KERN_INFO "myi2c: ISR %04x, ISR & ISR_IRF %04x\n", ISR, (ISR & ISR_IRF));
        mdelay(500);
    }
    printk(KERN_INFO "IDBR: %04x\n", IDBR);

    // Step 7
    printk(KERN_INFO "===%s: step 7\n", __FUNCTION__);
    //ISR &= ~ISR_IRF;  // clear interrupt
    ISR &= ~ISR_ITE;  // clear interrupt
    printk(KERN_INFO "IDBR: %04x\n", IDBR);
    
    // Step 8
    printk(KERN_INFO "===%s: step 8\n", __FUNCTION__);
    *result = IDBR;
    printk(KERN_INFO "IDBR: %04x\n", IDBR);

    // Step 9
    printk(KERN_INFO "===%s: step 9\n", __FUNCTION__);
    ICR &= ~ICR_STOP;   // clear stop
    ICR &= ~ICR_ACKNAK; // clear ack/nak status
    printk(KERN_INFO "IDBR: %04x\n", IDBR);

    return 0;
}

/**
 *  \brief Initialize myi2c module
 *
 *  The kernel space function which corresponds to inserting
 *  the module to the kernel.
 *
 *  \return 0 for success, otherwise error
 */
static int myi2c_init(void)
{
    // Variables
    int result;

    // Registering device
    result = register_chrdev(myi2c_major, "myi2c", &myi2c_fops);
    if (result < 0)
    {
        printk(KERN_ALERT "myi2c: cannot obtain major number %d\n", myi2c_major);
    	return result;
    }

    // Allocate space for message buffer
    msgBuffer = kmalloc(MAX_MSG_LEN, GFP_KERNEL);
    if (!msgBuffer)
    {
        printk("myi2c: cannot allocate space for message buffer\n");
        return -ENOMEM;
    }

    // I2C Initialization
    pxa_gpio_mode(X_SDA);    // Set I2C mode for this pin
    pxa_gpio_mode(X_SCL);    // Set I2C mode for this pin

    pxa_set_cken(CKEN14_I2C, 1); // Clock enable

    // Disable Interrupts
    // ICR_BEIE - Detected bus error interrupt, no ACK sent
    // ICR_IRFIE - Receive buffer full interrupt
    // ICR_ITEIE - transmit buffer empty interrupt
    ICR &= ~(ICR_BEIE | ICR_IRFIE | ICR_ITEIE);

    // Enable things
    // ICR_GCD - Desiables i2c unit response to general call messages as a slave
    // ICR_SCLE - enables i2c clock output for master mode
    // ICR_IUE - disable unit until we change settings
    ICR |= ICR_GCD | ICR_SCLE | ICR_IUE;
    
    ICR &= ~ICR_TB;     // clear transfer byte bit

    printk(KERN_INFO "myi2c: module loaded.\n");

	return 0;
}

/**
 *  \brief De-intializes myi2c module
 *
 *  The kernal space function which corresponds to removing
 *  the module from the kernel.
 *
 */
static void myi2c_exit(void)
{
    // Freeing the major number
    unregister_chrdev(myi2c_major, "myi2c");
    
    // Free up GPIOs
    gpio_free(X_SDA);
    gpio_free(X_SCL);

    // Free up buffer memory
    if (msgBuffer)
    {
        kfree(msgBuffer);
    }

    printk(KERN_INFO "myi2c: module unloaded.\n");
}

/**
 *  \brief Open the myi2c device as a file
 *
 *  The Kernel Space function, which corresponds to opening the
 *  /dev/mygpio file in user space. This function is not
 *  required to do anything for this driver
 *
 *  \param *inode - info structure with major/minor numbers
 *  \param *filp  - info structure with available file operations
 *  \return 0 for success
 */
static int myi2c_open(struct inode *inode, struct file *filp)
{
	// Nothing to do here
	return 0;
}

/**
 *  \brief Release the myi2c device as a file
 *
 *  The Kernel Space function which corresponds to closing the
 *  /dev/mygpio file in user space. This function is not
 *  required to do anything for this driver
 *
 *  \param *inode - info structure with major/minor numbers
 *  \param *filp  - info structure with available file operations
 *  \return 0 for success
 */
static int myi2c_release(struct inode *inode, struct file *filp)
{
	// Nothing to do here
	return 0;
}

/**
 * TODO
 */
static int myi2c_ioctl(struct inode *inode, struct file *file,
		unsigned int cmd, unsigned long arg)
{
    uint8_t result = 0;

	switch ( cmd ) 
    {
	    case I2C_INIT:
            printk(KERN_INFO "myi2c: I2C_INIT with slave address %02x\n", (uint8_t) arg);
		    if (arg > 0x7f)
            {
			    return -EINVAL;
            }
            ICR &= ~ICR_TB;     // clear transfer byte bit
            // Set slave Address in ISAR
            printk(KERN_INFO "ISAR: %04x\n", ISAR);
            ISAR = (uint8_t) arg;
            printk(KERN_INFO "ISAR: %04x\n", ISAR);
            ISAR = 0x6a;
            printk(KERN_INFO "ISAR: %04x\n", ISAR);
            ISAR = (uint8_t) arg;
            printk(KERN_INFO "ISAR: %04x\n", ISAR);
            printk(KERN_INFO "IDBR: %04x\n", IDBR);
            printk(KERN_INFO "ICR : %04x\n", ICR);

            // Disable Interrupts
            // ICR_BEIE - Detected bus error interrupt, no ACK sent
            // ICR_IRFIE - Receive buffer full interrupt
            // ICR_ITEIE - transmit buffer empty interrupt
            ICR &= ~(ICR_BEIE | ICR_IRFIE | ICR_ITEIE);
            printk(KERN_INFO "ICR : %04x\n", ICR);

            // Enable things
            // ICR_GCD - Desiables i2c unit response to general call messages as a slave
            // ICR_SCLE - enables i2c clock output for master mode
            // ICR_IUE - disable unit unti we change settings
            ICR |= ICR_GCD | ICR_SCLE | ICR_IUE;
            printk(KERN_INFO "ICR : %04x\n", ICR);

            return 0;

        case I2C_W1B:
            printk(KERN_INFO "myi2c: I2C_W1B...\n");
            break;
        
        case I2C_R1B:
            printk(KERN_INFO "myi2c: I2C_R1B, reading %02x\n", (uint8_t) arg);

            i2c_read_1_byte((uint8_t) arg, &result);

            printk(KERN_INFO "myi2c: I2C_R1B, result  %02x\n", result);

            break;
        
        default:
            break;
    }

    return 0;
}
/**
 * \brief Read the myi2c device as a file
 *
 * The kernel space function which corresponds to reading the
 * /dev/mygpio file in user space
 *
 * \param *filp  - info structure with available file ops
 * \param *buf   - output string
 * \param count  - number of chars/bytes to read
 * \param *f_pos - position in file
 * \return number of bytes read
 */
static ssize_t myi2c_read(struct file *filp, char *buf,
                            size_t count, loff_t *f_pos)
{
    // Variables
    char tmpStr[MAX_MSG_LEN];
    char *secStr = "";

    // Clear the buffer
    memset(msgBuffer, 0, MAX_MSG_LEN);

    // End of buffer reached
    if (*f_pos >= msgBufferLen)
    {
        return 0;
    }

    // Copy to user space
    if (copy_to_user(buf, msgBuffer, msgBufferLen))
    {
        printk(KERN_ALERT "Fault in copying to user space\n");
        return -EFAULT;
    }

    *f_pos += msgBufferLen;
    return msgBufferLen;
}


/**
 *  \brief Write to the myi2c device as a file
 *
 *  The kernel space function which corresponds to writing to the
 *  /dev/mygpio file in user space.
 *
 *  \param *filp  - info structure with available file operations
 *  \param *buf   - input string from user space
 *  \param count  - num of chars/bytes to write
 *  \param *f_pos - position of file
 *  \return number of bytes written
 */
static ssize_t myi2c_write(struct file *filp, const char *buf,
							size_t count, loff_t *f_pos)
{
    // Variables
    char tmp[MAX_WRT_LEN];
    char *pInput;
    unsigned long num;

    // Get string from user space
    if (copy_from_user(tmp, buf, count))
    {
        return -EFAULT;
    }
    
    return count;
}
