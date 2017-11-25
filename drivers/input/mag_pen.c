#include <linux/module.h>
#include <net/sock.h>
#include <linux/netlink.h>
#include <linux/skbuff.h>

#include <linux/module.h>
#include <linux/ptrace.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/io.h>

//#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/hardware/iomd.h>

#define NETLINK_USER 30 // 31 is used data BSP team, so it is changed to 30

static struct input_dev *mag_pen_dev;

typedef struct virtual_touch{
    int x;
    int y;
    int z;
    int theta;
    int pi;
    int mode;
}v_touch;

static struct sock *nl_sk = NULL;

static void mag_pen_nl_recv_msg(struct sk_buff *skb) {

    struct nlmsghdr *nlh;
#if 0
    int pid;
    struct sk_buff *skb_out;
    int msg_size;
    char *msg="Hello from kernel";
    int res;
#endif
    v_touch *touch;

    nlh=(struct nlmsghdr*)skb->data;

    touch = (v_touch*) NLMSG_DATA(nlh);
    pr_debug("mag_pen x = %d, y = %d, z = %d theta = %d, pi = %d mode = %d \n", touch->x, touch->y, touch->z, touch->theta, touch->pi, touch->mode);

    if( (touch->x >= 0 && touch->x < 800 ) &&
	(touch->y >= 0 && touch->y < 1280) &&
	(touch->z >= -50 && touch->z <= 50)){
        input_report_key(mag_pen_dev, BTN_TOOL_PEN, 1);
        input_sync(mag_pen_dev);
        input_report_abs(mag_pen_dev, ABS_X, touch->x);
        input_report_abs(mag_pen_dev, ABS_Y, touch->y);
        input_report_abs(mag_pen_dev, ABS_PRESSURE, 1);
        input_sync(mag_pen_dev);
	}else{
        input_report_key(mag_pen_dev, BTN_TOOL_PEN, 0);
        input_sync(mag_pen_dev);
    }
}

static int __init mag_pen_init(void) {
    int err = 0;
    /* This is for 3.6 kernels and above.*/
    struct netlink_kernel_cfg cfg = {
           .input = mag_pen_nl_recv_msg,
    };

    pr_info("Entering: %s\n",__FUNCTION__);
    nl_sk = netlink_kernel_create(&init_net, NETLINK_USER, &cfg);
    if(!nl_sk)
    {
	pr_err( "Error creating socket. %s\n",__FUNCTION__);
	return -10;
    }

    mag_pen_dev = input_allocate_device();
    if( !mag_pen_dev ){
	pr_info("mag_pen.c: Not enough memory\n");
	return -ENOMEM;
    }

    mag_pen_dev->name = "Mag Pen";
    mag_pen_dev->phys = "mag_pen_dev/input0";
    mag_pen_dev->id.bustype = BUS_HOST;
    mag_pen_dev->id.vendor = 0x0005;
    mag_pen_dev->id.product = 0x0001;
    mag_pen_dev->id.version = 0x0100;
    mag_pen_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
    mag_pen_dev->keybit[BIT_WORD(BTN_TOOL_PEN)] |= BIT_MASK(BTN_TOOL_PEN);
    mag_pen_dev->keybit[BIT_WORD(BTN_TOUCH)] |= BIT_MASK(BTN_TOUCH);
    mag_pen_dev->propbit[BIT_WORD(INPUT_PROP_POINTER)] |= BIT_MASK(INPUT_PROP_POINTER);
    mag_pen_dev->propbit[BIT_WORD(INPUT_PROP_SEMI_MT)] |= BIT_MASK(INPUT_PROP_SEMI_MT);
    set_bit(BTN_STYLUS2, mag_pen_dev->keybit);
    set_bit(BTN_0, mag_pen_dev->keybit);

    input_set_abs_params(mag_pen_dev, ABS_X, 0, 800, 0, 0);
    input_set_abs_params(mag_pen_dev, ABS_Y, 0, 1280, 0, 0);
    input_set_abs_params(mag_pen_dev, ABS_PRESSURE, 0, 255, 0, 0);
    input_set_abs_params(mag_pen_dev, ABS_TILT_X, -128, 127, 0, 0);
    input_set_abs_params(mag_pen_dev, ABS_TILT_Y, -128, 127, 0, 0);
    input_set_abs_params(mag_pen_dev, ABS_DISTANCE, 0, 1024, 0, 0);

    err = input_register_device(mag_pen_dev);
    if( err ){
	pr_err("mag_pen.c: Failed to register device\n");
	goto err_free_dev;
    }

    return 0;

err_free_dev:
    input_free_device(mag_pen_dev);
    return 0;
}

static void __exit mag_pen_exit(void) {
    pr_info("exiting mag_pen module. %s\n",__FUNCTION__);
    input_unregister_device(mag_pen_dev);
    netlink_kernel_release(nl_sk);
}

module_init(mag_pen_init); 
module_exit(mag_pen_exit);

MODULE_LICENSE("GPL");
