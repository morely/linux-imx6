/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright 2005 Phil Blundell
 * Copyright 2010, 2011 David Jander <david@protonic.nl>
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
//#include <linux/gpio_dcmotors.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/pwm.h>
//#include <linux/math.h>
#include <linux/motors.h>
#define PI 314159
#define D   85
#define WHEEL_E5_LENTH  (PI*D)

/*(WHEEL_LENTH :100*D*PI;2*(80:1)*1000)*/
#define vSPEED_TO_tusSPEED_BY_MOTOR(x)    (WHEEL_E5_LENTH/(16*x))
#define tusSPEED_TO_vSPEED_BY_MOTOR(x)    vSPEED_TO_tusSPEED_BY_MOTOR(x)
#define SQRT(x)                         ((x)/100)
enum motor_pos{leftback,rightback,leftfront,rightfront};
enum motor_direction{forward,backward};
enum status{enable,disable};

struct gpio_dcmotors_motor {//platform motor regist
	struct pwm_device	*pwm;
	unsigned int gpio_dr;
	unsigned int gpio_capture_a;
	unsigned int gpio_capture_b;
	unsigned int irqca;
	unsigned int irqcb;
	unsigned int pwm_period_ns;
    unsigned int pwm_duty_cycle;
    unsigned int pwm_duty;
    unsigned int pwm_polarity;
    unsigned int code;
    unsigned int gpio_forwoard_default_level;
	const char *desc;
	enum motor_pos pos;
	long int counter_a;
	long int counter_b;
	unsigned int type;
    enum status status;
};

struct gpio_motor_data {//board motor info
	struct gpio_dcmotors_motor *pmotor;
	struct motor_classdev cdev;
    struct input_dev *input;
	struct timer_list timer;
	struct work_struct work;
	//unsigned int timer_debounce;	/* in msecs */
	unsigned int speed;
	enum motor_direction direction;//forward/backward
	unsigned int cur_speed;
	unsigned int cur_speed_t; //time val :hall circle
	unsigned int cur_circle_index;
	unsigned int circle_speed[5];
	spinlock_t lock;
	bool disabled;
	//bool key_pressed;
};

struct gpio_dcmotors_platform_data {
    const char *name;
    struct gpio_dcmotors_motor *motors;
    unsigned int nmotors;
};


struct gpio_dcmotors_drvdata {//board motors info
	const struct gpio_dcmotors_platform_data *pdata;
	struct input_dev *input;
	struct mutex disable_lock;
	struct gpio_motor_data data[0];
};

/*
 * SYSFS interface for enabling/disabling keys and switches:
 *
 * There are 4 attributes under /sys/devices/platform/gpio-keys/
 *	keys [ro]              - bitmap of keys (EV_KEY) which can be
 *	                         disabled
 *	switches [ro]          - bitmap of switches (EV_SW) which can be
 *	                         disabled
 *	disabled_keys [rw]     - bitmap of keys currently disabled
 *	disabled_switches [rw] - bitmap of switches currently disabled
 *
 * Userland can change these values and hence disable event generation
 * for each key (or switch). Disabling a key means its interrupt line
 * is disabled.
 *
 * For example, if we have following switches set up as gpio-keys:
 *	SW_DOCK = 5
 *	SW_CAMERA_LENS_COVER = 9
 *	SW_KEYPAD_SLIDE = 10
 *	SW_FRONT_PROXIMITY = 11
 * This is read from switches:
 *	11-9,5
 * Next we want to disable proximity (11) and dock (5), we write:
 *	11,5
 * to file disabled_switches. Now proximity and dock IRQs are disabled.
 * This can be verified by reading the file disabled_switches:
 *	11,5
 * If we now want to enable proximity (11) switch we write:
 *	5
 * to disabled_switches.
 *
 * We can disable only those keys which don't allow sharing the irq.
 */

/**
 * get_n_events_by_type() - returns maximum number of events per @type
 * @type: type of button (%EV_KEY, %EV_SW)
 *
 * Return value of this function can be used to allocate bitmap
 * large enough to hold all bits for given type.
 */
static inline int get_n_events_by_type(int type)
{
	BUG_ON(type != EV_SW && type != EV_KEY);

	return (type == EV_KEY) ? KEY_CNT : SW_CNT;
}

/**
 * gpio_dcmotors_disable_button() - disables given GPIO button
 * @bdata: button data for button to be disabled
 *
 * Disables button pointed by @bdata. This is done by masking
 * IRQ line. After this function is called, button won't generate
 * input events anymore. Note that one can only disable buttons
 * that don't share IRQs.
 *
 * Make sure that @bdata->disable_lock is locked when entering
 * this function to avoid races when concurrent threads are
 * disabling buttons at the same time.
 */
static void gpio_dcmotors_disable_motor(struct gpio_motor_data *bdata)
{
	if (!bdata->disabled) {
		/*
		 * Disable IRQ and possible debouncing timer.
		 */
		disable_irq(bdata->pmotor->irqca);
		disable_irq(bdata->pmotor->irqcb);
		pwm_disable(bdata->pmotor->pwm);
		//pwm
		//if (bdata->timer_debounce)
		//	del_timer_sync(&bdata->timer);

		bdata->disabled = true;
	}
}

/**
 * gpio_dcmotors_enable_button() - enables given GPIO button
 * @bdata: button data for button to be disabled
 *
 * Enables given button pointed by @bdata.
 *
 * Make sure that @bdata->disable_lock is locked when entering
 * this function to avoid races with concurrent threads trying
 * to enable the same button at the same time.
 */
static void gpio_dcmotors_enable_motor(struct gpio_motor_data *bdata)
{
	if (bdata->disabled) {
		enable_irq(bdata->pmotor->irqca);
		enable_irq(bdata->pmotor->irqca);
		pwm_enable(bdata->pmotor->pwm);
		bdata->pmotor->status = enable;
        bdata->disabled = false;
	}
}


static void gpio_dcmotors_gpio_report_event(struct gpio_motor_data *bdata)
{
	struct input_dev *input = bdata->input;
/*		unsigned int type = button->type ?: EV_KEY;
	int state = (gpio_get_value_cansleep(button->gpio) ? 1 : 0) ^ button->active_low;

	if (type == EV_ABS) {
		if (state)
			input_event(input, type, button->code, button->value);
	} else {
		input_event(input, type, button->code, !!state);
	}*/
	input_sync(input);
}

static void gpio_dcmotors_motor_speed_update(struct gpio_motor_data *bdata);
static void gpio_dcmotors_gpio_work_func(struct work_struct *work)
{
	struct gpio_motor_data *bdata =
		container_of(work, struct gpio_motor_data, work);

	gpio_dcmotors_gpio_report_event(bdata);
    gpio_dcmotors_motor_speed_update(bdata);
	//if (bdata->button->wakeup)
	//	pm_relax(bdata->input->dev.parent);
}

void gpio_dcmotors_set_motor_direction(struct gpio_motor_data *bdata,enum motor_direction direction);
static void gpio_dcmotors_motor_speed_update(struct gpio_motor_data *bdata){
    struct gpio_dcmotors_motor *pmotor = bdata->pmotor;
    unsigned int diff_speed,next_speed;

//for test:
    next_speed = bdata->speed;
    if((bdata->direction == forward) && (bdata->pmotor->pwm_polarity == 1))
        gpio_dcmotors_set_motor_direction(bdata,bdata->direction);
    if((bdata->direction == backward) && (bdata->pmotor->pwm_polarity == 0))
        gpio_dcmotors_set_motor_direction(bdata,bdata->direction);
    goto pwm_cal;

    if(bdata->cur_speed == bdata->speed) return;
//step to set motor
    diff_speed = abs(bdata->speed - bdata->cur_speed);
    if(diff_speed > 300)diff_speed =300;

    if(bdata->speed > bdata->cur_speed) next_speed = bdata->cur_speed + diff_speed;
        else next_speed = bdata->cur_speed - diff_speed;
pwm_cal:
    pmotor->pwm_duty = SQRT(10000*next_speed/500);
    if((pmotor->pwm_duty >0) & (pmotor->pwm_duty < 20)) pmotor->pwm_duty = 20;
    if(bdata->direction == backward)
        pmotor->pwm_duty = 100 - pmotor->pwm_duty;
    pmotor->pwm_duty_cycle = pmotor->pwm_duty * pmotor->pwm_period_ns /100;
//printk("duty:%d\n",pmotor->pwm_duty);

    pwm_disable(pmotor->pwm);
    pwm_config(pmotor->pwm,pmotor->pwm_duty_cycle, pmotor->pwm_period_ns);
    if(pmotor->status == enable)
        pwm_enable(pmotor->pwm);

//for test:
    pwm_enable(pmotor->pwm);
  return;
}

void gpio_dcmotors_set_motor_direction(struct gpio_motor_data *bdata,enum motor_direction direction){
	struct gpio_dcmotors_motor *motor = bdata->pmotor;
	unsigned int gpio_dcmotors_pwm_polarity,gpio_dr_value;

	if(direction == forward){
		gpio_dcmotors_pwm_polarity = 0;
		gpio_dr_value = 0;
	}
	else {
		gpio_dcmotors_pwm_polarity = 1;
		gpio_dr_value = 1;
	}

	pwm_disable(motor->pwm);
	pwm_set_polarity(motor->pwm, gpio_dcmotors_pwm_polarity);//set pwm polarity;
    motor->pwm_polarity = gpio_dcmotors_pwm_polarity;
udelay(1000);
    gpio_set_value(motor->gpio_dr, gpio_dr_value);//set dir;
	printk("Change Motor %s direction to %s \n",
			motor->desc,(direction == forward)? "forward":"backward");
if(motor->status == enable)
	pwm_enable(motor->pwm);
}

static void gpio_dcmotors_gpio_timer(unsigned long _data)
{
	struct gpio_motor_data *bdata = (struct gpio_motor_data *)_data;

	schedule_work(&bdata->work);
    mod_timer(&bdata->timer,jiffies + msecs_to_jiffies(30));
}

static irqreturn_t gpio_dcmotors_gpio_capture_a_isr(int irq, void *dev_id)
{
	struct gpio_motor_data *bdata = dev_id;
	struct gpio_dcmotors_motor *pmotor = bdata->pmotor;
    unsigned int i,j;
    struct timeval tv;
    unsigned int  cur_counter_a,cur_speed_t;
	//enum motor_direction direction = bdata->direction;
    unsigned int counter_a = bdata->pmotor->counter_a;
    unsigned int cur_circle_index = bdata->cur_circle_index;
    unsigned int *circle_speed = &bdata->circle_speed[0];

    BUG_ON(irq != bdata->pmotor->irqca);

    do_gettimeofday(&tv); //cal time val us
    cur_counter_a = tv.tv_usec;//cal time val us

/*/if motor  real direction != motor->direction, clean speed cal
    if(((gpio_get_value(pmotor->gpio_capture_b)) & 1)==0){// a falling, b = 0: motor back running;
		if(bdata->direction == forward)
            goto clean;
	}else {// in or used be in forward; cal motor speed
        if(bdata->direction == backward)
            goto clean;
    }
*/
	if(counter_a != 0){
	    if(cur_circle_index > 4){
		    cur_circle_index = 0;
	    }
        if((cur_counter_a - counter_a) > 0xc00)
            circle_speed[cur_circle_index++] = cur_counter_a - counter_a;
        else printk("ca:%d;cca:%d\ndf:%d\n\n",counter_a, cur_counter_a,  cur_counter_a - counter_a);
    }

    j = 0;
    cur_speed_t = 0;
	for(i = 0; i<5;i++){
	    if(bdata->circle_speed[i]){
		    j++;
		    cur_speed_t = cur_speed_t + bdata->circle_speed[i];
	    }
    }

    if(cur_speed_t)
	bdata->cur_speed_t = cur_speed_t;//10*w

    bdata->pmotor->counter_a = cur_counter_a;
    bdata->cur_circle_index = cur_circle_index;
    goto out;

clean:
    for(i = 0; i<5;i++)
        bdata->circle_speed[i] = 0;
	bdata->cur_speed_t = 0;
    bdata->pmotor->counter_a = 0;
    bdata->cur_circle_index = 0;
		//first try use gettimeofday cal motor speed; if not enough,try hrtimer;
out:
	//printk("inta: bdata->cur speed %d (10*w)!\n",bdata->cur_speed);

	return IRQ_HANDLED;
}

void set_speed_v2w(struct motor_classdev *motor_cdev, int speed){
    struct gpio_motor_data *bdata =
        container_of(motor_cdev, struct gpio_motor_data, cdev);
    bdata->direction = (speed >= 0) ?   forward :   backward;
    bdata->speed = abs(speed);
    printk("abs(bdata->speed):%d ;direction %s\n",bdata->speed,(bdata->direction == forward) ? "forward" : "backward");
};

int get_speed_w2v(struct motor_classdev *motor_cdev){
    int speed;
    struct gpio_motor_data *bdata =
    container_of(motor_cdev, struct gpio_motor_data, cdev);
    speed = bdata->cur_speed;
    speed = (bdata->direction == forward) ? speed : (0 - speed);
    printk("abs(speed):%d ;direction %s\n",speed,(bdata->direction == forward) ? "forward" : "backward");
    return speed;
};

static int gpio_dcmotors_setup_motor(struct platform_device *pdev,
				struct input_dev *input,
				struct gpio_motor_data *bdata,
				struct gpio_dcmotors_motor *motor)
{
    struct motor_classdev *motor_cdev = &bdata->cdev;
    const char *desc = motor->desc;
	char *desca = kzalloc(strlen(desc) +9 , GFP_KERNEL);
	struct device *dev = &pdev->dev;
	irq_handler_t isr;
	unsigned long irqflags;
	int irq, error;

    strcat(desca,desc);
    strcat(desca,":capturea");
	spin_lock_init(&bdata->lock);

    motor_cdev->name = desc;
    motor_cdev->speed_set = set_speed_v2w;
    motor_cdev->speed_get = get_speed_w2v;

    bdata->direction = forward;
    bdata->speed = 0;

	if (gpio_is_valid(motor->gpio_dr)) {

		error = gpio_request_one(motor->gpio_dr,
            motor->gpio_forwoard_default_level & OF_GPIO_ACTIVE_LOW ? GPIOF_OUT_INIT_LOW : GPIOF_OUT_INIT_HIGH, desc);
		if (error < 0) {
			dev_err(dev, "Failed to request GPIO-DR %d, error %d\n",
				motor->gpio_dr, error);
			return error;
		}
	}

	if (gpio_is_valid(motor->gpio_capture_a)) {

		error = gpio_request_one(motor->gpio_capture_a, GPIOF_IN, desca);
		if (error < 0) {
			dev_err(dev, "Failed to request GPIO-CAPTURE-A %d, error %d\n",
				motor->gpio_capture_a, error);
			return error;
		}

//		if (button->debounce_interval) {
//			error = gpio_set_debounce(button->gpio,
//					button->debounce_interval * 1000);
			/* use timer if gpiolib doesn't provide debounce */
//			if (error < 0)
//				bdata->timer_debounce =
//						button->debounce_interval;
//		}

		irq = gpio_to_irq(motor->gpio_capture_a);
		if (irq < 0) {
			error = irq;
			dev_err(dev,
				"Unable to get irq number for GPIO %d, error %d\n",
				motor->gpio_capture_a, error);
			goto fail;
		}
		motor->irqca = irq;

		setup_timer(&bdata->timer,
			    gpio_dcmotors_gpio_timer, (unsigned long)bdata);

		isr = gpio_dcmotors_gpio_capture_a_isr;
		irqflags = IRQF_TRIGGER_FALLING;


		error = request_any_context_irq(motor->irqca, isr, irqflags, desca, bdata);
		if (error < 0) {
			dev_err(dev, "Unable to claim irq %d; error %d\n",
			motor->irqca, error);
			goto fail;
		}

	} else {
			dev_err(dev, "No GPIO-CAPTURE-A specified\n");
			return -EINVAL;
    }

	if (gpio_is_valid(motor->gpio_capture_b)) {
		error = gpio_request_one(motor->gpio_capture_b, GPIOF_IN, desc);
		if (error < 0) {
			dev_err(dev, "Failed to request GPIO-CAPTURE-B %d, error %d\n",
				motor->gpio_capture_b, error);
			return error;
		}

/*		irq = gpio_to_irq(motor->gpio_capture_b);
		if (irq < 0) {
			error = irq;
			dev_err(dev,"Unable to get irq number for GPIO %d, error %d\n",
				motor->gpio_capture_b, error);
			goto fail;
		}
		motor->irqcb = irq;

		isr = gpio_dcmotors_gpio_capture_b_isr;
		irqflags = IRQF_TRIGGER_FALLING;

		error = request_any_context_irq(motor->irqcb, isr, irqflags, descb, bdata);
        if(error < 0) {
			dev_err(dev, "Unable to claim irq %d; error %d\n",
			motor->irqcb, error);
			goto fail;
		}
*/
	} else {
		dev_err(dev, "No GPIO-CAPTURE-B specified\n");
		return -EINVAL;
	}

	bdata->input = input;
	bdata->pmotor = motor;

//	input_set_capability(input, button->type ?: EV_KEY, button->code);
	INIT_WORK(&bdata->work, gpio_dcmotors_gpio_work_func);
	mod_timer(&bdata->timer,jiffies + msecs_to_jiffies(1000));
    return 0;

fail:
	if (gpio_is_valid(motor->gpio_dr))
		gpio_free(motor->gpio_dr);
	if (gpio_is_valid(motor->gpio_capture_a))
		gpio_free(motor->gpio_capture_a);
	if (gpio_is_valid(motor->gpio_capture_b))
		gpio_free(motor->gpio_capture_b);
	return error;
}

static void gpio_dcmotors_report_state(struct gpio_dcmotors_drvdata *ddata)
{
	struct input_dev *input = ddata->input;
	int i;

	for (i = 0; i < ddata->pdata->nmotors; i++) {
		struct gpio_motor_data *motor = &ddata->data[i];
		if (gpio_is_valid(motor->pmotor->gpio_capture_a) || gpio_is_valid(motor->pmotor->gpio_capture_b))
			gpio_dcmotors_gpio_report_event(motor);
	}
	input_sync(input);
}

static int gpio_dcmotors_open(struct input_dev *input)
{
	struct gpio_dcmotors_drvdata *ddata = input_get_drvdata(input);
//	const struct gpio_dcmotors_platform_data *pdata = ddata->pdata;
//	int error;

//	if (pdata->enable) {
//		error = pdata->enable(input->dev.parent);
//		if (error)
//			return error;
//	}

	/* Report current state of buttons that are connected to GPIOs */
	gpio_dcmotors_report_state(ddata);

	return 0;
}

static void gpio_dcmotors_close(struct input_dev *input)
{
	//struct gpio_dcmotors_drvdata *ddata = input_get_drvdata(input);
	//const struct gpio_dcmotors_platform_data *pdata = ddata->pdata;

//	if (pdata->disable)
//		pdata->disable(input->dev.parent);
}

/*
 * Handlers for alternative sources of platform_data
 */

#ifdef CONFIG_OF
/*
 * Translate OpenFirmware node properties into platform_data
 */
static struct gpio_dcmotors_platform_data *
gpio_dcmotors_get_devtree_pdata(struct device *dev)
{
	struct device_node *node, *pp;
	struct gpio_dcmotors_platform_data *pdata;
	struct gpio_dcmotors_motor *motor;
	int error = 0,nmotors,i;

	node = dev->of_node;
	if (!node) {
		error = -ENODEV;
		goto err_out;
	}

	nmotors = of_get_child_count(node);
	if (nmotors == 0) {
		error = -ENODEV;
		goto err_out;
	}

	pdata = kzalloc(sizeof(*pdata) + nmotors * (sizeof *motor),
			GFP_KERNEL);
	if (!pdata) {
		error = -ENOMEM;
		goto err_out;
	}

	pdata->motors = (struct gpio_dcmotors_motor *)(pdata + 1);
	pdata->nmotors = nmotors;

	//pdata->rep = !!of_get_property(node, "autorepeat", NULL);

	i = 0;
	for_each_child_of_node(node, pp) {
		int gpio_dr,gpio_capture_a,gpio_capture_b;
		enum of_gpio_flags flags;

		if (!of_find_property(pp, "gpios", NULL)) {
			dev_err(dev, "Found motor control gpios error!\n");
			goto err_free_pdata;
		}

		if(of_gpio_count(pp) < 3){
			dev_err(dev, "error: motor control gpios <3 !\n");
			goto err_free_pdata;

		}

		gpio_dr = of_get_gpio_flags(pp, 0, &flags);
		gpio_capture_a = of_get_gpio_flags(pp, 1,0);
		gpio_capture_b = of_get_gpio_flags(pp, 2, 0);

		error = (gpio_dr<0) ? gpio_dr: ((gpio_capture_a< 0) ? gpio_capture_a : gpio_capture_b) ;
		if (error < 0 ) {
			if (error != -EPROBE_DEFER)
				dev_err(dev,
					"Failed to get motor gpio flags, error: %d\n",
					error);
			goto err_free_pdata;
		}
		motor = &pdata->motors[i++];
		motor->gpio_dr = gpio_dr;
		motor->gpio_capture_a = gpio_capture_a;
		motor->gpio_capture_b = gpio_capture_b;

		motor->gpio_forwoard_default_level = flags & OF_GPIO_ACTIVE_LOW;
		motor->desc = of_get_property(pp, "label", NULL);

		if (of_property_read_u32(pp, "linux,code", &motor->code)) {
			dev_err(dev, "motor %s without keycode\n",
				motor->desc);
			error = -EINVAL;
			goto err_free_pdata;
		}

		if((unsigned char)motor->desc[5] == 'l'){
            switch((unsigned char)motor->desc[6]){
			    case 'b':
				    motor->pos = leftback;
				    break;
			    case 'f':
				    motor->pos = leftfront;
				    break;
                default:
				    dev_err(dev, "motor positon configure error : %s can not be defined\n",
				    motor->desc);
                goto err_free_pdata;
                }
        }else if((unsigned char)motor->desc[5] == 'r'){
            switch((unsigned char)motor->desc[6]){
			    case 'b':
				    motor->pos = rightback;
				    break;
			    case 'f':
				    motor->pos = rightfront;
				    break;
                default:
				    dev_err(dev, "motor positon configure error : %s can not be defined\n",
				    motor->desc);
                goto err_free_pdata;
            }
        }else{
                goto err_free_pdata;
        }

		if (of_property_read_u32(pp, "linux,input-type", &motor->type))
			motor->type = EV_KEY;

		if (of_property_read_u32(pp, "pwm-duty",
					&motor->pwm_duty))
			motor->pwm_duty = 70;


		motor->pwm = of_pwm_get(pp, NULL);
		if (IS_ERR(motor->pwm)) {
			dev_err(dev, "**%s:unable to request PWM, trying legacy API\n",__FUNCTION__);
			goto err_free_pdata;
		}else {
            struct pwm_device **ptr = NULL;
            //ptr = devres_alloc(devm_pwm_release, sizeof(*ptr), GFP_KERNEL);
            if(ptr){
                *ptr = motor->pwm;
                devres_add(dev, ptr);
            }
        }
		motor->pwm_period_ns = pwm_get_period(motor->pwm);
		motor->pwm_polarity  = motor->gpio_forwoard_default_level;
        motor->status = disable;

        pwm_set_polarity(motor->pwm,motor->gpio_forwoard_default_level);
		motor->pwm_duty_cycle = motor->pwm_duty * motor->pwm_period_ns /100;
		pwm_config(motor->pwm,motor->pwm_duty_cycle, motor->pwm_period_ns);
	}


	return pdata;

err_free_pdata:
	kfree(pdata);
err_out:
	return ERR_PTR(error);
}

static struct of_device_id gpio_dcmotors_of_match[] = {
	{ .compatible = "gpio-dcmotors", },
	{ },
};
MODULE_DEVICE_TABLE(of, gpio_dcmotors_of_match);

#else

static inline struct gpio_dcmotors_platform_data *
gpio_dcmotors_get_devtree_pdata(struct device *dev)
{
	return ERR_PTR(-ENODEV);
}

#endif

static void gpio_remove_motor(struct gpio_motor_data *data)
{
    struct gpio_dcmotors_motor *motor = (struct gpio_dcmotors_motor *)data->pmotor;
	free_irq(motor->irqca, motor);
	free_irq(motor->irqcb, motor);
//	if (bdata->timer_debounce)
//		del_timer_sync(&bdata->timer);
//	cancel_work_sync(&bdata->work);
    if (gpio_is_valid(motor->gpio_dr))
        gpio_free(motor->gpio_dr);
    if (gpio_is_valid(motor->gpio_capture_a))
       gpio_free(motor->gpio_capture_a);
    if (gpio_is_valid(motor->gpio_capture_b))
       gpio_free(motor->gpio_capture_b);
}

static int gpio_dcmotors_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct gpio_dcmotors_platform_data *pdata = dev_get_platdata(dev);
	struct gpio_dcmotors_drvdata *ddata;
	struct input_dev *input;
	int i, error;

	if (!pdata) {
		pdata = gpio_dcmotors_get_devtree_pdata(dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}

	ddata = kzalloc(sizeof(struct gpio_dcmotors_drvdata) +
			pdata->nmotors * sizeof(struct gpio_motor_data),
			GFP_KERNEL);
	input = input_allocate_device();
	if (!ddata || !input) {
		dev_err(dev, "failed to allocate state\n");
		error = -ENOMEM;
		goto fail1;
	}

	ddata->pdata = pdata;
	ddata->input = input;
	mutex_init(&ddata->disable_lock);

	platform_set_drvdata(pdev, ddata);
	input_set_drvdata(input, ddata);

	input->name = pdata->name ? pdata->name : pdev->name;
	input->phys = "gpio-dcmotors/speed0";
	input->dev.parent = &pdev->dev;
	input->open = gpio_dcmotors_open;
	input->close = gpio_dcmotors_close;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0101;
	input->id.product = 0x0100;
	input->id.version = 0x0000;

	for (i = 0; i < pdata->nmotors; i++) {
		struct gpio_dcmotors_motor *pmotor = &pdata->motors[i];
		struct gpio_motor_data *bdata = &ddata->data[i];
		error = gpio_dcmotors_setup_motor(pdev, input, bdata, pmotor);
		if (error)
			goto fail2;
		error = motor_classdev_register(&pdev->dev,&bdata->cdev);
		if (error != 0)
			goto fail2;
        //pwm_enable(pmotor->pwm);
	}

	error = input_register_device(input);
	if (error) {
		dev_err(dev, "Unable to register input device, error: %d\n",
			error);
		goto fail2;
	}

	//device_init_wakeup(&pdev->dev, wakeup);

	return 0;

 fail2:
	while (--i >= 0)
		gpio_remove_motor(&ddata->data[i]);

	platform_set_drvdata(pdev, NULL);
 fail1:
	input_free_device(input);
	kfree(ddata);
	/* If we have no platform data, we allocated pdata dynamically. */
	if (!dev_get_platdata(&pdev->dev))
		kfree(pdata);

	return error;
}

static int gpio_dcmotors_remove(struct platform_device *pdev)
{
	struct gpio_dcmotors_drvdata *ddata = platform_get_drvdata(pdev);
    struct input_dev *input = ddata->input;
	int i;

//	sysfs_remove_group(&pdev->dev.kobj, &gpio_dcmotors_attr_group);

	device_init_wakeup(&pdev->dev, 0);

	for (i = 0; i < ddata->pdata->nmotors; i++)
		gpio_remove_motor(&ddata->data[i]);

	input_unregister_device(input);

	/* If we have no platform data, we allocated pdata dynamically. */
	if (!dev_get_platdata(&pdev->dev))
		kfree(ddata->pdata);

	kfree(ddata);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int gpio_dcmotors_suspend(struct device *dev)
{
/*	struct gpio_dcmotors_drvdata *ddata = dev_get_drvdata(dev);
	struct input_dev *input = ddata->input;
	int i;

	if (device_may_wakeup(dev)) {
		for (i = 0; i < ddata->pdata->nbuttons; i++) {
			struct gpio_motor_data *bdata = &ddata->data[i];
			if (bdata->button->wakeup)
				enable_irq_wake(bdata->irq);
		}
	} else {
		mutex_lock(&input->mutex);
		if (input->users)
			gpio_dcmotors_close(input);
		mutex_unlock(&input->mutex);
	}*/

	return 0;
}

static int gpio_dcmotors_resume(struct device *dev)
{
/*	struct gpio_dcmotors_drvdata *ddata = dev_get_drvdata(dev);
	struct input_dev *input = ddata->input;
	int error = 0;
	int i;

	if (device_may_wakeup(dev)) {
		for (i = 0; i < ddata->pdata->nbuttons; i++) {
			struct gpio_motor_data *bdata = &ddata->data[i];
			if (bdata->button->wakeup)
				disable_irq_wake(bdata->irq);
		}
	} else {
		mutex_lock(&input->mutex);
		if (input->users)
			error = gpio_dcmotors_open(input);
		mutex_unlock(&input->mutex);
	}

	if (error)
		return error;

	gpio_dcmotors_report_state(ddata);*/
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(gpio_dcmotors_pm_ops, gpio_dcmotors_suspend, gpio_dcmotors_resume);

static struct platform_driver gpio_dcmotors_device_driver = {
	.probe		= gpio_dcmotors_probe,
	.remove		= gpio_dcmotors_remove,
	.driver		= {
		.name	= "gpio-dcmotor",
		.owner	= THIS_MODULE,
		.pm	= &gpio_dcmotors_pm_ops,
		.of_match_table = of_match_ptr(gpio_dcmotors_of_match),
	}
};

static int __init gpio_dcmotors_init(void)
{
	return platform_driver_register(&gpio_dcmotors_device_driver);
}

static void __exit gpio_dcmotors_exit(void)
{
	platform_driver_unregister(&gpio_dcmotors_device_driver);
}

late_initcall(gpio_dcmotors_init);
module_exit(gpio_dcmotors_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Phil Blundell <pb@handhelds.org>");
MODULE_DESCRIPTION("Keyboard driver for GPIOs");
MODULE_ALIAS("platform:gpio-dcmotors");
