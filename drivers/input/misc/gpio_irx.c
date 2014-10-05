/*
 * Driver for ir on GPIO lines capable of generating interrupts.
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
//#include <linux/gpio_irx.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/spinlock.h>
#include <linux/time.h>

#define KEY_VENTER      28
static unsigned int hs0038b_keycode_t[]={
	139/*MENU*/,105/*LEFT*/,KEY_VENTER/*DPAD_CENTER*/,/*KEY_9*/4,
	103/*UP*/,  158/*HOME*/,108/*DOWN*/, /*KEY_6*/12,
	102/*BACK*/, 106/*RIGHT*/ ,KEY_3, KEY_BACKSLASH,
	KEY_1,KEY_2,KEY_4,
	KEY_5,KEY_6,KEY_7,KEY_8,
	KEY_9,KEY_0,231,/*CALL*/
	15,	/*TAB*/		57,	/*SPACE*/	KEY_0,
	107,	/*ENDCALL*/	14,	/*DEL*/		231,	/*CALL*/
	106,/*RIGHT*/	116,/*POWER*/	108,/*DOWN*/	212,	/*CAMERA*/
	165,/*M_PREV*/	200,/*M_PLAY*/	163,	/*M_NEXT*/
	166,/*M_STOP*/	115,/*VOLUME+*/	114,	/*VOLUME-*/
};

struct gpio_irx_platform_data {
	struct gpio_irx_data *irx_data;
    const char *name;

	unsigned int gpio;
	unsigned int pulse_width;
	unsigned int irq;
	unsigned long irqflags;

    int type;
	unsigned int keycodemax;
	unsigned int *keycode_t;

	void (*keydata_init)(struct gpio_irx_data *irx_data);
	unsigned int (*report_event)(struct gpio_irx_data *irx_data);
	unsigned int (*get_keycode)(struct gpio_irx_data *irx_data, unsigned int gpio_value);
};

struct gpio_irx_data {
	const struct gpio_irx_platform_data *pdata;
    const char *desc;
	struct mutex lock;
	struct input_dev *input;
	struct timer_list timer;
	struct work_struct work;

	unsigned int gpio;
    int type;
	unsigned int pulse_width;
	unsigned int irq;
	unsigned long irqflags;

	unsigned rx_started;
	bool timerout;

	void *keydata;
	void *keydata_cached;
	unsigned int *keycode_t;

	void (*keydata_init)(struct gpio_irx_data *irx_data);
	unsigned int (*report_event)(struct gpio_irx_data *irx_data);
	unsigned int (*get_keycode)(struct gpio_irx_data *irx_data, unsigned int gpio_value);
};

struct keydata{
	unsigned int bitnum,bytenum;
    unsigned int dataarry[4];
};

void hs0038b_keydata_init(struct gpio_irx_data *irx_data){
    struct keydata *kdata = kzalloc(sizeof(struct keydata) * 2, GFP_KERNEL);
    if( !kdata)
        printk("keydata init erro!");
    irx_data->keydata = kdata;
    irx_data->keydata_cached = (struct keydata *)(kdata +1);
    kdata->bitnum = 0x00;
    kdata->bytenum = 0x00;
    kdata->dataarry[0] =0x00;
    kdata =  (struct keydata *)(kdata +1);
    kdata->bitnum = 0x00;
    kdata->bytenum = 0x00;
    kdata->dataarry[0] =0x00;
}

static unsigned int hs0038b_report_event(struct gpio_irx_data *irx_data){
	struct keydata *kdata = irx_data->keydata_cached;
	unsigned int keycode,keycoden;
    //unsigned int i;
    //for(i =0; i <4; i++)printk("kdata->dataarry[%d]:%x \n",i,kdata->dataarry[i]);
    if(kdata->dataarry[0] == 0x00 && kdata->dataarry[1] == 0xff)
    {
        keycode =  kdata->dataarry[2];
        keycoden =  keycode & kdata->dataarry[3];
        if(keycoden == 0)
        {
            //keycode =  irx_data->keycode_t[keycode];
	        input_report_key(irx_data->input,keycode, 1);
	        input_report_key(irx_data->input,keycode, 0);
	        input_sync(irx_data->input);
            printk("report key:%d",keycode);
        }
    }
return 0;
}

static unsigned int hs0038b_get_keycode(struct gpio_irx_data *irx_data, unsigned int gpio_value){
	struct keydata *kdata = (struct keydata *) irx_data->keydata;
	struct keydata *kdata_cached = (struct keydata *) irx_data->keydata_cached;
	unsigned int bytenum = kdata->bytenum;
	unsigned int bitnum = kdata->bitnum;
	unsigned int keycode = kdata->dataarry[bytenum];
    if((bitnum ==0) && (bytenum == 0) && gpio_value)
    {
        schedule_work(&irx_data->work);
        return 0;/* read ir fimished*/
    }
    keycode = keycode << 1;
    keycode = keycode | gpio_value;
    kdata->dataarry[bytenum]= keycode;

    if(bytenum >2 && bitnum > 6)
    {
        kdata->bitnum = 0;
        kdata->bytenum = 0;
        irx_data->keydata = kdata_cached;
        irx_data->keydata_cached = kdata;
        schedule_work(&irx_data->work);
        return 0;/* read ir fimished*/
    }

    if(bitnum >6)
    {
        bytenum++;
        bitnum = 0;
        kdata->dataarry[bytenum]= 0x00;/*clear cached data*/
    }else bitnum++;

    kdata->bitnum =bitnum;
    kdata->bytenum = bytenum;

    return 1;/*contine read ir data*/
}

static void gpio_irx_keys_work_func(struct work_struct *work)
{
	struct gpio_irx_data *irx_data =
		container_of(work, struct gpio_irx_data, work);
	irx_data->report_event(irx_data);
}

static void gpio_irx_gpio_timer(unsigned long _data)
{
//	struct gpio_irx_data *irx_data = (struct gpio_irx_data *)_data;
    //unsigned int  irq = irx_data->irq;
	//enum of_gpio_flags irqflags = irx_data->irqflags;
	/*if(!irx_data->get_keycode(irx_data))
	{
		//mutex_lock(&irx_data->lock);
		irx_data->rx_started = false;
		irx_data->timerout = true;
		//mutex_unlock(&irx_data->lock);
		schedule_work(&irx_data->work);
	}
	irx_data->timerout = true;*/
    printk("IN timer! jiffies: 0x%x\n",(unsigned int )jiffies);
/*
    disable_irq_nosync(irq);
	irx_data->rx_started = 0;
    irqflags = IRQF_TRIGGER_FALLING;
	irx_data->irqflags =irqflags;
	irq_set_irq_type(irq, irqflags);
	enable_irq(irq);*/
}

static irqreturn_t gpio_irx_gpio_isr(int irq, void *dev_id)
{
	struct gpio_irx_data *irx_data = dev_id;
	enum of_gpio_flags irqflags = irx_data->irqflags;
    unsigned int counter = 0, gpio_value;
	BUG_ON(irq != irx_data->irq);
	disable_irq_nosync(irq);

    counter =0 ;
    gpio_direction_input(irx_data->gpio);
    gpio_value = gpio_get_value(irx_data->gpio) ? 1 :0;

    if(irx_data->rx_started == 3){
        do
        {
            counter++;
            udelay(5);
            gpio_value = gpio_get_value(irx_data->gpio) ? 1 :0;
        }while(gpio_value && (counter < 2000));
        if(counter > 120)
            gpio_value = 0x01;
        else gpio_value =0x0;
        //printk("%d",gpio_value);
        if(irx_data->get_keycode(irx_data,gpio_value))
            goto  out;/*continue read */
        goto fail;/*finshed read data*/
    }
	if(irx_data->rx_started == 2)
	{
            if(gpio_value == 1)
            goto fail;
            irx_data->rx_started = 3;
            irqflags = IRQF_TRIGGER_RISING;
            goto out;
	}else{
        if(irx_data->rx_started == 1)
        {
            if(gpio_value == 0)
                goto fail;
            do
            {
                counter++;
                udelay(5);
                gpio_value = gpio_get_value(irx_data->gpio) ? 1 :0;
            }while(gpio_value && (counter < 400));
            if(counter < 400 )
                goto fail;
            irx_data->rx_started = 2;
            irqflags = IRQF_TRIGGER_FALLING;
            goto out;
        }else if(irx_data->rx_started == 0)
                {
                    if(gpio_value == 1)
                    goto fail;
                    do{
                        counter++;
                        udelay(5);
                        gpio_value = gpio_get_value(irx_data->gpio) ? 1 :0;
                    }while((gpio_value == 0)  && (counter < 1200));
                    if(counter < 1200)
                        goto fail;
			        irx_data->rx_started = 1;
			        irqflags = IRQF_TRIGGER_RISING;
		            goto out;
                }
	}


fail:
    //mdelay(100);
    irx_data->rx_started = 0;
    irqflags = IRQF_TRIGGER_FALLING;
out:
	irx_data->irqflags =irqflags;	//gpio_ir_read(ir_data);
	irq_set_irq_type(irq, irqflags);
	enable_irq(irq);

	return IRQ_HANDLED;
}



static int gpio_irx_setup_gpio(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gpio_irx_data *irx_data = platform_get_drvdata(pdev);
	const char *desc = irx_data->desc ? irx_data->desc : "gpio_irx";

	irq_handler_t isr;
	unsigned long irqflags;
	int irq, error;

	if (gpio_is_valid(irx_data->gpio)) {

		error = gpio_request_one(irx_data->gpio, GPIOF_IN, desc);
		if (error < 0) {
			dev_err(dev, "Failed to request GPIO %d, error %d\n",
				irx_data->gpio, error);
			return error;
        }

        irq = gpio_to_irq(irx_data->gpio);
	    if (irq < 0) {
			error = irq;
			dev_err(dev,
				"Unable to get irq number for GPIO %d, error %d\n",
				irx_data->gpio, error);
			goto fail;
	    }

		irx_data->irq=irq;
		isr	= gpio_irx_gpio_isr;
		irqflags = IRQF_TRIGGER_FALLING;

		INIT_WORK(&irx_data->work, gpio_irx_keys_work_func);
        setup_timer(&irx_data->timer, gpio_irx_gpio_timer, (unsigned long)irx_data);
		error = request_any_context_irq(irx_data->irq, isr, irqflags, desc, irx_data);
		//error = request_irq(irq, ir_gpio_interrupt, irqflags, desc, ir_data);

		mod_timer(&irx_data->timer,jiffies+msecs_to_jiffies(1000));
		if (error) {
		printk("Unable to claim irq %d; error %d\n",irx_data->irq, error);
		goto fail;
		}

		return 0;
	}

fail:
	if (gpio_is_valid(irx_data->gpio))
		gpio_free(irx_data->gpio);

	return error;
}

#ifdef CONFIG_OF
/*
 * Translate OpenFirmware node properties into platform_data
 */
static struct gpio_irx_platform_data *
gpio_irx_get_devtree_pdata(struct device *dev)
{
	struct device_node *node;
	struct gpio_irx_platform_data *pdata;
	int error, gpio;
	enum of_gpio_flags flags;

	node = dev->of_node;
	if (!node) {
		error = -ENODEV;
		goto err_out;
	}

	pdata = kzalloc(sizeof(*pdata),GFP_KERNEL);

	if (!pdata) {
		error = -ENOMEM;
		goto err_out;
	}

	if( of_property_read_u32(node, "pulse_width", &pdata->pulse_width) ){
	    dev_err(dev,"read pulse_width erro!\n");
		goto err_out;
    }

	if (!of_find_property(node, "gpios", NULL)) {
			dev_warn(dev, "read gpios erro\n");
	}

	gpio = of_get_gpio_flags(node, 0, &flags);
	if (gpio < 0) {
		error = gpio;
		if (error != -EPROBE_DEFER)
			dev_err(dev,"Failed to get gpio flags, error: %d\n",
					error);
		goto err_free_pdata;
	}
	pdata->gpio = gpio;
    pdata->irqflags = flags;
	if (of_property_read_u32(node, "linux,input-type", &pdata->type))
		pdata->type = EV_KEY;

	pdata->keydata_init = hs0038b_keydata_init;
	pdata->report_event = hs0038b_report_event;
	pdata->get_keycode = hs0038b_get_keycode;
	pdata->keycode_t = &hs0038b_keycode_t;
	pdata->keycodemax = 37;
	return pdata;

err_free_pdata:
	kfree(pdata);
err_out:
	return ERR_PTR(error);
}

static struct of_device_id gpio_irx_of_match[] = {
	{ .compatible = "gpio-irx", },
	{ },
};
MODULE_DEVICE_TABLE(of, gpio_irx_of_match);

#else

static inline struct gpio_irx_platform_data *
gpio_irx_get_devtree_pdata(struct device *dev)
{
	return ERR_PTR(-ENODEV);
}

#endif

static void gpio_irx_remove_gpio(struct gpio_irx_data *bdata)
{
	free_irq(bdata->irq, bdata);
	if (bdata->pulse_width)
		del_timer_sync(&bdata->timer);
	cancel_work_sync(&bdata->work);
	if (gpio_is_valid(bdata->gpio))
		gpio_free(bdata->gpio);
}

static int gpio_irx_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct gpio_irx_platform_data *pdata = dev_get_platdata(dev);
	struct gpio_irx_data *irx_data;
//	struct irx_reciever *reciv;
//	struct irx_stick	*stick;
	struct input_dev *input;
	int i, error;
//	int wakeup = 0;

	if (!pdata) {
		pdata = gpio_irx_get_devtree_pdata(dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}

	irx_data = kzalloc(sizeof(struct gpio_irx_data) + pdata->keycodemax*sizeof(unsigned int),
			GFP_KERNEL);

	input = input_allocate_device();
	if (!irx_data || !input) {
		dev_err(dev, "failed to allocate state\n");
		error = -ENOMEM;
		goto fail1;
	}
	//pdata->irx_data = irx_data;
    irx_data->desc = "gpio_irx";
	irx_data->pdata = pdata;
	irx_data->input = input;
	irx_data->gpio = pdata->gpio;
    irx_data->irqflags = pdata->irqflags;
	irx_data->type = pdata->type;
	irx_data->pulse_width = pdata-> pulse_width;
	irx_data->keycode_t = (unsigned int *)(irx_data + 1);
	mutex_init(&irx_data->lock);
	irx_data->keydata_init = pdata->keydata_init;
	irx_data->report_event = pdata->report_event;
	irx_data->get_keycode = pdata->get_keycode;

    irx_data->keydata_init(irx_data);
	platform_set_drvdata(pdev, irx_data);
	input_set_drvdata(input, irx_data);

	input->name = pdata->name ? : pdev->name;
	input->phys = "gpio-irx/input0";
	input->dev.parent = &pdev->dev;
	//input->open = gpio_ir_open;
	//input->close = gpio_ir_close;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0xabcd;
	input->id.product = 0xecba;
	input->id.version = 0x0100;

	input->evbit[0] = BIT(EV_KEY);
	input->keycode = pdata->keycode_t;
	input->keycodesize = sizeof(unsigned int);
	input->keycodemax = pdata->keycodemax;
	__set_bit(EV_KEY, input->evbit);

	for (i = 0; i < pdata->keycodemax; i++) {
		irx_data->keycode_t[i]= pdata->keycode_t[i];
		set_bit(irx_data->keycode_t[i], input->keybit);
	}

	error = gpio_irx_setup_gpio(pdev);
	if (error) {
		dev_err(dev, "Unable to setup irx gpio, error: %d\n ", error);
		goto fail2;
	}

	error = input_register_device(input);
	if (error) {
		dev_err(dev, "Unable to register IR input device, error: %d\n",
			error);
		goto fail3;
	}

	return 0;

 fail3:
	input_free_device(input);
 fail2:
	gpio_irx_remove_gpio(irx_data);

	platform_set_drvdata(pdev, NULL);
 fail1:

	kfree(irx_data);
	/* If we have no platform data, we allocated pdata dynamically. */
	if (!dev_get_platdata(&pdev->dev))
		kfree(pdata);

	return error;
}

static int gpio_irx_remove(struct platform_device *pdev)
{
	struct gpio_irx_data *ddata = platform_get_drvdata(pdev);
	struct input_dev *input = ddata->input;
//	int i;

	input_unregister_device(input);

	/* If we have no platform data, we allocated pdata dynamically. */
	if (!dev_get_platdata(&pdev->dev))
		kfree(ddata->pdata);

	kfree(ddata);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int gpio_irx_suspend(struct device *dev)
{
//	struct gpio_irx_data *ddata = dev_get_drvdata(dev);
//	struct input_dev *input = ddata->input;

	return 0;
}

static int gpio_irx_resume(struct device *dev)
{
	struct gpio_irx_data *ddata = dev_get_drvdata(dev);
	struct input_dev *input = ddata->input;
	int error = 0;
	disable_irq_wake(ddata->irq);

	mutex_lock(&input->mutex);
	if (input->users)
		error = 0;
	mutex_unlock(&input->mutex);

	if (error)
		return error;

//	gpio_irx_report_state(ddata);
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(gpio_irx_pm_ops, gpio_irx_suspend, gpio_irx_resume);

static struct platform_driver gpio_irx_device_driver = {
	.probe		= gpio_irx_probe,
	.remove		= gpio_irx_remove,
	.driver		= {
		.name	= "gpio-irx",
		.owner	= THIS_MODULE,
		.pm	= &gpio_irx_pm_ops,
		.of_match_table = of_match_ptr(gpio_irx_of_match),
	}
};

static int __init gpio_irx_init(void)
{
	return platform_driver_register(&gpio_irx_device_driver);
}

static void __exit gpio_irx_exit(void)
{
	platform_driver_unregister(&gpio_irx_device_driver);
}

late_initcall(gpio_irx_init);
module_exit(gpio_irx_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Phil Blundell <pb@handhelds.org>");
MODULE_DESCRIPTION("Keyboard driver for GPIOs");
MODULE_ALIAS("platform:gpio-irx");
