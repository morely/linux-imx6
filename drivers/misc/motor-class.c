/*
 * MOTOR Class Core
 *
 * Copyright (C) 2005 John Lenz <lenz@cs.wisc.edu>
 * Copyright (C) 2005-2007 Richard Purdie <rpurdie@openedhand.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/timer.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/motors.h>
//#include "motors.h"
#define MAX_SPEED 1000
struct class *motors_class;

DECLARE_RWSEM(motors_list_lock);
EXPORT_SYMBOL_GPL(motors_list_lock);

LIST_HEAD(motors_list);
EXPORT_SYMBOL_GPL(motors_list);

/*
**show motor current speed: + forward / - backward
*/
static ssize_t motor_speed_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct motor_classdev *motor_cdev = dev_get_drvdata(dev);

	/* no lock needed for this */
	if (motor_cdev->speed_get)
		motor_cdev->speed = motor_cdev->speed_get(motor_cdev);

	return sprintf(buf, "%d\n", motor_cdev->speed);
}

/*
**set motor current speed: + forward / - backward
*/
static ssize_t motor_speed_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct motor_classdev *motor_cdev = dev_get_drvdata(dev);

	unsigned long speed;
    int  speed_int;
	ssize_t ret = -EINVAL;
    if(*buf == '-')
	    ret = kstrtoul(buf + 1, 10, &speed);
    else
	    ret = kstrtoul(buf, 10, &speed);
	if (ret)
        return ret;
    speed_int = (int)speed;
	if(speed > motor_cdev->max_speed)
        speed_int = motor_cdev->max_speed;
    if(*buf == '-')
        speed_int = 0 - speed_int;
	motor_cdev->speed = speed_int;
	motor_cdev->speed_set(motor_cdev, motor_cdev->speed);

	return size;
}

static ssize_t motor_max_speed_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct motor_classdev *motor_cdev = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", motor_cdev->max_speed);
}

static struct device_attribute motor_class_attrs[] = {
	__ATTR(speed, 0644, motor_speed_show, motor_speed_store),
	__ATTR(max_speed, 0444, motor_max_speed_show, NULL),
	__ATTR_NULL,
};


/**
 * motor_classdev_suspend - suspend an motor_classdev.
 * @motor_cdev: the motor_classdev to suspend.
 */
void motor_classdev_suspend(struct motor_classdev *motor_cdev)
{
	motor_cdev->flags |= motor_SUSPENDED;
	motor_cdev->speed_set(motor_cdev, 0);
}
EXPORT_SYMBOL_GPL(motor_classdev_suspend);

/**
 * motor_classdev_resume - resume an motor_classdev.
 * @motor_cdev: the motor_classdev to resume.
 */
void motor_classdev_resume(struct motor_classdev *motor_cdev)
{
	motor_cdev->speed_set(motor_cdev, motor_cdev->speed);
	motor_cdev->flags &= ~motor_SUSPENDED;
}
EXPORT_SYMBOL_GPL(motor_classdev_resume);

static int motor_suspend(struct device *dev, pm_message_t state)
{
	struct motor_classdev *motor_cdev = dev_get_drvdata(dev);

	if (motor_cdev->flags & motor_CORE_SUSPENDRESUME)
		motor_classdev_suspend(motor_cdev);

	return 0;
}

static int motor_resume(struct device *dev)
{
	struct motor_classdev *motor_cdev = dev_get_drvdata(dev);

	if (motor_cdev->flags & motor_CORE_SUSPENDRESUME)
		motor_classdev_resume(motor_cdev);

	return 0;
}

/**
 * motor_classdev_register - register a new object of motor_classdev class.
 * @parent: The device to register.
 * @motor_cdev: the motor_classdev structure for this device.
 */
int motor_classdev_register(struct device *parent, struct motor_classdev *motor_cdev)
{
	motor_cdev->dev = device_create(motors_class, parent, 0, motor_cdev,
				      "%s", motor_cdev->name);
	if (IS_ERR(motor_cdev->dev))
		return PTR_ERR(motor_cdev->dev);

	/* add to the list of motors */
	down_write(&motors_list_lock);
	list_add_tail(&motor_cdev->node, &motors_list);
	up_write(&motors_list_lock);

	if (!motor_cdev->max_speed)
		motor_cdev->max_speed = MAX_SPEED;

	dev_dbg(parent, "Registered motor device: %s\n",
			motor_cdev->name);

	return 0;
}
EXPORT_SYMBOL_GPL(motor_classdev_register);

/**
 * motor_classdev_unregister - unregisters a object of motor_properties class.
 * @motor_cdev: the motor device to unregister
 *
 * Unregisters a previously registered via motor_classdev_register object.
 */
void motor_classdev_unregister(struct motor_classdev *motor_cdev)
{

	motor_cdev->speed_set(motor_cdev, 0);

	device_unregister(motor_cdev->dev);

	down_write(&motors_list_lock);
	list_del(&motor_cdev->node);
	up_write(&motors_list_lock);
}
EXPORT_SYMBOL_GPL(motor_classdev_unregister);

static int __init motors_init(void)
{
	motors_class = class_create(THIS_MODULE, "motors");
	if (IS_ERR(motors_class))
		return PTR_ERR(motors_class);
	motors_class->suspend = motor_suspend;
	motors_class->resume = motor_resume;
	motors_class->dev_attrs = motor_class_attrs;
	return 0;
}

static void __exit motors_exit(void)
{
	class_destroy(motors_class);
}

subsys_initcall(motors_init);
module_exit(motors_exit);

MODULE_AUTHOR("John Lenz, Richard Purdie");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("motor Class Interface");
