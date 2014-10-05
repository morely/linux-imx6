/*
 * Driver model for motors and motor triggers
 *
 * Copyright (C) 2005 John Lenz <lenz@cs.wisc.edu>
 * Copyright (C) 2005 Richard Purdie <rpurdie@openedhand.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef __LINUX_MOTORS_H_INCLUDED
#define __LINUX_MOTORS_H_INCLUDED

#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/rwsem.h>
#include <linux/timer.h>
#include <linux/workqueue.h>

struct device;
/*
 * motor Core
 */
extern struct list_head motors_list;
extern struct rw_semaphore motors_list_lock;


struct motor_classdev {
	const char		*name;
	int			 speed;
	int			 max_speed;
	int			 flags;

	/* Lower 16 bits reflect status */
#define motor_SUSPENDED		(1 << 0)
	/* Upper 16 bits reflect control information */
#define motor_CORE_SUSPENDRESUME	(1 << 16)

	/* Set motor speed level */
	void		(*speed_set)(struct motor_classdev *motor_cdev,
					  int speed);
	/* Get motor speed level */
	int  (*speed_get)(struct motor_classdev *motor_cdev);

	struct device		*dev;
	struct list_head	 node;			/* motor Device list */

	struct work_struct	set_speed_work;
	int			delayed_set_value;

};

extern int motor_classdev_register(struct device *parent,
				 struct motor_classdev *motor_cdev);
extern void motor_classdev_unregister(struct motor_classdev *motor_cdev);
extern void motor_classdev_suspend(struct motor_classdev *motor_cdev);
extern void motor_classdev_resume(struct motor_classdev *motor_cdev);

#define motorS_GPIO_DEFSTATE_OFF		0
#define motorS_GPIO_DEFSTATE_ON		1
#define motorS_GPIO_DEFSTATE_KEEP		2

enum cpu_motor_event {
	CPU_motor_IDLE_START,	/* CPU enters idle */
	CPU_motor_IDLE_END,	/* CPU idle ends */
	CPU_motor_START,		/* Machine starts, especially resume */
	CPU_motor_STOP,		/* Machine stops, especially suspend */
	CPU_motor_HALTED,		/* Machine shutdown */
};

static inline void motortrig_cpu(enum cpu_motor_event evt)
{
	return;
}

#endif		/* __LINUX_motorS_H_INCLUDED */
