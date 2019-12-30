/*
 * irq.c
 *
 *  Created on: Nov 26, 2019
 *      Author: lay
 */

#include "irq.h"

irqreturn_t irq_image_cache_done(int irq, void *dev_id) {
	irqreturn_t ret = IRQ_HANDLED;
	struct fpga_blink *fb = (struct fpga_blink *)dev_id;
	printk("IRQ-> %d @%p\r\n", irq, fb);
	return ret;
}

irqreturn_t irq_th_image_cache_done(int irq, void *dev_id) {
	irqreturn_t ret = IRQ_HANDLED;
	struct fpga_blink *fb = (struct fpga_blink *)dev_id;
	printk("IRQ-> %d @%p\r\n", irq, fb);
	return ret;
}
