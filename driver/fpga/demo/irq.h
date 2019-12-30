/*
 * irq.h
 *
 *  Created on: Nov 26, 2019
 *      Author: lay
 */

#ifndef IRQ_H_
#define IRQ_H_

#include <linux/interrupt.h>

struct raw_pkg {
	u32 status;
	u32 expo;
	u32 phy_addr;
};

extern irqreturn_t irq_image_cache_done(int irq, void *dev_id);
extern irqreturn_t irq_th_image_cache_done(int irq, void *dev_id);

#endif /* IRQ_H_ */
