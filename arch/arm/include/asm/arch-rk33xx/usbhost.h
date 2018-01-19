/*
 * (C) Copyright 2008-2015 Rockchip Electronics
 */
#ifndef _USBHOST_H_
#define _USBHOST_H_

extern struct rkusb_hcd_cfg *rkusb_active_hcd;

struct rkusb_hcd_cfg {
	bool enable;
	void* regbase;
	int gpio_vbus;
	char *name;
	void (*hw_init)(void);
	void (*hw_deinit)(void);
};

#endif /* _USBHOST_H_ */
