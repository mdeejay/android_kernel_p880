/*
 * raw_ip_net.c
 *
 * USB network driver for RAW-IP modems.
 *
 * Copyright (c) 2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/etherdevice.h>
#include <linux/usb.h>
#include <linux/jiffies.h>
#define BASEBAND_USB_NET_DEV_NAME		"rmnet%d"

#define NET_IP_ETHERTYPE		0x08, 0x00

#define	TX_TIMEOUT		10

#ifndef USB_NET_BUFSIZ
#define USB_NET_BUFSIZ				8192
#endif  

#define MAX_INTFS	3

MODULE_LICENSE("GPL");

int max_intfs = MAX_INTFS;
unsigned long usb_net_raw_ip_vid = 0x1519;
unsigned long usb_net_raw_ip_pid = 0x0020;
unsigned long usb_net_raw_ip_intf[MAX_INTFS] = { 0x03, 0x05, 0x07 };
unsigned long usb_net_raw_ip_rx_debug;
unsigned long usb_net_raw_ip_tx_debug;

module_param(max_intfs, int, 0644);
MODULE_PARM_DESC(max_intfs, "usb net (raw-ip) - max. interfaces supported");
module_param(usb_net_raw_ip_vid, ulong, 0644);
MODULE_PARM_DESC(usb_net_raw_ip_vid, "usb net (raw-ip) - USB VID");
module_param(usb_net_raw_ip_pid, ulong, 0644);
MODULE_PARM_DESC(usb_net_raw_ip_pid, "usb net (raw-ip) - USB PID");
module_param(usb_net_raw_ip_rx_debug, ulong, 0644);
MODULE_PARM_DESC(usb_net_raw_ip_rx_debug, "usb net (raw-ip) - rx debug");
module_param(usb_net_raw_ip_tx_debug, ulong, 0644);
MODULE_PARM_DESC(usb_net_raw_ip_tx_debug, "usb net (raw-ip) - tx debug");
struct delayed_work raw_ip_init_failure_work;
struct workqueue_struct *raw_ip_init_failure_workqueue = NULL;
static int usb_net_raw_ip_init(void);
#define USB_NET_RAW_IP_INIT_DELAY_ON_FAIL 15000

struct baseband_usb {
	int baseband_index;
	struct net_device_stats stats;
	struct {
		struct usb_driver *driver;
		struct usb_device *device;
		struct usb_interface *interface;
		struct {
			struct {
				unsigned int in;
				unsigned int out;
			} isoch, bulk, interrupt;
		} pipe;
		struct urb *rx_urb;
		struct urb *tx_urb;
		struct usb_anchor tx_urb_deferred;
		struct workqueue_struct *tx_workqueue;
		struct work_struct tx_work;
	} usb;
	struct urb *urb_r;
	void *buff;
	int susp_count;
};

static struct baseband_usb *baseband_usb_net[MAX_INTFS] = { 0, 0, 0};

static struct net_device *usb_net_raw_ip_dev[MAX_INTFS] = { 0, 0, 0};

static unsigned int g_usb_interface_index[MAX_INTFS] = {~0U, ~0U, ~0U};
static struct usb_interface *g_usb_interface[MAX_INTFS];

static int usb_net_raw_ip_rx_urb_submit(struct baseband_usb *usb);
static void usb_net_raw_ip_rx_urb_comp(struct urb *urb);

static int usb_net_raw_ip_tx_urb_submit(struct baseband_usb *usb,
	struct sk_buff *skb);
static void usb_net_raw_ip_tx_urb_work(struct work_struct *work);
static void usb_net_raw_ip_tx_urb_comp(struct urb *urb);
static void find_usb_pipe(struct baseband_usb *usb);

static int baseband_usb_driver_probe(struct usb_interface *intf,
	const struct usb_device_id *id)
{
	int i = id->driver_info;
	struct baseband_usb *usb = NULL;
	if(i < 0 || i >= MAX_INTFS) {
		pr_err("%s(%d): Invalid device id - %d\n", __func__, __LINE__, i);
		return -EINVAL;
	}
	usb = baseband_usb_net[i];
	pr_debug("%s(%d) { intf %p id %p\n", __func__, __LINE__, intf, id);

	pr_debug("i %d\n", i);

	pr_debug("intf->cur_altsetting->desc.bInterfaceNumber %02x\n",
		intf->cur_altsetting->desc.bInterfaceNumber);
	pr_debug("intf->cur_altsetting->desc.bAlternateSetting %02x\n",
		intf->cur_altsetting->desc.bAlternateSetting);
	pr_debug("intf->cur_altsetting->desc.bNumEndpoints %02x\n",
		intf->cur_altsetting->desc.bNumEndpoints);
	pr_debug("intf->cur_altsetting->desc.bInterfaceClass %02x\n",
		intf->cur_altsetting->desc.bInterfaceClass);
	pr_debug("intf->cur_altsetting->desc.bInterfaceSubClass %02x\n",
		intf->cur_altsetting->desc.bInterfaceSubClass);
	pr_debug("intf->cur_altsetting->desc.bInterfaceProtocol %02x\n",
		intf->cur_altsetting->desc.bInterfaceProtocol);
	pr_debug("intf->cur_altsetting->desc.iInterface %02x\n",
		intf->cur_altsetting->desc.iInterface);

	
	if ((usb == NULL && g_usb_interface_index[i] !=
		intf->cur_altsetting->desc.bInterfaceNumber )
		|| (usb != NULL && usb_net_raw_ip_intf[i] != intf->cur_altsetting->desc.bInterfaceNumber)) {
		pr_debug("%s(%d) } -ENODEV\n", __func__, __LINE__);
		return -ENODEV;
	} else {
		g_usb_interface[i] = intf;
		if(usb != NULL){
			int err = 0;
			usb->usb.device = interface_to_usbdev(g_usb_interface[i]);
			usb->usb.interface = g_usb_interface[i];
			find_usb_pipe(usb);
			usb->usb.rx_urb = (struct urb *) 0;
			usb->usb.tx_urb = (struct urb *) 0;
			g_usb_interface_index[i] = ~0U;
			err = usb_net_raw_ip_rx_urb_submit(baseband_usb_net[i]);
			if (err < 0) {
				pr_debug("%s(%d) } submit rx failed. err  - %d\n", __func__, __LINE__, err);
				return err;
			}
		}
	}

	pr_debug("%s(%d) }\n", __func__, __LINE__);
	return 0;
}

static void baseband_usb_driver_disconnect(struct usb_interface *intf)
{
	int i;
	struct urb *urb;

	pr_debug("%s intf %p\n", __func__, intf);
	for (i = 0; i < max_intfs; i++) {
		pr_debug("[%d]\n", i);
		if (!baseband_usb_net[i])
			continue;
		if (baseband_usb_net[i]->usb.interface != intf) {
			pr_debug("%p != %p\n",
				baseband_usb_net[i]->usb.interface, intf);
			continue;
		}
		baseband_usb_net[i]->susp_count = 0;
		baseband_usb_net[i]->usb.interface
			= (struct usb_interface *) 0;
	}

}

#ifdef CONFIG_PM
static int baseband_usb_driver_suspend(struct usb_interface *intf,
	pm_message_t message)
{
	int i, err, susp_count;

	pr_debug("%s intf %p\n", __func__, intf);
	if(raw_ip_init_failure_workqueue) {
		destroy_workqueue(raw_ip_init_failure_workqueue);
		raw_ip_init_failure_workqueue = NULL;
	}
	for (i = 0; i < max_intfs; i++) {
		pr_debug("[%d]\n", i);
		if (!baseband_usb_net[i])
			continue;
		if (baseband_usb_net[i]->usb.interface != intf) {
			pr_debug("%p != %p\n",
				baseband_usb_net[i]->usb.interface, intf);
			continue;
		}
		susp_count = (baseband_usb_net[i]->susp_count)++;
		if (susp_count > 0) {
			pr_info("%s: susp_count %d > 0 (already suspended)\n",
				__func__, susp_count);
			continue;
		}
		if (susp_count < 0) {
			pr_info("%s: susp_count %d < 0 (ILLEGAL VALUE)\n",
				__func__, susp_count);
			baseband_usb_net[i]->susp_count = 0;
			continue;
		}
		pr_info("%s: susp_count = %d (suspending...)\n",
			__func__, susp_count);
		if (!baseband_usb_net[i]->usb.rx_urb) {
			pr_debug("rx_usb already killed\n");
			continue;
		}
		usb_kill_urb(baseband_usb_net[i]->usb.rx_urb);
		baseband_usb_net[i]->usb.rx_urb = (struct urb *) 0;

		
		if (!baseband_usb_net[i]->usb.tx_workqueue) {
			pr_err("%s: !tx_workqueue\n", __func__);
			continue;
	}

		pr_info("%s: try to cancel_work_sync...\n",__func__);
		cancel_work_sync(&baseband_usb_net[i]->usb.tx_work);
		pr_info("%s: try to cancel_work_sync...Done!\n",__func__);
	}

	return 0;
}

static int baseband_usb_driver_resume(struct usb_interface *intf)
{
	int i, err, susp_count;

	pr_debug("%s intf %p\n", __func__, intf);

	for (i = 0; i < max_intfs; i++) {
		pr_debug("[%d]\n", i);
		if (!baseband_usb_net[i])
			continue;
		if (baseband_usb_net[i]->usb.interface != intf) {
			pr_debug("%p != %p\n",
				baseband_usb_net[i]->usb.interface, intf);
			continue;
		}
		susp_count = --(baseband_usb_net[i]->susp_count);
		if (susp_count > 0) {
			pr_info("%s: susp_count %d > 0 (not resuming yet)\n",
				__func__, susp_count);
			continue;
		}
		if (susp_count < 0) {
			pr_info("%s: susp_count %d < 0 (ILLEGAL VALUE)\n",
				__func__, susp_count);
			baseband_usb_net[i]->susp_count = 0;
			continue;
		}
		pr_info("%s: susp_count = %d (resuming...)\n",
			__func__, susp_count);
		if (baseband_usb_net[i]->usb.rx_urb) {
			pr_debug("rx_usb already exists\n");
			continue;
		}
		err = usb_net_raw_ip_rx_urb_submit(baseband_usb_net[i]);
		if (err < 0) {
			pr_err("submit rx failed - err %d\n", err);
			continue;
		}
		if (!baseband_usb_net[i]->usb.tx_workqueue) {
			pr_err("%s: !tx_workqueue\n", __func__);
			continue;
		}
		queue_work(baseband_usb_net[i]->usb.tx_workqueue,
			&baseband_usb_net[i]->usb.tx_work);
	}

	return 0;
}
static int baseband_usb_driver_reset_resume(struct usb_interface *intf)
{
	pr_debug("%s intf %p\n", __func__, intf);
	return baseband_usb_driver_resume(intf);
}
#endif /* CONFIG_PM */

#if 1
static struct usb_device_id baseband_usb_driver_id_table0[] = {
	{ USB_DEVICE(0x1519, 0x0020), },
	{ }
};
MODULE_DEVICE_TABLE(usb, baseband_usb_driver_id_table0);
#endif

static struct usb_device_id baseband_usb_driver_id_table[MAX_INTFS][2];

static char baseband_usb_driver_name[MAX_INTFS][32];

static struct usb_driver baseband_usb_driver[MAX_INTFS] = {
	{
		.name = baseband_usb_driver_name[0],
		.probe = baseband_usb_driver_probe,
		.disconnect = baseband_usb_driver_disconnect,
		.id_table = baseband_usb_driver_id_table[0],
#ifdef CONFIG_PM
		.suspend = baseband_usb_driver_suspend,
		.resume = baseband_usb_driver_resume,
		.reset_resume = baseband_usb_driver_reset_resume,
		.supports_autosuspend = 1,
#endif
	},
	{
		.name = baseband_usb_driver_name[1],
		.probe = baseband_usb_driver_probe,
		.disconnect = baseband_usb_driver_disconnect,
		.id_table = baseband_usb_driver_id_table[1],
#ifdef CONFIG_PM
		.suspend = baseband_usb_driver_suspend,
		.resume = baseband_usb_driver_resume,
		.reset_resume = baseband_usb_driver_reset_resume,
		.supports_autosuspend = 1,
#endif
	},
	{
		.name = baseband_usb_driver_name[2],
		.probe = baseband_usb_driver_probe,
		.disconnect = baseband_usb_driver_disconnect,
		.id_table = baseband_usb_driver_id_table[2],
#ifdef CONFIG_PM
		.suspend = baseband_usb_driver_suspend,
		.resume = baseband_usb_driver_resume,
		.reset_resume = baseband_usb_driver_reset_resume,
		.supports_autosuspend = 1,
#endif
	},
};

static void find_usb_pipe(struct baseband_usb *usb)
{
	struct usb_device *usbdev = usb->usb.device;
	struct usb_interface *intf = usb->usb.interface;
	unsigned char numendpoint = intf->cur_altsetting->desc.bNumEndpoints;
	struct usb_host_endpoint *endpoint = intf->cur_altsetting->endpoint;
	unsigned char n;

	for (n = 0; n < numendpoint; n++) {
		if (usb_endpoint_is_isoc_in(&endpoint[n].desc)) {
			pr_debug("endpoint[%d] isochronous in\n", n);
			usb->usb.pipe.isoch.in = usb_rcvisocpipe(usbdev,
				endpoint[n].desc.bEndpointAddress);
		} else if (usb_endpoint_is_isoc_out(&endpoint[n].desc)) {
			pr_debug("endpoint[%d] isochronous out\n", n);
			usb->usb.pipe.isoch.out = usb_sndisocpipe(usbdev,
				endpoint[n].desc.bEndpointAddress);
		} else if (usb_endpoint_is_bulk_in(&endpoint[n].desc)) {
			pr_debug("endpoint[%d] bulk in\n", n);
			usb->usb.pipe.bulk.in = usb_rcvbulkpipe(usbdev,
				endpoint[n].desc.bEndpointAddress);
		} else if (usb_endpoint_is_bulk_out(&endpoint[n].desc)) {
			pr_debug("endpoint[%d] bulk out\n", n);
			usb->usb.pipe.bulk.out = usb_sndbulkpipe(usbdev,
				endpoint[n].desc.bEndpointAddress);
		} else if (usb_endpoint_is_int_in(&endpoint[n].desc)) {
			pr_debug("endpoint[%d] interrupt in\n", n);
			usb->usb.pipe.interrupt.in = usb_rcvintpipe(usbdev,
				endpoint[n].desc.bEndpointAddress);
		} else if (usb_endpoint_is_int_out(&endpoint[n].desc)) {
			pr_debug("endpoint[%d] interrupt out\n", n);
			usb->usb.pipe.interrupt.out = usb_sndintpipe(usbdev,
				endpoint[n].desc.bEndpointAddress);
		} else {
			pr_debug("endpoint[%d] skipped\n", n);
		}
	}
}

void baseband_usb_close(struct baseband_usb *usb);

struct baseband_usb *baseband_usb_open(int index,
	unsigned int vid,
	unsigned int pid,
	unsigned int intf)
{
	struct baseband_usb *usb;
	int err;

	pr_debug("baseband_usb_open {\n");

	usb = kzalloc(sizeof(struct baseband_usb),
		GFP_KERNEL);
	if (!usb)
		return (struct baseband_usb *) 0;

	sprintf(baseband_usb_driver_name[index],
		"baseband_usb_%x_%x_%x",
		vid, pid, intf);
	baseband_usb_driver_id_table[index][0].match_flags =
		USB_DEVICE_ID_MATCH_DEVICE;
	baseband_usb_driver_id_table[index][0].idVendor = vid;
	baseband_usb_driver_id_table[index][0].idProduct = pid;
	baseband_usb_driver_id_table[index][0].driver_info = index;
	g_usb_interface_index[index] = intf;
	g_usb_interface[index] = (struct usb_interface *) 0;
	err = usb_register(&baseband_usb_driver[index]);
	if (err < 0) {
		pr_err("cannot open usb driver - err %d\n", err);
		kfree(usb);
		return (struct baseband_usb *) 0;
	}
	usb->baseband_index = index;
	usb->usb.driver = &baseband_usb_driver[index];
	if (!g_usb_interface[index]) {
		pr_err("cannot open usb driver - !g_usb_interface[%d]\n",
			index);
		usb_deregister(usb->usb.driver);
		kfree(usb);
		return (struct baseband_usb *) 0;
	}
	usb->usb.device = interface_to_usbdev(g_usb_interface[index]);
	usb->usb.interface = g_usb_interface[index];
	find_usb_pipe(usb);
	usb->usb.rx_urb = (struct urb *) 0;
	usb->usb.tx_urb = (struct urb *) 0;
	g_usb_interface_index[index] = ~0U;
	g_usb_interface[index] = (struct usb_interface *) 0;
	pr_debug("usb->usb.driver->name %s\n", usb->usb.driver->name);
	pr_debug("usb->usb.device %p\n", usb->usb.device);
	pr_debug("usb->usb.interface %p\n", usb->usb.interface);
	pr_debug("usb->usb.pipe.isoch.in %x\n", usb->usb.pipe.isoch.in);
	pr_debug("usb->usb.pipe.isoch.out %x\n", usb->usb.pipe.isoch.out);
	pr_debug("usb->usb.pipe.bulk.in %x\n", usb->usb.pipe.bulk.in);
	pr_debug("usb->usb.pipe.bulk.out %x\n", usb->usb.pipe.bulk.out);
	pr_debug("usb->usb.pipe.interrupt.in %x\n", usb->usb.pipe.interrupt.in);
	pr_debug("usb->usb.pipe.interrupt.out %x\n",
		usb->usb.pipe.interrupt.out);

	pr_debug("baseband_usb_open }\n");
	return usb;
}

void baseband_usb_close(struct baseband_usb *usb)
{
	pr_debug("baseband_usb_close {\n");

	if (!usb)
		return;

	if (usb->usb.driver) {
		pr_debug("close usb driver {\n");
		usb_deregister(usb->usb.driver);
		usb->usb.driver = (struct usb_driver *) 0;
		pr_debug("close usb driver }\n");
	}
	kfree(usb);

	pr_debug("baseband_usb_close }\n");
}

static int baseband_usb_netdev_init(struct net_device *dev)
{
	pr_debug("baseband_usb_netdev_init\n");
	return 0;
}

static void baseband_usb_netdev_uninit(struct net_device *dev)
{
	pr_debug("baseband_usb_netdev_uninit\n");
}

static int baseband_usb_netdev_open(struct net_device *dev)
{
	pr_debug("baseband_usb_netdev_open\n");
	netif_start_queue(dev);
	return 0;
}

static int baseband_usb_netdev_stop(struct net_device *dev)
{
	pr_debug("baseband_usb_netdev_stop\n");
	netif_stop_queue(dev);
	return 0;
}

static netdev_tx_t baseband_usb_netdev_start_xmit(
	struct sk_buff *skb, struct net_device *dev)
{
	int i = 0;
	struct baseband_usb *usb;
	int err;

	pr_debug("baseband_usb_netdev_start_xmit\n");

	if (!skb) {
		pr_err("no skb\n");
		return NETDEV_TX_BUSY;
	}
	if (!dev) {
		pr_err("no net dev\n");
		return NETDEV_TX_BUSY;
	}

	for (i = 0; i < max_intfs; i++) {
		if (usb_net_raw_ip_dev[i] == dev)
			break;
	}
	if (i >= max_intfs) {
		pr_err("unknown net dev %p\n", dev);
		return NETDEV_TX_BUSY;
	}
	usb = baseband_usb_net[i];

	if(!usb || !usb->usb.interface) {
		pr_err("%s :N0 USB DEV for TX\n", __func__);
		return NETDEV_TX_BUSY;
	}
	err = usb_autopm_get_interface_async(usb->usb.interface);
	if(err < 0) {
		return NETDEV_TX_BUSY;
	}
	err = usb_net_raw_ip_tx_urb_submit(usb, skb);
	if (err < 0) {
		pr_err("tx urb submit error\n");
		netif_stop_queue(dev);
		usb->stats.tx_errors++;
		return NETDEV_TX_BUSY;
	}

	return NETDEV_TX_OK;
}

static struct net_device_stats *baseband_usb_netdev_get_stats(
				struct net_device *dev)
{
	int i;
	for (i = 0; i < max_intfs; i++) {
		if ( dev == usb_net_raw_ip_dev[i] ) {
			pr_info("%s idx(%d) \n", __func__, i);
			return &baseband_usb_net[i]->stats;
		}
	}
	pr_info("%s mismatch dev, default idx(0)\n", __func__);
	return &baseband_usb_net[0]->stats;
}

static struct net_device_ops usb_net_raw_ip_ops = {
	.ndo_init =		baseband_usb_netdev_init,
	.ndo_uninit =		baseband_usb_netdev_uninit,
	.ndo_open =		baseband_usb_netdev_open,
	.ndo_stop =		baseband_usb_netdev_stop,
	.ndo_start_xmit =	baseband_usb_netdev_start_xmit,
	.ndo_get_stats = baseband_usb_netdev_get_stats,
};

static int usb_net_raw_ip_rx_urb_submit(struct baseband_usb *usb)
{
	struct urb *urb;
	void *buf;
	int err;

	pr_debug("usb_net_raw_ip_rx_urb_submit { usb %p\n", usb);

	if (!usb) {
		pr_err("%s: !usb\n", __func__);
		return -EINVAL;
	}
	if (!usb->usb.interface) {
		pr_err("usb interface disconnected - not submitting rx urb\n");
		return -EINVAL;
	}
	if (usb->usb.rx_urb) {
		pr_err("previous urb still active\n");
		return -EBUSY;
	}
	if (!usb->urb_r || !usb->buff) {
		pr_err("no reusable rx urb found\n");
		return -ENOMEM;
	}

	urb = usb->urb_r;
	buf = usb->buff;
	usb_fill_bulk_urb(urb, usb->usb.device, usb->usb.pipe.bulk.in,
		buf, USB_NET_BUFSIZ,
		usb_net_raw_ip_rx_urb_comp,
		usb);
	urb->transfer_flags = 0;

	usb_mark_last_busy(usb->usb.device);
	usb->usb.rx_urb = urb;
	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err < 0) {
		pr_err("usb_submit_urb() failed - err %d\n", err);
		usb->usb.rx_urb = (struct urb *) 0;
		return err;
	}

	pr_debug("usb_net_raw_ip_rx_urb_submit }\n");
	return err;
}

static void usb_net_raw_ip_rx_urb_comp(struct urb *urb)
{
	struct baseband_usb *usb = NULL;
	int i = 0;
	struct sk_buff *skb;
	unsigned char *dst;
	unsigned char ethernet_header[14] = {
		0x00, 0x00,
		0x00, 0x00,
		0x00, 0x00,
		0x00, 0x00,
		0x00, 0x00,
		0x00, 0x00,
		NET_IP_ETHERTYPE,
	};

	pr_debug("usb_net_raw_ip_rx_urb_comp { urb %p\n", urb);

	if (!urb) {
		pr_err("no urb\n");
		return;
	}
	usb = (struct baseband_usb *) urb->context;
	i = usb->baseband_index;
	switch (urb->status) {
	case 0:
		break;
	case -ENOENT:
	case -ESHUTDOWN:
	case -EPROTO:
		pr_info("%s: rx urb %p - link shutdown %d\n",
			__func__, urb, urb->status);
		goto err_exit;
	default:
		pr_info("%s: rx urb %p - status %d\n",
			__func__, urb, urb->status);
		break;
	}

	if (urb->actual_length) {
		pr_debug("usb_net_raw_ip_rx_urb_comp - "
			"urb->actual_length %d\n", urb->actual_length);
		
		skb = netdev_alloc_skb(usb_net_raw_ip_dev[i],
			NET_IP_ALIGN + 14 + urb->actual_length);
		if (skb) {
		
			memcpy(ethernet_header + 0,
				usb_net_raw_ip_dev[i]->dev_addr, 6);
			memcpy(ethernet_header + 6,
				"0x01\0x02\0x03\0x04\0x05\0x06", 6);
		
			skb_reserve(skb, NET_IP_ALIGN);
			dst = skb_put(skb, 14);
			memcpy(dst, ethernet_header, 14);
			dst = skb_put(skb, urb->actual_length);
			memcpy(dst, urb->transfer_buffer, urb->actual_length);
			skb->protocol = eth_type_trans(skb,
				usb_net_raw_ip_dev[i]);
			if (netif_rx(skb) < 0) {
				pr_err("usb_net_raw_ip_rx_urb_comp_work - "
					"netif_rx(%p) failed\n", skb);
				kfree_skb(skb);
				usb->stats.rx_errors++;
			} else {
				usb->stats.rx_packets++;
				usb->stats.rx_bytes +=
				    (14 + urb->actual_length);
			}
		} else {
			pr_err("usb_net_raw_ip_rx_urb_comp_work - "
				"netdev_alloc_skb() failed\n");
		}
	}

	usb->usb.rx_urb = (struct urb *) 0;

	usb_net_raw_ip_rx_urb_submit(usb);
	return;

err_exit:
	usb->usb.rx_urb = (struct urb *) 0;

	pr_debug("usb_net_raw_ip_rx_urb_comp }\n");
	return;
}

static int usb_net_raw_ip_setup_rx_urb( struct baseband_usb *usb)
{
	pr_debug("usb_net_raw_ip_setup_rx_urb {\n");

	if (!usb) {
		pr_err("%s: !usb\n", __func__);
		return -EINVAL;
	}
	if (usb->urb_r) {
		pr_err("%s: reusable rx urb already allocated\n", __func__);
		return -EINVAL;
	}

	usb->urb_r = usb_alloc_urb(0, GFP_ATOMIC);
	if (!usb->urb_r) {
		pr_err("usb_alloc_urb() failed\n");
		return -ENOMEM;
	}
	usb->buff = kzalloc(USB_NET_BUFSIZ, GFP_ATOMIC);
	if (!usb->buff) {
		pr_err("usb buffer kzalloc() failed\n");
		usb_free_urb(usb->urb_r);
		usb->urb_r = (struct urb *) 0;
		return -ENOMEM;
	}

	pr_debug("usb_net_raw_setup_ip_rx_urb }\n");
	return 0;
}

static void usb_net_raw_ip_free_rx_urb(struct baseband_usb *usb)
{
	pr_debug("usb_net_raw_ip_free_rx_urb {\n");

	if (!usb) {
		pr_err("%s: !usb\n", __func__);
		return;
	}

	if (usb->urb_r) {
		usb_free_urb(usb->urb_r);
		usb->urb_r = (struct urb *) 0;
	}
	if (usb->buff) {
		kfree(usb->buff);
		usb->buff = (void *) 0;
	}

	pr_debug("usb_net_raw_ip_free_rx_urb }\n");
}

static int usb_net_raw_ip_tx_urb_submit(struct baseband_usb *usb,
	struct sk_buff *skb)
{
	struct urb *urb;
	unsigned char *buf;
	int err;

	pr_debug("usb_net_raw_ip_tx_urb_submit {\n");

	if (!usb) {
		pr_err("%s: !usb\n", __func__);
		if(skb) {
			kfree_skb(skb);
		}	
		return -EINVAL;
	}

	if (!usb->usb.interface) {
		pr_err("usb interface disconnected - not submitting tx urb\n");
		if(skb) {
		kfree_skb(skb);
		}	
		return -EINVAL;
	}

	if (!skb) {
		 pr_err("%s: !skb\n", __func__);
		 usb_autopm_put_interface_async(usb->usb.interface);
		return -EINVAL;
	}
	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!urb) {
		pr_err("usb_alloc_urb() failed\n");
		usb_autopm_put_interface_async(usb->usb.interface);
		kfree_skb(skb);
		return -ENOMEM;
	}
	buf = kzalloc(skb->len - 14, GFP_ATOMIC);
	if (!buf) {
		pr_err("usb buffer kzalloc() failed\n");
		usb_free_urb(urb);
		kfree_skb(skb);
		usb_autopm_put_interface_async(usb->usb.interface);
		return -ENOMEM;
	}
	err = skb_copy_bits(skb, 14, buf, skb->len - 14);
	if (err < 0) {
		pr_err("skb_copy_bits() failed - %d\n", err);
		kfree(buf);
		usb_free_urb(urb);
		kfree_skb(skb);
		usb_autopm_put_interface_async(usb->usb.interface);
		return err;
	}
	usb_fill_bulk_urb(urb, usb->usb.device, usb->usb.pipe.bulk.out,
		buf, skb->len - 14,
		usb_net_raw_ip_tx_urb_comp,
		usb);
	urb->transfer_flags = URB_ZERO_PACKET;

	usb_anchor_urb(urb, &usb->usb.tx_urb_deferred);
	queue_work(usb->usb.tx_workqueue, &usb->usb.tx_work);

	consume_skb(skb);

	pr_debug("usb_net_raw_ip_tx_urb_submit }\n");
	return 0;
}

static void usb_net_raw_ip_tx_urb_work(struct work_struct *work)
{
	struct baseband_usb *usb
		= container_of(work, struct baseband_usb, usb.tx_work);
	struct urb *urb;
	int err;

	pr_debug("usb_net_raw_ip_tx_urb_work {\n");

	if (usb == NULL ||  (!usb->usb.tx_urb && usb_anchor_empty(&usb->usb.tx_urb_deferred))) {
		pr_debug("%s: nothing to do!\n", __func__);
		return;
	}

	if (usb->susp_count > 0) {
		pr_info("%s: usb->susp_count %d > 0 (suspended)\n",
			__func__, usb->susp_count);
		return;
	}

	while ((urb = usb_get_from_anchor(&usb->usb.tx_urb_deferred))
		!= (struct urb *) 0) {
		usb_free_urb(urb);
		if (!usb->usb.interface) {
			pr_err("%s: not submitting tx urb %p"
				" - interface disconnected\n",
				__func__, urb);
			if (urb->transfer_buffer) {
				kfree(urb->transfer_buffer);
				urb->transfer_buffer = (void *) 0;
			}
			usb_free_urb(urb);
			continue;
		}
		usb_mark_last_busy(usb->usb.device);
#if 0
		err = usb_autopm_get_interface(usb->usb.interface);
		if (err < 0) {
			pr_err("%s: usb_autopm_get_interface(%p) failed %d\n",
				__func__, usb->usb.interface, err);
			if (urb->transfer_buffer) {
				kfree(urb->transfer_buffer);
				urb->transfer_buffer = (void *) 0;
			}
			usb_free_urb(urb);
			usb->stats.tx_errors++;
			continue;
		}
#endif

		err = usb_submit_urb(urb, GFP_ATOMIC);
		if (err < 0) {
			pr_err("%s: usb_submit_urb(%p) failed - err %d\n",
				__func__, urb, err);
			usb_autopm_put_interface_async(usb->usb.interface);
			if (urb->transfer_buffer) {
				kfree(urb->transfer_buffer);
				urb->transfer_buffer = (void *) 0;
			}
			usb_free_urb(urb);
			usb->stats.tx_errors++;
			continue;
		}
	
		usb_free_urb(urb);
	}

	pr_debug("usb_net_raw_ip_tx_urb_work }\n");
}

static void usb_net_raw_ip_tx_urb_comp(struct urb *urb)
{
	struct baseband_usb *usb = NULL;

	pr_debug("usb_net_raw_ip_tx_urb_comp {\n");

	if (!urb) {
		pr_err("no urb\n");
		return;
	}
	usb = (struct baseband_usb *) urb->context;
	switch (urb->status) {
	case 0:
		break;
	case -ENOENT:
	case -ESHUTDOWN:
	case -EPROTO:
		pr_info("%s: tx urb %p - link shutdown %d\n",
			__func__, urb, urb->status);
		usb_autopm_put_interface_async(usb->usb.interface);
		goto err_exit;
	default:
		pr_info("%s: tx urb %p - status %d\n",
			__func__, urb, urb->status);
		break;
	}

	if (urb && urb->status)
	    usb->stats.tx_errors++;
	else {
	    usb->stats.tx_packets++;
	    usb->stats.tx_bytes += urb->transfer_buffer_length;
	}

	if (!usb->usb.interface) {
		pr_err("%s: usb interface disconnected"
			" before tx urb completed!\n",
			__func__);
		goto err_exit;
	}
	usb_autopm_put_interface_async(usb->usb.interface);

err_exit:
	if (urb->transfer_buffer) {
		kfree(urb->transfer_buffer);
		urb->transfer_buffer = (void *) 0;
	}

	pr_debug("usb_net_raw_ip_tx_urb_comp }\n");
}

static void raw_ip_init_failure_work_handler(struct work_struct *work)
{
	usb_net_raw_ip_init();
}

static void raw_ip_failure_handler()
{
	pr_debug("raw_ip_failure_handler {\n");
	if(!raw_ip_init_failure_workqueue) {
		raw_ip_init_failure_workqueue
			= create_singlethread_workqueue("raw_failure_hnd");
		INIT_DELAYED_WORK(&raw_ip_init_failure_work, raw_ip_init_failure_work_handler);
	}
	queue_delayed_work(raw_ip_init_failure_workqueue, &raw_ip_init_failure_work, 
			msecs_to_jiffies(USB_NET_RAW_IP_INIT_DELAY_ON_FAIL));
}

static int usb_net_raw_ip_init(void)
{
	int i;
	int err;
	char name[32];

	pr_debug("usb_net_raw_ip_init {\n");

	for (i = 0; i < max_intfs; i++) {
		baseband_usb_net[i] = baseband_usb_open(i, usb_net_raw_ip_vid,
			usb_net_raw_ip_pid, usb_net_raw_ip_intf[i]);
		if (!baseband_usb_net[i]) {
			pr_err("cannot open baseband usb net\n");
			raw_ip_failure_handler();
			err = -1;
			goto error_exit;
		}
		usb_net_raw_ip_dev[i] = alloc_netdev(0,
			BASEBAND_USB_NET_DEV_NAME,
			ether_setup);
		if (!usb_net_raw_ip_dev[i]) {
			pr_err("alloc_netdev() failed\n");
			err = -ENOMEM;
			goto error_exit;
		}
		usb_net_raw_ip_dev[i]->netdev_ops = &usb_net_raw_ip_ops;
		usb_net_raw_ip_dev[i]->watchdog_timeo = TX_TIMEOUT;
		random_ether_addr(usb_net_raw_ip_dev[i]->dev_addr);
		err = register_netdev(usb_net_raw_ip_dev[i]);
		if (err < 0) {
			pr_err("cannot register network device - %d\n", err);
			goto error_exit;
		}
		pr_debug("registered baseband usb network device"
				" - dev %p name %s\n", usb_net_raw_ip_dev[i],
				 BASEBAND_USB_NET_DEV_NAME);
		err = usb_net_raw_ip_setup_rx_urb(baseband_usb_net[i]);
		if (err < 0) {
			pr_err("setup reusable rx urb failed - err %d\n", err);
			goto error_exit;
		}
		err = usb_net_raw_ip_rx_urb_submit(baseband_usb_net[i]);
		if (err < 0) {
			pr_err("submit rx failed - err %d\n", err);
			goto error_exit;
		}
		init_usb_anchor(&baseband_usb_net[i]->usb.tx_urb_deferred);
		sprintf(name, "raw_ip_tx_wq-%d",
			baseband_usb_net[i]->baseband_index);
		baseband_usb_net[i]->usb.tx_workqueue
			= create_singlethread_workqueue(name);
		if (!baseband_usb_net[i]->usb.tx_workqueue) {
			pr_err("cannot create workqueue\n");
			goto error_exit;
		}
		INIT_WORK(&baseband_usb_net[i]->usb.tx_work,
			usb_net_raw_ip_tx_urb_work);
	}

	pr_debug("usb_net_raw_ip_init }\n");
	return 0;

error_exit:
	for (i = 0; i < max_intfs; i++) {
		if (usb_net_raw_ip_dev[i]) {
			unregister_netdev(usb_net_raw_ip_dev[i]);
			free_netdev(usb_net_raw_ip_dev[i]);
			usb_net_raw_ip_dev[i] = (struct net_device *) 0;
		}
		if (baseband_usb_net[i]) {
			if (baseband_usb_net[i]->usb.tx_workqueue) {
				destroy_workqueue(baseband_usb_net[i]
					->usb.tx_workqueue);
				baseband_usb_net[i]->usb.tx_workqueue
					= (struct workqueue_struct *) 0;
			}
			if (baseband_usb_net[i]->usb.tx_urb) {
				usb_kill_urb(baseband_usb_net[i]->usb.tx_urb);
				baseband_usb_net[i]->usb.tx_urb
					= (struct urb *) 0;
			}
			if (baseband_usb_net[i]->usb.rx_urb) {
				usb_kill_urb(baseband_usb_net[i]->usb.rx_urb);
				baseband_usb_net[i]->usb.rx_urb
					= (struct urb *) 0;
			}
			usb_net_raw_ip_free_rx_urb(baseband_usb_net[i]);
			/* close usb */
			baseband_usb_close(baseband_usb_net[i]);
			baseband_usb_net[i] = (struct baseband_usb *) 0;
		}
	}

	return err;
}

static void usb_net_raw_ip_exit(void)
{
	int i;

	pr_debug("usb_net_raw_ip_exit {\n");

	for (i = 0; i < max_intfs; i++) {
		if (usb_net_raw_ip_dev[i]) {
			unregister_netdev(usb_net_raw_ip_dev[i]);
			free_netdev(usb_net_raw_ip_dev[i]);
			usb_net_raw_ip_dev[i] = (struct net_device *) 0;
		}
		if (baseband_usb_net[i]) {
			if (baseband_usb_net[i]->usb.tx_workqueue) {
				destroy_workqueue(baseband_usb_net[i]
					->usb.tx_workqueue);
				baseband_usb_net[i]->usb.tx_workqueue
					= (struct workqueue_struct *) 0;
			}
			if (baseband_usb_net[i]->usb.tx_urb) {
				usb_kill_urb(baseband_usb_net[i]->usb.tx_urb);
				baseband_usb_net[i]->usb.tx_urb
					= (struct urb *) 0;
			}
			if (baseband_usb_net[i]->usb.rx_urb) {
				usb_kill_urb(baseband_usb_net[i]->usb.rx_urb);
				baseband_usb_net[i]->usb.rx_urb
					= (struct urb *) 0;
			}
			usb_net_raw_ip_free_rx_urb(baseband_usb_net[i]);
			baseband_usb_close(baseband_usb_net[i]);
			baseband_usb_net[i] = (struct baseband_usb *) 0;
		}
	}

	pr_debug("usb_net_raw_ip_exit }\n");
}

module_init(usb_net_raw_ip_init)
module_exit(usb_net_raw_ip_exit)

