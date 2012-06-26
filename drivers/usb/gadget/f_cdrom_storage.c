/*
 * f_cdrom_storage.c -- Virtual CDROM USB Composite Function
 *
 * Copyright (C) 2003-2008 Alan Stern
 * Copyright (C) 2009 Samsung Electronics
 *                    Author: Michal Nazarewicz <m.nazarewicz@samsung.com>
 * Copyright (C) 2011 LG Electronics
 * 					  Author: Hyeon H. Park <hyunhui.park@lge.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The names of the above-listed copyright holders may not be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


/*
 * The Mass Storage Function acts as a USB Mass Storage device,
 * appearing to the host as a disk drive or as a CD-ROM drive.  In
 * addition to providing an example of a genuinely useful composite
 * function for a USB device, it also illustrates a technique of
 * double-buffering for increased throughput.
 *
 * Function supports multiple logical units (LUNs).  Backing storage
 * for each LUN is provided by a regular file or a block device.
 * Access for each LUN can be limited to read-only.  Moreover, the
 * function can indicate that LUN is removable and/or CD-ROM.  (The
 * later implies read-only access.)
 *
 * MSF is configured by specifying a fsg_config structure.  It has the
 * following fields:
 *
 *	nluns		Number of LUNs function have (anywhere from 1
 *				to FSG_MAX_LUNS which is 8).
 *	luns		An array of LUN configuration values.  This
 *				should be filled for each LUN that
 *				function will include (ie. for "nluns"
 *				LUNs).  Each element of the array has
 *				the following fields:
 *	->filename	The path to the backing file for the LUN.
 *				Required if LUN is not marked as
 *				removable.
 *	->ro		Flag specifying access to the LUN shall be
 *				read-only.  This is implied if CD-ROM
 *				emulation is enabled as well as when
 *				it was impossible to open "filename"
 *				in R/W mode.
 *	->removable	Flag specifying that LUN shall be indicated as
 *				being removable.
 *	->cdrom		Flag specifying that LUN shall be reported as
 *				being a CD-ROM.
 *
 *	lun_name_format	A printf-like format for names of the LUN
 *				devices.  This determines how the
 *				directory in sysfs will be named.
 *				Unless you are using several MSFs in
 *				a single gadget (as opposed to single
 *				MSF in many configurations) you may
 *				leave it as NULL (in which case
 *				"lun%d" will be used).  In the format
 *				you can use "%d" to index LUNs for
 *				MSF's with more than one LUN.  (Beware
 *				that there is only one integer given
 *				as an argument for the format and
 *				specifying invalid format may cause
 *				unspecified behaviour.)
 *	thread_name	Name of the kernel thread process used by the
 *				MSF.  You can safely set it to NULL
 *				(in which case default "file-storage"
 *				will be used).
 *
 *	vendor_name
 *	product_name
 *	release		Information used as a reply to INQUIRY
 *				request.  To use default set to NULL,
 *				NULL, 0xffff respectively.  The first
 *				field should be 8 and the second 16
 *				characters or less.
 *
 *	can_stall	Set to permit function to halt bulk endpoints.
 *				Disabled on some USB devices known not
 *				to work correctly.  You should set it
 *				to true.
 *
 * If "removable" is not set for a LUN then a backing file must be
 * specified.  If it is set, then NULL filename means the LUN's medium
 * is not loaded (an empty string as "filename" in the fsg_config
 * structure causes error).  The CD-ROM emulation includes a single
 * data track and no audio tracks; hence there need be only one
 * backing file per LUN.  Note also that the CD-ROM block length is
 * set to 512 rather than the more common value 2048.
 *
 *
 * MSF includes support for module parameters.  If gadget using it
 * decides to use it, the following module parameters will be
 * available:
 *
 *	file=filename[,filename...]
 *			Names of the files or block devices used for
 *				backing storage.
 *	ro=b[,b...]	Default false, boolean for read-only access.
 *	removable=b[,b...]
 *			Default true, boolean for removable media.
 *	cdrom=b[,b...]	Default false, boolean for whether to emulate
 *				a CD-ROM drive.
 *	luns=N		Default N = number of filenames, number of
 *				LUNs to support.
 *	stall		Default determined according to the type of
 *				USB device controller (usually true),
 *				boolean to permit the driver to halt
 *				bulk endpoints.
 *
 * The module parameters may be prefixed with some string.  You need
 * to consult gadget's documentation or source to verify whether it is
 * using those module parameters and if it does what are the prefixes
 * (look for FSG_MODULE_PARAMETERS() macro usage, what's inside it is
 * the prefix).
 *
 *
 * Requirements are modest; only a bulk-in and a bulk-out endpoint are
 * needed.  The memory requirement amounts to two 16K buffers, size
 * configurable by a parameter.  Support is included for both
 * full-speed and high-speed operation.
 *
 * Note that the driver is slightly non-portable in that it assumes a
 * single memory/DMA buffer will be useable for bulk-in, bulk-out, and
 * interrupt-in endpoints.  With most device controllers this isn't an
 * issue, but there may be some with hardware restrictions that prevent
 * a buffer from being used by more than one endpoint.
 *
 *
 * The pathnames of the backing files and the ro settings are
 * available in the attribute files "file" and "ro" in the lun<n> (or
 * to be more precise in a directory which name comes from
 * "lun_name_format" option!) subdirectory of the gadget's sysfs
 * directory.  If the "removable" option is set, writing to these
 * files will simulate ejecting/loading the medium (writing an empty
 * line means eject) and adjusting a write-enable tab.  Changes to the
 * ro setting are not allowed when the medium is loaded or if CD-ROM
 * emulation is being used.
 *
 * When a LUN receive an "eject" SCSI request (Start/Stop Unit),
 * if the LUN is removable, the backing file is released to simulate
 * ejection.
 *
 *
 * This function is heavily based on "File-backed Storage Gadget" by
 * Alan Stern which in turn is heavily based on "Gadget Zero" by David
 * Brownell.  The driver's SCSI command interface was based on the
 * "Information technology - Small Computer System Interface - 2"
 * document from X3T9.2 Project 375D, Revision 10L, 7-SEP-93,
 * available at <http://www.t10.org/ftp/t10/drafts/s2/s2-r10l.pdf>.
 * The single exception is opcode 0x23 (READ FORMAT CAPACITIES), which
 * was based on the "Universal Serial Bus Mass Storage Class UFI
 * Command Specification" document, Revision 1.0, December 14, 1998,
 * available at
 * <http://www.usb.org/developers/devclass_docs/usbmass-ufi10.pdf>.
 */


/*
 *				Driver Design
 *
 * The MSF is fairly straightforward.  There is a main kernel
 * thread that handles most of the work.  Interrupt routines field
 * callbacks from the controller driver: bulk- and interrupt-request
 * completion notifications, endpoint-0 events, and disconnect events.
 * Completion events are passed to the main thread by wakeup calls.  Many
 * ep0 requests are handled at interrupt time, but SetInterface,
 * SetConfiguration, and device reset requests are forwarded to the
 * thread in the form of "exceptions" using SIGUSR1 signals (since they
 * should interrupt any ongoing file I/O operations).
 *
 * The thread's main routine implements the standard command/data/status
 * parts of a SCSI interaction.  It and its subroutines are full of tests
 * for pending signals/exceptions -- all this polling is necessary since
 * the kernel has no setjmp/longjmp equivalents.  (Maybe this is an
 * indication that the driver really wants to be running in userspace.)
 * An important point is that so long as the thread is alive it keeps an
 * open reference to the backing file.  This will prevent unmounting
 * the backing file's underlying filesystem and could cause problems
 * during system shutdown, for example.  To prevent such problems, the
 * thread catches INT, TERM, and KILL signals and converts them into
 * an EXIT exception.
 *
 * In normal operation the main thread is started during the gadget's
 * fsg_bind() callback and stopped during fsg_unbind().  But it can
 * also exit when it receives a signal, and there's no point leaving
 * the gadget running when the thread is dead.  At of this moment, MSF
 * provides no way to deregister the gadget when thread dies -- maybe
 * a callback functions is needed.
 *
 * To provide maximum throughput, the driver uses a circular pipeline of
 * buffer heads (struct fsg_buffhd).  In principle the pipeline can be
 * arbitrarily long; in practice the benefits don't justify having more
 * than 2 stages (i.e., double buffering).  But it helps to think of the
 * pipeline as being a long one.  Each buffer head contains a bulk-in and
 * a bulk-out request pointer (since the buffer can be used for both
 * output and input -- directions always are given from the host's
 * point of view) as well as a pointer to the buffer and various state
 * variables.
 *
 * Use of the pipeline follows a simple protocol.  There is a variable
 * (fsg->next_buffhd_to_fill) that points to the next buffer head to use.
 * At any time that buffer head may still be in use from an earlier
 * request, so each buffer head has a state variable indicating whether
 * it is EMPTY, FULL, or BUSY.  Typical use involves waiting for the
 * buffer head to be EMPTY, filling the buffer either by file I/O or by
 * USB I/O (during which the buffer head is BUSY), and marking the buffer
 * head FULL when the I/O is complete.  Then the buffer will be emptied
 * (again possibly by USB I/O, during which it is marked BUSY) and
 * finally marked EMPTY again (possibly by a completion routine).
 *
 * A module parameter tells the driver to avoid stalling the bulk
 * endpoints wherever the transport specification allows.  This is
 * necessary for some UDCs like the SuperH, which cannot reliably clear a
 * halt on a bulk endpoint.  However, under certain circumstances the
 * Bulk-only specification requires a stall.  In such cases the driver
 * will halt the endpoint and set a flag indicating that it should clear
 * the halt in software during the next device reset.  Hopefully this
 * will permit everything to work correctly.  Furthermore, although the
 * specification allows the bulk-out endpoint to halt when the host sends
 * too much data, implementing this would cause an unavoidable race.
 * The driver will always use the "no-stall" approach for OUT transfers.
 *
 * One subtle point concerns sending status-stage responses for ep0
 * requests.  Some of these requests, such as device reset, can involve
 * interrupting an ongoing file I/O operation, which might take an
 * arbitrarily long time.  During that delay the host might give up on
 * the original ep0 request and issue a new one.  When that happens the
 * driver should not notify the host about completion of the original
 * request, as the host will no longer be waiting for it.  So the driver
 * assigns to each ep0 request a unique tag, and it keeps track of the
 * tag value of the request associated with a long-running exception
 * (device-reset, interface-change, or configuration-change).  When the
 * exception handler is finished, the status-stage response is submitted
 * only if the current ep0 request tag is equal to the exception request
 * tag.  Thus only the most recently received ep0 request will get a
 * status-stage response.
 *
 * Warning: This driver source file is too long.  It ought to be split up
 * into a header file plus about 3 separate .c files, to handle the details
 * of the Gadget, USB Mass Storage, and SCSI protocols.
 */

 /* This function driver is heavily derived by mass storage function driver.
  * Rather than using cdrom module param of LUN already in mass storage
  * function driver, we use seperated function driver because of binding
  * process of android gadget. This function driver will be used as dedicated
  * virtual cdrom for feature like Autorun.
  * 2011-03-02, hyunhui.park@lge.com
  */



#include <linux/blkdev.h>
#include <linux/completion.h>
#include <linux/dcache.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fcntl.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/kref.h>
#include <linux/kthread.h>
#include <linux/limits.h>
#include <linux/rwsem.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/freezer.h>
#include <linux/utsname.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

#include "gadget_chips.h"

#ifdef CONFIG_LGE_USB_GADGET_DRIVER
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/switch.h>
#endif


#define CDFSG_DRIVER_DESC		"cdrom_storage"
#define CDFSG_DRIVER_VERSION	"2011/12/02"

#define FSG_NO_INTR_EP 1
#define FSG_NO_DEVICE_STRINGS    1
#define FSG_NO_OTG               1
#define FSG_NO_INTR_EP           1

#define SC_LGE_SPE      		0xF1
#define SUB_CODE_MODE_CHANGE		0x01
#define SUB_CODE_GET_VALUE		0x02
#define SUB_CODE_PROBE_DEV		0xff
#define TYPE_MOD_CHG_TO_ACM		0x01
#define TYPE_MOD_CHG_TO_UMS		0x02
#define TYPE_MOD_CHG_TO_MTP		0x03
#define TYPE_MOD_CHG_TO_ASK		0x05
#define TYPE_MOD_CHG_TO_CGO		0x08
#define TYPE_MOD_CHG_TO_TET		0x09
#define TYPE_MOD_CHG2_TO_ACM		0x81
#define TYPE_MOD_CHG2_TO_UMS		0x82
#define TYPE_MOD_CHG2_TO_MTP		0x83
#define TYPE_MOD_CHG2_TO_ASK		0x85
#define TYPE_MOD_CHG2_TO_CGO		0x86
#define TYPE_MOD_CHG2_TO_TET		0x87

/* ACK TO SEND HOST PC */


#define ACK_STATUS_TO_HOST		0x20
#define ACK_STATUS_TO_HOST_VZW		0x10

#define ACK_SW_REV_TO_HOST		0x12
#define ACK_MEID_TO_HOST		0x13
#define ACK_MODEL_TO_HOST		0x14
#define ACK_SUB_VER_TO_HOST		0x15
#define SUB_ACK_STATUS_ACM		0x00
#define SUB_ACK_STATUS_MTP		0x01
#define SUB_ACK_STATUS_UMS		0x02
#define SUB_ACK_STATUS_ASK		0x03
#define SUB_ACK_STATUS_CGO		0x04
#define SUB_ACK_STATUS_TET		0x05

#ifdef CONFIG_USB_CSW_HACK
static int write_error_after_csw_sent;
#endif

struct cdrom_fsg_dev;




enum chg_mode_state{
	MODE_STATE_UNKNOWN = 0,
	MODE_STATE_ACM,
	MODE_STATE_MTP,
	MODE_STATE_UMS,
	MODE_STATE_ASK,
	MODE_STATE_CGO,
	MODE_STATE_TET,
	MODE_STATE_GET_VALUE,
	MODE_STATE_PROBE_DEV,
};

static const char *check_str[] = {
	"ACK_STATUS_ACM",
	"ACK_STATUS_MTP",
	"ACK_STATUS_UMS",
	"ACK_STATUS_ASK",
	"ACK_STATUS_CGO",
	"ACK_STATUS_TET",
};

enum check_mode_state {
	ACK_STATUS_ACM = SUB_ACK_STATUS_ACM,
	ACK_STATUS_MTP = SUB_ACK_STATUS_MTP,
	ACK_STATUS_UMS = SUB_ACK_STATUS_UMS,
	ACK_STATUS_ASK = SUB_ACK_STATUS_ASK,
	ACK_STATUS_CGO = SUB_ACK_STATUS_CGO,
	ACK_STATUS_TET = SUB_ACK_STATUS_TET,
	ACK_STATUS_ERR,
};

/* Data shared by all the FSG instances. */
struct cdrom_fsg_common {
	struct usb_gadget	*gadget;
	struct cdrom_fsg_dev		*fsg, *new_fsg;
	wait_queue_head_t	fsg_wait;

	struct rw_semaphore	filesem;

	spinlock_t		lock;

	struct usb_ep		*ep0;
	struct usb_request	*ep0req;
	unsigned int		ep0_req_tag;
	const char		*ep0req_name;

	struct fsg_buffhd	*next_buffhd_to_fill;
	struct fsg_buffhd	*next_buffhd_to_drain;
	struct fsg_buffhd	buffhds[FSG_NUM_BUFFERS];

	int			cmnd_size;
	u8			cmnd[MAX_COMMAND_SIZE];

	unsigned int		nluns;
	unsigned int		lun;
	struct fsg_lun		*luns;
	struct fsg_lun		*curlun;

	unsigned int		bulk_out_maxpacket;
	enum fsg_state		state;
	unsigned int		exception_req_tag;

	enum data_direction	data_dir;
	u32			data_size;
	u32			data_size_from_cmnd;
	u32			tag;
	u32			residue;
	u32			usb_amount_left;

	unsigned int		can_stall:1;
	unsigned int		free_storage_on_release:1;
	unsigned int		phase_error:1;
	unsigned int		short_packet_received:1;
	unsigned int		bad_lun_okay:1;
	unsigned int		running:1;

	int			thread_wakeup_needed;
	struct completion	thread_notifier;
	struct task_struct	*thread_task;

	int			(*thread_exits)(struct cdrom_fsg_common *common);
	void			*private_data;

	char inquiry_string[8 + 16 + 4 + 1];

	enum chg_mode_state mode_state;

	struct kref		ref;
};


struct cdrom_fsg_config {
	unsigned nluns;
	struct cdrom_fsg_lun_config {
		const char *filename;
		char ro;
		char removable;
		char cdrom;
	} luns[FSG_MAX_LUNS];

	const char		*lun_name_format;
	const char		*thread_name;

	int			(*thread_exits)(struct cdrom_fsg_common *common);
	void			*private_data;

	const char *vendor_name;
	const char *product_name;
	u16 release;

	char			can_stall;

};


struct cdrom_fsg_dev {
	struct usb_function	function;
	struct usb_gadget	*gadget;
	struct cdrom_fsg_common	*common;

	u16			interface_number;

	unsigned int		bulk_in_enabled:1;
	unsigned int		bulk_out_enabled:1;

	unsigned long		atomic_bitflags;
#define IGNORE_BULK_OUT		0

	struct usb_ep		*bulk_in;
	struct usb_ep		*bulk_out;
};

static const struct file_operations autorun_fops = {
	.owner = THIS_MODULE,
};

static struct miscdevice autorun_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "usb_autorun",
	.fops = &autorun_fops,
};
char *envp_ack[2] = { "AUTORUN=ACK", NULL };
static unsigned int user_mode = SUB_ACK_STATUS_CGO;

static inline int __cdrom_fsg_is_set(struct cdrom_fsg_common *common,
			       const char *func, unsigned line)
{
	if (common->fsg)
		return 1;
	ERROR(common, "common->fsg is NULL in %s at %u\n", func, line);
	return 0;
}

#define cdrom_fsg_is_set(common) likely(__cdrom_fsg_is_set(common, __func__, __LINE__))


static inline struct cdrom_fsg_dev *cdrom_fsg_from_func(struct usb_function *f)
{
	return container_of(f, struct cdrom_fsg_dev, function);
}


typedef void (*cdrom_fsg_routine_t)(struct cdrom_fsg_dev *);
static int cdrom_send_status(struct cdrom_fsg_common *common);

static int cdrom_exception_in_progress(struct cdrom_fsg_common *common)
{
	return common->state > FSG_STATE_IDLE;
}

/* Make bulk-out requests be divisible by the maxpacket size */
static void cdrom_set_bulk_out_req_length(struct cdrom_fsg_common *common,
		struct fsg_buffhd *bh, unsigned int length)
{
	unsigned int	rem;

	bh->bulk_out_intended_length = length;
	rem = length % common->bulk_out_maxpacket;
	if (rem > 0)
		length += common->bulk_out_maxpacket - rem;
	bh->outreq->length = length;
}


static int cdrom_fsg_set_halt(struct cdrom_fsg_dev *fsg, struct usb_ep *ep)
{
	const char	*name;

	if (ep == fsg->bulk_in)
		name = "bulk-in";
	else if (ep == fsg->bulk_out)
		name = "bulk-out";
	else
		name = ep->name;
	DBG(fsg, "%s set halt\n", name);
	return usb_ep_set_halt(ep);
}




static void cdrom_wakeup_thread(struct cdrom_fsg_common *common)
{
	common->thread_wakeup_needed = 1;
	if (common->thread_task)
		wake_up_process(common->thread_task);
}


static void cdrom_raise_exception(struct cdrom_fsg_common *common, enum fsg_state new_state)
{
	unsigned long		flags;

	spin_lock_irqsave(&common->lock, flags);
	if (common->state <= new_state) {
		common->exception_req_tag = common->ep0_req_tag;
		common->state = new_state;
		if (common->thread_task)
			send_sig_info(SIGUSR1, SEND_SIG_FORCED,
				      common->thread_task);
	}
	spin_unlock_irqrestore(&common->lock, flags);
}



static int cdrom_ep0_queue(struct cdrom_fsg_common *common)
{
	int	rc;

	rc = usb_ep_queue(common->ep0, common->ep0req, GFP_ATOMIC);
	common->ep0->driver_data = common;
	if (rc != 0 && rc != -ESHUTDOWN) {
		WARNING(common, "error in submission: %s --> %d\n",
			common->ep0->name, rc);
	}
	return rc;
}



static void cdrom_bulk_in_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct cdrom_fsg_common	*common = ep->driver_data;
	struct fsg_buffhd	*bh = req->context;

	if (req->status || req->actual != req->length)
		DBG(common, "%s --> %d, %u/%u\n", __func__,
				req->status, req->actual, req->length);
	if (req->status == -ECONNRESET)
		usb_ep_fifo_flush(ep);

	smp_wmb();
	spin_lock(&common->lock);
	bh->inreq_busy = 0;
	bh->state = BUF_STATE_EMPTY;
	cdrom_wakeup_thread(common);
	spin_unlock(&common->lock);
}

static void cdrom_bulk_out_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct cdrom_fsg_common	*common = ep->driver_data;
	struct fsg_buffhd	*bh = req->context;

	dump_msg(common, "bulk-out", req->buf, req->actual);
	if (req->status || req->actual != bh->bulk_out_intended_length)
		DBG(common, "%s --> %d, %u/%u\n", __func__,
				req->status, req->actual,
				bh->bulk_out_intended_length);
	if (req->status == -ECONNRESET)
		usb_ep_fifo_flush(ep);

	smp_wmb();
	spin_lock(&common->lock);
	bh->outreq_busy = 0;
	bh->state = BUF_STATE_FULL;
	cdrom_wakeup_thread(common);
	spin_unlock(&common->lock);
}




static int cdrom_fsg_setup(struct usb_function *f,
		const struct usb_ctrlrequest *ctrl)
{
	struct cdrom_fsg_dev		*fsg = cdrom_fsg_from_func(f);
	struct usb_request	*req = fsg->common->ep0req;
	u16			w_index = le16_to_cpu(ctrl->wIndex);
	u16			w_value = le16_to_cpu(ctrl->wValue);
	u16			w_length = le16_to_cpu(ctrl->wLength);

	if (!cdrom_fsg_is_set(fsg->common))
		return -EOPNOTSUPP;

	switch (ctrl->bRequest) {

	case USB_BULK_RESET_REQUEST:
		if (ctrl->bRequestType !=
		    (USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE))
			break;
		if (w_value != 0)
			return -EDOM;

		DBG(fsg, "bulk reset request\n");
		cdrom_raise_exception(fsg->common, FSG_STATE_RESET);
		return DELAYED_STATUS;

	case USB_BULK_GET_MAX_LUN_REQUEST:
		if (ctrl->bRequestType !=
		    (USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE))
			break;
		if (w_value != 0)
			return -EDOM;
		VDBG(fsg, "get max LUN\n");
		*(u8 *) req->buf = fsg->common->nluns - 1;

		req->length = min((u16)1, w_length);
		fsg->common->ep0req_name =
			ctrl->bRequestType & USB_DIR_IN ? "ep0-in" : "ep0-out";
		return cdrom_ep0_queue(fsg->common);
	}

	VDBG(fsg,
	     "unknown class-specific control req "
	     "%02x.%02x v%04x i%04x l%u\n",
	     ctrl->bRequestType, ctrl->bRequest,
	     le16_to_cpu(ctrl->wValue), w_index, w_length);
	return -EOPNOTSUPP;
}





static void cdrom_start_transfer(struct cdrom_fsg_dev *fsg, struct usb_ep *ep,
		struct usb_request *req, int *pbusy,
		enum fsg_buffer_state *state)
{
	int	rc;

	if (ep == fsg->bulk_in)
		dump_msg(fsg, "bulk-in", req->buf, req->length);

	spin_lock_irq(&fsg->common->lock);
	*pbusy = 1;
	*state = BUF_STATE_BUSY;
	spin_unlock_irq(&fsg->common->lock);
	rc = usb_ep_queue(ep, req, GFP_KERNEL);
	if (rc != 0) {
		*pbusy = 0;
		*state = BUF_STATE_EMPTY;

		if (rc != -ESHUTDOWN && !(rc == -EOPNOTSUPP &&
						req->length == 0))
			WARNING(fsg, "error in submission: %s --> %d\n",
					ep->name, rc);
	}
}

#define CDROM_START_TRANSFER_OR(common, ep_name, req, pbusy, state)		\
	if (cdrom_fsg_is_set(common))						\
		cdrom_start_transfer((common)->fsg, (common)->fsg->ep_name,	\
			       req, pbusy, state);			\
	else

#define CDROM_START_TRANSFER(common, ep_name, req, pbusy, state)		\
	CDROM_START_TRANSFER_OR(common, ep_name, req, pbusy, state) (void)0



static int cdrom_sleep_thread(struct cdrom_fsg_common *common)
{
	int	rc = 0;

	for (;;) {
		try_to_freeze();
		set_current_state(TASK_INTERRUPTIBLE);
		if (signal_pending(current)) {
			rc = -EINTR;
			break;
		}
		if (common->thread_wakeup_needed)
			break;
		schedule();
	}
	__set_current_state(TASK_RUNNING);
	common->thread_wakeup_needed = 0;
	return rc;
}



static int cdrom_do_read(struct cdrom_fsg_common *common)
{
	struct fsg_lun		*curlun = common->curlun;
	u32			lba;
	struct fsg_buffhd	*bh;
	int			rc;
	u32			amount_left;
	loff_t			file_offset, file_offset_tmp;
	unsigned int		amount;
	unsigned int		partial_page;
	ssize_t			nread;

	if (common->cmnd[0] == READ_6)
		lba = get_unaligned_be24(&common->cmnd[1]);
	else {
		lba = get_unaligned_be32(&common->cmnd[2]);

		if ((common->cmnd[1] & ~0x18) != 0) {
			curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
			return -EINVAL;
		}
	}
	if (lba >= curlun->num_sectors) {
		curlun->sense_data = SS_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE;
		return -EINVAL;
	}
	file_offset = ((loff_t) lba) << 9;

	amount_left = common->data_size_from_cmnd;
	if (unlikely(amount_left == 0))
		return -EIO;

	for (;;) {

		amount = min(amount_left, FSG_BUFLEN);
		amount = min((loff_t) amount,
				curlun->file_length - file_offset);
		partial_page = file_offset & (PAGE_CACHE_SIZE - 1);
		if (partial_page > 0)
			amount = min(amount, (unsigned int) PAGE_CACHE_SIZE -
					partial_page);

		bh = common->next_buffhd_to_fill;
		while (bh->state != BUF_STATE_EMPTY) {
			rc = cdrom_sleep_thread(common);
			if (rc)
				return rc;
		}

		if (amount == 0) {
			curlun->sense_data =
					SS_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE;
			curlun->sense_data_info = file_offset >> 9;
			curlun->info_valid = 1;
			bh->inreq->length = 0;
			bh->state = BUF_STATE_FULL;
			break;
		}

		file_offset_tmp = file_offset;
		nread = vfs_read(curlun->filp,
				(char __user *) bh->buf,
				amount, &file_offset_tmp);
		VLDBG(curlun, "file read %u @ %llu -> %d\n", amount,
				(unsigned long long) file_offset,
				(int) nread);
		if (signal_pending(current))
			return -EINTR;

		if (nread < 0) {
			LDBG(curlun, "error in file read: %d\n",
					(int) nread);
			nread = 0;
		} else if (nread < amount) {
			LDBG(curlun, "partial file read: %d/%u\n",
					(int) nread, amount);
			nread -= (nread & 511);
		}
		file_offset  += nread;
		amount_left  -= nread;
		common->residue -= nread;
		bh->inreq->length = nread;
		bh->state = BUF_STATE_FULL;

		if (nread < amount) {
			curlun->sense_data = SS_UNRECOVERED_READ_ERROR;
			curlun->sense_data_info = file_offset >> 9;
			curlun->info_valid = 1;
			break;
		}

		if (amount_left == 0)
			break;

		bh->inreq->zero = 0;
		CDROM_START_TRANSFER_OR(common, bulk_in, bh->inreq,
			       &bh->inreq_busy, &bh->state)
			return -EIO;
		common->next_buffhd_to_fill = bh->next;
	}

	return -EIO;
}



static int cdrom_do_write(struct cdrom_fsg_common *common)
{
	struct fsg_lun		*curlun = common->curlun;
	u32			lba;
	struct fsg_buffhd	*bh;
	int			get_some_more;
	u32			amount_left_to_req, amount_left_to_write;
	loff_t			usb_offset, file_offset, file_offset_tmp;
	unsigned int		amount;
	unsigned int		partial_page;
	ssize_t			nwritten;
	int			rc;

#ifdef CONFIG_USB_CSW_HACK
	int			csw_hack_sent = 0;
	int			i;
#endif
	if (curlun->ro) {
		curlun->sense_data = SS_WRITE_PROTECTED;
		return -EINVAL;
	}
	spin_lock(&curlun->filp->f_lock);
	curlun->filp->f_flags &= ~O_SYNC;
	spin_unlock(&curlun->filp->f_lock);

	if (common->cmnd[0] == WRITE_6)
		lba = get_unaligned_be24(&common->cmnd[1]);
	else {
		lba = get_unaligned_be32(&common->cmnd[2]);

		if (common->cmnd[1] & ~0x18) {
			curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
			return -EINVAL;
		}
		if (common->cmnd[1] & 0x08) {
			spin_lock(&curlun->filp->f_lock);
			curlun->filp->f_flags |= O_SYNC;
			spin_unlock(&curlun->filp->f_lock);
		}
	}
	if (lba >= curlun->num_sectors) {
		curlun->sense_data = SS_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE;
		return -EINVAL;
	}

	get_some_more = 1;
	file_offset = usb_offset = ((loff_t) lba) << 9;
	amount_left_to_req = common->data_size_from_cmnd;
	amount_left_to_write = common->data_size_from_cmnd;

	while (amount_left_to_write > 0) {

		bh = common->next_buffhd_to_fill;
		if (bh->state == BUF_STATE_EMPTY && get_some_more) {

			amount = min(amount_left_to_req, FSG_BUFLEN);
			amount = min((loff_t) amount, curlun->file_length -
					usb_offset);
			partial_page = usb_offset & (PAGE_CACHE_SIZE - 1);
			if (partial_page > 0)
				amount = min(amount,
	(unsigned int) PAGE_CACHE_SIZE - partial_page);

			if (amount == 0) {
				get_some_more = 0;
				curlun->sense_data =
					SS_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE;
				curlun->sense_data_info = usb_offset >> 9;
				curlun->info_valid = 1;
				continue;
			}
			amount -= (amount & 511);
			if (amount == 0) {

				get_some_more = 0;
				continue;
			}

			usb_offset += amount;
			common->usb_amount_left -= amount;
			amount_left_to_req -= amount;
			if (amount_left_to_req == 0)
				get_some_more = 0;

			bh->outreq->length = amount;
			bh->bulk_out_intended_length = amount;
			bh->outreq->short_not_ok = 1;
			CDROM_START_TRANSFER_OR(common, bulk_out, bh->outreq,
					  &bh->outreq_busy, &bh->state)
				return -EIO;
			common->next_buffhd_to_fill = bh->next;
			continue;
		}

		bh = common->next_buffhd_to_drain;
		if (bh->state == BUF_STATE_EMPTY && !get_some_more)
			break;
#ifdef CONFIG_USB_CSW_HACK
		if (bh->state == BUF_STATE_FULL && common->residue) {
#else
		if (bh->state == BUF_STATE_FULL) {
#endif
			smp_rmb();
			common->next_buffhd_to_drain = bh->next;
			bh->state = BUF_STATE_EMPTY;

			if (bh->outreq->status != 0) {
				curlun->sense_data = SS_COMMUNICATION_FAILURE;
				curlun->sense_data_info = file_offset >> 9;
				curlun->info_valid = 1;
				break;
			}

			amount = bh->outreq->actual;
			if (curlun->file_length - file_offset < amount) {
				LERROR(curlun,
	"write %u @ %llu beyond end %llu\n",
	amount, (unsigned long long) file_offset,
	(unsigned long long) curlun->file_length);
				amount = curlun->file_length - file_offset;
			}

			file_offset_tmp = file_offset;
			nwritten = vfs_write(curlun->filp,
					(char __user *) bh->buf,
					amount, &file_offset_tmp);
			VLDBG(curlun, "file write %u @ %llu -> %d\n", amount,
					(unsigned long long) file_offset,
					(int) nwritten);
			if (signal_pending(current))
				return -EINTR;

			if (nwritten < 0) {
				LDBG(curlun, "error in file write: %d\n",
						(int) nwritten);
				nwritten = 0;
			} else if (nwritten < amount) {
				LDBG(curlun, "partial file write: %d/%u\n",
						(int) nwritten, amount);
				nwritten -= (nwritten & 511);
			}
			file_offset += nwritten;
			amount_left_to_write -= nwritten;
			common->residue -= nwritten;

			if (nwritten < amount) {
				curlun->sense_data = SS_WRITE_ERROR;
				curlun->sense_data_info = file_offset >> 9;
				curlun->info_valid = 1;
#ifdef CONFIG_USB_CSW_HACK
				write_error_after_csw_sent = 1;
				goto write_error;
#endif
				break;
			}

#ifdef CONFIG_USB_CSW_HACK
write_error:
			if ((nwritten == amount) && !csw_hack_sent) {
				if (write_error_after_csw_sent)
					break;
				for (i = 0; i < FSG_NUM_BUFFERS; i++) {
					if (common->buffhds[i].state ==
							BUF_STATE_BUSY)
						break;
				}
				if (!amount_left_to_req && i == FSG_NUM_BUFFERS) {
					csw_hack_sent = 1;
					cdrom_send_status(common);
				}
			}
#endif
			if (bh->outreq->actual != bh->outreq->length) {
				common->short_packet_received = 1;
				break;
			}
			continue;
		}

		rc = cdrom_sleep_thread(common);
		if (rc)
			return rc;
	}

	return -EIO;
}



static int cdrom_do_synchronize_cache(struct cdrom_fsg_common *common)
{
	struct fsg_lun	*curlun = common->curlun;
	int		rc;

	rc = fsg_lun_fsync_sub(curlun);
	if (rc)
		curlun->sense_data = SS_WRITE_ERROR;
	return 0;
}



static void cdrom_invalidate_sub(struct fsg_lun *curlun)
{
	struct file	*filp = curlun->filp;
	struct inode	*inode = filp->f_path.dentry->d_inode;
	unsigned long	rc;

	rc = invalidate_mapping_pages(inode->i_mapping, 0, -1);
	VLDBG(curlun, "invalidate_mapping_pages -> %ld\n", rc);
}

static int cdrom_do_verify(struct cdrom_fsg_common *common)
{
	struct fsg_lun		*curlun = common->curlun;
	u32			lba;
	u32			verification_length;
	struct fsg_buffhd	*bh = common->next_buffhd_to_fill;
	loff_t			file_offset, file_offset_tmp;
	u32			amount_left;
	unsigned int		amount;
	ssize_t			nread;

	lba = get_unaligned_be32(&common->cmnd[2]);
	if (lba >= curlun->num_sectors) {
		curlun->sense_data = SS_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE;
		return -EINVAL;
	}

	if (common->cmnd[1] & ~0x10) {
		curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
		return -EINVAL;
	}

	verification_length = get_unaligned_be16(&common->cmnd[7]);
	if (unlikely(verification_length == 0))
		return -EIO;

	amount_left = verification_length << 9;
	file_offset = ((loff_t) lba) << 9;

	fsg_lun_fsync_sub(curlun);
	if (signal_pending(current))
		return -EINTR;

	cdrom_invalidate_sub(curlun);
	if (signal_pending(current))
		return -EINTR;

	while (amount_left > 0) {

		amount = min(amount_left, FSG_BUFLEN);
		amount = min((loff_t) amount,
				curlun->file_length - file_offset);
		if (amount == 0) {
			curlun->sense_data =
					SS_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE;
			curlun->sense_data_info = file_offset >> 9;
			curlun->info_valid = 1;
			break;
		}

		file_offset_tmp = file_offset;
		nread = vfs_read(curlun->filp,
				(char __user *) bh->buf,
				amount, &file_offset_tmp);
		VLDBG(curlun, "file read %u @ %llu -> %d\n", amount,
				(unsigned long long) file_offset,
				(int) nread);
		if (signal_pending(current))
			return -EINTR;

		if (nread < 0) {
			LDBG(curlun, "error in file verify: %d\n",
					(int) nread);
			nread = 0;
		} else if (nread < amount) {
			LDBG(curlun, "partial file verify: %d/%u\n",
					(int) nread, amount);
			nread -= (nread & 511);
		}
		if (nread == 0) {
			curlun->sense_data = SS_UNRECOVERED_READ_ERROR;
			curlun->sense_data_info = file_offset >> 9;
			curlun->info_valid = 1;
			break;
		}
		file_offset += nread;
		amount_left -= nread;
	}
	return 0;
}



static int cdrom_do_inquiry(struct cdrom_fsg_common *common, struct fsg_buffhd *bh)
{
	struct fsg_lun *curlun = common->curlun;
	u8	*buf = (u8 *) bh->buf;

	if (!curlun) {
		common->bad_lun_okay = 1;
		memset(buf, 0, 36);
		buf[0] = 0x7f;
		buf[4] = 31;
		return 36;
	}

	buf[0] = curlun->cdrom ? TYPE_ROM : TYPE_DISK;
	buf[1] = curlun->removable ? 0x80 : 0;
	buf[2] = 2;
	buf[3] = 2;
	buf[4] = 31;
	buf[5] = 0;
	buf[6] = 0;
	buf[7] = 0;
	memcpy(buf + 8, common->inquiry_string, sizeof common->inquiry_string);
	return 36;
}

static int cdrom_do_ack_status(struct cdrom_fsg_common *common, struct fsg_buffhd *bh, u8 ack)
{
	u8	*buf = (u8 *) bh->buf;

	if (!common->curlun) {
		common->bad_lun_okay = 1;
		memset(buf, 0, 1);
		buf[0] = 0xf;
		return 1;
	}

	buf[0] = ack;
	printk("cdrom_do_ack_status ACK = %d\r\n", ack);
	return 1;
}

#ifndef CONFIG_LGE_USB_GADGET_DRIVER
static int do_get_sw_rev(struct cdrom_fsg_common *common, struct fsg_buffhd *bh)
{
	u8	*buf = (u8 *) bh->buf;

	memset(buf, 0, 7);

	buf[0] = 2;
	buf[1] = 1;
	buf[5] = 1;
	buf[6] = 2;
	return 7;
}

static int do_get_serial(struct cdrom_fsg_common *common, struct fsg_buffhd *bh)
{
	u8	*buf = (u8 *) bh->buf;

	memset(buf, 0, 7);

	buf[0] = 3;
	buf[1] = 1;
	buf[5] = 1;
	buf[6] = 3;
	return 7;
}

static int do_get_model(struct cdrom_fsg_common *common, struct fsg_buffhd *bh)
{
	u8	*buf = (u8 *) bh->buf;

	memset(buf, 0, 7);

	buf[0] = 4;
	buf[1] = 1;
	buf[5] = 1;
	buf[6] = 4;
	return 7;
}

static int do_get_sub_ver(struct cdrom_fsg_common *common, struct fsg_buffhd *bh)
{
	u8	*buf = (u8 *) bh->buf;

	memset(buf, 0, 7);

	buf[0] = 5;
	buf[1] = 1;
	buf[5] = 1;
	buf[6] = 5;
	return 7;
}
#endif

static int cdrom_do_request_sense(struct cdrom_fsg_common *common, struct fsg_buffhd *bh)
{
	struct fsg_lun	*curlun = common->curlun;
	u8		*buf = (u8 *) bh->buf;
	u32		sd, sdinfo;
	int		valid;

	/*
	 * From the SCSI-2 spec., section 7.9 (Unit attention condition):
	 *
	 * If a REQUEST SENSE command is received from an initiator
	 * with a pending unit attention condition (before the target
	 * generates the contingent allegiance condition), then the
	 * target shall either:
	 *   a) report any pending sense data and preserve the unit
	 *	attention condition on the logical unit, or,
	 *   b) report the unit attention condition, may discard any
	 *	pending sense data, and clear the unit attention
	 *	condition on the logical unit for that initiator.
	 *
	 * FSG normally uses option a); enable this code to use option b).
	 */
#ifndef CONFIG_LGE_USB_GADGET_DRIVER
	if (curlun && curlun->unit_attention_data != SS_NO_SENSE) {
		curlun->sense_data = curlun->unit_attention_data;
		curlun->unit_attention_data = SS_NO_SENSE;
	}
#endif

	if (!curlun) {
		common->bad_lun_okay = 1;
		sd = SS_LOGICAL_UNIT_NOT_SUPPORTED;
		sdinfo = 0;
		valid = 0;
	} else {
		sd = curlun->sense_data;
		sdinfo = curlun->sense_data_info;
		valid = curlun->info_valid << 7;
		curlun->sense_data = SS_NO_SENSE;
		curlun->sense_data_info = 0;
		curlun->info_valid = 0;
	}

	memset(buf, 0, 18);
	buf[0] = valid | 0x70;
	buf[2] = SK(sd);
	put_unaligned_be32(sdinfo, &buf[3]);
	buf[7] = 18 - 8;
	buf[12] = ASC(sd);
	buf[13] = ASCQ(sd);
	return 18;
}


static int cdrom_do_read_capacity(struct cdrom_fsg_common *common, struct fsg_buffhd *bh)
{
	struct fsg_lun	*curlun = common->curlun;
	u32		lba = get_unaligned_be32(&common->cmnd[2]);
	int		pmi = common->cmnd[8];
	u8		*buf = (u8 *) bh->buf;

	if (pmi > 1 || (pmi == 0 && lba != 0)) {
		curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
		return -EINVAL;
	}

	put_unaligned_be32(curlun->num_sectors - 1, &buf[0]);
	put_unaligned_be32(512, &buf[4]);
	return 8;
}


static int cdrom_do_read_header(struct cdrom_fsg_common *common, struct fsg_buffhd *bh)
{
	struct fsg_lun	*curlun = common->curlun;
	int		msf = common->cmnd[1] & 0x02;
	u32		lba = get_unaligned_be32(&common->cmnd[2]);
	u8		*buf = (u8 *) bh->buf;

	if (common->cmnd[1] & ~0x02) {
		curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
		return -EINVAL;
	}
	if (lba >= curlun->num_sectors) {
		curlun->sense_data = SS_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE;
		return -EINVAL;
	}

	memset(buf, 0, 8);
	buf[0] = 0x01;
	store_cdrom_address(&buf[4], msf, lba);
	return 8;
}


static int cdrom_do_read_toc(struct cdrom_fsg_common *common, struct fsg_buffhd *bh)
{
	struct fsg_lun	*curlun = common->curlun;
	int		msf = common->cmnd[1] & 0x02;
	int		start_track = common->cmnd[6];
	u8		*buf = (u8 *) bh->buf;

	if ((common->cmnd[1] & ~0x02) != 0 ||
			start_track > 1) {
		curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
		return -EINVAL;
	}

	memset(buf, 0, 20);
	buf[1] = (20-2);
	buf[2] = 1;
	buf[3] = 1;
	buf[5] = 0x16;
	buf[6] = 0x01;
	store_cdrom_address(&buf[8], msf, 0);

	buf[13] = 0x16;
	buf[14] = 0xAA;
	store_cdrom_address(&buf[16], msf, curlun->num_sectors);
	return 20;
}


static int cdrom_do_mode_sense(struct cdrom_fsg_common *common, struct fsg_buffhd *bh)
{
	struct fsg_lun	*curlun = common->curlun;
	int		mscmnd = common->cmnd[0];
	u8		*buf = (u8 *) bh->buf;
	u8		*buf0 = buf;
	int		pc, page_code;
	int		changeable_values, all_pages;
	int		valid_page = 0;
	int		len, limit;

	if ((common->cmnd[1] & ~0x08) != 0) {
		curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
		return -EINVAL;
	}
	pc = common->cmnd[2] >> 6;
	page_code = common->cmnd[2] & 0x3f;
	if (pc == 3) {
		curlun->sense_data = SS_SAVING_PARAMETERS_NOT_SUPPORTED;
		return -EINVAL;
	}
	changeable_values = (pc == 1);
	all_pages = (page_code == 0x3f);

	memset(buf, 0, 8);
	if (mscmnd == MODE_SENSE) {
		buf[2] = (curlun->ro ? 0x80 : 0x00);
		buf += 4;
		limit = 255;
	} else {
		buf[3] = (curlun->ro ? 0x80 : 0x00);
		buf += 8;
		limit = 65535;
	}


	if (page_code == 0x08 || all_pages) {
		valid_page = 1;
		buf[0] = 0x08;
		buf[1] = 10;
		memset(buf+2, 0, 10);

		if (!changeable_values) {
			buf[2] = 0x04;
			put_unaligned_be16(0xffff, &buf[4]);
			put_unaligned_be16(0xffff, &buf[8]);
			put_unaligned_be16(0xffff, &buf[10]);
		}
		buf += 12;
	}

	len = buf - buf0;
	if (!valid_page || len > limit) {
		curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
		return -EINVAL;
	}

	if (mscmnd == MODE_SENSE)
		buf0[0] = len - 1;
	else
		put_unaligned_be16(len - 2, buf0);
	return len;
}


static int cdrom_do_start_stop(struct cdrom_fsg_common *common)
{
	struct fsg_lun	*curlun = common->curlun;
	int		loej, start;

	if (!curlun) {
		return -EINVAL;
	} else if (!curlun->removable) {
		curlun->sense_data = SS_INVALID_COMMAND;
		return -EINVAL;
	}

	loej = common->cmnd[4] & 0x02;
	start = common->cmnd[4] & 0x01;


	if ((common->cmnd[1] & ~0x01) != 0 ||
		(common->cmnd[4] & ~0x03) != 0) {
		curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
		return -EINVAL;
	}

	if (!start) {
		if (curlun->prevent_medium_removal) {
			LDBG(curlun, "unload attempt prevented\n");
			curlun->sense_data = SS_MEDIUM_REMOVAL_PREVENTED;
			return -EINVAL;
		}
		if (loej) {
			up_read(&common->filesem);
			down_write(&common->filesem);
			fsg_lun_close(curlun);
			up_write(&common->filesem);
			down_read(&common->filesem);
		}
	} else {

		if (!fsg_lun_is_open(curlun)) {
			curlun->sense_data = SS_MEDIUM_NOT_PRESENT;
			return -EINVAL;
		}
	}
	return 0;
}


static int cdrom_do_prevent_allow(struct cdrom_fsg_common *common)
{
	struct fsg_lun	*curlun = common->curlun;
	int		prevent;

	if (!common->curlun) {
		return -EINVAL;
	} else if (!common->curlun->removable) {
		common->curlun->sense_data = SS_INVALID_COMMAND;
		return -EINVAL;
	}

	prevent = common->cmnd[4] & 0x01;
	if ((common->cmnd[4] & ~0x01) != 0) {
		curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
		return -EINVAL;
	}

	if (curlun->prevent_medium_removal && !prevent)
		fsg_lun_fsync_sub(curlun);
	curlun->prevent_medium_removal = prevent;
	return 0;
}


static int cdrom_do_read_format_capacities(struct cdrom_fsg_common *common,
			struct fsg_buffhd *bh)
{
	struct fsg_lun	*curlun = common->curlun;
	u8		*buf = (u8 *) bh->buf;

	buf[0] = buf[1] = buf[2] = 0;
	buf[3] = 8;
	buf += 4;

	put_unaligned_be32(curlun->num_sectors, &buf[0]);
	put_unaligned_be32(512, &buf[4]);
	buf[4] = 0x02;
	return 12;
}


static int cdrom_do_mode_select(struct cdrom_fsg_common *common, struct fsg_buffhd *bh)
{
	struct fsg_lun	*curlun = common->curlun;

	if (curlun)
		curlun->sense_data = SS_INVALID_COMMAND;
	return -EINVAL;
}



static int cdrom_halt_bulk_in_endpoint(struct cdrom_fsg_dev *fsg)
{
	int	rc;

	rc = cdrom_fsg_set_halt(fsg, fsg->bulk_in);
	if (rc == -EAGAIN)
		VDBG(fsg, "delayed bulk-in endpoint halt\n");
	while (rc != 0) {
		if (rc != -EAGAIN) {
			WARNING(fsg, "usb_ep_set_halt -> %d\n", rc);
			rc = 0;
			break;
		}

		if (msleep_interruptible(100) != 0)
			return -EINTR;
		rc = usb_ep_set_halt(fsg->bulk_in);
	}
	return rc;
}

static int cdrom_wedge_bulk_in_endpoint(struct cdrom_fsg_dev *fsg)
{
	int	rc;

	DBG(fsg, "bulk-in set wedge\n");
	rc = usb_ep_set_wedge(fsg->bulk_in);
	if (rc == -EAGAIN)
		VDBG(fsg, "delayed bulk-in endpoint wedge\n");
	while (rc != 0) {
		if (rc != -EAGAIN) {
			WARNING(fsg, "usb_ep_set_wedge -> %d\n", rc);
			rc = 0;
			break;
		}

		if (msleep_interruptible(100) != 0)
			return -EINTR;
		rc = usb_ep_set_wedge(fsg->bulk_in);
	}
	return rc;
}

static int cdrom_pad_with_zeros(struct cdrom_fsg_dev *fsg)
{
	struct fsg_buffhd	*bh = fsg->common->next_buffhd_to_fill;
	u32			nkeep = bh->inreq->length;
	u32			nsend;
	int			rc;

	bh->state = BUF_STATE_EMPTY;
	fsg->common->usb_amount_left = nkeep + fsg->common->residue;
	while (fsg->common->usb_amount_left > 0) {

		while (bh->state != BUF_STATE_EMPTY) {
			rc = cdrom_sleep_thread(fsg->common);
			if (rc)
				return rc;
		}

		nsend = min(fsg->common->usb_amount_left, FSG_BUFLEN);
		memset(bh->buf + nkeep, 0, nsend - nkeep);
		bh->inreq->length = nsend;
		bh->inreq->zero = 0;
		cdrom_start_transfer(fsg, fsg->bulk_in, bh->inreq,
				&bh->inreq_busy, &bh->state);
		bh = fsg->common->next_buffhd_to_fill = bh->next;
		fsg->common->usb_amount_left -= nsend;
		nkeep = 0;
	}
	return 0;
}

static int cdrom_throw_away_data(struct cdrom_fsg_common *common)
{
	struct fsg_buffhd	*bh;
	u32			amount;
	int			rc;

	for (bh = common->next_buffhd_to_drain;
	     bh->state != BUF_STATE_EMPTY || common->usb_amount_left > 0;
	     bh = common->next_buffhd_to_drain) {

		if (bh->state == BUF_STATE_FULL) {
			smp_rmb();
			bh->state = BUF_STATE_EMPTY;
			common->next_buffhd_to_drain = bh->next;

			if (bh->outreq->actual != bh->outreq->length ||
					bh->outreq->status != 0) {
				cdrom_raise_exception(common,
						FSG_STATE_ABORT_BULK_OUT);
				return -EINTR;
			}
			continue;
		}

		bh = common->next_buffhd_to_fill;
		if (bh->state == BUF_STATE_EMPTY
		 && common->usb_amount_left > 0) {
			amount = min(common->usb_amount_left, FSG_BUFLEN);

			bh->outreq->length = amount;
			bh->bulk_out_intended_length = amount;
			bh->outreq->short_not_ok = 1;
			CDROM_START_TRANSFER_OR(common, bulk_out, bh->outreq,
					  &bh->outreq_busy, &bh->state)
				return -EIO;
			common->next_buffhd_to_fill = bh->next;
			common->usb_amount_left -= amount;
			continue;
		}

		rc = cdrom_sleep_thread(common);
		if (rc)
			return rc;
	}
	return 0;
}


static int cdrom_finish_reply(struct cdrom_fsg_common *common)
{
	struct fsg_buffhd	*bh = common->next_buffhd_to_fill;
	int			rc = 0;

	switch (common->data_dir) {
	case DATA_DIR_NONE:
		break;

	case DATA_DIR_UNKNOWN:
		if (!common->can_stall) {
		} else if (cdrom_fsg_is_set(common)) {
			cdrom_fsg_set_halt(common->fsg, common->fsg->bulk_out);
			rc = cdrom_halt_bulk_in_endpoint(common->fsg);
		} else {
			rc = -EIO;
		}
		break;

	case DATA_DIR_TO_HOST:
		if (common->data_size == 0) {

		} else if (common->residue == 0) {
			bh->inreq->zero = 0;
			CDROM_START_TRANSFER_OR(common, bulk_in, bh->inreq,
					  &bh->inreq_busy, &bh->state)
				return -EIO;
			common->next_buffhd_to_fill = bh->next;

		} else if (common->can_stall) {
			bh->inreq->zero = 1;
			CDROM_START_TRANSFER_OR(common, bulk_in, bh->inreq,
					  &bh->inreq_busy, &bh->state)
				rc = -EIO;
			common->next_buffhd_to_fill = bh->next;
			if (common->fsg)
				rc = cdrom_halt_bulk_in_endpoint(common->fsg);
		} else if (cdrom_fsg_is_set(common)) {
			rc = cdrom_pad_with_zeros(common->fsg);
		} else {
			rc = -EIO;
		}
		break;

	case DATA_DIR_FROM_HOST:
		if (common->residue == 0) {

		} else if (common->short_packet_received) {
			cdrom_raise_exception(common, FSG_STATE_ABORT_BULK_OUT);
			rc = -EINTR;

#ifndef CONFIG_LGE_USB_GADGET_DRIVER
		} else if (common->can_stall) {
			if (cdrom_fsg_is_set(common))
				cdrom_fsg_set_halt(common->fsg,
					     common->fsg->bulk_out);
			cdrom_raise_exception(common, FSG_STATE_ABORT_BULK_OUT);
			rc = -EINTR;
#endif

		} else {
			rc = cdrom_throw_away_data(common);
		}
		break;
	}
	return rc;
}


static int cdrom_send_status(struct cdrom_fsg_common *common)
{
	struct fsg_lun		*curlun = common->curlun;
	struct fsg_buffhd	*bh;
	struct bulk_cs_wrap	*csw;
	int			rc;
	u8			status = USB_STATUS_PASS;
	u32			sd, sdinfo = 0;

	bh = common->next_buffhd_to_fill;
	while (bh->state != BUF_STATE_EMPTY) {
		rc = cdrom_sleep_thread(common);
		if (rc)
			return rc;
	}

	if (curlun) {
		sd = curlun->sense_data;
		sdinfo = curlun->sense_data_info;
	} else if (common->bad_lun_okay)
		sd = SS_NO_SENSE;
	else
		sd = SS_LOGICAL_UNIT_NOT_SUPPORTED;

	if (common->phase_error) {
		DBG(common, "sending phase-error status\n");
		status = USB_STATUS_PHASE_ERROR;
		sd = SS_INVALID_COMMAND;
	} else if (sd != SS_NO_SENSE) {
		DBG(common, "sending command-failure status\n");
		status = USB_STATUS_FAIL;
		VDBG(common, "  sense data: SK x%02x, ASC x%02x, ASCQ x%02x;"
				"  info x%x\n",
				SK(sd), ASC(sd), ASCQ(sd), sdinfo);
	}

	csw = (void *)bh->buf;

	csw->Signature = cpu_to_le32(USB_BULK_CS_SIG);
	csw->Tag = common->tag;
	csw->Residue = cpu_to_le32(common->residue);
#ifdef CONFIG_USB_CSW_HACK
	if (write_error_after_csw_sent) {
		write_error_after_csw_sent = 0;
		csw->Residue = cpu_to_le32(common->residue);
	} else
		csw->Residue = 0;
#else
	csw->Residue = cpu_to_le32(common->residue);
#endif
	csw->Status = status;

	bh->inreq->length = USB_BULK_CS_WRAP_LEN;
	bh->inreq->zero = 0;
	CDROM_START_TRANSFER_OR(common, bulk_in, bh->inreq,
			  &bh->inreq_busy, &bh->state)
		return -EIO;

	common->next_buffhd_to_fill = bh->next;
	return 0;
}



static int cdrom_check_command(struct cdrom_fsg_common *common, int cmnd_size,
		enum data_direction data_dir, unsigned int mask,
		int needs_medium, const char *name)
{
	int			i;
	int			lun = common->cmnd[1] >> 5;
	static const char	dirletter[4] = {'u', 'o', 'i', 'n'};
	char			hdlen[20];
	struct fsg_lun		*curlun;
	int	lun_value = 0;

	hdlen[0] = 0;
	if (common->data_dir != DATA_DIR_UNKNOWN)
		sprintf(hdlen, ", H%c=%u", dirletter[(int) common->data_dir],
				common->data_size);
	VDBG(common, "SCSI command: %s;  Dc=%d, D%c=%u;  Hc=%d%s\n",
	     name, cmnd_size, dirletter[(int) data_dir],
	     common->data_size_from_cmnd, common->cmnd_size, hdlen);

	if (common->data_size_from_cmnd == 0)
		data_dir = DATA_DIR_NONE;
	if (common->data_size < common->data_size_from_cmnd) {
		common->data_size_from_cmnd = common->data_size;
		common->phase_error = 1;
	}
	common->residue = common->data_size;
	common->usb_amount_left = common->data_size;

	if (common->data_dir != data_dir
	 && common->data_size_from_cmnd > 0) {
		common->phase_error = 1;
		return -EINVAL;
	}

	if (cmnd_size != common->cmnd_size) {

		if (cmnd_size <= common->cmnd_size) {
			DBG(common, "%s is buggy! Expected length %d "
			    "but we got %d\n", name,
			    cmnd_size, common->cmnd_size);
			cmnd_size = common->cmnd_size;
		} else {
			common->phase_error = 1;
			return -EINVAL;
		}
	}

	if (common->lun != lun)
		DBG(common, "using LUN %d from CBW, not LUN %d from CDB\n",
		    common->lun, lun);

	lun_value = common->lun;
	if (lun_value >= 0 && common->lun < common->nluns) {
		curlun = &common->luns[common->lun];
		common->curlun = curlun;
		if (common->cmnd[0] != REQUEST_SENSE) {
			curlun->sense_data = SS_NO_SENSE;
			curlun->sense_data_info = 0;
			curlun->info_valid = 0;
		}
	} else {
		common->curlun = NULL;
		curlun = NULL;
		common->bad_lun_okay = 0;

		if (common->cmnd[0] != INQUIRY &&
		    common->cmnd[0] != REQUEST_SENSE) {
			DBG(common, "unsupported LUN %d\n", common->lun);
			return -EINVAL;
		}
	}

	if (curlun && curlun->unit_attention_data != SS_NO_SENSE &&
			common->cmnd[0] != INQUIRY &&
			common->cmnd[0] != REQUEST_SENSE) {
		curlun->sense_data = curlun->unit_attention_data;
		curlun->unit_attention_data = SS_NO_SENSE;
		return -EINVAL;
	}

	common->cmnd[1] &= 0x1f;
	for (i = 1; i < cmnd_size; ++i) {
		if (common->cmnd[i] && !(mask & (1 << i))) {
			if (curlun)
				curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
			return -EINVAL;
		}
	}

	if (curlun && !fsg_lun_is_open(curlun) && needs_medium) {
		curlun->sense_data = SS_MEDIUM_NOT_PRESENT;
		return -EINVAL;
	}

	return 0;
}

static struct cdrom_fsg_dev			*the_fsg;

static int cdrom_do_scsi_command(struct cdrom_fsg_common *common)
{
	struct fsg_buffhd	*bh;
	int			rc;
	int			reply = -EINVAL;
	int			i;
	static char		unknown[16];

	dump_cdb(common);

	bh = common->next_buffhd_to_fill;
	common->next_buffhd_to_drain = bh;
	while (bh->state != BUF_STATE_EMPTY) {
		rc = cdrom_sleep_thread(common);
		if (rc)
			return rc;
	}
	common->phase_error = 0;
	common->short_packet_received = 0;

	down_read(&common->filesem);
	switch (common->cmnd[0]) {

	case INQUIRY:
		common->data_size_from_cmnd = common->cmnd[4];
		reply = cdrom_check_command(common, 6, DATA_DIR_TO_HOST,
				      (1<<4), 0,
				      "INQUIRY");
		if (reply == 0)
			reply = cdrom_do_inquiry(common, bh);
		break;

	case SC_LGE_SPE:
		pr_debug("%s : SC_LGE_SPE - %x %x %x\n", __func__,
			  common->cmnd[0], common->cmnd[1], common->cmnd[2]);
		printk(KERN_INFO "%s : SC_LGE_SPE - %x %x %x\n", __func__,
			  common->cmnd[0], common->cmnd[1], common->cmnd[2]);

		common->mode_state = MODE_STATE_UNKNOWN;
		switch(common->cmnd[1])
		{
			case SUB_CODE_MODE_CHANGE:
				switch(common->cmnd[2])
				{
					case TYPE_MOD_CHG_TO_ACM :
					case TYPE_MOD_CHG2_TO_ACM :
						common->mode_state = MODE_STATE_ACM;
						break;
					case TYPE_MOD_CHG_TO_UMS :
					case TYPE_MOD_CHG2_TO_UMS :
						common->mode_state = MODE_STATE_UMS;
						break;
					case TYPE_MOD_CHG_TO_MTP :
					case TYPE_MOD_CHG2_TO_MTP :
						common->mode_state = MODE_STATE_MTP;
						break;
					case TYPE_MOD_CHG_TO_ASK :
					case TYPE_MOD_CHG2_TO_ASK :
						common->mode_state = MODE_STATE_ASK;
						break;
					case TYPE_MOD_CHG_TO_CGO :
					case TYPE_MOD_CHG2_TO_CGO :
						common->mode_state = MODE_STATE_CGO;
						break;
					case TYPE_MOD_CHG_TO_TET :
					case TYPE_MOD_CHG2_TO_TET :
						common->mode_state = MODE_STATE_TET;
						break;
					default:
						common->mode_state = MODE_STATE_UNKNOWN;
				}
				printk(KERN_INFO "%s: SC_LGE_MODE - %d\n", __func__, common->mode_state);
				kobject_uevent_env(&autorun_device.this_device->kobj, KOBJ_CHANGE, envp_ack);
				reply = 0;
				break;
			case SUB_CODE_GET_VALUE:
				switch(common->cmnd[2])
				{
					case ACK_STATUS_TO_HOST :
					case ACK_STATUS_TO_HOST_VZW :
						common->mode_state = MODE_STATE_GET_VALUE;
						if (user_mode >= ACK_STATUS_ERR) {
							pr_err("%s [AUTORUN] : Error on user mode setting, set default mode (ACM)\n", __func__);
							user_mode = ACK_STATUS_ACM;
						} else {
							printk(KERN_INFO "%s [AUTORUN] : send user mode to PC %s\n", __func__, check_str[user_mode]);
							pr_debug("%s [AUTORUN] : send user mode to PC %s\n", __func__, check_str[user_mode]);
						}

						common->data_size_from_cmnd = 1;
						if ((reply = cdrom_check_command(common, 6, DATA_DIR_TO_HOST,
										(7<<1), 1, check_str[user_mode])) == 0)
							reply=cdrom_do_ack_status(common, bh, user_mode);

						break;
				#ifndef CONFIG_LGE_USB_GADGET_DRIVER
					case ACK_SW_REV_TO_HOST :
						common->data_size_from_cmnd = 7;
						if ((reply = cdrom_check_command(common, 6, DATA_DIR_TO_HOST,
										(7<<1), 1, "ACK_SW_REV")) == 0)
							reply=do_get_sw_rev(common, bh);
						break;
					case ACK_MEID_TO_HOST :
						common->data_size_from_cmnd = 7;
						if ((reply = cdrom_check_command(common, 6, DATA_DIR_TO_HOST,
										(7<<1), 1, "ACK_SERIAL")) == 0)
							reply=do_get_serial(common, bh);
						break;
					case ACK_MODEL_TO_HOST :
						common->data_size_from_cmnd = 7;
						if ((reply = cdrom_check_command(common, 6, DATA_DIR_TO_HOST,
										(7<<1), 1, "ACK_MODEL_NAME")) == 0)
							reply=do_get_model(common, bh);
						break;
					case ACK_SUB_VER_TO_HOST:
						common->data_size_from_cmnd = 7;
						if ((reply = cdrom_check_command(common, 6, DATA_DIR_TO_HOST,
										(7<<1), 1, "ACK_SUB_VERSION")) == 0)
							reply=do_get_sub_ver(common, bh);
						break;
				#endif
					default:
						break;
				}
				break;
			case SUB_CODE_PROBE_DEV:
				common->mode_state = MODE_STATE_PROBE_DEV;
				reply=0;
				break;
			default:
				common->mode_state = MODE_STATE_UNKNOWN;
				reply=0;
				break;
		}
		break;

	case MODE_SELECT:
		common->data_size_from_cmnd = common->cmnd[4];
		reply = cdrom_check_command(common, 6, DATA_DIR_FROM_HOST,
				      (1<<1) | (1<<4), 0,
				      "MODE SELECT(6)");
		if (reply == 0)
			reply = cdrom_do_mode_select(common, bh);
		break;

	case MODE_SELECT_10:
		common->data_size_from_cmnd =
			get_unaligned_be16(&common->cmnd[7]);
		reply = cdrom_check_command(common, 10, DATA_DIR_FROM_HOST,
				      (1<<1) | (3<<7), 0,
				      "MODE SELECT(10)");
		if (reply == 0)
			reply = cdrom_do_mode_select(common, bh);
		break;

	case MODE_SENSE:
		common->data_size_from_cmnd = common->cmnd[4];
		reply = cdrom_check_command(common, 6, DATA_DIR_TO_HOST,
				      (1<<1) | (1<<2) | (1<<4), 0,
				      "MODE SENSE(6)");
		if (reply == 0)
			reply = cdrom_do_mode_sense(common, bh);
		break;

	case MODE_SENSE_10:
		common->data_size_from_cmnd =
			get_unaligned_be16(&common->cmnd[7]);
		reply = cdrom_check_command(common, 10, DATA_DIR_TO_HOST,
				      (1<<1) | (1<<2) | (3<<7), 0,
				      "MODE SENSE(10)");
		if (reply == 0)
			reply = cdrom_do_mode_sense(common, bh);
		break;

	case ALLOW_MEDIUM_REMOVAL:
		common->data_size_from_cmnd = 0;
		reply = cdrom_check_command(common, 6, DATA_DIR_NONE,
				      (1<<4), 0,
				      "PREVENT-ALLOW MEDIUM REMOVAL");
		if (reply == 0)
			reply = cdrom_do_prevent_allow(common);
		break;

	case READ_6:
		i = common->cmnd[4];
		common->data_size_from_cmnd = (i == 0 ? 256 : i) << 9;
		reply = cdrom_check_command(common, 6, DATA_DIR_TO_HOST,
				      (7<<1) | (1<<4), 1,
				      "READ(6)");
		if (reply == 0)
			reply = cdrom_do_read(common);
		break;

	case READ_10:
		common->data_size_from_cmnd =
				get_unaligned_be16(&common->cmnd[7]) << 9;
		reply = cdrom_check_command(common, 10, DATA_DIR_TO_HOST,
				      (1<<1) | (0xf<<2) | (3<<7), 1,
				      "READ(10)");
		if (reply == 0)
			reply = cdrom_do_read(common);
		break;

	case READ_12:
		common->data_size_from_cmnd =
				get_unaligned_be32(&common->cmnd[6]) << 9;
		reply = cdrom_check_command(common, 12, DATA_DIR_TO_HOST,
				      (1<<1) | (0xf<<2) | (0xf<<6), 1,
				      "READ(12)");
		if (reply == 0)
			reply = cdrom_do_read(common);
		break;

	case READ_CAPACITY:
		common->data_size_from_cmnd = 8;
		reply = cdrom_check_command(common, 10, DATA_DIR_TO_HOST,
				      (0xf<<2) | (1<<8), 1,
				      "READ CAPACITY");
		if (reply == 0)
			reply = cdrom_do_read_capacity(common, bh);
		break;

	case READ_HEADER:
		if (!common->curlun || !common->curlun->cdrom)
			goto unknown_cmnd;
		common->data_size_from_cmnd =
			get_unaligned_be16(&common->cmnd[7]);
		reply = cdrom_check_command(common, 10, DATA_DIR_TO_HOST,
				      (3<<7) | (0x1f<<1), 1,
				      "READ HEADER");
		if (reply == 0)
			reply = cdrom_do_read_header(common, bh);
		break;

	case READ_TOC:
		if (!common->curlun || !common->curlun->cdrom)
			goto unknown_cmnd;
		common->data_size_from_cmnd =
			get_unaligned_be16(&common->cmnd[7]);
		reply = cdrom_check_command(common, 10, DATA_DIR_TO_HOST,
				      (7<<6) | (1<<1), 1,
				      "READ TOC");
		if (reply == 0)
			reply = cdrom_do_read_toc(common, bh);
		break;

	case READ_FORMAT_CAPACITIES:
		common->data_size_from_cmnd =
			get_unaligned_be16(&common->cmnd[7]);
		reply = cdrom_check_command(common, 10, DATA_DIR_TO_HOST,
				      (3<<7), 1,
				      "READ FORMAT CAPACITIES");
		if (reply == 0)
			reply = cdrom_do_read_format_capacities(common, bh);
		break;

	case REQUEST_SENSE:
		common->data_size_from_cmnd = common->cmnd[4];
		reply = cdrom_check_command(common, 6, DATA_DIR_TO_HOST,
				      (1<<4), 0,
				      "REQUEST SENSE");
		if (reply == 0)
			reply = cdrom_do_request_sense(common, bh);
		break;

	case START_STOP:
		common->data_size_from_cmnd = 0;
		reply = cdrom_check_command(common, 6, DATA_DIR_NONE,
				      (1<<1) | (1<<4), 0,
				      "START-STOP UNIT");
		if (reply == 0)
			reply = cdrom_do_start_stop(common);
		break;

	case SYNCHRONIZE_CACHE:
		common->data_size_from_cmnd = 0;
		reply = cdrom_check_command(common, 10, DATA_DIR_NONE,
				      (0xf<<2) | (3<<7), 1,
				      "SYNCHRONIZE CACHE");
		if (reply == 0)
			reply = cdrom_do_synchronize_cache(common);
		break;

	case TEST_UNIT_READY:
		common->data_size_from_cmnd = 0;
		reply = cdrom_check_command(common, 6, DATA_DIR_NONE,
				0, 1,
				"TEST UNIT READY");
		break;

	case VERIFY:
		common->data_size_from_cmnd = 0;
		reply = cdrom_check_command(common, 10, DATA_DIR_NONE,
				      (1<<1) | (0xf<<2) | (3<<7), 1,
				      "VERIFY");
		if (reply == 0)
			reply = cdrom_do_verify(common);
		break;

	case WRITE_6:
		i = common->cmnd[4];
		common->data_size_from_cmnd = (i == 0 ? 256 : i) << 9;
		reply = cdrom_check_command(common, 6, DATA_DIR_FROM_HOST,
				      (7<<1) | (1<<4), 1,
				      "WRITE(6)");
		if (reply == 0)
			reply = cdrom_do_write(common);
		break;

	case WRITE_10:
		common->data_size_from_cmnd =
				get_unaligned_be16(&common->cmnd[7]) << 9;
		reply = cdrom_check_command(common, 10, DATA_DIR_FROM_HOST,
				      (1<<1) | (0xf<<2) | (3<<7), 1,
				      "WRITE(10)");
		if (reply == 0)
			reply = cdrom_do_write(common);
		break;

	case WRITE_12:
		common->data_size_from_cmnd =
				get_unaligned_be32(&common->cmnd[6]) << 9;
		reply = cdrom_check_command(common, 12, DATA_DIR_FROM_HOST,
				      (1<<1) | (0xf<<2) | (0xf<<6), 1,
				      "WRITE(12)");
		if (reply == 0)
			reply = cdrom_do_write(common);
		break;

	case FORMAT_UNIT:
	case RELEASE:
	case RESERVE:
	case SEND_DIAGNOSTIC:

	default:
unknown_cmnd:
		common->data_size_from_cmnd = 0;
		sprintf(unknown, "Unknown x%02x", common->cmnd[0]);
		reply = cdrom_check_command(common, common->cmnd_size,
				      DATA_DIR_UNKNOWN, 0xff, 0, unknown);
		if (reply == 0) {
			common->curlun->sense_data = SS_INVALID_COMMAND;
			reply = -EINVAL;
		}
		break;
	}
	up_read(&common->filesem);

	if (reply == -EINTR || signal_pending(current))
		return -EINTR;

	if (reply == -EINVAL)
		reply = 0;
	if (reply >= 0 && common->data_dir == DATA_DIR_TO_HOST) {
		reply = min((u32) reply, common->data_size_from_cmnd);
		bh->inreq->length = reply;
		bh->state = BUF_STATE_FULL;
		common->residue -= reply;
	}

	return 0;
}



static int cdrom_received_cbw(struct cdrom_fsg_dev *fsg, struct fsg_buffhd *bh)
{
	struct usb_request	*req = bh->outreq;
	struct fsg_bulk_cb_wrap	*cbw = req->buf;
	struct cdrom_fsg_common	*common = fsg->common;

	if (req->status || test_bit(IGNORE_BULK_OUT, &fsg->atomic_bitflags))
		return -EINVAL;

	if (req->actual != USB_BULK_CB_WRAP_LEN ||
			cbw->Signature != cpu_to_le32(
				USB_BULK_CB_SIG)) {
		DBG(fsg, "invalid CBW: len %u sig 0x%x\n",
				req->actual,
				le32_to_cpu(cbw->Signature));

		cdrom_wedge_bulk_in_endpoint(fsg);
		set_bit(IGNORE_BULK_OUT, &fsg->atomic_bitflags);
		return -EINVAL;
	}

	if (cbw->Lun >= FSG_MAX_LUNS || cbw->Flags & ~USB_BULK_IN_FLAG ||
			cbw->Length <= 0 || cbw->Length > MAX_COMMAND_SIZE) {
		DBG(fsg, "non-meaningful CBW: lun = %u, flags = 0x%x, "
				"cmdlen %u\n",
				cbw->Lun, cbw->Flags, cbw->Length);

		if (common->can_stall) {
			cdrom_fsg_set_halt(fsg, fsg->bulk_out);
			cdrom_halt_bulk_in_endpoint(fsg);
		}
		return -EINVAL;
	}

	common->cmnd_size = cbw->Length;
	memcpy(common->cmnd, cbw->CDB, common->cmnd_size);
	if (cbw->Flags & USB_BULK_IN_FLAG)
		common->data_dir = DATA_DIR_TO_HOST;
	else
		common->data_dir = DATA_DIR_FROM_HOST;
	common->data_size = le32_to_cpu(cbw->DataTransferLength);
	if (common->data_size == 0)
		common->data_dir = DATA_DIR_NONE;
	common->lun = cbw->Lun;
	common->tag = cbw->Tag;
	return 0;
}


static int cdrom_get_next_command(struct cdrom_fsg_common *common)
{
	struct fsg_buffhd	*bh;
	int			rc = 0;

	bh = common->next_buffhd_to_fill;
	while (bh->state != BUF_STATE_EMPTY) {
		rc = cdrom_sleep_thread(common);
		if (rc)
			return rc;
	}

	cdrom_set_bulk_out_req_length(common, bh, USB_BULK_CB_WRAP_LEN);
	bh->outreq->short_not_ok = 1;
	CDROM_START_TRANSFER_OR(common, bulk_out, bh->outreq,
			  &bh->outreq_busy, &bh->state)
		return -EIO;


	while (bh->state != BUF_STATE_FULL) {
		rc = cdrom_sleep_thread(common);
		if (rc)
			return rc;
	}
	smp_rmb();
	rc = cdrom_fsg_is_set(common) ? cdrom_received_cbw(common->fsg, bh) : -EIO;
	bh->state = BUF_STATE_EMPTY;

	return rc;
}



static int cdrom_enable_endpoint(struct cdrom_fsg_common *common, struct usb_ep *ep,
		const struct usb_endpoint_descriptor *d)
{
	int	rc;

	ep->driver_data = common;
	rc = usb_ep_enable(ep, d);
	if (rc)
		ERROR(common, "can't enable %s, result %d\n", ep->name, rc);
	return rc;
}

static int cdrom_alloc_request(struct cdrom_fsg_common *common, struct usb_ep *ep,
		struct usb_request **preq)
{
	*preq = usb_ep_alloc_request(ep, GFP_ATOMIC);
	if (*preq)
		return 0;
	ERROR(common, "can't allocate request for %s\n", ep->name);
	return -ENOMEM;
}

static int cdrom_do_set_interface(struct cdrom_fsg_common *common, struct cdrom_fsg_dev *new_fsg)
{
	struct cdrom_fsg_dev *fsg;
	int i, rc = 0;

	if (common->running)
		DBG(common, "reset interface\n");

reset:
	if (common->fsg) {
		fsg = common->fsg;

		for (i = 0; i < FSG_NUM_BUFFERS; ++i) {
			struct fsg_buffhd *bh = &common->buffhds[i];

			if (bh->inreq) {
				usb_ep_free_request(fsg->bulk_in, bh->inreq);
				bh->inreq = NULL;
			}
			if (bh->outreq) {
				usb_ep_free_request(fsg->bulk_out, bh->outreq);
				bh->outreq = NULL;
			}
		}


		common->fsg = NULL;
		wake_up(&common->fsg_wait);
	}

	common->running = 0;
	if (!new_fsg || rc)
		return rc;

	common->fsg = new_fsg;
	fsg = common->fsg;


	for (i = 0; i < FSG_NUM_BUFFERS; ++i) {
		struct fsg_buffhd	*bh = &common->buffhds[i];

		rc = cdrom_alloc_request(common, fsg->bulk_in, &bh->inreq);
		if (rc)
			goto reset;
		rc = cdrom_alloc_request(common, fsg->bulk_out, &bh->outreq);
		if (rc)
			goto reset;
		bh->inreq->buf = bh->outreq->buf = bh->buf;
		bh->inreq->context = bh->outreq->context = bh;
		bh->inreq->complete = cdrom_bulk_in_complete;
		bh->outreq->complete = cdrom_bulk_out_complete;
	}

	common->running = 1;
	for (i = 0; i < common->nluns; ++i)
		common->luns[i].unit_attention_data = SS_RESET_OCCURRED;
	return rc;
}




static int cdrom_fsg_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
	struct cdrom_fsg_dev *fsg = cdrom_fsg_from_func(f);
	struct cdrom_fsg_common *common = fsg->common;
	const struct usb_endpoint_descriptor *d;
	int rc;

	d = fsg_ep_desc(common->gadget,
			&fsg_fs_bulk_in_desc, &fsg_hs_bulk_in_desc);
	rc = cdrom_enable_endpoint(common, fsg->bulk_in, d);
	if (rc)
		return rc;
	fsg->bulk_in_enabled = 1;

	d = fsg_ep_desc(common->gadget,
			&fsg_fs_bulk_out_desc, &fsg_hs_bulk_out_desc);
	rc = cdrom_enable_endpoint(common, fsg->bulk_out, d);
	if (rc) {
		usb_ep_disable(fsg->bulk_in);
		fsg->bulk_in_enabled = 0;
		return rc;
	}
	fsg->bulk_out_enabled = 1;
	common->bulk_out_maxpacket = le16_to_cpu(d->wMaxPacketSize);
	clear_bit(IGNORE_BULK_OUT, &fsg->atomic_bitflags);
	fsg->common->new_fsg = fsg;
	cdrom_raise_exception(fsg->common, FSG_STATE_CONFIG_CHANGE);
	return 0;
}

static void cdrom_fsg_disable(struct usb_function *f)
{
	struct cdrom_fsg_dev *fsg = cdrom_fsg_from_func(f);

	if (fsg->bulk_in_enabled) {
		usb_ep_disable(fsg->bulk_in);
		fsg->bulk_in_enabled = 0;
		fsg->bulk_in->driver_data = NULL;
	}
	if (fsg->bulk_out_enabled) {
		usb_ep_disable(fsg->bulk_out);
		fsg->bulk_out_enabled = 0;
		fsg->bulk_out->driver_data = NULL;
	}
	fsg->common->new_fsg = NULL;
	cdrom_raise_exception(fsg->common, FSG_STATE_CONFIG_CHANGE);
}




static void cdrom_handle_exception(struct cdrom_fsg_common *common)
{
	siginfo_t		info;
	int			i;
	struct fsg_buffhd	*bh;
	enum fsg_state		old_state;
	struct fsg_lun		*curlun;
	unsigned int		exception_req_tag;

	for (;;) {
		int sig =
			dequeue_signal_lock(current, &current->blocked, &info);
		if (!sig)
			break;
		if (sig != SIGUSR1) {
			if (common->state < FSG_STATE_EXIT)
				DBG(common, "Main thread exiting on signal\n");
			cdrom_raise_exception(common, FSG_STATE_EXIT);
		}
	}

	if (likely(common->fsg)) {
		for (i = 0; i < FSG_NUM_BUFFERS; ++i) {
			bh = &common->buffhds[i];
			if (bh->inreq_busy)
				usb_ep_dequeue(common->fsg->bulk_in, bh->inreq);
			if (bh->outreq_busy)
				usb_ep_dequeue(common->fsg->bulk_out,
					       bh->outreq);
		}

		for (;;) {
			int num_active = 0;
			for (i = 0; i < FSG_NUM_BUFFERS; ++i) {
				bh = &common->buffhds[i];
				num_active += bh->inreq_busy + bh->outreq_busy;
			}
			if (num_active == 0)
				break;
			if (cdrom_sleep_thread(common))
				return;
		}

		if (common->fsg->bulk_in_enabled)
			usb_ep_fifo_flush(common->fsg->bulk_in);
		if (common->fsg->bulk_out_enabled)
			usb_ep_fifo_flush(common->fsg->bulk_out);
	}

	spin_lock_irq(&common->lock);

	for (i = 0; i < FSG_NUM_BUFFERS; ++i) {
		bh = &common->buffhds[i];
		bh->state = BUF_STATE_EMPTY;
	}
	common->next_buffhd_to_fill = &common->buffhds[0];
	common->next_buffhd_to_drain = &common->buffhds[0];
	exception_req_tag = common->exception_req_tag;
	old_state = common->state;

	if (old_state == FSG_STATE_ABORT_BULK_OUT)
		common->state = FSG_STATE_STATUS_PHASE;
	else {
		for (i = 0; i < common->nluns; ++i) {
			curlun = &common->luns[i];
			curlun->prevent_medium_removal = 0;
			curlun->sense_data = SS_NO_SENSE;
			curlun->unit_attention_data = SS_NO_SENSE;
			curlun->sense_data_info = 0;
			curlun->info_valid = 0;
		}
		common->state = FSG_STATE_IDLE;
	}
	spin_unlock_irq(&common->lock);

	switch (old_state) {
	case FSG_STATE_ABORT_BULK_OUT:
		cdrom_send_status(common);
		spin_lock_irq(&common->lock);
		if (common->state == FSG_STATE_STATUS_PHASE)
			common->state = FSG_STATE_IDLE;
		spin_unlock_irq(&common->lock);
		break;

	case FSG_STATE_RESET:
		if (!cdrom_fsg_is_set(common))
			break;
		common->ep0req->length = 0;
		if (test_and_clear_bit(IGNORE_BULK_OUT,
				       &common->fsg->atomic_bitflags))
			usb_ep_clear_halt(common->fsg->bulk_in);

		if (common->ep0_req_tag == exception_req_tag)
			cdrom_ep0_queue(common);

		break;

	case FSG_STATE_CONFIG_CHANGE:
		cdrom_do_set_interface(common, common->new_fsg);
		break;

	case FSG_STATE_EXIT:
	case FSG_STATE_TERMINATED:
		cdrom_do_set_interface(common, NULL);
		spin_lock_irq(&common->lock);
		common->state = FSG_STATE_TERMINATED;
		spin_unlock_irq(&common->lock);
		break;

	case FSG_STATE_INTERFACE_CHANGE:
	case FSG_STATE_DISCONNECT:
	case FSG_STATE_COMMAND_PHASE:
	case FSG_STATE_DATA_PHASE:
	case FSG_STATE_STATUS_PHASE:
	case FSG_STATE_IDLE:
		break;
	}
}



static int cdrom_fsg_main_thread(void *common_)
{
	struct cdrom_fsg_common	*common = common_;

	allow_signal(SIGINT);
	allow_signal(SIGTERM);
	allow_signal(SIGKILL);
	allow_signal(SIGUSR1);

	set_freezable();

	set_fs(get_ds());

	while (common->state != FSG_STATE_TERMINATED) {
		if (cdrom_exception_in_progress(common) || signal_pending(current)) {
			cdrom_handle_exception(common);
			continue;
		}

		if (!common->running) {
			cdrom_sleep_thread(common);
			continue;
		}

		if (cdrom_get_next_command(common))
			continue;

		spin_lock_irq(&common->lock);
		if (!cdrom_exception_in_progress(common))
			common->state = FSG_STATE_DATA_PHASE;
		spin_unlock_irq(&common->lock);

		if (cdrom_do_scsi_command(common) || cdrom_finish_reply(common))
			continue;

		spin_lock_irq(&common->lock);
		if (!cdrom_exception_in_progress(common))
			common->state = FSG_STATE_STATUS_PHASE;
		spin_unlock_irq(&common->lock);

#ifdef CONFIG_USB_CSW_HACK
		if (!(write_error_after_csw_sent) &&
			(common->cmnd[0] == WRITE_6
			|| common->cmnd[0] == WRITE_10
			|| common->cmnd[0] == WRITE_12))
			continue;
#endif
		if (cdrom_send_status(common))
			continue;

		spin_lock_irq(&common->lock);
		if (!cdrom_exception_in_progress(common))
			common->state = FSG_STATE_IDLE;
		spin_unlock_irq(&common->lock);
	}

	spin_lock_irq(&common->lock);
	common->thread_task = NULL;
	spin_unlock_irq(&common->lock);

	if (!common->thread_exits || common->thread_exits(common) < 0) {
		struct fsg_lun *curlun = common->luns;
		unsigned i = common->nluns;

		down_write(&common->filesem);
		for (; i--; ++curlun) {
			if (!fsg_lun_is_open(curlun))
				continue;

			fsg_lun_close(curlun);
			curlun->unit_attention_data = SS_MEDIUM_NOT_PRESENT;
		}
		up_write(&common->filesem);
	}

	complete_and_exit(&common->thread_notifier, 0);
}

static ssize_t fsg_show_usbmode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	ret = sprintf(buf, "%d", user_mode);
	return ret;
}

static ssize_t fsg_store_usbmode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	unsigned long tmp;

	ret = strict_strtoul(buf, 10, &tmp);
	if (ret)
		return count;

	user_mode = (unsigned int)tmp;

	pr_info("autorun user mode : %d\n", user_mode);
	return count;
}


static DEVICE_ATTR(cdrom_ro, 0644, fsg_show_ro, fsg_store_ro);
static DEVICE_ATTR(cdrom_file, 0644, fsg_show_file, fsg_store_file);
static DEVICE_ATTR(cdrom_nofua, 0644, fsg_show_nofua, fsg_store_nofua);
static DEVICE_ATTR(cdrom_usbmode, 0664, fsg_show_usbmode, fsg_store_usbmode);



static void cdrom_fsg_common_release(struct kref *ref);

static void cdrom_fsg_lun_release(struct device *dev)
{
}

static inline void cdrom_fsg_common_get(struct cdrom_fsg_common *common)
{
	kref_get(&common->ref);
}

static inline void cdrom_fsg_common_put(struct cdrom_fsg_common *common)
{
	kref_put(&common->ref, cdrom_fsg_common_release);
}


static struct cdrom_fsg_common *cdrom_fsg_common_init(struct cdrom_fsg_common *common,
					  struct usb_composite_dev *cdev,
					  struct cdrom_fsg_config *cfg)
{
	struct usb_gadget *gadget = cdev->gadget;
	struct fsg_buffhd *bh;
	struct fsg_lun *curlun;
	struct cdrom_fsg_lun_config *lcfg;
	int nluns, i, rc;
	char *pathbuf;

	nluns = cfg->nluns;
	if (nluns < 1 || nluns > FSG_MAX_LUNS) {
		dev_err(&gadget->dev, "invalid number of LUNs: %u\n", nluns);
		return ERR_PTR(-EINVAL);
	}

	if (!common) {
		common = kzalloc(sizeof *common, GFP_KERNEL);
		if (!common)
			return ERR_PTR(-ENOMEM);
		common->free_storage_on_release = 1;
	} else {
		memset(common, 0, sizeof common);
		common->free_storage_on_release = 0;
	}

	common->private_data = cfg->private_data;

	common->gadget = gadget;
	common->ep0 = gadget->ep0;
	common->ep0req = cdev->req;

	if (fsg_strings[FSG_STRING_INTERFACE].id == 0) {
		rc = usb_string_id(cdev);
		if (unlikely(rc < 0))
			goto error_release;
		fsg_strings[FSG_STRING_INTERFACE].id = rc;
		fsg_intf_desc.iInterface = rc;
	}

	curlun = kzalloc(nluns * sizeof *curlun, GFP_KERNEL);
	if (unlikely(!curlun)) {
		rc = -ENOMEM;
		goto error_release;
	}
	common->luns = curlun;

	init_rwsem(&common->filesem);

	for (i = 0, lcfg = cfg->luns; i < nluns; ++i, ++curlun, ++lcfg) {
		curlun->cdrom = !!lcfg->cdrom;
		curlun->ro = lcfg->cdrom || lcfg->ro;
		curlun->removable = lcfg->removable;
		curlun->dev.release = cdrom_fsg_lun_release;

		curlun->dev.parent = &gadget->dev;
		dev_set_drvdata(&curlun->dev, &common->filesem);
#ifndef CONFIG_LGE_USB_GADGET_DRIVER
		dev_set_name(&curlun->dev,
			     cfg->lun_name_format
			   ? cfg->lun_name_format
			   : "lun%d",
			     i);
#else
		dev_set_name(&curlun->dev,
				 cfg->lun_name_format
			   ? cfg->lun_name_format
			   : "lun%d",
				 i+2);
#endif
		rc = device_register(&curlun->dev);
		if (rc) {
			ERROR(common, "CDROM failed to register LUN%d: %d\n", i, rc);
			common->nluns = i;
			goto error_release;
		}

		rc = device_create_file(&curlun->dev, &dev_attr_cdrom_ro);
		if (rc)
			goto error_luns;
		rc = device_create_file(&curlun->dev, &dev_attr_cdrom_file);
		if (rc)
			goto error_luns;
		rc = device_create_file(&curlun->dev, &dev_attr_cdrom_nofua);
		if (rc)
			goto error_luns;
		rc = device_create_file(&curlun->dev, &dev_attr_cdrom_usbmode);
		if (rc)
			goto error_luns;
		if (lcfg->filename) {
			rc = fsg_lun_open(curlun, lcfg->filename);
			if (rc)
				goto error_luns;
		} else if (!curlun->removable) {
			ERROR(common, "no file given for LUN%d\n", i);
			rc = -EINVAL;
			goto error_luns;
		}
	}
	common->nluns = nluns;


	bh = common->buffhds;
	i = FSG_NUM_BUFFERS;
	goto buffhds_first_it;
	do {
		bh->next = bh + 1;
		++bh;
buffhds_first_it:
		bh->buf = kmalloc(FSG_BUFLEN, GFP_KERNEL);
		if (unlikely(!bh->buf)) {
			rc = -ENOMEM;
			goto error_release;
		}
	} while (--i);
	bh->next = common->buffhds;

	cfg->can_stall = 1;

	if (cfg->release != 0xffff) {
		i = cfg->release;
	} else {
		i = usb_gadget_controller_number(gadget);
		if (i >= 0) {
			i = 0x0300 + i;
		} else {
			WARNING(common, "controller '%s' not recognized\n",
				gadget->name);
			i = 0x0399;
		}
	}
#define OR(x, y) ((x) ? (x) : (y))
	snprintf(common->inquiry_string, sizeof common->inquiry_string,
		 "%-8s%-16s%04x",
		 OR(cfg->vendor_name, "Linux   "),
		 OR(cfg->product_name, common->luns->cdrom
				     ? "File-Stor Gadget"
				     : "File-CD Gadget  "),
		 i);


	common->can_stall = cfg->can_stall &&
		!(gadget_is_at91(common->gadget));


	spin_lock_init(&common->lock);
	kref_init(&common->ref);


	common->thread_exits = cfg->thread_exits;
	common->thread_task =
#ifndef CONFIG_LGE_USB_GADGET_DRIVER
	kthread_create(cdrom_fsg_main_thread, common,
		       OR(cfg->thread_name, "file-storage"));
#else
	kthread_create(cdrom_fsg_main_thread, common,
			   OR(cfg->thread_name, "cdrom-storage"));

#endif
	if (IS_ERR(common->thread_task)) {
		rc = PTR_ERR(common->thread_task);
		goto error_release;
	}
	init_completion(&common->thread_notifier);
	init_waitqueue_head(&common->fsg_wait);
#undef OR


	DBG(common, CDFSG_DRIVER_DESC ", version: " CDFSG_DRIVER_VERSION "\n");
	DBG(common, "Number of LUNs=%d\n", common->nluns);

	pathbuf = kmalloc(PATH_MAX, GFP_KERNEL);
	for (i = 0, nluns = common->nluns, curlun = common->luns;
	     i < nluns;
	     ++curlun, ++i) {
		char *p = "(no medium)";
		if (fsg_lun_is_open(curlun)) {
			p = "(error)";
			if (pathbuf) {
				p = d_path(&curlun->filp->f_path,
					   pathbuf, PATH_MAX);
				if (IS_ERR(p))
					p = "(error)";
			}
		}
		LDBG(curlun, "LUN: %s%s%sfile: %s\n",
		      curlun->removable ? "removable " : "",
		      curlun->ro ? "read only " : "",
		      curlun->cdrom ? "CD-ROM " : "",
		      p);
	}
	kfree(pathbuf);

	DBG(common, "I/O thread pid: %d\n", task_pid_nr(common->thread_task));

	rc = misc_register(&autorun_device);
	if (rc) {
		printk(KERN_ERR "USB cdrom gadget driver failed to initialize\n");
		goto error_release;
	}


	wake_up_process(common->thread_task);

	return common;


error_luns:
	common->nluns = i + 1;
error_release:
	common->state = FSG_STATE_TERMINATED;
	cdrom_fsg_common_release(&common->ref);
	return ERR_PTR(rc);
}


static void cdrom_fsg_common_release(struct kref *ref)
{
	struct cdrom_fsg_common *common = container_of(ref, struct cdrom_fsg_common, ref);

	if (common->state != FSG_STATE_TERMINATED) {
		cdrom_raise_exception(common, FSG_STATE_EXIT);
		wait_for_completion(&common->thread_notifier);

		complete(&common->thread_notifier);
	}

	if (likely(common->luns)) {
		struct fsg_lun *lun = common->luns;
		unsigned i = common->nluns;

		for (; i; --i, ++lun) {
			device_remove_file(&lun->dev, &dev_attr_cdrom_ro);
			device_remove_file(&lun->dev, &dev_attr_cdrom_file);
			device_remove_file(&lun->dev, &dev_attr_cdrom_nofua);
			device_remove_file(&lun->dev, &dev_attr_cdrom_usbmode);
			fsg_lun_close(lun);
			device_unregister(&lun->dev);
		}

		kfree(common->luns);
	}

	{
		struct fsg_buffhd *bh = common->buffhds;
		unsigned i = FSG_NUM_BUFFERS;
		do {
			kfree(bh->buf);
		} while (++bh, --i);
	}

	if (common->free_storage_on_release)
		kfree(common);

	misc_deregister(&autorun_device);
}




static void cdrom_fsg_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct cdrom_fsg_dev		*fsg = cdrom_fsg_from_func(f);
	struct cdrom_fsg_common	*common = fsg->common;

	DBG(fsg, "unbind\n");
	if (fsg->common->fsg == fsg) {
		fsg->common->new_fsg = NULL;
		cdrom_raise_exception(fsg->common, FSG_STATE_CONFIG_CHANGE);
		wait_event(common->fsg_wait, common->fsg != fsg);
	}

	cdrom_fsg_common_put(common);
	usb_free_descriptors(fsg->function.descriptors);
	usb_free_descriptors(fsg->function.hs_descriptors);
	kfree(fsg);
}


static int cdrom_fsg_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct cdrom_fsg_dev		*fsg = cdrom_fsg_from_func(f);
	struct usb_gadget	*gadget = c->cdev->gadget;
	int			i;
	struct usb_ep		*ep;

	fsg->gadget = gadget;

	i = usb_interface_id(c, f);
	if (i < 0)
		return i;
	fsg_intf_desc.bInterfaceNumber = i;
	fsg->interface_number = i;

	ep = usb_ep_autoconfig(gadget, &fsg_fs_bulk_in_desc);
	if (!ep)
		goto autoconf_fail;
	ep->driver_data = fsg->common;
	fsg->bulk_in = ep;

	ep = usb_ep_autoconfig(gadget, &fsg_fs_bulk_out_desc);
	if (!ep)
		goto autoconf_fail;
	ep->driver_data = fsg->common;
	fsg->bulk_out = ep;

	f->descriptors = usb_copy_descriptors(fsg_fs_function);
	if (unlikely(!f->descriptors))
		return -ENOMEM;

	if (gadget_is_dualspeed(gadget)) {
		fsg_hs_bulk_in_desc.bEndpointAddress =
			fsg_fs_bulk_in_desc.bEndpointAddress;
		fsg_hs_bulk_out_desc.bEndpointAddress =
			fsg_fs_bulk_out_desc.bEndpointAddress;
		f->hs_descriptors = usb_copy_descriptors(fsg_hs_function);
		if (unlikely(!f->hs_descriptors)) {
			usb_free_descriptors(f->descriptors);
			return -ENOMEM;
		}
	}

	return 0;

autoconf_fail:
	ERROR(fsg, "unable to autoconfigure all endpoints\n");
	return -ENOTSUPP;
}



static struct usb_gadget_strings *cdrom_fsg_strings_array[] = {
	&fsg_stringtab,
	NULL,
};

static int cdrom_fsg_bind_config(struct usb_composite_dev *cdev,
		   struct usb_configuration *c,
		   struct cdrom_fsg_common *common)
{
	struct cdrom_fsg_dev *fsg;
	int rc;

	fsg = kzalloc(sizeof *fsg, GFP_KERNEL);
	if (unlikely(!fsg))
		return -ENOMEM;

	the_fsg = fsg;

	fsg->function.name        = CDFSG_DRIVER_DESC;
	fsg->function.strings     = cdrom_fsg_strings_array;
	fsg->function.bind        = cdrom_fsg_bind;
	fsg->function.unbind      = cdrom_fsg_unbind;
	fsg->function.setup       = cdrom_fsg_setup;
	fsg->function.set_alt     = cdrom_fsg_set_alt;
	fsg->function.disable     = cdrom_fsg_disable;

	fsg->common               = common;

	rc = usb_add_function(c, &fsg->function);
	if (unlikely(rc))
		kfree(fsg);
	else
		cdrom_fsg_common_get(fsg->common);
	return rc;
}





struct cdrom_fsg_module_parameters {
	char		*file[FSG_MAX_LUNS];
	int		ro[FSG_MAX_LUNS];
	int		removable[FSG_MAX_LUNS];
	int		cdrom[FSG_MAX_LUNS];

	unsigned int	file_count, ro_count, removable_count, cdrom_count;
	unsigned int	luns;
	int		stall;
};

static void
cdrom_fsg_config_from_params(struct cdrom_fsg_config *cfg,
		       const struct cdrom_fsg_module_parameters *params)
{
	struct cdrom_fsg_lun_config *lun;
	unsigned i;

	cfg->nluns =
		min(params->luns ?: (params->file_count ?: 1u),
		    (unsigned)FSG_MAX_LUNS);
	for (i = 0, lun = cfg->luns; i < cfg->nluns; ++i, ++lun) {
		lun->ro = !!params->ro[i];
		lun->cdrom = !!params->cdrom[i];
		lun->removable =
			params->removable_count <= i || params->removable[i];
		lun->filename =
			params->file_count > i && params->file[i][0]
			? params->file[i]
			: 0;
	}

	cfg->lun_name_format = 0;
	cfg->thread_name = 0;
	cfg->vendor_name = 0;
	cfg->product_name = 0;
	cfg->release = 0xffff;

	cfg->thread_exits = 0;
	cfg->private_data = 0;

	cfg->can_stall = params->stall;
}

static inline struct cdrom_fsg_common *
cdrom_fsg_common_from_params(struct cdrom_fsg_common *common,
		       struct usb_composite_dev *cdev,
		       const struct cdrom_fsg_module_parameters *params)
	__attribute__((unused));
static inline struct cdrom_fsg_common *
cdrom_fsg_common_from_params(struct cdrom_fsg_common *common,
		       struct usb_composite_dev *cdev,
		       const struct cdrom_fsg_module_parameters *params)
{
	struct cdrom_fsg_config cfg;
	cdrom_fsg_config_from_params(&cfg, params);
	return cdrom_fsg_common_init(common, cdev, &cfg);
}
