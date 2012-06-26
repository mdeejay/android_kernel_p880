/*
* Motorola TS0710 driver.
*
* Copyright (C) 2002-2004  Motorola
* Copyright (C) 2006 Harald Welte <laforge@openezx.org>
* Copyright (C) 2009 Ilya Petrov <ilya.muromec@gmail.com>
* Copyright (C) 2009 Daniel Ribeiro <drwyrm@gmail.com>
*
* Portions derived from rfcomm.c, original header as follows:
*
* Copyright (C) 2000, 2001  Axis Communications AB
*
* Author: Mats Friden <mats.friden@axis.com>
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
*
* Exceptionally, Axis Communications AB grants discretionary and
* conditional permissions for additional use of the text contained
* in the company's release of the AXIS OpenBT Stack under the
* provisions set forth hereunder.
*
* Provided that, if you use the AXIS OpenBT Stack with other files,
* that do not implement functionality as specified in the Bluetooth
* System specification, to produce an executable, this does not by
* itself cause the resulting executable to be covered by the GNU
* General Public License. Your use of that executable is in no way
* restricted on account of using the AXIS OpenBT Stack code with it.
*
* This exception does not however invalidate any other reasons why
* the executable file might be covered by the provisions of the GNU
* General Public License.
*
*/
#include <linux/module.h>
#include <linux/types.h>

#include <linux/kernel.h>
#include <linux/proc_fs.h>

#include <linux/serial.h>

#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/fcntl.h>
#include <linux/string.h>
#include <linux/major.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/poll.h>

#include <asm/uaccess.h>
#include <asm/system.h>
#include <asm/bitops.h>
#include <asm/mach-types.h>

#include <linux/freezer.h>
#define LGE_ENABLE_RIL_RECOVERY_MODE

#ifdef LGE_ENABLE_RIL_RECOVERY_MODE
#include <linux/delay.h>
#include <mach/gpio.h>
#include "../gpio-names.h"  
#include "../../../arch/arm/mach-tegra/baseband-xmm-power.h" 

#define MODEM_PWR_ON        TEGRA_GPIO_PO0
#define MODEM_RESET         TEGRA_GPIO_PV1
#endif


#ifdef CONFIG_LGE_KERNEL_MUX
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#define TS0710MAX_CHANNELS 32
#define TS0710MAX_PRIORITY_NUMBER 64
#else
#define TS0710MAX_CHANNELS 23
#include "ts0710_mux_usb.h"
#endif

#include "ts0710.h"
#include "ts0710_mux.h"

#include <linux/input.h>    

#define MUX_WRITE_MAX_RETRY 10
#define LOCK_T          spinlock_t
#define CREATELOCK(_l)  spin_lock_init(&(_l))
#define DELETELOCK(_l)
#define LOCK(_l)        spin_lock_bh(&(_l))
#define WAIT_UNLOCK(_l)	spin_unlock_wait(&(_l))
#define UNLOCK(_l)      spin_unlock_bh(&(_l))
#define ATOMIC(_l,_f)   spin_lock_irqsave(&(_l),(_f))
#define UNATOMIC(_l,_f) spin_unlock_irqrestore(&(_l),(_f)) 

static int NR_MUXS;
static struct tty_struct *ipc_tty = NULL;
#define EVENT_KEY KEY_F23
struct input_dev *input_mux;



typedef struct
{
    u8 cmdtty;
    u8 datatty;
}
dlci_tty;

static u8 *iscmdtty;

typedef struct
{
    volatile u8 buf[TS0710MUX_SEND_BUF_SIZE];
    volatile u8 *frame;
    unsigned long flags;
    volatile u16 length;
    volatile u8 filled;
    volatile u8 dummy;	/* Allignment to 4*n bytes */
}
mux_send_struct;

#define MAX_PACKET_SIZE 2048

#ifdef CONFIG_LGE_KERNEL_MUX
struct mux_data
{
    enum {OUT_OF_FRAME, INSIDE_FRAME_HEADER, INSIDE_FRAME_BODY } state;
    size_t chunk_size;
    int copy_index;
    int framelen;
    unsigned char chunk[MAX_FRAME_SIZE];
};
#else
struct mux_data
{
    enum {INSIDE_PACKET, OUT_OF_PACKET} state;
    size_t chunk_size;
    unsigned char chunk[MAX_PACKET_SIZE];
};
#endif 
/* Bit number in flags of mux_send_struct */

struct mux_recv_packet_tag
{
    u8 *data;
    u32 length;
    struct mux_recv_packet_tag *next;
};
typedef struct mux_recv_packet_tag mux_recv_packet;

struct mux_recv_struct_tag
{
    u8 data[TS0710MUX_RECV_BUF_SIZE];
    u32 length;
    u32 total;
    mux_recv_packet *mux_packet;
    struct mux_recv_struct_tag *next;
    int no_tty;
    volatile u8 post_unthrottle;
};
typedef struct mux_recv_struct_tag mux_recv_struct;

static unsigned long mux_recv_flags = 0;

static mux_send_struct *mux_send_info[TS0710MAX_CHANNELS];
static volatile u8 mux_send_info_flags[TS0710MAX_CHANNELS];
static volatile u8 mux_send_info_idx = TS0710MAX_CHANNELS;

static mux_recv_struct *mux_recv_info[TS0710MAX_CHANNELS];
static volatile u8 mux_recv_info_flags[TS0710MAX_CHANNELS];
static mux_recv_struct *mux_recv_queue = NULL;

/* Local for 2.6? */
static struct tty_driver *mux_driver;

static struct tty_struct *mux_table[TS0710MAX_CHANNELS];
static struct ktermios *mux_termios[TS0710MAX_CHANNELS];
static struct ktermios *mux_termios_locked[TS0710MAX_CHANNELS];
static volatile short int mux_tty[TS0710MAX_CHANNELS];

static volatile struct file * mux_filp[TS0710MAX_CHANNELS];


#ifdef min
#undef min
#define min(a, b)	((a) < (b) ? (a) : (b))
#endif

static ts0710_con ts0710_connection;

#ifdef CONFIG_LGE_KERNEL_MUX
static DEFINE_SEMAPHORE(spi_write_sema); /* use semaphore to synchronize different threads*/

static struct semaphore spi_write_data_sema[TS0710_MAX_CHN];

struct spi_data_recived_struct
{
    struct tty_struct *tty;
    const u8 *data;
    char *flags;
    int size;
    int updated;
};
#define MAX_WAITING_FRAMES 400  /* should be more maximum sum of waiting frames count */
static DECLARE_WAIT_QUEUE_HEAD(wq);
static volatile short int frames_to_send_count[TS0710MAX_CHANNELS];

struct spi_data_send_struct
{
    struct spi_data_send_struct *next;

    u8 dlci;

    u8 *data;
    int size;
};
static LOCK_T frame_nodes_lock = SPIN_LOCK_UNLOCKED;

static struct spi_data_send_struct frame_node[MAX_WAITING_FRAMES];
static struct spi_data_send_struct *frame_to_send[TS0710MAX_PRIORITY_NUMBER];
static struct spi_data_recived_struct spi_data_recieved;

static struct task_struct *task = NULL;
static struct task_struct *write_task = NULL;


static u8 default_priority_table[] = 
{
   0,                              /* multiplexer control channel */
   7,  7,  7,  7,  7,  7,  7,      /* 1-7 DLC   */
  15, 15, 15, 15, 15, 15, 15, 15,  /* 8-15 DLC  */
  23, 23, 23, 23, 23, 23, 23, 23,  /* 16-23 DLC */
  31, 31, 31, 31, 31, 31, 31, 31,  /* 24-31 DLC */
  39, 39, 39, 39, 39, 39, 39, 39,  /* 32-39 DLC */
  47, 47, 47, 47, 47, 47, 47, 47,  /* 40-47 DLC */
  55, 55, 55, 55, 55, 55, 55, 55,  /* 48-55 DLC */
  61, 61, 61, 61, 61, 61,          /* 56-61 DLC */
  63, 63                           /* 62-63 DLC */
};
static u8 max_waiting_frames_count[] = 
{
   20,                         /* multiplexer control channel */
   3,  3, 3,  3, 3,  3, 3,     /* 1-7 DLC */
   3,  3, 3,  3, 3,  3, 3,  3, /* 8-15 DLC  */
   3,  3, 3,  3, 3, 47, 3, 47, /* 16-23 DLC */
   3, 47, 3, 47, 3,  3, 3,  3, /* 24-31 DLC */
   3,  3, 3,  3, 3,  3, 3,  3, /* 32-39 DLC */
   3,  3, 3,  3, 3,  3, 3,  3, /* 40-47 DLC */
   3,  3, 3,  3, 3,  3, 3,  3, /* 48-55 DLC */
   3,  3, 3,  3, 3,  3,        /* 56-61 DLC */
   3,  3                       /* 62-63 DLC */
};


static int max_frame_usage = 0;


#if defined(LGE_ACM_RX_DUMP_ENABLE) //this is pair define RX_DUMP_ENABLE in CDC-ACM.c
extern void acm_rx_dump(void); 
#endif


#define LGE_VT_CALL_SESSION 
#define DLCI_VIDEO_TELEPHONY    12 


#if defined(LGE_VT_DATA_DUMP)
#define VT_MUX12_DEBUG
#define VT_SPECIAL_CHAR        1

unsigned int mux_written_counter;
static void dump_mux_rd_buffer(const unsigned char *buf, int count);
#endif 


#if defined(LGE_VT_CALL_SESSION) || defined(VT_MUX12_DEBUG) 
bool mux_12_open = 0;
#endif



#ifdef LGE_DUMP_MUX_BUFFER
#define COL_SIZE 50
static void dump_mux_buffer(const unsigned char *txt, const unsigned char *buf, int count)
{
    char dump_buf_str[COL_SIZE + 1];

    if (buf != NULL)
    {
        int j = 0;
        char *cur_str = dump_buf_str;
        unsigned char ch;
        while ((j < COL_SIZE) && (j < count))
        {
            ch = buf[j];
            if ((ch < 32) || (ch > 126))
            {
                *cur_str = '.';
            }
            else
            {
                *cur_str = ch;
            }
            cur_str++;
            j++;
        }
        *cur_str = 0;
        printk("%s:count:%d [%s]\n", txt, count, dump_buf_str);
    }
    else
    {
        printk("%s: buffer is NULL\n", txt);
    }
}

static void dump_mux_rd_buffer(const unsigned char *buf, int count)
{
    char dump_buf_str[COL_SIZE + 1];

    if (buf != NULL)
    {
        int j = 0;
        char *cur_str = dump_buf_str;
        unsigned char ch;
        while ((j < COL_SIZE) && (j < count))
        {
            ch = buf[j];
            if ((ch < 32) || (ch > 126))
            {
                *cur_str = '.';
            }
            else
            {
                *cur_str = ch;
            }
            cur_str++;
            j++;
        }
        *cur_str = 0;
        printk("MUX RD :length:%d [%s]\n", count, dump_buf_str);
    }
}

#else
#define dump_mux_buffer(...)
#define dump_mux_rd_buffer(...)
#endif

static void nodes_init(void)
{
    int i;
    
    LOCK(frame_nodes_lock);

    for (i = 0; i < MAX_WAITING_FRAMES;i++)
    {
        frame_node[i].next = NULL;

        frame_node[i].dlci = 0;

        frame_node[i].data = NULL;
        frame_node[i].size = 0;
    }
    for (i = 0; i < TS0710MAX_PRIORITY_NUMBER;i++)
    {
        frame_to_send[i] = NULL;
    }
    for (i = 0; i < TS0710MAX_CHANNELS; i++)
    {
        frames_to_send_count[i] = 0;
    }
    UNLOCK(frame_nodes_lock);

}

static int node_put_to_send(u8 dlci, u8 *data, int size)
{
    int i;
    int retval = -ENOMEM;
    int tmp_int = -1;
    int free_node = MAX_WAITING_FRAMES + 1;
    struct spi_data_send_struct *new_frame;

    u8 priority;
#ifdef TS0710LOG
    unsigned int smp_id = smp_processor_id();
#endif
    int dont_wait = (in_interrupt() || ((mux_filp[dlci] != NULL) && (mux_filp[dlci]->f_flags & O_NONBLOCK)));


    if (dont_wait)
    {
        if ((frames_to_send_count[dlci] >= max_waiting_frames_count[dlci]))
        {
            TS0710_LOG("node_put_to_send: no memory in queue for non-waiting call. return \n");
            return retval;
        }
    }
    else
    {
        while (0 != tmp_int)
        {
            tmp_int = wait_event_interruptible(wq, (frames_to_send_count[dlci] < max_waiting_frames_count[dlci]));

        }
    }
    priority = ts0710_connection.dlci[dlci].priority;
    LOCK(frame_nodes_lock);

    for (i = 0; i < MAX_WAITING_FRAMES;i++)
    {
        if (frame_node[i].size == 0)
        {
            free_node = i;
            if (free_node > max_frame_usage)
            {
                max_frame_usage = free_node;
                printk("\nTS0710:node_put_to_send:max_frame_usage=%d\n", max_frame_usage);
            }
            break;
        }
    }
    if (free_node < MAX_WAITING_FRAMES)
    {
        frame_node[free_node].next = NULL;

        frame_node[free_node].dlci = dlci;
        frame_node[free_node].data = data;

        frame_node[free_node].size = size;
        retval = size;
        if (frame_to_send[priority] == NULL)
        {
            frame_to_send[priority] = &frame_node[free_node];
        }
        else
        {
            new_frame = frame_to_send[priority];
            while (new_frame->next != NULL)
            {
                new_frame = new_frame->next;
            }
            new_frame->next = &frame_node[free_node];
        }
        
        ++frames_to_send_count[dlci];
        
    }
    else
    {
        
        kfree(data);
        TS0710_PRINTK("ERRROR: Frame node full. No Space.\n");
        
        retval = -ENOMEM;
    }
    

    
    
    UNLOCK(frame_nodes_lock);
    
    if (retval > 0 )
        up(&spi_write_sema);
    return retval;
}

#endif

static void ts0710_reset_dlci(u8 j)
{
    if (j >= TS0710_MAX_CHN)
        return ;

    ts0710_connection.dlci[j].state = DISCONNECTED;
    ts0710_connection.dlci[j].flow_control = 0;
    ts0710_connection.dlci[j].mtu = DEF_TS0710_MTU;
    ts0710_connection.dlci[j].initiated = 0;
    ts0710_connection.dlci[j].initiator = 0;
    init_waitqueue_head(&ts0710_connection.dlci[j].open_wait);
    init_waitqueue_head(&ts0710_connection.dlci[j].close_wait);
}

#ifdef CONFIG_LGE_KERNEL_MUX
static void ts0710_reset_dlci_priority()
{
    u8 j;

    for (j = 0; j < TS0710_MAX_CHN; j++)
        ts0710_connection.dlci[j].priority = default_priority_table[j];
}
#endif

static void ts0710_reset_con(void)
{
    u8 j;

    ts0710_connection.initiator = 0;
    ts0710_connection.mtu = DEF_TS0710_MTU + TS0710_MAX_HDR_SIZE;
    ts0710_connection.be_testing = 0;
    ts0710_connection.test_errs = 0;
    init_waitqueue_head(&ts0710_connection.test_wait);

#ifdef CONFIG_LGE_KERNEL_MUX
    ts0710_reset_dlci_priority();
#endif

    for (j = 0; j < TS0710_MAX_CHN; j++)
        ts0710_reset_dlci(j);
}

static void ts0710_init(void)
{
    fcs_init();

    ts0710_reset_con();
}

static void ts0710_upon_disconnect(void)
{
    ts0710_con *ts0710 = &ts0710_connection;
    u8 j;

    for (j = 0; j < TS0710_MAX_CHN; j++)
    {
        ts0710->dlci[j].state = DISCONNECTED;
        wake_up_interruptible(&ts0710->dlci[j].open_wait);
        wake_up_interruptible(&ts0710->dlci[j].close_wait);
    }
    ts0710->be_testing = 0;
    wake_up_interruptible(&ts0710->test_wait);
    ts0710_reset_con();
}

/* Sending packet functions */

/* See TS 07.10's Section 5.2.1.6 */
static u_int8_t fcs_table[0x100];
static void fcs_init(void)
{
    int i, bit;
    u8 reg, nreg;

    for (i = 0; i < 0x100; i++)
    {
        for (reg = bit = 0; bit < 8; bit++, reg = nreg)
        {
            nreg = reg >> 1;
            if (((i >> bit) ^ reg) & 1)
                nreg ^= 0xe0;	/* x^8 + x^2 + x + 1 */
        }

        fcs_table[i] = reg;
    }
}

/* Computes the FCS checksum for a chunk of data.  */
static u_int8_t mux_fcs_compute(const u8 payload[], int len)
{
    u8 gen_reg;

    gen_reg = ~0;
    while (len--)
        gen_reg = fcs_table[gen_reg ^ *(payload++)];

    return ~gen_reg;
}


#ifndef CONFIG_LGE_KERNEL_MUX 
/* Returns 1 if the given chunk of data has a correct FCS appended.  */
static int mux_fcs_check(const u8 payload[], int len)
{
    return mux_fcs_compute(payload, len + 1) == 0xcf;
}
#endif 

/* See TS 07.10's Section 5.2.1 */
static int mux_send_frame(u8 dlci, int initiator,
                          enum mux_frametype frametype, const u8 data[], int len)
{

    u8 *framebuf = NULL;

    int pos = 0;
    int pf, crc_len, res;
    int cr = initiator & 0x1;

    framebuf = kmalloc(2048, GFP_ATOMIC);
    if (framebuf == NULL)
    {
        TS0710_PRINTK("mux_send_frame: ERROR - Memory Allocation fail\n");
        return -ENOMEM;
    }
    switch (frametype)
    {
      case MUX_UIH:
      case ACK:
        pf = 0;
        crc_len = len;
        break;
      case MUX_UI:
        pf = 0;
        crc_len = 0;
        break;
      default:
        pf = 1;
        crc_len = 0;
    }

    
    framebuf[pos++] = MUX_BASIC_FLAG_SEQ;
    
    framebuf[pos++] = MUX_EA | (cr << 1) | (dlci << 2);

    /* Control field.  */
    framebuf[pos++] = frametype | (pf << 4);

    /* Length indicator.  */
    
    if (len & ~0x7f)
    {
        framebuf[pos ++] = 0 | ((len & 0x7f) << 1);
        framebuf[pos ++] = len >> 7;
    }
    else
        framebuf[pos ++] = 1 | (len << 1);

    if (len)
        memcpy(&framebuf[pos], data, len);
    pos += len;

    framebuf[pos++] = mux_fcs_compute(framebuf + 1, pos - 1 - crc_len);

   
    framebuf[pos ++] = MUX_BASIC_FLAG_SEQ;
    
#ifdef CONFIG_LGE_KERNEL_MUX
    res = node_put_to_send( dlci, framebuf, pos);
#else
    res = ipc_tty->ops->write(ipc_tty, framebuf, pos);
#endif

    if (res != pos)
    {
        TS0710_PRINTK("mux_send_frame error %d\n", res);
        
        if (res < 0)
            return res;
        else
        
            return -1;
    }

#ifdef CONFIG_LGE_KERNEL_MUX
    /* Function should return number of sent information bytes.
    Therefore need to decrease ret variable */
    if (len & ~0x7f)
    {
        /* Length indicator have two octet size. */
        res -= 7;
    }
    else
    {
        res -= 6;
    }
#endif

    return res;
}

/* Creates a UA packet and puts it at the beginning of the pkt pointer */

static void send_ua(ts0710_con * ts0710, u8 dlci)
{
    mux_send_frame(dlci, !ts0710->initiator, MUX_UA, 0, 0);
}

/* Creates a DM packet and puts it at the beginning of the pkt pointer */

static void send_dm(ts0710_con * ts0710, u8 dlci)
{
    mux_send_frame(dlci, !ts0710->initiator, MUX_DM, 0, 0);
}

static void send_sabm(ts0710_con * ts0710, u8 dlci)
{
    mux_send_frame(dlci, ts0710->initiator, MUX_SABM, 0, 0);
}


#ifndef CONFIG_LGE_KERNEL_MUX
static void send_disc(ts0710_con * ts0710, u8 dlci)
{
    mux_send_frame(dlci, !ts0710->initiator, MUX_DISC, 0, 0);
}
#endif 


/* Multiplexer command packets functions */

/* Turns on the ts0710 flow control */

static void ts0710_fcon_msg(ts0710_con * ts0710, u8 cr)
{
    mux_send_uih(ts0710, cr, FCON, 0, 0);
}

/* Turns off the ts0710 flow control */

static void ts0710_fcoff_msg(ts0710_con * ts0710, u8 cr)
{
    mux_send_uih(ts0710, cr, FCOFF, 0, 0);
}


/* Sends an PN-messages and sets the not negotiable parameters to their
   default values in ts0710 */

static void send_pn_msg(ts0710_con * ts0710, u8 prior, u32 frame_size,
                        u8 credit_flow, u8 credits, u8 dlci, u8 cr)
{
    u8 data[8];
    pn_t *send = (pn_t *)data;

    send->res1 = 0;
    send->res2 = 0;
    send->dlci = dlci;
    send->frame_type = 0;
    send->credit_flow = credit_flow;
    send->prior = prior;
    send->ack_timer = 0;

    send->frame_sizel = frame_size & 0xFF;
    send->frame_sizeh = frame_size >> 8;

    send->credits = credits;
    send->max_nbrof_retrans = 0;

    mux_send_uih(ts0710, cr, PN, data, 8);
}

/* Send a Not supported command - command, which needs 3 bytes */

static void send_nsc_msg(ts0710_con * ts0710, mcc_type cmd, u8 cr)
{
    mux_send_uih(ts0710, cr, NSC, (u8 *) &cmd, 1);
}

static void mux_send_uih(ts0710_con * ts0710, u8 cr, u8 type, u8 *data, int len)
{
    //Description: data throughput
    u8 *send = kmalloc(len + 2, GFP_ATOMIC);

    mcc_short_frame_head *head = NULL;
    if (send == NULL)
    {
        TS0710_PRINTK("mux_send_uih: ERRROR - Memory Allocation fail\n");
        return ;
    }

    head = (mcc_short_frame_head *)send;
    head->type.ea = 1;
    head->type.cr = cr;
    head->type.type = type;
    head->length.ea = 1;
    head->length.len = len;

    if (len)
        memcpy(send + 2, data, len);

    mux_send_frame(CTRL_CHAN, ts0710->initiator, MUX_UIH, send, len + 2);

    kfree(send);
}

static int mux_send_uih_data(ts0710_con * ts0710, u8 dlci, u8 *data, int len)
{
    int ret = 0;
#ifdef CONFIG_LGE_KERNEL_MUX
    u8 *send = NULL;
    if (len)
    {

        if (data != NULL)
        {


            ret = mux_send_frame(dlci, ts0710->initiator, MUX_UIH, data, len);

        }


    }
#else
    u8 *send = kmalloc(len + 2, GFP_ATOMIC);
    *send = CMDTAG;

    if (len)
        memcpy(send + 1, data, len);

    ret = mux_send_frame(dlci, ts0710->initiator, MUX_UIH, send, len + 1);

    kfree(send);
#endif
    return ret;
}

static int ts0710_msc_msg(ts0710_con * ts0710, u8 value, u8 cr, u8 dlci)
{
    u8 buf[2];
    msc_t *send = (msc_t *)buf;

    send->dlci.ea = 1;
    send->dlci.cr = 1;
    send->dlci.d = dlci & 1;
    send->dlci.server_chn = (dlci >> 1) & 0x1f;

    send->v24_sigs = value;

    mux_send_uih(ts0710, cr, MSC, buf, 2);

    return 0;
}

/* Parses a multiplexer control channel packet */

void process_mcc(u8 * data, u32 len, ts0710_con * ts0710, int longpkt)
{
    mcc_short_frame *mcc_short_pkt;
    int j;

    if (longpkt)
        mcc_short_pkt = (mcc_short_frame *) (((long_frame *) data)->data);
    else
        mcc_short_pkt = (mcc_short_frame *) (((short_frame *) data)->data);


    switch (mcc_short_pkt->h.type.type)
    {

      case FCON: 		/*Flow control on command */
        TS0710_PRINTK("MUX Received Flow control(all channels) on command\n");
        if (mcc_short_pkt->h.type.cr == MCC_CMD)
        {
            
            for (j = 0; j < TS0710_MAX_CHN; j++)
            {
                ts0710->dlci[j].state = CONNECTED;
                up(&spi_write_data_sema[j]);
                (void)down_trylock(&spi_write_data_sema[j]);
            }
            
            ts0710_fcon_msg(ts0710, MCC_RSP);
        }
        break;

      case FCOFF: 		/*Flow control off command */
        TS0710_PRINTK("MUX Received Flow control(all channels) off command\n");
        if (mcc_short_pkt->h.type.cr == MCC_CMD)
        {
            for (j = 0; j < TS0710_MAX_CHN; j++)
            {
                ts0710->dlci[j].state = FLOW_STOPPED;
            }
            ts0710_fcoff_msg(ts0710, MCC_RSP);
        }
        break;

      case MSC: 		/*Modem status command */
        {
            u8 dlci;
            u8 v24_sigs;

            dlci = (mcc_short_pkt->value[0]) >> 2;
            v24_sigs = mcc_short_pkt->value[1];
            
#ifndef CONFIG_LGE_KERNEL_MUX
            if ((ts0710->dlci[dlci].state != CONNECTED)
                    && (ts0710->dlci[dlci].state != FLOW_STOPPED))
            {
                send_dm(ts0710, dlci);
                break;
            }
#endif 
           

            if (mcc_short_pkt->h.type.cr == MCC_CMD)
            {
                TS0710_DEBUG("Received Modem status command\n");
                if ((v24_sigs & 2) && (ts0710->dlci[dlci].state == CONNECTED))
                {
                    TS0710_LOG ("MUX Received Flow off on dlci %d\n", dlci);
                    ts0710->dlci[dlci].state = FLOW_STOPPED;
                }
                else if (MUX_STOPPED(ts0710, dlci))
                {
                    ts0710->dlci[dlci].state = CONNECTED;
                    TS0710_LOG ("MUX Received Flow on on dlci %d\n", dlci);
           
                    up(&spi_write_data_sema[dlci]);
                    (void)down_trylock(&spi_write_data_sema[dlci]);
           
                }

                ts0710_msc_msg(ts0710, v24_sigs, MCC_RSP, dlci);
            }
            else
            {
                TS0710_DEBUG("Received Modem status response\n");

                if (v24_sigs & 2)
                {
                    TS0710_DEBUG("Flow stop accepted\n");
                }
            }
            break;
        }

      case PN: 		/*DLC parameter negotiation */
        {
            u8 dlci;
            u16 frame_size;
            pn_msg *pn_pkt;

            pn_pkt = (pn_msg *) data;
            dlci = pn_pkt->dlci;
            frame_size = GET_PN_MSG_FRAME_SIZE(pn_pkt);
            TS0710_DEBUG("Received DLC parameter negotiation, PN\n");

            if (pn_pkt->mcc_s_head.type.cr == MCC_CMD)
            {
                TS0710_DEBUG("received PN command with:\n");
                TS0710_DEBUG("Frame size:%d\n", frame_size);

                frame_size = min(frame_size, ts0710->dlci[dlci].mtu);
                send_pn_msg(ts0710, pn_pkt->prior, frame_size, 0, 0, dlci, MCC_RSP);
                ts0710->dlci[dlci].mtu = frame_size;
                TS0710_DEBUG("process_mcc : mtu set to %d\n", ts0710->dlci[dlci].mtu);
            }
            else
            {
                TS0710_DEBUG("received PN response with:\n");
                TS0710_DEBUG("Frame size:%d\n", frame_size);

                frame_size = min(frame_size, ts0710->dlci[dlci].mtu);
                ts0710->dlci[dlci].mtu = frame_size;


                if (ts0710->dlci[dlci].state == NEGOTIATING)
                {
                    ts0710->dlci[dlci].state = CONNECTING;
                    wake_up_interruptible(&ts0710->dlci[dlci].open_wait);
                }
            }
            break;
        }

      case NSC: 		/*Non supported command resonse */
        TS0710_LOG("MUX Received Non supported command response\n");
        break;

      default: 		/*Non supported command received */
        TS0710_LOG("MUX Received a non supported command\n");
        send_nsc_msg(ts0710, mcc_short_pkt->h.type, MCC_RSP);
        break;
    }
}

static void free_mux_recv_struct(mux_recv_struct * recv_info)
{
    if (!recv_info)
        return ;

    kfree(recv_info);
}

static inline void add_post_recv_queue(mux_recv_struct ** head,
                                       mux_recv_struct * new_item)
{
    new_item->next = *head;
    *head = new_item;
}

#define MUX_CONTROL_BUG_FIX 
static void ts0710_flow_on(u8 dlci, ts0710_con * ts0710)
{
#if defined(MUX_CONTROL_BUG_FIX) 
    if (!((ts0710->dlci[0].state) & (CONNECTED | FLOW_STOPPED)))
        return ;

    if (!((ts0710->dlci[dlci].state) & (CONNECTED | FLOW_STOPPED)))
        return ;
#else
    if (!(ts0710->dlci[0].state) & (CONNECTED | FLOW_STOPPED))
        return ;

    if (!(ts0710->dlci[dlci].state) & (CONNECTED | FLOW_STOPPED))
        return ;
#endif

    if (!(ts0710->dlci[dlci].flow_control))
        return ;

    ts0710_msc_msg(ts0710, EA | RTC | RTR | DV, MCC_CMD, dlci);

    ts0710->dlci[dlci].flow_control = 0;
}


#ifndef CONFIG_LGE_KERNEL_MUX 

static void ts0710_flow_off(struct tty_struct *tty, u8 dlci,
                            ts0710_con * ts0710)
{
    int i;

    TS0710_PRINTK("flow off\n");

    if (test_and_set_bit(TTY_THROTTLED, &tty->flags))
        return ;


    if ((ts0710->dlci[0].state != CONNECTED) && (ts0710->dlci[0].state != FLOW_STOPPED))
        return ;

    if ((ts0710->dlci[dlci].state != CONNECTED)
            && (ts0710->dlci[dlci].state != FLOW_STOPPED))
        return ;

    if (ts0710->dlci[dlci].flow_control)
        return ;

    for (i = 0; i < 3; i++)
    {
        if (ts0710_msc_msg(ts0710, EA | FC | RTC | RTR | DV, MCC_CMD, dlci) < 0)
            continue;

        TS0710_LOG("\nMUX send Flow off on dlci %d\n", dlci);
        ts0710->dlci[dlci].flow_control = 1;
        break;
    }
}

#endif 



void ts0710_recv_data_server(ts0710_con * ts0710, short_frame *short_pkt, int len)
{
    u8 be_connecting;
    u8 *uih_data_start;
    u32 uih_len;
    long_frame *long_pkt;

    switch (CLR_PF(short_pkt->h.control))
    {
      case SABM:
        TS0710_DEBUG("SABM-packet received\n");
        TS0710_DEBUG("server channel == 0\n");
        ts0710->dlci[0].state = CONNECTED;

        TS0710_DEBUG("sending back UA - control channel\n");
        send_ua(ts0710, 0);
        wake_up_interruptible(&ts0710->dlci[0].open_wait);

        break;
      case UA:
        TS0710_DEBUG("UA packet received\n");

        TS0710_DEBUG("server channel == 0\n");

        if (ts0710->dlci[0].state == CONNECTING)
        {
            ts0710->dlci[0].state = CONNECTED;
            wake_up_interruptible(&ts0710->dlci[0].
                                  open_wait);
        }
        else if (ts0710->dlci[0].state == DISCONNECTING)
        {
            ts0710_upon_disconnect();
        }
        else
        {
            TS0710_DEBUG(" Something wrong receiving UA packet\n");
        }

        break;
      case DM:
        TS0710_DEBUG("DM packet received\n");
        TS0710_DEBUG("server channel == 0\n");

        if (ts0710->dlci[0].state == CONNECTING)
        {
            be_connecting = 1;
        }
        else
        {
            be_connecting = 0;
        }
        ts0710_upon_disconnect();
        if (be_connecting)
        {
            ts0710->dlci[0].state = REJECTED;
        }

        break;
      case DISC:
        TS0710_DEBUG("DISC packet received\n");

        TS0710_DEBUG("server channel == 0\n");

        send_ua(ts0710, 0);
        TS0710_DEBUG("DISC, sending back UA\n");

        ts0710_upon_disconnect();

        break;

      case UIH:
        TS0710_DEBUG("UIH packet received\n");

        if (GET_PF(short_pkt->h.control))
        {
            TS0710_LOG("MUX Error %s: UIH packet with P/F set, discard it!\n",__FUNCTION__);
            break;
        }

        /* FIXME: connected and flow */

        if ((short_pkt->h.length.ea) == 0)
        {
            TS0710_DEBUG("Long UIH packet received\n");
            long_pkt = (long_frame *) short_pkt;
            uih_len = GET_LONG_LENGTH(long_pkt->h.length);
            uih_data_start = long_pkt->h.data;
            TS0710_DEBUG("long packet length %d\n", uih_len);

        }
        else
        {
            TS0710_DEBUG("Short UIH pkt received\n");
            uih_len = short_pkt->h.length.len;
            uih_data_start = short_pkt->data;

        }
        TS0710_DEBUG("UIH on serv_channel 0\n");
        process_mcc((u8 *)short_pkt, len, ts0710,
                    !(short_pkt->h.length.ea));

        break;

      default:
        TS0710_DEBUG("illegal packet\n");
        break;
    }

}

void process_uih(ts0710_con * ts0710, char *data, int len, u8 dlci)
{

    short_frame *short_pkt = (short_frame *) data;
    long_frame *long_pkt;
    u8 *uih_data_start;
    u32 uih_len;
#ifndef CONFIG_LGE_KERNEL_MUX
    u8 tag;
#endif
    u8 tty_idx;
    struct tty_struct *tty;
    u8 flow_control;
    mux_recv_struct *recv_info;
    int recv_room;


    if ((ts0710->dlci[dlci].state != CONNECTED)
            && (ts0710->dlci[dlci].state != FLOW_STOPPED))
    {
        TS0710_LOG("MUX Error %s: DLCI %d not connected, discard it!\n",__FUNCTION__, dlci);
        send_dm(ts0710, dlci);
        return ;
    }

    if ((short_pkt->h.length.ea) == 0)
    {
        TS0710_DEBUG("Long UIH packet received\n");
        long_pkt = (long_frame *) data;
        uih_len = GET_LONG_LENGTH(long_pkt->h.length);
        uih_data_start = long_pkt->h.data;
        TS0710_DEBUG("long packet length %d\n", uih_len);

    }
    else
    {
        TS0710_DEBUG("Short UIH pkt received\n");
        uih_len = short_pkt->h.length.len;
        uih_data_start = short_pkt->data;

    }

    TS0710_DEBUG("UIH on channel %d uih_len %d \n", dlci, uih_len);
#ifdef LGE_DUMP_MUX_BUFFER
    dump_mux_buffer("MUX[R]", data, len);
#endif
    if (uih_len > ts0710->dlci[dlci].mtu)
    {
        TS0710_PRINTK("MUX Error:  DLCI:%d, uih_len:%d is bigger than mtu:%d, discard data!\n",
                        dlci, uih_len, ts0710->dlci[dlci].mtu);
        return ;
    }
#ifndef CONFIG_LGE_KERNEL_MUX
    tag = *uih_data_start;
    uih_data_start++;
    uih_len--;

    if (!uih_len)
        return ;

    printk("TS07.10:uih tag %x on dlci %d\n", tag, dlci);
#endif

    tty_idx = dlci;
    tty = mux_table[tty_idx];
    if ((!mux_tty[tty_idx]) || (!tty))
    {
        TS0710_PRINTK("MUX: No application waiting for, discard it! /dev/mux%d\n",tty_idx);
        return ;

    }

    if ((!mux_recv_info_flags[tty_idx])
            || (!mux_recv_info[tty_idx]))
    {
        TS0710_PRINTK("MUX Error: No mux_recv_info, discard it! /dev/mux%d\n", tty_idx);
        return ;
    }

    recv_info = mux_recv_info[tty_idx];
    if (recv_info->total > 8192)
    {
        TS0710_PRINTK("MUX : discard data for tty_idx:%d, recv_info->total > 8192 \n", tty_idx);
        return ;
    }

#if defined(LGE_VT_DATA_DUMP)
    if ((12 == dlci) && mux_12_open)
    {
        //dump_mux_rd_buffer(data, len);
    }
#endif

    flow_control = 0;
    recv_room = 65535;

    if (tty->receive_room)
        recv_room = tty->receive_room;

    if ((recv_room - (uih_len + recv_info->total)) <
            ts0710->dlci[dlci].mtu)
    {
        TS0710_DEBUG("Flow OFF dlci=%d, tty->receive_room=%d uih_len=%d mtu=%d recv_info->total=%d\n", dlci, tty->receive_room, uih_len, ts0710->dlci[dlci].mtu, recv_info->total);
        flow_control = 1;
    }
    tty->ldisc->ops->receive_buf(tty, uih_data_start, NULL, uih_len);
    
    if ((dlci == 12) && (flow_control))
    {
        printk("TS07.10 DLC12 data may be lost on next recieves!\n");
        return ;
    }
    
    if (flow_control)
    
        tty_throttle(tty);
    

}

void ts0710_recv_data(ts0710_con * ts0710, char *data, int len)
{
    short_frame *short_pkt;
    u8 dlci;

    short_pkt = (short_frame *) data;

    dlci = short_pkt->h.addr.server_chn << 1 | short_pkt->h.addr.d;
    if (!dlci)
        return ts0710_recv_data_server(ts0710, short_pkt, len);

    switch (CLR_PF(short_pkt->h.control))
    {
      case SABM:
        TS0710_PRINTK("Incomming connect on channel %d\n", dlci);

        send_ua(ts0710, dlci);

        ts0710->dlci[dlci].state = CONNECTED;
        wake_up_interruptible(&ts0710->dlci[dlci].open_wait);

        break;
      case UA:
        TS0710_DEBUG("Incomming UA on channel %d\n", dlci);

        if (ts0710->dlci[dlci].state == CONNECTING)
        {
            ts0710->dlci[dlci].state = CONNECTED;
            wake_up_interruptible(&ts0710->dlci[dlci].
                                  open_wait);
        }
        else if (ts0710->dlci[dlci].state == DISCONNECTING)
        {
            ts0710->dlci[dlci].state = DISCONNECTED;
            wake_up_interruptible(&ts0710->dlci[dlci].
                                  open_wait);
            wake_up_interruptible(&ts0710->dlci[dlci].
                                  close_wait);
            ts0710_reset_dlci(dlci);
        }

        break;
      case DM:
        TS0710_DEBUG("Incomming DM on channel %d\n", dlci);

        if (ts0710->dlci[dlci].state == CONNECTING)
            ts0710->dlci[dlci].state = REJECTED;
        else
            ts0710->dlci[dlci].state = DISCONNECTED;

        wake_up_interruptible(&ts0710->dlci[dlci].open_wait);
        wake_up_interruptible(&ts0710->dlci[dlci].close_wait);
        ts0710_reset_dlci(dlci);


        break;
      case DISC:
        send_ua(ts0710, dlci);

        ts0710->dlci[dlci].state = DISCONNECTED;
        wake_up_interruptible(&ts0710->dlci[dlci].open_wait);
        wake_up_interruptible(&ts0710->dlci[dlci].close_wait);
        ts0710_reset_dlci(dlci);

        break;
      case UIH:
        process_uih(ts0710, data, len, dlci);

        break;
      default:
        TS0710_PRINTK("illegal packet\n");
        TS0710_PRINTK("ts0710_recv_data:illegal packet for DLCi %d, hdr=%x \n", dlci, CLR_PF(short_pkt->h.control));
        break;
    }
}


#ifndef CONFIG_LGE_KERNEL_MUX 
static void ts0710_close_channel(u8 dlci)
{
    ts0710_con *ts0710 = &ts0710_connection;
    int try;
    unsigned long t;

    TS0710_PRINTK("ts0710_close_channel command on channel %d\n", dlci);

    if ((ts0710->dlci[dlci].state == DISCONNECTED)
            || (ts0710->dlci[dlci].state == REJECTED))
    {
        return ;
    }
    else if (ts0710->dlci[dlci].state == DISCONNECTING)
    {
        /* Reentry */
        return ;
    }

    ts0710->dlci[dlci].state = DISCONNECTING;
    try = 3;
    while (try--)
    {
        t = jiffies;
        send_disc(ts0710, dlci);
        interruptible_sleep_on_timeout(&ts0710->dlci[dlci].
                                       close_wait,
                                       TS0710MUX_TIME_OUT);
        if (ts0710->dlci[dlci].state == DISCONNECTED)
        {
            break;
        }
        else if (signal_pending(current))
        {
            TS0710_PRINTK ("MUX DLCI %d Send DISC got signal!\n", dlci);
            break;
        }
        else if ((jiffies - t) >= TS0710MUX_TIME_OUT)
        {
            TS0710_PRINTK ("MUX DLCI %d Send DISC timeout!\n", dlci);
            continue;
        }
    }

    if (ts0710->dlci[dlci].state != DISCONNECTED)
    {
        if (dlci == 0)
        {	/* Control Channel */
            ts0710_upon_disconnect();
        }
        else
        {	/* Other Channel */
            ts0710->dlci[dlci].state = DISCONNECTED;
            wake_up_interruptible(&ts0710->dlci[dlci].
                                  close_wait);
            ts0710_reset_dlci(dlci);
        }
    }
}

#endif 


int ts0710_open_channel(u8 dlci)
{
    ts0710_con *ts0710 = &ts0710_connection;
    int try;
    int retval;
    unsigned long t;

    retval = -ENODEV;
    if (dlci == 0)
    {	/* control channel */
        if (ts0710->dlci[0].state & (CONNECTED | FLOW_STOPPED))
        {
            return 0;
        }
        else if (ts0710->dlci[0].state == CONNECTING)
        {
            /* Reentry */
            TS0710_PRINTK("MUX DLCI: 0, reentry to open DLCI 0, pid: %d, %s !\n",
                          current->pid, current->comm);
            try = 11;
            while (try--)
            {
                t = jiffies;
                interruptible_sleep_on_timeout(&ts0710->dlci[0].
                                               open_wait,
                                               TS0710MUX_TIME_OUT);
                if ((ts0710->dlci[0].state == CONNECTED)
                        || (ts0710->dlci[0].state ==
                            FLOW_STOPPED))
                {
                    retval = 0;
                    break;
                }
                else if (ts0710->dlci[0].state == REJECTED)
                {
                    retval = -EREJECTED;
                    break;
                }
                else if (ts0710->dlci[0].state ==
                         DISCONNECTED)
                {
                    break;
                }
                else if (signal_pending(current))
                {
                    TS0710_PRINTK ("MUX DLCI:%d Wait for connecting got signal!\n", dlci);
                    retval = -EAGAIN;
                    break;
                }
                else if ((jiffies - t) >= TS0710MUX_TIME_OUT)
                {
                    TS0710_PRINTK("MUX DLCI:%d Wait for connecting timeout!\n", dlci);
                    continue;
                }
                else if (ts0710->dlci[0].state == CONNECTING)
                {
                    continue;
                }
            }

            if (ts0710->dlci[0].state == CONNECTING)
            {
                ts0710->dlci[0].state = DISCONNECTED;
            }
        }
        else if ((ts0710->dlci[0].state != DISCONNECTED)
                 && (ts0710->dlci[0].state != REJECTED))
        {
            TS0710_PRINTK("MUX DLCI:%d state is invalid!\n", dlci);
            return retval;
        }
        else
        {
            ts0710->initiator = 1;
            ts0710->dlci[0].state = CONNECTING;
            ts0710->dlci[0].initiator = 1;

            t = jiffies;
            send_sabm(ts0710, 0);
            interruptible_sleep_on_timeout(&ts0710->dlci[0].
                                           open_wait,
                                           TS0710MUX_TIME_OUT);
            if ((ts0710->dlci[0].state == CONNECTED)
                    || (ts0710->dlci[0].state ==
                        FLOW_STOPPED))
            {
                retval = 0;
            }
            else if (ts0710->dlci[0].state == REJECTED)
            {
                TS0710_PRINTK("MUX DLCI:%d Send SABM got rejected!\n", dlci);
                retval = -EREJECTED;
            }
            else if (signal_pending(current))
            {
                TS0710_PRINTK ("MUX DLCI:%d Send SABM got signal!\n", dlci);
                retval = -EAGAIN;
            }
            else if ((jiffies - t) >= TS0710MUX_TIME_OUT)
            {
                TS0710_PRINTK("MUX DLCI:%d Send SABM timeout!\n",dlci);
                retval = -ENODEV;
            }

            if (ts0710->dlci[0].state == CONNECTING)
            {
                ts0710->dlci[0].state = DISCONNECTED;
            }
            wake_up_interruptible(&ts0710->dlci[0].open_wait);
        }
    }
    else
    {		/* other channel */
        if ((ts0710->dlci[0].state != CONNECTED)
                && (ts0710->dlci[0].state != FLOW_STOPPED))
        {
            return retval;
        }
        else if ((ts0710->dlci[dlci].state == CONNECTED)
                 || (ts0710->dlci[dlci].state == FLOW_STOPPED))
        {
            return 0;
        }
        else if ((ts0710->dlci[dlci].state == NEGOTIATING)
                 || (ts0710->dlci[dlci].state == CONNECTING))
        {
            /* Reentry */

            t = jiffies;
            interruptible_sleep_on_timeout(&ts0710->
                                           dlci[dlci].
                                           open_wait,
                                           TS0710MUX_TIME_OUT);
            if ((ts0710->dlci[dlci].state == CONNECTED)
                    || (ts0710->dlci[dlci].state ==
                        FLOW_STOPPED))
            {
                retval = 0;
            }
            else if (ts0710->dlci[dlci].state == REJECTED)
            {
                retval = -EREJECTED;
            }
            else if (signal_pending(current))
            {
                retval = -EAGAIN;
            }

            if ((ts0710->dlci[dlci].state == NEGOTIATING)
                    || (ts0710->dlci[dlci].state == CONNECTING))
            {
                ts0710->dlci[dlci].state = DISCONNECTED;
            }
        }
        else if ((ts0710->dlci[dlci].state != DISCONNECTED)
                 && (ts0710->dlci[dlci].state != REJECTED))
        {
            TS0710_PRINTK("MUX DLCI:%d state is invalid!\n", dlci);
            return retval;
        }
        else
        {
#ifdef CONFIG_LGE_KERNEL_MUX
            ts0710->dlci[dlci].state = CONNECTING;
#else
            ts0710->dlci[dlci].state = NEGOTIATING;
            ts0710->dlci[dlci].initiator = 1;

            t = jiffies;
            send_pn_msg(ts0710, 7, ts0710->dlci[dlci].mtu,
                        0, 0, dlci, 1);
            interruptible_sleep_on_timeout(&ts0710->
                                           dlci[dlci].
                                           open_wait,
                                           TS0710MUX_TIME_OUT);
            if (signal_pending(current))
            {
                TS0710_PRINTK("MUX DLCI:%d Send pn_msg got signal!\n",dlci);
                retval = -EAGAIN;
            }
#endif
            if (ts0710->dlci[dlci].state == CONNECTING)
            {

                t = jiffies;
                send_sabm(ts0710, dlci);
                interruptible_sleep_on_timeout(&ts0710->
                                               dlci
                                               [dlci].
                                               open_wait,
                                               TS0710MUX_TIME_OUT);
                if ((ts0710->dlci[dlci].state ==
                        CONNECTED)
                        || (ts0710->dlci[dlci].state ==
                            FLOW_STOPPED))
                {
                    retval = 0;
                }
                else if (ts0710->dlci[dlci].state ==
                         REJECTED)
                {
                    TS0710_PRINTK("MUX DLCI:%d Send SABM got rejected!\n", dlci);
                    retval = -EREJECTED;
                }
                else if (signal_pending(current))
                {
                    TS0710_PRINTK("MUX DLCI:%d Send SABM got signal!\n", dlci);
                    retval = -EAGAIN;
                }
            }

            if ((ts0710->dlci[dlci].state == NEGOTIATING)
                    || (ts0710->dlci[dlci].state == CONNECTING))
            {
                ts0710->dlci[dlci].state = DISCONNECTED;
            }
            wake_up_interruptible(&ts0710->dlci[dlci].open_wait);
        }
    }
    return retval;
}

/****************************
 * TTY driver routines
*****************************/

static void mux_close(struct tty_struct *tty, struct file *filp)
{
    ts0710_con *ts0710 = &ts0710_connection;
    int line;
    u8 dlci;

    printk("%s \n",__FUNCTION__);
    
    UNUSED_PARAM(filp);
    
    if (mux_filp[tty->index] == filp)
    {
        mux_filp[tty->index] = NULL;
    }
    
#if defined(LGE_VT_CALL_SESSION) || defined(VT_MUX12_DEBUG) 
    if (DLCI_VIDEO_TELEPHONY == tty->index)
    {
        mux_12_open = 0;
    }
#endif
    line = tty->index;
    if (MUX_INVALID(line))
        return ;
    if (mux_tty[line] > 0)
        mux_tty[line]--;

    dlci = line;
    if (mux_tty[line] == 0)
#ifdef CONFIG_LGE_KERNEL_MUX 
        /*Workaround Infineon modem - DLS opened once will never be closed*/
        return ;
#else
        ts0710_close_channel(dlci);
#endif


    if (mux_tty[line] != 0)
        return ;

    if ((mux_send_info_flags[line]) && (mux_send_info[line]))
    {
        mux_send_info_flags[line] = 0;
        kfree(mux_send_info[line]);
        mux_send_info[line] = 0;
        TS0710_DEBUG("Free mux_send_info for /dev/mux%d\n", line);
    }

    if ((mux_recv_info_flags[line])
            && (mux_recv_info[line])
            && (mux_recv_info[line]->total == 0))
    {
        mux_recv_info_flags[line] = 0;
        free_mux_recv_struct(mux_recv_info[line]);
        mux_recv_info[line] = 0;
        TS0710_DEBUG("Free mux_recv_info for /dev/mux%d\n", line);
    }
    
    tty_unthrottle(tty);
    

    wake_up_interruptible(&tty->read_wait);
    wake_up_interruptible(&tty->write_wait);
    tty->packet = 0;
}

static void mux_throttle(struct tty_struct *tty)
{
    ts0710_con *ts0710 = &ts0710_connection;
    int line;
    int i;
    u8 dlci;

    line = tty->index;
    if (MUX_INVALID(line))
        return ;


    TS0710_DEBUG("Enter into %s, minor number is: %d\n", __FUNCTION__,line);

    dlci = line;
    if ((ts0710->dlci[0].state != CONNECTED)
            && (ts0710->dlci[0].state != FLOW_STOPPED))
    {
        return ;
    }
    else if ((ts0710->dlci[dlci].state != CONNECTED)
             && (ts0710->dlci[dlci].state != FLOW_STOPPED))
    {
        return ;
    }

    if (ts0710->dlci[dlci].flow_control)
    {
        return ;
    }

    for (i = 0; i < 3; i++)
    {
        if (ts0710_msc_msg
                (ts0710, EA | FC | RTC | RTR | DV, MCC_CMD, dlci) < 0)
        {
            continue;
        }
        else
        {
            TS0710_LOG("MUX Send Flow off on dlci %d\n", dlci);
            ts0710->dlci[dlci].flow_control = 1;
            break;
        }
    }
}

static void mux_unthrottle(struct tty_struct *tty)
{
    ts0710_con *ts0710 = &ts0710_connection;
    int line;
    u8 dlci;
    mux_recv_struct *recv_info;

    line = tty->index;
    if (MUX_INVALID(line))
        return ;

    if ((!mux_recv_info_flags[line]) || (!mux_recv_info[line]))
    {
        return ;
    }

    TS0710_DEBUG("Enter into %s, minor number is: %d\n", __FUNCTION__,
                 line);

    recv_info = mux_recv_info[line];
    dlci = line;
    
    ts0710_flow_on(dlci, ts0710);
    
}

static int mux_chars_in_buffer(struct tty_struct *tty)
{
    ts0710_con *ts0710 = &ts0710_connection;
    int retval;
    int line;
    u8 dlci;
    mux_send_struct *send_info;

    retval = TS0710MUX_MAX_CHARS_IN_BUF;

    line = tty->index;
    if (MUX_INVALID(line))
    {
        goto out;
    }

    dlci = line;

    if (! MUX_USABLE(ts0710, dlci))
        goto out;

    if (!(mux_send_info_flags[line]))
    {
        goto out;
    }
    send_info = mux_send_info[line];
    if (!send_info)
    {
        goto out;
    }
    if (send_info->filled)
    {
        goto out;
    }

    retval = 0;

out:
    return retval;
}

static int mux_write(struct tty_struct *tty,
                     const unsigned char *buf, int count)
{
    ts0710_con *ts0710 = &ts0710_connection;
    u8 dlci;
    int written = 0;
#ifdef CONFIG_LGE_KERNEL_MUX
    int frame_size = 0;
    int frame_written = 0;
#endif

#if defined(LGE_VT_DATA_DUMP)
    static int pts12_write_cnt = 0;
    int data_limit = 0;
#endif

#ifdef LGE_DUMP_MUX_BUFFER
    dump_mux_buffer("MUX[W]", buf, count);
    
#elif defined(LGE_VT_DATA_DUMP)
    if ((160 == count) && (12 == tty->index))
    {

#if defined(LGE_VT_MUX_DATA_REPLACE)
        /* Compose MUX data with serial number (1,2,3,4,...) */
        if (pts12_write_cnt < 128)
        {
            data_limit = pts12_write_cnt;
        }
        else
        {
            data_limit = pts12_write_cnt % 128;
        }

        memset(buf, (void*) (VT_SPECIAL_CHAR + data_limit) , count);
#endif

        printk("MUX WR[%d]\n", pts12_write_cnt );

        mux_written_counter = pts12_write_cnt;

        pts12_write_cnt++;
    }
#endif

    if (!count)
        return 0;

    dlci = tty->index;

#ifdef CONFIG_LGE_KERNEL_MUX 
    
    while (count > 0)
    
    {
    
        if (ts0710->dlci[dlci].state == FLOW_STOPPED)
        {
            TS0710_DEBUG("TS0710 Write: Flow OFF state = %d \n", ts0710->dlci[dlci].state);

            if (in_interrupt())
            {
                TS0710_DEBUG("TS0710 Write: returning EIO for flow stopped channel in interrupt\n");
                return -EIO;
            }

            if ((mux_filp[dlci] != NULL) && (mux_filp[dlci]->f_flags & O_NONBLOCK))
            {
                TS0710_DEBUG("TS0710 Write: returning EWOULDBLOCK for flow stopped channel\n");
                return -EWOULDBLOCK;
            }

            if (down_interruptible(&spi_write_data_sema[dlci]))
            {
                TS0710_DEBUG("TS0710 Write: returning ERESTARTSYS for flow stopped channel because of signal\n");
                return -ERESTARTSYS;
            }
        }
        frame_size = min(count, ts0710->dlci[dlci].mtu);


        frame_written = mux_send_uih_data(ts0710, dlci, (u8 *)(buf + written), frame_size);

        if (frame_written < 0)
        {
            TS0710_DEBUG("\nTS0710 Write: returning error = %d \n", frame_written);
            if (in_interrupt())
            {
                TS0710_DEBUG("TS0710 Write: returning EIO for error send in interrupt\n");
                return -EIO;
            }
            if ((mux_filp[dlci] != NULL) && (mux_filp[dlci]->f_flags & O_NONBLOCK))
            {
                TS0710_DEBUG("TS0710 Write: returning EWOULDBLOCK for error send\n");
                return -EWOULDBLOCK;
            }
            /* send frame error */
            return frame_written;
            
        }
        else
        {
            written += frame_written;
            count -= frame_written;
        }
    };
#else
    written = mux_send_uih_data(ts0710, dlci, (u8 *)buf, count) - 7;
#endif
    return written;
}

static int mux_write_room(struct tty_struct *tty)
{
    ts0710_con *ts0710 = &ts0710_connection;
    int retval;
    int line;
    u8 dlci;
    mux_send_struct *send_info;

    retval = 0;

    line = tty->index;
    if (MUX_INVALID(line))
        goto out;

    dlci = line;
    if (ts0710->dlci[0].state == FLOW_STOPPED)
    {
        TS0710_DEBUG("Flow stopped on all channels, returning ZERO\n");
        goto out;
    }
    else if (ts0710->dlci[dlci].state == FLOW_STOPPED)
    {
        TS0710_DEBUG("Flow stopped, returning ZERO\n");
        goto out;
    }
    else if (ts0710->dlci[dlci].state != CONNECTED)
    {
        TS0710_DEBUG("DLCI %d not connected\n", dlci);
        goto out;
    }

    if (!(mux_send_info_flags[line]))
        goto out;

    send_info = mux_send_info[line];
    if (!send_info)
        goto out;

    if (send_info->filled)
        goto out;

    retval = ts0710->dlci[dlci].mtu - 1;

out:
    
    return retval;
}

static void mux_flush_buffer(struct tty_struct *tty)
{
    int line;

    line = tty->index;
    if (MUX_INVALID(line))
        return ;


    TS0710_PRINTK("MUX %s: line is:%d\n", __FUNCTION__, line);

    if ((mux_send_info_flags[line])
            && (mux_send_info[line])
            && (mux_send_info[line]->filled))
    {

        mux_send_info[line]->filled = 0;
    }

    wake_up_interruptible(&tty->write_wait);

    if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) &&
            tty->ldisc->ops->write_wakeup)
    {
        tty->ldisc->ops->write_wakeup(tty);
    }
}

static int mux_open(struct tty_struct *tty, struct file *filp)
{
    int retval;
    int line;
    u8 dlci;
    mux_send_struct *send_info;
    mux_recv_struct *recv_info;


    retval = -ENODEV;

    line = tty->index;
    
    
    //Description: data throughput
    if (mux_filp[line] != NULL)
        TS0710_PRINTK("mux_open start: mux_filp[%d] opened again", line);

    mux_filp[line] = filp;

    TS0710_PRINTK("mux_open start: %s ch: %d\n", current->comm, line);
    if (!(ipc_tty && line))
    {
        return -ENODEV;
    }

    
    mux_tty[line]++;
    dlci = line;
    mux_table[line] = tty;

    /* Open server channel 0 first */
    if ((retval = ts0710_open_channel(0)) != 0)
    {
        TS0710_PRINTK("MUX: Can't connect server channel 0!\n");
        ts0710_init();

        mux_tty[line]--;
        goto out;
    }

    /* Allocate memory first. As soon as connection has been established, MUX may receive */
    if (mux_send_info_flags[line] == 0)
    {
        send_info =
            (mux_send_struct *) kmalloc(sizeof(mux_send_struct),
                                        GFP_KERNEL);
        if (!send_info)
        {
            retval = -ENOMEM;

            mux_tty[line]--;
            goto out;
        }
        send_info->length = 0;
        send_info->flags = 0;
        send_info->filled = 0;
        mux_send_info[line] = send_info;
        mux_send_info_flags[line] = 1;
        TS0710_DEBUG("Allocate mux_send_info for /dev/pts%d", line);
    }

    if (mux_recv_info_flags[line] == 0)
    {
        recv_info =
            (mux_recv_struct *) kmalloc(sizeof(mux_recv_struct),
                                        GFP_KERNEL);
        if (!recv_info)
        {
            mux_send_info_flags[line] = 0;
            kfree(mux_send_info[line]);
            mux_send_info[line] = 0;
            TS0710_DEBUG("Free mux_send_info for /dev/pts%d", line);
            retval = -ENOMEM;

            mux_tty[line]--;
            goto out;
        }
        recv_info->length = 0;
        recv_info->total = 0;
        recv_info->next = 0;
        recv_info->no_tty = line;
        recv_info->post_unthrottle = 0;
        mux_recv_info[line] = recv_info;
        mux_recv_info_flags[line] = 1;
        TS0710_DEBUG("Allocate mux_recv_info for /dev/mux%d", line);
    }

    /* Now establish DLCI connection */
    if (mux_tty[dlci] > 0)
    {
        if ((retval = ts0710_open_channel(dlci)) != 0)
        {
            TS0710_PRINTK("MUX: Can't connected channel %d!\n",dlci);
            ts0710_reset_dlci(dlci);

            mux_send_info_flags[line] = 0;
            kfree(mux_send_info[line]);
            mux_send_info[line] = 0;
            TS0710_DEBUG("Free mux_send_info for /dev/mux%d", line);

            mux_recv_info_flags[line] = 0;
            free_mux_recv_struct(mux_recv_info[line]);
            mux_recv_info[line] = 0;
            TS0710_DEBUG("Free mux_recv_info for /dev/mux%d", line);

            mux_tty[line]--;
            goto out;
        }
    }
    

#if defined(LGE_VT_CALL_SESSION) || defined(VT_MUX12_DEBUG) 
    if (DLCI_VIDEO_TELEPHONY == dlci)
    {
        mux_12_open = 1;
#ifdef VT_MUX12_DEBUG
        TS0710_PRINTK("mux_open : mux12 \n");
#endif 
        

    }
#endif

    retval = 0;
out:
    return retval;
}

/* mux dispatcher, call from serial.c receiver_chars() */
void mux_dispatcher(struct tty_struct *tty)
{
    UNUSED_PARAM(tty);

    
}


#ifndef CONFIG_LGE_KERNEL_MUX
static void send_ack(ts0710_con * ts0710, u8 seq_num)
{
    mux_send_frame(CTRL_CHAN, ts0710->initiator, ACK, &seq_num, 1);
}
#endif 


static void ts_ldisc_rx_post(struct tty_struct *tty, const u8 *data, char *flags, int count)
{
#ifndef CONFIG_LGE_KERNEL_MUX
    static u8 expect_seq = 0;
#endif
    int framelen;
    short_frame *short_pkt;
    long_frame *long_pkt;

    short_pkt = (short_frame *) (data + ADDRESS_FIELD_OFFSET);
    if (short_pkt->h.length.ea == 1)
    {
        framelen = TS0710_MAX_HDR_SIZE + short_pkt->h.length.len + 1 + SEQ_FIELD_SIZE;
    }
    else
    {
        long_pkt = (long_frame *) (data + ADDRESS_FIELD_OFFSET);
        framelen = TS0710_MAX_HDR_SIZE + GET_LONG_LENGTH(long_pkt->h.length) + 2 + SEQ_FIELD_SIZE;
    }
#ifdef CONFIG_LGE_KERNEL_MUX 
    /* To remove Motorola's modem specific*/
    ts0710_recv_data(&ts0710_connection,
                     (char*)(data + 1),
                     framelen - 2);
#else
    TS0710_PRINTK("MUX: ts_ldisc_rx_post: expect_seq = %x \n", expect_seq );
    TS0710_PRINTK("MUX: ts_ldisc_rx_post: *(data + SLIDE_BP_SEQ_OFFSET) = %x \n", *(data + SLIDE_BP_SEQ_OFFSET) );
    if (expect_seq == *(data + SLIDE_BP_SEQ_OFFSET))
    {
        expect_seq++;
        if (expect_seq >= 4)
            expect_seq = 0;

        send_ack(&ts0710_connection, expect_seq);

        ts0710_recv_data(&ts0710_connection,
                         (char*)(data + ADDRESS_FIELD_OFFSET),
                         framelen - 2 - SEQ_FIELD_SIZE);
    }
    else
    {
        TS0710_PRINTK("miss seq. possibly lost sync!\n");
    }
#endif

}

void ts_ldisc_rx(struct tty_struct *tty, const u8 *data, char *flags, int size)
{
    struct mux_data *st = (struct mux_data *)tty->disc_data;
#ifdef CONFIG_LGE_KERNEL_MUX
    int i;
    short_frame *short_pkt;
    long_frame *long_pkt;
    for (i = 0; i < size; i++)
    {
        switch (st->state)
        {
          case OUT_OF_FRAME:
            if (data[i] == (u8)TS0710_BASIC_FLAG)
            {
                st->state = INSIDE_FRAME_HEADER;
                st->copy_index = 0;
            }
            else
            {
                printk("\nTS0710:ts_ldisc_rx:bad_data,%x\n", data[i]);
                break;
            }
          case INSIDE_FRAME_HEADER:
            if (st->copy_index < TS0710_MAX_HDR_SIZE)
            {
                if ((st->copy_index == 1) && (data[i] == (u8)TS0710_BASIC_FLAG)) /* this is fix of out of sync with seq F9 F9*/
                    st->copy_index = 0;
                st->chunk[st->copy_index++] = data[i];
                break;
            }
            else
            {
                short_pkt = (short_frame *) (st->chunk + ADDRESS_FIELD_OFFSET);
                if (short_pkt->h.length.ea == 1)
                {
                    st->framelen = TS0710_MAX_HDR_SIZE + short_pkt->h.length.len + 1 + SEQ_FIELD_SIZE;
                }
                else
                {
                    long_pkt = (long_frame *) short_pkt;
                    st->framelen = TS0710_MAX_HDR_SIZE + GET_LONG_LENGTH(long_pkt->h.length) + 2 + SEQ_FIELD_SIZE;
                }
                if (st->framelen > MAX_FRAME_SIZE)
                {
                    printk("\nTS0710:ts_ldisc_rx:Too big frame to copy %d!\n", st->framelen);
                    st->state = OUT_OF_FRAME ;
                    break;
                }
                st->state = INSIDE_FRAME_BODY;
            }
          case INSIDE_FRAME_BODY:
            if (st->copy_index < st->framelen)
            {
                st->chunk[st->copy_index++] = data[i];
            }
            if (!(st->copy_index < st->framelen))
            {
                TS0710_DEBUG("Good Frame : data[%d]=0x%x framelen=%d\n", i, data[i], st->framelen);
                ts_ldisc_rx_post(tty, st->chunk, flags, st->framelen);
                st->state = OUT_OF_FRAME;
            }
            break;
          default:
            printk("\nTS0710:ts_ldisc_rx:unknown state!!!!!!!\n");
            return ;
        }
    }
#else
    u8 *packet_start = data;
    while (size--)
    {
        if (*data == 0xF9)
        {
            if (st->state == OUT_OF_PACKET)
            {
                st->state = INSIDE_PACKET;
                packet_start = data;
            }
            else
            {
                /* buffer points at ending 0xF9 */
                int packet_size = (data - packet_start) + 1;

                if (packet_size + st->chunk_size == 2)
                {
                    packet_start++;
                    data++;
                    continue;
                }

                st->state = OUT_OF_PACKET;

                if (!st->chunk_size)
                    ts_ldisc_rx_post(
                        tty,
                        packet_start,
                        flags,
                        packet_size
                    );
                else
                { /* use existing chunk */
                    memcpy(st->chunk + st->chunk_size, packet_start, packet_size);
                    ts_ldisc_rx_post(tty, st->chunk, flags,
                                     st->chunk_size + packet_size);

                    st->chunk_size = 0;
                }
            }
        }
        data++;
    }
    if (st->state == INSIDE_PACKET) /* create/update chunk */
    {
        size_t new_chunk_size = data - packet_start; /* buffer points right after data end */
        memcpy(st->chunk + st->chunk_size, packet_start, new_chunk_size);
        st->chunk_size += new_chunk_size;
    }
#endif
}

#ifdef CONFIG_LGE_KERNEL_MUX 
volatile int time_to_stop;

static int ts_ldisc_tx_looper(void *param)
{
    int i, res;

    u8 dlci = 0;
    int writeCount = 0;
    u8 *write_ptr = NULL;

    u8 *data_ptr;
    int data_size;
    
    int is_frames = 0;
    struct spi_data_send_struct *next_ptr = NULL;
    struct spi_data_send_struct *prev_ptr = NULL;
    
    unsigned long flag;		

    set_freezable(); 
    
    while (!time_to_stop && (ipc_tty != NULL))
    {
     
        LOCK(frame_nodes_lock);
     
        data_size = 0;
        data_ptr = NULL;
     
        is_frames = 0;
        for (i = 0; i < TS0710MAX_PRIORITY_NUMBER ; i++)
        {
            if (frame_to_send[i] != NULL)
            {
                dlci = frame_to_send[i]->dlci;
                is_frames = 1;
                if (ts0710_connection.dlci[dlci].state == FLOW_STOPPED)
                {
                    prev_ptr = frame_to_send[i];
                    next_ptr = prev_ptr->next;
                    while (next_ptr != NULL)
                    {
                        dlci = next_ptr->dlci;
                        if (ts0710_connection.dlci[dlci].state != FLOW_STOPPED)
                        {
                            prev_ptr->next = next_ptr->next;
                            next_ptr->next = frame_to_send[i];
                            frame_to_send[i] = next_ptr;
                            break;
                        }
                        prev_ptr = next_ptr;
                        next_ptr = next_ptr->next;
                    }
                    if (next_ptr == NULL)
                    {
                        continue;
                    }
                }
                data_ptr = frame_to_send[i]->data;
                data_size = frame_to_send[i]->size;
                next_ptr = frame_to_send[i]->next;
                frame_to_send[i]->size = 0;
                frame_to_send[i]->dlci = 0;
                frame_to_send[i]->data = NULL;
                frame_to_send[i]->next = NULL;
                frame_to_send[i] = next_ptr;
                --frames_to_send_count[dlci];
                
                break;
            }
        }
       

        UNLOCK(frame_nodes_lock);
        if (i < TS0710MAX_PRIORITY_NUMBER)
        {
            wake_up_interruptible(&wq);
            write_ptr = data_ptr;    // don't change data_ptr. used to free memory
            while ((data_size > 0) && (write_ptr != NULL) && (ipc_tty != NULL))
            {

                res = ipc_tty->ops->write(ipc_tty, write_ptr, data_size);
                if (res < 0 )
                {

                    printk("\nTS0710:ts_ldisc_tx_looper: ERROR Writing on IPC: data_size=%d, res=%d\n", data_size, res);
                    writeCount++;
                    if (writeCount >= MUX_WRITE_MAX_RETRY)
                    {
                        printk("ts_ldisc_tx_looper: FATAL Writing on IPC: Max Retry count (%d) exceeded. Dropping the Packet\n", writeCount);
                        break;
                    }
                }
                else if (res >= 0 && res < data_size)
                {

                    printk("\nTS0710:ts_ldisc_tx_looper: ERROR Writing on IPC: data_size=%d, res=%d\n", data_size, res);
                    write_ptr += res;
                    data_size -= res;
                    writeCount++;
                    if (writeCount >= MUX_WRITE_MAX_RETRY)
                    {
                        printk("ts_ldisc_tx_looper: FATAL Writing on IPC: Max Retry count (%d) exceeded. Dropping the Packet\n", writeCount);
                        break;
                    }
                }
                else
                {
                    //IPC WRITE OK
                    data_size -= res;  //or break;
                }
            }

            


            kfree(data_ptr);
        }
        
        if (is_frames)
        {
        
            TS0710_PRINTK("ts_ldisc_tx_looper: spi_write_sema: up\n");
            up(&spi_write_data_sema[dlci]);
  
        }
	
	try_to_freeze();

        down_timeout(&spi_write_sema, TS0710MUX_TIME_OUT);
    
    }
    
    while (!kthread_should_stop())
    {
        msleep(1);
    }
    
    LOCK(frame_nodes_lock);
    TS0710_PRINTK("ts_ldisc_tx_looper: free the write queue\n");
    for (i = 0; i < TS0710MAX_PRIORITY_NUMBER;i++)
    {
        while (frame_to_send[i] != NULL)
        {
            if (frame_to_send[i]->data)
            {
                kfree(frame_to_send[i]->data);
                frame_to_send[i]->data = NULL;
            }
            frame_to_send[i]->size = 0;
            frame_to_send[i] = frame_to_send[i]->next;
        } // end of while
    } // end of for
    TS0710_PRINTK("ts_ldisc_tx_looper: write queue is freed\n");
    UNLOCK(frame_nodes_lock);
    for (i = 0; i < TS0710_MAX_CHN; i++)
    {
        up(&spi_write_data_sema[i]);
    }
    write_task = NULL;
    TS0710_PRINTK("ts_ldisc_tx_looper: stopped\n");
    
    return 0;
}

#endif

static int ts_ldisc_open(struct tty_struct *tty)
{
    struct mux_data *disc_data;
    int i = 0;

    tty->receive_room = 65536;

    disc_data = kzalloc(sizeof(struct mux_data), GFP_KERNEL);
    
    if (disc_data == NULL)
    {
        TS0710_PRINTK("ldisc_open fail : Memory Allocation fail\n");
        return -ENOMEM;
    }

    
    disc_data->state = OUT_OF_FRAME;
    disc_data->chunk_size = 0;

    tty->disc_data = disc_data;

    for (i = 0; i < TS0710_MAX_CHN; i++)
    {
        spi_write_data_sema[i].lock = __SPIN_LOCK_UNLOCKED(spi_write_data_sema[i].lock);
        spi_write_data_sema[i].count = 0;
        spi_write_data_sema[i].wait_list.next = &(spi_write_data_sema[i].wait_list);
        spi_write_data_sema[i].wait_list.prev = &(spi_write_data_sema[i].wait_list);
    }
    ipc_tty = tty;

#ifdef CONFIG_LGE_KERNEL_MUX 
    if (write_task == NULL)
    {
        CREATELOCK(frame_nodes_lock);
    }


    nodes_init();
    ts0710_reset_dlci_priority();
    spi_data_recieved.tty = NULL;
    spi_data_recieved.data = NULL;
    spi_data_recieved.flags = NULL;
    spi_data_recieved.size = 0;
    spi_data_recieved.updated = 0;
    time_to_stop = 0;
    task = NULL;

    write_task = NULL;
    write_task = kthread_run(ts_ldisc_tx_looper, NULL, "%s", "ts_ldisc_tx_looper");
    if (write_task == NULL)
        TS0710_DEBUG("ts_ldisc_open WRITE_THREAD is not started!!!\n");
#endif
    TS0710_PRINTK("ts_ldisc_open executed\n");

    return 0;
}
#ifdef LGE_ENABLE_RIL_RECOVERY_MODE 

int is_usb_disconnect = 0;
int is_cp_crash = 0;    

void ifx_cp_reset(void)
{

#if 0 //spi

#else
    // USB-HSIC
    baseband_xmm_power_switch(0);
    mdelay(100);
    baseband_xmm_power_switch(1);
    mdelay(3000);
#endif

}



#if 0 //defined(RECOVERY_CNT)
int ril_recovery_cnt = 0;
#endif
static void ts_ldisc_close(struct tty_struct *tty)
{
    ipc_tty = 0;

    printk("**************************\n");
    printk("!!!!! ts_ldisc_close !!!!!\n", __FUNCTION__);
    printk("**************************\n");

    #if defined(LGE_ACM_RX_DUMP_ENABLE)
    acm_rx_dump();
    #endif

    
    printk("### CP Crash : %d, USB DISCONNECT : %d ###\n",is_cp_crash, is_usb_disconnect);
    if(is_cp_crash == 1) {
        is_usb_disconnect = 0;
        is_cp_crash = 0;
    }
    if(is_usb_disconnect)
    {
        printk("!!!!! ts_ldisc_close called( ) by USB DISCONNECT !!!!!\n");
        is_usb_disconnect = 0;

        input_report_key(input_mux, EVENT_KEY, 1);
        input_report_key(input_mux, EVENT_KEY, 0);
        input_sync(input_mux);
        printk("######### input_report_key(): %d\n", EVENT_KEY);
       
        return 0;
    }
 
    #if 0 //defined(RECOVERY_CNT)

    #endif
#ifdef CONFIG_LGE_KERNEL_MUX
    if (task != NULL)
    {
        kthread_stop(task);
        printk("ts_ldisc_close READ_THREAD is stopped\n");
    }
    if (write_task != NULL)
    {
        time_to_stop = 1;
        up(&spi_write_sema); 
        kthread_stop(write_task);
        printk("ts_ldisc_close WRITE_THREAD is stopped\n");
    }
    


    ts0710_upon_disconnect();  
    
    tty_ldisc_flush(tty);
    mdelay(1000);


    #if defined(LGE_VT_CALL_SESSION) || defined(VT_MUX12_DEBUG)
    mux_12_open = 0;
    #endif 
   
#endif 
    
}
#else 


static void ts_ldisc_close(struct tty_struct *tty)
{
    ipc_tty = 0;


#ifdef CONFIG_LGE_KERNEL_MUX
    if (task != NULL)
    {
        kthread_stop(task);
        TS0710_DEBUG("ts_ldisc_close READ_THREAD is stopped\n");
    }
    if (write_task != NULL)
    {
        time_to_stop = 1;

        
        up(&spi_write_sema); 

        kthread_stop(write_task);
        TS0710_DEBUG("ts_ldisc_close WRITE_THREAD is stopped\n");
        
        write_task = NULL;
        LOCK(frame_nodes_lock);
        for (i = 0; i < TS0710MAX_PRIORITY_NUMBER;i++)
        {
            while (frame_to_send[i] != NULL)
            {
                if (frame_to_send[i]->data)
                {
                    kfree(frame_to_send[i]->data);
                    frame_to_send[i]->data = NULL;
                }
                frame_to_send[i]->size = 0;
                frame_to_send[i] = frame_to_send[i]->next;
            }
        } // end of for
        UNLOCK(frame_nodes_lock);
        TS0710_PRINTK("ts_ldisc_close write queue is freed\n");
        
    }
    


#if defined(LGE_VT_CALL_SESSION) || defined(VT_MUX12_DEBUG) 
    mux_12_open = 0;
#endif 
   

#endif
}
#endif 


static void ts_ldisc_wake(struct tty_struct *tty)
{
    printk("ts wake\n");
}

static ssize_t ts_ldisc_read(struct tty_struct *tty, struct file *file,
                             unsigned char __user *buf, size_t nr)
{
    return 0;
}

static ssize_t ts_ldisc_write(struct tty_struct *tty, struct file *file,
                              const unsigned char *data, size_t count)
{
    return 0;
}

static int ts_ldisc_ioctl(struct tty_struct *tty, struct file * file,
                          unsigned int cmd, unsigned long arg)
{
    return -1;
}

static unsigned int ts_ldisc_poll(struct tty_struct *tty,
                                  struct file *filp, poll_table *wait)
{
    return 0;
}


struct tty_operations mux_ops =
{
    .open   = mux_open,
    .close  = mux_close,
    .write  = mux_write,
    .write_room     = mux_write_room,
    .flush_buffer   = mux_flush_buffer,
    .chars_in_buffer= mux_chars_in_buffer,
    .throttle       = mux_throttle,
    .unthrottle     = mux_unthrottle,
};

static struct tty_ldisc_ops ts_ldisc =
{
    .owner	= THIS_MODULE,
    .magic	= TTY_LDISC_MAGIC,
    .name	= "ts07.10",
    .open	= ts_ldisc_open,
    .close	= ts_ldisc_close,
    .read	= ts_ldisc_read,
    .write	= ts_ldisc_write,
    .poll	= ts_ldisc_poll,
    .receive_buf	= ts_ldisc_rx,
    .write_wakeup	= ts_ldisc_wake,
    .ioctl	= ts_ldisc_ioctl,
};

#ifndef CONFIG_LGE_KERNEL_MUX
static u8 iscmdtty_gen1[16] =
    { 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0 };

static u8 iscmdtty_gen2[ 23 ] =
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
#endif

static int __init mux_init(void)
{
    u8 j;
    int result;
#ifdef CONFIG_LGE_KERNEL_MUX
    NR_MUXS = TS0710MAX_CHANNELS;
    iscmdtty = NULL;
    mux_send_info_idx = TS0710MAX_CHANNELS;
#else
    if (machine_is_ezx_a1200() || machine_is_ezx_e6())
    {
        NR_MUXS = 23;
        iscmdtty = iscmdtty_gen2;
        mux_send_info_idx = 23;
    }
    else
    {
        NR_MUXS = 16;
        iscmdtty = iscmdtty_gen1;
        mux_send_info_idx = 16;
    }
#endif
    TS0710_PRINTK("[e]initializing mux with %d channels\n", NR_MUXS);

    ts0710_init();

    for (j = 0; j < NR_MUXS; j++)
    {
        mux_send_info_flags[j] = 0;
        mux_send_info[j] = 0;
        mux_recv_info_flags[j] = 0;
        mux_recv_info[j] = 0;
    }
    mux_send_info_idx = NR_MUXS;
    mux_recv_queue = NULL;
    mux_recv_flags = 0;

    mux_driver = alloc_tty_driver(NR_MUXS);
    if (!mux_driver)
        return -ENOMEM;

    mux_driver->owner = THIS_MODULE;
    mux_driver->driver_name = "ts0710mux";
#ifdef CONFIG_LGE_KERNEL_MUX
    mux_driver->name = "pts";
#else
    mux_driver->name = "mux";
#endif
    mux_driver->major = TS0710MUX_MAJOR;
    mux_driver->minor_start = TS0710MUX_MINOR_START;
    mux_driver->type = TTY_DRIVER_TYPE_SERIAL;
    mux_driver->subtype = SERIAL_TYPE_NORMAL;
    mux_driver->flags = TTY_DRIVER_RESET_TERMIOS | TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;


    mux_driver->init_termios = tty_std_termios;
    mux_driver->init_termios.c_iflag = 0;
    mux_driver->init_termios.c_oflag = 0;
    mux_driver->init_termios.c_cflag = B38400 | CS8 | CREAD;
    mux_driver->init_termios.c_lflag = 0;

    /* mux_driver.ttys = mux_table; */
    mux_driver->termios = mux_termios;
    mux_driver->termios_locked = mux_termios_locked;
    /* mux_driver.driver_state = mux_state; */
    mux_driver->other = NULL;

    tty_set_operations(mux_driver, &mux_ops);

    
    result = tty_register_driver(mux_driver);
    
    if (result != 0 )
    {
       
        printk("TS0710:tty_register_driver returns %d", result);
        panic("TS0710 :Couldn't register mux driver");
    }

    for (j = 0; j < NR_MUXS; j++)
        tty_register_device(mux_driver, j, NULL);

    
    result = tty_register_ldisc(N_GSM0710, &ts_ldisc);
    
    if (result != 0)
    {
    
        printk("TS0710 :oops. cant register ldisc\n");
    }


    int err;

    input_mux = input_allocate_device();
    if (!input_mux) {
        err = -ENOMEM;
        printk("Failed to allocate input device ps\n");
        goto exit_unregister_mux;
    }

    set_bit(EV_KEY, input_mux->evbit);
    set_bit(EV_SYN, input_mux->evbit);
    set_bit(EVENT_KEY, input_mux->keybit);
    input_mux->name = "usb_disconnection";

    err = input_register_device(input_mux);
    if (err) {
        err = -ENOMEM;
        printk("Unable to register input device ps: %s\n",
        input_mux->name);
        goto exit_unregister_mux;
    }

    return 0;


exit_unregister_mux:
    input_unregister_device(input_mux);
exit:
    return err;

}

static void __exit mux_exit(void)
{
    u8 j;


    mux_send_info_idx = NR_MUXS;
    mux_recv_queue = NULL;
    for (j = 0; j < NR_MUXS; j++)
    {
        if ((mux_send_info_flags[j]) && (mux_send_info[j]))
        {
            kfree(mux_send_info[j]);
        }
        mux_send_info_flags[j] = 0;
        mux_send_info[j] = 0;

        if ((mux_recv_info_flags[j]) && (mux_recv_info[j]))
        {
            free_mux_recv_struct(mux_recv_info[j]);
        }
        mux_recv_info_flags[j] = 0;
        mux_recv_info[j] = 0;
    }

    for (j = 0; j < NR_MUXS; j++)
        tty_unregister_device(mux_driver, j);

    if (tty_unregister_driver(mux_driver))
        panic("Couldn't unregister mux driver");
}

bool IsVideoTelephonyActivated(void)
{
#if defined(LGE_VT_CALL_SESSION)
    return mux_12_open;
#else
    return 0;
#endif
}


module_init(mux_init);
module_exit(mux_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Harald Welte <laforge@openezx.org>");
MODULE_DESCRIPTION("TS 07.10 Multiplexer");
