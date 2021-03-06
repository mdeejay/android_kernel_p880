/*
 * Broadcom Dongle Host Driver (DHD), common DHD core.
 *
 * Copyright (C) 1999-2011, Broadcom Corporation
 * 
 *         Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 * 
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 * 
 *      Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Broadcom software provided under a license
 * other than the GPL, without Broadcom's express prior written consent.
 *
 * $Id: dhd_common.c,v 1.57.2.22 2011-02-01 18:38:37 Exp $
 */
#include <typedefs.h>
#include <osl.h>

#include <epivers.h>
#include <bcmutils.h>

#include <bcmendian.h>
#include <dngl_stats.h>
#include <wlioctl.h>
#include <dhd.h>

#include <proto/bcmevent.h>

#include <dhd_bus.h>
#include <dhd_proto.h>
#include <dhd_dbg.h>
#include <msgtrace.h>

#ifdef WL_CFG80211
#include <wl_cfg80211.h>
#endif
#include <proto/bt_amp_hci.h>
#include <dhd_bta.h>
#ifdef SET_RANDOM_MAC_SOFTAP
#include <linux/random.h>
#include <linux/jiffies.h>
#endif

#ifdef PROP_TXSTATUS
#include <wlfc_proto.h>
#include <dhd_wlfc.h>
#endif


#ifdef WLMEDIA_HTSF
extern void htsf_update(struct dhd_info *dhd, void *data);
#endif
int dhd_msg_level = DHD_ERROR_VAL;


#include <wl_iw.h>

char fw_path[MOD_PARAM_PATHLEN];
char nv_path[MOD_PARAM_PATHLEN];

#if defined(CONFIG_LGE_BCM432X_PATCH)
char config_path[MOD_PARAM_PATHLEN] = "";
#endif
#ifdef SOFTAP
char fw_path2[MOD_PARAM_PATHLEN];
extern bool softap_enabled;
#endif

uint32 dhd_conn_event;
uint32 dhd_conn_status;
uint32 dhd_conn_reason;

#define htod32(i) i
#define htod16(i) i
#define dtoh32(i) i
#define dtoh16(i) i
extern int dhd_iscan_request(void * dhdp, uint16 action);
extern void dhd_ind_scan_confirm(void *h, bool status);
extern int dhd_iscan_in_progress(void *h);
void dhd_iscan_lock(void);
void dhd_iscan_unlock(void);
extern int dhd_change_mtu(dhd_pub_t *dhd, int new_mtu, int ifidx);
bool ap_cfg_running = FALSE;
bool ap_fw_loaded = FALSE;

#if defined(KEEP_ALIVE)
int dhd_keep_alive_onoff(dhd_pub_t *dhd);
#endif 

#ifndef DHD_SDALIGN
#define DHD_SDALIGN	32
#endif
#if !ISPOWEROF2(DHD_SDALIGN)
#error DHD_SDALIGN is not a power of 2!
#endif

#ifdef DHD_DEBUG
const char dhd_version[] = "Dongle Host Driver, version " EPI_VERSION_STR "\nCompiled on "
	__DATE__ " at " __TIME__;
#else
const char dhd_version[] = "Dongle Host Driver, version " EPI_VERSION_STR;
#endif

void dhd_set_timer(void *bus, uint wdtick);



enum {
	IOV_VERSION = 1,
	IOV_MSGLEVEL,
	IOV_BCMERRORSTR,
	IOV_BCMERROR,
	IOV_WDTICK,
	IOV_DUMP,
	IOV_CLEARCOUNTS,
	IOV_LOGDUMP,
	IOV_LOGCAL,
	IOV_LOGSTAMP,
	IOV_GPIOOB,
	IOV_IOCTLTIMEOUT,
	IOV_HCI_CMD,		
	IOV_HCI_ACL_DATA,	
#if defined(DHD_DEBUG)
	IOV_CONS,
	IOV_DCONSOLE_POLL,
#endif 
#ifdef PROP_TXSTATUS
	IOV_PROPTXSTATUS_ENABLE,
	IOV_PROPTXSTATUS_MODE,
#endif
	IOV_BUS_TYPE,
#ifdef WLMEDIA_HTSF
	IOV_WLPKTDLYSTAT_SZ,
#endif
	IOV_CHANGEMTU,
	IOV_LAST
};

const bcm_iovar_t dhd_iovars[] = {
	{"version", 	IOV_VERSION,	0,	IOVT_BUFFER,	sizeof(dhd_version) },
#ifdef DHD_DEBUG
	{"msglevel",	IOV_MSGLEVEL,	0,	IOVT_UINT32,	0 },
#endif 
	{"bcmerrorstr", IOV_BCMERRORSTR, 0, IOVT_BUFFER,	BCME_STRLEN },
	{"bcmerror",	IOV_BCMERROR,	0,	IOVT_INT8,	0 },
	{"wdtick",	IOV_WDTICK, 0,	IOVT_UINT32,	0 },
	{"dump",	IOV_DUMP,	0,	IOVT_BUFFER,	DHD_IOCTL_MAXLEN },
#ifdef DHD_DEBUG
	{"cons",	IOV_CONS,	0,	IOVT_BUFFER,	0 },
	{"dconpoll",	IOV_DCONSOLE_POLL, 0,	IOVT_UINT32,	0 },
#endif
	{"clearcounts", IOV_CLEARCOUNTS, 0, IOVT_VOID,	0 },
	{"gpioob",	IOV_GPIOOB,	0,	IOVT_UINT32,	0 },
	{"ioctl_timeout",	IOV_IOCTLTIMEOUT,	0,	IOVT_UINT32,	0 },
	{"HCI_cmd",	IOV_HCI_CMD,	0,	IOVT_BUFFER,	0},
	{"HCI_ACL_data", IOV_HCI_ACL_DATA, 0,	IOVT_BUFFER,	0},
#ifdef PROP_TXSTATUS
	{"proptx",	IOV_PROPTXSTATUS_ENABLE,	0,	IOVT_UINT32,	0 },
	
	{"ptxmode",	IOV_PROPTXSTATUS_MODE,	0,	IOVT_UINT32,	0 },
#endif
	{"bustype", IOV_BUS_TYPE, 0, IOVT_UINT32, 0},
#ifdef WLMEDIA_HTSF
	{"pktdlystatsz", IOV_WLPKTDLYSTAT_SZ, 0, IOVT_UINT8, 0 },
#endif
	{"changemtu", IOV_CHANGEMTU, 0, IOVT_UINT32, 0 },
	{NULL, 0, 0, 0, 0 }
};

struct dhd_cmn *
dhd_common_init(osl_t *osh)
{
	dhd_cmn_t *cmn;

	if (!(cmn = MALLOC(osh, sizeof(dhd_cmn_t)))) {
		DHD_ERROR(("%s: MALLOC failed\n", __FUNCTION__));
		return NULL;
	}
	memset(cmn, 0, sizeof(dhd_cmn_t));
	cmn->osh = osh;

#ifdef CONFIG_BCMDHD_FW_PATH
	bcm_strncpy_s(fw_path, sizeof(fw_path), CONFIG_BCMDHD_FW_PATH, MOD_PARAM_PATHLEN-1);
#else 
	fw_path[0] = '\0';
#endif 
#ifdef CONFIG_BCMDHD_NVRAM_PATH
	bcm_strncpy_s(nv_path, sizeof(nv_path), CONFIG_BCMDHD_NVRAM_PATH, MOD_PARAM_PATHLEN-1);
#else 
	nv_path[0] = '\0';
#endif 
#ifdef SOFTAP
	fw_path2[0] = '\0';
#endif
	return cmn;
}

void
dhd_common_deinit(dhd_pub_t *dhd_pub, dhd_cmn_t *sa_cmn)
{
	osl_t *osh;
	dhd_cmn_t *cmn;

	if (dhd_pub != NULL)
		cmn = dhd_pub->cmn;
	else
		cmn = sa_cmn;

	if (!cmn)
		return;

	osh = cmn->osh;

	if (dhd_pub != NULL)
	dhd_pub->cmn = NULL;
	MFREE(osh, cmn, sizeof(dhd_cmn_t));
}

static int
dhd_dump(dhd_pub_t *dhdp, char *buf, int buflen)
{
	char eabuf[ETHER_ADDR_STR_LEN];

	struct bcmstrbuf b;
	struct bcmstrbuf *strbuf = &b;

	bcm_binit(strbuf, buf, buflen);

	bcm_bprintf(strbuf, "%s\n", dhd_version);
	bcm_bprintf(strbuf, "\n");
	bcm_bprintf(strbuf, "pub.up %d pub.txoff %d pub.busstate %d\n",
	            dhdp->up, dhdp->txoff, dhdp->busstate);
	bcm_bprintf(strbuf, "pub.hdrlen %d pub.maxctl %d pub.rxsz %d\n",
	            dhdp->hdrlen, dhdp->maxctl, dhdp->rxsz);
	bcm_bprintf(strbuf, "pub.iswl %d pub.drv_version %ld pub.mac %s\n",
	            dhdp->iswl, dhdp->drv_version, bcm_ether_ntoa(&dhdp->mac, eabuf));
	bcm_bprintf(strbuf, "pub.bcmerror %d tickcnt %d\n", dhdp->bcmerror, dhdp->tickcnt);

	bcm_bprintf(strbuf, "dongle stats:\n");
	bcm_bprintf(strbuf, "tx_packets %ld tx_bytes %ld tx_errors %ld tx_dropped %ld\n",
	            dhdp->dstats.tx_packets, dhdp->dstats.tx_bytes,
	            dhdp->dstats.tx_errors, dhdp->dstats.tx_dropped);
	bcm_bprintf(strbuf, "rx_packets %ld rx_bytes %ld rx_errors %ld rx_dropped %ld\n",
	            dhdp->dstats.rx_packets, dhdp->dstats.rx_bytes,
	            dhdp->dstats.rx_errors, dhdp->dstats.rx_dropped);
	bcm_bprintf(strbuf, "multicast %ld\n", dhdp->dstats.multicast);

	bcm_bprintf(strbuf, "bus stats:\n");
	bcm_bprintf(strbuf, "tx_packets %ld tx_multicast %ld tx_errors %ld\n",
	            dhdp->tx_packets, dhdp->tx_multicast, dhdp->tx_errors);
	bcm_bprintf(strbuf, "tx_ctlpkts %ld tx_ctlerrs %ld\n",
	            dhdp->tx_ctlpkts, dhdp->tx_ctlerrs);
	bcm_bprintf(strbuf, "rx_packets %ld rx_multicast %ld rx_errors %ld \n",
	            dhdp->rx_packets, dhdp->rx_multicast, dhdp->rx_errors);
	bcm_bprintf(strbuf, "rx_ctlpkts %ld rx_ctlerrs %ld rx_dropped %ld\n",
	            dhdp->rx_ctlpkts, dhdp->rx_ctlerrs, dhdp->rx_dropped);
	bcm_bprintf(strbuf, "rx_readahead_cnt %ld tx_realloc %ld\n",
	            dhdp->rx_readahead_cnt, dhdp->tx_realloc);
	bcm_bprintf(strbuf, "\n");

	dhd_prot_dump(dhdp, strbuf);
	bcm_bprintf(strbuf, "\n");

	dhd_bus_dump(dhdp, strbuf);

	return (!strbuf->size ? BCME_BUFTOOSHORT : 0);
}

int
dhd_wl_ioctl_cmd(dhd_pub_t *dhd_pub, int cmd, void *arg, int len, uint8 set, int ifindex)
{
	wl_ioctl_t ioc;

	ioc.cmd = cmd;
	ioc.buf = arg;
	ioc.len = len;
	ioc.set = set;

	return dhd_wl_ioctl(dhd_pub, ifindex, &ioc, arg, len);
}


int
dhd_wl_ioctl(dhd_pub_t *dhd_pub, int ifindex, wl_ioctl_t *ioc, void *buf, int len)
{
	int ret;

	dhd_os_proto_block(dhd_pub);

	ret = dhd_prot_ioctl(dhd_pub, ifindex, ioc, buf, len);


	dhd_os_proto_unblock(dhd_pub);
	return ret;
}

static int
dhd_doiovar(dhd_pub_t *dhd_pub, const bcm_iovar_t *vi, uint32 actionid, const char *name,
            void *params, int plen, void *arg, int len, int val_size)
{
	int bcmerror = 0;
	int32 int_val = 0;

	DHD_TRACE(("%s: Enter\n", __FUNCTION__));
	DHD_TRACE(("%s: actionid = %d; name %s\n", __FUNCTION__, actionid, name));

	if ((bcmerror = bcm_iovar_lencheck(vi, arg, len, IOV_ISSET(actionid))) != 0)
		goto exit;

	if (plen >= (int)sizeof(int_val))
		bcopy(params, &int_val, sizeof(int_val));

	switch (actionid) {
	case IOV_GVAL(IOV_VERSION):
		bcm_strncpy_s((char*)arg, len, dhd_version, len);
		break;

	case IOV_GVAL(IOV_MSGLEVEL):
		int_val = (int32)dhd_msg_level;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_SVAL(IOV_MSGLEVEL):
		dhd_msg_level = int_val;
		break;
	case IOV_GVAL(IOV_BCMERRORSTR):
		bcm_strncpy_s((char *)arg, len, bcmerrorstr(dhd_pub->bcmerror), BCME_STRLEN);
		((char *)arg)[BCME_STRLEN - 1] = 0x00;
		break;

	case IOV_GVAL(IOV_BCMERROR):
		int_val = (int32)dhd_pub->bcmerror;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_GVAL(IOV_WDTICK):
		int_val = (int32)dhd_watchdog_ms;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_SVAL(IOV_WDTICK):
		if (!dhd_pub->up) {
			bcmerror = BCME_NOTUP;
			break;
		}
		dhd_os_wd_timer(dhd_pub, (uint)int_val);
		break;

	case IOV_GVAL(IOV_DUMP):
		bcmerror = dhd_dump(dhd_pub, arg, len);
		break;

#ifdef DHD_DEBUG
	case IOV_GVAL(IOV_DCONSOLE_POLL):
		int_val = (int32)dhd_console_ms;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_SVAL(IOV_DCONSOLE_POLL):
		dhd_console_ms = (uint)int_val;
		break;

	case IOV_SVAL(IOV_CONS):
		if (len > 0)
			bcmerror = dhd_bus_console_in(dhd_pub, arg, len - 1);
		break;
#endif 
	case IOV_SVAL(IOV_CLEARCOUNTS):
		dhd_pub->tx_packets = dhd_pub->rx_packets = 0;
		dhd_pub->tx_errors = dhd_pub->rx_errors = 0;
		dhd_pub->tx_ctlpkts = dhd_pub->rx_ctlpkts = 0;
		dhd_pub->tx_ctlerrs = dhd_pub->rx_ctlerrs = 0;
		dhd_pub->rx_dropped = 0;
		dhd_pub->rx_readahead_cnt = 0;
		dhd_pub->tx_realloc = 0;
		dhd_pub->wd_dpc_sched = 0;
		memset(&dhd_pub->dstats, 0, sizeof(dhd_pub->dstats));
		dhd_bus_clearcounts(dhd_pub);
#ifdef PROP_TXSTATUS
		if (dhd_pub->wlfc_state) {
			athost_wl_status_info_t *wlfc =
			        (athost_wl_status_info_t*)dhd_pub->wlfc_state;
			wlfc_hanger_t* hanger;

			memset(&wlfc->stats, 0, sizeof(athost_wl_stat_counters_t));

			hanger = (wlfc_hanger_t*)wlfc->hanger;
			hanger->pushed = 0;
			hanger->popped = 0;
			hanger->failed_slotfind = 0;
			hanger->failed_to_pop = 0;
			hanger->failed_to_push = 0;
		}
#endif
		break;


	case IOV_GVAL(IOV_IOCTLTIMEOUT): {
		int_val = (int32)dhd_os_get_ioctl_resp_timeout();
		bcopy(&int_val, arg, sizeof(int_val));
		break;
	}

	case IOV_SVAL(IOV_IOCTLTIMEOUT): {
		if (int_val <= 0)
			bcmerror = BCME_BADARG;
		else
			dhd_os_set_ioctl_resp_timeout((unsigned int)int_val);
		break;
	}

	case IOV_SVAL(IOV_HCI_CMD): {
		amp_hci_cmd_t *cmd = (amp_hci_cmd_t *)arg;

		if (len < HCI_CMD_PREAMBLE_SIZE)
			return BCME_BUFTOOSHORT;

		if (len < (int)(HCI_CMD_PREAMBLE_SIZE + cmd->plen))
			return BCME_BUFTOOSHORT;

		dhd_bta_docmd(dhd_pub, cmd, len);
		break;
	}

	case IOV_SVAL(IOV_HCI_ACL_DATA): {
		amp_hci_ACL_data_t *ACL_data = (amp_hci_ACL_data_t *)arg;

		if (len < HCI_ACL_DATA_PREAMBLE_SIZE)
			return BCME_BUFTOOSHORT;

		if (len < (int)(HCI_ACL_DATA_PREAMBLE_SIZE + ACL_data->dlen))
			return BCME_BUFTOOSHORT;

		dhd_bta_tx_hcidata(dhd_pub, ACL_data, len);
		break;
	}

#ifdef PROP_TXSTATUS
	case IOV_GVAL(IOV_PROPTXSTATUS_ENABLE):
		int_val = dhd_pub->wlfc_enabled? 1 : 0;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_SVAL(IOV_PROPTXSTATUS_ENABLE):
		dhd_pub->wlfc_enabled = int_val? 1 : 0;
		break;

	case IOV_GVAL(IOV_PROPTXSTATUS_MODE): {
		athost_wl_status_info_t *wlfc =
		        (athost_wl_status_info_t*)dhd_pub->wlfc_state;
		int_val = dhd_pub->wlfc_state ? (int32)wlfc->proptxstatus_mode : 0;
		bcopy(&int_val, arg, val_size);
		break;
	}

	case IOV_SVAL(IOV_PROPTXSTATUS_MODE):
		if (dhd_pub->wlfc_state) {
			athost_wl_status_info_t *wlfc =
			        (athost_wl_status_info_t*)dhd_pub->wlfc_state;
			wlfc->proptxstatus_mode = int_val & 0xff;
		}
		break;
#endif 

	case IOV_GVAL(IOV_BUS_TYPE):
#ifdef BCMDHDUSB
		int_val = BUS_TYPE_USB;
#endif
		int_val = BUS_TYPE_SDIO;
		bcopy(&int_val, arg, val_size);
		break;


#ifdef WLMEDIA_HTSF
	case IOV_GVAL(IOV_WLPKTDLYSTAT_SZ):
		int_val = dhd_pub->htsfdlystat_sz;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_SVAL(IOV_WLPKTDLYSTAT_SZ):
		dhd_pub->htsfdlystat_sz = int_val & 0xff;
		printf("Setting tsfdlystat_sz:%d\n", dhd_pub->htsfdlystat_sz);
		break;
#endif
	case IOV_SVAL(IOV_CHANGEMTU):
		int_val &= 0xffff;
		bcmerror = dhd_change_mtu(dhd_pub, int_val, 0);
		break;

	default:
		bcmerror = BCME_UNSUPPORTED;
		break;
	}

exit:
	DHD_TRACE(("%s: actionid %d, bcmerror %d\n", __FUNCTION__, actionid, bcmerror));
	return bcmerror;
}

void
dhd_store_conn_status(uint32 event, uint32 status, uint32 reason)
{
	
	if (!(event == WLC_E_SET_SSID && status == WLC_E_STATUS_FAIL &&
	      dhd_conn_event == WLC_E_PRUNE)) {
		dhd_conn_event = event;
		dhd_conn_status = status;
		dhd_conn_reason = reason;
	}
}

bool
dhd_prec_enq(dhd_pub_t *dhdp, struct pktq *q, void *pkt, int prec)
{
	void *p;
	int eprec = -1;		
	bool discard_oldest;


	if (!pktq_pfull(q, prec) && !pktq_full(q)) {
		pktq_penq(q, prec, pkt);
		return TRUE;
	}
	
	if (pktq_pfull(q, prec))
	eprec = prec;
	else if (pktq_full(q)) {
		p = pktq_peek_tail(q, &eprec);
		ASSERT(p);
		if (eprec > prec || eprec < 0)
			return FALSE;
	}

	if (eprec >= 0) {
		ASSERT(!pktq_pempty(q, eprec));
		discard_oldest = AC_BITMAP_TST(dhdp->wme_dp, eprec);
		if (eprec == prec && !discard_oldest)
			return FALSE;		
		p = discard_oldest ? pktq_pdeq(q, eprec) : pktq_pdeq_tail(q, eprec);
		ASSERT(p);

		PKTFREE(dhdp->osh, p, TRUE);
	}

	p = pktq_penq(q, prec, pkt);
	ASSERT(p);

	return TRUE;
}

static int
dhd_iovar_op(dhd_pub_t *dhd_pub, const char *name,
	void *params, int plen, void *arg, int len, bool set)
{
	int bcmerror = 0;
	int val_size;
	const bcm_iovar_t *vi = NULL;
	uint32 actionid;

	DHD_TRACE(("%s: Enter\n", __FUNCTION__));

	ASSERT(name);
	ASSERT(len >= 0);

	ASSERT(set || (arg && len));

	ASSERT(!set || (!params && !plen));

	if ((vi = bcm_iovar_lookup(dhd_iovars, name)) == NULL) {
		bcmerror = BCME_UNSUPPORTED;
		goto exit;
	}

	DHD_CTL(("%s: %s %s, len %d plen %d\n", __FUNCTION__,
		name, (set ? "set" : "get"), len, plen));

	if (params == NULL) {
		params = arg;
		plen = len;
	}

	if (vi->type == IOVT_VOID)
		val_size = 0;
	else if (vi->type == IOVT_BUFFER)
		val_size = len;
	else
		val_size = sizeof(int);

	actionid = set ? IOV_SVAL(vi->varid) : IOV_GVAL(vi->varid);

	bcmerror = dhd_doiovar(dhd_pub, vi, actionid, name, params, plen, arg, len, val_size);

exit:
	return bcmerror;
}

int
dhd_ioctl(dhd_pub_t * dhd_pub, dhd_ioctl_t *ioc, void * buf, uint buflen)
{
	int bcmerror = 0;

	DHD_TRACE(("%s: Enter\n", __FUNCTION__));

	if (!buf) {
		return BCME_BADARG;
	}

	switch (ioc->cmd) {
	case DHD_GET_MAGIC:
		if (buflen < sizeof(int))
			bcmerror = BCME_BUFTOOSHORT;
		else
			*(int*)buf = DHD_IOCTL_MAGIC;
		break;

	case DHD_GET_VERSION:
		if (buflen < sizeof(int))
			bcmerror = -BCME_BUFTOOSHORT;
		else
			*(int*)buf = DHD_IOCTL_VERSION;
		break;

	case DHD_GET_VAR:
	case DHD_SET_VAR: {
		char *arg;
		uint arglen;

		for (arg = buf, arglen = buflen; *arg && arglen; arg++, arglen--)
			;

		if (*arg) {
			bcmerror = BCME_BUFTOOSHORT;
			break;
		}

		arg++, arglen--;

		if (ioc->cmd == DHD_GET_VAR)
			bcmerror = dhd_iovar_op(dhd_pub, buf, arg, arglen,
			buf, buflen, IOV_GET);
		else
			bcmerror = dhd_iovar_op(dhd_pub, buf, NULL, 0, arg, arglen, IOV_SET);
		if (bcmerror != BCME_UNSUPPORTED)
			break;

		if (ioc->cmd == DHD_GET_VAR)
			bcmerror = dhd_prot_iovar_op(dhd_pub, buf, arg,
				arglen, buf, buflen, IOV_GET);
		else
			bcmerror = dhd_prot_iovar_op(dhd_pub, buf,
				NULL, 0, arg, arglen, IOV_SET);
		if (bcmerror != BCME_UNSUPPORTED)
			break;

		if (ioc->cmd == DHD_GET_VAR) {
			bcmerror = dhd_bus_iovar_op(dhd_pub, buf,
				arg, arglen, buf, buflen, IOV_GET);
		} else {
			bcmerror = dhd_bus_iovar_op(dhd_pub, buf,
				NULL, 0, arg, arglen, IOV_SET);
		}

		break;
	}

	default:
		bcmerror = BCME_UNSUPPORTED;
	}

	return bcmerror;
}

#ifdef SHOW_EVENTS
static void
wl_show_host_event(wl_event_msg_t *event, void *event_data)
{
	uint i, status, reason;
	bool group = FALSE, flush_txq = FALSE, link = FALSE;
	const char *auth_str;
	const char *event_name;
	uchar *buf;
	char err_msg[256], eabuf[ETHER_ADDR_STR_LEN];
	uint event_type, flags, auth_type, datalen;

	event_type = ntoh32(event->event_type);
	flags = ntoh16(event->flags);
	status = ntoh32(event->status);
	reason = ntoh32(event->reason);
	auth_type = ntoh32(event->auth_type);
	datalen = ntoh32(event->datalen);

	sprintf(eabuf, "%02x:%02x:%02x:%02x:%02x:%02x",
	        (uchar)event->addr.octet[0]&0xff,
	        (uchar)event->addr.octet[1]&0xff,
	        (uchar)event->addr.octet[2]&0xff,
	        (uchar)event->addr.octet[3]&0xff,
	        (uchar)event->addr.octet[4]&0xff,
	        (uchar)event->addr.octet[5]&0xff);

	event_name = "UNKNOWN";
	for (i = 0; i < (uint)bcmevent_names_size; i++)
		if (bcmevent_names[i].event == event_type)
			event_name = bcmevent_names[i].name;

	if (flags & WLC_EVENT_MSG_LINK)
		link = TRUE;
	if (flags & WLC_EVENT_MSG_GROUP)
		group = TRUE;
	if (flags & WLC_EVENT_MSG_FLUSHTXQ)
		flush_txq = TRUE;

	switch (event_type) {
	case WLC_E_START:
	case WLC_E_DEAUTH:
	case WLC_E_DISASSOC:
		DHD_EVENT(("MACEVENT: %s, MAC %s\n", event_name, eabuf));
		break;

	case WLC_E_ASSOC_IND:
	case WLC_E_REASSOC_IND:

		DHD_EVENT(("MACEVENT: %s, MAC %s\n", event_name, eabuf));
		break;

	case WLC_E_ASSOC:
	case WLC_E_REASSOC:
		if (status == WLC_E_STATUS_SUCCESS) {
			DHD_EVENT(("MACEVENT: %s, MAC %s, SUCCESS\n", event_name, eabuf));
		} else if (status == WLC_E_STATUS_TIMEOUT) {
			DHD_EVENT(("MACEVENT: %s, MAC %s, TIMEOUT\n", event_name, eabuf));
		} else if (status == WLC_E_STATUS_FAIL) {
			DHD_EVENT(("MACEVENT: %s, MAC %s, FAILURE, reason %d\n",
			       event_name, eabuf, (int)reason));
		} else {
			DHD_EVENT(("MACEVENT: %s, MAC %s, unexpected status %d\n",
			       event_name, eabuf, (int)status));
		}
		break;

	case WLC_E_DEAUTH_IND:
	case WLC_E_DISASSOC_IND:
		DHD_EVENT(("MACEVENT: %s, MAC %s, reason %d\n", event_name, eabuf, (int)reason));
		break;

	case WLC_E_AUTH:
	case WLC_E_AUTH_IND:
		if (auth_type == DOT11_OPEN_SYSTEM)
			auth_str = "Open System";
		else if (auth_type == DOT11_SHARED_KEY)
			auth_str = "Shared Key";
		else {
			sprintf(err_msg, "AUTH unknown: %d", (int)auth_type);
			auth_str = err_msg;
		}
		if (event_type == WLC_E_AUTH_IND) {
			DHD_EVENT(("MACEVENT: %s, MAC %s, %s\n", event_name, eabuf, auth_str));
		} else if (status == WLC_E_STATUS_SUCCESS) {
			DHD_EVENT(("MACEVENT: %s, MAC %s, %s, SUCCESS\n",
				event_name, eabuf, auth_str));
		} else if (status == WLC_E_STATUS_TIMEOUT) {
			DHD_EVENT(("MACEVENT: %s, MAC %s, %s, TIMEOUT\n",
				event_name, eabuf, auth_str));
		} else if (status == WLC_E_STATUS_FAIL) {
			DHD_EVENT(("MACEVENT: %s, MAC %s, %s, FAILURE, reason %d\n",
			       event_name, eabuf, auth_str, (int)reason));
		}

		break;

	case WLC_E_JOIN:
	case WLC_E_ROAM:
	case WLC_E_SET_SSID:
		if (status == WLC_E_STATUS_SUCCESS) {
			DHD_EVENT(("MACEVENT: %s, MAC %s\n", event_name, eabuf));
		} else if (status == WLC_E_STATUS_FAIL) {
			DHD_EVENT(("MACEVENT: %s, failed\n", event_name));
		} else if (status == WLC_E_STATUS_NO_NETWORKS) {
			DHD_EVENT(("MACEVENT: %s, no networks found\n", event_name));
		} else {
			DHD_EVENT(("MACEVENT: %s, unexpected status %d\n",
				event_name, (int)status));
		}
		break;

	case WLC_E_BEACON_RX:
		if (status == WLC_E_STATUS_SUCCESS) {
			DHD_EVENT(("MACEVENT: %s, SUCCESS\n", event_name));
		} else if (status == WLC_E_STATUS_FAIL) {
			DHD_EVENT(("MACEVENT: %s, FAIL\n", event_name));
		} else {
			DHD_EVENT(("MACEVENT: %s, status %d\n", event_name, status));
		}
		break;

	case WLC_E_LINK:
		DHD_EVENT(("MACEVENT: %s %s\n", event_name, link?"UP":"DOWN"));
		break;

	case WLC_E_MIC_ERROR:
		DHD_EVENT(("MACEVENT: %s, MAC %s, Group %d, Flush %d\n",
		       event_name, eabuf, group, flush_txq));
		break;

	case WLC_E_ICV_ERROR:
	case WLC_E_UNICAST_DECODE_ERROR:
	case WLC_E_MULTICAST_DECODE_ERROR:
		DHD_EVENT(("MACEVENT: %s, MAC %s\n",
		       event_name, eabuf));
		break;

	case WLC_E_TXFAIL:
		DHD_EVENT(("MACEVENT: %s, RA %s\n", event_name, eabuf));
		break;

	case WLC_E_SCAN_COMPLETE:
	case WLC_E_PMKID_CACHE:
		DHD_EVENT(("MACEVENT: %s\n", event_name));
		break;

	case WLC_E_PFN_NET_FOUND:
	case WLC_E_PFN_NET_LOST:
	case WLC_E_PFN_SCAN_COMPLETE:
	case WLC_E_PFN_SCAN_NONE:
	case WLC_E_PFN_SCAN_ALLGONE:
		DHD_EVENT(("PNOEVENT: %s\n", event_name));
		break;

	case WLC_E_PSK_SUP:
	case WLC_E_PRUNE:
		DHD_EVENT(("MACEVENT: %s, status %d, reason %d\n",
		           event_name, (int)status, (int)reason));
		break;

#ifdef WIFI_ACT_FRAME
	case WLC_E_ACTION_FRAME:
		DHD_TRACE(("MACEVENT: %s Bssid %s\n", event_name, eabuf));
		break;
#endif 
	case WLC_E_TRACE: {
		static uint32 seqnum_prev = 0;
		msgtrace_hdr_t hdr;
		uint32 nblost;
		char *s, *p;

		buf = (uchar *) event_data;
		memcpy(&hdr, buf, MSGTRACE_HDRLEN);

		if (hdr.version != MSGTRACE_VERSION) {
			printf("\nMACEVENT: %s [unsupported version --> "
			       "dhd version:%d dongle version:%d]\n",
			       event_name, MSGTRACE_VERSION, hdr.version);
			datalen = 0;
			break;
		}

		buf[MSGTRACE_HDRLEN + ntoh16(hdr.len)] = '\0';

		if (ntoh32(hdr.discarded_bytes) || ntoh32(hdr.discarded_printf)) {
			printf("\nWLC_E_TRACE: [Discarded traces in dongle -->"
			       "discarded_bytes %d discarded_printf %d]\n",
			       ntoh32(hdr.discarded_bytes), ntoh32(hdr.discarded_printf));
		}

		nblost = ntoh32(hdr.seqnum) - seqnum_prev - 1;
		if (nblost > 0) {
			printf("\nWLC_E_TRACE: [Event lost --> seqnum %d nblost %d\n",
			       ntoh32(hdr.seqnum), nblost);
		}
		seqnum_prev = ntoh32(hdr.seqnum);

		
		p = (char *)&buf[MSGTRACE_HDRLEN];
		while ((s = strstr(p, "\n")) != NULL) {
			*s = '\0';
			printf("%s\n", p);
			p = s+1;
		}
		printf("%s\n", p);

		datalen = 0;
		break;
	}


	case WLC_E_RSSI:
		DHD_EVENT(("MACEVENT: %s %d\n", event_name, ntoh32(*((int *)event_data))));
		break;

	default:
		DHD_EVENT(("MACEVENT: %s %d, MAC %s, status %d, reason %d, auth %d\n",
		       event_name, event_type, eabuf, (int)status, (int)reason,
		       (int)auth_type));
		break;
	}

	if (datalen) {
		buf = (uchar *) event_data;
		DHD_EVENT((" data (%d) : ", datalen));
		for (i = 0; i < datalen; i++)
			DHD_EVENT((" 0x%02x ", *buf++));
		DHD_EVENT(("\n"));
	}
}
#endif 
#if defined(WLP2P) 
extern int dhd_use_p2p;
#define P2P_INTERFACE_NAME "p2p"
#endif 

int
wl_host_event(dhd_pub_t *dhd_pub, int *ifidx, void *pktdata,
              wl_event_msg_t *event, void **data_ptr)
{
	bcm_event_t *pvt_data = (bcm_event_t *)pktdata;
	char *event_data;
	uint32 type, status, reason, datalen;
	uint16 flags;
	int evlen;

	if (bcmp(BRCM_OUI, &pvt_data->bcm_hdr.oui[0], DOT11_OUI_LEN)) {
		DHD_ERROR(("%s: mismatched OUI, bailing\n", __FUNCTION__));
		return (BCME_ERROR);
	}

	if (ntoh16_ua((void *)&pvt_data->bcm_hdr.usr_subtype) != BCMILCP_BCM_SUBTYPE_EVENT) {
		DHD_ERROR(("%s: mismatched subtype, bailing\n", __FUNCTION__));
		return (BCME_ERROR);
	}

	*data_ptr = &pvt_data[1];
	event_data = *data_ptr;

	memcpy(event, &pvt_data->event, sizeof(wl_event_msg_t));

	type = ntoh32_ua((void *)&event->event_type);
	flags = ntoh16_ua((void *)&event->flags);
	status = ntoh32_ua((void *)&event->status);
	reason = ntoh32_ua((void *)&event->reason);
	datalen = ntoh32_ua((void *)&event->datalen);
	evlen = datalen + sizeof(bcm_event_t);

	switch (type) {
#ifdef PROP_TXSTATUS
	case WLC_E_FIFO_CREDIT_MAP:
		dhd_wlfc_event(dhd_pub->info);
		dhd_wlfc_FIFOcreditmap_event(dhd_pub->info, event_data);
		WLFC_DBGMESG(("WLC_E_FIFO_CREDIT_MAP:(AC0,AC1,AC2,AC3),(BC_MC),(OTHER): "
			"(%d,%d,%d,%d),(%d),(%d)\n", event_data[0], event_data[1],
			event_data[2],
			event_data[3], event_data[4], event_data[5]));
		break;
#endif

	case WLC_E_IF:
		{
		dhd_if_event_t *ifevent = (dhd_if_event_t *)event_data;
#ifdef PROP_TXSTATUS
{
		uint8* ea = pvt_data->eth.ether_dhost;
		WLFC_DBGMESG(("WLC_E_IF: idx:%d, action:%s, iftype:%s, "
		              "[%02x:%02x:%02x:%02x:%02x:%02x]\n",
		              ifevent->ifidx,
		              ((ifevent->action == WLC_E_IF_ADD) ? "ADD":"DEL"),
		              ((ifevent->is_AP == 0) ? "STA":"AP "),
		              ea[0], ea[1], ea[2], ea[3], ea[4], ea[5]));
		(void)ea;

		dhd_wlfc_interface_event(dhd_pub->info,
		                         ((ifevent->action == WLC_E_IF_ADD) ?
		                          eWLFC_MAC_ENTRY_ACTION_ADD : eWLFC_MAC_ENTRY_ACTION_DEL),
		                         ifevent->ifidx, ifevent->is_AP, ea);

		if (ifevent->ifidx == 0)
			break;
}
#endif 

#ifdef WL_CFG80211
			if (wl_cfg80211_is_progress_ifchange()) {
				DHD_ERROR(("%s:  ifidx %d for %s action %d\n",
					__FUNCTION__, ifevent->ifidx,
					event->ifname, ifevent->action));
			if (ifevent->action == WLC_E_IF_ADD)
					wl_cfg80211_notify_ifchange();
				return (BCME_OK);
			}
#endif 
#if defined(WLP2P) 
		if (dhd_use_p2p) {
			if (strncmp(pvt_data->event.ifname,"wl",2) == 0) {
			memmove(&(pvt_data->event.ifname[1]),&(pvt_data->event.ifname[0]),6);
			memcpy(pvt_data->event.ifname,P2P_INTERFACE_NAME,strlen(P2P_INTERFACE_NAME));
			memcpy(event, &pvt_data->event, sizeof(wl_event_msg_t));
			}
		}
#endif 
				if (ifevent->ifidx > 0 && ifevent->ifidx < DHD_MAX_IFS) {
					if (ifevent->action == WLC_E_IF_ADD) {
						if (dhd_add_if(dhd_pub->info, ifevent->ifidx,
							NULL, event->ifname,
							event->addr.octet,
							ifevent->flags, ifevent->bssidx)) {
							DHD_ERROR(("%s: dhd_add_if failed!!"
									" ifidx: %d for %s\n",
									__FUNCTION__,
									ifevent->ifidx,
									event->ifname));
							return (BCME_ERROR);
						}
					}
			else
				dhd_del_if(dhd_pub->info, ifevent->ifidx);
		} else {
#ifndef PROP_TXSTATUS
			DHD_ERROR(("%s: Invalid ifidx %d for %s\n",
			           __FUNCTION__, ifevent->ifidx, event->ifname));
#endif
		}
			}
			*ifidx = dhd_ifname2idx(dhd_pub->info, event->ifname);
			dhd_event(dhd_pub->info, (char *)pvt_data, evlen, *ifidx);
			break;


#ifdef WLMEDIA_HTSF
	case WLC_E_HTSFSYNC:
		htsf_update(dhd_pub->info, event_data);
		break;
#endif 
	case WLC_E_NDIS_LINK: {
		uint32 temp = hton32(WLC_E_LINK);

		memcpy((void *)(&pvt_data->event.event_type), &temp,
		       sizeof(pvt_data->event.event_type));
	}
#if defined(BCMCCX) && defined(BCMDBG_EVENT) 
		case WLC_E_PRUNE:
			{
#define WLC_E_PRUNE_CCXFAST_PREVAP	11	
#define WLC_E_PRUNE_CCXFAST_DROAM	14	
#define WLC_E_PRUNE_QBSS_LOAD		16	
#define WLC_E_PRUNE_AP_BLOCKED		18	
#define WLC_E_PRUNE_NO_DIAG_SUPPORT	19	
				uint reason;
				char eabuf[ETHER_ADDR_STR_LEN];
				reason = ntoh32_ua((void *)&event->reason);
				sprintf(eabuf, "%02x:%02x:%02x:%02x:%02x:%02x",
				        (uchar)event->addr.octet[0]&0xff,
				        (uchar)event->addr.octet[1]&0xff,
				        (uchar)event->addr.octet[2]&0xff,
				        (uchar)event->addr.octet[3]&0xff,
				        (uchar)event->addr.octet[4]&0xff,
				        (uchar)event->addr.octet[5]&0xff);

				switch(reason) {
					case WLC_E_PRUNE_CCXFAST_PREVAP:
						DHD_ERROR(("PRUNE %s: PRUNE: WLC_E_PRUNE_CCXFAST_PREVAP\n", eabuf));
						break;
					case WLC_E_PRUNE_CCXFAST_DROAM:
						DHD_ERROR(("PRUNE %s: PRUNE: WLC_E_PRUNE_CCXFAST_DROAM\n", eabuf));
						break;
					case WLC_E_PRUNE_QBSS_LOAD:
						DHD_ERROR(("PRUNE %s: PRUNE: WLC_E_PRUNE_QBSS_LOAD\n", eabuf));
						break;
					case WLC_E_PRUNE_AP_BLOCKED:
						DHD_ERROR(("PRUNE %s: PRUNE: WLC_E_PRUNE_AP_BLOCKED\n", eabuf));
						break;
					case WLC_E_PRUNE_NO_DIAG_SUPPORT:
						DHD_ERROR(("PRUNE %s: PRUNE: WLC_E_PRUNE_NO_DIAG_SUPPORT\n", eabuf));
						break;
					default:
						DHD_ERROR(("PRUNE %s: status %d, reason %d\n",
						           eabuf, status, reason));
						break;
				}
				break;
			}
		case WLC_E_ADDTS_IND:
			{
				uint reason;
				char eabuf[ETHER_ADDR_STR_LEN];
				reason = ntoh32_ua((void *)&event->reason);
				sprintf(eabuf, "%02x:%02x:%02x:%02x:%02x:%02x",
				        (uchar)event->addr.octet[0]&0xff,
				        (uchar)event->addr.octet[1]&0xff,
				        (uchar)event->addr.octet[2]&0xff,
				        (uchar)event->addr.octet[3]&0xff,
				        (uchar)event->addr.octet[4]&0xff,
				        (uchar)event->addr.octet[5]&0xff);
				DHD_ERROR(("Junlim WLC_E_ADDTS_IND %s: status %d, reason %d\n",
				           eabuf, status, reason));
			}
			break;
		case WLC_E_DELTS_IND:
			{
				uint reason;
				char eabuf[ETHER_ADDR_STR_LEN];
				reason = ntoh32_ua((void *)&event->reason);
				sprintf(eabuf, "%02x:%02x:%02x:%02x:%02x:%02x",
				        (uchar)event->addr.octet[0]&0xff,
				        (uchar)event->addr.octet[1]&0xff,
				        (uchar)event->addr.octet[2]&0xff,
				        (uchar)event->addr.octet[3]&0xff,
				        (uchar)event->addr.octet[4]&0xff,
				        (uchar)event->addr.octet[5]&0xff);
				DHD_ERROR(("Junlim WLC_E_DELTS_IND %s: status %d, reason %d\n",
				           eabuf, status, reason));
			}
			break;
#endif 
	case WLC_E_LINK:
	case WLC_E_DEAUTH:
	case WLC_E_DEAUTH_IND:
	case WLC_E_DISASSOC:
	case WLC_E_DISASSOC_IND:
		DHD_EVENT(("%s: Link event %d, flags %x, status %x\n",
		           __FUNCTION__, type, flags, status));
	default:
		*ifidx = dhd_ifname2idx(dhd_pub->info, event->ifname);
		dhd_event(dhd_pub->info, (char *)pvt_data, evlen, *ifidx);
		DHD_TRACE(("%s: MAC event %d, flags %x, status %x\n",
		           __FUNCTION__, type, flags, status));

		if (type == WLC_E_NDIS_LINK) {
			uint32 temp;

			temp = ntoh32_ua((void *)&event->event_type);
			DHD_TRACE(("Converted to WLC_E_LINK type %d\n", temp));

			temp = ntoh32(WLC_E_NDIS_LINK);
			memcpy((void *)(&pvt_data->event.event_type), &temp,
			       sizeof(pvt_data->event.event_type));
		}
		break;
	}

#ifdef SHOW_EVENTS
	wl_show_host_event(event, (void *)event_data);
#endif 

	return (BCME_OK);
}

void
wl_event_to_host_order(wl_event_msg_t * evt)
{
	
	evt->event_type = ntoh32(evt->event_type);
	evt->flags = ntoh16(evt->flags);
	evt->status = ntoh32(evt->status);
	evt->reason = ntoh32(evt->reason);
	evt->auth_type = ntoh32(evt->auth_type);
	evt->datalen = ntoh32(evt->datalen);
	evt->version = ntoh16(evt->version);
}

void
dhd_print_buf(void *pbuf, int len, int bytes_per_line)
{
#ifdef DHD_DEBUG
	int i, j = 0;
	unsigned char *buf = pbuf;

	if (bytes_per_line == 0) {
		bytes_per_line = len;
	}

	for (i = 0; i < len; i++) {
		printf("%2.2x", *buf++);
		j++;
		if (j == bytes_per_line) {
			printf("\n");
			j = 0;
		} else {
			printf(":");
		}
	}
	printf("\n");
#endif 
}

#define strtoul(nptr, endptr, base) bcm_strtoul((nptr), (endptr), (base))

static int
wl_pattern_atoh(char *src, char *dst)
{
	int i;
	if (strncmp(src, "0x", 2) != 0 &&
	    strncmp(src, "0X", 2) != 0) {
		DHD_ERROR(("Mask invalid format. Needs to start with 0x\n"));
		return -1;
	}
	src = src + 2; 
	if (strlen(src) % 2 != 0) {
		DHD_ERROR(("Mask invalid format. Needs to be of even length\n"));
		return -1;
	}
	for (i = 0; *src != '\0'; i++) {
		char num[3];
		bcm_strncpy_s(num, sizeof(num), src, 2);
		num[2] = '\0';
		dst[i] = (uint8)strtoul(num, NULL, 16);
		src += 2;
	}
	return i;
}

void
dhd_pktfilter_offload_enable(dhd_pub_t * dhd, char *arg, int enable, int master_mode)
{
	char				*argv[8];
	int					i = 0;
	const char 			*str;
	int					buf_len;
	int					str_len;
	char				*arg_save = 0, *arg_org = 0;
	int					rc;
	char				buf[128];
	wl_pkt_filter_enable_t	enable_parm;
	wl_pkt_filter_enable_t	* pkt_filterp;

	if (!arg)
		return;

	if (!(arg_save = MALLOC(dhd->osh, strlen(arg) + 1))) {
		DHD_ERROR(("%s: kmalloc failed\n", __FUNCTION__));
		goto fail;
	}
	arg_org = arg_save;
	memcpy(arg_save, arg, strlen(arg) + 1);

	argv[i] = bcmstrtok(&arg_save, " ", 0);

	i = 0;
	if (argv[i] == NULL) {
		DHD_ERROR(("No args provided\n"));
		goto fail;
	}

	str = "pkt_filter_enable";
	str_len = strlen(str);
	bcm_strncpy_s(buf, sizeof(buf), str, str_len);
	buf[str_len] = '\0';
	buf_len = str_len + 1;

	pkt_filterp = (wl_pkt_filter_enable_t *)(buf + str_len + 1);

	enable_parm.id = htod32(strtoul(argv[i], NULL, 0));

	enable_parm.enable = htod32(enable);

	buf_len += sizeof(enable_parm);
	memcpy((char *)pkt_filterp,
	       &enable_parm,
	       sizeof(enable_parm));

	rc = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, buf, buf_len, TRUE, 0);
	rc = rc >= 0 ? 0 : rc;
	if (rc)
		DHD_TRACE(("%s: failed to add pktfilter %s, retcode = %d\n",
		__FUNCTION__, arg, rc));
	else
		DHD_TRACE(("%s: successfully added pktfilter %s\n",
		__FUNCTION__, arg));

	bcm_mkiovar("pkt_filter_mode", (char *)&master_mode, 4, buf, sizeof(buf));
	rc = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, buf, sizeof(buf), TRUE, 0);
	rc = rc >= 0 ? 0 : rc;
	if (rc)
		DHD_TRACE(("%s: failed to add pktfilter %s, retcode = %d\n",
		__FUNCTION__, arg, rc));

fail:
	if (arg_org)
		MFREE(dhd->osh, arg_org, strlen(arg) + 1);
}

void
dhd_pktfilter_offload_set(dhd_pub_t * dhd, char *arg)
{
	const char 			*str;
	wl_pkt_filter_t		pkt_filter;
	wl_pkt_filter_t		*pkt_filterp;
	int					buf_len;
	int					str_len;
	int 				rc;
	uint32				mask_size;
	uint32				pattern_size;
	char				*argv[8], * buf = 0;
	int					i = 0;
	char				*arg_save = 0, *arg_org = 0;
#define BUF_SIZE		2048


	if (!arg)
		return;

	if (!(arg_save = MALLOC(dhd->osh, strlen(arg) + 1))) {
		DHD_ERROR(("%s: kmalloc failed\n", __FUNCTION__));
		goto fail;
	}

	arg_org = arg_save;

	if (!(buf = MALLOC(dhd->osh, BUF_SIZE))) {
		DHD_ERROR(("%s: kmalloc failed\n", __FUNCTION__));
		goto fail;
	}

	memcpy(arg_save, arg, strlen(arg) + 1);

	if (strlen(arg) > BUF_SIZE) {
		DHD_ERROR(("Not enough buffer %d < %d\n", (int)strlen(arg), (int)sizeof(buf)));
		goto fail;
	}

	argv[i] = bcmstrtok(&arg_save, " ", 0);
	while (argv[i++])
		argv[i] = bcmstrtok(&arg_save, " ", 0);

	i = 0;
	if (argv[i] == NULL) {
		DHD_ERROR(("No args provided\n"));
		goto fail;
	}

	str = "pkt_filter_add";
	str_len = strlen(str);
	bcm_strncpy_s(buf, BUF_SIZE, str, str_len);
	buf[ str_len ] = '\0';
	buf_len = str_len + 1;

	pkt_filterp = (wl_pkt_filter_t *) (buf + str_len + 1);

	pkt_filter.id = htod32(strtoul(argv[i], NULL, 0));

	if (argv[++i] == NULL) {
		DHD_ERROR(("Polarity not provided\n"));
		goto fail;
	}

	pkt_filter.negate_match = htod32(strtoul(argv[i], NULL, 0));

	if (argv[++i] == NULL) {
		DHD_ERROR(("Filter type not provided\n"));
		goto fail;
	}

	pkt_filter.type = htod32(strtoul(argv[i], NULL, 0));

	if (argv[++i] == NULL) {
		DHD_ERROR(("Offset not provided\n"));
		goto fail;
	}

	pkt_filter.u.pattern.offset = htod32(strtoul(argv[i], NULL, 0));

	if (argv[++i] == NULL) {
		DHD_ERROR(("Bitmask not provided\n"));
		goto fail;
	}

	mask_size =
		htod32(wl_pattern_atoh(argv[i], (char *) pkt_filterp->u.pattern.mask_and_pattern));

	if (argv[++i] == NULL) {
		DHD_ERROR(("Pattern not provided\n"));
		goto fail;
	}

	pattern_size =
		htod32(wl_pattern_atoh(argv[i],
	         (char *) &pkt_filterp->u.pattern.mask_and_pattern[mask_size]));

	if (mask_size != pattern_size) {
		DHD_ERROR(("Mask and pattern not the same size\n"));
		goto fail;
	}

	pkt_filter.u.pattern.size_bytes = mask_size;
	buf_len += WL_PKT_FILTER_FIXED_LEN;
	buf_len += (WL_PKT_FILTER_PATTERN_FIXED_LEN + 2 * mask_size);

	memcpy((char *)pkt_filterp,
	       &pkt_filter,
	       WL_PKT_FILTER_FIXED_LEN + WL_PKT_FILTER_PATTERN_FIXED_LEN);

	rc = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, buf, buf_len, TRUE, 0);
	rc = rc >= 0 ? 0 : rc;

	if (rc)
		DHD_TRACE(("%s: failed to add pktfilter %s, retcode = %d\n",
		__FUNCTION__, arg, rc));
	else
		DHD_TRACE(("%s: successfully added pktfilter %s\n",
		__FUNCTION__, arg));

fail:
	if (arg_org)
		MFREE(dhd->osh, arg_org, strlen(arg) + 1);

	if (buf)
		MFREE(dhd->osh, buf, BUF_SIZE);
}


#ifdef ARP_OFFLOAD_SUPPORT
void
dhd_arp_offload_set(dhd_pub_t * dhd, int arp_mode)
{
	char iovbuf[32];
	int retcode;

	bcm_mkiovar("arp_ol", (char *)&arp_mode, 4, iovbuf, sizeof(iovbuf));
	retcode = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, iovbuf, sizeof(iovbuf), TRUE, 0);
	retcode = retcode >= 0 ? 0 : retcode;
	if (retcode)
		DHD_TRACE(("%s: failed to set ARP offload mode to 0x%x, retcode = %d\n",
		__FUNCTION__, arp_mode, retcode));
	else
		DHD_TRACE(("%s: successfully set ARP offload mode to 0x%x\n",
		__FUNCTION__, arp_mode));
}

void
dhd_arp_offload_enable(dhd_pub_t * dhd, int arp_enable)
{
	char iovbuf[32];
	int retcode;

	bcm_mkiovar("arpoe", (char *)&arp_enable, 4, iovbuf, sizeof(iovbuf));
	retcode = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, iovbuf, sizeof(iovbuf), TRUE, 0);
	retcode = retcode >= 0 ? 0 : retcode;
	if (retcode)
		DHD_TRACE(("%s: failed to enabe ARP offload to %d, retcode = %d\n",
		__FUNCTION__, arp_enable, retcode));
	else
		DHD_TRACE(("%s: successfully enabed ARP offload to %d\n",
		__FUNCTION__, arp_enable));
}

void dhd_aoe_arp_clr(dhd_pub_t *dhd)
{
	int ret = 0;
	int iov_len = 0;
	char iovbuf[128];

	if (dhd == NULL) return;

	iov_len = bcm_mkiovar("arp_table_clear", 0, 0, iovbuf, sizeof(iovbuf));
	if ((ret  = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, iovbuf, iov_len, TRUE, 0) < 0))
		DHD_ERROR(("%s failed code %d\n", __FUNCTION__, ret));
}

void dhd_aoe_hostip_clr(dhd_pub_t *dhd)
{
	int ret = 0;
	int iov_len = 0;
	char iovbuf[128];

	if (dhd == NULL) return;

	iov_len = bcm_mkiovar("arp_hostip_clear", 0, 0, iovbuf, sizeof(iovbuf));
	if ((ret  = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, iovbuf, iov_len, TRUE, 0)) < 0)
		DHD_ERROR(("%s failed code %d\n", __FUNCTION__, ret));
}

void dhd_arp_offload_add_ip(dhd_pub_t *dhd, uint32 ipaddr)
{
	int iov_len = 0;
	char iovbuf[32];
	int retcode;

	iov_len = bcm_mkiovar("arp_hostip", (char *)&ipaddr, 4, iovbuf, sizeof(iovbuf));
	retcode = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, iovbuf, iov_len, TRUE, 0);

	if (retcode)
		DHD_TRACE(("%s: ARP ip addr add failed, retcode = %d\n",
		__FUNCTION__, retcode));
	else
		DHD_TRACE(("%s: sARP H ipaddr entry added \n",
		__FUNCTION__));
}


int dhd_arp_get_arp_hostip_table(dhd_pub_t *dhd, void *buf, int buflen)
{
	int retcode, i;
	int iov_len = 0;
	uint32 *ptr32 = buf;
	bool clr_bottom = FALSE;

	if (!buf)
		return -1;

	iov_len = bcm_mkiovar("arp_hostip", 0, 0, buf, buflen);
	retcode = dhd_wl_ioctl_cmd(dhd, WLC_GET_VAR, buf, buflen, TRUE, 0);

	if (retcode) {
		DHD_TRACE(("%s: ioctl WLC_GET_VAR error %d\n",
		__FUNCTION__, retcode));

		return -1;
	}

	for (i = 0; i < MAX_IPV4_ENTRIES; i++) {

		if (!clr_bottom) {
			if (*ptr32 == 0)
			clr_bottom = TRUE;
		} else {
			*ptr32 = 0;
		}
		ptr32++;
	}

	return 0;
}
#endif 
#if defined(CONFIG_LGE_BCM432X_PATCH)
#include <linux/fs.h>
#include <linux/ctype.h>

#if 0

CONFIG FILE FORMAT
==================

AVAILABLE PARAMETERS
~~~~~~~~~~~~~~~~~~~~
+====================+=========================================================+
| VARIABLE NAME      | DESCRIPTION                                             |
+====================+=========================================================+
| btc_mode           | BTCoexist                                               |
|                    | 0: disable, 1: enable                                   |
+--------------------+---------------------------------------------------------+
| country            | Country Code                                            |
|                    | KR, EU, US or AU ...                                    |
+--------------------+---------------------------------------------------------+
| vlan_mode          | Specifies the use of 802.1Q Tags (ON, OFF, AUTO).       |
|                    | 0: off, 1: on, -1: auto                                 |
+--------------------+---------------------------------------------------------+
| mpc                | Minimum Power Consumption                               |
|                    | 0: disable, 1: enable                                   |
+--------------------+---------------------------------------------------------+
| wme                | WME QoS                                                 |
|                    | 0: disable, 1: enable                                   |
+--------------------+---------------------------------------------------------+
| wme_apsd           | WME APSD (Advanced Power Save Delivery)                 |
|                    | 0: disable, 1: enable                                   |
+--------------------+---------------------------------------------------------+
| wme_qosinfo        | Set APSD parameters on STA.                             |
|                    | - max_sp_len = number of frames per USP: 0 (all), 2, 4, |
|                    |   or 6                                                  |
|                    | - be, bk, vi, and vo = 0 to disable, 1 to enable U-APSD |
|                    |   per AC                                                |
|                    |        <max_sp_len> <be> <bk> <vi> <vo>                 |
|                    | 0x0f =      0         1    1    1    1                  |
|                    | 0x2f =      2         1    1    1    1                  |
|                    | 0x4f =      4         1    1    1    1                  |
|                    | 0x6f =      6         1    1    1    1                  |
|                    | 0x03 =      0         0    0    1    1                  |
+--------------------+---------------------------------------------------------+
| wme_auto_trigger   | 0: disable, 1: enable                                   |
+--------------------+---------------------------------------------------------+
| wme_apsd_trigger   | in msec, 0: disable                                     |
+--------------------+---------------------------------------------------------+
| roam_off           | 0: roaming on, 1: roaming off                           |
+--------------------+---------------------------------------------------------+
| roam_scan_period   | in sec                                                  |
+--------------------+---------------------------------------------------------+
| roam_delta         | in dB                                                   |
+--------------------+---------------------------------------------------------+
| roam_trigger       | in dBm                                                  |
+--------------------+---------------------------------------------------------+
| PM                 | Power Saving Mode                                       |
|                    | 0: off, 1: max, 2: fast                                 |
+--------------------+---------------------------------------------------------+
| assoc_listen       | The Listen Interval sent in association requests        |
|                    | number of beacon                                        |
+--------------------+---------------------------------------------------------+

EXAMPLE
~~~~~~~
btc_mode=1
country=AU
vlan_mode=0
mpc=1
wme=1
wme_apsd=0
wme_qosinfo=0x00
wme_auto_trigger=1
wme_apsd_trigger=0
roam_off=0
roam_scan_period=10
roam_delta=20
roam_trigger=-70
PM=2
assoc_listen=1

#endif

#if defined (CONFIG_LGE_BCM432X_PATCH)
bool PM_control = TRUE;
#endif

static int dhd_preinit_proc(dhd_pub_t *dhd, int ifidx, char *name, char *value)
{
	int var_int;

	/* For specific country style. ex) AU/2 */
	wl_country_t cspec = {{0}, -1, {0}};
	char *revstr;
	char *endptr = NULL;
	int iolen;
	char smbuf[WLC_IOCTL_SMLEN*2];


	if (!strcmp(name, "country")) {

		revstr = strchr(value, '/');
		if (revstr) {
			cspec.rev = strtoul(revstr + 1, &endptr, 10);
			memcpy(cspec.country_abbrev,value,WLC_CNTRY_BUF_SZ);
			cspec.country_abbrev[2] = '\0';
			memcpy(cspec.ccode,cspec.country_abbrev,WLC_CNTRY_BUF_SZ);
			memset(smbuf, 0, sizeof(smbuf));
			printf("config country code is country : %s, rev : %d !!\n", cspec.country_abbrev,cspec.rev);
			iolen = bcm_mkiovar("country", (char*)&cspec, sizeof(cspec), smbuf, sizeof(smbuf));

			return dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR,
					smbuf, iolen, TRUE, 0);
		}

		return dhd_wl_ioctl_cmd(dhd, WLC_SET_COUNTRY,
				value, WLC_CNTRY_BUF_SZ, TRUE, 0);
	} else if (!strcmp(name, "roam_scan_period")) {
		var_int = (int)simple_strtol(value, NULL, 0);
		return dhd_wl_ioctl_cmd(dhd,WLC_SET_ROAM_SCAN_PERIOD,
				&var_int, sizeof(var_int), TRUE, 0);
	} else if (!strcmp(name, "roam_delta") || !strcmp(name, "roam_trigger")) {
		struct {
			int val;
			int band;
		} x;
		x.val = (int)simple_strtol(value, NULL, 0);
		x.band = WLC_BAND_ALL;	
		return dhd_wl_ioctl_cmd(dhd, strcmp(name, "roam_delta") ?
				WLC_SET_ROAM_TRIGGER : WLC_SET_ROAM_DELTA, &x, sizeof(x), TRUE, 0);
	} else if (!strcmp(name, "PM")) {
		var_int = (int)simple_strtol(value, NULL, 0);
		
#if defined (CONFIG_LGE_BCM432X_PATCH)
		if(var_int == 0){
			PM_control = FALSE;
			printk("%s var_int=%d don't control PM\n", __func__,var_int);
		}
		else
		{
			PM_control = TRUE;
			printk("%s var_int=%d do control PM\n", __func__,var_int);
		}
#endif		
		return dhd_wl_ioctl_cmd(dhd, WLC_SET_PM,
				&var_int, sizeof(var_int), TRUE, 0);
	} else if(!strcmp(name,"cur_etheraddr")) {
		struct ether_addr ea;
		char buf[32];
		uint iovlen;
		int ret;

		bcm_ether_atoe(value, &ea);

#if !defined (CONFIG_LGE_BCM432X_PATCH)
		ret = memcmp( &ea.octet, dhd->mac.octet, ETHER_ADDR_LEN);
		if(ret == 0){
			DHD_ERROR(("%s: Same Macaddr\n",__FUNCTION__));
			return 0;
		}
#endif

		DHD_ERROR(("%s: Change Macaddr = %02X:%02X:%02X:%02X:%02X:%02X\n",__FUNCTION__,
					ea.octet[0], ea.octet[1], ea.octet[2],
					ea.octet[3], ea.octet[4], ea.octet[5]));

		iovlen = bcm_mkiovar("cur_etheraddr", (char*)&ea, ETHER_ADDR_LEN, buf, 32);

		ret = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, buf, iovlen, TRUE, 0);
		if (ret < 0) {
			DHD_ERROR(("%s: can't set MAC address , error=%d\n", __FUNCTION__, ret));
			return ret;
		}
		else{
			memcpy(dhd->mac.octet, (void *)&ea, ETHER_ADDR_LEN);
			return ret;
		}
	} else {
		uint iovlen;
		char iovbuf[WLC_IOCTL_SMLEN];

		var_int = (int)simple_strtol(value, NULL, 0);

		if(!strcmp(name, "roam_off")) {
			if (var_int) {
				uint bcn_timeout = 2;
				bcm_mkiovar("bcn_timeout", (char *)&bcn_timeout, 4, iovbuf, sizeof(iovbuf));
				dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, iovbuf, sizeof(iovbuf), TRUE, 0);
			}
		}

#ifdef BCMCCX								
		if(var_int)
			printk(" roam_off =%d BCMCCX roam_off should be 0\n",var_int);
#endif											
		iovlen = bcm_mkiovar(name, (char *)&var_int, sizeof(var_int),
				iovbuf, sizeof(iovbuf));
		return dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR,
				iovbuf, iovlen, TRUE, 0);
	}

	return 0;
}

static int dhd_preinit_config(dhd_pub_t *dhd, int ifidx)
{
	mm_segment_t old_fs;
	struct kstat stat;
	struct file *fp = NULL;
	unsigned int len;
	char *buf = NULL, *p, *name, *value;
	int ret = 0;

	if (!*config_path)
		return 0;

	old_fs = get_fs();
	set_fs(get_ds());
	if ((ret = vfs_stat(config_path, &stat))) {
		set_fs(old_fs);
		printk(KERN_ERR "%s: Failed to get information (%d)\n",
				config_path, ret);
		return ret;
	}
	set_fs(old_fs);

	if (!(buf = MALLOC(dhd->osh, stat.size + 1))) {
		printk(KERN_ERR "Failed to allocate memory %llu bytes\n", stat.size);
		return -ENOMEM;
	}

	if (!(fp = dhd_os_open_image(config_path)) ||
		(len = dhd_os_get_image_block(buf, stat.size, fp)) < 0)
		goto err;

	buf[stat.size] = '\0';
	for (p = buf; *p; p++) {
		if (isspace(*p))
			continue;
		for (name = p++; *p && !isspace(*p); p++) {
			if (*p == '=') {
				*p = '\0';
				p++;
				for (value = p; *p && !isspace(*p); p++);
				*p = '\0';
				if ((ret = dhd_preinit_proc(dhd, ifidx, name, value)) < 0)
					printk(KERN_ERR "%s: %s=%s\n",
							bcmerrorstr(ret), name, value);
				break;
			}
		}
	}
	ret = 0;

out:
	if (fp)
		dhd_os_close_image(fp);
	if (buf)
		MFREE(dhd->osh, buf, stat.size+1);
	return ret;

err:
	ret = -1;
	goto out;
}
#endif 

int
dhd_preinit_ioctls(dhd_pub_t *dhd)
{
	int ret = 0;
	char eventmask[WL_EVENTING_MASK_LEN];
	char iovbuf[WL_EVENTING_MASK_LEN + 12];	

	uint up = 0;
#if !defined(CONFIG_LGE_BCM432X_PATCH)
	uint power_mode = PM_FAST;
#endif 
	uint32 dongle_align = DHD_SDALIGN;
	uint32 glom = 0;
	uint bcn_timeout = 4;
	uint retry_max = 3;
#if defined(ARP_OFFLOAD_SUPPORT)
	int arpoe = 1;
#endif
#ifndef BCMCCX								
	int scan_assoc_time = 40;
	int scan_unassoc_time = 40;
#endif
	const char 				*str;
	wl_pkt_filter_t		pkt_filter;
	wl_pkt_filter_t		*pkt_filterp;
	int						buf_len;
	int						str_len;
	uint32					mask_size;
	uint32					pattern_size;
	char buf[WLC_IOCTL_SMLEN];
	char *ptr;
#if defined(CONFIG_LGE_BCM432X_PATCH)
	uint filter_mode = 0;
#else
	uint filter_mode = 1;
#endif 
	uint32 listen_interval = LISTEN_INTERVAL; 
#if defined(SOFTAP)
	uint dtim = 1;
#endif
#if (defined(AP) && !defined(WLP2P)) || (!defined(AP) && defined(WL_CFG80211))
	uint32 mpc = 0; 
#endif

#if defined(AP) || defined(WLP2P)
	uint32 apsta = 1; 
#endif 
#ifdef GET_CUSTOM_MAC_ENABLE
	struct ether_addr ea_addr;
#endif 

	dhd->op_mode = 0;
#ifdef GET_CUSTOM_MAC_ENABLE
	ret = dhd_custom_get_mac_address(ea_addr.octet);
	if (!ret) {
		memset(buf, 0, sizeof(buf));
		bcm_mkiovar("cur_etheraddr", (void *)&ea_addr, ETHER_ADDR_LEN, buf, sizeof(buf));
		ret = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, buf, sizeof(buf), TRUE, 0);
		if (ret < 0) {
			DHD_ERROR(("%s: can't set MAC address , error=%d\n", __FUNCTION__, ret));
			return BCME_NOTUP;
		}
	} else {
#endif 
		memset(buf, 0, sizeof(buf));
		bcm_mkiovar("cur_etheraddr", 0, 0, buf, sizeof(buf));
		if ((ret = dhd_wl_ioctl_cmd(dhd, WLC_GET_VAR, buf, sizeof(buf),
			FALSE, 0)) < 0) {
			DHD_ERROR(("%s: can't get MAC address , error=%d\n", __FUNCTION__, ret));
			return BCME_NOTUP;
		}
		memcpy(dhd->mac.octet, buf, ETHER_ADDR_LEN);
#ifdef GET_CUSTOM_MAC_ENABLE
	}
#endif 
#if defined(CONFIG_LGE_BCM432X_PATCH)
	dhd_preinit_config(dhd, 0);
#endif

#ifdef SET_RANDOM_MAC_SOFTAP
	if (strstr(fw_path, "_apsta") != NULL) {
		uint rand_mac;

		srandom32((uint)jiffies);
		rand_mac = random32();
		iovbuf[0] = 0x02;              
		iovbuf[1] = 0x1A;
		iovbuf[2] = 0x11;
		iovbuf[3] = (unsigned char)(rand_mac & 0x0F) | 0xF0;
		iovbuf[4] = (unsigned char)(rand_mac >> 8);
		iovbuf[5] = (unsigned char)(rand_mac >> 16);

		bcm_mkiovar("cur_etheraddr", (void *)iovbuf, ETHER_ADDR_LEN, buf, sizeof(buf));
		ret = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, buf, sizeof(buf), TRUE, 0);
		if (ret < 0) {
			DHD_ERROR(("%s: can't set MAC address , error=%d\n", __FUNCTION__, ret));
		} else
			memcpy(dhd->mac.octet, iovbuf, ETHER_ADDR_LEN);
	}
#endif 

	DHD_TRACE(("Firmware = %s\n", fw_path));
#if !defined(AP) && defined(WLP2P)
	if (strstr(fw_path, "_p2p") != NULL) {
		bcm_mkiovar("apsta", (char *)&apsta, 4, iovbuf, sizeof(iovbuf));
		if ((ret = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR,
			iovbuf, sizeof(iovbuf), TRUE, 0)) < 0) {
			DHD_ERROR(("%s APSTA for WFD failed ret= %d\n", __FUNCTION__, ret));
		} else {
			dhd->op_mode |= WFD_MASK;
#if defined(ARP_OFFLOAD_SUPPORT)
			arpoe = 0;
#endif
			dhd_pkt_filter_enable = FALSE;
		}
	}
#endif 

#if !defined(AP) && defined(WL_CFG80211)
	if (strstr(fw_path, "_apsta") != NULL) {
			bcm_mkiovar("mpc", (char *)&mpc, 4, iovbuf, sizeof(iovbuf));
			if ((ret = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, iovbuf,
				sizeof(iovbuf), TRUE, 0)) < 0) {
				DHD_ERROR(("%s mpc for HostAPD failed  %d\n", __FUNCTION__, ret));
			} else {
				dhd->op_mode |= HOSTAPD_MASK;
#if defined(ARP_OFFLOAD_SUPPORT)
				arpoe = 0;
#endif 
				dhd_pkt_filter_enable = FALSE;
			}
	}
#endif 

	if ((dhd->op_mode != WFD_MASK) && (dhd->op_mode != HOSTAPD_MASK)) {
		printf("dhd->op_mode=%d\n", dhd->op_mode);
		dhd->op_mode |= STA_MASK;
		dhd_pkt_filter_enable = TRUE;
	}

	DHD_ERROR(("Firmware up: op_mode=%d, "
			"Broadcom Dongle Host Driver mac=%.2x:%.2x:%.2x:%.2x:%.2x:%.2x\n",
			dhd->op_mode,
			dhd->mac.octet[0], dhd->mac.octet[1], dhd->mac.octet[2],
			dhd->mac.octet[3], dhd->mac.octet[4], dhd->mac.octet[5]));

	if (dhd->dhd_cspec.ccode[0] != 0) {
		bcm_mkiovar("country", (char *)&dhd->dhd_cspec,
			sizeof(wl_country_t), iovbuf, sizeof(iovbuf));
		if ((ret = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, iovbuf, sizeof(iovbuf), TRUE, 0)) < 0)
			DHD_ERROR(("%s: country code setting failed\n", __FUNCTION__));
	}

	bcm_mkiovar("assoc_listen", (char *)&listen_interval, 4, iovbuf, sizeof(iovbuf));
	if ((ret = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, iovbuf, sizeof(iovbuf), TRUE, 0)) < 0)
		DHD_ERROR(("%s assoc_listen failed %d\n", __FUNCTION__, ret));

	memset(buf, 0, sizeof(buf));
	ptr = buf;
	bcm_mkiovar("ver", (char *)&buf, 4, buf, sizeof(buf));
	if ((ret  = dhd_wl_ioctl_cmd(dhd, WLC_GET_VAR, buf, sizeof(buf), FALSE, 0)) < 0)
		DHD_ERROR(("%s failed %d\n", __FUNCTION__, ret));
	else {
		bcmstrtok(&ptr, "\n", 0);
		DHD_ERROR(("Firmware version = %s\n", buf));
	}

#if !defined(CONFIG_LGE_BCM432X_PATCH)
	dhd_wl_ioctl_cmd(dhd, WLC_SET_PM, (char *)&power_mode, sizeof(power_mode), TRUE, 0);
#endif 

	bcm_mkiovar("bus:txglomalign", (char *)&dongle_align, 4, iovbuf, sizeof(iovbuf));
	dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, iovbuf, sizeof(iovbuf), TRUE, 0);

	bcm_mkiovar("bus:txglom", (char *)&glom, 4, iovbuf, sizeof(iovbuf));
	dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, iovbuf, sizeof(iovbuf), TRUE, 0);

	bcm_mkiovar("bcn_timeout", (char *)&bcn_timeout, 4, iovbuf, sizeof(iovbuf));
	dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, iovbuf, sizeof(iovbuf), TRUE, 0);
	bcm_mkiovar("assoc_retry_max", (char *)&retry_max, 4, iovbuf, sizeof(iovbuf));
	dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, iovbuf, sizeof(iovbuf), TRUE, 0);
#if defined(AP) && !defined(WLP2P)
	bcm_mkiovar("mpc", (char *)&mpc, 4, iovbuf, sizeof(iovbuf));
	dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, iovbuf, sizeof(iovbuf), TRUE, 0);

	bcm_mkiovar("apsta", (char *)&apsta, 4, iovbuf, sizeof(iovbuf));
	dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, iovbuf, sizeof(iovbuf), TRUE, 0);
#endif
#if defined(SOFTAP)
	if (ap_fw_loaded == TRUE) {
		dhd_wl_ioctl_cmd(dhd, WLC_SET_DTIMPRD, (char *)&dtim, sizeof(dtim), TRUE, 0);
	}
#endif 

#if defined(KEEP_ALIVE)
	{
	int res;

#if defined(SOFTAP)
	if (ap_fw_loaded == FALSE)
#endif 
		if ((res = dhd_keep_alive_onoff(dhd)) < 0)
			DHD_ERROR(("%s set keeplive failed %d\n",
			__FUNCTION__, res));
	}
#endif 
	
	ret = dhd_wl_ioctl_cmd(dhd, WLC_UP, (char *)&up, sizeof(up), TRUE, 0);
	if (ret < 0)
		goto done;



	bcm_mkiovar("event_msgs", eventmask, WL_EVENTING_MASK_LEN, iovbuf, sizeof(iovbuf));
	dhd_wl_ioctl_cmd(dhd, WLC_GET_VAR, iovbuf, sizeof(iovbuf), FALSE, 0);
	if (ret < 0)
		goto done;
	bcopy(iovbuf, eventmask, WL_EVENTING_MASK_LEN);

	setbit(eventmask, WLC_E_SET_SSID);
	setbit(eventmask, WLC_E_PRUNE);
	setbit(eventmask, WLC_E_AUTH);
	setbit(eventmask, WLC_E_REASSOC);
	setbit(eventmask, WLC_E_REASSOC_IND);
	setbit(eventmask, WLC_E_DEAUTH_IND);
	setbit(eventmask, WLC_E_DISASSOC_IND);
	setbit(eventmask, WLC_E_DISASSOC);
	setbit(eventmask, WLC_E_JOIN);
	setbit(eventmask, WLC_E_ASSOC_IND);
	setbit(eventmask, WLC_E_PSK_SUP);
	setbit(eventmask, WLC_E_LINK);
	setbit(eventmask, WLC_E_NDIS_LINK);
	setbit(eventmask, WLC_E_MIC_ERROR);
	setbit(eventmask, WLC_E_PMKID_CACHE);
	setbit(eventmask, WLC_E_TXFAIL);
	setbit(eventmask, WLC_E_JOIN_START);
	setbit(eventmask, WLC_E_SCAN_COMPLETE);
#ifdef WLMEDIA_HTSF
	setbit(eventmask, WLC_E_HTSFSYNC);
#endif

#ifdef PNO_SUPPORT
	setbit(eventmask, WLC_E_PFN_NET_FOUND);
#endif
#if defined(BCMCCX) && defined(BCMDBG_EVENT)
	setbit(eventmask, WLC_E_ADDTS_IND);
	setbit(eventmask, WLC_E_DELTS_IND);
#endif 

	bcm_mkiovar("event_msgs", eventmask, WL_EVENTING_MASK_LEN, iovbuf, sizeof(iovbuf));
	dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, iovbuf, sizeof(iovbuf), TRUE, 0);
#ifndef BCMCCX								
	dhd_wl_ioctl_cmd(dhd, WLC_SET_SCAN_CHANNEL_TIME, (char *)&scan_assoc_time,
		sizeof(scan_assoc_time), TRUE, 0);
	dhd_wl_ioctl_cmd(dhd, WLC_SET_SCAN_UNASSOC_TIME, (char *)&scan_unassoc_time,
		sizeof(scan_unassoc_time), TRUE, 0);
#endif								

	str = "pkt_filter_add";
	str_len = strlen(str);
	strncpy(buf, str, str_len);
	buf[ str_len ] = '\0';
	buf_len = str_len + 1;

	pkt_filterp = (wl_pkt_filter_t *) (buf + str_len + 1);

	pkt_filter.id = htod32(100);

	pkt_filter.negate_match = htod32(0);

	pkt_filter.type = htod32(0);

	pkt_filter.u.pattern.offset = htod32(0);

#if defined(CONFIG_LGE_BCM432X_PATCH)
	mask_size =	htod32(wl_pattern_atoh("0xff",
		(char *) pkt_filterp->u.pattern.mask_and_pattern));

	if (mask_size < 0)	{
		DHD_ERROR(("%s : mask_size(%d) is fail\n", __FUNCTION__, mask_size));
		return -EINVAL;
	}

	pattern_size = htod32(wl_pattern_atoh("0xff",
		(char *) &pkt_filterp->u.pattern.mask_and_pattern[mask_size]));
#else
	mask_size =	htod32(wl_pattern_atoh("0x01",
		(char *) pkt_filterp->u.pattern.mask_and_pattern));
	if (mask_size < 0)	{
		DHD_ERROR(("%s : mask_size(%d) is fail\n", __FUNCTION__, mask_size));
		return -EINVAL;
	}
	pattern_size = htod32(wl_pattern_atoh("0x00",
		(char *) &pkt_filterp->u.pattern.mask_and_pattern[mask_size]));
#endif 

	if (mask_size != pattern_size) {
		DHD_ERROR(("Mask and pattern not the same size\n"));
		return -EINVAL;
	}

	pkt_filter.u.pattern.size_bytes = mask_size;
	buf_len += WL_PKT_FILTER_FIXED_LEN;
	buf_len += (WL_PKT_FILTER_PATTERN_FIXED_LEN + 2 * mask_size);

	
	memcpy((char *)pkt_filterp, &pkt_filter,
		WL_PKT_FILTER_FIXED_LEN + WL_PKT_FILTER_PATTERN_FIXED_LEN);

	dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, buf, buf_len, TRUE, 0);

	bcm_mkiovar("pkt_filter_mode", (char *)&filter_mode, 4, iovbuf, sizeof(iovbuf));
	dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, iovbuf, sizeof(iovbuf), TRUE, 0);

#ifdef ARP_OFFLOAD_SUPPORT
	if (arpoe && !ap_fw_loaded) {
		dhd_arp_offload_set(dhd, dhd_arp_mode);
		dhd_arp_offload_enable(dhd, arpoe);
	} else {
		dhd_arp_offload_set(dhd, 0);
		dhd_arp_offload_enable(dhd, FALSE);
	}
#endif 

#ifdef PKT_FILTER_SUPPORT
	if (ap_fw_loaded) {
		int i;
		for (i = 0; i < dhd->pktfilter_count; i++) {
			dhd_pktfilter_offload_enable(dhd, dhd->pktfilter[i],
				0, dhd_master_mode);
		}
	}
#endif 


done:
	return ret;
}

void
dhd_sendup_event_common(dhd_pub_t *dhdp, wl_event_msg_t *event, void *data)
{
	switch (ntoh32(event->event_type)) {
	case WLC_E_BTA_HCI_EVENT:
		break;
	default:
		break;
	}

	dhd_sendup_event(dhdp, event, data);
}



#ifdef SIMPLE_ISCAN

uint iscan_thread_id = 0;
iscan_buf_t * iscan_chain = 0;

iscan_buf_t *
dhd_iscan_allocate_buf(dhd_pub_t *dhd, iscan_buf_t **iscanbuf)
{
	iscan_buf_t *iscanbuf_alloc = 0;
	iscan_buf_t *iscanbuf_head;

	DHD_ISCAN(("%s: Entered\n", __FUNCTION__));
	dhd_iscan_lock();

	iscanbuf_alloc = (iscan_buf_t*)MALLOC(dhd->osh, sizeof(iscan_buf_t));
	if (iscanbuf_alloc == NULL)
		goto fail;

	iscanbuf_alloc->next = NULL;
	iscanbuf_head = *iscanbuf;

	DHD_ISCAN(("%s: addr of allocated node = 0x%X"
		   "addr of iscanbuf_head = 0x%X dhd = 0x%X\n",
		   __FUNCTION__, iscanbuf_alloc, iscanbuf_head, dhd));

	if (iscanbuf_head == NULL) {
		*iscanbuf = iscanbuf_alloc;
		DHD_ISCAN(("%s: Head is allocated\n", __FUNCTION__));
		goto fail;
	}

	while (iscanbuf_head->next)
		iscanbuf_head = iscanbuf_head->next;

	iscanbuf_head->next = iscanbuf_alloc;

fail:
	dhd_iscan_unlock();
	return iscanbuf_alloc;
}

void
dhd_iscan_free_buf(void *dhdp, iscan_buf_t *iscan_delete)
{
	iscan_buf_t *iscanbuf_free = 0;
	iscan_buf_t *iscanbuf_prv = 0;
	iscan_buf_t *iscanbuf_cur;
	dhd_pub_t *dhd = dhd_bus_pub(dhdp);
	DHD_ISCAN(("%s: Entered\n", __FUNCTION__));

	dhd_iscan_lock();

	iscanbuf_cur = iscan_chain;

	if (!iscan_delete) {
		while (iscanbuf_cur) {
			iscanbuf_free = iscanbuf_cur;
			iscanbuf_cur = iscanbuf_cur->next;
			iscanbuf_free->next = 0;
			MFREE(dhd->osh, iscanbuf_free, sizeof(iscan_buf_t));
		}
		iscan_chain = 0;
	} else {
		while (iscanbuf_cur) {
			if (iscanbuf_cur == iscan_delete)
				break;
			iscanbuf_prv = iscanbuf_cur;
			iscanbuf_cur = iscanbuf_cur->next;
		}
		if (iscanbuf_prv)
			iscanbuf_prv->next = iscan_delete->next;

		iscan_delete->next = 0;
		MFREE(dhd->osh, iscan_delete, sizeof(iscan_buf_t));

		if (!iscanbuf_prv)
			iscan_chain = 0;
	}
	dhd_iscan_unlock();
}

iscan_buf_t *
dhd_iscan_result_buf(void)
{
	return iscan_chain;
}

int
dhd_iscan_issue_request(void * dhdp, wl_iscan_params_t *pParams, uint32 size)
{
	int rc = -1;
	dhd_pub_t *dhd = dhd_bus_pub(dhdp);
	char *buf;
	char iovar[] = "iscan";
	uint32 allocSize = 0;
	wl_ioctl_t ioctl;

	if (pParams) {
		allocSize = (size + strlen(iovar) + 1);
		if ((allocSize < size) || (allocSize < strlen(iovar)))
		{
			DHD_ERROR(("%s: overflow - allocation size too large %d < %d + %d!\n",
				__FUNCTION__, allocSize, size, strlen(iovar)));
			goto cleanUp;
		}
		buf = MALLOC(dhd->osh, allocSize);

		if (buf == NULL)
			{
			DHD_ERROR(("%s: malloc of size %d failed!\n", __FUNCTION__, allocSize));
			goto cleanUp;
			}
		ioctl.cmd = WLC_SET_VAR;
		bcm_mkiovar(iovar, (char *)pParams, size, buf, allocSize);
		rc = dhd_wl_ioctl(dhd, 0, &ioctl, buf, allocSize);
	}

cleanUp:
	if (buf) {
		MFREE(dhd->osh, buf, allocSize);
	}

	return rc;
}

static int
dhd_iscan_get_partial_result(void *dhdp, uint *scan_count)
{
	wl_iscan_results_t *list_buf;
	wl_iscan_results_t list;
	wl_scan_results_t *results;
	iscan_buf_t *iscan_cur;
	int status = -1;
	dhd_pub_t *dhd = dhd_bus_pub(dhdp);
	int rc;
	wl_ioctl_t ioctl;

	DHD_ISCAN(("%s: Enter\n", __FUNCTION__));

	iscan_cur = dhd_iscan_allocate_buf(dhd, &iscan_chain);
	if (!iscan_cur) {
		DHD_ERROR(("%s: Failed to allocate node\n", __FUNCTION__));
		dhd_iscan_free_buf(dhdp, 0);
		dhd_iscan_request(dhdp, WL_SCAN_ACTION_ABORT);
		dhd_ind_scan_confirm(dhdp, FALSE);
		goto fail;
	}

	dhd_iscan_lock();

	memset(iscan_cur->iscan_buf, 0, WLC_IW_ISCAN_MAXLEN);
	list_buf = (wl_iscan_results_t*)iscan_cur->iscan_buf;
	results = &list_buf->results;
	results->buflen = WL_ISCAN_RESULTS_FIXED_SIZE;
	results->version = 0;
	results->count = 0;

	memset(&list, 0, sizeof(list));
	list.results.buflen = htod32(WLC_IW_ISCAN_MAXLEN);
	bcm_mkiovar("iscanresults", (char *)&list, WL_ISCAN_RESULTS_FIXED_SIZE,
		iscan_cur->iscan_buf, WLC_IW_ISCAN_MAXLEN);
	ioctl.cmd = WLC_GET_VAR;
	ioctl.set = FALSE;
	rc = dhd_wl_ioctl(dhd, 0, &ioctl, iscan_cur->iscan_buf, WLC_IW_ISCAN_MAXLEN);

	results->buflen = dtoh32(results->buflen);
	results->version = dtoh32(results->version);
	*scan_count = results->count = dtoh32(results->count);
	status = dtoh32(list_buf->status);
	DHD_ISCAN(("%s: Got %d resuls status = (%x)\n", __FUNCTION__, results->count, status));

	dhd_iscan_unlock();

	if (!(*scan_count)) {
		dhd_iscan_free_buf(dhdp, 0);
	}
fail:
	return status;
}

#endif 

int
dhd_get_dtim_skip(dhd_pub_t *dhd)
{
	int bcn_li_dtim;
	int ret;
	uint8 bssid[6];
	int dtim_assoc = 0;

	if ((dhd->dtim_skip == 0) || (dhd->dtim_skip == 1))
		bcn_li_dtim = 3;
	else
		bcn_li_dtim = dhd->dtim_skip;

	if ((ret = dhd_wl_ioctl_cmd(dhd, WLC_GET_BSSID,
		(char *)&bssid, ETHER_ADDR_LEN, FALSE, 0)) == BCME_NOTASSOCIATED) {
		DHD_TRACE(("%s NOT assoc ret %d\n", __FUNCTION__, ret));
		goto exit;
	}

	if ((ret = dhd_wl_ioctl_cmd(dhd, WLC_GET_DTIMPRD,
		&dtim_assoc, sizeof(dtim_assoc), FALSE, 0)) < 0) {
		DHD_ERROR(("%s failed code %d\n", __FUNCTION__, ret));
		goto exit;
	}

	DHD_ERROR(("%s bcn_li_dtim=%d DTIM=%d Listen=%d\n",
		__FUNCTION__, bcn_li_dtim, dtim_assoc, LISTEN_INTERVAL));

	if (dtim_assoc == 0) {
		goto exit;
	}

	if (dtim_assoc > LISTEN_INTERVAL) {
		bcn_li_dtim = 1;
		DHD_ERROR(("%s DTIM=%d > Listen=%d : too big ...\n",
			__FUNCTION__, dtim_assoc, LISTEN_INTERVAL));
		goto exit;
	}

	if ((bcn_li_dtim * dtim_assoc) > LISTEN_INTERVAL) {
		bcn_li_dtim = (int)(LISTEN_INTERVAL / dtim_assoc);
		DHD_TRACE(("%s agjust dtim_skip as %d\n", __FUNCTION__, bcn_li_dtim));
	}

exit:
	return bcn_li_dtim;
}
bool dhd_check_ap_wfd_mode_set(dhd_pub_t *dhd)
{
#ifdef  WL_CFG80211
	if (((dhd->op_mode & HOSTAPD_MASK) == HOSTAPD_MASK) ||
		((dhd->op_mode & WFD_MASK) == WFD_MASK))
		return TRUE;
	else
#endif 	
		return FALSE;
}

#ifdef PNO_SUPPORT
int
dhd_pno_clean(dhd_pub_t *dhd)
{
	char iovbuf[128];
	int pfn_enabled = 0;
	int iov_len = 0;
	int ret;

	
	iov_len = bcm_mkiovar("pfn", (char *)&pfn_enabled, 4, iovbuf, sizeof(iovbuf));
	if ((ret = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, iovbuf, sizeof(iovbuf), TRUE, 0)) >= 0) {
		
		iov_len = bcm_mkiovar("pfnclear", 0, 0, iovbuf, sizeof(iovbuf));
		if (iov_len) {
			if ((ret = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, iovbuf,
			                            iov_len, TRUE, 0)) < 0) {
				DHD_ERROR(("%s failed code %d\n", __FUNCTION__, ret));
			}
		}
		else {
			ret = -1;
			DHD_ERROR(("%s failed code %d\n", __FUNCTION__, iov_len));
		}
	}
	else
		DHD_ERROR(("%s failed code %d\n", __FUNCTION__, ret));

	return ret;
}

int
dhd_pno_enable(dhd_pub_t *dhd, int pfn_enabled)
{
	char iovbuf[128];
	uint8 bssid[6];
	int ret = -1;

	if ((!dhd) && ((pfn_enabled != 0) || (pfn_enabled != 1))) {
		DHD_ERROR(("%s error exit\n", __FUNCTION__));
		return ret;
	}

	if (dhd_check_ap_wfd_mode_set(dhd) == TRUE)
		return (ret);

	memset(iovbuf, 0, sizeof(iovbuf));
	if ((pfn_enabled) &&
		((ret = dhd_wl_ioctl_cmd(dhd, WLC_GET_BSSID,
		(char *)&bssid, ETHER_ADDR_LEN, TRUE, 0)) == BCME_NOTASSOCIATED)) {
			DHD_TRACE(("%s pno enable called in disassoc mode\n", __FUNCTION__));
	}
	else if (pfn_enabled) {
			DHD_ERROR(("%s pno enable called in assoc mode ret=%d\n",
				__FUNCTION__, ret));
			return ret;
	}
	if ((ret = bcm_mkiovar("pfn", (char *)&pfn_enabled, 4, iovbuf, sizeof(iovbuf))) > 0) {
		if ((ret = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, iovbuf,
		                            sizeof(iovbuf), TRUE, 0)) < 0) {
			DHD_ERROR(("%s failed for error=%d\n", __FUNCTION__, ret));
			return ret;
		}
		else {
			dhd->pno_enable = pfn_enabled;
			DHD_TRACE(("%s set pno as %d\n", __FUNCTION__, dhd->pno_enable));
		}
	}
	else DHD_ERROR(("%s failed err=%d\n", __FUNCTION__, ret));

	return ret;
}

int
dhd_pno_set(dhd_pub_t *dhd, wlc_ssid_t* ssids_local, int nssid, ushort scan_fr,
	int pno_repeat, int pno_freq_expo_max)
{
	int err = -1;
	char iovbuf[128];
	int k, i;
	wl_pfn_param_t pfn_param;
	wl_pfn_t	pfn_element;
	uint len = 0;

	DHD_TRACE(("%s nssid=%d nchan=%d\n", __FUNCTION__, nssid, scan_fr));

	if ((!dhd) && (!ssids_local)) {
		DHD_ERROR(("%s error exit\n", __FUNCTION__));
		err = -1;
	}

	if (dhd_check_ap_wfd_mode_set(dhd) == TRUE)
		return (err);

	for (k = 0; k < nssid; k++) {
		if (!ssids_local[k].SSID_len) {
			DHD_ERROR(("%d: Broadcast SSID is ilegal for PNO setting\n", k));
			return err;
		}
	}
#ifdef PNO_DUMP
	{
		int j;
		for (j = 0; j < nssid; j++) {
			DHD_ERROR(("%d: scan  for  %s size =%d\n", j,
				ssids_local[j].SSID, ssids_local[j].SSID_len));
		}
	}
#endif 

	if  ((err = dhd_pno_clean(dhd)) < 0) {
		DHD_ERROR(("%s failed error=%d\n", __FUNCTION__, err));
		return err;
	}
	memset(iovbuf, 0, sizeof(iovbuf));
	memset(&pfn_param, 0, sizeof(pfn_param));
	memset(&pfn_element, 0, sizeof(pfn_element));

	pfn_param.version = htod32(PFN_VERSION);
	pfn_param.flags = htod16((PFN_LIST_ORDER << SORT_CRITERIA_BIT));

	if ((pno_repeat != 0) || (pno_freq_expo_max != 0)) {
		pfn_param.flags |= htod16(ENABLE << ENABLE_ADAPTSCAN_BIT);
		pfn_param.repeat = (uchar) (pno_repeat);
		pfn_param.exp = (uchar) (pno_freq_expo_max);
	}
	if (scan_fr  != 0)
		pfn_param.scan_freq = htod32(scan_fr);

	if (pfn_param.scan_freq > PNO_SCAN_MAX_FW_SEC) {
		DHD_ERROR(("%s pno freq above %d sec\n", __FUNCTION__, PNO_SCAN_MAX_FW_SEC));
		return err;
	}
	if (pfn_param.scan_freq < PNO_SCAN_MIN_FW_SEC) {
		DHD_ERROR(("%s pno freq less %d sec\n", __FUNCTION__, PNO_SCAN_MIN_FW_SEC));
		return err;
	}

	len = bcm_mkiovar("pfn_set", (char *)&pfn_param, sizeof(pfn_param), iovbuf, sizeof(iovbuf));
	if ((err = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, iovbuf, len, TRUE, 0)) < 0) {
				DHD_ERROR(("%s pfn_set failed for error=%d\n",
					__FUNCTION__, err));
				return err;
	}

	for (i = 0; i < nssid; i++) {

		pfn_element.infra = htod32(DOT11_BSSTYPE_INFRASTRUCTURE);
		pfn_element.auth = (DOT11_OPEN_SYSTEM);
		pfn_element.wpa_auth = htod32(WPA_AUTH_PFN_ANY);
		pfn_element.wsec = htod32(0);
		pfn_element.infra = htod32(1);

		memcpy((char *)pfn_element.ssid.SSID, ssids_local[i].SSID, ssids_local[i].SSID_len);
		pfn_element.ssid.SSID_len = ssids_local[i].SSID_len;

		if ((len =
		bcm_mkiovar("pfn_add", (char *)&pfn_element,
			sizeof(pfn_element), iovbuf, sizeof(iovbuf))) > 0) {
			if ((err =
			dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, iovbuf, len, TRUE, 0)) < 0) {
				DHD_ERROR(("%s failed for i=%d error=%d\n",
					__FUNCTION__, i, err));
				return err;
			}
			else
				DHD_ERROR(("%s set OK with PNO time=%d repeat=%d max_adjust=%d\n",
					__FUNCTION__, pfn_param.scan_freq,
					pfn_param.repeat, pfn_param.exp));
		}
		else DHD_ERROR(("%s failed err=%d\n", __FUNCTION__, err));
	}

	return err;
}

int
dhd_pno_get_status(dhd_pub_t *dhd)
{
	int ret = -1;

	if (!dhd)
		return ret;
	else
		return (dhd->pno_enable);
}

#endif 

#if defined(KEEP_ALIVE)
int dhd_keep_alive_onoff(dhd_pub_t *dhd)
{
	char 				buf[256];
	const char 			*str;
	wl_mkeep_alive_pkt_t	mkeep_alive_pkt;
	wl_mkeep_alive_pkt_t	*mkeep_alive_pktp;
	int					buf_len;
	int					str_len;
	int res 				= -1;

	if (dhd_check_ap_wfd_mode_set(dhd) == TRUE)
		return (res);

	DHD_TRACE(("%s execution\n", __FUNCTION__));

	str = "mkeep_alive";
	str_len = strlen(str);
	strncpy(buf, str, str_len);
	buf[ str_len ] = '\0';
	mkeep_alive_pktp = (wl_mkeep_alive_pkt_t *) (buf + str_len + 1);
	mkeep_alive_pkt.period_msec = KEEP_ALIVE_PERIOD;
	buf_len = str_len + 1;
	mkeep_alive_pkt.version = htod16(WL_MKEEP_ALIVE_VERSION);
	mkeep_alive_pkt.length = htod16(WL_MKEEP_ALIVE_FIXED_LEN);
	mkeep_alive_pkt.keep_alive_id = 0;
	mkeep_alive_pkt.len_bytes = 0;
	buf_len += WL_MKEEP_ALIVE_FIXED_LEN;
	
	memcpy((char *)mkeep_alive_pktp, &mkeep_alive_pkt, WL_MKEEP_ALIVE_FIXED_LEN);

	res = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, buf, buf_len, TRUE, 0);

	return res;
}
#endif 
int
wl_iw_parse_data_tlv(char** list_str, void *dst, int dst_size, const char token,
                     int input_size, int *bytes_left)
{
	char* str = *list_str;
	uint16 short_temp;
	uint32 int_temp;

	if ((list_str == NULL) || (*list_str == NULL) ||(bytes_left == NULL) || (*bytes_left < 0)) {
		DHD_ERROR(("%s error paramters\n", __FUNCTION__));
		return -1;
	}

	memset(dst, 0, dst_size);
	while (*bytes_left > 0) {

		if (str[0] != token) {
			DHD_TRACE(("%s NOT Type=%d get=%d left_parse=%d \n",
				__FUNCTION__, token, str[0], *bytes_left));
			return -1;
		}

		*bytes_left -= 1;
		str += 1;

		if (input_size == 1) {
			memcpy(dst, str, input_size);
		}
		else if (input_size == 2) {
			memcpy(dst, (char *)htod16(memcpy(&short_temp, str, input_size)),
				input_size);
		}
		else if (input_size == 4) {
			memcpy(dst, (char *)htod32(memcpy(&int_temp, str, input_size)),
				input_size);
		}

		*bytes_left -= input_size;
		str += input_size;
		*list_str = str;
		return 1;
	}
	return 1;
}


int
wl_iw_parse_channel_list_tlv(char** list_str, uint16* channel_list,
                             int channel_num, int *bytes_left)
{
	char* str = *list_str;
	int idx = 0;

	if ((list_str == NULL) || (*list_str == NULL) ||(bytes_left == NULL) || (*bytes_left < 0)) {
		DHD_ERROR(("%s error paramters\n", __FUNCTION__));
		return -1;
	}

	while (*bytes_left > 0) {

		if (str[0] != CSCAN_TLV_TYPE_CHANNEL_IE) {
			*list_str = str;
			DHD_TRACE(("End channel=%d left_parse=%d %d\n", idx, *bytes_left, str[0]));
			return idx;
		}
		*bytes_left -= 1;
		str += 1;

		if (str[0] == 0) {
			
			channel_list[idx] = 0x0;
		}
		else {
			channel_list[idx] = (uint16)str[0];
			DHD_TRACE(("%s channel=%d \n", __FUNCTION__,  channel_list[idx]));
		}
		*bytes_left -= 1;
		str += 1;

		if (idx++ > 255) {
			DHD_ERROR(("%s Too many channels \n", __FUNCTION__));
			return -1;
		}
	}

	*list_str = str;
	return idx;
}

int
wl_iw_parse_ssid_list_tlv(char** list_str, wlc_ssid_t* ssid, int max, int *bytes_left)
{
	char* str =  *list_str;
	int idx = 0;

	if ((list_str == NULL) || (*list_str == NULL) || (*bytes_left < 0)) {
		DHD_ERROR(("%s error paramters\n", __FUNCTION__));
		return -1;
	}

	while (*bytes_left > 0) {

		if (str[0] != CSCAN_TLV_TYPE_SSID_IE) {
			*list_str = str;
			DHD_TRACE(("nssid=%d left_parse=%d %d\n", idx, *bytes_left, str[0]));
			return idx;
		}

		*bytes_left -= 1;
		str += 1;

		if (str[0] == 0) {
			ssid[idx].SSID_len = 0;
			memset((char*)ssid[idx].SSID, 0x0, DOT11_MAX_SSID_LEN);
			*bytes_left -= 1;
			str += 1;

			DHD_TRACE(("BROADCAST SCAN  left=%d\n", *bytes_left));
		}
		else if (str[0] <= DOT11_MAX_SSID_LEN) {
			ssid[idx].SSID_len = str[0];
			*bytes_left -= 1;
			str += 1;

			if (ssid[idx].SSID_len > *bytes_left) {
				DHD_ERROR(("%s out of memory range len=%d but left=%d\n",
				__FUNCTION__, ssid[idx].SSID_len, *bytes_left));
				return -1;
			}

			memcpy((char*)ssid[idx].SSID, str, ssid[idx].SSID_len);

			*bytes_left -= ssid[idx].SSID_len;
			str += ssid[idx].SSID_len;

			DHD_TRACE(("%s :size=%d left=%d\n",
				(char*)ssid[idx].SSID, ssid[idx].SSID_len, *bytes_left));
		}
		else {
			DHD_ERROR(("### SSID size more that %d\n", str[0]));
			return -1;
		}

		if (idx++ >  max) {
			DHD_ERROR(("%s number of SSIDs more that %d\n", __FUNCTION__, idx));
			return -1;
		}
	}

	*list_str = str;
	return idx;
}


int
wl_iw_parse_ssid_list(char** list_str, wlc_ssid_t* ssid, int idx, int max)
{
	char* str, *ptr;

	if ((list_str == NULL) || (*list_str == NULL))
		return -1;

	for (str = *list_str; str != NULL; str = ptr) {

		if (!strncmp(str, GET_CHANNEL, strlen(GET_CHANNEL))) {
			*list_str	 = str + strlen(GET_CHANNEL);
			return idx;
		}

		if ((ptr = strchr(str, ',')) != NULL) {
			*ptr++ = '\0';
		}

		if (strlen(str) > DOT11_MAX_SSID_LEN) {
			DHD_ERROR(("ssid <%s> exceeds %d\n", str, DOT11_MAX_SSID_LEN));
			return -1;
		}

		if (strlen(str) == 0)
			ssid[idx].SSID_len = 0;

		if (idx < max) {
			bcm_strcpy_s((char*)ssid[idx].SSID, sizeof(ssid[idx].SSID), str);
			ssid[idx].SSID_len = strlen(str);
		}
		idx++;
	}
	return idx;
}


int
wl_iw_parse_channel_list(char** list_str, uint16* channel_list, int channel_num)
{
	int num;
	int val;
	char* str;
	char* endptr = NULL;

	if ((list_str == NULL)||(*list_str == NULL))
		return -1;

	str = *list_str;
	num = 0;
	while (strncmp(str, GET_NPROBE, strlen(GET_NPROBE))) {
		val = (int)strtoul(str, &endptr, 0);
		if (endptr == str) {
			printf("could not parse channel number starting at"
				" substring \"%s\" in list:\n%s\n",
				str, *list_str);
			return -1;
		}
		str = endptr + strspn(endptr, " ,");

		if (num == channel_num) {
			DHD_ERROR(("too many channels (more than %d) in channel list:\n%s\n",
				channel_num, *list_str));
			return -1;
		}

		channel_list[num++] = (uint16)val;
	}
	*list_str = str;
	return num;
}
