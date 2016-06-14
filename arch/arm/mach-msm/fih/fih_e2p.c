#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/io.h>
#include <fih/share/e2p.h>

#define FIH_E2P_DATA_ST  FIH_CSP_E2P_V02

#define FIH_E2P_ST_ADDR  0x10881000
#define FIH_E2P_ST_SIZE  4096

static int proc_calc_metrics(char *page, char **start, off_t off, int count,
		int *eof, int len)
{
	if (len <= off+count) *eof = 1;
	*start = page + off;
	len -= off;
	if (len>count) len = count;
	if (len<0) len = 0;
	return len;
}

static int fih_e2p_proc_read_pid(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	int len;
	FIH_E2P_DATA_ST *e2p;

	e2p = (FIH_E2P_DATA_ST *)ioremap(FIH_E2P_ST_ADDR, sizeof(FIH_E2P_DATA_ST));
	if (e2p == NULL) {
		pr_err("%s: ioremap fail\n", __func__);
		return (0);
	}

	len = snprintf(page, PAGE_SIZE, "%s\n", e2p->pid);

	iounmap(e2p);

	return proc_calc_metrics(page, start, off, count, eof, len);
}

static int fih_e2p_proc_read_bt_mac(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	int len;
	FIH_E2P_DATA_ST *e2p;

	e2p = (FIH_E2P_DATA_ST *)ioremap(FIH_E2P_ST_ADDR, sizeof(FIH_E2P_DATA_ST));
	if (e2p == NULL) {
		pr_err("%s: ioremap fail\n", __func__);
		return (0);
	}

	len = snprintf(page, PAGE_SIZE, "%02X:%02X:%02X:%02X:%02X:%02X\n",
		e2p->bt_mac[0], e2p->bt_mac[1], e2p->bt_mac[2], e2p->bt_mac[3], e2p->bt_mac[4], e2p->bt_mac[5]);

	iounmap(e2p);

	return proc_calc_metrics(page, start, off, count, eof, len);
}

static int fih_e2p_proc_read_wifi_mac(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	int len;
	FIH_E2P_DATA_ST *e2p;

	e2p = (FIH_E2P_DATA_ST *)ioremap(FIH_E2P_ST_ADDR, sizeof(FIH_E2P_DATA_ST));
	if (e2p == NULL) {
		pr_err("%s: ioremap fail\n", __func__);
		return (0);
	}

	len = snprintf(page, PAGE_SIZE, "%02X:%02X:%02X:%02X:%02X:%02X\n",
		e2p->wifi_mac[0], e2p->wifi_mac[1], e2p->wifi_mac[2], e2p->wifi_mac[3], e2p->wifi_mac[4], e2p->wifi_mac[5]);

	iounmap(e2p);

	return proc_calc_metrics(page, start, off, count, eof, len);
}

static struct {
		char *name;
		int (*read_proc)(char*,char**,off_t,int,int*,void*);
} *p, fih_e2p[] = {
	{"productid", fih_e2p_proc_read_pid},
	{"bt_mac", fih_e2p_proc_read_bt_mac},
	{"wifi_mac", fih_e2p_proc_read_wifi_mac},
	{NULL}, };

static int __init fih_e2p_init(void)
{
	if (FIH_E2P_ST_SIZE < sizeof(FIH_E2P_DATA_ST)) {
		pr_err("%s: WARNNING!! FIH_E2P_ST_SIZE (%d) < sizeof(FIH_E2P_DATA_ST) (%d)",
			__func__, FIH_E2P_ST_SIZE, sizeof(FIH_E2P_DATA_ST));
		return (1);
	}

	for (p = fih_e2p; p->name; p++) {
		if (create_proc_read_entry(p->name, 0, NULL, p->read_proc, NULL) == NULL) {
			pr_err("fail to create proc/%s\n", p->name);
		}
	}

	return (0);
}

static void __exit fih_e2p_exit(void)
{
	for (p = fih_e2p; p->name; p++) {
		remove_proc_entry(p->name, NULL);
	}
}

module_init(fih_e2p_init);
module_exit(fih_e2p_exit);
