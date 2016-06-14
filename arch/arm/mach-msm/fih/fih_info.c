#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <fih/hwid.h>

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

static int fih_info_proc_read_project(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	int len;
	char msg[8];

	switch (fih_hwid_fetch(FIH_HWID_PRJ)) {
		case FIH_PRJ_PHX: strcpy(msg, "PHX"); break;
		default: strcpy(msg, "N/A"); break;
	}
	len = snprintf(page, PAGE_SIZE, "%s\n", msg);

	return proc_calc_metrics(page, start, off, count, eof, len);
}

static int fih_info_proc_read_hw_rev(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	int len;
	char msg[8];

	switch (fih_hwid_fetch(FIH_HWID_REV)) {
		case FIH_REV_EVB:  strcpy(msg, "EVB"); break;
		case FIH_REV_EVB2: strcpy(msg, "EVB2"); break;
		case FIH_REV_EVB3: strcpy(msg, "EVB3"); break;
		case FIH_REV_EVT:  strcpy(msg, "EVT"); break;
		case FIH_REV_EVT2: strcpy(msg, "EVT2"); break;
		case FIH_REV_EVT3: strcpy(msg, "EVT3"); break;
		case FIH_REV_EVT4: strcpy(msg, "EVT4"); break;
		case FIH_REV_EVT5: strcpy(msg, "EVT5"); break;
		case FIH_REV_EVT6: strcpy(msg, "EVT6"); break;
		case FIH_REV_DVT:  strcpy(msg, "DVT"); break;
		case FIH_REV_DVT2: strcpy(msg, "DVT2"); break;
		case FIH_REV_DVT3: strcpy(msg, "DVT3"); break;
		case FIH_REV_DVT4: strcpy(msg, "DVT4"); break;
		case FIH_REV_DVT5: strcpy(msg, "DVT5"); break;
		case FIH_REV_DVT6: strcpy(msg, "DVT6"); break;
		case FIH_REV_PVT:  strcpy(msg, "PVT"); break;
		case FIH_REV_PVT2: strcpy(msg, "PVT2"); break;
		case FIH_REV_PVT3: strcpy(msg, "PVT3"); break;
		case FIH_REV_MP:   strcpy(msg, "MP"); break;
		case FIH_REV_MP2:  strcpy(msg, "MP2"); break;
		case FIH_REV_MP3:  strcpy(msg, "MP3"); break;
		default: strcpy(msg, "N/A"); break;
	}
	len = snprintf(page, PAGE_SIZE, "%s\n", msg);

	return proc_calc_metrics(page, start, off, count, eof, len);
}

static int fih_info_proc_read_rf_band(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	int len;
	char msg[128];

	switch (fih_hwid_fetch(FIH_HWID_RF)) {
		case FIH_RF_G_850_900_1800_1900_W_1_2_L_1_3_5_7_9_17:
			strcpy(msg, "G_850_900_1800_1900^W_1_2^L_1_3_5_7_9_17"); break;
		case FIH_RF_G_850_900_1800_1900_W_1_2_L_1_3_5_7_8_9:
			strcpy(msg, "G_850_900_1800_1900^W_1_2^L_1_3_5_7_8_9"); break;
		/* NO BAND */
		case FIH_RF_NONE: strcpy(msg, "NONE"); break;
		default: strcpy(msg, "UNKNOWN\n"); break;
	}
	len = snprintf(page, PAGE_SIZE, "%s\n", msg);

	return proc_calc_metrics(page, start, off, count, eof, len);
}

static int fih_info_proc_read_simslot(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	int slot = 0;
	int len;

	slot = 1;
	len = snprintf(page, PAGE_SIZE, "%d\n", slot);

	return proc_calc_metrics(page, start, off, count, eof, len);
}

static struct {
		char *name;
		int (*read_proc)(char*,char**,off_t,int,int*,void*);
} *p, fih_info[] = {
	{"devmodel", fih_info_proc_read_project},
	{"baseband", fih_info_proc_read_hw_rev},
	{"bandinfo", fih_info_proc_read_rf_band},
	{"SIMSlot", fih_info_proc_read_simslot},
	{NULL}, };

static int __init fih_info_init(void)
{
	for (p = fih_info; p->name; p++) {
		if (create_proc_read_entry(p->name, 0, NULL, p->read_proc, NULL) == NULL) {
			pr_err("fail to create proc/%s\n", p->name);
		}
	}

	return (0);
}

static void __exit fih_info_exit(void)
{
	for (p = fih_info; p->name; p++) {
		remove_proc_entry(p->name, NULL);
	}
}

module_init(fih_info_init);
module_exit(fih_info_exit);
