#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/io.h>

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

static int fih_cda_proc_read_skuid(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	int len = snprintf(page, PAGE_SIZE, "%s\n", "21LKR");
	return proc_calc_metrics(page, start, off, count, eof, len);
}

static int fih_cda_proc_read_simid(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	int len = snprintf(page, PAGE_SIZE, "%s\n", "21LKR");
	return proc_calc_metrics(page, start, off, count, eof, len);
}

static struct {
		char *name;
		int (*read_proc)(char*,char**,off_t,int,int*,void*);
} *p, fih_cda[] = {
	{"cda/skuid", fih_cda_proc_read_skuid},
	{"cda/simid", fih_cda_proc_read_simid},
	{NULL}, };

static int __init fih_cda_init(void)
{
	proc_mkdir("cda", NULL);
	for (p = fih_cda; p->name; p++) {
		if (create_proc_read_entry(p->name, 0, NULL, p->read_proc, NULL) == NULL) {
			//proc_mkdir("cda", NULL);
			if (create_proc_read_entry(p->name, 0, NULL, p->read_proc, NULL) == NULL) {
				pr_err("fail to create proc/%s\n", p->name);
			}
		}
	}

	return (0);
}

static void __exit fih_cda_exit(void)
{
	for (p = fih_cda; p->name; p++) {
		remove_proc_entry(p->name, NULL);
	}
}

module_init(fih_cda_init);
module_exit(fih_cda_exit);
