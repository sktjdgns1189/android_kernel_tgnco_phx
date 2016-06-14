#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/delay.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <mach/msm_iomap.h>
#include <linux/io.h>
#include <linux/init.h>
#include <fih/hwid.h>

#define FIH_CUST_NV_ST_ADDR 0x10300000
#define FIH_CUST_NV_ST_SIZE SZ_2M

#define SIM_OFFSET 512
#define SIM_SIZE 256
#define SIM_XML "/hidden/data/CDALog/SIM_Config"

static int fih_def_sim_to_cust(void)
{
	char *path = NULL;
	int fsize = 0;
	char sim_xml[16];

	path = (char *)ioremap(FIH_CUST_NV_ST_ADDR, FIH_CUST_NV_ST_SIZE);
	if (path == NULL) {
		pr_err("[%s] ioremap fail\n", __func__);
		return 0;
	}

	fsize = sizeof(sim_xml);
	if (fsize > SIM_SIZE) {
		pr_err("[%s] warning: fsize (%d > %d)\n", __func__, fsize, SIM_SIZE);
		fsize = SIM_SIZE;
	}

	strcpy(sim_xml, "single\n");

	memset((path + (SZ_2M - SIM_OFFSET)), 0x0, SIM_SIZE);
	memcpy((path + (SZ_2M - SIM_OFFSET)), sim_xml, fsize);
	path[(SZ_2M - SIM_OFFSET + fsize)] = '\0';
	iounmap(path);

	pr_info("[%s] done\n", __func__);
	return 3;
}

int fih_update_sim_to_cust(void)
{
	struct file *fp = NULL;
	mm_segment_t oldfs;
	char *buf = NULL;
	char *path = NULL;
	int fsize = 0;
	int output_size = 0;

	oldfs = get_fs();
	set_fs(KERNEL_DS);

	fp = filp_open(SIM_XML, O_RDONLY, 0);
	if (IS_ERR(fp)){
		pr_err("[%s] fail to open %s\n", __func__, SIM_XML);
		set_fs(oldfs);
		return fih_def_sim_to_cust();
	}

	fsize = fp->f_op->llseek(fp, 0, SEEK_END);
	fp->f_op->llseek(fp, 0, SEEK_SET);

	pr_info("[%s] fsize = %d\n", __func__, fsize);
	buf = kzalloc(fsize, GFP_KERNEL);
	if (buf == NULL) {
		pr_err("[%s] kzalloc buf fail\n", __func__);
		filp_close(fp,NULL);
		set_fs(oldfs);
		return fih_def_sim_to_cust();
	}

	output_size = fp->f_op->read(fp, buf, fsize, &fp->f_pos);
	if (output_size < fsize) {
		pr_err("[%s] fail to read %s\n", __func__, SIM_XML);
		kfree(buf);
		filp_close(fp,NULL);
		set_fs(oldfs);
		return fih_def_sim_to_cust();
	}

	filp_close(fp, NULL);
	set_fs(oldfs);

	if (fsize > SIM_SIZE) {
		fsize = SIM_SIZE;
		pr_err("[%s] warning: fsize (%d > %d)\n", __func__, fsize, SIM_SIZE);
	}

	pr_info("[%s] xml = (%s)\n", __func__, buf);
	path = (char *)ioremap(FIH_CUST_NV_ST_ADDR, FIH_CUST_NV_ST_SIZE);
	memset((path + (SZ_2M - SIM_OFFSET)), 0x0, SIM_SIZE);
	memcpy((path + (SZ_2M - SIM_OFFSET)), buf, fsize);
	path[(SZ_2M - SIM_OFFSET + fsize)] = '\0';
	iounmap(path);
	kfree(buf);

	pr_info("[%s] done\n", __func__);
	return 1;
}
EXPORT_SYMBOL(fih_update_sim_to_cust);

static char *my_proc_name = "mdm_sim_xml";
static unsigned int my_proc_addr = FIH_CUST_NV_ST_ADDR;
static unsigned int my_proc_size = FIH_CUST_NV_ST_SIZE;
static unsigned int my_proc_len = 256;

/* This function is called at the beginning of a sequence.
 * ie, when:
 * - the /proc file is read (first time)
 * - after the function stop (end of sequence)
 */
static void *my_seq_start(struct seq_file *s, loff_t *pos)
{
	if (((*pos)*PAGE_SIZE) >= my_proc_len) return NULL;
	return (void *)((unsigned long) *pos+1);
}

/* This function is called after the beginning of a sequence.
 * It's called untill the return is NULL (this ends the sequence).
 */
static void *my_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	++*pos;
	return my_seq_start(s, pos);
}

/* This function is called at the end of a sequence
 */
static void my_seq_stop(struct seq_file *s, void *v)
{
	/* nothing to do, we use a static value in start() */
}

/* This function is called for each "step" of a sequence
 */
static int my_seq_show(struct seq_file *s, void *v)
{
	int n = (int)v-1;
	char *buf = (char *)ioremap(my_proc_addr, my_proc_size);
	char *tmp = NULL;
	unsigned int offset = SIM_OFFSET;
	unsigned int size = SIM_SIZE;
	unsigned int i;

	if (buf == NULL) {
		return 0;
	}

	tmp = buf + (my_proc_size - offset);
	for (i = 0; i < size; i++ ) {
		if (tmp[i] == 0x00) break;
	}
	my_proc_len = i;

	if (my_proc_len < (PAGE_SIZE*(n+1))) {
		seq_write(s, (tmp+(PAGE_SIZE*n)), (my_proc_len - (PAGE_SIZE*n)));
	} else {
		seq_write(s, (tmp+(PAGE_SIZE*n)), PAGE_SIZE);
	}

	iounmap(buf);

	return 0;
}

/* This structure gather "function" to manage the sequence
 */
static struct seq_operations my_seq_ops = {
	.start = my_seq_start,
	.next  = my_seq_next,
	.stop  = my_seq_stop,
	.show  = my_seq_show
};

/* This function is called when the /proc file is open.
 */
static int my_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &my_seq_ops);
};

/* This structure gather "function" that manage the /proc file
 */
static struct file_operations my_file_ops = {
	.owner   = THIS_MODULE,
	.open    = my_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release
};

/* This function is called when the module is loaded
 */
static int __init my_module_init(void)
{
	struct proc_dir_entry *entry;

	entry = create_proc_entry(my_proc_name, 0444, NULL);
	if (entry) {
		entry->proc_fops = &my_file_ops;
	}

	return 0;
}

/* This function is called when the module is unloaded.
 */
static void __exit my_module_exit(void)
{
	remove_proc_entry(my_proc_name, NULL);
}

module_init(my_module_init);
module_exit(my_module_exit);
