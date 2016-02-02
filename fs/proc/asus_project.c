#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/platform_device.h>   // <asus-olaf20150715+>

static struct proc_dir_entry *project_id_proc_file;
static int project_id_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d\n", asus_PRJ_ID);
	return 0;
}

static int project_id_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, project_id_proc_read, NULL);
}


static struct file_operations project_id_proc_ops = {
	.open = project_id_proc_open,
	.read = seq_read,
	.release = single_release,
};

static void create_project_id_proc_file(void)
{
    printk("create_project_id_proc_file\n");
    project_id_proc_file = proc_create("apid", 0444,NULL, &project_id_proc_ops);
    if(project_id_proc_file){
        printk("create project_id_proc_file sucessed!\n");
    }else{
		printk("create project_id_proc_file failed!\n");
    }
}

static struct proc_dir_entry *project_stage_proc_file;
static int project_stage_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%s\n", asus_project_stage);
	return 0;
}

static int project_stage_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, project_stage_proc_read, NULL);
}


static struct file_operations project_stage_proc_ops = {
	.open = project_stage_proc_open,
	.read = seq_read,
	.release = single_release,
};

static void create_project_stage_proc_file(void)
{
    printk("create_project_stage_proc_file\n");
    project_stage_proc_file = proc_create("apsta", 0444,NULL, &project_stage_proc_ops);
    if(project_stage_proc_file){
        printk("create project_stage_proc_file sucessed!\n");
    }else{
		printk("create project_stage_proc_file failed!\n");
    }
}

static struct proc_dir_entry *project_RFsku_proc_file;
static int project_RFsku_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%s\n", asus_project_RFsku);
	return 0;
}

static int project_RFsku_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, project_RFsku_proc_read, NULL);
}


static struct file_operations project_RFsku_proc_ops = {
	.open = project_RFsku_proc_open,
	.read = seq_read,
	.release = single_release,
};

static void create_project_RFsku_proc_file(void)
{
    printk("create_project_RFsku_proc_file\n");
    project_RFsku_proc_file = proc_create("aprf", 0444,NULL, &project_RFsku_proc_ops);
    if(project_RFsku_proc_file){
        printk("create project_RFsku_proc_file sucessed!\n");
    }else{
		printk("create project_RFsku_proc_file failed!\n");
    }
}

static struct proc_dir_entry *project_lte_proc_file;
static int project_lte_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%s\n", asus_project_lte);
	return 0;
}

static int project_lte_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, project_lte_proc_read, NULL);
}


static struct file_operations project_lte_proc_ops = {
	.open = project_lte_proc_open,
	.read = seq_read,
	.release = single_release,
};

static void create_project_lte_proc_file(void)
{
    printk("create_project_lte_proc_file\n");
    project_lte_proc_file = proc_create("aplte", 0444,NULL, &project_lte_proc_ops);
    if(project_lte_proc_file){
        printk("create project_lte_proc_file sucessed!\n");
    }else{
		printk("create project_lte_proc_file failed!\n");
    }
}

static struct proc_dir_entry *project_mem_proc_file;
static int project_mem_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%s\n", asus_project_mem);
	return 0;
}

static int project_mem_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, project_mem_proc_read, NULL);
}


static struct file_operations project_mem_proc_ops = {
	.open = project_mem_proc_open,
	.read = seq_read,
	.release = single_release,
};

static void create_project_mem_proc_file(void)
{
    printk("create_project_mem_proc_file\n");
    project_mem_proc_file = proc_create("apmem", 0444,NULL, &project_mem_proc_ops);
    if(project_mem_proc_file){
        printk("create project_mem_proc_file sucessed!\n");
    }else{
		printk("create project_mem_proc_file failed!\n");
    }
}

extern char asus_project_hd[2];

static struct proc_dir_entry *project_hd_proc_file;
static int project_hd_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%s\n", asus_project_hd);
	return 0;
}

static int project_hd_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, project_hd_proc_read, NULL);
}


static struct file_operations project_hd_proc_ops = {
	.open = project_hd_proc_open,
	.read = seq_read,
	.release = single_release,	
};

static void create_project_hd_proc_file(void)
{
    printk("create_project_hd_proc_file\n");
    project_hd_proc_file = proc_create("aphd", 0444,NULL, &project_hd_proc_ops);
    if(project_hd_proc_file){
        printk("create project_hd_proc_file sucessed!\n");
    }else{
		printk("create project_hd_proc_file failed!\n");
    }
}

// <asus-olaf20150904+>
int ZE600KL_Project_ID_Pin_Pinctrl_Configure(struct device *dev, bool active)
{
	struct pinctrl_state *set_state;
	int retval = 0;
	if (active) {
		set_state =	pinctrl_lookup_state(devm_pinctrl_get(dev), "Project_ID_Pin_active");
		if (IS_ERR(set_state)) {
			printk("cannot get project id pin pinctrl active state\n");
			return PTR_ERR(set_state);
		}
	} else {
		set_state = pinctrl_lookup_state(devm_pinctrl_get(dev), "Project_ID_Pin_suspend");
		if (IS_ERR(set_state)) {
			printk("cannot get project id pin pinctrl sleep state\n");
			return PTR_ERR(set_state);
		}
	}
	retval = pinctrl_select_state(devm_pinctrl_get(dev), set_state);
	if (retval) {
		printk("cannot set project id pin pinctrl active state\n");
		return retval;
	}
	return retval;
}

int ZD550KL_Project_ID_Pin_Pinctrl_Configure(struct device *dev, bool active)
{
	struct pinctrl_state *set_state;
	int retval = 0;
	if (active) {
		set_state =	pinctrl_lookup_state(devm_pinctrl_get(dev), "Project_ID_Pin_active");
		if (IS_ERR(set_state)) {
			printk("cannot get project id pin pinctrl active state\n");
			return PTR_ERR(set_state);
		}
	} else {
		set_state = pinctrl_lookup_state(devm_pinctrl_get(dev), "Project_ID_Pin_suspend");
		if (IS_ERR(set_state)) {
			printk("cannot get project id pin pinctrl sleep state\n");
			return PTR_ERR(set_state);
		}
	}
	retval = pinctrl_select_state(devm_pinctrl_get(dev), set_state);
	if (retval) {
		printk("cannot set project id pin pinctrl active state\n");
		return retval;
	}
	return retval;
}


static int Project_ID_Pin_Probe(struct platform_device *pdev)
{
	int error = 0;
	struct device *dev = &pdev->dev;
	switch (asus_PRJ_ID) {
		case 1: // ASUS_ZE600KL     config project gpio ID pin to "Input Pull Up 2mA"
			printk("Set ZE600KL Project ID Pin Input Pull Up 2mA\n");
			error = ZE600KL_Project_ID_Pin_Pinctrl_Configure(dev, true);
			if (error) {
				printk("cannot set ZE600KL project id pin pinctrl active state\n");
			}
			break;
		case 3: // ASUS_ZD550KL     config project gpio ID pin to "Input Pull Up 2mA"
			printk("Set ZD550KL Project ID Pin Input Pull Up 2mA\n");
			error = ZD550KL_Project_ID_Pin_Pinctrl_Configure(dev, true);
			if (error) {
				printk("cannot set ZD550KL project id pin pinctrl active state\n");
			}
			break;
		default:
			break;
	}
	return error;
}

static int Project_ID_Pin_Suspend(struct device *dev)
{
	int error = 0;
	switch (asus_PRJ_ID) {
		case 1: // ASUS_ZE600KL     set project gpio ID pin to "Input Pull Down 2mA" in Suspend state
			error = ZE600KL_Project_ID_Pin_Pinctrl_Configure(dev, false);
			if (error) {
				printk("failed to pull down ID pin in suspend state\n");
			}
			printk("pull ID pin down in suspend state success\n");
			break;
		case 3: // ASUS_ZD550KL     set project gpio ID pin to "Input Pull Down 2mA" in Suspend state
			error = ZD550KL_Project_ID_Pin_Pinctrl_Configure(dev, false);
			if (error) {
				printk("failed to pull down ID pin in suspend state\n");
			}
			printk("pull ID pin down in suspend state success\n");
			break;
		default:
			break;
	}
	return error;
}

static int Project_ID_Pin_Resume(struct device *dev)
{
	int error = 0;
	switch (asus_PRJ_ID) {
		case 1: // ASUS_ZE600KL     set project gpio ID pin to "Input Pull Up 2mA" in Resume state
			error = ZE600KL_Project_ID_Pin_Pinctrl_Configure(dev, true);
			if (error) {
				printk("failed to pull up ID pin in resume state\n");
			}
			printk("pull ID pin up in resume state success\n");
			break;
		case 3: // ASUS_ZD550KL     set project gpio ID pin to "Input Pull Up 2mA" in Resume state
			error = ZD550KL_Project_ID_Pin_Pinctrl_Configure(dev, true);
			if (error) {
				printk("failed to pull up ID pin in resume state\n");
			}
			printk("pull ID pin up in resume state success\n");
			break;
		default:
			break;
	}
	return error;
}

#define DRIVER_NAME "Project_ID_Pin"

static const struct platform_device_id Project_ID_Pin_table[] = {
        {DRIVER_NAME, 1},
};

static struct of_device_id Project_ID_Pin_match_table[] = {
	{ .compatible = "qcom,Project_ID_Pin",},
	{},
};

static SIMPLE_DEV_PM_OPS(Project_ID_Pin_PM_ops, Project_ID_Pin_Suspend, Project_ID_Pin_Resume);

static struct platform_driver Project_ID_Pin_Init = {
    .driver = {
        .name = DRIVER_NAME,
        .owner = THIS_MODULE,
		.pm	= &Project_ID_Pin_PM_ops,
        .of_match_table = Project_ID_Pin_match_table,
    },
    .probe    = Project_ID_Pin_Probe,	
    .id_table = Project_ID_Pin_table,
};
// <asus-olaf20150904->

static int __init proc_asusPRJ_init(void)
{
	create_project_id_proc_file();
	create_project_lte_proc_file();
	create_project_RFsku_proc_file();
	create_project_stage_proc_file();
	create_project_mem_proc_file();
	create_project_hd_proc_file();
	platform_driver_register(&Project_ID_Pin_Init);   // <asus-olaf20150715+>
	return 0;
}
module_init(proc_asusPRJ_init);
