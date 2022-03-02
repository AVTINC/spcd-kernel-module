/*
    SPCD Kernel Driver
    Copyright (C) 2022 AVT, Inc.

    TODO: Contact Info (email)
    TODO: Address

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  US
*/
struct spcd_data {
	struct device	*dev;
	struct cdev	cdev;
	dev_t		cdev_num;


	struct gpio_desc *gpiod_in_12v_status;
	int              irq_12v_status;

	struct gpio_desc *gpiod_in_valve_open;
	int              irq_valve;

	struct gpio_desc *gpiod_in_overpressure;
	int              irq_overpressure;

	struct gpio_desc *gpiod_in_stuckon;
	int              irq_stuckon;

	struct gpio_desc *gpiod_in_dealer_mode_sw;
	int              irq_dealer_sw;


	struct gpio_desc *gpiod_out_valve_ctrl;
	struct gpio_desc *gpiod_out_blower_ctrl;
	struct gpio_desc *gpiod_out_pwr_hold; // Unused.
	struct gpio_desc *gpiod_out_buzzer_low;
	struct gpio_desc *gpiod_out_buzzer_medium;
	struct gpio_desc *gpiod_out_buzzer_high;
	struct gpio_desc *gpiod_out_blower_stat;
	struct gpio_desc *gpiod_out_sbc_1min;
};

#define SPCD_DEVICE_NAME "spcd"
#define SPCD_CLASS "spcd-class"
struct class *spcd_class;

static DECLARE_WAIT_QUEUE_HEAD(spcd_rq);

static int spcd_set_state(struct spcd_data *spcd) {
	// TODO: Update the gpiod / pwm values based upon the state of the data struct.

	return 0;
};

static int spcd_read_state(struct spcd_data *spcd) {
	// TODO: Read input values into state member vars of the spcd_data struct.

	// gpiod_get_value is a memory access.

	return 0;
};

static irqreturn_t spcd_handle_irq(int irq, void *dev_id) {
	struct spcd_data *spcd = dev_id;

	spcd_read_state(spcd);
//	spcd->input_dirty = true;
	wave_up_interruptible(&spcd_rq);

	return IRQ_HANDLED;
}


// TODO: sysfs attributes.

static int spcd_open(struct inode *inode, struct file *filp) {
	unsigned int maj = imajor(inode);
	unsigned int min = iminor(inode);

	struct spcd_data *spcd_data = NULL;
	spcd_data = container_of(inode->i_cdev, struct wavecushion_data, cdev);

	if (min < 0) {
		pr_err("device not found\n");
		return -ENODEV;
	}

	filp->private_data = spcd_data;

	return 0;
}


static int spcd_release(struct inode *inode, struct file *filp) {
	struct spcd_data *spcd_data = NULL;
	spcd_data = container_of(inode->i_cdev, struct spcd_data, cdev);

	return 0;
}


ssize_t spcd_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos) {
	return 0;
}

ssize_t spcd_read(struct file *filp, char __user *buf, size_t count, loff_t *pos) {
	return 0;
}

static unsigned int spcd_poll(Struct file *file, poll_table *wait) {
	struct spcd_data *spcd_data = file->private_data;
	unsigned int reval_mask = 0;

	poll_wait(file, &spcd_rq, wait);

	if (spcd_data->input_dirty) {
		reval_mask |= (POLLIN | POLLRDNORM);
	}

	return reval_mask;
}

static const struct file_operations spcd_fops {
	.owner = THIS_MODULE,
	.write = spcd_write,
	.read = spcd_read,
	.open = spcd_open,
	.release = spcd_release,
	.poll = spcd_poll,
};


static int spcd_probe(struct platform_device *pdev) {
	struct device *dev = &pdev->dev;
	struct spcd_data *spcd_data;

	int ret = 0;

	pr_alert(" %s\n", __FUNCTION__);

	// Create the driver data
	spcd_data = kzalloc(sizeof(struct spcd_data), GFP_KERNEL);
	if (!spcd_data) {
		return -ENOMEM;
	}
	spcd_data->dev = dev;
	spcd_data->input_dirty = false;

	// PWM

	// Obtain GPIO from DT bindings.

	// Set initial GPIO states...

	platform_set_drvdata(pdev, spcd_data);

	// sync initial state
	spcd_set_state(spcd_data);
	spcd_read_state(spcd_data);

	// Associate attribute groups.

	// Character Device Support....
	alloc_chrdev_region(&(spcd_data->cdev_num), 0, 1, WC_DEVICE_NAME);
	spcd_class = class_create(THIS_MODULE, WC_CLASS);

	cdev_init(&(wavecushion_data->cdev), &spcd_fops);
	spcd_data->cdev.owner = THIS_MODULE;
	spcd_data->cdev_num = MKDEV(MAJOR(spcd_data->cdev_num), MINOR(spcd_data->cdev_num));
	cdev_add(&(spcd_data->cdev), spcd_data->dev_num, 1);

	// Create the device node /dev/spcd
	device_create(spcd_class,
	              NULL, // no parent
	              spcd_data->cdev_num,
	              NULL, // no additional data
	              WC_DEVICE_NAME "%d", 0;

	return ret;
};

static int spcd_remove(struct platform_device *pdev) {
	struct spcd_data *spcd_data = platform_get_drvdata(pdev);

	device_destroy(spcd_class, spcd_data->cdev_num);
	class_destroy(spcd_class);

//	sysfs_remove_group(&pdev->dev.kobj, &spcd_group);

	unregister_chrdev_region(spcd_data->cdev_num, 1);
	kfree(spcd_data);
	return 0;
};

static const struct of_device_id of_spcd_match[] = {
	{ .compatible = "avt,spcd", },
	{},
};

MODULE_DEVICE_TABLE(of, of_spcd_match);


static struct platform_drive spcd_driver = {
	.probe                = spcd_probe,
	.remove               = spcd_remove,
	.driver               = {
                .name = "spcd",
                .owner = THIS_MODULE,
                .of_match_table = of_spcd_match,
	},
};

module_platform_driver(spcd_driver);

MODULE_AUTHOR("bryan.varner@e-gineering.com");
MODULE_DESCRIPTION("AVT SPCD Platform Driver.");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:spcd");
