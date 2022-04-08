/*
    SPCD Kernel Driver
    Copyright (C) 2022 AVT, Inc.

    info@avtcare.com
    1125 N 13th St.
    Lafayette, IN 47904

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
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/pwm.h>
#include <linux/uaccess.h>

struct spcd_data {
	struct device		*dev;
	struct  gpio_desc	*gpio_in_12v_status;
	int			irq_12v_status;
};


// Top half IRQ Handler.
static irqreturn_t spcd_handle_irq(int irq, void *dev_id) {
	struct spcd_data *spcd = dev_id;

	// TODO: Read spcd sate.
	return IRQ_HANDLED;
};


static int spcd_probe(struct platform_device *pdev) {
	struct device *dev = &pdev->dev;
	struct spcd_data *spcd_data;

	int ret = 0;

	pr_alert(" %s\n", __FUNCTION__);

	// create the driver data....
	spcd_data = kzalloc(sizeof(struct spcd_data), GFP_KERNEL);
	if (!spcd_data) {
		return -ENOMEM;
	}
	spcd_data->dev = dev;

	// Get an input GPIO.
	spcd_data->gpio_in_12v_status = devm_gpiod_get(dev, "in_12v_status", GPIOD_IN);
	if (IS_ERR(spcd_data->gpio_in_12v_status)) {
		dev_err(dev, "failed to get in_12v_status-gpiod: err=%ld\n", PTR_ERR(spcd_data->gpio_in_12v_status));
		return PTR_ERR(spcd_data->gpio_in_12v_status);
	}
	// TODO: Request an IRQ for the input GPIO.
	spcd_data->irq_12v_status = gpiod_to_irq(spcd_data->gpio_in_12v_status);
	if (spcd_data->irq_12v_status < 0) {
		dev_err(dev, "failed to get IRQ for in_12v_status: err=%d\n", spcd_data->irq_12v_status);
		return spcd_data->irq_12v_status;
	};
	devm_request_irq(dev, spcd_data->irq_12v_status, spcd_handle_irq, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "spcd_12v_status", spcd_data);

	// TODO: Initial GPIO state tracking vars.


	platform_set_drvdata(pdev, spcd_data);

	// TODO sync state with a set / read.

	// Associate sysfs attribute groups.

	return ret;
};

static int spcd_remove(struct platform_device *pdev) {
	struct spcd_data *spcd_data = platform_get_drvdata(pdev);

	pr_alert(" %s\n", __FUNCTION__);

	kfree(spcd_data);

	return 0;
};

static struct platform_driver spcd_driver = {
	.probe = spcd_probe,
	.remove = spcd_remove,
	.driver = {
		.name = "spcd",
		.owner = THIS_MODULE,
	},
};
module_platform_driver(spcd_driver);

MODULE_AUTHOR("bryan.varner@e-gineering.com");
MODULE_DESCRIPTION("AVT SPCD Platform Driver.");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:spcd");
