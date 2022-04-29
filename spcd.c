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
#include <linux/uaccess.h>
#include <linux/hrtimer.h>

struct spcd_data {
	struct device		*dev;
	struct  gpio_desc	*gpio_in_12v_status;
	int			irq_12v_status;

	struct gpio_desc	*gpio_in_valve_open;
	int			irq_valve_open;

	struct gpio_desc	*gpio_in_overpressure;
	int			irq_overpressure;

	struct gpio_desc	*gpio_in_stuckon;
	int			irq_stuckon;

	struct gpio_desc	*gpio_in_mode;
	int			irq_mode;


	struct gpio_desc	*gpio_out_pwr_hold; // Unused
	struct gpio_desc	*gpio_out_wdt_alert;

	struct gpio_desc	*gpio_out_buzzer_low;
	struct gpio_desc	*gpio_out_buzzer_medium_0;
	struct gpio_desc	*gpio_out_buzzer_medium_1;
	struct gpio_desc	*gpio_out_buzzer_high_0;
	struct gpio_desc	*gpio_out_buzzer_high_1;
	struct gpio_desc	*gpio_out_buzzer_high_2;
	struct gpio_desc	*gpio_out_blower_stat;
	struct gpio_desc	*gpio_out_1min;
	struct gpio_desc	*gpio_out_blower_control;
	struct gpio_desc	*gpio_out_valve_control;

	ktime_t blower_period;
	u8	blower_duty;
	ktime_t blower_duty_on;
	ktime_t blower_duty_off;
	u8    blower_duty_state;

	struct hrtimer blower_timer;
};

enum hrtimer_restart blower_timer_callback(struct hrtimer *timer) {
	struct spcd_data *spcd = container_of(timer, struct spcd_data, blower_timer);

	// Swap from on to off.
	spcd->blower_duty_state = spcd->blower_duty_state ^ 1;
	gpiod_set_value(spcd->gpio_out_blower_control, spcd->blower_duty_state);

	// If the pins are off, wait for the time they should be off.
	// If the pins are on, wait for the time they should be on.
	hrtimer_forward_now(&spcd->blower_timer, spcd->blower_duty_state == 0 ? spcd->blower_duty_off : spcd->blower_duty_on);

	return HRTIMER_RESTART;
};


// Top half IRQ Handler.
static irqreturn_t spcd_handle_irq(int irq, void *dev_id) {
	// struct spcd_data *spcd = dev_id;

	// TODO: Read spcd sate.
	return IRQ_HANDLED;
};


static void spcd_timer_update(struct spcd_data *spcd) {
	pr_alert(" %s\n", __FUNCTION__);

	// Clear any pending timers.
	hrtimer_try_to_cancel(&spcd->blower_timer);

	// Set the pins low. (blower OFF)
	pr_alert("  setting blower_control 0\n");
	gpiod_set_value(spcd->gpio_out_blower_control, 0);
	pr_alert("  setting blower_stat 0\n");
	gpiod_set_value(spcd->gpio_out_blower_stat, 0);

	pr_alert(" blower off.\n");

	// Recalculate the blower duty.
	if (spcd->blower_period > 0 && spcd->blower_duty > 0) {
		pr_alert("   period and duty set.\n");

		spcd->blower_duty_on = ktime_set(0, ktime_to_ns(spcd->blower_period) / 100 * spcd->blower_duty);
		spcd->blower_duty_off = ktime_set(0, ktime_sub(spcd->blower_period, spcd->blower_duty_on));


		pr_alert("  on:%lld    off:%lld\n", ktime_to_ns(spcd->blower_duty_on), ktime_to_ns(spcd->blower_duty_off));

		// Set the pin states.
		// We're going to turn the blower on.
		gpiod_set_value(spcd->gpio_out_blower_stat, 1);
		pr_alert("  blower_stat on.\n");

		// We're going to pulse this pin.
		spcd->blower_duty_state = 1;
		gpiod_set_value(spcd->gpio_out_blower_control, spcd->blower_duty_state);
		pr_alert("  blower_control on.\n");

		// If the duty is 100, blower_duty_off will be 0, and we don't need the timer.
		if (ktime_to_ns(spcd->blower_duty_off) > 0) {
			pr_alert("  ENABLING BLOWER PWM TIMER\n");
			// Set a timer for the duty to be on.
			hrtimer_forward_now(&spcd->blower_timer, spcd->blower_duty_on);
			// fire the timer.
			hrtimer_restart(&spcd->blower_timer);
		} else {
			pr_alert(" FULL DUTY. NO TIMER\n");
		}
	} else {
		pr_alert("   period or duty zero. Leaving off.\n");
		spcd->blower_duty_on = ktime_set(0, 0);
		spcd->blower_duty_off = ktime_set(0, 0);
	}

	return;
};




// Expose sysfs attributes
static ssize_t spcd_blower_show_duty_cycle(struct device *dev, struct device_attribute *attr, char *buf) {
	struct spcd_data *spcd = dev_get_drvdata(dev);
	int len;

	len = sprintf(buf, "%d\n", spcd->blower_duty);
	if (len <= 0) {
		dev_err(dev, "spcd: Invalid sprintf len: %d\n", len);
	}
	return len;
}

static ssize_t spcd_blower_store_duty_cycle(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
	struct spcd_data *spcd = dev_get_drvdata(dev);

	kstrtou8(buf, 10, &spcd->blower_duty);

	spcd_timer_update(spcd);

	return count;
}

static DEVICE_ATTR(blower_duty_cycle, S_IRUGO | S_IWUSR, spcd_blower_show_duty_cycle, spcd_blower_store_duty_cycle);


static ssize_t spcd_blower_show_period(struct device *dev, struct device_attribute *attr, char *buf) {
	struct spcd_data *spcd = dev_get_drvdata(dev);
	int len;

	len = sprintf(buf, "%lld\n", ktime_to_ns(spcd->blower_period));
	if (len <= 0) {
		dev_err(dev, "spcd: Invalid sprintf len: %d\n", len);
	}
	return len;
}

static ssize_t spcd_blower_store_period(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
	struct spcd_data *spcd = dev_get_drvdata(dev);
	long periodnanos;

	kstrtol(buf, 10, &periodnanos);
	spcd->blower_period = ktime_set(0, periodnanos);

	spcd_timer_update(spcd);

	return count;
}

static DEVICE_ATTR(blower_period, S_IRUGO | S_IWUSR, spcd_blower_show_period, spcd_blower_store_period);





static struct attribute *spcd_attrs[] = {
	&dev_attr_blower_duty_cycle.attr,
	&dev_attr_blower_period.attr,

	NULL
};
ATTRIBUTE_GROUPS(spcd);









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

	// Setup Input GPIOS and IRQs
	spcd_data->gpio_in_12v_status = devm_gpiod_get(dev, "in-12v-status", GPIOD_IN);
	if (IS_ERR(spcd_data->gpio_in_12v_status)) {
		dev_err(dev, "failed to get in-12v-status-gpio: err=%ld\n", PTR_ERR(spcd_data->gpio_in_12v_status));
		return PTR_ERR(spcd_data->gpio_in_12v_status);
	}
	spcd_data->irq_12v_status = gpiod_to_irq(spcd_data->gpio_in_12v_status);
	if (spcd_data->irq_12v_status < 0) {
		dev_err(dev, "failed to get IRQ for in_12v_status: err=%d\n", spcd_data->irq_12v_status);
		return spcd_data->irq_12v_status;
	}
	devm_request_irq(dev, spcd_data->irq_12v_status, spcd_handle_irq, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "spcd_12v_status", spcd_data);


	spcd_data->gpio_in_valve_open = devm_gpiod_get(dev, "in-valve-open", GPIOD_IN);
	if (IS_ERR(spcd_data->gpio_in_valve_open)) {
		dev_err(dev, "failed to get in-valve-open-gpio: err=%ld\n", PTR_ERR(spcd_data->gpio_in_valve_open));
		return PTR_ERR(spcd_data->gpio_in_valve_open);
	}
	spcd_data->irq_valve_open = gpiod_to_irq(spcd_data->gpio_in_valve_open);
	if (spcd_data->irq_valve_open < 0) {
		dev_err(dev, "failed to get IRQ for in_valve_open: err=%d\n", spcd_data->irq_valve_open);
		return spcd_data->irq_valve_open;
	}
	devm_request_irq(dev, spcd_data->irq_valve_open, spcd_handle_irq, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "spcd_valve_open", spcd_data);


	spcd_data->gpio_in_overpressure = devm_gpiod_get(dev, "in-overpressure", GPIOD_IN);
	if (IS_ERR(spcd_data->gpio_in_overpressure)) {
		dev_err(dev, "failed to get in-overpressure-gpio: err=%ld\n", PTR_ERR(spcd_data->gpio_in_overpressure));
		return PTR_ERR(spcd_data->gpio_in_overpressure);
	}
	spcd_data->irq_overpressure = gpiod_to_irq(spcd_data->gpio_in_overpressure);
	if (spcd_data->irq_overpressure < 0) {
		dev_err(dev, "failed to get IRQ for in_overpressure: err=%d\n", spcd_data->irq_overpressure);
		return spcd_data->irq_overpressure;
	}
	devm_request_irq(dev, spcd_data->irq_overpressure, spcd_handle_irq, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "spcd_overpressure", spcd_data);


	spcd_data->gpio_in_stuckon = devm_gpiod_get(dev, "in-stuckon", GPIOD_IN);
	if (IS_ERR(spcd_data->gpio_in_stuckon)) {
		dev_err(dev, "failed to get in-stuckon-gpio: err=%ld\n", PTR_ERR(spcd_data->gpio_in_stuckon));
		return PTR_ERR(spcd_data->gpio_in_stuckon);
	}
	spcd_data->irq_stuckon = gpiod_to_irq(spcd_data->gpio_in_stuckon);
	if (spcd_data->irq_stuckon < 0) {
		dev_err(dev, "failed to get IRQ for in_stuckon: err=%d\n", spcd_data->irq_stuckon);
		return spcd_data->irq_stuckon;
	}
	devm_request_irq(dev, spcd_data->irq_stuckon, spcd_handle_irq, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "spcd_stuckon", spcd_data);


	spcd_data->gpio_in_mode = devm_gpiod_get(dev, "in-mode", GPIOD_IN);
	if (IS_ERR(spcd_data->gpio_in_mode)) {
		dev_err(dev, "failed to get in-mode-gpio: err=%ld\n", PTR_ERR(spcd_data->gpio_in_mode));
		return PTR_ERR(spcd_data->gpio_in_mode);
	}
	spcd_data->irq_mode = gpiod_to_irq(spcd_data->gpio_in_mode);
	if (spcd_data->irq_mode < 0) {
		dev_err(dev, "failed to get IRQ for in_mode: err=%d\n", spcd_data->irq_mode);
		return spcd_data->irq_mode;
	}
	devm_request_irq(dev, spcd_data->irq_mode, spcd_handle_irq, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "spcd_mode", spcd_data);



	// Push-pull outputs.
	spcd_data->gpio_out_pwr_hold = devm_gpiod_get(dev, "out-pwr-hold", GPIOD_OUT_LOW);
	if (IS_ERR(spcd_data->gpio_out_pwr_hold)) {
		dev_err(dev, "failed to get out-pwr-hold-gpio: err=%ld\n", PTR_ERR(spcd_data->gpio_out_pwr_hold));
		return PTR_ERR(spcd_data->gpio_out_pwr_hold);
	}

	spcd_data->gpio_out_wdt_alert = devm_gpiod_get(dev, "out-wdt-alert", GPIOD_OUT_LOW);
	if (IS_ERR(spcd_data->gpio_out_wdt_alert)) {
		dev_err(dev, "failed to get out-wdt-alert-gpio: err=%ld\n", PTR_ERR(spcd_data->gpio_out_wdt_alert));
		return PTR_ERR(spcd_data->gpio_out_wdt_alert);
	}

	spcd_data->gpio_out_blower_control = devm_gpiod_get(dev, "out-blower-control", GPIOD_OUT_LOW);
	if (IS_ERR(spcd_data->gpio_out_blower_control)) {
		dev_err(dev, "failed to get out-blower-control-gpio: err=%ld\n", PTR_ERR(spcd_data->gpio_out_blower_control));
		return PTR_ERR(spcd_data->gpio_out_blower_control);
	}

	spcd_data->gpio_out_valve_control = devm_gpiod_get(dev, "out-valve-control", GPIOD_OUT_LOW);
	if (IS_ERR(spcd_data->gpio_out_valve_control)) {
		dev_err(dev, "failed to get out-valve-control-gpio: err=%ld\n", PTR_ERR(spcd_data->gpio_out_valve_control));
		return PTR_ERR(spcd_data->gpio_out_valve_control);
	}



	// All outputs on the I2C expander are open-drain.
	spcd_data->gpio_out_buzzer_low = devm_gpiod_get(dev, "out-buzzer-low", GPIOD_OUT_HIGH_OPEN_DRAIN);
	if (IS_ERR(spcd_data->gpio_out_buzzer_low)) {
		dev_err(dev, "failed to get out-buzzer-low-gpio: err=%ld\n", PTR_ERR(spcd_data->gpio_out_buzzer_low));
		return PTR_ERR(spcd_data->gpio_out_buzzer_low);
	}
	spcd_data->gpio_out_buzzer_medium_0 = devm_gpiod_get_index(dev, "out-buzzer-medium", 0, GPIOD_OUT_HIGH_OPEN_DRAIN);
	if (IS_ERR(spcd_data->gpio_out_buzzer_medium_0)) {
		dev_err(dev, "failed to get out-buzzer-medium-gpios[0]: err=%ld\n", PTR_ERR(spcd_data->gpio_out_buzzer_medium_0));
		return PTR_ERR(spcd_data->gpio_out_buzzer_medium_0);
	}
	spcd_data->gpio_out_buzzer_medium_1 = devm_gpiod_get_index(dev, "out-buzzer-medium", 1, GPIOD_OUT_HIGH_OPEN_DRAIN);
	if (IS_ERR(spcd_data->gpio_out_buzzer_medium_1)) {
		dev_err(dev, "failed to get out-buzzer-medium-gpios[1]: err=%ld\n", PTR_ERR(spcd_data->gpio_out_buzzer_medium_1));
		return PTR_ERR(spcd_data->gpio_out_buzzer_medium_1);
	}
	spcd_data->gpio_out_buzzer_high_0 = devm_gpiod_get_index(dev, "out-buzzer-high", 0, GPIOD_OUT_HIGH_OPEN_DRAIN);
	if (IS_ERR(spcd_data->gpio_out_buzzer_high_0)) {
		dev_err(dev, "failed to get out-buzzer-high-gpios[0]: err=%ld\n", PTR_ERR(spcd_data->gpio_out_buzzer_high_0));
		return PTR_ERR(spcd_data->gpio_out_buzzer_high_0);
	}
	spcd_data->gpio_out_buzzer_high_1 = devm_gpiod_get_index(dev, "out-buzzer-high", 1, GPIOD_OUT_HIGH_OPEN_DRAIN);
	if (IS_ERR(spcd_data->gpio_out_buzzer_high_1)) {
		dev_err(dev, "failed to get out-buzzer-high-gpios[1]: err=%ld\n", PTR_ERR(spcd_data->gpio_out_buzzer_high_1));
		return PTR_ERR(spcd_data->gpio_out_buzzer_high_1);
	}
	spcd_data->gpio_out_buzzer_high_2 = devm_gpiod_get_index(dev, "out-buzzer-high", 2, GPIOD_OUT_HIGH_OPEN_DRAIN);
	if (IS_ERR(spcd_data->gpio_out_buzzer_high_2)) {
		dev_err(dev, "failed to get out-buzzer-high-gpios[2]: err=%ld\n", PTR_ERR(spcd_data->gpio_out_buzzer_high_2));
		return PTR_ERR(spcd_data->gpio_out_buzzer_high_2);
	}
	spcd_data->gpio_out_blower_stat = devm_gpiod_get(dev, "out-blower-stat", GPIOD_OUT_LOW_OPEN_DRAIN);
	if (IS_ERR(spcd_data->gpio_out_blower_stat)) {
		dev_err(dev, "failed to get out-blower-stat-gpio: err=%ld\n", PTR_ERR(spcd_data->gpio_out_blower_stat));
		return PTR_ERR(spcd_data->gpio_out_blower_stat);
	}
	spcd_data->gpio_out_1min = devm_gpiod_get(dev, "out-1min", GPIOD_OUT_LOW_OPEN_DRAIN);
	if (IS_ERR(spcd_data->gpio_out_1min)) {
		dev_err(dev, "failed to get out-1min-gpio: err=%ld\n", PTR_ERR(spcd_data->gpio_out_1min));
		return PTR_ERR(spcd_data->gpio_out_1min);
	}

	spcd_data->blower_duty = 0;
	spcd_data->blower_period = ktime_set(0, 0);
	spcd_data->blower_duty_state = 0;

	// TODO: Initial GPIO state tracking vars.


	platform_set_drvdata(pdev, spcd_data);

	// Setup softPWM hardware timers.
	hrtimer_init( &spcd_data->blower_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_PINNED_HARD);
	spcd_data->blower_timer.function = blower_timer_callback;
	spcd_timer_update(spcd_data);


	// TODO sync state with a set / read.


	// Associate sysfs attribute groups.
	ret = sysfs_create_group(&pdev->dev.kobj, &spcd_group);
	if (ret) {
		dev_err(dev, "sysfs creation failed\n");
		return ret;
	}

	return ret;
};

static int spcd_remove(struct platform_device *pdev) {
	struct spcd_data *spcd_data = platform_get_drvdata(pdev);

	pr_alert(" %s\n", __FUNCTION__);

	kfree(spcd_data);

	return 0;
};


static const struct of_device_id of_spcd_match[] = {
        { .compatible = "avt,spcd", },
        {},
};

MODULE_DEVICE_TABLE(of, of_spcd_match);

static struct platform_driver spcd_driver = {
	.probe = spcd_probe,
	.remove = spcd_remove,
	.driver = {
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
