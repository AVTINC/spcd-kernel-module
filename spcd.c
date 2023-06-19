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
// Added to support sysfs_emit, being pulled in from newer kernel releases.
#include <linux/mm.h>
#include <linux/bug.h>
#include <linux/slab.h>

struct spcd_data {
    struct device *dev;
    struct gpio_desc *gpio_in_12v_status;
    int irq_12v_status;

    struct gpio_desc *gpio_in_valve_open;
    int irq_valve_open;
    int irq_error_log;

    struct gpio_desc *gpio_in_overpressure;
    int irq_overpressure;

    struct gpio_desc *gpio_in_stuckon;
    int irq_stuckon;

    struct gpio_desc *gpio_in_mode;
    int irq_mode;


    struct gpio_desc *gpio_out_pwr_hold; // Unused
    struct gpio_desc *gpio_out_wdt_alert;

    struct gpio_desc *gpio_out_buzzer_low;
    struct gpio_descs *gpio_out_buzzer_medium;
    struct gpio_descs *gpio_out_buzzer_high;
    struct gpio_desc *gpio_out_blower_stat;
    struct gpio_desc *gpio_out_1min;
    struct gpio_desc *gpio_out_blower_control;
    struct gpio_desc *gpio_out_valve_control;


    ktime_t blower_period;
    ktime_t blower_duty_on;
    ktime_t blower_duty_off;
    u8 blower_duty_state;
    struct hrtimer blower_timer;


    ktime_t valve_period;
    ktime_t valve_duty_on;
    ktime_t valve_duty_off;
    u8 valve_duty_state;
    struct hrtimer valve_timer;
};


// Shamelessly cribbed from newer kernel versions (5.17.5)
// Having this here, greatly reduces our repetition.
// https://elixir.bootlin.com/linux/v5.17.5/C/ident/sysfs_emit
int sysfs_emit(char *buf, const char *fmt, ...) {
    va_list args;
    int len;

    if (WARN(!buf || offset_in_page(buf), "invalid sysfs_emit: buf:%p\n", buf)) {
        return 0;
    }

    va_start(args, fmt);
    len = vscnprintf(buf, PAGE_SIZE, fmt, args);
    va_end(args);

    return len;
}



// Blower PWM Callback
enum hrtimer_restart blower_timer_callback(struct hrtimer *timer) {
    struct spcd_data *spcd = container_of(timer, struct spcd_data, blower_timer);

    // Swap from on to off.
    spcd->blower_duty_state = spcd->blower_duty_state ^ 1;
    gpiod_set_value(spcd->gpio_out_blower_control, spcd->blower_duty_state);

    // If the pins are off, wait for the time they should be off.
    // If the pins are on, wait for the time they should be on.
    hrtimer_forward_now(&spcd->blower_timer, spcd->blower_duty_state == 0 ? spcd->blower_duty_off : spcd->blower_duty_on);

    return HRTIMER_RESTART;
}


// Valve PWM Callback
enum hrtimer_restart valve_timer_callback(struct hrtimer *timer) {
    struct spcd_data *spcd = container_of(timer, struct spcd_data, valve_timer);

    spcd->valve_duty_state = spcd->valve_duty_state ^ 1;
    gpiod_set_value(spcd->gpio_out_valve_control, spcd->valve_duty_state);

    hrtimer_forward_now(&spcd->valve_timer, spcd->valve_duty_state == 0 ? spcd->valve_duty_off : spcd->valve_duty_on);

    return HRTIMER_RESTART;
}


static void spcd_blower_timer_update(struct spcd_data *spcd) {
    pr_debug(" %s\n", __FUNCTION__);

    // Clear any pending timers.
    hrtimer_try_to_cancel(&spcd->blower_timer);

    // Set the pins low. (blower OFF)
    // TODO: Check if we can safely move this to the else block handling zero-ing out timers. If so, we can eliminate a 'hiccup'.
    gpiod_set_value(spcd->gpio_out_blower_control, 0);
    gpiod_set_value_cansleep(spcd->gpio_out_blower_stat, 0);

    // Recalculate the blower duty.
    if (spcd->blower_period > 0 && spcd->blower_duty_on > 0) {
        pr_debug("   period and duty set.\n");

        spcd->blower_duty_off = ktime_set(0, ktime_sub(spcd->blower_period, spcd->blower_duty_on));

        pr_debug("  on:%lld    off:%lld\n", ktime_to_ns(spcd->blower_duty_on), ktime_to_ns(spcd->blower_duty_off));

        // Set the pin states.
        // We're going to turn the blower on.
	/*JP Modified this so we can troubleshoot what might be an issue 
		with the check for flipping every 30 seconds*/
        gpiod_set_value_cansleep(spcd->gpio_out_blower_stat, 0);
        pr_debug("  blower_stat on.\n");

        // We're going to pulse this pin.
        spcd->blower_duty_state = 1;
        gpiod_set_value(spcd->gpio_out_blower_control, spcd->blower_duty_state);
        pr_debug("  blower_control on.\n");

        // If the duty is 100, blower_duty_off will be 0, and we don't need the timer.
        if (ktime_to_ns(spcd->blower_duty_off) > 0) {
            pr_debug("  ENABLING BLOWER PWM TIMER\n");
            // Set a timer for the duty to be on.
            hrtimer_forward_now(&spcd->blower_timer, spcd->blower_duty_on);
            // fire the timer.
            hrtimer_restart(&spcd->blower_timer);
        } else {
            pr_debug(" FULL DUTY. NO TIMER\n");
        }
    } else {
        pr_debug("   period or duty zero. Leaving off.\n");
        spcd->blower_duty_on = ktime_set(0, 0);
        spcd->blower_duty_off = ktime_set(0, 0);
    }

    return;
}


static void spcd_valve_timer_update(struct spcd_data *spcd) {
    pr_debug(" %s\n", __FUNCTION__);

    // Clear any pending timers.
    hrtimer_try_to_cancel(&spcd->valve_timer);

    // Set the pins low. (valve OFF)
    gpiod_set_value(spcd->gpio_out_valve_control, 0);

    // Recalculate the valve duty.
    if (spcd->valve_period > 0 && spcd->valve_duty_on > 0) {
        pr_debug("   valve period and duty set.\n");

        spcd->valve_duty_off = ktime_set(0, ktime_sub(spcd->valve_period, spcd->valve_duty_on));

        pr_debug("  on:%lld    off:%lld\n", ktime_to_ns(spcd->valve_duty_on), ktime_to_ns(spcd->valve_duty_off));

        // Set the pin states.
        // We're going to pulse this pin.
        spcd->valve_duty_state = 1;
        gpiod_set_value(spcd->gpio_out_valve_control, spcd->valve_duty_state);
        pr_debug("  valve_control on.\n");

        // If the duty is 100, valve_duty_off will be 0, and we don't need the timer.
        if (ktime_to_ns(spcd->valve_duty_off) > 0) {
            pr_debug("  ENABLING VALVE PWM TIMER\n");
            // Set a timer for the duty to be on.
            hrtimer_forward_now(&spcd->valve_timer, spcd->valve_duty_on);
            // fire the timer.
            hrtimer_restart(&spcd->valve_timer);
        } else {
            pr_debug(" FULL DUTY. NO TIMER\n");
        }
    } else {
        pr_debug("   period or duty zero. Leaving off.\n");
        spcd->valve_duty_on = ktime_set(0, 0);
        spcd->valve_duty_off = ktime_set(0, 0);
    }

    return;
}


/* -- sysfs attributes -- PWM -- */
static ssize_t blower_duty_cycle_show(struct device *dev, struct device_attribute *attr, char *buf) {
    struct spcd_data *spcd = dev_get_drvdata(dev);

    return sysfs_emit(buf, "%lld\n", ktime_to_ns(spcd->blower_duty_on));
}

static ssize_t blower_duty_cycle_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
    struct spcd_data *spcd = dev_get_drvdata(dev);
    long dutyonnanos;
    int err;

    err = kstrtol(buf, 10, &dutyonnanos);
    if (err) {
        return err;
    }

    spcd->blower_duty_on = ktime_set(0, dutyonnanos);
    spcd_blower_timer_update(spcd);

    return count;
}

static DEVICE_ATTR_RW(blower_duty_cycle);

static ssize_t blower_period_show(struct device *dev, struct device_attribute *attr, char *buf) {
    struct spcd_data *spcd = dev_get_drvdata(dev);

    return sysfs_emit(buf, "%lld\n", ktime_to_ns(spcd->blower_period));
}

static ssize_t blower_period_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
    struct spcd_data *spcd = dev_get_drvdata(dev);
    long periodnanos;
    int err;

    err = kstrtol(buf, 10, &periodnanos);
    if (err) {
        return err;
    }

    spcd->blower_period = ktime_set(0, periodnanos);
    spcd_blower_timer_update(spcd);

    return count;
}

static DEVICE_ATTR_RW(blower_period);



static ssize_t valve_duty_cycle_show(struct device *dev, struct device_attribute *attr, char *buf) {
    struct spcd_data *spcd = dev_get_drvdata(dev);

    return sysfs_emit(buf, "%lld\n", ktime_to_ns(spcd->valve_duty_on));
}

static ssize_t valve_duty_cycle_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
    struct spcd_data *spcd = dev_get_drvdata(dev);
    long dutynanos;
    int err;

    err = kstrtol(buf, 10, &dutynanos);
    if (err) {
            return err;
    }

    spcd->valve_duty_on = ktime_set(0, dutynanos);
    spcd_valve_timer_update(spcd);

    return count;
}

static DEVICE_ATTR_RW(valve_duty_cycle);



static ssize_t valve_period_show(struct device *dev, struct device_attribute *attr, char *buf) {
    struct spcd_data *spcd = dev_get_drvdata(dev);

    return sysfs_emit(buf, "%lld\n", ktime_to_ns(spcd->valve_period));
}

static ssize_t valve_period_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
    struct spcd_data *spcd = dev_get_drvdata(dev);
    long periodnanos;
    int err;

    err = kstrtol(buf, 10, &periodnanos);
    if (err) {
        return err;
    }

    spcd->valve_period = ktime_set(0, periodnanos);
    spcd_valve_timer_update(spcd);

    return count;
}

static DEVICE_ATTR_RW(valve_period);


/* sysfs Attributes -- INPUTS */
static ssize_t status_12v_show(struct device *dev, struct device_attribute *attr, char *buf) {
    struct spcd_data *spcd = dev_get_drvdata(dev);

    int val = gpiod_get_value(spcd->gpio_in_12v_status);
    if (val < 0) {
        return val;
    }

    return sysfs_emit(buf, "%d\n", val);
}
static DEVICE_ATTR_RO(status_12v);

static ssize_t valve_open_show(struct device *dev, struct device_attribute *attr, char *buf) {
    struct spcd_data *spcd = dev_get_drvdata(dev);

    int val = gpiod_get_value_cansleep(spcd->gpio_in_valve_open);
    if (val < 0) {
        return val;
    }

    return sysfs_emit(buf, "%d\n", val);
}
static DEVICE_ATTR_RO(valve_open);

static ssize_t error_log_show(struct device *dev, struct device_attribute *attr, char *buf) {
    struct spcd_data *spcd = dev_get_drvdata(dev);

    int val = 13;
    if (val < 0) {
        return val;
    }

    return sysfs_emit(buf, "%d\n", val);
}
static DEVICE_ATTR_RO(error_log);



static ssize_t overpressure_show(struct device *dev, struct device_attribute *attr, char *buf) {
    struct spcd_data *spcd = dev_get_drvdata(dev);

    int val = gpiod_get_value_cansleep(spcd->gpio_in_overpressure);
    if (val < 0) {
        return val;
    }

    return sysfs_emit(buf, "%d\n", val);
}
static DEVICE_ATTR_RO(overpressure);



static ssize_t stuck_on_show(struct device *dev, struct device_attribute *attr, char *buf) {
    struct spcd_data *spcd = dev_get_drvdata(dev);

    int val = gpiod_get_value_cansleep(spcd->gpio_in_stuckon);
    if (val < 0) {
        return val;
    }

    return sysfs_emit(buf, "%d\n", val);
}
static DEVICE_ATTR_RO(stuck_on);


/* sfs attributes -- OUTPUTS */
static ssize_t mode_switch_show(struct device *dev, struct device_attribute *attr, char *buf) {
    struct spcd_data *spcd = dev_get_drvdata(dev);

    int val = gpiod_get_value_cansleep(spcd->gpio_in_mode);
    if (val < 0) {
        return val;
    }

    return sysfs_emit(buf, "%d\n", val);
}
static DEVICE_ATTR_RO(mode_switch);

static ssize_t pwr_hold_show(struct device *dev, struct device_attribute *attr, char *buf) {
    struct spcd_data *spcd = dev_get_drvdata(dev);

    int val = gpiod_get_value(spcd->gpio_out_pwr_hold);
    if (val < 0) {
        return val;
    }

    return sysfs_emit(buf, "%d\n", val);
}

static ssize_t pwr_hold_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
    struct spcd_data *spcd = dev_get_drvdata(dev);
    int val;
    int err;

    err = kstrtoint(buf, 10, &val);
    if (err) {
        return err;
    }

    gpiod_set_value(spcd->gpio_out_pwr_hold, val);

    return count;
}
static DEVICE_ATTR_RW(pwr_hold);



static ssize_t wdt_alert_show(struct device *dev, struct device_attribute *attr, char *buf) {
    struct spcd_data *spcd = dev_get_drvdata(dev);

    int val = gpiod_get_value(spcd->gpio_out_wdt_alert);
    if (val < 0) {
        return val;
    }

    return sysfs_emit(buf, "%d\n", val);
}

static ssize_t wdt_alert_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
    struct spcd_data *spcd = dev_get_drvdata(dev);
    int val;
    int err;

    err = kstrtoint(buf, 10, &val);
    if (err) {
        return err;
    }

    gpiod_set_value(spcd->gpio_out_wdt_alert, val);

    return count;
}
static DEVICE_ATTR_RW(wdt_alert);



static ssize_t one_min_show(struct device *dev, struct device_attribute *attr, char *buf) {
    struct spcd_data *spcd = dev_get_drvdata(dev);

    int val = gpiod_get_value_cansleep(spcd->gpio_out_1min);
    if (val < 0) {
        return val;
    }

    return sysfs_emit(buf, "%d\n", val);
}

static ssize_t one_min_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
    struct spcd_data *spcd = dev_get_drvdata(dev);
    int val;
    int err;

    err = kstrtoint(buf, 10, &val);
    if (err) {
        return err;
    }

    gpiod_set_value_cansleep(spcd->gpio_out_1min, val);

    return count;
}
static DEVICE_ATTR_RW(one_min);


static ssize_t buzzer_show(struct device *dev, struct device_attribute *attr, char *buf) {
    struct spcd_data *spcd = dev_get_drvdata(dev);

    unsigned long *values;
    unsigned long *zeros;
    int val;


    pr_debug("  check low\n");
    val = gpiod_get_value_cansleep(spcd->gpio_out_buzzer_low);
    if (val == 0) { // Not low, check medium.
        values = bitmap_alloc(3, GFP_KERNEL);
        if (!values) {
            return -ENOMEM;
        }
        bitmap_zero(values, 3);

        zeros = bitmap_alloc(3, GFP_KERNEL);
        if (!zeros) {
            return -ENOMEM;
        }
        bitmap_zero(zeros, 3);

        pr_debug("   check medium\n");
        val = gpiod_get_array_value_cansleep(2, spcd->gpio_out_buzzer_medium->desc, spcd->gpio_out_buzzer_medium->info, values);
        if (val >= 0 && bitmap_equal(values, zeros, 2) == 0) {
            val = 2;
        }

        if (val == 0) { // Not medium. Check high.
            pr_debug("    check high\n");
            val = gpiod_get_array_value_cansleep(3, spcd->gpio_out_buzzer_high->desc, spcd->gpio_out_buzzer_high->info, values);
            if (val >= 0 && bitmap_equal(values, zeros, 3) == 0) {
                val = 3;
            }
        }

        bitmap_free(values);
        bitmap_free(zeros);
    }

    if (val < 0) {
        return val;
    }

    return sysfs_emit(buf, "%d\n", val);
}

static ssize_t buzzer_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
    struct spcd_data *spcd = dev_get_drvdata(dev);

    unsigned long *values;
    int val;
    int err;

    // Parse the mode value.
    err = kstrtoint(buf, 10, &val);
    if (err) {
        return err;
    }
    if (val < 0 || val > 3) {
        return -EINVAL;
    }

    // Allocate a bitmap of appropriate size
    values = bitmap_alloc(3, GFP_KERNEL);
    if (!values) {
        return -ENOMEM;
    }
    bitmap_zero(values, 3);

    // Turn off everything
    gpiod_set_value_cansleep(spcd->gpio_out_buzzer_low, 0);
    gpiod_set_array_value_cansleep(2, spcd->gpio_out_buzzer_medium->desc, spcd->gpio_out_buzzer_medium->info, values);
    gpiod_set_array_value_cansleep(3, spcd->gpio_out_buzzer_high->desc, spcd->gpio_out_buzzer_high->info, values);

    if (val == 1) {
        gpiod_set_value_cansleep(spcd->gpio_out_buzzer_low, 1);
    } else if (val == 2) {
        pr_debug("set medium\n");
        pr_debug(" before fill=%ld\n", values);
        bitmap_fill(values, 2);
        pr_debug(" after fill=%ld\n", values);
        gpiod_set_array_value_cansleep(2, spcd->gpio_out_buzzer_medium->desc, spcd->gpio_out_buzzer_medium->info, values);
    } else if (val == 3) {
        pr_debug("set high\n");
        bitmap_fill(values, 3);
        gpiod_set_array_value_cansleep(3, spcd->gpio_out_buzzer_high->desc, spcd->gpio_out_buzzer_high->info, values);
    }

    bitmap_free(values);

    return count;
}
static DEVICE_ATTR_RW(buzzer);





static struct attribute *spcd_attrs[] = {
        &dev_attr_blower_duty_cycle.attr,
        &dev_attr_blower_period.attr,
        &dev_attr_valve_duty_cycle.attr,
        &dev_attr_valve_period.attr,
        &dev_attr_status_12v.attr,
        &dev_attr_valve_open.attr,
        &dev_attr_overpressure.attr,
        &dev_attr_stuck_on.attr,
        &dev_attr_mode_switch.attr,
        &dev_attr_pwr_hold.attr,
        &dev_attr_wdt_alert.attr,
        &dev_attr_one_min.attr,
        &dev_attr_buzzer.attr,
	&dev_attr_error_log.attr,

        NULL
};
ATTRIBUTE_GROUPS(spcd);


/* -- Interrupt Handlers -- */
/*
 * All interrupt handlers are treated as bottom half threaded interrupts.
 * This is due to the use of the max7313 i2C GPIO expander, which allows
 * gpio lines (reads / writes) to 'sleep'. Since these are not memory-mapped
 * output lines, it takes some overhead for the i2c communications.
 *
 * Moving to bottom-half IRQs shouldn't greatly impact things.
 */
static irqreturn_t spcd_handle_12v_status_irq(int irq, void *dev_id) {
    // struct spcd_data *spcd = dev_id;
    pr_debug(" %s\n", __FUNCTION__);
    return IRQ_HANDLED;
}

static irqreturn_t spcd_handle_valve_irq(int irq, void *dev_id) {
    // struct spcd_data *spcd = dev_id;
    pr_debug(" %s\n", __FUNCTION__);
    return IRQ_HANDLED;
}

static irqreturn_t spcd_handle_overpressure_irq(int irq, void *dev_id) {
    // struct spcd_data *spcd = dev_id;
    pr_debug(" %s\n", __FUNCTION__);
    return IRQ_HANDLED;
}

static irqreturn_t spcd_handle_stuckon_irq(int irq, void *dev_id) {
    // struct spcd_data *spcd = dev_id;
    pr_debug(" %s\n", __FUNCTION__);
    return IRQ_HANDLED;
}

static irqreturn_t spcd_handle_mode_irq(int irq, void *dev_id) {
    // struct spcd_data *spcd = dev_id;
    pr_debug(" %s\n", __FUNCTION__);
    return IRQ_HANDLED;
}







static int spcd_probe(struct platform_device *pdev) {
    struct device *dev = &pdev->dev;
    struct spcd_data *spcd_data;

    int ret = 0;

    pr_debug(" %s\n", __FUNCTION__);

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
    spcd_data->gpio_out_buzzer_low = devm_gpiod_get(dev, "out-buzzer-low", GPIOD_OUT_LOW_OPEN_DRAIN);
    if (IS_ERR(spcd_data->gpio_out_buzzer_low)) {
        dev_err(dev, "failed to get out-buzzer-low-gpio: err=%ld\n", PTR_ERR(spcd_data->gpio_out_buzzer_low));
        return PTR_ERR(spcd_data->gpio_out_buzzer_low);
    }
    spcd_data->gpio_out_buzzer_medium = devm_gpiod_get_array(dev, "out-buzzer-medium", GPIOD_OUT_LOW_OPEN_DRAIN);
    if (IS_ERR(spcd_data->gpio_out_buzzer_medium)) {
        dev_err(dev, "failed to get out-buzzer-medium-gpios: err=%ld\n", PTR_ERR(spcd_data->gpio_out_buzzer_medium));
        return PTR_ERR(spcd_data->gpio_out_buzzer_medium);
    }
    spcd_data->gpio_out_buzzer_high = devm_gpiod_get_array(dev, "out-buzzer-high", GPIOD_OUT_LOW_OPEN_DRAIN);
    if (IS_ERR(spcd_data->gpio_out_buzzer_high)) {
        dev_err(dev, "failed to get out-buzzer-high-gpio: err=%ld\n", PTR_ERR(spcd_data->gpio_out_buzzer_high));
        return PTR_ERR(spcd_data->gpio_out_buzzer_high);
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

    spcd_data->blower_period = ktime_set(0, 0);
    spcd_data->blower_duty_on = ktime_set(0, 0);
    spcd_data->blower_duty_off = ktime_set(0, 0);
    spcd_data->blower_duty_state = 0;

    spcd_data->valve_period = ktime_set(0, 0);
    spcd_data->valve_duty_on = ktime_set(0, 0);
    spcd_data->valve_duty_off = ktime_set(0, 0);
    spcd_data->valve_duty_state = 0;

    // TODO: Initial GPIO state tracking vars.

    // Setup softPWM hardware timers.
    hrtimer_init(&spcd_data->blower_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_PINNED_HARD);
    spcd_data->blower_timer.function = blower_timer_callback;

    hrtimer_init(&spcd_data->valve_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_PINNED_HARD);
    spcd_data->valve_timer.function = valve_timer_callback;


    platform_set_drvdata(pdev, spcd_data);

    // TODO sync state with a set / read.
    spcd_blower_timer_update(spcd_data);
    spcd_valve_timer_update(spcd_data);


    // Associate sysfs attribute groups.
    ret = sysfs_create_groups(&pdev->dev.kobj, spcd_groups);
    if (ret) {
        dev_err(dev, "sysfs creation failed\n");
        return ret;
    }

    // Now that everything is setup and initialized, request IRQs and assign handlers.
    ret = devm_request_threaded_irq(dev, spcd_data->irq_12v_status, NULL, spcd_handle_12v_status_irq, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "spcd_12v_status", spcd_data);
    if (ret == -ENOSYS) {
        return -EPROBE_DEFER;
    }
    if (ret) {
        dev_err(&pdev->dev, "couldn't request irq %s %d: %d\n", "irq_12v_status", spcd_data->irq_12v_status, ret);
        return ret;
    }

    ret = devm_request_threaded_irq(dev, spcd_data->irq_valve_open, NULL, spcd_handle_valve_irq, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "spcd_valve_open", spcd_data);
    if (ret == -ENOSYS) {
        return -EPROBE_DEFER;
    }
    if (ret) {
        dev_err(&pdev->dev, "couldn't request irq %s %d: %d\n", "irq_valve_open", spcd_data->irq_valve_open, ret);
        return ret;
    }
    ret = devm_request_threaded_irq(dev, spcd_data->irq_error_log, NULL, spcd_handle_valve_irq, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "spcd_error_log", spcd_data);
    if (ret == -ENOSYS) {
        return -EPROBE_DEFER;
    }
    if (ret) {
        dev_err(&pdev->dev, "couldn't request irq %s %d: %d\n", "irq_error_log", spcd_data->irq_error_log, ret);
        return ret;
    }

    ret = devm_request_threaded_irq(dev, spcd_data->irq_overpressure, NULL, spcd_handle_overpressure_irq, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "spcd_overpressure", spcd_data);
    if (ret == -ENOSYS) {
        return -EPROBE_DEFER;
    }
    if (ret) {
        dev_err(&pdev->dev, "couldn't request irq %s %d: %d\n", "irq_overpressure", spcd_data->irq_overpressure, ret);
        return ret;
    }

    ret = devm_request_threaded_irq(dev, spcd_data->irq_stuckon, NULL, spcd_handle_stuckon_irq, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "spcd_stuckon", spcd_data);
    if (ret == -ENOSYS) {
        return -EPROBE_DEFER;
    }
    if (ret) {
        dev_err(&pdev->dev, "couldn't request irq %s %d: %d\n", "irq_stuckon", spcd_data->irq_stuckon, ret);
        return ret;
    }

    ret = devm_request_threaded_irq(dev, spcd_data->irq_mode, NULL, spcd_handle_mode_irq, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "spcd_mode", spcd_data);
    if (ret == -ENOSYS) {
        return -EPROBE_DEFER;
    }
    if (ret) {
        dev_err(&pdev->dev, "couldn't request irq %s %d: %d\n", "irq_mode", spcd_data->irq_mode, ret);
        return ret;
    }

    // Fall through to success
    return ret;
}

static int spcd_remove(struct platform_device *pdev) {
    struct spcd_data *spcd_data = platform_get_drvdata(pdev);

    pr_debug(" %s\n", __FUNCTION__);

    kfree(spcd_data);

    return 0;
}


static const struct of_device_id of_spcd_match[] = {
        {.compatible = "avt,spcd",},
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
