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
#include <linux/hrtimer.h>
// Added to support sysfs_emit, being pulled in from newer kernel releases.
#include <linux/mm.h>
#include <linux/bug.h>
#include <linux/slab.h>

struct spcd_data {
    struct device       *dev;
    struct gpio_desc    *gpio_in_12v_status;
    int                 irq_12v_status;

    struct gpio_desc    *gpio_in_valve_open;
    int                 irq_valve_open;

    struct gpio_desc    *gpio_in_overpressure;
    int                 irq_overpressure;

    struct gpio_desc    *gpio_in_stuckon;
    int                 irq_stuckon;

    struct gpio_desc    *gpio_in_mode;
    int                 irq_mode;

    struct pwm_device   *pwmd_blower;
    struct pwm_device   *pwmd_valve;


    struct gpio_desc    *gpio_out_pwr_hold; // Unused
    struct gpio_desc    *gpio_out_wdt_alert;

    struct gpio_desc    *gpio_out_failsafe;
    struct gpio_desc    *gpio_out_blower_stat;
    struct gpio_desc    *gpio_out_1min;

    struct pwm_state    blower_state;
    struct pwm_state    valve_state;
    u8                  failsafe_enable;

    struct hrtimer      pulse_timer;
    u8                  pulse_value;
    ktime_t             pulse_period;
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

// Pulse Timer Callback
enum hrtimer_restart pulse_timer_callback(struct hrtimer *timer) {
    pr_debug(" %s\n", __FUNCTION__);
    struct spcd_data *spcd = container_of(timer, struct spcd_data, pulse_timer);

    // toggle pin state.
    spcd->pulse_value = spcd->pulse_value ^ 1;
    gpiod_set_value_cansleep(spcd->gpio_out_1min, spcd->pulse_value);
    hrtimer_forward_now(&spcd->pulse_timer, spcd->pulse_period);

    return HRTIMER_RESTART;
}



static int spcd_set_state(struct spcd_data *spcd) {
    gpiod_set_value(spcd->gpio_out_failsafe, spcd->failsafe_enable);

    gpiod_set_value(spcd->gpio_out_blower_stat, spcd->blower_state.enabled ? 1 : 0);
    hrtimer_try_to_cancel(&spcd->pulse_timer);
    if (spcd->blower_state.enabled) {
        hrtimer_forward_now(&spcd->pulse_timer, spcd->pulse_period);
        hrtimer_restart(&spcd->pulse_timer);
    }
//    gpiod_set_value(spcd->gpio_out_pwr_hold, 0); // TODO: Future. Currently DNP.
//    gpiod_set_value(spcd->gpio_out_wdt_alert, 0); // TODO: Set this if we restart 'unclean'.

    pwm_apply_state(spcd->pwmd_blower, &(spcd->blower_state));
    pwm_apply_state(spcd->pwmd_valve, &(spcd->valve_state));

    return 0;
}

static int spcd_read_state(struct spcd_data *spcd) {
    // TODO: read the input lines and derive that into state on the spcd struct.
    return 0;
}




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

static ssize_t failsafe_show(struct device *dev, struct device_attribute *attr, char *buf) {
    struct spcd_data *spcd = dev_get_drvdata(dev);

    int val = gpiod_get_value(spcd->gpio_out_failsafe);
    if (val < 0) {
        return val;
    }

    return sysfs_emit(buf, "%d\n", val);
}

static ssize_t failsafe_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
    struct spcd_data *spcd = dev_get_drvdata(dev);

    int val;
    int err;

    err = kstrtoint(buf, 10, &val);
    if (err) {
        return err;
    }

    gpiod_set_value(spcd->gpio_out_failsafe, val);

    return count;
}
static DEVICE_ATTR_RW(failsafe);





static struct attribute *spcd_attrs[] = {
        &dev_attr_status_12v.attr,
        &dev_attr_valve_open.attr,
        &dev_attr_overpressure.attr,
        &dev_attr_stuck_on.attr,
        &dev_attr_mode_switch.attr,
        &dev_attr_pwr_hold.attr,
        &dev_attr_wdt_alert.attr,
        &dev_attr_failsafe.attr,

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
    spcd_data->pulse_value = 0;
    spcd_data->pulse_period = ktime_set(30, 0);

    hrtimer_init(&spcd_data->pulse_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_PINNED_HARD);
    spcd_data->pulse_timer.function = pulse_timer_callback;

    // spcd_data->input_dirty = false;

    // PWM devices from device tree bindings
    spcd_data->pwmd_blower = devm_pwm_get(dev, "blower-pwm");
    if (IS_ERR(spcd_data->pwmd_blower)) {
        dev_err(dev, "failed to acquire blower PWM device. err=%ld\n", PTR_ERR(spcd_data->pwmd_blower));
        return PTR_ERR(spcd_data->pwmd_blower);
    }
    pwm_init_state(spcd_data->pwmd_blower, &(spcd_data->blower_state));
    spcd_data->blower_state.period = 100000000;
    spcd_data->blower_state.polarity = PWM_POLARITY_NORMAL;
    pwm_set_relative_duty_cycle(&(spcd_data->blower_state), 50, 100);
    spcd_data->blower_state.enabled = false;


    spcd_data->pwmd_valve = devm_pwm_get(dev, "valve-pwm");
    if (IS_ERR(spcd_data->pwmd_valve)) {
        dev_err(dev, "failed to acquire valve PWM device. err=%ld\n", PTR_ERR(spcd_data->pwmd_valve));
        return PTR_ERR(spcd_data->pwmd_valve);
    }
    pwm_init_state(spcd_data->pwmd_valve, &(spcd_data->valve_state));
    spcd_data->valve_state.period = 1000000000;
    spcd_data->valve_state.polarity = PWM_POLARITY_NORMAL;
    pwm_set_relative_duty_cycle(&(spcd_data->valve_state), 50, 100);
    spcd_data->blower_state.enabled = false;


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


    // All outputs on the I2C expander are open-drain.
    spcd_data->gpio_out_failsafe = devm_gpiod_get(dev, "out-failsafe", GPIOD_OUT_LOW_OPEN_DRAIN);
    if (IS_ERR(spcd_data->gpio_out_failsafe)) {
        dev_err(dev, "failed to get out-failsafe-gpio: err=%ld\n", PTR_ERR(spcd_data->gpio_out_failsafe));
        return PTR_ERR(spcd_data->gpio_out_failsafe);
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

    // Setup initial GPIO states
    spcd_data->failsafe_enable = 0;

    platform_set_drvdata(pdev, spcd_data);

    // sync initial state
    spcd_set_state(spcd_data);
    // spcd_read_state(spcd_data);


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

MODULE_AUTHOR("bryan.varner@robustified.com");
MODULE_DESCRIPTION("AVT SPCD Platform Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:spcd");
