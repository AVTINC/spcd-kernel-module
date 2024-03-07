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
#include <linux/workqueue.h>
// Added to support sysfs_emit, being pulled in from newer kernel releases.
#include <linux/mm.h>
#include <linux/bug.h>
#include <linux/slab.h>

struct spcd_valve_state {
    u64 duty_cycle;
    u64 period;
    s64 duration;
};

struct spcd_data {
    struct device *dev;
    struct cdev cdev;
    dev_t cdev_num;

    bool input_dirty;
    bool status_12v;
    bool status_preboot_stat;
    bool status_failsafe;
    bool status_valve_open;
    bool status_overpressure;
    bool status_stuckon;
    bool status_dealer_enable;
    bool status_postboot_stat;

    struct gpio_desc *gpio_in_12v_status;
    int irq_12v_status;

    struct gpio_desc *gpio_in_valve_open;
    int irq_valve_open;

    struct gpio_desc *gpio_in_overpressure;
    int irq_overpressure;

    struct gpio_desc *gpio_in_stuckon;
    int irq_stuckon;

    struct gpio_desc *gpio_in_mode;
    int irq_mode;

    struct gpio_desc *gpio_in_failsafe;
    int irq_failsafe_status;

    struct gpio_desc *gpio_in_preboot_stat; // Don't request an IRQ for this one.

    struct pwm_device *pwmd_blower;
    struct pwm_device *pwmd_valve;


    struct gpio_desc *gpio_out_pwr_hold; // Unused
    struct gpio_desc *gpio_out_postboot_stat;

    struct gpio_desc *gpio_out_failsafe_enable;
    struct gpio_desc *gpio_out_blower_stat;
    struct gpio_desc *gpio_out_cpu_heartbeat;

    struct pwm_state blower_state;
    struct pwm_state valve_state;
    u8 failsafe_enable;

    struct hrtimer cpu_heartbeat_timer;
    u8 cpu_heartbeat_value;
    ktime_t cpu_heartbeat_period;

    struct work_struct heartbeat;
    wait_queue_head_t heartbeat_queue;

    /* workqueue support for reading gpio lines on i2c expanders */
    struct mutex readexp_mutex;
    struct work_struct readexp;
    wait_queue_head_t readexp_queue;
    u8 expLinesToRead;

    struct spcd_valve_state *valve_states;
    struct hrtimer valve_timer;
    u8 valve_state_count;
    u8 valve_state_current;

    struct work_struct valve_ctrl;
    wait_queue_head_t valve_ctrl_queue;
};

#define CMD_SET_BLOWER_PWM 0x01
#define CMD_SET_VALVE_PWM 0x02
#define CMD_START_VALVE_CYCLE 0x04
#define CMD_STOP_VALVE_CYCLE 0x08

#define READ_VALVE_OPEN 0x01
#define READ_OVERPRESSURE 0x02
#define READ_STUCKON 0x04
#define READ_DEALER 0x08

#define SPCD_DEVICE_NAME "spcd"
#define SPCD_CLASS "spcd-class"
struct class *spcd_class;

static DECLARE_WAIT_QUEUE_HEAD(spcd_rq);

// chardev I/O

// u64 blower period
// u64 blower duty_cycle cycle
// u64 valve period
// u64 valve duty_cycle cycle



// Shamelessly cribbed from newer kernel versions (5.17.5)
// Having this here, greatly reduces our repetition.
// https://elixir.bootlin.com/linux/v5.17.5/C/ident/sysfs_emit
int sysfs_emit(char *buf, const char *fmt, ...) {
    va_list args;
    int len;

    if (WARN(!buf || offset_in_page(buf), "invalid sysfs_emit: buf:%p\n", (void*)buf)) {
        return 0;
    }

    va_start(args, fmt);
    len = vscnprintf(buf, PAGE_SIZE, fmt, args);
    va_end(args);

    return len;
}

static void valve_ctrl_handler(struct work_struct *work) {
    struct spcd_data *spcd = container_of(work, struct spcd_data, valve_ctrl);
    pwm_apply_state(spcd->pwmd_valve, &(spcd->valve_state));
}

static void heartbeat_handler(struct work_struct *work) {
    struct spcd_data *spcd = container_of(work, struct spcd_data, heartbeat);

    pr_debug(" write heartbeat: %d\n", spcd->cpu_heartbeat_value);
    gpiod_set_value_cansleep(spcd->gpio_out_cpu_heartbeat, spcd->cpu_heartbeat_value);
}


static void read_exp_handler(struct work_struct *work) {
    struct spcd_data *spcd = container_of(work, struct spcd_data, readexp);
    pr_debug("   read_exp_handler\n");
    mutex_lock(&spcd->readexp_mutex);
    if (spcd->expLinesToRead & READ_VALVE_OPEN) {
        spcd->status_valve_open = gpiod_get_value_cansleep(spcd->gpio_in_valve_open) == 1;
        spcd->expLinesToRead &= ~READ_VALVE_OPEN;
        pr_debug("   READ_VALVE_OPEN: %s\n", spcd->status_valve_open ? "true" : "false");
    }
    if (spcd->expLinesToRead & READ_OVERPRESSURE) {
        spcd->status_overpressure = gpiod_get_value_cansleep(spcd->gpio_in_overpressure) == 1;
        spcd->expLinesToRead &= ~READ_OVERPRESSURE;
        pr_debug("   READ_OVERPRESSURE: %s\n", spcd->status_overpressure ? "true" : "false");
    }
    if (spcd->expLinesToRead & READ_STUCKON) {
        spcd->status_stuckon = gpiod_get_value_cansleep(spcd->gpio_in_stuckon) == 1;
        spcd->expLinesToRead &= ~READ_STUCKON;
        pr_debug("   READ_STUCKON: %s\n", spcd->status_stuckon ? "true" : "false");
    }
    if (spcd->expLinesToRead & READ_DEALER) {
        spcd->status_dealer_enable = gpiod_get_value_cansleep(spcd->gpio_in_mode) == 1;
        spcd->expLinesToRead &= ~READ_DEALER;
        pr_debug("   READ_DEALER: %s\n", spcd->status_dealer_enable ? "true" : "false");
    }
    spcd->input_dirty = true;
    mutex_unlock(&spcd->readexp_mutex);
    wake_up_interruptible(&spcd_rq);
}



// Timer that fires every 30 seconds to produce the heartbeat pulse.
// If the blower has a non-0 duty_cycle cycle the pin state will be toggled.
// If the blower has a 0 duty_cycle cycle, the pin will be set 0 and held there.
static enum hrtimer_restart cpu_heartbeat_timer_callback(struct hrtimer *timer) {
    struct spcd_data *spcd = container_of(timer, struct spcd_data, cpu_heartbeat_timer);

    if (spcd->blower_state.duty_cycle > 0) {
        spcd->cpu_heartbeat_value = spcd->cpu_heartbeat_value ^ 1;
        schedule_work(&spcd->heartbeat);
    } else if (spcd->cpu_heartbeat_value != 0) {
        spcd->cpu_heartbeat_value = 0;
        schedule_work(&spcd->heartbeat);
    }

    hrtimer_forward_now(&spcd->cpu_heartbeat_timer, spcd->cpu_heartbeat_period);
    return HRTIMER_RESTART;
}

/**
 * Invoked by the valve timer. Represents the top-half of the hrtimer interrupt handler.
 * This atomic context cannot do i2c comms to apply PWM. Due to scheduling calls for delays in the pwm driver.
 *
 * @param timer
 * @return
 */
static enum hrtimer_restart valve_timer_callback(struct hrtimer *timer) {
    struct spcd_data *spcd = container_of(timer, struct spcd_data, valve_timer);
    pr_debug(" %s\n", __FUNCTION__);

    // Next step
    spcd->valve_state_current++;
    // Handle looping.
    if (spcd->valve_state_current >= spcd->valve_state_count) {
        spcd->valve_state_current = 0;
    }

    // Apply the state, set the timer, go on with life.
    spcd->valve_state.period = ktime_set(0, spcd->valve_states[spcd->valve_state_current].period);
    spcd->valve_state.duty_cycle = ktime_set(0, spcd->valve_states[spcd->valve_state_current].duty_cycle);
    schedule_work(&spcd->valve_ctrl);
    hrtimer_forward_now(&spcd->valve_timer, ktime_set(0, spcd->valve_states[spcd->valve_state_current].duration));

    return HRTIMER_RESTART;
}


static int spcd_set_state(struct spcd_data *spcd) {
    pr_debug("spcd_set_state():\n");
    pr_debug("  failsafe_enable: %s\n", spcd->failsafe_enable > 0 ? "on" : "off");
    gpiod_set_value_cansleep(spcd->gpio_out_failsafe_enable, spcd->failsafe_enable);
    pr_debug("  blower_stat: %s\n", spcd->blower_state.duty_cycle > 0 ? "on" : "off");
    gpiod_set_value_cansleep(spcd->gpio_out_blower_stat, spcd->blower_state.duty_cycle > 0 ? 1 : 0);

//    gpiod_set_value(spcd->gpio_out_pwr_hold, 0); // TODO: Future. Currently DNP.
    gpiod_set_value(spcd->gpio_out_postboot_stat, spcd->status_postboot_stat == true ? 1 : 0);

    pr_debug("  blower [duty_cycle:%llu, period:%llu, enabled: %s]\n", spcd->blower_state.duty_cycle, spcd->blower_state.period, spcd->blower_state.enabled ? "true" : "false");
    pwm_apply_state(spcd->pwmd_blower, &(spcd->blower_state));
    pr_debug("  valve [duty_cycle:%llu, period:%llu, enabled: %s]\n", spcd->valve_state.duty_cycle, spcd->valve_state.period, spcd->valve_state.enabled ? "true" : "false");
    pwm_apply_state(spcd->pwmd_valve, &(spcd->valve_state));

    return 0;
}

static int spcd_read_state(struct spcd_data *spcd) {
    spcd->status_12v = gpiod_get_value(spcd->gpio_in_12v_status) == 1;
    spcd->status_preboot_stat = gpiod_get_value(spcd->gpio_in_preboot_stat) == 1;
    spcd->status_failsafe = gpiod_get_value(spcd->gpio_in_failsafe) == 1;
    // The following reads can sleep.
    spcd->status_valve_open = gpiod_get_value_cansleep(spcd->gpio_in_valve_open) == 1;
    spcd->status_overpressure = gpiod_get_value_cansleep(spcd->gpio_in_overpressure) == 1;
    spcd->status_stuckon = gpiod_get_value_cansleep(spcd->gpio_in_stuckon) == 1;
    spcd->status_dealer_enable = gpiod_get_value_cansleep(spcd->gpio_in_mode) == 1;
    spcd->input_dirty = true;

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

static ssize_t failsafe_status_show(struct device *dev, struct device_attribute *attr, char *buf) {
    struct spcd_data *spcd = dev_get_drvdata(dev);

    int val = gpiod_get_value(spcd->gpio_in_failsafe);
    if (val < 0) {
        return val;
    }

    return sysfs_emit(buf, "%d\n", val);
}

static DEVICE_ATTR_RO(failsafe_status);


/* sfs attributes -- OUTPUTS */
static ssize_t preboot_stat_show(struct device *dev, struct device_attribute *attr, char *buf) {
    struct spcd_data *spcd = dev_get_drvdata(dev);

    int val = gpiod_get_value(spcd->gpio_in_preboot_stat);
    if (val < 0) {
        return val;
    }

    return sysfs_emit(buf, "%d\n", val);
}

static DEVICE_ATTR_RO(preboot_stat);

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


static ssize_t postboot_stat_show(struct device *dev, struct device_attribute *attr, char *buf) {
    struct spcd_data *spcd = dev_get_drvdata(dev);

    int val = gpiod_get_value(spcd->gpio_out_postboot_stat);
    if (val < 0) {
        return val;
    }

    return sysfs_emit(buf, "%d\n", val);
}

static ssize_t postboot_stat_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
    struct spcd_data *spcd = dev_get_drvdata(dev);
    int val;
    int err;

    err = kstrtoint(buf, 10, &val);
    if (err) {
        return err;
    }

    gpiod_set_value(spcd->gpio_out_postboot_stat, val);
    return count;
}

static DEVICE_ATTR_RW(postboot_stat);

static ssize_t failsafe_enable_show(struct device *dev, struct device_attribute *attr, char *buf) {
    struct spcd_data *spcd = dev_get_drvdata(dev);

    int val = gpiod_get_value_cansleep(spcd->gpio_out_failsafe_enable);
    if (val < 0) {
        return val;
    }

    return sysfs_emit(buf, "%d\n", val);
}

static ssize_t failsafe_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
    struct spcd_data *spcd = dev_get_drvdata(dev);

    int val;
    int err;

    err = kstrtoint(buf, 10, &val);
    if (err) {
        return err;
    }

    gpiod_set_value_cansleep(spcd->gpio_out_failsafe_enable, val);

    return count;
}

static DEVICE_ATTR_RW(failsafe_enable);

/* -- sysfs attributes -- PWM -- */
static ssize_t blower_duty_cycle_show(struct device *dev, struct device_attribute *attr, char *buf) {
    struct spcd_data *spcd = dev_get_drvdata(dev);
    return sysfs_emit(buf, "%llu\n", ktime_to_ns(spcd->blower_state.duty_cycle));
}

static ssize_t blower_duty_cycle_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
    struct spcd_data *spcd = dev_get_drvdata(dev);
    long dutyonnanos;
    int err;
    err = kstrtol(buf, 10, &dutyonnanos);
    if (err) {
        return err;
    }
    spcd->blower_state.duty_cycle = ktime_set(0, dutyonnanos);
    spcd_set_state(spcd);
    return count;
}

static DEVICE_ATTR_RW(blower_duty_cycle);


static ssize_t blower_period_show(struct device *dev, struct device_attribute *attr, char *buf) {
    struct spcd_data *spcd = dev_get_drvdata(dev);
    return sysfs_emit(buf, "%llu\n", ktime_to_ns(spcd->blower_state.period));
}

static ssize_t blower_period_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
    struct spcd_data *spcd = dev_get_drvdata(dev);
    long periodnanos;
    int err;
    err = kstrtol(buf, 10, &periodnanos);
    if (err) {
        return err;
    }
    spcd->blower_state.period = ktime_set(0, periodnanos);
    spcd_set_state(spcd);
    return count;
}

static DEVICE_ATTR_RW(blower_period);


static ssize_t valve_duty_cycle_show(struct device *dev, struct device_attribute *attr, char *buf) {
    struct spcd_data *spcd = dev_get_drvdata(dev);
    return sysfs_emit(buf, "%llu\n", ktime_to_ns(spcd->valve_state.duty_cycle));
}

static ssize_t valve_duty_cycle_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
    struct spcd_data *spcd = dev_get_drvdata(dev);
    long dutynanos;
    int err;

    err = kstrtol(buf, 10, &dutynanos);
    if (err) {
        return err;
    }
    spcd->valve_state.duty_cycle = ktime_set(0, dutynanos);
    spcd_set_state(spcd);
    return count;
}

static DEVICE_ATTR_RW(valve_duty_cycle);

static ssize_t valve_period_show(struct device *dev, struct device_attribute *attr, char *buf) {
    struct spcd_data *spcd = dev_get_drvdata(dev);
    return sysfs_emit(buf, "%llu\n", ktime_to_ns(spcd->valve_state.period));
}

static ssize_t valve_period_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
    struct spcd_data *spcd = dev_get_drvdata(dev);
    long periodnanos;
    int err;

    err = kstrtol(buf, 10, &periodnanos);
    if (err) {
        return err;
    }
    spcd->valve_state.period = ktime_set(0, periodnanos);
    spcd_set_state(spcd);
    return count;
}

static DEVICE_ATTR_RW(valve_period);


static struct attribute *spcd_attrs[] = {
        &dev_attr_status_12v.attr,
        &dev_attr_valve_open.attr,
        &dev_attr_overpressure.attr,
        &dev_attr_stuck_on.attr,
        &dev_attr_failsafe_status.attr,
        &dev_attr_mode_switch.attr,
        &dev_attr_pwr_hold.attr,
        &dev_attr_preboot_stat.attr,
        &dev_attr_postboot_stat.attr,
        &dev_attr_failsafe_enable.attr,
        &dev_attr_blower_duty_cycle.attr,
        &dev_attr_blower_period.attr,
        &dev_attr_valve_period.attr,
        &dev_attr_valve_duty_cycle.attr,

        NULL
};
ATTRIBUTE_GROUPS(spcd);


/* -- Interrupt Handlers -- */
/*
 * All interrupt handlers are treated as bottom half threaded interrupts.
 * Due to the use of the max7313 i2C GPIO expander, which allows
 * gpio lines (reads / writes) to 'sleep' any I/O to read the state of that
 * device will need to happen in a work_queue.
 */
static irqreturn_t spcd_handle_status_12v(int irq, void *dev_id) {
    struct spcd_data *spcd = dev_id;
    pr_debug(" %s\n", __FUNCTION__);
    spcd->status_12v = gpiod_get_value(spcd->gpio_in_12v_status) == 1;
    spcd->input_dirty = true;
    wake_up_interruptible(&spcd_rq);

    return IRQ_HANDLED;
}

static irqreturn_t spcd_handle_failsafe_status_irq(int irq, void *dev_id) {
    struct spcd_data *spcd = dev_id;
    pr_debug(" %s\n", __FUNCTION__);
    spcd->status_failsafe = gpiod_get_value(spcd->gpio_in_failsafe) == 1;
    spcd->input_dirty = true;
    wake_up_interruptible(&spcd_rq);

    return IRQ_HANDLED;
}

static irqreturn_t spcd_handle_valve_irq(int irq, void *dev_id) {
    struct spcd_data *spcd = dev_id;
    pr_debug(" %s\n", __FUNCTION__);

    mutex_lock(&spcd->readexp_mutex);
    spcd->expLinesToRead |= READ_VALVE_OPEN;
    mutex_unlock(&spcd->readexp_mutex);

    schedule_work(&spcd->readexp);

    return IRQ_HANDLED;
}

static irqreturn_t spcd_handle_overpressure_irq(int irq, void *dev_id) {
    struct spcd_data *spcd = dev_id;
    pr_debug(" %s\n", __FUNCTION__);

    mutex_lock(&spcd->readexp_mutex);
    spcd->expLinesToRead |= READ_OVERPRESSURE;
    mutex_unlock(&spcd->readexp_mutex);

    schedule_work(&spcd->readexp);

    return IRQ_HANDLED;
}

static irqreturn_t spcd_handle_stuckon_irq(int irq, void *dev_id) {
    struct spcd_data *spcd = dev_id;
    pr_debug(" %s\n", __FUNCTION__);

    mutex_lock(&spcd->readexp_mutex);
    spcd->expLinesToRead |= READ_STUCKON;
    mutex_unlock(&spcd->readexp_mutex);

    schedule_work(&spcd->readexp);

    return IRQ_HANDLED;
}

static irqreturn_t spcd_handle_mode_irq(int irq, void *dev_id) {
    struct spcd_data *spcd = dev_id;
    pr_debug(" %s\n", __FUNCTION__);
    pr_debug("  %s\n", __FUNCTION__);
    mutex_lock(&spcd->readexp_mutex);
    spcd->expLinesToRead |= READ_DEALER;
    mutex_unlock(&spcd->readexp_mutex);

    pr_debug("   scheduling READ_DEALER");
    schedule_work(&spcd->readexp);

    return IRQ_HANDLED;
}

static int spcd_open(struct inode *inode, struct file *filp) {
    unsigned int maj = imajor(inode);
    unsigned int min = iminor(inode);

    struct spcd_data *spcd_data = NULL;
    spcd_data = container_of(inode->i_cdev, struct spcd_data, cdev);

    if (min < 0) {
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
    struct spcd_data *spcd_data = filp->private_data;
    char *buf_read_loc = (char*)buf;
    u8 cmd = 0x00;
    u8 cycles = 0;
    u64 l = 0;

    int i;
    struct spcd_valve_state *nStates;

    // First byte is the command.
    copy_from_user(&cmd, buf_read_loc, sizeof cmd);
    buf_read_loc+=sizeof(cmd);

    // Anytime we stop, start, or reset valve cycles, make sure we stop the timer (if any)
    // and open the valve.
    if (cmd == CMD_STOP_VALVE_CYCLE || cmd == CMD_START_VALVE_CYCLE || cmd == CMD_SET_VALVE_PWM) {
        // Disable the valve cycle timers.
        hrtimer_cancel(&(spcd_data->valve_timer));
        spcd_data->valve_state_current = 0;
        // Open the valve
        spcd_data->valve_state.duty_cycle = 0;
        pwm_apply_state(spcd_data->pwmd_valve, &(spcd_data->valve_state));
    }

    // Continue processing further commands.
    if (cmd == CMD_SET_BLOWER_PWM) {
        pr_debug("  set blower\n");
        copy_from_user(&l, buf_read_loc, sizeof(u64));
        pr_debug("    current blower_state.period: %llu\n", spcd_data->blower_state.period);
        buf_read_loc+=sizeof(u64);
        spcd_data->blower_state.period = l;
        pr_debug("    period: %llu  blower_state.period: %llu\n", l, spcd_data->blower_state.period);

        copy_from_user(&l, buf_read_loc, sizeof(u64));
        buf_read_loc+=sizeof(u64);
        pr_debug("    current blower_state.duty: %llu\n", (spcd_data->blower_state.duty_cycle));
        spcd_data->blower_state.duty_cycle = l;
        pr_debug("    duty_cycle: %llu  blower_state.duty: %llu\n", l, (spcd_data->blower_state.duty_cycle));

        // Set the enabled flag based on the current duty_cycle.
        spcd_data->blower_state.enabled = spcd_data->blower_state.duty_cycle > 0;

        // Update the stat pin based on the current duty_cycle.
        gpiod_set_value_cansleep(spcd_data->gpio_out_blower_stat, spcd_data->blower_state.duty_cycle > 0 ? 1 : 0);
        // Set the blower PWM.
        pwm_apply_state(spcd_data->pwmd_blower, &(spcd_data->blower_state));
    } else if (cmd == CMD_SET_VALVE_PWM) {
        pr_debug("  set valve pwm\n");
        copy_from_user(&cycles, buf_read_loc, sizeof(u8));
        buf_read_loc+=sizeof(u8);
        pr_debug("    cycles to read: %d\n", cycles);

        // cyles is a byte, so we don't need to swap byte order.
        nStates = kcalloc(cycles, sizeof(struct spcd_valve_state), GFP_KERNEL);
        for (i = 0; i < cycles; i++) {
            pr_debug("      reading cycle: %d\n", i);
            copy_from_user(&(nStates[i].period), buf_read_loc, sizeof(u64));
            buf_read_loc+=sizeof(u64);
            pr_debug("        period: %llu\n", nStates[i].period);

            copy_from_user(&(nStates[i].duty_cycle), buf_read_loc, sizeof(u64));
            buf_read_loc+=sizeof(u64);
            pr_debug("        duty_cycle: %llu\n", nStates[i].duty_cycle);

            copy_from_user(&(nStates[i].duration), buf_read_loc, sizeof(u64));
            buf_read_loc+=sizeof(u64);
            pr_debug("        duration: %lld\n", nStates[i].duration);
        }

        // Update the list of states
        if (spcd_data->valve_states != NULL) {
            kfree(spcd_data->valve_states);
        }
        spcd_data->valve_state_current = 0;
        spcd_data->valve_states = nStates;
        spcd_data->valve_state_count = cycles;
    } else if (cmd == CMD_START_VALVE_CYCLE) {
        if (spcd_data->valve_state_count > 0) {
            pr_debug("  Start valve_timer");
            // Set the current step to a number we won't normally get to.
            // When the callback is invoked, we will start at the first cycle.
            spcd_data->valve_state_current = spcd_data->valve_state_count;
            hrtimer_start(&spcd_data->valve_timer, 0, HRTIMER_MODE_REL);
        }
    }

    // Report that we read everything.
    *f_pos += count;
    return count;
}

ssize_t spcd_read(struct file *filp, char __user *buf, size_t count, loff_t *pos) {
    struct spcd_data *spcd_data = filp->private_data;
    ssize_t readlen = 1;
    u8 state = 0x00;

    // If the data hasn't changes, return that we should try again later.
    if (spcd_data->input_dirty == false) {
        return -EAGAIN;
    }

    // If we want fewer than the amount of data we should read, only copy what was asked for.
    // Likely to never happen for this driver (since everything fits in one byte right now) but it's good paranoia.
    if (count < readlen) {
        readlen = count;
    }
    // Note: This device _never_ returns an EOF. The next read is an atomic attempt to read the entire device state.

    // Pack the bits into a single byte and copy that to the user space buffer.
    state |= spcd_data->status_preboot_stat << 6;
    state |= spcd_data->status_12v << 5;
    state |= spcd_data->status_failsafe << 4;
    state |= spcd_data->status_valve_open << 3;
    state |= spcd_data->status_overpressure << 2;
    state |= spcd_data->status_stuckon << 1;
    state |= spcd_data->status_dealer_enable << 0;
    // Since this is a single byte we do not need to enforce byte order
    if (copy_to_user(buf, &state, readlen) != 0) {
        return -EIO;
    }

    // Adjust the kernel position pointer to track the number of bytes copied from this fd.
    *pos += readlen;

    // Mark that there is no data left before returning to userspace.
    spcd_data->input_dirty = false;

    return readlen;
}

static unsigned int spcd_poll(struct file *file, poll_table *wait) {
    struct spcd_data *spcd_data = file->private_data;
    unsigned int reval_mask = 0;

    poll_wait(file, &spcd_rq, wait);

    if (spcd_data->input_dirty) {
        reval_mask |= (POLLIN | POLLRDNORM);
    }

    return reval_mask;
}



static const struct file_operations spcd_fops = {
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

    pr_debug(" %s\n", __FUNCTION__);

    // create the driver data....
    spcd_data = kzalloc(sizeof(struct spcd_data), GFP_KERNEL);
    if (!spcd_data) {
        return -ENOMEM;
    }
    spcd_data->dev = dev;
    spcd_data->cpu_heartbeat_value = 0;
    spcd_data->cpu_heartbeat_period = ktime_set(30, 0);
    mutex_init(&spcd_data->readexp_mutex);
    spcd_data->valve_state_count = 0;
    spcd_data->valve_state_current = 0;
    spcd_data->valve_states = NULL;

    INIT_WORK(&spcd_data->valve_ctrl, valve_ctrl_handler);
    init_waitqueue_head(&spcd_data->valve_ctrl_queue);

    INIT_WORK(&spcd_data->heartbeat, heartbeat_handler);
    init_waitqueue_head(&spcd_data->heartbeat_queue);

    INIT_WORK(&spcd_data->readexp, read_exp_handler);
    init_waitqueue_head(&spcd_data->readexp_queue);

    // spcd_data->input_dirty = false;

    // PWM devices from device tree bindings
    spcd_data->pwmd_blower = devm_pwm_get(dev, "blower");
    if (IS_ERR(spcd_data->pwmd_blower)) {
        dev_err(dev, "failed to acquire blower PWM device. err=%ld\n", PTR_ERR(spcd_data->pwmd_blower));
        return PTR_ERR(spcd_data->pwmd_blower);
    }
    pwm_init_state(spcd_data->pwmd_blower, &(spcd_data->blower_state));
    spcd_data->blower_state.period = 2000000; //500hz
    spcd_data->blower_state.polarity = PWM_POLARITY_NORMAL;
    pwm_set_relative_duty_cycle(&(spcd_data->blower_state), 0, 100);


    spcd_data->pwmd_valve = devm_pwm_get(dev, "valve");
    if (IS_ERR(spcd_data->pwmd_valve)) {
        dev_err(dev, "failed to acquire valve PWM device. err=%ld\n", PTR_ERR(spcd_data->pwmd_valve));
        return PTR_ERR(spcd_data->pwmd_valve);
    }
    pwm_init_state(spcd_data->pwmd_valve, &(spcd_data->valve_state));
    spcd_data->valve_state.period = 2000000; //500hz
    spcd_data->valve_state.polarity = PWM_POLARITY_NORMAL;
    pwm_set_relative_duty_cycle(&(spcd_data->valve_state), 0, 100);


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

    spcd_data->gpio_in_preboot_stat = devm_gpiod_get(dev, "in-preboot-stat", GPIOD_IN);
    if (IS_ERR(spcd_data->gpio_in_preboot_stat)) {
        dev_err(dev, "failed to get in-preboot-stat-gpio: err=%ld\n", PTR_ERR(spcd_data->gpio_in_preboot_stat));
        return PTR_ERR(spcd_data->gpio_in_preboot_stat);
    }

    spcd_data->gpio_in_failsafe = devm_gpiod_get(dev, "in-failsafe-status", GPIOD_IN);
    if (IS_ERR(spcd_data->gpio_in_failsafe)) {
        dev_err(dev, "failed to get in-failsafe-status-gpio: err=%ld\n", PTR_ERR(spcd_data->gpio_in_failsafe));
        return PTR_ERR(spcd_data->gpio_in_failsafe);
    }
    spcd_data->irq_failsafe_status = gpiod_to_irq(spcd_data->gpio_in_failsafe);
    if (spcd_data->irq_failsafe_status < 0) {
        dev_err(dev, "failed to get IRQ for in_failsafe_status: err=%d\n", spcd_data->irq_failsafe_status);
        return spcd_data->irq_failsafe_status;
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

    // SBC Outputs default to open-drain with internal pull-ups. Electrical 'defaults to high' pre init on the SBC GPIOS is a problematic flicker.
    // So we set those up open drain to get around it.
    spcd_data->gpio_out_postboot_stat = devm_gpiod_get(dev, "out-postboot-stat", GPIOD_OUT_LOW_OPEN_DRAIN);
    if (IS_ERR(spcd_data->gpio_out_postboot_stat)) {
        dev_err(dev, "failed to get out-postboot-stat-gpio: err=%ld\n", PTR_ERR(spcd_data->gpio_out_postboot_stat));
        return PTR_ERR(spcd_data->gpio_out_postboot_stat);
    }


    // All outputs on the I2C expander are open-drain.
    spcd_data->gpio_out_failsafe_enable = devm_gpiod_get(dev, "out-failsafe-enable", GPIOD_OUT_LOW_OPEN_DRAIN);
    if (IS_ERR(spcd_data->gpio_out_failsafe_enable)) {
        dev_err(dev, "failed to get out-failsafe-enable-gpio: err=%ld\n", PTR_ERR(spcd_data->gpio_out_failsafe_enable));
        return PTR_ERR(spcd_data->gpio_out_failsafe_enable);
    }
    spcd_data->gpio_out_blower_stat = devm_gpiod_get(dev, "out-blower-stat", GPIOD_OUT_LOW_OPEN_DRAIN);
    if (IS_ERR(spcd_data->gpio_out_blower_stat)) {
        dev_err(dev, "failed to get out-blower-stat-gpio: err=%ld\n", PTR_ERR(spcd_data->gpio_out_blower_stat));
        return PTR_ERR(spcd_data->gpio_out_blower_stat);
    }
    spcd_data->gpio_out_cpu_heartbeat = devm_gpiod_get(dev, "out-cpu-heartbeat", GPIOD_OUT_LOW_OPEN_DRAIN);
    if (IS_ERR(spcd_data->gpio_out_cpu_heartbeat)) {
        dev_err(dev, "failed to get out-cpu-heartbeat-gpio: err=%ld\n", PTR_ERR(spcd_data->gpio_out_cpu_heartbeat));
        return PTR_ERR(spcd_data->gpio_out_cpu_heartbeat);
    }

    // Setup initial GPIO states
    spcd_data->failsafe_enable = 0;

    platform_set_drvdata(pdev, spcd_data);

    // sync initial state
    pr_debug("  Synchronizing struct & device state\n");
    spcd_set_state(spcd_data);
    spcd_read_state(spcd_data);

    // Associate sysfs attribute groups.
    ret = sysfs_create_groups(&pdev->dev.kobj, spcd_groups);
    if (ret) {
        dev_err(dev, "sysfs creation failed\n");
        return ret;
    }

    // Now that everything is setup and initialized, request IRQs and assign handlers.
    ret = devm_request_threaded_irq(dev, spcd_data->irq_12v_status, NULL, spcd_handle_status_12v, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "spcd_12v_status", spcd_data);
    if (ret == -ENOSYS) {
        return -EPROBE_DEFER;
    }
    if (ret) {
        dev_err(&pdev->dev, "couldn't request irq %s %d: %d\n", "irq_12v_status", spcd_data->irq_12v_status, ret);
        return ret;
    }

    ret = devm_request_threaded_irq(dev, spcd_data->irq_failsafe_status, NULL, spcd_handle_failsafe_status_irq, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "spcd_failsafe_status", spcd_data);
    if (ret == -ENOSYS) {
        return -EPROBE_DEFER;
    }
    if (ret) {
        dev_err(&pdev->dev, "couldn't request irq %s %d: %d\n", "irq_failsafe_status", spcd_data->irq_failsafe_status, ret);
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

    // Then startup the heartbeat timer.
    hrtimer_init(&spcd_data->cpu_heartbeat_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    spcd_data->cpu_heartbeat_timer.function = cpu_heartbeat_timer_callback;
    hrtimer_start(&spcd_data->cpu_heartbeat_timer, spcd_data->cpu_heartbeat_period, HRTIMER_MODE_REL);

    // Setup the valve timer.
    hrtimer_init(&spcd_data->valve_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    spcd_data->valve_timer.function = valve_timer_callback;

    // Character Device Support
    alloc_chrdev_region(&(spcd_data->cdev_num), 0, 1, SPCD_DEVICE_NAME);
    spcd_class = class_create(THIS_MODULE, SPCD_CLASS);

    cdev_init(&(spcd_data->cdev), &spcd_fops);
    spcd_data->cdev.owner = THIS_MODULE;
    spcd_data->cdev_num = MKDEV(MAJOR(spcd_data->cdev_num), MINOR(spcd_data->cdev_num));
    cdev_add(&(spcd_data->cdev), spcd_data->cdev_num, 1);

    // Create the device node /dev/spcd
    device_create(spcd_class,
                  NULL, // no parent
                  spcd_data->cdev_num,
                  NULL, // no additional data
                  SPCD_DEVICE_NAME "%d", 0);

    return ret;
}

static void spcd_shutdown(struct platform_device *pdev) {
    struct spcd_data *spcd_data = platform_get_drvdata(pdev);

    pr_debug(" %s\n", __FUNCTION__);

    // Cancel any pending timers.
    hrtimer_cancel(&spcd_data->valve_timer);
    hrtimer_cancel(&spcd_data->cpu_heartbeat_timer);

    // Force off the PWMs.
    spcd_data->blower_state.duty_cycle = 0;
    pwm_apply_state(spcd_data->pwmd_blower, &(spcd_data->blower_state));
    spcd_data->valve_state.duty_cycle = 0;
    pwm_apply_state(spcd_data->pwmd_valve, &(spcd_data->valve_state));

    return;
}

static int spcd_remove(struct platform_device *pdev) {
    struct spcd_data *spcd_data = platform_get_drvdata(pdev);

    pr_debug(" %s\n", __FUNCTION__);

    spcd_shutdown(pdev);

    // Free any mem allocated for valve_states.
    if (spcd_data->valve_states != NULL) {
        kfree(spcd_data->valve_states);
    }

    device_destroy(spcd_class, spcd_data->cdev_num);
    class_destroy(spcd_class);

    sysfs_remove_group(&pdev->dev.kobj, &spcd_group);
    unregister_chrdev_region(spcd_data->cdev_num, 1);

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
        .shutdown = spcd_shutdown,
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
