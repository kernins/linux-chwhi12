/*
 *  Driver for Goodix Touchscreens
 *
 *  Copyright (c) 2014 Red Hat Inc.
 *  Copyright (c) 2015 K. Merker <merker@debian.org>
 *
 *  This code is based on gt9xx.c authored by andrew@goodix.com:
 *
 *  2010 - 2012 Goodix Technology.
 */

/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; version 2 of the License.
 */

#define DEBUG

#include <linux/kernel.h>
#include <linux/dmi.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/acpi.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <asm/unaligned.h>

struct goodix_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	int abs_x_max;
	int abs_y_max;
	bool swapped_x_y;
	bool inverted_x;
	bool inverted_y;
	unsigned int max_touch_num;
	unsigned int int_trigger_type;
	int cfg_len;
	struct gpio_desc *gpiod_int;
	struct gpio_desc *gpiod_rst;
	bool inverted_gpios;
	u16 id;
	u16 version;
	const char *cfg_name;
	struct completion firmware_loading_complete;
	unsigned long irq_flags;
	bool suspended;
	atomic_t inpdev_busy;			//whether input_dev is currently being consumed by some userspace proc(s)
	struct mutex mutex;				//Protects power management calls and access to suspended flag 
};

#define GOODIX_GPIO_INT_NAME		"irq"
#define GOODIX_GPIO_RST_NAME		"reset"

#define GOODIX_MAX_HEIGHT			4096
#define GOODIX_MAX_WIDTH			4096
#define GOODIX_INT_TRIGGER			1
#define GOODIX_CONTACT_SIZE		8
#define GOODIX_MAX_CONTACTS		10

#define GOODIX_CONFIG_MAX_LENGTH	240
#define GOODIX_CONFIG_911_LENGTH	186
#define GOODIX_CONFIG_967_LENGTH	228

/* Register defines */
#define GOODIX_REG_COMMAND			0x8040
#define GOODIX_CMD_SCREEN_OFF		0x05

#define GOODIX_READ_COOR_ADDR		0x814E
#define GOODIX_REG_CONFIG_DATA	0x8047
#define GOODIX_REG_ID				0x8140

#define RESOLUTION_LOC				1
#define MAX_CONTACTS_LOC			5
#define TRIGGER_LOC					6

#define GOODIX_AUTOSUSPEND_DELAY_MS	2000


static const unsigned long goodix_irq_flags[] = {
	IRQ_TYPE_EDGE_RISING,
	IRQ_TYPE_EDGE_FALLING,
	IRQ_TYPE_LEVEL_LOW,
	IRQ_TYPE_LEVEL_HIGH,
};

static ushort do_reset = 1;
module_param(do_reset, ushort, S_IRUGO);
MODULE_PARM_DESC(do_reset, "1 = do hard chip reset only (default), 2 = #1 + load FW afterwards, 0 - no reset");

/*
 * Those tablets have their coordinates origin at the bottom right
 * of the tablet, as if rotated 180 degrees
 */
static const struct dmi_system_id rotated_screen[] = {
#if defined(CONFIG_DMI) && defined(CONFIG_X86)
	{
		.ident = "WinBook TW100",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "WinBook"),
			DMI_MATCH(DMI_PRODUCT_NAME, "TW100")
		}
	},
	{
		.ident = "WinBook TW700",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "WinBook"),
			DMI_MATCH(DMI_PRODUCT_NAME, "TW700")
		},
	},
#endif
	{}
};

/*
 * Some platforms specify the gpio pins for interrupt and reset properly
 * in ACPI, but do not describe them using _DSD properties.
 */
static const struct dmi_system_id goodix_gpios_int_first_support[] = {
#if defined(CONFIG_DMI) && defined(CONFIG_X86)
	{
		.ident = "Chuwi Hi12",
		.matches = {
			DMI_MATCH(DMI_BOARD_VENDOR, "Hampoo"),
			DMI_MATCH(DMI_BOARD_NAME, "Cherry Trail CR")
		}
	},
#endif
	{}
};


static void goodix_dump_gpio_state(struct goodix_ts_data *ts, const char *realm)
{
	if (ts->gpiod_rst) {
		dev_dbg(&ts->client->dev, "[%s] GPIO-RST: dir %u / val %u / valRaw %u\n", 
			realm,
			gpiod_get_direction(ts->gpiod_rst), 
			gpiod_get_value(ts->gpiod_rst), 
			gpiod_get_raw_value(ts->gpiod_rst)
		);
	} else {
		dev_dbg(&ts->client->dev, "[%s] GPIO-RST: not available\n", realm);
	}

	if (ts->gpiod_int) {
		dev_dbg(&ts->client->dev, "[%s] GPIO-INT: dir %u / val %u / valRaw %u\n", 
			realm,
			gpiod_get_direction(ts->gpiod_int), 
			gpiod_get_value(ts->gpiod_int), 
			gpiod_get_raw_value(ts->gpiod_int)
		);
	} else {
		dev_dbg(&ts->client->dev, "[%s] GPIO-INT: not available\n", realm);
	}
}


/**
 * goodix_i2c_read - read data from a register of the i2c slave device.
 *
 * @client: i2c device.
 * @reg: the register to read from.
 * @buf: raw write data buffer.
 * @len: length of the buffer to write
 */
static int goodix_i2c_read(struct i2c_client *client, u16 reg, u8 *buf, int len)
{
	struct i2c_msg msgs[2];
	u16 wbuf = cpu_to_be16(reg);
	int ret;

	msgs[0].flags = 0;
	msgs[0].addr  = client->addr;
	msgs[0].len   = 2;
	msgs[0].buf   = (u8 *)&wbuf;

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = client->addr;
	msgs[1].len   = len;
	msgs[1].buf   = buf;

	ret = i2c_transfer(client->adapter, msgs, 2);
	return ret < 0 ? ret : (ret != ARRAY_SIZE(msgs) ? -EIO : 0);
}

/**
 * goodix_i2c_write - write data to a register of the i2c slave device.
 *
 * @client: i2c device.
 * @reg: the register to write to.
 * @buf: raw data buffer to write.
 * @len: length of the buffer to write
 */
static int goodix_i2c_write(struct i2c_client *client, u16 reg, const u8 *buf,
			    unsigned len)
{
	u8 *addr_buf;
	struct i2c_msg msg;
	int ret;

	addr_buf = kmalloc(len + 2, GFP_KERNEL);
	if (!addr_buf)
		return -ENOMEM;

	addr_buf[0] = reg >> 8;
	addr_buf[1] = reg & 0xFF;
	memcpy(&addr_buf[2], buf, len);

	msg.flags = 0;
	msg.addr = client->addr;
	msg.buf = addr_buf;
	msg.len = len + 2;

	ret = i2c_transfer(client->adapter, &msg, 1);
	kfree(addr_buf);
	return ret < 0 ? ret : (ret != 1 ? -EIO : 0);
}

static int goodix_i2c_write_u8(struct i2c_client *client, u16 reg, u8 value)
{
	return goodix_i2c_write(client, reg, &value, sizeof(value));
}

static int goodix_get_cfg_len(u16 id)
{
	switch (id) {
	case 911:
	case 9271:
	case 9110:
	case 9111: //chuwi hi12 tablet
	case 927:
	case 928:
		return GOODIX_CONFIG_911_LENGTH;

	case 912:
	case 967:
		return GOODIX_CONFIG_967_LENGTH;

	default:
		return GOODIX_CONFIG_MAX_LENGTH;
	}
}


static int goodix_rpm_enable(struct device *dev)
{
	int error;

	dev_dbg(dev, "Activating RuntimePM\n");
	
	//PM state must be set before pm_runtime_enable()
	error = pm_runtime_set_active(dev);
	if (error) {
		dev_err(dev, "RPM: failed to set device active: %d\n", error);
		return error;
	}

	pm_runtime_set_autosuspend_delay(dev, GOODIX_AUTOSUSPEND_DELAY_MS);
	pm_runtime_use_autosuspend(dev);

	pm_runtime_enable(dev);
	return 0;
}

static void goodix_rpm_disable(struct device *dev)
{
	if (pm_runtime_enabled(dev)) {
		/* First, resume device if there is a request to do so pending, cancel any other pending PM requests,
	 	 * and wait for PM ops currently in progress to complete before disabling RPM callbacks
	 	 */
		pm_runtime_barrier(dev);
		pm_runtime_disable(dev);

		pm_runtime_dont_use_autosuspend(dev);
		pm_runtime_set_suspended(dev);

		/* There is a race between inp_dev close() callback and rpm_disable() being called in remove(),
		 * usually resulting in goodix_inpdev_close() being called after rpm is already disabled.
		 * As open()/close() callbacks are invoked for the very first/last consumers only, 
		 * there could be at most 1 "ref" held due to them. Releasing it now.
		 * Calling pm_runtime_put_noidle() while dev->power.usage_count==0 is harmless
		 */
		pm_runtime_put_noidle(dev);
	}
}

static int goodix_rpm_hold_active(struct device *dev)
{
	int error;

	if (pm_runtime_enabled(dev)) {
		/* increments usage counter and calls pm_runtime_resume(), 
	 	 * returns its result (1 if already active)
	 	 */
		error = pm_runtime_get_sync(dev);
		if (error < 0) {
			pm_runtime_put_noidle(dev);
			dev_err(dev, "RPM: pm_runtime_get_sync() failed: %d\n", error);
			return error;
		} else {
			dev_dbg(dev, "RPM: holding device active: refCnt %d", atomic_read(&dev->power.usage_count));
		}
	}
	return 0;
}

static int goodix_rpm_release_active(struct device *dev)
{
	int error;

	if (pm_runtime_enabled(dev)) {
		pm_runtime_mark_last_busy(dev);
		error = pm_runtime_put_autosuspend(dev);
		if (error < 0) {
			dev_err(dev, "RPM: pm_runtime_put_autosuspend() failed: %d\n", error);
			return error;
		} else {
			dev_dbg(dev, "RPM: released device: refCnt %d", atomic_read(&dev->power.usage_count));
		}
	}
	return 0;
}


static int goodix_ts_read_input_report(struct goodix_ts_data *ts, u8 *data)
{
	int touch_num;
	int error;

	error = goodix_i2c_read(ts->client, GOODIX_READ_COOR_ADDR, data,
				GOODIX_CONTACT_SIZE + 1);
	if (error) {
		dev_err(&ts->client->dev, "I2C transfer error: %d\n", error);
		return error;
	}

	if (!(data[0] & 0x80))
		return -EAGAIN;

	touch_num = data[0] & 0x0f;
	if (touch_num > ts->max_touch_num)
		return -EPROTO;

	if (touch_num > 1) {
		data += 1 + GOODIX_CONTACT_SIZE;
		error = goodix_i2c_read(ts->client,
					GOODIX_READ_COOR_ADDR +
						1 + GOODIX_CONTACT_SIZE,
					data,
					GOODIX_CONTACT_SIZE * (touch_num - 1));
		if (error)
			return error;
	}

	dev_dbg(&ts->client->dev, "goodix_ts_read_input_report(): Touch num %u", touch_num);

	return touch_num;
}

static void goodix_ts_report_touch(struct goodix_ts_data *ts, u8 *coor_data)
{
	int id = coor_data[0] & 0x0F;
	int input_x = get_unaligned_le16(&coor_data[1]);
	int input_y = get_unaligned_le16(&coor_data[3]);
	int input_w = get_unaligned_le16(&coor_data[5]);

	/* Inversions have to happen before axis swapping */
	if (ts->inverted_x)
		input_x = ts->abs_x_max - input_x;
	if (ts->inverted_y)
		input_y = ts->abs_y_max - input_y;
	if (ts->swapped_x_y)
		swap(input_x, input_y);

	input_mt_slot(ts->input_dev, id);
	input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
	input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, input_w);
}

/**
 * goodix_process_events - Process incoming events
 *
 * @ts: our goodix_ts_data pointer
 *
 * Called when the IRQ is triggered. Read the current device state, and push
 * the input events to the user space.
 */
static void goodix_process_events(struct goodix_ts_data *ts)
{
	u8  point_data[1 + GOODIX_CONTACT_SIZE * GOODIX_MAX_CONTACTS];
	int touch_num;
	int i;

	touch_num = goodix_ts_read_input_report(ts, point_data);
	if (touch_num < 0)
		return;

	/*
	 * Bit 4 of the first byte reports the status of the capacitive Windows/Home button.
	 *
	 * Note: According to the datasheet, it could be that it actually reports OR-ed state of all buttons,
	 * and individual button states are at GOODIX_READ_COOR_ADDR + (1 + GOODIX_CONTACT_SIZE * GOODIX_MAX_CONTACTS), 
	 * i.e. right after last touch-point's data block. In the datasheet that reg is named 'keyvalue'
	 */
	input_report_key(ts->input_dev, KEY_LEFTMETA, !!(point_data[0] & BIT(4)));

	for (i = 0; i < touch_num; i++)
		goodix_ts_report_touch(ts,
				&point_data[1 + GOODIX_CONTACT_SIZE * i]);

	input_mt_sync_frame(ts->input_dev);
	input_sync(ts->input_dev);
}

/**
 * goodix_ts_irq_handler - The IRQ handler
 *
 * @irq: interrupt number.
 * @dev_id: private data pointer.
 */
static irqreturn_t goodix_ts_irq_handler(int irq, void *dev_id)
{
	struct goodix_ts_data *ts = (struct goodix_ts_data *)dev_id;

	dev_dbg(&ts->client->dev, "goodix_ts_irq_handler(%u) / users %u\n", irq, ts->input_dev->users);
	//goodix_dump_gpio_state(ts, "irq-hndl");

	goodix_process_events(ts);

	if (goodix_i2c_write_u8(ts->client, GOODIX_READ_COOR_ADDR, 0) < 0)
		dev_err(&ts->client->dev, "I2C write end_cmd error\n");

	return IRQ_HANDLED;
}

static void goodix_free_irq(struct goodix_ts_data *ts)
{
	dev_dbg(&ts->client->dev, "goodix_free_irq(%u)\n", ts->client->irq);
	devm_free_irq(&ts->client->dev, ts->client->irq, ts);
}

static int goodix_request_irq(struct goodix_ts_data *ts)
{
	dev_dbg(&ts->client->dev, "goodix_request_irq(%u), type %u\n", ts->client->irq, irq_get_trigger_type(ts->client->irq));
	goodix_dump_gpio_state(ts, "req-irq");

	return devm_request_threaded_irq(&ts->client->dev, ts->client->irq,
					 NULL, goodix_ts_irq_handler,
					 ts->irq_flags, ts->client->name, ts);
}

static int goodix_read_cfg(struct goodix_ts_data *ts, u8 *buf)
{
	int error;

#if defined(DEBUG)
	char hexdump[GOODIX_CONFIG_MAX_LENGTH * 2 + 1];
	int i;
#endif

	error = goodix_i2c_read(ts->client, GOODIX_REG_CONFIG_DATA, buf, ts->cfg_len);
	if (error) {
		dev_warn(&ts->client->dev, "read_config() failed: %d\n",  error);
	} else {
#if defined(DEBUG)
		for(i=0; i<ts->cfg_len; i++) scnprintf(hexdump + i*2, (GOODIX_CONFIG_MAX_LENGTH - i)*2 + 1, "%02X", buf[i]);
		dev_dbg(&ts->client->dev, "Read CFG data from IC: %s\n", hexdump);
#endif
	}

	return error;
}

/**
 * goodix_check_cfg - Checks if config fw is valid
 *
 * @ts: goodix_ts_data pointer
 * @cfg: firmware config data
 */
static int goodix_check_cfg(struct goodix_ts_data *ts,
			    const struct firmware *cfg)
{
	int i, raw_cfg_len;
	u8 check_sum = 0;

	if (cfg->size > GOODIX_CONFIG_MAX_LENGTH) {
		dev_err(&ts->client->dev,
			"The length of the config fw is not correct");
		return -EINVAL;
	}

	raw_cfg_len = cfg->size - 2;
	for (i = 0; i < raw_cfg_len; i++)
		check_sum += cfg->data[i];
	check_sum = (~check_sum) + 1;
	if (check_sum != cfg->data[raw_cfg_len]) {
		dev_err(&ts->client->dev,
			"The checksum of the config fw is not correct");
		return -EINVAL;
	}

	if (cfg->data[raw_cfg_len + 1] != 1) {
		dev_err(&ts->client->dev,
			"Config fw must have Config_Fresh register set");
		return -EINVAL;
	}

	return 0;
}

/**
 * goodix_send_cfg - Write fw config to device
 *
 * @ts: goodix_ts_data pointer
 * @cfg: config firmware to write to device
 */
static int goodix_send_cfg(struct goodix_ts_data *ts,
			   const struct firmware *cfg)
{
	int error;

	error = goodix_check_cfg(ts, cfg);
	if (error)
		return error;

	error = goodix_i2c_write(ts->client, GOODIX_REG_CONFIG_DATA, cfg->data,
				 cfg->size);
	if (error) {
		dev_err(&ts->client->dev, "Failed to write config data: %d",
			error);
		return error;
	}
	dev_dbg(&ts->client->dev, "Config sent successfully.");

	/* Let the firmware reconfigure itself, so sleep for 10ms */
	usleep_range(10000, 11000);

	return 0;
}

static int goodix_int_sync(struct goodix_ts_data *ts)
{
	int error;

	error = gpiod_direction_output(ts->gpiod_int, 0 ^ ts->inverted_gpios);
	if (error)
		return error;

	msleep(50);				//T5: 50ms

	goodix_dump_gpio_state(ts, "intsync-be4-inp");

	error = gpiod_direction_input(ts->gpiod_int);
	if (error)
		return error;

	return 0;
}

/**
 * goodix_reset - Reset device during power on
 *
 * @ts: goodix_ts_data pointer
 */
static int goodix_reset(struct goodix_ts_data *ts)
{
	/*	Chuwi-Hi12 notes
	 *		_DSD is missing for TCS1 device, however INT gpio could still be aquired via acpi_dev_gpio_irq_get()
	 *		which internally uses acpi_get_gpiod_by_index() to aquire gpio desc by _CRS idx
	 *
	 * 	RST is effectively ActiveHigh. Either there is a mosfet in between, or GT9111 has ActiveHigh reset, which is unlikely.
	 *		I.e. set_raw_value(1) means put the IC into reset state, set_raw_value(0) - release reset and let the IC to go to active state
	 *
	 *		INT is ActiveHigh from output perspective, i.e. set_raw_value(1) results in HI level to be seen by GT IC
	 *		However ACPI GpioInt() declares it as ActiveLow, so this line also requires inversion.
	 *
	 *		Attempts to override that AL flag via acpi_gpio_mapping gave no results, it is still AL no matter acpi_gpio_params.AcliveLow value
	 *		Should some clean method to override it be found, don't forget to check IRQ trigType, it is initially derived from EgdeSpec+LvlSpec and so may change to edge_rising
	 * 	For now simply working around that inversion with ts->inverted_gpios
	 *
	 *		TODO: dig deeper into gpiolib and i2c-core
	 *		TODO: what is the reason of those (always exactly 4) spurious interrupts generated right after chip reset / wakeup?
	 */

	int error;

	goodix_dump_gpio_state(ts, "be4 HR");

	//initial reset state as seen by GT IC: RST=0 (reset), INT=0
	error = gpiod_direction_output(ts->gpiod_int, 0 ^ ts->inverted_gpios);
	if (error)
		return error;
	error = gpiod_direction_output(ts->gpiod_rst, 0 ^ ts->inverted_gpios);
	if (error)
		return error;
	usleep_range(11000, 20000); //>10ms

	goodix_dump_gpio_state(ts, "HR-start");

	//selecting I2C addr: HIGH: 0x28/0x29, LOW: 0xBA/0xBB
	//state as seen by GT IC: RST=0 (reset), INT=1 for 0x14, 0 otherwise
	gpiod_set_value(ts->gpiod_int, (ts->client->addr==0x14) ^ ts->inverted_gpios);
	usleep_range(150, 2000);		/* T3: > 100us */

	//releasing RST#
	//state as seen by GT IC: RST=1 (IC active), INT=1 for 0x14, 0 otherwise
	gpiod_set_value(ts->gpiod_rst, 1 ^ ts->inverted_gpios);
	usleep_range(6000, 10000);		/* T4: > 5ms */

	goodix_dump_gpio_state(ts, "int1-rst1");

	/* Further sequence is unclear. 
	 * Some datasheets (e.g. GT968) specify switching straight to input at this point, letting GT IC to drive INT line,
	 * while others (e.g. GT911) show additional OUT-LOW - msleep(50) sequence for INT-pin before switching it to input.
	 * Hi12's GT9111 seem to require the latter approach
	 */

	//finishing addr sel
	//state as seen by GT IC: RST=1 (IC active), INT=0
	//50ms delay, INT changed to input
	error = goodix_int_sync(ts);
	if (error)
		return error;

	//not really necessary (however may save a tiny-tiny-tiny-bit of pwr),
	//but doesn't hurt either - there must be pull-up on GT RST# pin anyway
	error = gpiod_direction_input(ts->gpiod_rst);
	if (error)
		return error;

	return 0;
}

static ssize_t goodix_config_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = i2c_verify_client(dev);
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	u8 config[GOODIX_CONFIG_MAX_LENGTH];
	int error, count = 0, i;

	//TODO: add mutex?

	error = goodix_rpm_hold_active(&ts->client->dev);
	if (error)
		return error;

	error = goodix_read_cfg(ts, config);
	goodix_rpm_release_active(&ts->client->dev);

	if (error) {
		return error;
	}

	for (i = 0; i < ts->cfg_len; i++)
		count += scnprintf(buf + count, PAGE_SIZE - count, "%02X", config[i]);

	return count;
}

static DEVICE_ATTR(config, S_IRUGO, goodix_config_show, NULL);

static struct attribute *goodix_attrs[] = {
	&dev_attr_config.attr,
	NULL
};

static const struct attribute_group goodix_attr_group = {
	.attrs = goodix_attrs,
};


static const struct acpi_gpio_params goodix_first_gpio = { 0, 0, true }; 	//declared as ActiveLow in ACPI GpioInt(), setting it false here has no effect
static const struct acpi_gpio_params goodix_second_gpio = { 1, 0, false }; //this one has HW inversion (or GT9111 has active-hight RST), so physical-high means reset-active

static const struct acpi_gpio_mapping goodix_gpios_int_first[] = {
	{ GOODIX_GPIO_INT_NAME "-gpios", &goodix_first_gpio, 1 },
	{ GOODIX_GPIO_RST_NAME "-gpios", &goodix_second_gpio, 1 },
	{ },
};

/**
 * goodix_get_gpio_config - Get GPIO config from ACPI/DT
 *
 * @ts: goodix_ts_data pointer
 */
static int goodix_get_gpio_config(struct goodix_ts_data *ts)
{
	int error;
	struct device *dev;
	struct gpio_desc *gpiod;

	if (!ts->client)
		return -EINVAL;
	dev = &ts->client->dev;

	if (dmi_check_system(goodix_gpios_int_first_support) && ACPI_HANDLE(dev)) {
		error = devm_acpi_dev_add_driver_gpios(dev, goodix_gpios_int_first);
		if (error)
			return error;
		ts->inverted_gpios = true;
		dev_dbg(dev, "Applying gpios quirk\n");
	}

	/* Get the interrupt GPIO pin number */
	gpiod = devm_gpiod_get_optional(dev, GOODIX_GPIO_INT_NAME, GPIOD_IN);
	if (IS_ERR(gpiod)) {
		error = PTR_ERR(gpiod);
		dev_dbg(dev, "Get GPIO [%s] err: %d\n", GOODIX_GPIO_INT_NAME, error);
		if (error != -EPROBE_DEFER)
			dev_dbg(dev, "Failed to get %s GPIO: %d\n",
				GOODIX_GPIO_INT_NAME, error);
		//goto out_err;
		return error;
	}

	ts->gpiod_int = gpiod;

	/* Get the reset line GPIO pin number */
	gpiod = devm_gpiod_get_optional(dev, GOODIX_GPIO_RST_NAME, GPIOD_IN);
	if (IS_ERR(gpiod)) {
		error = PTR_ERR(gpiod);
		dev_dbg(dev, "Get GPIO [%s] err: %d\n", GOODIX_GPIO_RST_NAME, error);
		if (error != -EPROBE_DEFER)
			dev_dbg(dev, "Failed to get %s GPIO: %d\n",
				GOODIX_GPIO_RST_NAME, error);
		//goto out_err;
		return error;
	}

	ts->gpiod_rst = gpiod;

	if (ts->gpiod_int) {
		dev_dbg(dev, "GPIO-INT can-sleep: %u / AL %u\n", 
			gpiod_cansleep(ts->gpiod_int), 
			gpiod_is_active_low(ts->gpiod_int)
		);
	}
	if (ts->gpiod_rst) {
		dev_dbg(dev, "GPIO-RST can-sleep: %u / AL %u\n", 
			gpiod_cansleep(ts->gpiod_rst), 
			gpiod_is_active_low(ts->gpiod_rst)
		);
	}
	goodix_dump_gpio_state(ts, "just-ACQ");

	return 0;

//let devres to handle it
/*out_err:
	if (ACPI_HANDLE(dev))
		acpi_dev_remove_driver_gpios(ACPI_COMPANION(dev));
	return error; */
}

/**
 * goodix_read_version - Read goodix touchscreen version
 *
 * @ts: our goodix_ts_data pointer
 */
static int goodix_read_version(struct goodix_ts_data *ts)
{
	int error, attempt = 0;
	u8 buf[6];
	char id_str[5];

	while (++attempt <= 3) {
		error = goodix_i2c_read(ts->client, GOODIX_REG_ID, buf, sizeof(buf));
		if (!error) break;
		
		dev_err(&ts->client->dev, "read_version() attempt#%u failed: %d\n", attempt, error);
		usleep_range(10000, 30000);
	}

	if (!error) {
		memcpy(id_str, buf, 4);
		id_str[4] = 0;

		if (kstrtou16(id_str, 10, &ts->id)) ts->id = 0x1001;
		ts->version = get_unaligned_le16(&buf[4]);

		dev_info(&ts->client->dev, "ID %d, version: %04x\n", ts->id, ts->version);
	}
	return error;
}

static int goodix_inpdev_open(struct input_dev *input_dev)
{
	struct goodix_ts_data *ts = input_get_drvdata(input_dev);
	int error;

	dev_dbg(&ts->client->dev, "goodix_inpdev_open()\n");

	if (do_reset == 2 && ts->gpiod_int && ts->gpiod_rst)
		wait_for_completion(&ts->firmware_loading_complete);

	error = goodix_rpm_hold_active(&ts->client->dev);
	if (error)
		return error;
	
	/* open/close callbacks are invoked for
	 * the very first/last consumers only
	 */
	atomic_set(&ts->inpdev_busy, 1);
	return 0;
}

static void goodix_inpdev_close(struct input_dev *input_dev)
{
	struct goodix_ts_data *ts = input_get_drvdata(input_dev);

	dev_dbg(&ts->client->dev, "goodix_inpdev_close()\n");

	goodix_rpm_release_active(&ts->client->dev);
	atomic_set(&ts->inpdev_busy, 0);
}

/**
 * goodix_request_input_dev - Allocate, populate and register the input device
 *
 * @ts: our goodix_ts_data pointer
 *
 * Must be called during probe
 */
static int goodix_request_input_dev(struct goodix_ts_data *ts)
{
	int error;

	ts->input_dev = devm_input_allocate_device(&ts->client->dev);
	if (!ts->input_dev) {
		dev_err(&ts->client->dev, "Failed to allocate input device.");
		return -ENOMEM;
	}

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,
			     0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,
			     0, ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);

	input_mt_init_slots(ts->input_dev, ts->max_touch_num,
			    INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);

	ts->input_dev->name = "Goodix Capacitive TouchScreen";
	ts->input_dev->phys = "input/ts";
	ts->input_dev->id.bustype = BUS_I2C;

	ts->input_dev->id.vendor = 0x0416;
	ts->input_dev->id.product = ts->id;
	ts->input_dev->id.version = ts->version;

	ts->input_dev->open = goodix_inpdev_open;
	ts->input_dev->close = goodix_inpdev_close;
	input_set_drvdata(ts->input_dev, ts);

	/* Capacitive Windows/Home button on some devices */
	input_set_capability(ts->input_dev, EV_KEY, KEY_LEFTMETA);

	error = input_register_device(ts->input_dev);
	if (error) {
		dev_err(&ts->client->dev,
			"Failed to register input device: %d", error);
		return error;
	}

	return 0;
}

/**
 * goodix_configure_dev - Finish device initialization
 *
 * @ts: our goodix_ts_data pointer
 *
 * Must be called from probe to finish initialization of the device.
 * Contains the common initialization code for both devices that
 * declare gpio pins and devices that do not. It is either called
 * directly from probe or from request_firmware_wait callback.
 */
static int goodix_configure_dev(struct goodix_ts_data *ts)
{
	u8 config[GOODIX_CONFIG_MAX_LENGTH];
	int error;

	error = goodix_read_cfg(ts, config);
	if (!error) {
		ts->abs_x_max = get_unaligned_le16(&config[RESOLUTION_LOC]);
		ts->abs_y_max = get_unaligned_le16(&config[RESOLUTION_LOC + 2]);
		ts->int_trigger_type = config[TRIGGER_LOC] & 0x03;
		ts->max_touch_num = config[MAX_CONTACTS_LOC] & 0x0f;
	}
	
	if (error || !ts->abs_x_max || !ts->abs_y_max || !ts->max_touch_num) {
		dev_err(&ts->client->dev, "Read config failed (%d) or invalid data received, using defaults\n", error);
		ts->abs_x_max = GOODIX_MAX_WIDTH;
		ts->abs_y_max = GOODIX_MAX_HEIGHT;
		ts->int_trigger_type = GOODIX_INT_TRIGGER;
		ts->max_touch_num = GOODIX_MAX_CONTACTS;
	}

	ts->swapped_x_y = device_property_read_bool(&ts->client->dev, "touchscreen-swapped-x-y");
	if (ts->swapped_x_y) swap(ts->abs_x_max, ts->abs_y_max);

	if (dmi_check_system(rotated_screen)) {
		dev_dbg(&ts->client->dev, "Applying '180 degrees rotated screen' quirk\n");
		ts->inverted_x = true;
		ts->inverted_y = true;
	} else {
		ts->inverted_x = device_property_read_bool(&ts->client->dev, "touchscreen-inverted-x");
		ts->inverted_y = device_property_read_bool(&ts->client->dev, "touchscreen-inverted-y");
	}

	error = goodix_request_input_dev(ts);
	if (error)
		return error;

	ts->irq_flags = goodix_irq_flags[ts->int_trigger_type] | IRQF_ONESHOT;
	dev_info(&ts->client->dev, "IRQ trig type %u, flags %lu\n", ts->int_trigger_type, ts->irq_flags);
	error = goodix_request_irq(ts); //TODO: request only after inpdev open()?
	if (error) {
		dev_err(&ts->client->dev, "request IRQ failed: %d\n", error);
		return error;
	}

	return 0;
}

/**
 * goodix_config_cb - Callback to finish device init
 *
 * @ts: our goodix_ts_data pointer
 *
 * request_firmware_wait callback that finishes
 * initialization of the device.
 */
static void goodix_config_cb(const struct firmware *cfg, void *ctx)
{
	struct goodix_ts_data *ts = (struct goodix_ts_data *)ctx;
	int error;

	if (cfg) {
		/* send device configuration to the firmware */
		error = goodix_send_cfg(ts, cfg);
		if (error)
			goto err_release_cfg;
	}

	if (ts->gpiod_int && (goodix_rpm_enable(&ts->client->dev) == 0)) {
		goodix_rpm_hold_active(&ts->client->dev);
		goodix_configure_dev(ts);
		goodix_rpm_release_active(&ts->client->dev);
	} else {
		goodix_configure_dev(ts);
	}

err_release_cfg:
	release_firmware(cfg);
	complete_all(&ts->firmware_loading_complete);
}

//TODO: replace to probe_new(), struct i2c_device_id is unused
static int goodix_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct goodix_ts_data *ts;
	int error;

	//TODO: normalize do_reset based on gpio availability to simplify conds across the code

	dev_dbg(&client->dev, "I2C Address: 0x%02x, IRQ: %u / trigType %u\n", 
		client->addr, 
		client->irq, 
		irq_get_trigger_type(client->irq)
	);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C check functionality failed.\n");
		return -ENXIO;
	}

	ts = devm_kzalloc(&client->dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->client = client;
	i2c_set_clientdata(client, ts);
	init_completion(&ts->firmware_loading_complete);
	mutex_init(&ts->mutex);

	error = goodix_get_gpio_config(ts);
	if (error)
		return error;

	if (do_reset >= 1) {
		if (ts->gpiod_int && ts->gpiod_rst) {
			/* reset the controller */
			dev_dbg(&client->dev, "Hard-resetting device...");
			error = goodix_reset(ts);
			if (error) {
				dev_err(&client->dev, "Controller hard-reset failed.\n");
				return error;
			}
		} else {
			dev_warn(&client->dev, "Skipping controller hard-reset due to GPIO int/rst not available.\n");
		}
	} else {
		dev_dbg(&client->dev, "Skipping controller hard-reset due to do_reset=%u.\n", do_reset);
	}

	error = goodix_read_version(ts);
	if (error) {
		dev_err(&client->dev, "Read version failed.\n");
		return error;
	}

	ts->cfg_len = goodix_get_cfg_len(ts->id);

	if (do_reset == 2 && ts->gpiod_int && ts->gpiod_rst) {
		ts->cfg_name = devm_kasprintf(&client->dev, GFP_KERNEL, "goodix_%d_cfg.bin", ts->id);
		if (!ts->cfg_name)
			return -ENOMEM;

		dev_info(&client->dev, "Requesting firmware: %s, len %d\n", ts->cfg_name, ts->cfg_len);
		error = request_firmware_nowait(THIS_MODULE, true, ts->cfg_name,
						&client->dev, GFP_KERNEL, ts,
						goodix_config_cb);
		if (error) {
			dev_err(&client->dev,
				"Failed to invoke firmware loader: %d\n",
				error);
			return error;
		}
	} else {
		dev_dbg(&client->dev, "Skipping FW load, do_reset=%u.\n", do_reset);

		if (ts->gpiod_int && (goodix_rpm_enable(&ts->client->dev) == 0)) {
			goodix_rpm_hold_active(&ts->client->dev);
			error = goodix_configure_dev(ts);
			goodix_rpm_release_active(&ts->client->dev);
		} else {
			error = goodix_configure_dev(ts);
		}

		if (error)
			return error;
	}

	goodix_dump_gpio_state(ts, "prb done");

	error = sysfs_create_group(&client->dev.kobj, &goodix_attr_group);
	if (error) {
		dev_err(&client->dev, "Failed to create sysfs group: %d\n", error);
		return error;
	}
	return 0;
}

static int goodix_ts_remove(struct i2c_client *client)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "remove().\n");

	if (do_reset == 2 && ts->gpiod_int && ts->gpiod_rst)
		wait_for_completion(&ts->firmware_loading_complete);

	goodix_dump_gpio_state(ts, "rmv");

	//TODO: suspend device?

	goodix_rpm_disable(&ts->client->dev);
	dev_dbg(&client->dev, "RPM disabled.\n");
	
	sysfs_remove_group(&client->dev.kobj, &goodix_attr_group);

	//now dev-managed
	//if (ACPI_HANDLE(&ts->client->dev))
	//	acpi_dev_remove_driver_gpios(ACPI_COMPANION(&ts->client->dev));

	return 0;
}

static int __maybe_unused goodix_sleep(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev); //replace to verify_client?
	struct goodix_ts_data *ts = i2c_get_clientdata(client);
	int error;

	dev_dbg(dev, "goodix_sleep(%u)\n", ts->suspended);
	goodix_dump_gpio_state(ts, "PM-S");

	if (ts->gpiod_int) {
		if (do_reset == 2 && ts->gpiod_rst)
			wait_for_completion(&ts->firmware_loading_complete);

		mutex_lock(&ts->mutex);

		if (ts->suspended)
			goto out;

		/* Free IRQ as IRQ pin is used as output in the suspend sequence */
		goodix_free_irq(ts);

		/* Output LOW on the INT pin for 5 ms */
		error = gpiod_direction_output(ts->gpiod_int, 0 ^ ts->inverted_gpios);
		if (error) {
			dev_err(&ts->client->dev, "sleep(): failed to change gpio_int to out: %d\n", error);
			goto out_error;
		}

		usleep_range(5000, 6000);
		goodix_dump_gpio_state(ts, "PM-S-be4-cmd");

		error = goodix_i2c_write_u8(ts->client, GOODIX_REG_COMMAND, GOODIX_CMD_SCREEN_OFF);
		if (error) {
			dev_err(&ts->client->dev, "sleep(): screen off command failed: %d\n", error);
			gpiod_direction_input(ts->gpiod_int);
			error = -EAGAIN;
			goto out_error;
		}

		/*
	 	* The datasheet specifies that the interval between sending screen-off
	 	* command and wake-up should be longer than 58 ms. To avoid waking up
	 	* sooner, delay 58ms here.
	 	*/
		msleep(58);
	
		ts->suspended = true;
out:
		mutex_unlock(&ts->mutex);
	}
	return 0;

out_error:
	goodix_request_irq(ts);
	mutex_unlock(&ts->mutex);
	return error;
}

static int __maybe_unused goodix_wakeup(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev); //replace to verify_client?
	struct goodix_ts_data *ts = i2c_get_clientdata(client);
	int error = 0;

	dev_dbg(dev, "goodix_wakeup(%u)\n", ts->suspended);
	goodix_dump_gpio_state(ts, "PM-R");

	if (ts->gpiod_int) {
		mutex_lock(&ts->mutex);

		if (!ts->suspended)
			goto out;

		//Exit sleep mode by outputting HIGH level to INT pin for 2ms~5ms.
		error = gpiod_direction_output(ts->gpiod_int, 1 ^ ts->inverted_gpios);
		if (error) {
			dev_err(&ts->client->dev, "wakeup(): failed to change gpio_int to out: %d\n", error);
			goto out;
		}

		usleep_range(2000, 5000);

		goodix_dump_gpio_state(ts, "PM-R-INTHI");

		/* Despite datasheet doesn't specify OUT-LOW + msleep(50) sequence for wakeup
		 * it is actually necessary, at least on my ChuwiHi12 (GT9111)
		 */
		error = goodix_int_sync(ts);
		if (error) {
			dev_err(&ts->client->dev, "wakeup(): goodix_int_sync() failed: %d\n", error);
			goto out;
		}

		error = goodix_request_irq(ts);
		if (error) {
			dev_err(&ts->client->dev, "wakeup(): failed to aquire IRQ: %d\n", error);
			goto out;
		}

		ts->suspended = false;
out:
		mutex_unlock(&ts->mutex);
	}
	return error;
}

static int __maybe_unused goodix_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev); //replace to verify_client?
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	dev_dbg(dev, "goodix_resume(%d)\n", atomic_read(&ts->inpdev_busy));

	return atomic_read(&ts->inpdev_busy)? goodix_wakeup(dev) : 0;
}

static const struct dev_pm_ops goodix_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(goodix_sleep, goodix_resume)
	SET_RUNTIME_PM_OPS(goodix_sleep, goodix_wakeup, NULL)
};

static const struct i2c_device_id goodix_ts_id[] = {
	{ "GDIX1001:00", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, goodix_ts_id);

#ifdef CONFIG_ACPI
static const struct acpi_device_id goodix_acpi_match[] = {
	{ "GDIX1001", 0 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, goodix_acpi_match);
#endif

#ifdef CONFIG_OF
static const struct of_device_id goodix_of_match[] = {
	{ .compatible = "goodix,gt911" },
	{ .compatible = "goodix,gt9110" },
	{ .compatible = "goodix,gt9111" }, //chuwi hi12 tablet
	{ .compatible = "goodix,gt912" },
	{ .compatible = "goodix,gt927" },
	{ .compatible = "goodix,gt9271" },
	{ .compatible = "goodix,gt928" },
	{ .compatible = "goodix,gt967" },
	{ }
};
MODULE_DEVICE_TABLE(of, goodix_of_match);
#endif

static struct i2c_driver goodix_ts_driver = {
	.probe = goodix_ts_probe,
	.remove = goodix_ts_remove,
	.id_table = goodix_ts_id,
	.driver = {
		.name = "Goodix-TS",
		.acpi_match_table = ACPI_PTR(goodix_acpi_match),
		.of_match_table = of_match_ptr(goodix_of_match),
		.pm = &goodix_pm_ops,
	},
};
module_i2c_driver(goodix_ts_driver);

MODULE_AUTHOR("Benjamin Tissoires <benjamin.tissoires@gmail.com>");
MODULE_AUTHOR("Bastien Nocera <hadess@hadess.net>");
MODULE_DESCRIPTION("Goodix touchscreen driver");
MODULE_LICENSE("GPL v2");
