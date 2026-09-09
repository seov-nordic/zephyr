/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nordic_npm10xx_uicr

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/otp.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/math_extras.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(otp_npm10xx, CONFIG_OTP_LOG_LEVEL);

#define NPM10XX_OTP_TASKS           0xECU
#define NPM10XX_OTP_EVENTS_UICR_SET 0xEDU
#define NPM10XX_OTP_EVENTS_UICR_CLR 0xEEU
#define NPM10XX_OTP_PROGMODE        0xF1U
#define NPM10XX_OTP_LOCK            0xF2U
#define NPM10XX_OTP_READLOCK        0xF3U
#define NPM10XX_OTP_REQUEST         0xF4U
#define NPM10XX_OTP_ADDR0           0xF5U
#define NPM10XX_OTP_READDATA        0xF7U

/* TASKS (0xEC) */
#define NPM10XX_OTP_TASKS_PROG_Msk BIT(0)
#define NPM10XX_OTP_TASKS_READ_Msk BIT(1)

/* EVENTS_UICR_SET (0xED) / EVENTS_UICR_CLR (0xEE) */
#define NPM10XX_OTP_EVENTS_USERPROGMODE_Msk    BIT(0)
#define NPM10XX_OTP_EVENTS_USERPROGSUCC_Msk    BIT(1)
#define NPM10XX_OTP_EVENTS_USERLOCKERR_Msk     BIT(2)
#define NPM10XX_OTP_EVENTS_USERADDRERR_Msk     BIT(3)
#define NPM10XX_OTP_EVENTS_USERREADMODE_Msk    BIT(4)
#define NPM10XX_OTP_EVENTS_USERREAD_Msk        BIT(5)
#define NPM10XX_OTP_EVENTS_USERREADCOMPT_Msk   BIT(6)
#define NPM10XX_OTP_EVENTS_USERREADADDRERR_Msk BIT(7)
#define NPM10XX_OTP_EVENTS_ALL_Msk             0xFFU

/* PROGMODE (0xF1) */
#define NPM10XX_OTP_PROGMODE_STATUS_Msk BIT(0)
#define NPM10XX_OTP_PROGMODE_READ_Msk   BIT(1)

/* READLOCK (0xF3) */
#define NPM10XX_OTP_READLOCK_STATE BIT(0)

/* REQUEST (0xF4) */
#define NPM10XX_OTP_REQUEST_USERPROGMODE_Msk BIT(0)
#define NPM10XX_OTP_REQUEST_USERREADMODE_Msk BIT(1)

/* Number of user-programmable UICR bits and the packed byte size that holds them. */
#define NPM10XX_UICR_BITS 104U
#define NPM10XX_UICR_SIZE DIV_ROUND_UP(NPM10XX_UICR_BITS, 8U)

/* Bounded poll parameters for the OTP state-machine handshakes. */
#define NPM10XX_OTP_POLL_INTERVAL_US 100
#define NPM10XX_OTP_POLL_COUNT       1000U

struct otp_npm10xx_config {
	struct i2c_dt_spec i2c;
};

struct otp_npm10xx_data {
	struct k_mutex lock;
};

/* Poll a register until all bits in mask are set, or timeout. */
static int npm10xx_otp_poll_set(const struct i2c_dt_spec *i2c, uint8_t reg, uint8_t mask)
{
	for (uint32_t i = 0U; i < NPM10XX_OTP_POLL_COUNT; i++) {
		uint8_t val;
		int ret;

		ret = i2c_reg_read_byte_dt(i2c, reg, &val);
		if (ret < 0) {
			return ret;
		}

		if ((val & mask) == mask) {
			return 0;
		}

		k_busy_wait(NPM10XX_OTP_POLL_INTERVAL_US);
	}

	return -ETIMEDOUT;
}

/* Poll a register until any bit in mask is set, returning the read value in *out. */
static int npm10xx_otp_poll_any(const struct i2c_dt_spec *i2c, uint8_t reg, uint8_t mask,
				uint8_t *out)
{
	for (uint32_t i = 0U; i < NPM10XX_OTP_POLL_COUNT; i++) {
		uint8_t val;
		int ret;

		ret = i2c_reg_read_byte_dt(i2c, reg, &val);
		if (ret < 0) {
			return ret;
		}

		if ((val & mask) != 0U) {
			*out = val;
			return 0;
		}

		k_busy_wait(NPM10XX_OTP_POLL_INTERVAL_US);
	}

	return -ETIMEDOUT;
}

/* Poll a register until all bits in mask are cleared, or timeout. */
static int npm10xx_otp_poll_clear(const struct i2c_dt_spec *i2c, uint8_t reg, uint8_t mask)
{
	for (uint32_t i = 0U; i < NPM10XX_OTP_POLL_COUNT; i++) {
		uint8_t val;
		int ret;

		ret = i2c_reg_read_byte_dt(i2c, reg, &val);
		if (ret < 0) {
			return ret;
		}

		if ((val & mask) == 0U) {
			return 0;
		}

		k_busy_wait(NPM10XX_OTP_POLL_INTERVAL_US);
	}

	return -ETIMEDOUT;
}

static int npm10xx_uicr_do_read(const struct i2c_dt_spec *i2c, uint8_t *buf, uint8_t start,
				uint8_t len)
{
	int ret;
	uint8_t events, bit_addr;

	/* UICR reading sequence, ref. datasheet chapter 4.2.12 */
	/* 2. Power up the OTP memory in read mode */
	ret = i2c_reg_write_byte_dt(i2c, NPM10XX_OTP_REQUEST, NPM10XX_OTP_REQUEST_USERREADMODE_Msk);
	if (ret < 0) {
		return ret;
	}

	for (uint8_t i = 0U; i < len; i++) {
		bit_addr = start + i * 8U;

		/* 4. Clear events */
		ret = i2c_reg_write_byte_dt(i2c, NPM10XX_OTP_EVENTS_UICR_CLR,
					    NPM10XX_OTP_EVENTS_ALL_Msk);
		if (ret < 0) {
			goto exit_read_mode;
		}

		/* 5. Write start address */
		ret = i2c_reg_write_byte_dt(i2c, NPM10XX_OTP_ADDR0, bit_addr);
		if (ret < 0) {
			goto exit_read_mode;
		}

		/* 6. Activate UICR read */
		ret = i2c_reg_write_byte_dt(i2c, NPM10XX_OTP_TASKS, NPM10XX_OTP_TASKS_READ_Msk);
		if (ret < 0) {
			goto exit_read_mode;
		}

		/* 7. Poll events and errors. Once USERREAD=1, an 8-bit chunk has been read */
		ret = npm10xx_otp_poll_any(i2c, NPM10XX_OTP_EVENTS_UICR_SET,
					   NPM10XX_OTP_EVENTS_USERREAD_Msk |
						   NPM10XX_OTP_EVENTS_USERREADADDRERR_Msk,
					   &events);
		if (ret < 0) {
			LOG_ERR("UICR read chunk %zu timed out (%d)", i, ret);
			goto exit_read_mode;
		}

		if (events & NPM10XX_OTP_EVENTS_USERREADADDRERR_Msk) {
			LOG_ERR("UICR read address error at bit %u", bit_addr);
			ret = -EIO;
			goto exit_read_mode;
		}

		/* 8. Read READDATA */
		ret = i2c_reg_read_byte_dt(i2c, NPM10XX_OTP_READDATA, &buf[i]);
		if (ret < 0) {
			goto exit_read_mode;
		}

		/* 9. Poll USERREADCOMPT */
		ret = npm10xx_otp_poll_set(i2c, NPM10XX_OTP_EVENTS_UICR_SET,
					   NPM10XX_OTP_EVENTS_USERREADCOMPT_Msk);
		if (ret < 0) {
			LOG_ERR("UICR read completion timed out at chunk %zu (%d)", i, ret);
			goto exit_read_mode;
		}

		/* 10. Repeat incrementing the start address by 8 */
	}

	/* 11. Exit reading mode */
exit_read_mode: {
	int exit_ret = i2c_reg_write_byte_dt(i2c, NPM10XX_OTP_REQUEST, 0U);

	if (ret == 0) {
		ret = exit_ret;
	}
}

	return ret;
}

static int otp_npm10xx_read(const struct device *dev, off_t offset, void *data, size_t len)
{
	const struct otp_npm10xx_config *config = dev->config;
	struct otp_npm10xx_data *dev_data = dev->data;
	size_t end;
	int ret;

	if ((offset < 0) || size_add_overflow((size_t)offset, len, &end) ||
	    (end > NPM10XX_UICR_SIZE)) {
		LOG_ERR("UICR read out of bounds [0,%u)", NPM10XX_UICR_SIZE);
		return -EINVAL;
	}

	if (len == 0U) {
		return 0;
	}

	(void)k_mutex_lock(&dev_data->lock, K_FOREVER);
	ret = npm10xx_uicr_do_read(&config->i2c, data, (uint8_t)offset, (uint8_t)len);
	(void)k_mutex_unlock(&dev_data->lock);

	return ret;
}

#if defined(CONFIG_OTP_PROGRAM)
/* Program a single UICR bit at the given absolute bit address. */
static int npm10xx_uicr_bit_program(const struct i2c_dt_spec *i2c, uint8_t bit_addr)
{
	uint8_t events;
	int ret;

	ret = i2c_reg_write_byte_dt(i2c, NPM10XX_OTP_EVENTS_UICR_CLR, NPM10XX_OTP_EVENTS_ALL_Msk);
	if (ret < 0) {
		return ret;
	}

	ret = i2c_reg_write_byte_dt(i2c, NPM10XX_OTP_ADDR0, (uint8_t)(bit_addr & 0xFFU));
	if (ret < 0) {
		return ret;
	}

	if (IS_ENABLED(CONFIG_OTP_NPM10XX_DRY_RUN)) {
		LOG_INF("Dry run: would program UICR bit %u", bit_addr);
		return 0;
	}

	ret = i2c_reg_write_byte_dt(i2c, NPM10XX_OTP_TASKS, NPM10XX_OTP_TASKS_PROG_Msk);
	if (ret < 0) {
		return ret;
	}

	/* Wait for any programming result (success or error) and decode it. */
	ret = npm10xx_otp_poll_any(i2c, NPM10XX_OTP_EVENTS_UICR_SET,
				   NPM10XX_OTP_EVENTS_USERPROGSUCC_Msk |
					   NPM10XX_OTP_EVENTS_USERLOCKERR_Msk |
					   NPM10XX_OTP_EVENTS_USERADDRERR_Msk,
				   &events);
	if (ret < 0) {
		LOG_ERR("UICR bit %u: no programming result (%d)", bit_addr, ret);
		return ret;
	}

	if (events & NPM10XX_OTP_EVENTS_USERLOCKERR_Msk) {
		LOG_ERR("UICR bit %u: lock error", bit_addr);
		return -EACCES;
	}

	if (events & NPM10XX_OTP_EVENTS_USERADDRERR_Msk) {
		LOG_ERR("UICR bit %u: address error", bit_addr);
		return -EINVAL;
	}

	if (!(events & NPM10XX_OTP_EVENTS_USERPROGSUCC_Msk)) {
		LOG_ERR("UICR bit %u: programming failed (events 0x%02x)", bit_addr, events);
		return -EIO;
	}

	return 0;
}

static int otp_npm10xx_program(const struct device *dev, off_t offset, const void *data, size_t len)
{
	const struct otp_npm10xx_config *config = dev->config;
	struct otp_npm10xx_data *dev_data = dev->data;
	const uint8_t *src = data;
	size_t end;
	int ret;
	uint8_t reg;

	if ((offset < 0) || size_add_overflow((size_t)offset, len, &end) ||
	    (end > NPM10XX_UICR_SIZE)) {
		return -EINVAL;
	}

	if (len == 0U) {
		return 0;
	}

	(void)k_mutex_lock(&dev_data->lock, K_FOREVER);

	/* Check lock state before doing anything else */
	ret = i2c_reg_read_byte_dt(&config->i2c, NPM10XX_OTP_READLOCK, &reg);
	if (reg & NPM10XX_OTP_READLOCK_STATE) {
		LOG_ERR("UICR locked, programming not possible");
		ret = -EACCES;
		goto unlock;
	}

	/* Enter user program mode */
	ret = i2c_reg_write_byte_dt(&config->i2c, NPM10XX_OTP_REQUEST,
				    NPM10XX_OTP_REQUEST_USERPROGMODE_Msk);
	if (ret < 0) {
		goto unlock;
	}

	ret = npm10xx_otp_poll_set(&config->i2c, NPM10XX_OTP_PROGMODE,
				   NPM10XX_OTP_PROGMODE_STATUS_Msk);
	if (ret < 0) {
		LOG_ERR("UICR program mode not entered - is VBUS supplied? (%d)", ret);
		goto exit_prog_mode;
	}

	for (size_t byte = 0U; byte < len; byte++) {
		uint8_t val = src[byte];

		for (uint8_t bit = 0U; bit < 8U; bit++) {
			uint16_t bit_addr;

			if ((val & BIT(bit)) == 0U) {
				continue;
			}

			bit_addr = (uint8_t)((offset + byte) * 8U + bit);
			ret = npm10xx_uicr_bit_program(&config->i2c, bit_addr);
			if (ret < 0) {
				goto exit_prog_mode;
			}
		}
	}

exit_prog_mode: {
	int exit_ret;

	exit_ret = i2c_reg_write_byte_dt(&config->i2c, NPM10XX_OTP_REQUEST, 0U);
	if (ret == 0) {
		ret = exit_ret;
	}

	exit_ret = npm10xx_otp_poll_clear(&config->i2c, NPM10XX_OTP_PROGMODE,
					  NPM10XX_OTP_PROGMODE_STATUS_Msk);
	if (ret == 0) {
		ret = exit_ret;
	}
}

unlock:
	(void)k_mutex_unlock(&dev_data->lock);

	return ret;
}
#endif /* CONFIG_OTP_PROGRAM */

static int otp_npm10xx_init(const struct device *dev)
{
	const struct otp_npm10xx_config *config = dev->config;
	struct otp_npm10xx_data *dev_data = dev->data;

	if (!i2c_is_ready_dt(&config->i2c)) {
		LOG_ERR("I2C bus is not ready");
		return -ENODEV;
	}

	(void)k_mutex_init(&dev_data->lock);

	return 0;
}

static DEVICE_API(otp, otp_npm10xx_api) = {
#if defined(CONFIG_OTP_PROGRAM)
	.program = otp_npm10xx_program,
#endif
	.read = otp_npm10xx_read,
};

#define OTP_NPM10XX_DEFINE(n)                                                                      \
	static const struct otp_npm10xx_config otp_npm10xx_config_##n = {                          \
		.i2c = I2C_DT_SPEC_GET(DT_INST_PARENT(n)),                                         \
	};                                                                                         \
                                                                                                   \
	static struct otp_npm10xx_data otp_npm10xx_data_##n;                                       \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, otp_npm10xx_init, NULL, &otp_npm10xx_data_##n,                    \
			      &otp_npm10xx_config_##n, POST_KERNEL,                                \
			      CONFIG_OTP_NPM10XX_INIT_PRIORITY, &otp_npm10xx_api);

DT_INST_FOREACH_STATUS_OKAY(OTP_NPM10XX_DEFINE)
