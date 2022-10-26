// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for the AP1302 external camera ISP from ON Semiconductor
 *
 * Copyright (C) 2021, Witekio, Inc.
 * Copyright (C) 2021, Xilinx, Inc.
 * Copyright (C) 2021, Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 * Copyright (C) 2021, IMD Technologies Ltd
 *
 */

#include "ap1302.h"

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/regmap.h>
#include <linux/ktime.h>

static int ap1302_set_mipi_t3_clk(struct ap1302_device *ap1302)
{
	unsigned int mipi_t3, t_clk_post, t_clk_pre;
	int ret;

	/* Set the Tclk_post and Tclk_pre values */
	ret = ap1302_read(ap1302, AP1302_ADV_HINF_MIPI_T3, &mipi_t3);
	if (ret)
		return ret;

	/* Read Tclk post default setting and increment by 2 */
	t_clk_post = ((mipi_t3 & AP1302_TCLK_POST_MASK)
					>> AP1302_TCLK_POST_SHIFT) + 0x5;
	/* Read Tclk pre default setting and increment by 1 */
	t_clk_pre = ((mipi_t3 & AP1302_TCLK_PRE_MASK)
					>> AP1302_TCLK_PRE_SHIFT) + 0x1;

	mipi_t3 = ((mipi_t3 & ~(AP1302_TCLK_POST_MASK))
					& ~(AP1302_TCLK_PRE_MASK));
	mipi_t3 = (mipi_t3 | (t_clk_pre << AP1302_TCLK_PRE_SHIFT)
					| t_clk_post);

	/* Write MIPI_T3 register with updated Tclk_post and Tclk_pre values */
	return ap1302_write(ap1302, AP1302_ADV_HINF_MIPI_T3, mipi_t3, NULL);
}

int ap1302_request_firmware(struct ap1302_device *ap1302)
{
	static const char * const suffixes[] = {
		"",
		"_single",
		"_dual",
	};

	const struct ap1302_firmware_header *fw_hdr;
	unsigned int num_sensors;
	unsigned int fw_size;
	unsigned int i;
	char name[64];
	int ret;

	for (i = 0, num_sensors = 0; i < ARRAY_SIZE(ap1302->sensors); ++i) {
		if (ap1302->sensors[i].present)
			num_sensors++;
	}

	ret = snprintf(name, sizeof(name), "ap1302_%s%s_fw.bin",
		       ap1302->sensor_info->name, suffixes[num_sensors]);
	if (ret >= sizeof(name)) {
		dev_err(ap1302->dev, "Firmware name too long\n");
		return -EINVAL;
	}

	dev_dbg(ap1302->dev, "Requesting firmware %s\n", name);

	ret = request_firmware(&ap1302->fw, name, ap1302->dev);
	if (ret) {
		dev_err(ap1302->dev, "Failed to request firmware: %d\n", ret);
		return ret;
	}

	/*
	 * The firmware binary contains a header defined by the
	 * ap1302_firmware_header structure. The firmware itself (also referred
	 * to as bootdata) follows the header. Perform sanity checks to ensure
	 * the firmware is valid.
	 */
	fw_hdr = (const struct ap1302_firmware_header *)ap1302->fw->data;
	fw_size = ap1302->fw->size - sizeof(*fw_hdr);

	if (fw_hdr->pll_init_size > fw_size) {
		dev_err(ap1302->dev,
			"Invalid firmware: PLL init size too large\n");
		return -EINVAL;
	}

	return 0;
}

/*
 * ap1302_write_fw_window() - Write a piece of firmware to the AP1302
 * @win_pos: Firmware load window current position
 * @buf: Firmware data buffer
 * @len: Firmware data length
 *
 * The firmware is loaded through a window in the registers space. Writes are
 * sequential starting at address 0x8000, and must wrap around when reaching
 * 0x9fff. This function write the firmware data stored in @buf to the AP1302,
 * keeping track of the window position in the @win_pos argument.
 */
static int ap1302_write_fw_window(struct ap1302_device *ap1302, const u8 *buf,
				  u32 len, unsigned int *win_pos)
{
	while (len > 0) {
		unsigned int write_addr;
		unsigned int write_size;
		int ret;

		/*
		 * Write at most len bytes, from the current position to the
		 * end of the window.
		 */
		write_addr = *win_pos + AP1302_FW_WINDOW_OFFSET;
		write_size = min(len, AP1302_FW_WINDOW_SIZE - *win_pos);

		dev_dbg(ap1302->dev, "%u bytes remaining", len);

		if (ap1302->spi_dev) {
			ret = ap1302_spi_write_block(ap1302, write_addr, buf,
							write_size);
		} else {
			ret = regmap_raw_write(ap1302->regmap16, write_addr,
						buf, write_size);
		}

		if (ret) {
			dev_err(ap1302->dev, "ret = %d", ret);
			return ret;
		}

		buf += write_size;
		len -= write_size;

		*win_pos += write_size;
		if (*win_pos >= AP1302_FW_WINDOW_SIZE)
			*win_pos = 0;

		msleep(10);
	}

	return 0;
}

int ap1302_load_firmware(struct ap1302_device *ap1302)
{
	const struct ap1302_firmware_header *fw_hdr;
	unsigned int fw_size;
	const u8 *fw_data;
	unsigned int win_pos = 0;
	unsigned int crc = 0;
	int ret;
	u64 start_time, stop_time, elapsed_time;

	dev_info(ap1302->dev, "Loading firmware");

	start_time = ktime_get_ns();

	fw_hdr = (const struct ap1302_firmware_header *)ap1302->fw->data;
	fw_data = (u8 *)&fw_hdr[1];
	fw_size = ap1302->fw->size - sizeof(*fw_hdr);

	/* Clear the CRC register. */
	ret = ap1302_write(ap1302, AP1302_SIP_CRC, 0xffff, NULL);
	if (ret)
		return ret;

	/*
	 * Load the PLL initialization settings, set the bootdata stage to 2 to
	 * apply the basic_init_hp settings, and wait 1ms for the PLL to lock.
	 */
	ret = ap1302_write_fw_window(ap1302, fw_data, fw_hdr->pll_init_size,
				     &win_pos);
	if (ret)
		return ret;

	ret = ap1302_write(ap1302, AP1302_BOOTDATA_STAGE, 0x0002, NULL);
	if (ret)
		return ret;

	usleep_range(1200, 2000);

	/* Load the rest of the bootdata content and verify the CRC. */
	ret = ap1302_write_fw_window(ap1302, fw_data + fw_hdr->pll_init_size,
				     fw_size - fw_hdr->pll_init_size, &win_pos);
	if (ret)
		return ret;

	msleep(1);

	stop_time = ktime_get_ns();

	elapsed_time = (stop_time - start_time) / 1000000;

	dev_info(ap1302->dev, "Finished loading firmware; took %llu ms",
			elapsed_time);

	/*
	 * Write 0xffff to the bootdata_stage register to indicate to the
	 * AP1302 that the whole bootdata content has been loaded.
	 */
	ret = ap1302_write(ap1302, AP1302_BOOTDATA_STAGE, 0xffff, NULL);
	if (ret)
		return ret;

	/* Wait for the BOOTDATA_CHECKSUM register to be non-zero */

	while (0 == crc) {
		ret = ap1302_read(ap1302, AP1302_BOOTDATA_CHECKSUM, &crc);
		if (ret)
			return ret;

		msleep(10);
	}

	/* Check the CRC against the expected value */

	if (crc != fw_hdr->crc) {
		dev_warn(ap1302->dev,
			 "CRC mismatch: expected 0x%04x, got 0x%04x\n",
			 fw_hdr->crc, crc);
		return -EAGAIN;
	}

	dev_info(ap1302->dev, "CRC matches expected value (0x%04x)", crc);

	/* The AP1302 starts outputting frames right after boot, stop it. */
	ret = ap1302_stall(ap1302, true);
	if (ret)
		return ret;

	/* Adjust MIPI TCLK timings */
	return ap1302_set_mipi_t3_clk(ap1302);
}
