/** 
 * @file linect-dev-k1414.c
 * @author Arturo Casal
 * @date 2010
 * @version v0.1
 *
 * @brief Driver for MS Kinect
 *
 * @note Copyright (C) Arturo Casal
 *
 * @par Licences
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/kref.h>

#include <linux/usb.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>

#include "linect.h"


//=============================================================================
//
// Kinect API
//
//=============================================================================

struct cam_hdr {
	uint8_t magic[2];
	uint16_t len;
	uint16_t cmd;
	uint16_t tag;
};

struct caminit {
	uint16_t command;
	uint16_t tag;
	int cmdlen;
	int replylen;
	uint8_t cmddata[1024];
	uint8_t replydata[1024];
};

static struct caminit inits[] = {
	{
		0x03, 0x1267, 4, 2,
		{0x06, 0x00, 0x00, 0x00},
		{0x00, 0x00},
	},
	{
		0x03, 0x1268, 4, 2,
		{0x12, 0x00, 0x03, 0x00},
		{0x00, 0x00},
	},
	{
		0x03, 0x1269, 4, 2,
		{0x13, 0x00, 0x01, 0x00},
		{0x00, 0x00},
	},
	{
		0x03, 0x126a, 4, 2,
		{0x14, 0x00, 0x1e, 0x00},
		{0x00, 0x00},
	},
	{
		0x03, 0x126b, 4, 2,
		{0x06, 0x00, 0x02, 0x00},
		{0x00, 0x00},
	},
	{
		0x03, 0x126e, 4, 2,
		{0x06, 0x00, 0x00, 0x00},
		{0x00, 0x00},
	},
	{
		0x03, 0x126f, 4, 2,
		{0x12, 0x00, 0x03, 0x00},
		{0x00, 0x00},
	},
	{
		0x03, 0x1270, 4, 2,
		{0x13, 0x00, 0x01, 0x00},
		{0x00, 0x00},
	},
	{
		0x03, 0x1271, 4, 2,
		{0x14, 0x00, 0x1e, 0x00},
		{0x00, 0x00},
	},
	{
		0x03, 0x1272, 4, 2,
		{0x16, 0x00, 0x01, 0x00},
		{0x00, 0x00},
	},
	{
		0x03, 0x1273, 4, 2,
		{0x18, 0x00, 0x00, 0x00},
		{0x00, 0x00},
	},
	{
		0x03, 0x1274, 4, 2,
		{0x02, 0x00, 0x00, 0x00},
		{0x00, 0x00},
	},
	{
		0x03, 0x1275, 4, 2,
		{0x05, 0x01, 0x00, 0x00},
		{0x00, 0x00},
	},
	{
		0x03, 0x1276, 4, 2,
		{0x24, 0x00, 0x01, 0x00},
		{0x00, 0x00},
	},
	{
		0x03, 0x1277, 4, 2,
		{0x2d, 0x00, 0x01, 0x00},
		{0x00, 0x00},
	},
	{
		0x03, 0x1278, 4, 2,
		{0x06, 0x00, 0x02, 0x00},
		{0x00, 0x00},
	},
	{
		0x03, 0x1279, 4, 2,
		{0x05, 0x00, 0x00, 0x00},
		{0x00, 0x00},
	},
	{
		0x03, 0x127a, 4, 2,
		{0x0c, 0x00, 0x01, 0x00},
		{0x00, 0x00},
	},
	{
		0x03, 0x127b, 4, 2,
		{0x0d, 0x00, 0x01, 0x00},
		{0x00, 0x00},
	},
	{
		0x03, 0x127c, 4, 2,
		{0x0e, 0x00, 0x1e, 0x00},
		{0x00, 0x00},
	},
	{
		0x03, 0x127d, 4, 2,
		{0x05, 0x00, 0x01, 0x00},
		{0x00, 0x00},
	},
	{
		0x03, 0x127e, 4, 2,
		{0x47, 0x00, 0x00, 0x00},
		{0x00, 0x00},
	},
	{
		0x03, 0x127f, 4, 2,
		{0x0c, 0x00, 0x00, 0x00},
		{0x00, 0x00},
	},
	{
		0x03, 0x1280, 4, 2,
		{0x05, 0x00, 0x00, 0x00},
		{0x00, 0x00},
	},
	{
		0x03, 0x1281, 4, 2,
		{0x0d, 0x00, 0x01, 0x00},
		{0x00, 0x00},
	},
	{
		0x03, 0x1282, 4, 2,
		{0x0e, 0x00, 0x1e, 0x00},
		{0x00, 0x00},
	},
	{
		0x03, 0x1283, 4, 2,
		{0x05, 0x00, 0x01, 0x00},
		{0x00, 0x00},
	},
	{
		0x03, 0x1284, 4, 2,
		{0x47, 0x00, 0x00, 0x00},
		{0x00, 0x00},
	},
};

#define K_NUM_INITS 28

/** 
 * @param dev Device structure
 * 
 * @returns 0 if all is OK
 *
 * @brief This function permits to initialize the device.
 *
 * This function must be called at first. It's the start of the
 * initialization process. After this process, the device is
 * completly initalized and it's ready.
 *
 * This function is written from the USB log.
 */
int linect_cam_init(struct usb_linect *dev)
{
	int i, j, ret;
	//uint8_t obuf[0x2000];
	//uint8_t ibuf[0x2000];
	uint8_t *obuf;
	uint8_t *ibuf;
	struct cam_hdr *chdr;
	struct cam_hdr *rhdr;
	const struct caminit *ip;
	
	if (!dev->cam->startupinit) return 0;

	LNT_INFO("Initialize Kinect\n");
	
	if (!dev->freeled)
		linect_motor_set_led(dev, LED_BLINK_RED_YELLOW);
	
	//obuf = linect_rvmalloc(0x400);
	//ibuf = linect_rvmalloc(0x200);

obuf = kzalloc(0x400, GFP_KERNEL);
ibuf = kzalloc(0x200, GFP_KERNEL);
	
	chdr = (void*)obuf;
	rhdr = (void*)ibuf;
	
	ret = usb_control_msg(dev->cam->udev, usb_rcvctrlpipe(dev->cam->udev, 0), 0x06, 0x80, 0x3ee, 0, ibuf, 0x12, 0);
	
	LNT_DEBUG("First xfer: %d\n", ret);

	chdr->magic[0] = 0x47;
	chdr->magic[1] = 0x4d;
	
	for (i=0; i<K_NUM_INITS; i++) {
		ip = &inits[i];
		chdr->cmd = ip->command;
		chdr->tag = ip->tag;
		chdr->len = ip->cmdlen / 2;
		memcpy(obuf+sizeof(*chdr), ip->cmddata, ip->cmdlen);
		ret = usb_control_msg(dev->cam->udev, usb_sndctrlpipe(dev->cam->udev, 0), 0, 0x40, 0, 0, obuf, ip->cmdlen + sizeof(*chdr), 0);
		LNT_DEBUG("CTL CMD %04x %04x = %d\n", chdr->cmd, chdr->tag, ret);
		do {
			ret = usb_control_msg(dev->cam->udev, usb_rcvctrlpipe(dev->cam->udev, 0), 0, 0xc0, 0, 0, ibuf, 0x200, 0);
		} while (ret == 0);
		LNT_DEBUG("CTL RES = %d\n", ret);
		if (rhdr->magic[0] != 0x52 || rhdr->magic[1] != 0x42) {
			LNT_DEBUG("Bad magic %02x %02x\n", rhdr->magic[0], rhdr->magic[1]);
			continue;
		}
		if (rhdr->cmd != chdr->cmd) {
			LNT_DEBUG("Bad cmd %02x != %02x\n", rhdr->cmd, chdr->cmd);
			continue;
		}
		if (rhdr->tag != chdr->tag) {
			LNT_DEBUG("Bad tag %04x != %04x\n", rhdr->tag, chdr->tag);
			continue;
		}
		if (rhdr->len != (ret-sizeof(*rhdr))/2) {
			LNT_DEBUG("Bad len %04x != %04x\n", rhdr->len, (int)(ret-sizeof(*rhdr))/2);
			continue;
		}
		if (rhdr->len != (ip->replylen/2) || memcmp(ibuf+sizeof(*rhdr), ip->replydata, ip->replylen)) {
			LNT_DEBUG("Expected: ");
			for (j=0; j<ip->replylen; j++) {
				LNT_DEBUG("%02x ", ip->replydata[j]);
			}
			LNT_DEBUG("\nGot:      ");
			for (j=0; j<(rhdr->len*2); j++) {
				LNT_DEBUG("%02x ", ibuf[j+sizeof(*rhdr)]);
			}
			LNT_DEBUG("\n");
		}
	}
	
	// Device is initialized and is ready !!!
	LNT_INFO("Kinect is ready\n");
	
	//kfree(obuf);
	//kfree(ibuf);

	kfree(obuf);
	kfree(ibuf);
	
	if (!dev->freeled)
		linect_motor_set_led(dev, LED_GREEN);

	return 0;
}

int linect_cam_send_cmd(struct usb_linect *dev, uint16_t cmd, void *cmdbuf, unsigned int cmd_len, void *replybuf, unsigned int reply_len)
{
	int res, actual_len;
	uint8_t *obuf;
	uint8_t *ibuf;
	struct cam_hdr *chdr;
	struct cam_hdr *rhdr;

	if (cmd_len & 1 || cmd_len > (0x400 - sizeof(*chdr))) {
		LNT_ERROR("send_cmd: Invalid command length (0x%x)\n", cmd_len);
		return -1;
	}
	
	obuf = kzalloc(0x400, GFP_KERNEL);
	ibuf = kzalloc(0x200, GFP_KERNEL);
	
	chdr = (void*)obuf;
	rhdr = (void*)ibuf;

	chdr->magic[0] = 0x47;
	chdr->magic[1] = 0x4d;
	chdr->cmd = cmd;
	chdr->tag = dev->cam->cam_tag;
	chdr->len = cmd_len / 2;

	if (dev->cam->cam_tag==0) usb_control_msg(dev->cam->udev, usb_rcvctrlpipe(dev->cam->udev, 0), 0x06, 0x80, 0x3ee, 0, ibuf, 0x12, 0);

	memcpy(obuf+sizeof(*chdr), cmdbuf, cmd_len);
	
	res = usb_control_msg(dev->cam->udev, usb_sndctrlpipe(dev->cam->udev, 0), 0, 0x40, 0, 0, obuf, cmd_len + sizeof(*chdr), 0);
			
	
	LNT_DEBUG("Control cmd=%04x tag=%04x len=%04x: %d\n", cmd, dev->cam->cam_tag, cmd_len, res);
	if (res < 0) {
		LNT_ERROR("send_cmd: Output control transfer failed (%d)\n", res);
		kfree(obuf);
		kfree(ibuf);
		return res;
	}

	do {
		actual_len = usb_control_msg(dev->cam->udev, usb_rcvctrlpipe(dev->cam->udev, 0), 0, 0xc0, 0, 0, ibuf, 0x200, 0);
	} while (actual_len == 0);

	LNT_DEBUG("Control reply: %d\n", res);
	if (actual_len < sizeof(*rhdr)) {
		LNT_ERROR("send_cmd: Input control transfer failed (%d)\n", res);
		kfree(obuf);
		kfree(ibuf);
		return res;
	}
	actual_len -= sizeof(*rhdr);

	if (rhdr->magic[0] != 0x52 || rhdr->magic[1] != 0x42) {
		LNT_ERROR("send_cmd: Bad magic %02x %02x\n", rhdr->magic[0], rhdr->magic[1]);
		kfree(obuf);
		kfree(ibuf);
		return -1;
	}
	if (rhdr->cmd != chdr->cmd) {
		LNT_ERROR("send_cmd: Bad cmd %02x != %02x\n", rhdr->cmd, chdr->cmd);
		kfree(obuf);
		kfree(ibuf);
		return -1;
	}
	if (rhdr->tag != chdr->tag) {
		LNT_ERROR("send_cmd: Bad tag %04x != %04x\n", rhdr->tag, chdr->tag);
		kfree(obuf);
		kfree(ibuf);
		return -1;
	}
	if (rhdr->len != (actual_len/2)) {
		LNT_ERROR("send_cmd: Bad len %04x != %04x\n", rhdr->len, (int)(actual_len/2));
		kfree(obuf);
		kfree(ibuf);
		return -1;
	}

	if (actual_len > reply_len) {
		LNT_WARNING("send_cmd: Data buffer is %d bytes long, but got %d bytes\n", reply_len, actual_len);
		memcpy(replybuf, ibuf+sizeof(*rhdr), reply_len);
	} else {
		memcpy(replybuf, ibuf+sizeof(*rhdr), actual_len);
	}

	dev->cam->cam_tag++;

	kfree(obuf);
	kfree(ibuf);

	return actual_len;
}

int linect_cam_write_register(struct usb_linect *dev, uint16_t reg, uint16_t data)
{
	uint16_t reply[2];
	uint16_t cmd[2];
	int res;

	cmd[0] = reg;
	cmd[1] = data;

	LNT_DEBUG("Write Reg 0x%04x <= 0x%02x\n", reg, data);
	res = linect_cam_send_cmd(dev, 0x03, cmd, 4, reply, 4);
	if (res < 0)
		return res;
	if (res != 2) {
		LNT_WARNING("send_cmd returned %d [%04x %04x], 0000 expected\n", res, reply[0], reply[1]);
	}
	return 0;
}

int linect_cam_start_rgb(struct usb_linect *dev)
{
	if (dev->cam->startupinit) return 0;
	mutex_lock(&dev->cam->mutex_cam);
	linect_cam_write_register(dev, 0x05, 0x00); // reset rgb stream
	linect_cam_write_register(dev, 0x0c, 0x00);
	linect_cam_write_register(dev, 0x0d, 0x01);
	linect_cam_write_register(dev, 0x0e, 0x1e); // 30Hz bayer
	linect_cam_write_register(dev, 0x05, 0x01); // start rgb stream
	linect_cam_write_register(dev, 0x47, 0x00); // disable Hflip
	mutex_unlock(&dev->cam->mutex_cam);
	return 0;
}

int linect_cam_stop_rgb(struct usb_linect *dev)
{
	if (dev->cam->startupinit) return 0;
	mutex_lock(&dev->cam->mutex_cam);
	linect_cam_write_register(dev, 0x05, 0x00);
	mutex_unlock(&dev->cam->mutex_cam);
	return 0;
}

int linect_cam_start_depth(struct usb_linect *dev)
{
	if (dev->cam->startupinit) return 0;
	mutex_lock(&dev->cam->mutex_cam);
	linect_cam_write_register(dev, 0x06, 0x00); // reset depth stream
	linect_cam_write_register(dev, 0x12, 0x03);
	linect_cam_write_register(dev, 0x13, 0x01);
	linect_cam_write_register(dev, 0x14, 0x1e);
	//linect_cam_write_register(dev, 0x06, 0x02);
	//linect_cam_write_register(dev, 0x06, 0x00);
	//linect_cam_write_register(dev, 0x16, 0x01);
	linect_cam_write_register(dev, 0x06, 0x02);
	mutex_unlock(&dev->cam->mutex_cam);
	return 0;
}

int linect_cam_stop_depth(struct usb_linect *dev)
{
	if (dev->cam->startupinit) return 0;
	mutex_lock(&dev->cam->mutex_cam);
	linect_cam_write_register(dev, 0x06, 0x00); // stop depth stream
	mutex_unlock(&dev->cam->mutex_cam);
	return 0;
}





