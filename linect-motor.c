/** 
 * @file linect-motor.c
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
#include <linux/mm.h>

#include <linux/usb.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>

#include <linux/proc_fs.h>

#include "linect.h"

#define MAX_TILT_ANGLE 31
#define MIN_TILT_ANGLE (-31)

#define GRAVITY 9.80665

#define FREENECT_COUNTS_PER_G 819

int linect_motor_set_led(struct usb_linect *dev, int led)
{
	int ret;
	uint8_t empty[0x1];
	
	if (!dev->motor_udev) return -EACCES;
	
	if (led < 0 || led > 6)  return -EINVAL;
	mutex_lock(&dev->mutex_motor);
	ret = usb_control_msg(dev->motor_udev, usb_sndctrlpipe(dev->motor_udev, 0), 0x06, 0x40, (uint16_t)led, 0x0, empty, 0x0, 0);
	dev->last_led_status = led;
	mutex_unlock(&dev->mutex_motor);
	return ret;
}

int linect_motor_set_tilt_degs(struct usb_linect *dev, int angle)
{
	int ret;
	uint8_t empty[0x1];
	
	if (!dev->motor_udev) return -EACCES;

	angle = (angle<MIN_TILT_ANGLE) ? MIN_TILT_ANGLE : ((angle>MAX_TILT_ANGLE) ? MAX_TILT_ANGLE : angle);
	angle = angle * 2;
	
	mutex_lock(&dev->mutex_motor);
	ret = usb_control_msg(dev->motor_udev, usb_sndctrlpipe(dev->motor_udev, 0), 0x31, 0x40, (uint16_t)angle, 0x0, empty, 0x0, 0);
	dev->last_motor_status = angle;
	mutex_unlock(&dev->mutex_motor);
	return ret;
}

int linect_get_raw_accel(struct usb_linect *dev, int16_t* x, int16_t* y, int16_t* z)
{
	uint8_t buf[10];
	uint16_t ux, uy, uz;
	int ret;
	
	if (!dev->motor_udev) return -EACCES;
	
	mutex_lock(&dev->mutex_motor);
	ret = usb_control_msg(dev->motor_udev, usb_rcvctrlpipe(dev->motor_udev, 0), 0x32, 0xC0, 0x0, 0x0, buf, 10, 0);
	mutex_unlock(&dev->mutex_motor);
	if (ret != 10) {
		LNT_DEBUG("Error in accelerometer reading, control msg returned %d\n", ret);
		return ret < 0 ? ret : -1;
	}

	ux = ((uint16_t)buf[2] << 8) | buf[3];
	uy = ((uint16_t)buf[4] << 8) | buf[5];
	uz = ((uint16_t)buf[6] << 8) | buf[7];
	*x = (int16_t)ux;
	*y = (int16_t)uy;
	*z = (int16_t)uz;

	return ret;
}

