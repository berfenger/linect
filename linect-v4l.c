/** 
 * @file linect-v4l.c
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
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/videodev.h>


#include <linux/usb.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>

#include "linect.h"
#include "linect_v4l_ctrl.h"


static struct v4l2_file_operations v4l_linect_rgb_fops;
static struct v4l2_file_operations v4l_linect_depth_fops;


/**
 * @var linect_image_sizes
 *   List of all resolutions supported by the driver
 */
/*const struct linect_coord linect_image_sizes[LNT_NBR_SIZES] = {
	{   80,   60 },
	{  128,   96 },
	{  160,  120 },
	{  213,  160 },
	{  320,  240 },
	{  640,  480 },
	{  800,  600 },
	{ 1024,  768 },
	{ 1280, 1024 }
};*/


/**
 * @var linect_controls
 *   List of all V4Lv2 controls supported by the driver
 */
static struct v4l2_queryctrl linect_controls[] = {
	{
		.id      = V4L2_CID_BRIGHTNESS,
		.type    = V4L2_CTRL_TYPE_INTEGER,
		.name    = "Brightness",
		.minimum = 0,
		.maximum = 0xff00,
		.step    = 1,
		.default_value = 0x7f00
	}, 
	{
		.id      = V4L2_CCID_MOTOR,
		.type    = V4L2_CTRL_TYPE_INTEGER,
		.name    = "Motor Position",
		.minimum = -31,
		.maximum = 31,
		.step    = 1,
		.default_value = 0
	},
	{
		.id      = V4L2_CCID_LED,
		.type    = V4L2_CTRL_TYPE_INTEGER,
		.name    = "Led color",
		.minimum = 0,
		.maximum = 6,
		.step    = 1,
		.default_value = 1,
		.flags	 = V4L2_CTRL_FLAG_SLIDER
	}
};

static struct v4l2_queryctrl linect_depth_controls[] = {
	{
		.id      = V4L2_CCID_MOTOR,
		.type    = V4L2_CTRL_TYPE_INTEGER,
		.name    = "Motor Position",
		.minimum = -31,
		.maximum = 31,
		.step    = 1,
		.default_value = 0
	},
	{
		.id      = V4L2_CCID_LED,
		.type    = V4L2_CTRL_TYPE_INTEGER,
		.name    = "Led color",
		.minimum = 0,
		.maximum = 6,
		.step    = 1,
		.default_value = 1,
		.flags	 = V4L2_CTRL_FLAG_SLIDER
	}
};


/** 
 * @param dev
 * @param width Width of wished resolution
 * @param height Height of wished resolution
 * 
 * @returns 0 if all is OK
 *
 * @brief Select a video mode
 *
 * This function permits to check and select a video mode.
 */
int v4l_linect_select_video_mode(struct usb_linect *dev, int width, int height)
{
	dev->cam->view.x = 640;
	dev->cam->view.y = 480;

	dev->cam->image.x = 640;
	dev->cam->image.y = 480;
	dev->cam->frame_size = dev->cam->image.x * dev->cam->image.y;

	switch (dev->cam->vsettings.palette) {
		case LNT_PALETTE_RGB24:
		case LNT_PALETTE_BGR24:
			dev->cam->view_size = 3 * dev->cam->view.x * dev->cam->view.y;
			dev->cam->image_size = 3 * dev->cam->frame_size;
			break;

		case LNT_PALETTE_RGB32:
		case LNT_PALETTE_BGR32:
			dev->cam->view_size = 3 * dev->cam->view.x * dev->cam->view.y;
			dev->cam->image_size = 4 * dev->cam->frame_size;
			break;

		case LNT_PALETTE_UYVY:
		case LNT_PALETTE_YUYV:
			dev->cam->view_size = 2 * dev->cam->view.x * dev->cam->view.y;
			dev->cam->image_size = 2 * dev->cam->frame_size;
			break;
	}
	
	dev->cam->view_depth.x = 640;
	dev->cam->view_depth.y = 480;

	dev->cam->image_depth.x = 640;
	dev->cam->image_depth.y = 480;
	dev->cam->frame_size_depth = dev->cam->image_depth.x * dev->cam->image_depth.y;

	switch (dev->cam->depth_vsettings.palette) {
		case LNT_PALETTE_RGB24:
		case LNT_PALETTE_BGR24:
			dev->cam->view_size_depth = 3 * dev->cam->view_depth.x * dev->cam->view_depth.y;
			dev->cam->image_size_depth = 3 * dev->cam->frame_size_depth;
			break;

		case LNT_PALETTE_RGB32:
		case LNT_PALETTE_BGR32:
			dev->cam->view_size_depth = 3 * dev->cam->view_depth.x * dev->cam->view_depth.y;
			dev->cam->image_size_depth = 4 * dev->cam->frame_size_depth;
			break;

		case LNT_PALETTE_UYVY:
		case LNT_PALETTE_YUYV:
			dev->cam->view_size_depth = 2 * dev->cam->view_depth.x * dev->cam->view_depth.y;
			dev->cam->image_size_depth = 2 * dev->cam->frame_size_depth;
			break;
	}

	return 0;
}


/** 
 * @param fp File pointer
 * 
 * @returns 0 if all is OK
 *
 * @brief Open the video device
 *
 * This function permits to open a video device (/dev/videoX)
 */
static int v4l_linect_rgb_open(struct file *fp)
{
	int err;

	struct usb_linect *dev;
	struct video_device *vdev;
	
	vdev = video_devdata(fp);
	dev = video_get_drvdata(video_devdata(fp));

	if (dev == NULL) {
		LNT_ERROR("Device not initialized !!!\n");
		BUG();
	}

	if (dev->cam->vopen_rgb) {
		LNT_DEBUG("RGB Cam is busy, someone is using the device\n");
		return -EBUSY;
	}

	LNT_DEBUG("v4l: RGB camera open");

	mutex_lock(&dev->cam->modlock_rgb);

	// Allocate memory
	err = linect_allocate_rgb_buffers(dev);

	if (err < 0) {
		LNT_ERROR("Failed to allocate buffer memory !\n");
		mutex_unlock(&dev->cam->modlock_rgb);
		return err;
	}
	
	// Reset buffers and parameters
	linect_reset_rgb_buffers(dev);

	// Settings
	dev->cam->error_status = 0;
	dev->cam->visoc_errors = 0;
	dev->cam->vframes_error = 0;
	dev->cam->vframes_dumped = 0;
	dev->cam->vsettings.depth = 24;
	dev->cam->vsettings.palette = LNT_PALETTE_RGB24;

	// Select the resolution by default
	v4l_linect_select_video_mode(dev, 640, 480);

	// Init Isoc and URB
	/*err = usb_linect_rgb_isoc_init(dev);

	if (err) {
		LNT_ERROR("Failed to init ISOC stuff !\n");
		usb_linect_rgb_isoc_cleanup(dev);
		linect_free_rgb_buffers(dev);
		mutex_unlock(&dev->cam->modlock_rgb);
		return err;
	}*/

	dev->cam->vopen_rgb++;
	fp->private_data = vdev;

	mutex_unlock(&dev->cam->modlock_rgb);
	
	if (!dev->freeled)
		linect_motor_set_led(dev, LED_RED);

	return 0;
}

static int v4l_linect_depth_open(struct file *fp)
{
	int err;

	struct usb_linect *dev;
	struct video_device *vdev;
	
	vdev = video_devdata(fp);
	dev = video_get_drvdata(video_devdata(fp));

	if (dev == NULL) {
		LNT_ERROR("Device not initialized !!!\n");
		BUG();
	}

	if (dev->cam->vopen_depth) {
		LNT_DEBUG("Depth Cam is busy, someone is using the device\n");
		return -EBUSY;
	}

	LNT_DEBUG("v4l: Depth camera open");

	mutex_lock(&dev->cam->modlock_depth);

	// Allocate memory
	err = linect_allocate_depth_buffers(dev);

	if (err < 0) {
		LNT_ERROR("Failed to allocate buffer memory !\n");
		mutex_unlock(&dev->cam->modlock_depth);
		return err;
	}
	
	// Reset buffers and parameters
	linect_reset_depth_buffers(dev);
	
	dev->cam->depth_vsettings.depth = 24;
	dev->cam->depth_vsettings.palette = LNT_PALETTE_RGB24;

	// Init Isoc and URB
	/*err = usb_linect_depth_isoc_init(dev);

	if (err) {
		LNT_ERROR("Failed to init ISOC stuff !\n");
		usb_linect_depth_isoc_cleanup(dev);
		linect_free_depth_buffers(dev);
		mutex_unlock(&dev->cam->modlock_depth);
		return err;
	}*/

	dev->cam->vopen_depth++;
	fp->private_data = vdev;

	mutex_unlock(&dev->cam->modlock_depth);
	
	if (!dev->freeled)
		linect_motor_set_led(dev, LED_RED);

	return 0;
}


/** 
 * @param fp File pointer
 * 
 * @returns 0 if all is OK
 *
 * @brief Release an opened file.
 *
 * This function permits to release an opened file with the 'open' method.
 */
static int v4l_linect_rgb_release(struct file *fp)
{
	struct usb_linect *dev;
	struct video_device *vdev;
	
	vdev = video_devdata(fp);
	dev = video_get_drvdata(video_devdata(fp));

	if (dev->cam->vopen_rgb == 0)
		LNT_ERROR("v4l_release called on closed device\n");

	LNT_DEBUG("v4l: RGB camera close");

	// ISOC and URB cleanup
	usb_linect_rgb_isoc_cleanup(dev);

	// Free memory
	linect_free_rgb_buffers(dev);

	dev->cam->vopen_rgb--;
	
	if (!dev->freeled && dev->cam->vopen_rgb == 0 && dev->cam->vopen_depth == 0) linect_motor_set_led(dev, LED_GREEN);

	return 0;
}

static int v4l_linect_depth_release(struct file *fp)
{
	struct usb_linect *dev;
	struct video_device *vdev;
	
	vdev = video_devdata(fp);
	dev = video_get_drvdata(video_devdata(fp));

	if (dev->cam->vopen_depth == 0)
		LNT_ERROR("v4l_release called on closed device\n");

	LNT_DEBUG("v4l: Depth camera close");

	// ISOC and URB cleanup
	usb_linect_depth_isoc_cleanup(dev);

	// Free memory
	linect_free_depth_buffers(dev);

	// Unregister interface on power management
//	usb_autopm_put_interface(dev->cam->interface);

	dev->cam->vopen_depth--;
	
	if (!dev->freeled && dev->cam->vopen_rgb == 0 && dev->cam->vopen_depth == 0) linect_motor_set_led(dev, LED_GREEN);

	return 0;
}


/** 
 * @param fp File pointer
 *
 * @retval buf Buffer in user space
 * @retval count 
 * @retval f_pos 
 * 
 * @returns Count value
 *
 * @brief Read the video device
 *
 * This function is called by the application is reading the video device.
 */
static ssize_t v4l_linect_rgb_read(struct file *fp, char __user *buf,
		size_t count, loff_t *f_pos)
{
	int noblock = fp->f_flags & O_NONBLOCK;

	struct usb_linect *dev;
	struct video_device *vdev;

	int bytes_to_read;
	void *image_buffer_addr;
	
	DECLARE_WAITQUEUE(wait, current);

	vdev = video_devdata(fp);
	dev = video_get_drvdata(video_devdata(fp));
	
	usb_linect_rgb_isoc_init(dev);

	LNT_DEBUG("Read vdev=0x%p, buf=0x%p, count=%zd\n", vdev, buf, count);

	if (dev == NULL)
		return -EFAULT;

	if (vdev == NULL)
		return -EFAULT;

	mutex_lock(&dev->cam->modlock_rgb);

	if (dev->cam->image_read_pos == 0) {
		add_wait_queue(&dev->cam->wait_rgb_frame, &wait);

		while (dev->cam->full_frames == NULL) {
			if (dev->cam->error_status) {
				remove_wait_queue(&dev->cam->wait_rgb_frame, &wait);
				set_current_state(TASK_RUNNING);
				mutex_unlock(&dev->cam->modlock_rgb);
				return -dev->cam->error_status ;
			}

			if (noblock) {
				remove_wait_queue(&dev->cam->wait_rgb_frame, &wait);
				set_current_state(TASK_RUNNING);
				mutex_unlock(&dev->cam->modlock_rgb);
				return -EWOULDBLOCK;
			}

			if (signal_pending(current)) {
				remove_wait_queue(&dev->cam->wait_rgb_frame, &wait);
				set_current_state(TASK_RUNNING);
				mutex_unlock(&dev->cam->modlock_rgb);
				return -ERESTARTSYS;
			}

			schedule();
			set_current_state(TASK_INTERRUPTIBLE);
		}

		remove_wait_queue(&dev->cam->wait_rgb_frame, &wait);
		set_current_state(TASK_RUNNING);

		if (linect_handle_rgb_frame(dev)) {
			mutex_unlock(&dev->cam->modlock_rgb);
			return -EFAULT;
		}
	}

	bytes_to_read = dev->cam->view_size;

	if (count + dev->cam->image_read_pos > bytes_to_read)
		count = bytes_to_read - dev->cam->image_read_pos;

	image_buffer_addr = dev->cam->image_data;
	image_buffer_addr += dev->cam->images[dev->cam->fill_image].offset;
	image_buffer_addr += dev->cam->image_read_pos;

	if (copy_to_user(buf, image_buffer_addr, count)) {
		mutex_unlock(&dev->cam->modlock_rgb);
		return -EFAULT;
	}
	
	dev->cam->image_read_pos += count;
	
	if (dev->cam->image_read_pos >= bytes_to_read) {
		dev->cam->image_read_pos = 0;
		linect_next_rgb_image(dev);
	}

	mutex_unlock(&dev->cam->modlock_rgb);

	return count;
}

static ssize_t v4l_linect_depth_read(struct file *fp, char __user *buf,
		size_t count, loff_t *f_pos)
{
	int noblock = fp->f_flags & O_NONBLOCK;

	struct usb_linect *dev;
	struct video_device *vdev;

	int bytes_to_read;
	void *image_buffer_addr;
	
	DECLARE_WAITQUEUE(wait, current);

	vdev = video_devdata(fp);
	dev = video_get_drvdata(video_devdata(fp));
	
	usb_linect_depth_isoc_init(dev);

	LNT_DEBUG("Read vdev=0x%p, buf=0x%p, count=%zd\n", vdev, buf, count);

	if (dev == NULL)
		return -EFAULT;

	if (vdev == NULL)
		return -EFAULT;

	mutex_lock(&dev->cam->modlock_depth);

	if (dev->cam->image_read_pos_depth == 0) {
		add_wait_queue(&dev->cam->wait_depth_frame, &wait);

		while (dev->cam->full_frames_depth == NULL) {
			if (dev->cam->error_status) {
				remove_wait_queue(&dev->cam->wait_depth_frame, &wait);
				set_current_state(TASK_RUNNING);
				mutex_unlock(&dev->cam->modlock_depth);
				return -dev->cam->error_status ;
			}

			if (noblock) {
				remove_wait_queue(&dev->cam->wait_depth_frame, &wait);
				set_current_state(TASK_RUNNING);
				mutex_unlock(&dev->cam->modlock_depth);
				return -EWOULDBLOCK;
			}

			if (signal_pending(current)) {
				remove_wait_queue(&dev->cam->wait_depth_frame, &wait);
				set_current_state(TASK_RUNNING);
				mutex_unlock(&dev->cam->modlock_depth);
				return -ERESTARTSYS;
			}

			schedule();
			set_current_state(TASK_INTERRUPTIBLE);
		}

		remove_wait_queue(&dev->cam->wait_depth_frame, &wait);
		set_current_state(TASK_RUNNING);

		if (linect_handle_depth_frame(dev)) {
			mutex_unlock(&dev->cam->modlock_depth);
			return -EFAULT;
		}
	}

	bytes_to_read = 640*480*3;

	if (count + dev->cam->image_read_pos_depth > bytes_to_read)
		count = bytes_to_read - dev->cam->image_read_pos_depth;

	image_buffer_addr = dev->cam->image_data_depth;
	image_buffer_addr += dev->cam->images_depth[dev->cam->fill_image_depth].offset;
	image_buffer_addr += dev->cam->image_read_pos_depth;

	if (copy_to_user(buf, image_buffer_addr, count)) {
		mutex_unlock(&dev->cam->modlock_depth);
		return -EFAULT;
	}
	
	dev->cam->image_read_pos_depth += count;
	
	if (dev->cam->image_read_pos_depth >= bytes_to_read) {
		dev->cam->image_read_pos_depth = 0;
		linect_next_depth_image(dev);
	}

	mutex_unlock(&dev->cam->modlock_depth);

	return count;
}


/** 
 * @param fp File pointer
 * @param wait 
 * 
 * @returns 0 if all is OK
 *
 * @brief Polling function
 */
static unsigned int v4l_linect_rgb_poll(struct file *fp, poll_table *wait)
{
	struct usb_linect *dev;
	struct video_device *vdev;
	unsigned int ret;
	
	vdev = video_devdata(fp);
	dev = video_get_drvdata(video_devdata(fp));

	if (vdev == NULL)
		return -EFAULT;

	if (dev == NULL)
		return -EFAULT;

	poll_wait(fp, &dev->cam->wait_rgb_frame, wait);

	if (dev->cam->error_status)
		ret = POLLERR;

	if (dev->cam->full_frames != NULL)
		return (POLLIN | POLLRDNORM);

	return 0;
}

static unsigned int v4l_linect_depth_poll(struct file *fp, poll_table *wait)
{
	struct usb_linect *dev;
	struct video_device *vdev;
	unsigned int ret;
	
	vdev = video_devdata(fp);
	dev = video_get_drvdata(video_devdata(fp));

	if (vdev == NULL)
		return -EFAULT;

	if (dev == NULL)
		return -EFAULT;

	poll_wait(fp, &dev->cam->wait_depth_frame, wait);

	if (dev->cam->error_status)
		ret = POLLERR;

	if (dev->cam->full_frames_depth != NULL)
		return (POLLIN | POLLRDNORM);

	return 0;
}


/** 
 * @param fp File pointer
 * @param vma VMA structure
 * 
 * @returns 0 if all is OK
 *
 * @brief Memory map
 *
 * This function permits to map a memory space.
 */
static int v4l_linect_rgb_mmap(struct file *fp, struct vm_area_struct *vma)
{
	unsigned int i;

	unsigned long size;
	unsigned long start;
	unsigned long pos;
	unsigned long page;

	struct usb_linect *dev;

	struct video_device *vdev;
	
	vdev = video_devdata(fp);
	dev = video_get_drvdata(video_devdata(fp));

	LNT_DEBUG("mmap\n");

	start = vma->vm_start;
	size = vma->vm_end - vma->vm_start;

	// Find the buffer for this mapping...
	for (i=0; i<dev->cam->nbuffers; i++) {
		pos = dev->cam->images[i].offset;

		if ((pos >> PAGE_SHIFT) == vma->vm_pgoff)
			break;
	}

	// If no buffer found !
	if (i == LNT_MAX_IMAGES) {
		LNT_ERROR("mmap no buffer found !\n");
		return -EINVAL;
	}

	if (i == 0) {
		unsigned long total_size;

		total_size = dev->cam->nbuffers * dev->cam->len_per_image;

		if (size != dev->cam->len_per_image && size != total_size) {
			LNT_ERROR("Wrong size (%lu) needed to be len_per_image=%d or total_size=%lu\n",
				size, dev->cam->len_per_image, total_size);
				
			return -EINVAL;
		}
	}
	else if (size > dev->cam->len_per_image)
		return -EINVAL;

	vma->vm_flags |= VM_IO;

	pos = (unsigned long) dev->cam->image_data;

	while (size > 0) {
		page = vmalloc_to_pfn((void *) pos);

		if (remap_pfn_range(vma, start, page, PAGE_SIZE, PAGE_SHARED))
			return -EAGAIN;

		start += PAGE_SIZE;
		pos += PAGE_SIZE;

		if (size > PAGE_SIZE)
			size -= PAGE_SIZE;
		else
			size = 0;
	}

	return 0;
}

static int v4l_linect_depth_mmap(struct file *fp, struct vm_area_struct *vma)
{
	unsigned int i;

	unsigned long size;
	unsigned long start;
	unsigned long pos;
	unsigned long page;

	struct usb_linect *dev;

	struct video_device *vdev;
	
	vdev = video_devdata(fp);
	dev = video_get_drvdata(video_devdata(fp));

	LNT_DEBUG("mmap\n");

	start = vma->vm_start;
	size = vma->vm_end - vma->vm_start;

	// Find the buffer for this mapping...
	for (i=0; i<dev->cam->nbuffers; i++) {
		pos = dev->cam->images_depth[i].offset;

		if ((pos >> PAGE_SHIFT) == vma->vm_pgoff)
			break;
	}

	// If no buffer found !
	if (i == LNT_MAX_IMAGES) {
		LNT_ERROR("mmap no buffer found !\n");
		return -EINVAL;
	}

	if (i == 0) {
		unsigned long total_size;

		total_size = dev->cam->nbuffers * dev->cam->len_per_image;

		if (size != dev->cam->len_per_image && size != total_size) {
			LNT_ERROR("Wrong size (%lu) needed to be len_per_image=%d or total_size=%lu\n",
				size, dev->cam->len_per_image, total_size);
				
			return -EINVAL;
		}
	}
	else if (size > dev->cam->len_per_image)
		return -EINVAL;

	vma->vm_flags |= VM_IO;

	pos = (unsigned long) dev->cam->image_data_depth;

	while (size > 0) {
		page = vmalloc_to_pfn((void *) pos);

		if (remap_pfn_range(vma, start, page, PAGE_SIZE, PAGE_SHARED))
			return -EAGAIN;

		start += PAGE_SIZE;
		pos += PAGE_SIZE;

		if (size > PAGE_SIZE)
			size -= PAGE_SIZE;
		else
			size = 0;
	}

	return 0;
}


/** 
 * @param fp File pointer
 * @param cmd Command
 * @param arg Arguments of the command
 * 
 * @returns 0 if all is OK
 *
 * @brief Manage IOCTL
 *
 * This function permits to manage all the IOCTL from the application.
 */
static long v4l_linect_rgb_do_ioctl(struct file *fp,
		unsigned int cmd, void __user *arg)
{
	struct usb_linect *dev;
	struct video_device *vdev;

	DECLARE_WAITQUEUE(wait, current);
	
	vdev = video_devdata(fp);
	dev = video_get_drvdata(video_devdata(fp));

#if (CONFIG_LINECT_DEBUG == 1)
	//v4l_printk_ioctl(cmd);
#endif

	switch (cmd) {
		// Video 4 Linux v1

		case VIDIOCGCAP:
			{
				struct video_capability *cap = arg;

				LNT_DEBUG("VIDIOCGCAP\n");

				memset(cap, 0, sizeof(*cap));
				strlcpy(cap->name, "linect", sizeof(cap->name));
				cap->type = VID_TYPE_CAPTURE;
				cap->channels = 1;
				cap->audios = 0;
				cap->minwidth = 640;
				cap->minheight = 480;
				cap->maxwidth = 640;
				cap->maxheight = 480;
			}
			break;
	
		case VIDIOCGCHAN:
			{
			    struct video_channel *v = arg;

				LNT_DEBUG("VIDIOCGCHAN\n");

			    if (v->channel != 0)
				    return -EINVAL;
			
				v->flags = 0;
			    v->tuners = 0;
			    v->type = VIDEO_TYPE_CAMERA;
			    strcpy(v->name, "Webcam");
			}
			break;

		case VIDIOCSCHAN:
			{
				struct video_channel *v = arg;

				LNT_DEBUG("VIDIOCSCHAN\n");

				if (v->channel != 0)
					return -EINVAL;
			}
			break;

		case VIDIOCGPICT:
			{
				struct video_picture *p = arg;

				LNT_DEBUG("VIDIOCGPICT\n");

				p->brightness = dev->cam->vsettings.brightness;
				p->depth = dev->cam->vsettings.depth;
				p->palette = dev->cam->vsettings.palette;

				switch (dev->cam->vsettings.palette) {
					case LNT_PALETTE_BGR24:
						p->palette = VIDEO_PALETTE_RGB24;
						break;

					case LNT_PALETTE_BGR32:
						p->palette = VIDEO_PALETTE_RGB32;
						break;

					case LNT_PALETTE_UYVY:
						p->palette = VIDEO_PALETTE_UYVY;
						break;

					case LNT_PALETTE_YUYV:
						p->palette = VIDEO_PALETTE_YUYV;
						break;
				}
			}
			break;

		case VIDIOCSPICT:
			{
				struct video_picture *p = arg;

				LNT_DEBUG("VIDIOCSPICT\n");

				dev->cam->vsettings.brightness = p->brightness;
				
				if (p->palette && p->palette != dev->cam->vsettings.palette) {
					switch (p->palette) {
						case VIDEO_PALETTE_RGB24:
							dev->cam->vsettings.depth = 24;
							dev->cam->vsettings.palette = LNT_PALETTE_BGR24;
							break;

						case VIDEO_PALETTE_RGB32:
							dev->cam->vsettings.depth = 32;
							dev->cam->vsettings.palette = LNT_PALETTE_BGR32;
							break;

						case VIDEO_PALETTE_UYVY:
							dev->cam->vsettings.depth = 16;
							dev->cam->vsettings.palette = LNT_PALETTE_UYVY;
							break;

						case VIDEO_PALETTE_YUYV:
							dev->cam->vsettings.depth = 16;
							dev->cam->vsettings.palette = LNT_PALETTE_YUYV;
							break;

						default:
							return -EINVAL;
					}
				}

				LNT_DEBUG("VIDIOCSPICT done\n");
			}
			break;

		case VIDIOCGWIN:
			{
				struct video_window *vw = arg;

				LNT_DEBUG("VIDIOCGWIN\n");

				vw->x = 0;
				vw->y = 0;
				vw->width = dev->cam->view.x;
				vw->height = dev->cam->view.y;
				vw->chromakey = 0;
			}
			break;

		/*case VIDIOCSWIN:
			{
				struct video_window *vw = arg;

				LNT_DEBUG("VIDIOCSWIN\n");

				LNT_DEBUG("Set x=%d, y=%d\n", vw->x, vw->y);
				LNT_DEBUG("Set width=%d, height=%d\n", vw->width, vw->height);
				LNT_DEBUG("Flags = %X\n", vw->flags);
			
				// ISOC and URB cleanup
				usb_linect_rgb_isoc_cleanup(dev);

				// Select the new video mode
				if (v4l_linect_select_video_mode(dev, vw->width, vw->height)) {
					LNT_ERROR("Select video mode failed !\n");
					return -EAGAIN;
				}

				// Clear the buffers
				linect_clear_rgb_buffers(dev);

				// ISOC and URB init
				usb_linect_rgb_isoc_init(dev);
			}
			break;*/

		case VIDIOCGFBUF:
			{
				struct video_buffer *vb = arg;

				LNT_DEBUG("VIDIOCGFBUF\n");

				memset(vb, 0, sizeof(*vb));
			}
			break;

		case VIDIOCGMBUF:
			{
				int i;
				struct video_mbuf *vm = arg;

				LNT_DEBUG("VIDIOCGMBUF\n");

				memset(vm, 0, sizeof(*vm));

				vm->size = dev->cam->nbuffers * dev->cam->len_per_image;
				vm->frames = dev->cam->nbuffers;

				for (i=0; i<dev->cam->nbuffers; i++)
					vm->offsets[i] = i * dev->cam->len_per_image;
			}
			break;

		case VIDIOCMCAPTURE:
			{
				struct video_mmap *vm = arg;

				LNT_DEBUG("VIDIOCMCAPTURE format=%d\n", vm->format);

				if (vm->frame < 0 || vm->frame >= dev->cam->nbuffers)
					return -EINVAL;

				if (vm->format) {
					switch (vm->format) {
						case VIDEO_PALETTE_RGB32:
							break;

						case VIDEO_PALETTE_RGB24:
							break;

						case VIDEO_PALETTE_UYVY:
							break;

						case VIDEO_PALETTE_YUYV:
							break;

						default:
							return -EINVAL;
					}
				}

				if ((vm->width != dev->cam->view.x) || (vm->height != dev->cam->view.y)) 
					return -EAGAIN;

				if (dev->cam->image_used[vm->frame])
					return -EBUSY;

				dev->cam->image_used[vm->frame] = 1;

				LNT_DEBUG("VIDIOCMCAPTURE done\n");
			}
			break;

		/*case VIDIOCSYNC:
			{
				int ret;
				int *mbuf = arg;

				LNT_DEBUG("VIDIOCSYNC\n");

				if (*mbuf < 0 || *mbuf >= dev->cam->nbuffers)
					return -EINVAL;

				if (dev->cam->image_used[*mbuf] == 0)
					return -EINVAL;

				add_wait_queue(&dev->cam->wait_rgb_frame, &wait);

				while (dev->cam->full_frames == NULL) {
					if (dev->cam->error_status) {
						remove_wait_queue(&dev->cam->wait_rgb_frame, &wait);
						set_current_state(TASK_RUNNING);
						return -dev->cam->error_status;
					}

					if (signal_pending(current)) {
						remove_wait_queue(&dev->cam->wait_rgb_frame, &wait);
						set_current_state(TASK_RUNNING);
						return -ERESTARTSYS;
					}

					schedule();
					set_current_state(TASK_INTERRUPTIBLE);
				}

				remove_wait_queue(&dev->cam->wait_rgb_frame, &wait);
				set_current_state(TASK_RUNNING);

				LNT_DEBUG("VIDIOCSYNC: frame ready\n");

				dev->cam->fill_image = *mbuf;

				ret = linect_handle_rgb_frame(dev);

				if (ret != 0)
					LNT_ERROR("VIDIOCSYNC error !\n");

				dev->cam->image_used[*mbuf] = 0;
			}
			break;*/

		case VIDIOCGAUDIO:
			LNT_DEBUG("VIDIOCGAUDIO\n");
			return -EINVAL;
			break;

		case VIDIOCSAUDIO:
			LNT_DEBUG("VIDIOCSAUDIO\n");
			return -EINVAL;
			break;

		case VIDIOCGUNIT:
			{
				struct video_unit *vu = arg;

				vu->video = dev->cam->vdev->minor & 0x3f;
				vu->audio = -1;
				vu->vbi = -1;
				vu->radio = -1;
				vu->teletext = -1;
			}
			break;


		// Video 4 Linux v2

		case VIDIOC_QUERYCAP:
			{
				struct v4l2_capability *cap = arg;

				LNT_DEBUG("VIDIOC_QUERYCAP\n");

				memset(cap, 0, sizeof(*cap));
				strlcpy(cap->driver, "linect", sizeof(cap->driver));

				cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_READWRITE | V4L2_CAP_STREAMING;
				cap->version = (__u32) DRIVER_VERSION_NUM, strlcpy(cap->card, dev->cam->vdev->name, sizeof(cap->card));
			
				if (usb_make_path(dev->cam->udev, cap->bus_info, sizeof(cap->bus_info)) < 0)
					strlcpy(cap->bus_info, dev->cam->vdev->name, sizeof(cap->bus_info));
			}
			break;

		case VIDIOC_ENUMINPUT:
			{
				struct v4l2_input *i = arg;

				LNT_DEBUG("VIDIOC_ENUMINPUT %d\n", i->index);

				if (i->index)
					return -EINVAL;

				strlcpy(i->name, "USB", sizeof(i->name));
				i->type = V4L2_INPUT_TYPE_CAMERA;
			}
			break;

		case VIDIOC_G_INPUT:
			{
				struct v4l2_input *i = arg;

				LNT_DEBUG("GET INPUT %d\n", i->index);

				if (i->index)
					return -EINVAL;
			}
			break;

		case VIDIOC_S_INPUT:
			{
				struct v4l2_input *i = arg;

				LNT_DEBUG("SET INPUT %d\n", i->index);

				if (i->index != 0)
					return -EINVAL;
			}
			break;

		case VIDIOC_QUERYCTRL:
			{
				int i;
				int nbr;
				struct v4l2_queryctrl *c = arg;

				LNT_DEBUG("VIDIOC_QUERYCTRL id = %d\n", c->id);

				nbr = sizeof(linect_controls)/sizeof(struct v4l2_queryctrl);

				for (i=0; i<nbr; i++) {
					if (linect_controls[i].id == c->id) {
						LNT_DEBUG("VIDIOC_QUERYCTRL found\n");
						memcpy(c, &linect_controls[i], sizeof(struct v4l2_queryctrl));
						break;
					}
				}

				if (i >= nbr)
					return -EINVAL;
			}
			break;

		case VIDIOC_G_CTRL:
			{
				struct v4l2_control *c = arg;

				LNT_DEBUG("GET CTRL id=%d\n", c->id);

				switch (c->id) {
					case V4L2_CID_BRIGHTNESS:
						c->value = dev->cam->vsettings.brightness;
						break;
					case V4L2_CCID_MOTOR:
						c->value = dev->last_motor_status;
						break;
					case V4L2_CCID_LED:
						c->value = dev->last_led_status;
						break;

					default:
						return -EINVAL;
				}
			}
			break;

		case VIDIOC_S_CTRL:
			{
				struct v4l2_control *c = arg;

				LNT_DEBUG("SET CTRL id=%d value=%d\n", c->id, c->value);

				switch (c->id) {
					case V4L2_CID_BRIGHTNESS:
						dev->cam->vsettings.brightness = (0xff00 & c->value);
						break;
					case V4L2_CCID_MOTOR:
						if (c->value<-31 || c->value>31) return -EINVAL;
						linect_motor_set_tilt_degs(dev, c->value);
						//dev->last_motor_status = c->value;
						//printk("Set motor to: %d\n", c->value);
						break;
					case V4L2_CCID_LED:
						if (c->value<0 || c->value>6) return -EINVAL;
						linect_motor_set_led(dev, c->value);
						break;

					default:
						return -EINVAL;
				}
			}
			break;

		case VIDIOC_ENUM_FMT:
			{
				int index;
				struct v4l2_fmtdesc *fmtd = arg;

				LNT_DEBUG("VIDIOC_ENUM_FMT %d\n", fmtd->index);

				if (fmtd->index != 0)
					return -EINVAL;

				index = fmtd->index;

				memset(fmtd, 0, sizeof(*fmtd));

				fmtd->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
				fmtd->index = index;

				switch (index) {
					case 0:
						fmtd->flags = 0;
						fmtd->pixelformat = V4L2_PIX_FMT_RGB24;

						strcpy(fmtd->description, "rgb24");
						break;

					case 1:
						fmtd->flags = 0;
						fmtd->pixelformat = V4L2_PIX_FMT_RGB32;

						strcpy(fmtd->description, "rgb32");
						break;

					case 2:
						fmtd->flags = 0;
						fmtd->pixelformat = V4L2_PIX_FMT_BGR24;

						strcpy(fmtd->description, "bgr24");
						break;

					case 3:
						fmtd->flags = 0;
						fmtd->pixelformat = V4L2_PIX_FMT_BGR32;

						strcpy(fmtd->description, "bgr32");
						break;

					case 4:
						fmtd->flags = 0;
						fmtd->pixelformat = V4L2_PIX_FMT_UYVY;

						strcpy(fmtd->description, "uyvy");
						break;

					case 5:
						fmtd->flags = 0;
						fmtd->pixelformat = V4L2_PIX_FMT_YUYV;

						strcpy(fmtd->description, "yuyv");
						break;

					default:
						return -EINVAL;
				}
			}
			break;

		case VIDIOC_G_FMT:
			{
				struct v4l2_format *fmtd = arg;
				struct v4l2_pix_format pix_format;

				LNT_DEBUG("GET FMT %d\n", fmtd->type);

				if (fmtd->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
					return -EINVAL;

				pix_format.width = dev->cam->view.x;
				pix_format.height = dev->cam->view.y;
				pix_format.field = V4L2_FIELD_NONE;
				pix_format.colorspace = V4L2_COLORSPACE_SRGB;
				pix_format.priv = 0;

				switch (dev->cam->vsettings.palette) {
					case LNT_PALETTE_RGB24:
						pix_format.pixelformat = V4L2_PIX_FMT_RGB24;
						pix_format.sizeimage = pix_format.width * pix_format.height * 3;
						pix_format.bytesperline = 3 * pix_format.width;
						break;

					case LNT_PALETTE_RGB32:
						pix_format.pixelformat = V4L2_PIX_FMT_RGB32;
						pix_format.sizeimage = pix_format.width * pix_format.height * 4;
						pix_format.bytesperline = 4 * pix_format.width;
						break;

					case LNT_PALETTE_BGR24:
						pix_format.pixelformat = V4L2_PIX_FMT_BGR24;
						pix_format.sizeimage = pix_format.width * pix_format.height * 3;
						pix_format.bytesperline = 3 * pix_format.width;
						break;

					case LNT_PALETTE_BGR32:
						pix_format.pixelformat = V4L2_PIX_FMT_BGR32;
						pix_format.sizeimage = pix_format.width * pix_format.height * 4;
						pix_format.bytesperline = 4 * pix_format.width;
						break;

					case LNT_PALETTE_UYVY:
						pix_format.pixelformat = V4L2_PIX_FMT_UYVY;
						pix_format.sizeimage = pix_format.width * pix_format.height * 2;
						pix_format.bytesperline = 2 * pix_format.width;
						break;

					case LNT_PALETTE_YUYV:
						pix_format.pixelformat = V4L2_PIX_FMT_YUYV;
						pix_format.sizeimage = pix_format.width * pix_format.height * 2;
						pix_format.bytesperline = 2 * pix_format.width;
						break;
				}

				memcpy(&(fmtd->fmt.pix), &pix_format, sizeof(pix_format));
			}
			break;

		case VIDIOC_TRY_FMT:
			{
				struct v4l2_format *fmtd = arg;

				LNT_DEBUG("TRY FMT %d\n", fmtd->type);

				if (fmtd->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
					return -EINVAL;

				switch (fmtd->fmt.pix.pixelformat) {
					case V4L2_PIX_FMT_RGB24:
					case V4L2_PIX_FMT_BGR24:
						dev->cam->vsettings.depth = 24;
						break;

					case V4L2_PIX_FMT_RGB32:
					case V4L2_PIX_FMT_BGR32:
						dev->cam->vsettings.depth = 32;
						break;

					case V4L2_PIX_FMT_UYVY:
					case V4L2_PIX_FMT_YUYV:
						dev->cam->vsettings.depth = 16;
						break;

					default:
						return -EINVAL;
				}
				
				fmtd->fmt.pix.width = 640;
				fmtd->fmt.pix.height = 480;

			}
			break;

		case VIDIOC_S_FMT:
			{
				struct v4l2_format *fmtd = arg;

				LNT_DEBUG("SET FMT %d : %d\n", fmtd->type, fmtd->fmt.pix.pixelformat);

				if (fmtd->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
					return -EINVAL;

				switch (fmtd->fmt.pix.pixelformat) {
					case V4L2_PIX_FMT_RGB24:
						dev->cam->vsettings.depth = 24;
						dev->cam->vsettings.palette = LNT_PALETTE_RGB24;
						break;

					case V4L2_PIX_FMT_RGB32:
						dev->cam->vsettings.depth = 32;
						dev->cam->vsettings.palette = LNT_PALETTE_RGB32;
						break;

					case V4L2_PIX_FMT_BGR24:
						dev->cam->vsettings.depth = 24;
						dev->cam->vsettings.palette = LNT_PALETTE_BGR24;
						break;

					case V4L2_PIX_FMT_BGR32:
						dev->cam->vsettings.depth = 32;
						dev->cam->vsettings.palette = LNT_PALETTE_BGR32;
						break;

					case V4L2_PIX_FMT_UYVY:
						dev->cam->vsettings.depth = 16;
						dev->cam->vsettings.palette = LNT_PALETTE_UYVY;
						break;

					case V4L2_PIX_FMT_YUYV:
						dev->cam->vsettings.depth = 16;
						dev->cam->vsettings.palette = LNT_PALETTE_YUYV;
						break;

					default:
						return -EINVAL;
				}

				LNT_DEBUG("Set width=%d, height=%d\n", fmtd->fmt.pix.width, fmtd->fmt.pix.height);
				
				if (dev->cam->rgb_isoc_init_ok) {
				
					// ISOC and URB cleanup
					usb_linect_rgb_isoc_cleanup(dev);

					// Select the new video mode
					if (v4l_linect_select_video_mode(dev, fmtd->fmt.pix.width, fmtd->fmt.pix.height)) {
						LNT_ERROR("Select video mode failed !\n");
						return -EAGAIN;
					}

					// Clear the buffers
					linect_clear_rgb_buffers(dev);

					// ISOC and URB init
					usb_linect_rgb_isoc_init(dev);
				
				}
			}
			break;

		case VIDIOC_QUERYSTD:
			{
				LNT_DEBUG("QUERY STD\n");
				return -EINVAL;
			}
			break;

		case VIDIOC_G_STD:
			{
				v4l2_std_id *std = arg;

				LNT_DEBUG("GET STD\n");
		
				*std = V4L2_STD_UNKNOWN;
			}
			break;

		case VIDIOC_S_STD:
			{
				v4l2_std_id *std = arg;

				LNT_DEBUG("SET STD\n");
				
				if (*std != V4L2_STD_UNKNOWN)
					return -EINVAL;
			}
			break;

		case VIDIOC_ENUMSTD:
			{
				struct v4l2_standard *std = arg;

				LNT_DEBUG("VIDIOC_ENUMSTD\n");

				if (std->index != 0)
					return -EINVAL;

				std->id = V4L2_STD_UNKNOWN;
				strncpy(std->name, "webcam", sizeof(std->name));

				break;
			}

		case VIDIOC_REQBUFS:
			{
				int nbuffers;
				struct v4l2_requestbuffers *rb = arg;

				if (rb->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
					return -EINVAL;

				if (rb->memory != V4L2_MEMORY_MMAP)
					return -EINVAL;

				nbuffers = rb->count;

				if (nbuffers < 2)
					nbuffers = 2;
				else if (nbuffers > dev->cam->nbuffers)
					nbuffers = dev->cam->nbuffers;

				rb->count = dev->cam->nbuffers;
			}
			break;

		case VIDIOC_QUERYBUF:
			{
				int index;
				struct v4l2_buffer *buf = arg;

				LNT_DEBUG("QUERY BUFFERS %d %d\n", buf->index, dev->cam->nbuffers);

				if (buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
					return -EINVAL;

				if (buf->memory != V4L2_MEMORY_MMAP) 
					return -EINVAL;

				index = buf->index;

				if (index < 0 || index >= dev->cam->nbuffers)
					return -EINVAL;

				memset(buf, 0, sizeof(struct v4l2_buffer));

				buf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
				buf->index = index;
				buf->m.offset = index * dev->cam->len_per_image;
				buf->bytesused = dev->cam->view_size;
				buf->field = V4L2_FIELD_NONE;
				buf->memory = V4L2_MEMORY_MMAP;
				buf->length = dev->cam->len_per_image;
			}
			break;

		case VIDIOC_QBUF:
			{
				struct v4l2_buffer *buf = arg;

				//LNT_DEBUG("VIDIOC_QBUF\n");

				if (buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
					return -EINVAL;

				if (buf->memory != V4L2_MEMORY_MMAP)
					return -EINVAL;

				if (buf->index < 0 || buf->index >= dev->cam->nbuffers)
					return -EINVAL;

				buf->flags |= V4L2_BUF_FLAG_QUEUED;
				buf->flags &= ~V4L2_BUF_FLAG_DONE;
			}
			break;

		case VIDIOC_DQBUF:
			{
				int ret;
				struct v4l2_buffer *buf = arg;

				//LNT_DEBUG("VIDIOC_DQBUF\n");
				
				if (buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
					return -EINVAL;

				add_wait_queue(&dev->cam->wait_rgb_frame, &wait);

				while (dev->cam->full_frames == NULL) {
					if (dev->cam->error_status) {
						remove_wait_queue(&dev->cam->wait_rgb_frame, &wait);
						set_current_state(TASK_RUNNING);

						return -dev->cam->error_status;
					}

					if (signal_pending(current)) {
						remove_wait_queue(&dev->cam->wait_rgb_frame, &wait);
						set_current_state(TASK_RUNNING);

						return -ERESTARTSYS;
					}

					schedule();
					set_current_state(TASK_INTERRUPTIBLE);
				}

				remove_wait_queue(&dev->cam->wait_rgb_frame, &wait);
				set_current_state(TASK_RUNNING);

				//LNT_DEBUG("VIDIOC_DQBUF : frame ready.\n");

				ret = linect_handle_rgb_frame(dev);

				if (ret)
					return -EFAULT;

				buf->index = dev->cam->fill_image;
				buf->bytesused = dev->cam->view_size;
				buf->flags = V4L2_BUF_FLAG_MAPPED;
				buf->field = V4L2_FIELD_NONE;
				do_gettimeofday(&buf->timestamp);
				buf->sequence = 0;
				buf->memory = V4L2_MEMORY_MMAP;
				buf->m.offset = dev->cam->fill_image * dev->cam->len_per_image;
				buf->length = dev->cam->len_per_image; //buf->bytesused;

				linect_next_rgb_image(dev);
			}
			break;

		case VIDIOC_STREAMON:
			{
				LNT_DEBUG("VIDIOC_STREAMON\n");

				usb_linect_rgb_isoc_init(dev);
			}
			break;

		case VIDIOC_STREAMOFF:
			{
				LNT_DEBUG("VIDIOC_STREAMOFF\n");

				usb_linect_rgb_isoc_cleanup(dev);
			}
			break;

		case VIDIOC_G_PARM:
			{
				struct v4l2_streamparm *sp = arg;

				LNT_DEBUG("GET PARM %d\n", sp->type);

				if (sp->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
					return -EINVAL;

				sp->parm.capture.capability = 0;
				sp->parm.capture.capturemode = 0;
				sp->parm.capture.timeperframe.numerator = 1;
				sp->parm.capture.timeperframe.denominator = 30;
				sp->parm.capture.readbuffers = 2;
				sp->parm.capture.extendedmode = 0;
			}
			break;


		case VIDIOC_G_AUDIO:
			LNT_DEBUG("GET AUDIO\n");
			return -EINVAL;
			break;

		case VIDIOC_S_AUDIO:
			LNT_DEBUG("SET AUDIO\n");
			return -EINVAL;
			break;

		case VIDIOC_S_TUNER:
			LNT_DEBUG("SET TUNER\n");
			return -EINVAL;
			break;

		case VIDIOC_G_FBUF:
		case VIDIOC_S_FBUF:
		case VIDIOC_OVERLAY:
			return -EINVAL;
			break;

		case VIDIOC_G_TUNER:
		case VIDIOC_G_FREQUENCY:
		case VIDIOC_S_FREQUENCY:
			return -EINVAL;
			break;

		case VIDIOC_QUERYMENU:
			return -EINVAL;
			break;
/*
		case VIDIOC_CROPCAP:
			{
				struct v4l2_cropcap cc;

				cc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
				cc.pixelaspect.numerator = 1;
				cc.pixelaspect.denominator = 1;
				cc.bounds.top = 0;
				cc.bounds.left = 0;
				cc.bounds.width = 640;
				cc.bounds.height = 480;
				cc.defrect.top = 0;
				cc.defrect.left = 0;
				cc.defrect.width = 640;
				cc.defrect.height = 480;

				memcpy(arg, &cc, sizeof(cc));
			}
			break;
*/
		default:
			LNT_DEBUG("IOCTL unknown !\n");
			return -ENOIOCTLCMD;
	}

	return 0;
}

static long v4l_linect_depth_do_ioctl(struct file *fp,
		unsigned int cmd, void __user *arg)
{
	struct usb_linect *dev;
	struct video_device *vdev;

	DECLARE_WAITQUEUE(wait, current);
	
	vdev = video_devdata(fp);
	dev = video_get_drvdata(video_devdata(fp));

#if (CONFIG_LINECT_DEBUG == 1)
	v4l_printk_ioctl(cmd);
#endif

	switch (cmd) {
		// Video 4 Linux v1

		case VIDIOCGCAP:
			{
				struct video_capability *cap = arg;

				LNT_DEBUG("VIDIOCGCAP\n");

				memset(cap, 0, sizeof(*cap));
				strlcpy(cap->name, "linect", sizeof(cap->name));
				cap->type = VID_TYPE_CAPTURE;
				cap->channels = 1;
				cap->audios = 0;
				cap->minwidth = 640;
				cap->minheight = 480;
				cap->maxwidth = 640;
				cap->maxheight = 480;
			}
			break;
	
		case VIDIOCGCHAN:
			{
			    struct video_channel *v = arg;

				LNT_DEBUG("VIDIOCGCHAN\n");

			    if (v->channel != 0)
				    return -EINVAL;
			
				v->flags = 0;
			    v->tuners = 0;
			    v->type = VIDEO_TYPE_CAMERA;
			    strcpy(v->name, "Webcam");
			}
			break;

		case VIDIOCSCHAN:
			{
				struct video_channel *v = arg;

				LNT_DEBUG("VIDIOCSCHAN\n");

				if (v->channel != 0)
					return -EINVAL;
			}
			break;

		case VIDIOCGPICT:
			{
				struct video_picture *p = arg;

				LNT_DEBUG("VIDIOCGPICT\n");

				p->brightness = dev->cam->depth_vsettings.brightness;
				p->depth = 24;
				p->palette = VIDEO_PALETTE_RGB24;
			}
			break;

		case VIDIOCGWIN:
			{
				struct video_window *vw = arg;

				LNT_DEBUG("VIDIOCGWIN\n");

				vw->x = 0;
				vw->y = 0;
				vw->width = dev->cam->view.x;
				vw->height = dev->cam->view.y;
				vw->chromakey = 0;
			}
			break;

		case VIDIOCGFBUF:
			{
				struct video_buffer *vb = arg;

				LNT_DEBUG("VIDIOCGFBUF\n");

				memset(vb, 0, sizeof(*vb));
			}
			break;

		case VIDIOCGMBUF:
			{
				int i;
				struct video_mbuf *vm = arg;

				LNT_DEBUG("VIDIOCGMBUF\n");

				memset(vm, 0, sizeof(*vm));

				vm->size = dev->cam->nbuffers * dev->cam->len_per_image;
				vm->frames = dev->cam->nbuffers;

				for (i=0; i<dev->cam->nbuffers; i++)
					vm->offsets[i] = i * dev->cam->len_per_image;
			}
			break;

		case VIDIOCMCAPTURE:
			{
				struct video_mmap *vm = arg;

				LNT_DEBUG("VIDIOCMCAPTURE format=%d\n", vm->format);

				if (vm->frame < 0 || vm->frame >= dev->cam->nbuffers)
					return -EINVAL;

				if (vm->format) {
					switch (vm->format) {
						case VIDEO_PALETTE_RGB24:
							break;

						default:
							return -EINVAL;
					}
				}

				if ((vm->width != dev->cam->view.x) || (vm->height != dev->cam->view.y)) 
					return -EAGAIN;

				if (dev->cam->image_used[vm->frame])
					return -EBUSY;

				dev->cam->image_used_depth[vm->frame] = 1;

				LNT_DEBUG("VIDIOCMCAPTURE done\n");
			}
			break;

		case VIDIOCSYNC:
			{
				int ret;
				int *mbuf = arg;

				LNT_DEBUG("VIDIOCSYNC\n");

				if (*mbuf < 0 || *mbuf >= dev->cam->nbuffers)
					return -EINVAL;

				if (dev->cam->image_used_depth[*mbuf] == 0)
					return -EINVAL;

				add_wait_queue(&dev->cam->wait_depth_frame, &wait);

				while (dev->cam->full_frames_depth == NULL) {
					if (dev->cam->error_status) {
						remove_wait_queue(&dev->cam->wait_depth_frame, &wait);
						set_current_state(TASK_RUNNING);
						return -dev->cam->error_status;
					}

					if (signal_pending(current)) {
						remove_wait_queue(&dev->cam->wait_depth_frame, &wait);
						set_current_state(TASK_RUNNING);
						return -ERESTARTSYS;
					}

					schedule();
					set_current_state(TASK_INTERRUPTIBLE);
				}

				remove_wait_queue(&dev->cam->wait_depth_frame, &wait);
				set_current_state(TASK_RUNNING);

				LNT_DEBUG("VIDIOCSYNC: frame ready\n");

				dev->cam->fill_image_depth = *mbuf;

				ret = linect_handle_depth_frame(dev);

				if (ret != 0)
					LNT_ERROR("VIDIOCSYNC error !\n");

				dev->cam->image_used_depth[*mbuf] = 0;
			}
			break;

		case VIDIOCGAUDIO:
			LNT_DEBUG("VIDIOCGAUDIO\n");
			return -EINVAL;
			break;

		case VIDIOCSAUDIO:
			LNT_DEBUG("VIDIOCSAUDIO\n");
			return -EINVAL;
			break;

		case VIDIOCGUNIT:
			{
				struct video_unit *vu = arg;

				vu->video = dev->cam->vdev->minor & 0x3f;
				vu->audio = -1;
				vu->vbi = -1;
				vu->radio = -1;
				vu->teletext = -1;
			}
			break;


		// Video 4 Linux v2

		case VIDIOC_QUERYCAP:
			{
				struct v4l2_capability *cap = arg;

				LNT_DEBUG("VIDIOC_QUERYCAP\n");

				memset(cap, 0, sizeof(*cap));
				strlcpy(cap->driver, "linect", sizeof(cap->driver));

				cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_READWRITE | V4L2_CAP_STREAMING;
				cap->version = (__u32) DRIVER_VERSION_NUM, strlcpy(cap->card, dev->cam->vdev->name, sizeof(cap->card));
			
				if (usb_make_path(dev->cam->udev, cap->bus_info, sizeof(cap->bus_info)) < 0)
					strlcpy(cap->bus_info, dev->cam->vdev->name, sizeof(cap->bus_info));
			}
			break;

		case VIDIOC_ENUMINPUT:
			{
				struct v4l2_input *i = arg;

				LNT_DEBUG("VIDIOC_ENUMINPUT %d\n", i->index);

				if (i->index)
					return -EINVAL;

				strlcpy(i->name, "USB", sizeof(i->name));
				i->type = V4L2_INPUT_TYPE_CAMERA;
			}
			break;

		case VIDIOC_G_INPUT:
			{
				struct v4l2_input *i = arg;

				LNT_DEBUG("GET INPUT %d\n", i->index);

				if (i->index)
					return -EINVAL;
			}
			break;

		case VIDIOC_S_INPUT:
			{
				struct v4l2_input *i = arg;

				LNT_DEBUG("SET INPUT %d\n", i->index);

				if (i->index != 0)
					return -EINVAL;
			}
			break;

		case VIDIOC_QUERYCTRL:
			{
				int i;
				int nbr;
				struct v4l2_queryctrl *c = arg;

				LNT_DEBUG("VIDIOC_QUERYCTRL id = %d\n", c->id);

				nbr = sizeof(linect_depth_controls)/sizeof(struct v4l2_queryctrl);

				for (i=0; i<nbr; i++) {
					if (linect_depth_controls[i].id == c->id) {
						LNT_DEBUG("VIDIOC_QUERYCTRL found\n");
						memcpy(c, &linect_depth_controls[i], sizeof(struct v4l2_queryctrl));
						break;
					}
				}

				if (i >= nbr)
					return -EINVAL;
			}
			break;

		case VIDIOC_G_CTRL:
			{
				struct v4l2_control *c = arg;

				LNT_DEBUG("GET CTRL id=%d\n", c->id);

				switch (c->id) {
					case V4L2_CCID_MOTOR:
						c->value = dev->last_motor_status;
						break;
					case V4L2_CCID_LED:
						c->value = dev->last_led_status;
						break;

					default:
						return -EINVAL;
				}
			}
			break;

		case VIDIOC_S_CTRL:
			{
				struct v4l2_control *c = arg;

				LNT_DEBUG("SET CTRL id=%d value=%d\n", c->id, c->value);

				switch (c->id) {
					case V4L2_CCID_MOTOR:
						if (c->value<-31 || c->value>31) return -EINVAL;
						linect_motor_set_tilt_degs(dev, c->value);
						break;
					case V4L2_CCID_LED:
						if (c->value<0 || c->value>6) return -EINVAL;
						linect_motor_set_led(dev, c->value);
						break;

					default:
						return -EINVAL;
				}
			}
			break;

		case VIDIOC_ENUM_FMT:
			{
				int index;
				struct v4l2_fmtdesc *fmtd = arg;

				LNT_DEBUG("VIDIOC_ENUM_FMT %d\n", fmtd->index);

				if (fmtd->index != 0)
					return -EINVAL;

				index = fmtd->index;

				memset(fmtd, 0, sizeof(*fmtd));

				

				switch (index) {
					case 0:
						fmtd->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
						fmtd->index = index;
						fmtd->flags = 0;
						fmtd->pixelformat = V4L2_PIX_FMT_RGB24;

						strcpy(fmtd->description, "rgb24");
						break;
						
					case 1:
						fmtd->type = V4L2_BUF_TYPE_PRIVATE;
						fmtd->index = index;
						fmtd->flags = 0;
						fmtd->pixelformat = V4L2_PIX_FMT_DV;

						strcpy(fmtd->description, "raw depth info");
						break;

					default:
						return -EINVAL;
				}
			}
			break;

		case VIDIOC_G_FMT:
			{
				struct v4l2_format *fmtd = arg;
				struct v4l2_pix_format pix_format;

				LNT_DEBUG("GET FMT %d\n", fmtd->type);

				if (fmtd->type != V4L2_BUF_TYPE_VIDEO_CAPTURE && fmtd->type != V4L2_BUF_TYPE_PRIVATE)
					return -EINVAL;

				pix_format.width = dev->cam->view_depth.x;
				pix_format.height = dev->cam->view_depth.y;
				pix_format.field = V4L2_FIELD_NONE;
				pix_format.colorspace = V4L2_COLORSPACE_SRGB;

				switch (dev->cam->depth_vsettings.palette) {
					case LNT_PALETTE_RGB24:
						pix_format.pixelformat = V4L2_PIX_FMT_RGB24;
						pix_format.sizeimage = pix_format.width * pix_format.height * 3;
						pix_format.bytesperline = 3 * pix_format.width;
						pix_format.priv = 0;
						fmtd->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
						break;
					
					case LNT_PALETTE_DEPTHRAW:
						pix_format.pixelformat = V4L2_PIX_FMT_DV;
						pix_format.sizeimage = pix_format.width * pix_format.height * 2;
						pix_format.bytesperline = 2 * pix_format.width;
						pix_format.priv = 0;
						fmtd->type = V4L2_BUF_TYPE_PRIVATE;
						break;
				}
						

				memcpy(&(fmtd->fmt.pix), &pix_format, sizeof(pix_format));
			}
			break;

		case VIDIOC_TRY_FMT:
			{
				struct v4l2_format *fmtd = arg;

				LNT_DEBUG("TRY FMT %d\n", fmtd->type);

				if (fmtd->type != V4L2_BUF_TYPE_VIDEO_CAPTURE && fmtd->type != V4L2_BUF_TYPE_PRIVATE)
					return -EINVAL;
				
				switch (fmtd->type) {
					case V4L2_BUF_TYPE_VIDEO_CAPTURE:
						if (fmtd->fmt.pix.pixelformat == V4L2_PIX_FMT_RGB24) {
							dev->cam->vsettings.depth = 24;
						} else return -EINVAL;
					break;
					case V4L2_BUF_TYPE_PRIVATE:
						dev->cam->vsettings.depth = 16;
						break;
					default:
						return -EINVAL;
				}

				fmtd->fmt.pix.width = 640;
				fmtd->fmt.pix.height = 480;

			}
			break;

		case VIDIOC_S_FMT:
			{
				struct v4l2_format *fmtd = arg;

				LNT_DEBUG("SET FMT %d : %d\n", fmtd->type, fmtd->fmt.pix.pixelformat);

				if (fmtd->type != V4L2_BUF_TYPE_VIDEO_CAPTURE && fmtd->type != V4L2_BUF_TYPE_PRIVATE)
					return -EINVAL;

				switch (fmtd->type) {
					case V4L2_BUF_TYPE_VIDEO_CAPTURE:
						if (fmtd->fmt.pix.pixelformat == V4L2_PIX_FMT_RGB24) {
							dev->cam->depth_vsettings.depth = 24;
							dev->cam->depth_vsettings.palette = LNT_PALETTE_RGB24;
						} else return -EINVAL;
					break;
					case V4L2_BUF_TYPE_PRIVATE:
						dev->cam->depth_vsettings.depth = 16;
						dev->cam->depth_vsettings.palette = LNT_PALETTE_DEPTHRAW;
						break;
					default:
						return -EINVAL;
				}

				LNT_DEBUG("Set width=%d, height=%d\n", fmtd->fmt.pix.width, fmtd->fmt.pix.height);
				
				if (dev->cam->depth_isoc_init_ok) {
					// ISOC and URB cleanup
					usb_linect_depth_isoc_cleanup(dev);

					// Clear the buffers
					linect_clear_depth_buffers(dev);

					// ISOC and URB init
					usb_linect_depth_isoc_init(dev);
				
				}
			}
			break;

		case VIDIOC_QUERYSTD:
			{
				LNT_DEBUG("QUERY STD\n");
				return -EINVAL;
			}
			break;

		case VIDIOC_G_STD:
			{
				v4l2_std_id *std = arg;

				LNT_DEBUG("GET STD\n");
		
				*std = V4L2_STD_UNKNOWN;
			}
			break;

		case VIDIOC_S_STD:
			{
				v4l2_std_id *std = arg;

				LNT_DEBUG("SET STD\n");
				
				if (*std != V4L2_STD_UNKNOWN)
					return -EINVAL;
			}
			break;

		case VIDIOC_ENUMSTD:
			{
				struct v4l2_standard *std = arg;

				LNT_DEBUG("VIDIOC_ENUMSTD\n");

				if (std->index != 0)
					return -EINVAL;

				std->id = V4L2_STD_UNKNOWN;
				strncpy(std->name, "webcam", sizeof(std->name));

				break;
			}

		case VIDIOC_REQBUFS:
			{
				int nbuffers;
				struct v4l2_requestbuffers *rb = arg;

				if (rb->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
					return -EINVAL;

				if (rb->memory != V4L2_MEMORY_MMAP)
					return -EINVAL;

				nbuffers = rb->count;

				if (nbuffers < 2)
					nbuffers = 2;
				else if (nbuffers > dev->cam->nbuffers)
					nbuffers = dev->cam->nbuffers;

				rb->count = dev->cam->nbuffers;
			}
			break;

		case VIDIOC_QUERYBUF:
			{
				int index;
				struct v4l2_buffer *buf = arg;

				LNT_DEBUG("QUERY BUFFERS %d %d\n", buf->index, dev->cam->nbuffers);

				if (buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
					return -EINVAL;

				if (buf->memory != V4L2_MEMORY_MMAP) 
					return -EINVAL;

				index = buf->index;

				if (index < 0 || index >= dev->cam->nbuffers)
					return -EINVAL;

				memset(buf, 0, sizeof(struct v4l2_buffer));

				buf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
				buf->index = index;
				buf->m.offset = index * dev->cam->len_per_image;
				buf->bytesused = dev->cam->view_size;
				buf->field = V4L2_FIELD_NONE;
				buf->memory = V4L2_MEMORY_MMAP;
				buf->length = dev->cam->len_per_image;
			}
			break;

		case VIDIOC_QBUF:
			{
				struct v4l2_buffer *buf = arg;

				LNT_DEBUG("VIDIOC_QBUF\n");

				if (buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
					return -EINVAL;

				if (buf->memory != V4L2_MEMORY_MMAP)
					return -EINVAL;

				if (buf->index < 0 || buf->index >= dev->cam->nbuffers)
					return -EINVAL;

				buf->flags |= V4L2_BUF_FLAG_QUEUED;
				buf->flags &= ~V4L2_BUF_FLAG_DONE;
			}
			break;

		case VIDIOC_DQBUF:
			{
				int ret;
				struct v4l2_buffer *buf = arg;

				LNT_DEBUG("VIDIOC_DQBUF\n");
				
				if (buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
					return -EINVAL;

				add_wait_queue(&dev->cam->wait_depth_frame, &wait);

				while (dev->cam->full_frames_depth == NULL) {
					if (dev->cam->error_status) {
						remove_wait_queue(&dev->cam->wait_depth_frame, &wait);
						set_current_state(TASK_RUNNING);

						return -dev->cam->error_status;
					}

					if (signal_pending(current)) {
						remove_wait_queue(&dev->cam->wait_depth_frame, &wait);
						set_current_state(TASK_RUNNING);

						return -ERESTARTSYS;
					}

					schedule();
					set_current_state(TASK_INTERRUPTIBLE);
				}

				remove_wait_queue(&dev->cam->wait_depth_frame, &wait);
				set_current_state(TASK_RUNNING);

				LNT_DEBUG("VIDIOC_DQBUF : frame ready.\n");

				ret = linect_handle_depth_frame(dev);

				if (ret)
					return -EFAULT;

				buf->index = dev->cam->fill_image_depth;
				buf->bytesused = dev->cam->view_size;
				buf->flags = V4L2_BUF_FLAG_MAPPED;
				buf->field = V4L2_FIELD_NONE;
				do_gettimeofday(&buf->timestamp);
				buf->sequence = 0;
				buf->memory = V4L2_MEMORY_MMAP;
				buf->m.offset = dev->cam->fill_image_depth * dev->cam->len_per_image;
				buf->length = dev->cam->len_per_image; //buf->bytesused;

				linect_next_depth_image(dev);
			}
			break;

		case VIDIOC_STREAMON:
			{
				LNT_DEBUG("VIDIOC_STREAMON\n");

				usb_linect_depth_isoc_init(dev);
			}
			break;

		case VIDIOC_STREAMOFF:
			{
				LNT_DEBUG("VIDIOC_STREAMOFF\n");

				usb_linect_depth_isoc_cleanup(dev);
			}
			break;

		case VIDIOC_G_PARM:
			{
				struct v4l2_streamparm *sp = arg;

				LNT_DEBUG("GET PARM %d\n", sp->type);

				if (sp->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
					return -EINVAL;

				sp->parm.capture.capability = 0;
				sp->parm.capture.capturemode = 0;
				sp->parm.capture.timeperframe.numerator = 1;
				sp->parm.capture.timeperframe.denominator = 30;
				sp->parm.capture.readbuffers = 2;
				sp->parm.capture.extendedmode = 0;
			}
			break;


		case VIDIOC_G_AUDIO:
			LNT_DEBUG("GET AUDIO\n");
			return -EINVAL;
			break;

		case VIDIOC_S_AUDIO:
			LNT_DEBUG("SET AUDIO\n");
			return -EINVAL;
			break;

		case VIDIOC_S_TUNER:
			LNT_DEBUG("SET TUNER\n");
			return -EINVAL;
			break;

		case VIDIOC_G_FBUF:
		case VIDIOC_S_FBUF:
		case VIDIOC_OVERLAY:
			return -EINVAL;
			break;

		case VIDIOC_G_TUNER:
		case VIDIOC_G_FREQUENCY:
		case VIDIOC_S_FREQUENCY:
			return -EINVAL;
			break;

		case VIDIOC_QUERYMENU:
			return -EINVAL;
			break;
/*
		case VIDIOC_CROPCAP:
			{
				struct v4l2_cropcap cc;

				cc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
				cc.pixelaspect.numerator = 1;
				cc.pixelaspect.denominator = 1;
				cc.bounds.top = 0;
				cc.bounds.left = 0;
				cc.bounds.width = 640;
				cc.bounds.height = 480;
				cc.defrect.top = 0;
				cc.defrect.left = 0;
				cc.defrect.width = 640;
				cc.defrect.height = 480;

				memcpy(arg, &cc, sizeof(cc));
			}
			break;
*/
		default:
			LNT_DEBUG("IOCTL unknown !\n");
			return -ENOIOCTLCMD;
	}

	return 0;
}


/** 
 * @param fp File pointer
 * @param cmd Command
 * @param arg Arguements of the command
 * 
 * @returns 0 if all is OK
 *
 * @brief Manage IOCTL
 *
 * This function permits to manage all the IOCTL from the application.
 */
static long v4l_linect_rgb_ioctl(struct file *fp,
		unsigned int cmd, unsigned long arg)
{
	long err;
	struct usb_linect *dev;
	struct video_device *vdev;
	
	vdev = video_devdata(fp);
	dev = video_get_drvdata(video_devdata(fp));

	//LNT_DEBUG("v4l_linect_ioctl %02X\n", (unsigned char) cmd);

	if (dev == NULL)
		return -EFAULT;

	if (vdev == NULL)
		return -EFAULT;

	mutex_lock(&dev->cam->modlock_rgb); 

	err = video_usercopy(fp, cmd, arg, v4l_linect_rgb_do_ioctl);

	mutex_unlock(&dev->cam->modlock_rgb);

	return err;
}

static long v4l_linect_depth_ioctl(struct file *fp,
		unsigned int cmd, unsigned long arg)
{
	long err;
	struct usb_linect *dev;
	struct video_device *vdev;
	
	vdev = video_devdata(fp);
	dev = video_get_drvdata(video_devdata(fp));

	//LNT_DEBUG("v4l_linect_ioctl %02X\n", (unsigned char) cmd);

	if (dev == NULL)
		return -EFAULT;

	if (vdev == NULL)
		return -EFAULT;

	mutex_lock(&dev->cam->modlock_depth); 

	err = video_usercopy(fp, cmd, arg, v4l_linect_depth_do_ioctl);

	mutex_unlock(&dev->cam->modlock_depth);

	return err;
}


/** 
 * @param dev Device structure
 * 
 * @returns 0 if all is OK
 *
 * @brief Register the video device
 *
 * This function permits to register the USB device to the video device.
 */
int v4l_linect_register_rgb_video_device(struct usb_linect *dev)
{
	int err;

	snprintf(dev->cam->vdev->name, 15, "%s %d", DRIVER_V4L_NAME, dev->index);

	dev->cam->vdev->dev = dev->cam->interface->dev;
	dev->cam->vdev->fops = &v4l_linect_rgb_fops;
	dev->cam->vdev->release = video_device_release;
	dev->cam->vdev->minor = -1;

	video_set_drvdata(dev->cam->vdev, dev);

	err = video_register_device(dev->cam->vdev, VFL_TYPE_GRABBER, -1);

	if (err)
		LNT_ERROR("Video register fail !\n");
	else
		LNT_INFO("Linect is now controlling video device /dev/video%d\n", dev->cam->vdev->minor);

	return err;
}

int v4l_linect_register_depth_video_device(struct usb_linect *dev)
{
	int err;

	snprintf(dev->cam->vdev->name, 15, "%s %d", DRIVER_V4L_NAME, dev->index);

	dev->cam->depth_vdev->dev = dev->cam->interface->dev;
	dev->cam->depth_vdev->fops = &v4l_linect_depth_fops;
	dev->cam->depth_vdev->release = video_device_release;
	dev->cam->depth_vdev->minor = -1;

	video_set_drvdata(dev->cam->depth_vdev, dev);

	err = video_register_device(dev->cam->depth_vdev, VFL_TYPE_GRABBER, -1);

	if (err)
		LNT_ERROR("Video register fail !\n");
	else
		LNT_INFO("Linect is now controlling depth video device /dev/video%d\n", dev->cam->depth_vdev->minor);

	return err;
}


/** 
 * @param dev Device structure
 * 
 * @returns 0 if all is OK
 *
 * @brief Unregister the video device
 *
 * This function permits to unregister the video device.
 */
int v4l_linect_unregister_rgb_video_device(struct usb_linect *dev)
{
	LNT_INFO("Kinect release resources video device /dev/video%d\n", dev->cam->vdev->minor);

	video_set_drvdata(dev->cam->vdev, NULL);
	video_unregister_device(dev->cam->vdev);

	return 0;
}

int v4l_linect_unregister_depth_video_device(struct usb_linect *dev)
{
	LNT_INFO("Kinect release depth resources video device /dev/video%d\n", dev->cam->vdev->minor);

	video_set_drvdata(dev->cam->depth_vdev, NULL);
	video_unregister_device(dev->cam->depth_vdev);

	return 0;
}


/**
 * @var v4l_linect_fops
 *
 * This variable contains some callback
 */
static struct v4l2_file_operations v4l_linect_rgb_fops = {
	.owner = THIS_MODULE,
	.open = v4l_linect_rgb_open,
	.release = v4l_linect_rgb_release,
	.read = v4l_linect_rgb_read,
	.poll = v4l_linect_rgb_poll,
	.mmap = v4l_linect_rgb_mmap,
	.ioctl = v4l_linect_rgb_ioctl,
/*#ifdef CONFIG_COMPAT
	.compat_ioctl = v4l_compat_ioctl32,
#endif*/
};

static struct v4l2_file_operations v4l_linect_depth_fops = {
	.owner = THIS_MODULE,
	.open = v4l_linect_depth_open,
	.release = v4l_linect_depth_release,
	.read = v4l_linect_depth_read,
	.poll = v4l_linect_depth_poll,
	.mmap = v4l_linect_depth_mmap,
	.ioctl = v4l_linect_depth_ioctl,
/*#ifdef CONFIG_COMPAT
	.compat_ioctl = v4l_compat_ioctl32,
#endif*/
};

