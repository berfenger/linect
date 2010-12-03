/** 
 * @file linect-buf.c
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

#include <linux/usb.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>

#include "linect.h"


/** 
 * @var default_nbrframebuf
 *   Number of frame buffer by default
 */
static int default_nbrframebuf = 3;


/** 
 * @param size Size of memory
 * 
 * @returns Address on the allocated memory
 *
 * @brief Allocate a buffer.
 *
 * This function permits to allocate a buffer in memory.
 */
void * linect_rvmalloc(unsigned long size)
{
	void *mem;
	unsigned long addr;

	size = PAGE_ALIGN(size);
	mem = vmalloc(size);

	if (!mem)
		return NULL;

	memset(mem, 0, size);

	addr = (unsigned long) mem;

	while (size > 0) {
		SetPageReserved(vmalloc_to_page((void *) addr));
		addr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}

	return mem;
}


/** 
 * @param mem Memory address
 * @param size Size of allocated memory
 *
 * @brief Free a buffer
 *
 * This function permits to free a buffer.
 */
void linect_rvfree(void *mem, unsigned long size)
{
	unsigned long addr;

	if (!mem)
		return;

	addr = (unsigned long) mem;

	while ((long) size > 0) {
		ClearPageReserved(vmalloc_to_page((void *) addr));
		addr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}

	vfree(mem);
}


/** 
 * @param dev Device structure
 * 
 * @returns 0 if all is OK
 *
 * @brief Allocate all ISOC buffers.
 *
 * This function permits to reserved the memory for each ISOC buffer.
 */
int linect_allocate_rgb_buffers(struct usb_linect *dev)
{
	int i;
	void *kbuf;

	LNT_DEBUG("Allocate video buffers\n");

	if (dev == NULL)
		return -ENXIO;

	// Allocate frame buffer structure
	if (dev->cam->framebuf == NULL) {
		kbuf = kzalloc(default_nbrframebuf * sizeof(struct linect_frame_buf), GFP_KERNEL);

		if (kbuf == NULL) {
			LNT_ERROR("Failed to allocate frame buffer structure\n");
			return -ENOMEM;
		}

		dev->cam->framebuf = kbuf;
	}

	// Create frame buffers and make circular ring
	for (i=0; i<default_nbrframebuf; i++) {
		if (dev->cam->framebuf[i].data == NULL) {
			kbuf = vmalloc(LNT_FRAME_SIZE);

			if (kbuf == NULL) {
				LNT_ERROR("Failed to allocate frame buffer %d\n", i);
				return -ENOMEM;
			}

			dev->cam->framebuf[i].data = kbuf;
			memset(kbuf, 0, LNT_FRAME_SIZE);
		}
	}

	// Allocate image buffer; double buffer for mmap()
	kbuf = linect_rvmalloc(dev->cam->nbuffers * dev->cam->len_per_image);

	if (kbuf == NULL) {
		LNT_ERROR("Failed to allocate image buffer(s). needed (%d)\n",
				dev->cam->nbuffers * dev->cam->len_per_image);
		return -ENOMEM;
	}

	dev->cam->image_data = kbuf;

	for (i = 0; i < dev->cam->nbuffers; i++) {
		dev->cam->images[i].offset = i * dev->cam->len_per_image;
		dev->cam->images[i].vma_use_count = 0;
	}

	for (; i < LNT_MAX_IMAGES; i++)
		dev->cam->images[i].offset = 0;

	kbuf = NULL;
	
	return 0;
}

int linect_allocate_depth_buffers(struct usb_linect *dev)
{
	int i;
	void *kbuf;

	LNT_DEBUG("Allocate video buffers\n");

	if (dev == NULL)
		return -ENXIO;

	// Allocate frame buffer structure
	if (dev->cam->framebuf_depth == NULL) {
		kbuf = kzalloc(default_nbrframebuf * sizeof(struct linect_frame_buf), GFP_KERNEL);

		if (kbuf == NULL) {
			LNT_ERROR("Failed to allocate frame buffer structure\n");
			return -ENOMEM;
		}

		dev->cam->framebuf_depth = kbuf;
	}

	// Create frame buffers and make circular ring
	for (i=0; i<default_nbrframebuf; i++) {
		if (dev->cam->framebuf_depth[i].data == NULL) {
			kbuf = vmalloc(LNT_FRAME_SIZE);

			if (kbuf == NULL) {
				LNT_ERROR("Failed to allocate frame buffer %d\n", i);
				return -ENOMEM;
			}

			dev->cam->framebuf_depth[i].data = kbuf;
			memset(kbuf, 0, LNT_FRAME_SIZE);
		}
	}

	// Allocate image buffer; double buffer for mmap()
	kbuf = linect_rvmalloc(dev->cam->nbuffers_depth * dev->cam->len_per_image_depth);

	if (kbuf == NULL) {
		LNT_ERROR("Failed to allocate image buffer(s). needed (%d)\n",
				dev->cam->nbuffers_depth * dev->cam->len_per_image_depth);
		return -ENOMEM;
	}

	dev->cam->image_data_depth = kbuf;

	for (i = 0; i < dev->cam->nbuffers_depth; i++) {
		dev->cam->images_depth[i].offset = i * dev->cam->len_per_image_depth;
		dev->cam->images_depth[i].vma_use_count = 0;
	}

	for (; i < LNT_MAX_IMAGES; i++)
		dev->cam->images_depth[i].offset = 0;

	kbuf = NULL;

	kbuf = linect_rvmalloc(640*480*2);

	if (kbuf == NULL) {
		LNT_ERROR("Failed to allocate image temp buffer. needed (%d)\n",
				640*480*2);
		return -ENOMEM;
	}
	dev->cam->image_tmp = kbuf;
	
	
	/*kbuf = linect_rvmalloc(4096);

	if (kbuf == NULL) {
		LNT_ERROR("Failed to allocate depth gamma buffer. needed (%d)\n",
				4096);
		return -ENOMEM;
	}
	dev->cam->depth_gamma = kbuf;
	
	// Init gamma
	for (i=0; i<2048; i++) {
		v = i/2048.0;
		v = v*v*v* 6;
		dev->cam->depth_gamma[i] = v*6*256;
	}*/
	
	return 0;
}


/** 
 * @param dev Device structure
 * 
 * @returns 0 if all is OK
 *
 * @brief Reset all ISOC buffers.
 *
 * This function permits to reset all ISOC buffers.
 */
int linect_reset_rgb_buffers(struct usb_linect *dev)
{
	int i;
	unsigned long flags;

	LNT_DEBUG("Reset all buffers\n");

	spin_lock_irqsave(&dev->cam->spinlock_rgb, flags);

	dev->cam->full_frames = NULL;
	dev->cam->full_frames_tail = NULL;

	for (i=0; i<dev->cam->nbuffers; i++) {
		dev->cam->framebuf[i].filled = 0;
		dev->cam->framebuf[i].errors = 0;

		if (i > 0)
			dev->cam->framebuf[i].next = &dev->cam->framebuf[i - 1];
		else
			dev->cam->framebuf->next = NULL;
	}

	dev->cam->empty_frames = &dev->cam->framebuf[dev->cam->nbuffers - 1];
	dev->cam->empty_frames_tail = dev->cam->framebuf;
	dev->cam->read_frame = NULL;
	dev->cam->fill_frame = dev->cam->empty_frames;
	dev->cam->empty_frames = dev->cam->empty_frames->next;

	dev->cam->image_read_pos = 0;
	dev->cam->fill_image = 0;

	spin_unlock_irqrestore(&dev->cam->spinlock_rgb, flags);

	for (i=0; i<dev->cam->nbuffers; i++)
		dev->cam->image_used[i] = 0;
	
	return 0;
}

int linect_reset_depth_buffers(struct usb_linect *dev)
{
	int i;
	unsigned long flags;

	LNT_DEBUG("Reset all buffers\n");

	spin_lock_irqsave(&dev->cam->spinlock_depth, flags);

	dev->cam->full_frames_depth = NULL;
	dev->cam->full_frames_tail_depth = NULL;

	for (i=0; i<dev->cam->nbuffers_depth; i++) {
		dev->cam->framebuf_depth[i].filled = 0;
		dev->cam->framebuf_depth[i].errors = 0;

		if (i > 0)
			dev->cam->framebuf_depth[i].next = &dev->cam->framebuf_depth[i - 1];
		else
			dev->cam->framebuf_depth->next = NULL;
	}

	dev->cam->empty_frames_depth = &dev->cam->framebuf_depth[dev->cam->nbuffers_depth - 1];
	dev->cam->empty_frames_tail_depth = dev->cam->framebuf_depth;
	dev->cam->read_frame_depth = NULL;
	dev->cam->fill_frame_depth = dev->cam->empty_frames_depth;
	dev->cam->empty_frames_depth = dev->cam->empty_frames_depth->next;

	dev->cam->image_read_pos_depth = 0;
	dev->cam->fill_image_depth = 0;

	spin_unlock_irqrestore(&dev->cam->spinlock_depth, flags);

	for (i=0; i<dev->cam->nbuffers_depth; i++)
		dev->cam->image_used_depth[i] = 0;
	
	return 0;
}


/** 
 * @param dev Device structure
 * 
 * @returns 0 if all is OK
 *
 * @brief Clear current buffers.
 *
 * This function permits to clear the memory.
 */
int linect_clear_rgb_buffers(struct usb_linect *dev)
{
	memset(dev->cam->image_data, 0x00, dev->cam->nbuffers * dev->cam->len_per_image);

	return 0;
}

int linect_clear_depth_buffers(struct usb_linect *dev)
{
	memset(dev->cam->image_data_depth, 0x00, dev->cam->nbuffers_depth * dev->cam->len_per_image_depth);

	return 0;
}


/** 
 * @param dev Device structure
 * 
 * @returns 0 if all is OK
 *
 * @brief Release all buffers.
 *
 * This function permits to release and free the memory for each ISOC buffer.
 */
int linect_free_rgb_buffers(struct usb_linect *dev)
{
	int i;

	LNT_DEBUG("Free buffers\n");

	if (dev == NULL)
		return -1;

	// Release frame buffers
	if (dev->cam->framebuf != NULL) {
		for (i=0; i<default_nbrframebuf; i++) {
			if (dev->cam->framebuf[i].data != NULL) {
				vfree(dev->cam->framebuf[i].data);
				dev->cam->framebuf[i].data = NULL;
			}
		}

		kfree(dev->cam->framebuf);
		dev->cam->framebuf = NULL;
	}

	// Release image buffers
	if (dev->cam->image_data != NULL)
		linect_rvfree(dev->cam->image_data, dev->cam->nbuffers * dev->cam->len_per_image);

	dev->cam->image_data = NULL;

	return 0;
}

int linect_free_depth_buffers(struct usb_linect *dev)
{
	int i;

	LNT_DEBUG("Free buffers\n");

	if (dev == NULL)
		return -1;

	// Release frame buffers
	if (dev->cam->framebuf_depth != NULL) {
		for (i=0; i<default_nbrframebuf; i++) {
			if (dev->cam->framebuf_depth[i].data != NULL) {
				vfree(dev->cam->framebuf_depth[i].data);
				dev->cam->framebuf_depth[i].data = NULL;
			}
		}

		kfree(dev->cam->framebuf_depth);
		dev->cam->framebuf_depth = NULL;
	}

	// Release image buffers
	if (dev->cam->image_data_depth != NULL)
		linect_rvfree(dev->cam->image_data_depth, dev->cam->nbuffers_depth * dev->cam->len_per_image_depth);

	dev->cam->image_data_depth = NULL;
	
	
	if (dev->cam->image_tmp != NULL)
		linect_rvfree(dev->cam->image_tmp, 640*480*2);
	
	dev->cam->image_tmp = NULL;
	
	/*if (dev->cam->depth_gamma != NULL)
		linect_rvfree(dev->cam->depth_gamma, 4096);
	
	dev->cam->depth_gamma = NULL;*/

	return 0;
}


/** 
 * @param dev Device structure
 *
 * @brief Prepare the next image.
 *
 * This function is called when an image is ready, so as to prepare the next image.
 */
void linect_next_rgb_image(struct usb_linect *dev)
{
	dev->cam->image_used[dev->cam->fill_image] = 0;
	dev->cam->fill_image = (dev->cam->fill_image + 1) % dev->cam->nbuffers;
}

void linect_next_depth_image(struct usb_linect *dev)
{
	dev->cam->image_used_depth[dev->cam->fill_image_depth] = 0;
	dev->cam->fill_image_depth = (dev->cam->fill_image_depth + 1) % dev->cam->nbuffers;
}


/** 
 * @param dev Device structure
 * 
 * @returns 0 if all is OK
 *
 * @brief Prepare the next frame.
 *
 * This function is called when a frame is ready, so as to prepare the next frame.
 */
int linect_next_rgb_frame(struct usb_linect *dev)
{
	int ret = 0;
	unsigned long flags;

	//LNT_DEBUG("Select next frame\n");

	spin_lock_irqsave(&dev->cam->spinlock_rgb, flags);

	if (dev->cam->fill_frame != NULL) {
		if (dev->cam->full_frames == NULL) {
			dev->cam->full_frames = dev->cam->fill_frame;
			dev->cam->full_frames_tail = dev->cam->full_frames;
		}
		else {
			dev->cam->full_frames_tail->next = dev->cam->fill_frame;
			dev->cam->full_frames_tail = dev->cam->fill_frame;
		}
	}

	if (dev->cam->empty_frames != NULL) {
		dev->cam->fill_frame = dev->cam->empty_frames;
		dev->cam->empty_frames = dev->cam->empty_frames->next;
	}
	else {
		if (dev->cam->full_frames == NULL) {
			LNT_ERROR("Neither empty or full frames available!\n");
			spin_unlock_irqrestore(&dev->cam->spinlock_rgb, flags);
			return -EINVAL;
		}

		dev->cam->fill_frame = dev->cam->full_frames;
		dev->cam->full_frames = dev->cam->full_frames->next;

		ret = 1;
	}

	dev->cam->fill_frame->next = NULL;

	spin_unlock_irqrestore(&dev->cam->spinlock_rgb, flags);

	return ret;
}

int linect_next_depth_frame(struct usb_linect *dev)
{
	int ret = 0;
	unsigned long flags;

	//LNT_DEBUG("Select next frame\n");

	spin_lock_irqsave(&dev->cam->spinlock_depth, flags);

	if (dev->cam->fill_frame_depth != NULL) {
		if (dev->cam->full_frames_depth == NULL) {
			dev->cam->full_frames_depth = dev->cam->fill_frame_depth;
			dev->cam->full_frames_tail_depth = dev->cam->full_frames_depth;
		}
		else {
			dev->cam->full_frames_tail_depth->next = dev->cam->fill_frame_depth;
			dev->cam->full_frames_tail_depth = dev->cam->fill_frame_depth;
		}
	}

	if (dev->cam->empty_frames_depth != NULL) {
		dev->cam->fill_frame_depth = dev->cam->empty_frames_depth;
		dev->cam->empty_frames_depth = dev->cam->empty_frames_depth->next;
	}
	else {
		if (dev->cam->full_frames_depth == NULL) {
			LNT_ERROR("Neither empty or full frames available!\n");
			spin_unlock_irqrestore(&dev->cam->spinlock_depth, flags);
			return -EINVAL;
		}

		dev->cam->fill_frame_depth = dev->cam->full_frames_depth;
		dev->cam->full_frames_depth = dev->cam->full_frames_depth->next;

		ret = 1;
	}

	dev->cam->fill_frame_depth->next = NULL;

	spin_unlock_irqrestore(&dev->cam->spinlock_depth, flags);

	return ret;
}


/** 
 * @param dev Device structure
 * 
 * @returns 0 if all is OK
 *
 * @brief Handler frame
 *
 * This function gets called for the isochronous pipe. This function is only called 
 * when a frame is ready. So we have to be fast to decompress the data.
 */
int linect_handle_rgb_frame(struct usb_linect *dev)
{
	int ret = 0;
	unsigned long flags;

	LNT_DEBUG("Sync Handle Frame\n");

	spin_lock_irqsave(&dev->cam->spinlock_rgb, flags);

	if (dev->cam->read_frame != NULL) {
		spin_unlock_irqrestore(&dev->cam->spinlock_rgb, flags);
		return ret;
	}

	if (dev->cam->full_frames == NULL) {
	}
	else {
		dev->cam->read_frame = dev->cam->full_frames;
		dev->cam->full_frames = dev->cam->full_frames->next;
		dev->cam->read_frame->next = NULL;
	}

	if (dev->cam->read_frame != NULL) {
		spin_unlock_irqrestore(&dev->cam->spinlock_rgb, flags);
		ret = linect_rgb_decompress(dev);
		spin_lock_irqsave(&dev->cam->spinlock_rgb, flags);

		if (dev->cam->empty_frames == NULL) {
			dev->cam->empty_frames = dev->cam->read_frame;
			dev->cam->empty_frames_tail = dev->cam->empty_frames;
		}
		else {
			dev->cam->empty_frames_tail->next = dev->cam->read_frame;
			dev->cam->empty_frames_tail = dev->cam->read_frame;
		}

		dev->cam->read_frame = NULL;
	}

	spin_unlock_irqrestore(&dev->cam->spinlock_rgb, flags);

	return ret;
}

int linect_handle_depth_frame(struct usb_linect *dev)
{
	int ret = 0;
	unsigned long flags;

	LNT_DEBUG("Sync Handle Frame\n");

	spin_lock_irqsave(&dev->cam->spinlock_depth, flags);

	if (dev->cam->read_frame_depth != NULL) {
		spin_unlock_irqrestore(&dev->cam->spinlock_depth, flags);
		return ret;
	}

	if (dev->cam->full_frames_depth == NULL) {
	}
	else {
		dev->cam->read_frame_depth = dev->cam->full_frames_depth;
		dev->cam->full_frames_depth = dev->cam->full_frames_depth->next;
		dev->cam->read_frame_depth->next = NULL;
	}

	if (dev->cam->read_frame_depth != NULL) {
		spin_unlock_irqrestore(&dev->cam->spinlock_depth, flags);
		ret = linect_depth_decompress(dev);
		//ret = linect_decompress(dev);
		//LNT_DEBUG("descomprime frame depth\n");
		spin_lock_irqsave(&dev->cam->spinlock_depth, flags);

		if (dev->cam->empty_frames_depth == NULL) {
			dev->cam->empty_frames_depth = dev->cam->read_frame_depth;
			dev->cam->empty_frames_tail_depth = dev->cam->empty_frames_depth;
		}
		else {
			dev->cam->empty_frames_tail_depth->next = dev->cam->read_frame_depth;
			dev->cam->empty_frames_tail_depth = dev->cam->read_frame_depth;
		}

		dev->cam->read_frame_depth = NULL;
	}

	spin_unlock_irqrestore(&dev->cam->spinlock_depth, flags);

	return ret;
}

