/** 
 * @file linect-usb.c
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

/* Module params */
static int freemotor = 0;
static int freeled = 0;
static int startupinit = 0;


// Index of Kinect device
int kindex = 1;

struct proc_dir_entry *linect_proc_root_entry;

struct usb_device *tmp_motor_dev = NULL;
int tmp_motor_index = -1;
struct mutex modlock_tmpmotor;
struct mutex modlock_proc;
int nmotors = 0;

//warn_slowpath_common


/**
 * @var linect_table
 * Define all the hotplug supported devices by this driver
 */
static struct usb_device_id linect_table[] = {
	{ USB_DEVICE(USB_MS_VENDOR_ID, USB_K1414_PRODUCT_ID) },
	{ USB_DEVICE(USB_MS_VENDOR_ID, USB_K1414_MOTOR_PRODUCT_ID) },
	{ }
};


MODULE_DEVICE_TABLE(usb, linect_table);		/**< Define the supported devices */

/* LINECT functs */

int fnusb_start_iso(struct usb_linect *dev, fnusb_isoc_stream *strm, int ep, int xfers, int pkts, int len)
{
	int ret, i, j;
	struct urb *urb;
	uint8_t *bufp;
	
	strm->dev = dev;
	strm->num_xfers = xfers;
	strm->pkts = pkts;
	strm->len = len;
	//strm->buffer = linect_rvmalloc(xfers * pkts * len);
	strm->buffer = kzalloc(xfers * pkts * len, GFP_KERNEL);
	strm->xfers = kzalloc(sizeof(struct urb*) * xfers, GFP_KERNEL);

	bufp = strm->buffer;

	for (i=0; i<xfers; i++) {
		LNT_DEBUG("Creating EP %02x transfer #%d\n", ep, i);
		strm->xfers[i] = usb_alloc_urb(pkts, GFP_KERNEL);
		
		urb = strm->xfers[i];

		urb->interval = 1; 
		urb->dev = dev->cam->udev;
		urb->pipe = usb_rcvisocpipe(dev->cam->udev, ep);
		urb->transfer_flags = URB_ISO_ASAP;
		urb->transfer_buffer = bufp;
		urb->transfer_buffer_length = pkts*len;
		urb->complete = usb_linect_isoc_handler;
		urb->context = strm;
		urb->start_frame = 0;
		urb->number_of_packets = pkts;
		
		for (j=0; j<pkts; j++) {
			urb->iso_frame_desc[j].offset = j * len;
			urb->iso_frame_desc[j].length = len;
		}

		ret = usb_submit_urb(strm->xfers[i], GFP_KERNEL);

		if (ret)
			LNT_ERROR("isoc_init() submit_urb %d failed with error %d\n", i, ret);
		else
			LNT_DEBUG("URB 0x%p submitted.\n", strm->xfers[i]);

		switch (ret) {
			case -ENOMEM:
				LNT_ERROR("ENOMEM\n");
				break;
			case -ENODEV:
				LNT_ERROR("ENODEV\n");
				break;
			case -ENXIO:
				LNT_ERROR("ENXIO\n");
				break;
			case -EINVAL:
				LNT_ERROR("EINVAL\n");
				break;
			case -EAGAIN:
				LNT_ERROR("EAGAIN\n");
				break;
			case -EFBIG:
				LNT_ERROR("EFBIG\n");
				break;
			case -EPIPE:
				LNT_ERROR("EPIPE\n");
				break;
			case -EMSGSIZE:
				LNT_ERROR("EMSGSIZE\n");
				break;
		}

		bufp += pkts*len;
	}

	return 0;

}

int linect_start_depth(struct usb_linect *dev)
{
	int res;
	
	dev->cam->depth_stream.dev = dev;
	dev->cam->depth_stream.pkts_per_frame = DEPTH_PKTS_PER_FRAME;
	dev->cam->depth_stream.pkt_size = DEPTH_PKTDSIZE;
	dev->cam->depth_stream.synced = 0;
	dev->cam->depth_stream.flag = 0x70;
	
	dev->cam->depth_isoc.type = ISOC_DEPTH;

	res = fnusb_start_iso(dev, &dev->cam->depth_isoc, 0x82, NUM_XFERS, PKTS_PER_XFER, DEPTH_PKTBUF);
	
	if (res) return res;
	
	dev->cam->depth_isoc_init_ok = 1;

	return res;
}

int linect_start_rgb(struct usb_linect *dev)
{
	int res;
	
	dev->cam->rgb_stream.dev = dev;
	dev->cam->rgb_stream.pkts_per_frame = RGB_PKTS_PER_FRAME;
	dev->cam->rgb_stream.pkt_size = RGB_PKTDSIZE;
	dev->cam->rgb_stream.synced = 0;
	dev->cam->rgb_stream.flag = 0x80;
	
	dev->cam->rgb_isoc.type = ISOC_RGB;

	res = fnusb_start_iso(dev, &dev->cam->rgb_isoc, 0x81, NUM_XFERS, PKTS_PER_XFER, RGB_PKTBUF);
	
	if (res) return res;
	
	dev->cam->rgb_isoc_init_ok = 1;
	
	return res;
}


/** 
 * @param dev Device structure
 * 
 * @returns 0 if all is OK
 *
 * @brief Initilize an isochronous pipe.
 *
 * This function permits to initialize an URB transfert (or isochronous pipe).
 */
int usb_linect_rgb_isoc_init(struct usb_linect *dev)
{

	if (dev == NULL)
		return -EFAULT;

	if (dev->cam->rgb_isoc_init_ok)
		return 0;


	LNT_DEBUG("usb_linect_isoc_init() rgb\n");
	
	linect_cam_start_rgb(dev);
	mutex_lock(&dev->cam->mutex_cam);
	linect_start_rgb(dev);
	mutex_unlock(&dev->cam->mutex_cam);
	
		
	// DEPTH

	return 0;
}

int usb_linect_depth_isoc_init(struct usb_linect *dev)
{

	if (dev == NULL)
		return -EFAULT;

	if (dev->cam->depth_isoc_init_ok)
		return 0;


	LNT_DEBUG("usb_linect_isoc_init() depth\n");
	
	
	
	linect_cam_start_depth(dev);
	mutex_lock(&dev->cam->mutex_cam);
	linect_start_depth(dev);
	mutex_unlock(&dev->cam->mutex_cam);

	return 0;
}

int stream_process(packet_stream *strm, uint8_t *buf, uint8_t *pkt, int len)
{
	struct pkt_hdr *hdr;
	uint8_t *data;
	int datalen;
	uint8_t sof, mof, eof;
	int got_frame;
	uint8_t lost;
	int left;
	uint8_t *dbuf;
	
	if (len < 12)
		return 0;
	
	hdr = (void*)pkt;
	data = pkt + sizeof(*hdr);
	datalen = len - sizeof(*hdr);

	if (hdr->magic[0] != 'R' || hdr->magic[1] != 'B') {
		LNT_DEBUG("[Stream %02x] Invalid magic %02x%02x\n", strm->flag, hdr->magic[0], hdr->magic[1]);
		return 0;
	}

	//printf("[Stream %02x] %02x\n", strm->flag, hdr->flag);

	sof = strm->flag|1;
	mof = strm->flag|2;
	eof = strm->flag|5;

	// sync if required, dropping packets until SOF
	if (!strm->synced) {
		if (hdr->flag != sof) {
			//printf("[Stream %02x] not synced yet...\n", strm->flag);
			return 0;
		}
		strm->synced = 1;
		strm->seq = hdr->seq;
		strm->pkt_num = 0;
		strm->valid_pkts = 0;
		strm->got_pkts = 0;
	}

	got_frame = 0;

	// handle lost packets
	if (strm->seq != hdr->seq) {
		lost = strm->seq - hdr->seq;
		LNT_DEBUG("[Stream %02x] lost %d packets\n", strm->flag, lost);
		if (lost > 5) {
			LNT_DEBUG("[Stream %02x] lost too many packets, resyncing...\n", strm->flag);
			strm->synced = 0;
			return 0;
		}
		strm->seq = hdr->seq;
		left = strm->pkts_per_frame - strm->pkt_num;
		if (left <= lost) {
			strm->pkt_num = lost - left;
			strm->valid_pkts = strm->got_pkts;
			strm->got_pkts = 0;
			got_frame = 1;
			strm->timestamp = strm->last_timestamp;
		} else {
			strm->pkt_num += lost;
		}
	}

	// check the header to make sure it's what we expect
	if (!(strm->pkt_num == 0 && hdr->flag == sof) &&
	    !(strm->pkt_num == strm->pkts_per_frame-1 && hdr->flag == eof) &&
	    !(strm->pkt_num > 0 && strm->pkt_num < strm->pkts_per_frame-1 && hdr->flag == mof)) {
		LNT_DEBUG("[Stream %02x] Inconsistent flag %02x with %d packets in buf (%d total), resyncing...\n",
		       strm->flag, hdr->flag, strm->pkt_num, strm->pkts_per_frame);
		strm->synced = 0;
		return 0;
	}

	// copy data
	if (datalen > strm->pkt_size) {
		LNT_DEBUG("[Stream %02x] Expected %d data bytes, but got %d. Dropping...\n", strm->flag, strm->pkt_size, datalen);
		return 0;
	}

	if (datalen != strm->pkt_size && hdr->flag != eof)
		LNT_DEBUG("[Stream %02x] Expected %d data bytes, but got only %d\n", strm->flag, strm->pkt_size, datalen);

	dbuf = buf + strm->pkt_num * strm->pkt_size;
	memcpy(dbuf, data, datalen);

	strm->pkt_num++;
	strm->seq++;
	strm->got_pkts++;

	strm->last_timestamp = hdr->timestamp;

	if (strm->pkt_num == strm->pkts_per_frame) {
		strm->pkt_num = 0;
		strm->valid_pkts = strm->got_pkts;
		strm->got_pkts = 0;
		strm->timestamp = hdr->timestamp;
		return 1;
	} else {
		return got_frame;
	}
}


/** 
 * @param urb URB structure
 *
 * @brief ISOC handler
 *
 * This function is called as an URB transfert is complete (Isochronous pipe).
 * So, the traitement is done in interrupt time, so it has be fast, not crash,
 * ans not stall. Neat.
 */
void usb_linect_isoc_handler(struct urb *urb)
{
	int i;
	int ret;

	int awake = 0;
	int framestatus;
	int framelen;

	unsigned char *fill = NULL;
	unsigned char *iso_buf = NULL;

	struct usb_linect *dev;
	fnusb_isoc_stream *isoc_stream;
	struct linect_frame_buf *framebuf;
	int got_frame;

	//LNT_DEBUG("Isoc handler\n");

	isoc_stream = (fnusb_isoc_stream *) urb->context;
	dev = isoc_stream->dev;

	if (dev == NULL) {
		LNT_ERROR("isoc_handler called with NULL device !\n");
		return;
	}

	if (urb->status == -ENOENT || urb->status == -ECONNRESET) {
		LNT_DEBUG("URB unlinked synchronuously !\n");
		return;
	}

	if (urb->status != -EINPROGRESS && urb->status != 0) {
		const char *errmsg;

		errmsg = "Unknown";

		switch(urb->status) {
			case -ENOSR:
				errmsg = "Buffer error (overrun)";
				break;

			case -EPIPE:
				errmsg = "Stalled (device not responding)";
				break;

			case -EOVERFLOW:
				errmsg = "Babble (bad cable?)";
				break;

			case -EPROTO:
				errmsg = "Bit-stuff error (bad cable?)";
				break;

			case -EILSEQ:
				errmsg = "CRC/Timeout (could be anything)";
				break;

			case -ETIMEDOUT:
				errmsg = "NAK (device does not respond)";
				break;
		}

		LNT_ERROR("isoc_handler() called with status %d [%s].\n", urb->status, errmsg);

		dev->cam->visoc_errors++;

		if (isoc_stream->type == ISOC_RGB)
		wake_up_interruptible(&dev->cam->wait_rgb_frame);
		else wake_up_interruptible(&dev->cam->wait_depth_frame);

		urb->dev = dev->cam->udev;
		ret = usb_submit_urb(urb, GFP_ATOMIC);

		if (ret != 0) {
			LNT_ERROR("Error (%d) re-submitting urb in linect_isoc_handler.\n", ret);
		}

		return;
	}

	if (isoc_stream->type == ISOC_RGB)
		framebuf = dev->cam->fill_frame;
	else
		framebuf = dev->cam->fill_frame_depth;

	if (framebuf == NULL) {
		LNT_ERROR("isoc_handler without valid fill frame !\n");
		
		if (isoc_stream->type == ISOC_RGB)
			wake_up_interruptible(&dev->cam->wait_rgb_frame);
		else
			wake_up_interruptible(&dev->cam->wait_depth_frame);

		urb->dev = dev->cam->udev;
		ret = usb_submit_urb(urb, GFP_ATOMIC);

		if (ret != 0) {
			LNT_ERROR("Error (%d) re-submitting urb in linect_isoc_handler.\n", ret);
		}

		return;
	}
	else {
		//fill = framebuf->data + framebuf->filled;
		fill = framebuf->data;
	}

	// Reset ISOC error counter
	dev->cam->visoc_errors = 0;

	// Compact data
	for (i=0; i<urb->number_of_packets; i++) {
		framestatus = urb->iso_frame_desc[i].status;
		framelen = urb->iso_frame_desc[i].actual_length;
		iso_buf = urb->transfer_buffer + urb->iso_frame_desc[i].offset;

		if (framestatus == 0) {
			if (isoc_stream->type == ISOC_RGB) {
				got_frame = stream_process(&dev->cam->rgb_stream, fill, iso_buf, framelen);
				if (got_frame) {
					// If there are errors, we skip a frame...
					if (linect_next_rgb_frame(dev))
						dev->cam->vframes_dumped++;

					awake = 1;
					framebuf = dev->cam->fill_frame;
					framebuf->filled = 0;
					framebuf->errors = 0;
					fill = framebuf->data;
				} else {
					framebuf->filled += RGB_PKTDSIZE;
				}
			} else if (isoc_stream->type == ISOC_DEPTH) {
				// DEPTH
				got_frame = stream_process(&dev->cam->depth_stream, fill, iso_buf, framelen);
				if (got_frame) {
					// If there are errors, we skip a frame...
					if (linect_next_depth_frame(dev))
						dev->cam->vframes_dumped++;

					awake = 1;
					framebuf = dev->cam->fill_frame_depth;
					framebuf->filled = 0;
					framebuf->errors = 0;
					fill = framebuf->data;
				} else {
					framebuf->filled += DEPTH_PKTDSIZE;
				}
			}
		}
		else {
			LNT_ERROR("Iso frame %d of USB has error %d\n", i, framestatus);
		}
	}

	if (awake == 1) {
		if (isoc_stream->type == ISOC_RGB)
			wake_up_interruptible(&dev->cam->wait_rgb_frame);
		else
			wake_up_interruptible(&dev->cam->wait_depth_frame);
		
	}

	urb->dev = dev->cam->udev;
	ret = usb_submit_urb(urb, GFP_ATOMIC);

	if (ret != 0) {
		LNT_ERROR("Error (%d) re-submitting urb in linect_isoc_handler.\n", ret);
	}
}

/* Kinect ISOC cleanup*/

int linect_isoc_depth_cleanup(struct usb_linect *dev)
{
	struct urb *urb;
	int i;
	for (i=0; i<dev->cam->depth_isoc.num_xfers; i++) {
	

		urb = dev->cam->depth_isoc.xfers[i];

		if (urb != 0) {
			if (dev->cam->depth_isoc_init_ok) {
				usb_kill_urb(urb);
			}
			
			usb_free_urb(urb);
			dev->cam->depth_isoc.xfers[i] = NULL;
		}
	}
	//linect_rvfree(dev->cam->depth_isoc.buffer, dev->cam->depth_isoc.num_xfers * dev->cam->depth_isoc.pkts * dev->cam->depth_isoc.len);
	kfree(dev->cam->depth_isoc.buffer);
	
	kfree(dev->cam->depth_isoc.xfers);
	return 0;
}

int linect_isoc_rgb_cleanup(struct usb_linect *dev)
{
	struct urb *urb;
	int i;
	for (i=0; i<dev->cam->rgb_isoc.num_xfers; i++) {
	

		urb = dev->cam->rgb_isoc.xfers[i];

		if (urb != 0) {
			if (dev->cam->rgb_isoc_init_ok) {
				usb_kill_urb(urb);
			}
			
			usb_free_urb(urb);
			dev->cam->rgb_isoc.xfers[i] = NULL;
		}
	}
	//linect_rvfree(dev->cam->rgb_isoc.buffer, dev->cam->rgb_isoc.num_xfers * dev->cam->rgb_isoc.pkts * dev->cam->rgb_isoc.len);
	kfree(dev->cam->rgb_isoc.buffer);
	kfree(dev->cam->rgb_isoc.xfers);
	return 0;
}

/** 
 * @param dev Device structure
 *
 * @brief Clean-up all the ISOC buffers
 *
 * This function permits to clean-up all the ISOC buffers.
 */
void usb_linect_rgb_isoc_cleanup(struct usb_linect *dev)
{

	LNT_DEBUG("rgb Isoc cleanup\n");

	if (dev == NULL)
		return;

	if (dev->cam->rgb_isoc_init_ok == 0)
		return;

	linect_cam_stop_rgb(dev);
	mutex_lock(&dev->cam->mutex_cam);
	linect_isoc_rgb_cleanup(dev);
	mutex_unlock(&dev->cam->mutex_cam);
	linect_cam_stop_rgb(dev);
	
	// DEPTH

	// All is done
	dev->cam->rgb_isoc_init_ok = 0;
}

void usb_linect_depth_isoc_cleanup(struct usb_linect *dev)
{

	LNT_DEBUG("depth Isoc cleanup\n");

	if (dev == NULL)
		return;

	if (dev->cam->depth_isoc_init_ok == 0)
		return;

	linect_cam_stop_depth(dev);
	mutex_lock(&dev->cam->mutex_cam);
	linect_isoc_depth_cleanup(dev);
	mutex_unlock(&dev->cam->mutex_cam);
	linect_cam_stop_depth(dev);

	// All is done
	dev->cam->depth_isoc_init_ok = 0;
}


/** 
 * @param dev 
 * 
 * @returns 0 if all is OK
 *
 * @brief Set the default value about the video settings.
 *
 * This function permits to set the video settings for each video camera model.
 * 
 */
static int usb_linect_default_settings(struct usb_linect *dev)
{
	dev->cam->vsettings.palette = LNT_PALETTE_RGB24;
	dev->cam->vsettings.depth = 24;
	dev->cam->vsettings.brightness = 0x7f00;
	
	dev->cam->depth_vsettings.palette = LNT_PALETTE_RGB24;
	dev->cam->depth_vsettings.depth = 24;
	dev->cam->depth_vsettings.brightness = 0x7f00;
	
	dev->cam->cam_tag = 0;

	return 0;
}


/** 
 * @param interface 
 * @param id 
 * 
 * @returns 0 if all is OK
 *
 * @brief Load the driver
 *
 * This function detects the device and allocate the buffers for the device
 * and the video interface.
 */
static int usb_linect_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
	int err;

	int vendor_id;
	int product_id;
	int webcam_model;
	int tmpi, try, found;

	struct usb_linect *dev = NULL;
	struct usb_device *udev = interface_to_usbdev(interface);


	// Get USB VendorID and ProductID
	vendor_id = le16_to_cpu(udev->descriptor.idVendor);
	product_id = le16_to_cpu(udev->descriptor.idProduct);

	// Check if we can handle this device
	LNT_DEBUG("Probe function called with VendorID=%04X, ProductID=%04X and InterfaceNumber=%d\n",
			vendor_id, product_id, interface->cur_altsetting->desc.bInterfaceNumber);

	if (interface->cur_altsetting->desc.bInterfaceNumber > 0)
		return -ENODEV;

	// Detect device
	if (vendor_id == USB_MS_VENDOR_ID) {
		switch (product_id) {

			case USB_K1414_PRODUCT_ID:
				LNT_INFO("MS Kinect Model 1414 found.\n");
				webcam_model = KNT_K1414;
				break;
			case USB_K1414_MOTOR_PRODUCT_ID:
				// Motor
				LNT_DEBUG("Kinect Motor found.\n");
				tmpi = -1;
				found = 0; try = 10;
				while(1) {
					mutex_lock(&modlock_tmpmotor);
					if (tmp_motor_dev == NULL) {
						tmp_motor_dev = udev;
						tmp_motor_index = kindex++;
						LNT_DEBUG("Got number %d current %d.\n", tmp_motor_index, kindex);
						tmpi = tmp_motor_index;
						found = 1;
						mutex_unlock(&modlock_tmpmotor);
						break;
					}
					mutex_unlock(&modlock_tmpmotor);
					schedule_timeout(50);
					try--;
					if (try == 0) break;
				}
				
				if (!found) { LNT_INFO("Kinect unplugged. Please connect the Kinect power source.\n"); return -ENODEV;}
				
				dev = kzalloc(sizeof(struct usb_linect), GFP_KERNEL);

				if (dev == NULL) {
					LNT_ERROR("Out of memory !\n");
					return -ENOMEM;
				}
				dev->type = KNT_TYPE_MOTOR;
				dev->index = tmpi;
				dev->motor_udev = udev;
				
				// Proc
				linect_proc_create(dev);
				
				mutex_lock(&modlock_tmpmotor);
				nmotors++;
				mutex_unlock(&modlock_tmpmotor);
				
				LNT_DEBUG("Kinect Motor ready.\n");
				
				usb_set_intfdata(interface, dev);
				return 0;
				break;
			default:
				LNT_ERROR("usb_linect_probe failed ! Kinect product 0x%04X is not supported.\n",
						le16_to_cpu(udev->descriptor.idProduct));
				return -ENODEV;
		}
	}
	else
		return -ENODEV;

	// Allocate structure, initialize pointers, mutexes, etc. and link it to the usb_device
	dev = kzalloc(sizeof(struct usb_linect), GFP_KERNEL);
	
	
	if (dev == NULL) {
		LNT_ERROR("Out of memory !\n");
		return -ENOMEM;
	}
	
	dev->cam = kzalloc(sizeof(struct linect_cam), GFP_KERNEL);
	
	if (dev->cam == NULL) {
		LNT_ERROR("Out of memory !\n");
		return -ENOMEM;
	}

	// Init mutexes, spinlock, etc.
	init_MUTEX(&dev->cam->mutex_rgb);
	init_MUTEX(&dev->cam->mutex_depth);
	mutex_init(&dev->mutex_motor);
	mutex_init(&dev->cam->mutex_cam);
	mutex_init(&dev->cam->modlock_rgb);
	mutex_init(&dev->cam->modlock_depth);
	spin_lock_init(&dev->cam->spinlock_rgb);
	spin_lock_init(&dev->cam->spinlock_depth);
	init_waitqueue_head(&dev->cam->wait_rgb_frame);
	init_waitqueue_head(&dev->cam->wait_depth_frame);

	// Save pointers
	dev->cam->webcam_model = webcam_model;
	dev->cam->udev = udev;
	dev->cam->interface = interface;

	// Read the product release 
	dev->cam->release = le16_to_cpu(udev->descriptor.bcdDevice);
	LNT_INFO("Release: %04x\n", dev->cam->release);

	// Constructor
	dev->cam->nbuffers = 2;
	dev->cam->len_per_image = PAGE_ALIGN((640 * 480 * 4));
	
	dev->cam->nbuffers_depth = 2;
	dev->cam->len_per_image_depth = PAGE_ALIGN((640 * 480 * 4));
	
	// Get Motor
	while(1) {
		mutex_lock(&modlock_tmpmotor);
		if (tmp_motor_dev != NULL) {
			dev->motor_udev = tmp_motor_dev;
			LNT_DEBUG("Got Kinect motor ref to dev!\n");
			tmp_motor_dev = NULL;
			dev->index = tmp_motor_index;
			tmp_motor_index = -1;
			mutex_unlock(&modlock_tmpmotor);
			break;
		}
		mutex_unlock(&modlock_tmpmotor);
	}
	
	// Params
	dev->freemotor = freemotor ? 1 : 0;
	dev->freeled = freeled ? 1 : 0;
	dev->cam->startupinit = startupinit ? 1 : 0;
	
	dev->type = KNT_TYPE_CAM;
	
	// Led
	if (!dev->freeled)
		linect_motor_set_led(dev, LED_GREEN);
	
	// Reset motor
	if (!dev->freemotor)
		linect_motor_set_tilt_degs(dev, 0);
	
	// V4L2 !!

	// Initialize the video device
	dev->cam->vdev = video_device_alloc();
	dev->cam->depth_vdev = video_device_alloc();

	if (!dev->cam->vdev) {
		kfree(dev);
		return -ENOMEM;
	}
	
	if (!dev->cam->depth_vdev) {
		kfree(dev);
		return -ENOMEM;
	}

	// Initialize the camera
	linect_cam_init(dev);
	
	// Register the video device
	err = v4l_linect_register_rgb_video_device(dev);

	if (err) {
		kfree(dev);
		return err;
	}
	
	err = v4l_linect_register_depth_video_device(dev);

	if (err) {
		kfree(dev);
		return err;
	}

	// Save our data pointer in this interface device
	usb_set_intfdata(interface, dev);

	// Default settings video device
	usb_linect_default_settings(dev);

	return 0;
}


/** 
 * @param interface 
 *
 * @brief This function is called when the device is disconnected
 *   or when the kernel module is unloaded.
 */
static void usb_linect_disconnect(struct usb_interface *interface)
{
	struct usb_linect *dev = usb_get_intfdata(interface);
	
	if (!dev) return;
	
	if (dev->type == KNT_TYPE_CAM) {
	
		LNT_INFO("Kinect camera disconnected.\n");

		usb_set_intfdata(interface, NULL);

		// We got unplugged; this is signalled by an EPIPE error code
		if (dev->cam->vopen_rgb || dev->cam->vopen_depth) {
			LNT_INFO("Kinect camera disconnected while in use !\n");
			dev->cam->error_status = EPIPE;
		}

		// Alert waiting processes
		wake_up_interruptible(&dev->cam->wait_rgb_frame);
		wake_up_interruptible(&dev->cam->wait_depth_frame);

		// Wait until device is closed
		while (dev->cam->vopen_rgb)
			schedule();
		while (dev->cam->vopen_depth)
			schedule();

		// Unregister the video device
		v4l_linect_unregister_rgb_video_device(dev);
		v4l_linect_unregister_depth_video_device(dev);
	
	} else {
		LNT_INFO("Kinect motor disconnected.\n");
		
		mutex_lock(&modlock_tmpmotor);
			nmotors--;
			if (tmp_motor_dev != NULL) {
				tmp_motor_dev = NULL;
				
			}
		mutex_unlock(&modlock_tmpmotor);
		
		LNT_DEBUG("Quedan %d motores\n", nmotors);
		
		linect_proc_destroy(dev);
		
		usb_set_intfdata(interface, NULL);
	}
}




/**
 * @var usb_linect_driver
 *
 * This variable contains some callback
 */
static struct usb_driver usb_linect_driver = {
	.name = "usb_linect_driver",
	.probe = usb_linect_probe,
	.disconnect = usb_linect_disconnect,
	.id_table = linect_table,
};


module_param(freemotor, int, 0444);
module_param(freeled, int, 0444);
module_param(startupinit, int, 0444);


/** 
 * @returns 0 if all is OK
 *
 * @brief Initialize the driver.
 *
 * This function is called at first.
 * This function permits to define the default values from the command line.
 */
static int __init usb_linect_init(void)
{
	int result;


	LNT_INFO("MS Kinect driver startup.\n");

	mutex_init(&modlock_tmpmotor);
	mutex_init(&modlock_proc);

	// Register the driver with the USB subsystem
	result = usb_register(&usb_linect_driver);

	if (result)
		LNT_ERROR("usb_register failed ! Error number %d\n", result);

	LNT_INFO(DRIVER_VERSION " : " DRIVER_DESC "\n");

	return result;
}


/** 
 * @brief Close the driver
 *
 * This function is called at last when you unload the driver.
 */
static void __exit usb_linect_exit(void)
{
	LNT_INFO("MS Kinect driver shutdown\n");

	// Deregister this driver with the USB subsystem
	usb_deregister(&usb_linect_driver);
}


module_init(usb_linect_init);
module_exit(usb_linect_exit);


MODULE_PARM_DESC(freemotor, "Driver does not move the motor, but offers interface.");
MODULE_PARM_DESC(freeled, "Driver does not manage the led, but offers interface.");
MODULE_PARM_DESC(startupinit, "Initialize Kinect device on startup.");


MODULE_LICENSE("GPL");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_SUPPORTED_DEVICE(DRIVER_SUPPORT);


