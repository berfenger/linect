/** 
 * @file linect.h
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

#ifndef LINECT_H
#define LINECT_H


#define DRIVER_NAME			"linect"
#define DRIVER_VERSION			"v0.1.2"
#define DRIVER_VERSION_NUM		0x000102
#define DRIVER_DESC			"A MS Kinect driver."
#define DRIVER_AUTHOR			"Arturo Casal <berfenger [at] gmail.com>"
#define DRIVER_V4L_NAME			"MS Kinect"

#define PREFIX				DRIVER_NAME ": "

#define USB_MS_VENDOR_ID		0x045e

#define USB_K1414_PRODUCT_ID		0x02ae
#define USB_K1414_MOTOR_PRODUCT_ID	0x02b0

#define KNT_TYPE_CAM 1
#define KNT_TYPE_MOTOR 2

/* KINECT defines */

#define FREENECT_FRAME_W 640
#define FREENECT_FRAME_H 480
#define FREENECT_FRAME_PIX (FREENECT_FRAME_H*FREENECT_FRAME_W)
#define FREENECT_RGB_SIZE (FREENECT_FRAME_PIX*3)
#define FREENECT_BAYER_SIZE (FREENECT_FRAME_PIX)
#define FREENECT_DEPTH_SIZE (FREENECT_FRAME_PIX*sizeof(freenect_depth))

#define DEPTH_RAW_SIZE 422400
#define FRAME_H FREENECT_FRAME_H
#define FRAME_W FREENECT_FRAME_W
#define FRAME_PIX FREENECT_FRAME_PIX

#define DEPTH_PKTSIZE 1760
#define RGB_PKTSIZE 1920

#define DEPTH_PKTDSIZE (DEPTH_PKTSIZE-12)
#define RGB_PKTDSIZE (RGB_PKTSIZE-12)

#define DEPTH_PKTS_PER_FRAME ((DEPTH_RAW_SIZE+DEPTH_PKTDSIZE-1)/DEPTH_PKTDSIZE)
#define RGB_PKTS_PER_FRAME ((FRAME_PIX+RGB_PKTDSIZE-1)/RGB_PKTDSIZE)

#define PKTS_PER_XFER 16
#define NUM_XFERS 16
#define DEPTH_PKTBUF 1920
#define RGB_PKTBUF 1920

#define ISOC_RGB 1
#define ISOC_DEPTH 2


/* Buffers */

#define MAX_ISO_BUFS				16
#define ISO_FRAMES_PER_DESC			10
#define ISO_MAX_FRAME_SIZE			3 * 1024
#define ISO_BUFFER_SIZE				(ISO_FRAMES_PER_DESC * ISO_MAX_FRAME_SIZE)


/* Image frame buffer */
#define LNT_MAX_IMAGES			10
#define LNT_FRAME_SIZE			(1280 * 1024 * 4)

/* Info print */

#ifndef CONFIG_LINECT_DEBUG
#define CONFIG_LINECT_DEBUG 			1
#endif

#if CONFIG_LINECT_DEBUG

#define LNT_INFO(str, args...)			printk(KERN_INFO PREFIX str, ##args)
#define LNT_ERROR(str, args...)			printk(KERN_ERR PREFIX str, ##args)
#define LNT_WARNING(str, args...)		printk(KERN_WARNING PREFIX str, ##args)
#define LNT_DEBUG(str, args...)			printk(KERN_DEBUG PREFIX str, ##args)

#else

#define LNT_INFO(str, args...)			printk(KERN_INFO PREFIX str, ##args)
#define LNT_ERROR(str, args...)			printk(KERN_ERR PREFIX str, ##args)
#define LNT_WARNING(str, args...)		printk(KERN_WARNING PREFIX str, ##args)
#define LNT_DEBUG(str, args...)			do { } while(0)

#endif

/* KINECT Models */

/**
 * @enum T_SYNTEK_DEVICE Video camera supported by the driver
 */
typedef enum {
	KNT_K1414 = 1
} T_LNT_DEVICE;


/** 
 * @enum T_LNT_RESOLUTION Video resolution
 */
typedef enum {
        LNT_80x60,
        LNT_128x96,
        LNT_160x120,
        LNT_213x160,
        LNT_320x240,
        LNT_640x480,
        LNT_800x600,
        LNT_1024x768,
        LNT_1280x1024,
        LNT_NBR_SIZES
} T_LNT_RESOLUTION;


/**
 * @enum T_LNT_PALETTE Color palette
 */
typedef enum {
	LNT_PALETTE_RGB24 = 1,
	LNT_PALETTE_RGB32 = 2,
	LNT_PALETTE_BGR24 = 3,
	LNT_PALETTE_BGR32 = 4,
	LNT_PALETTE_UYVY = 5,
	LNT_PALETTE_YUYV = 6,
	LNT_PALETTE_DEPTHRAW = 7
} T_LNT_PALETTE;


/**
 * @struct linect_iso_buf
 */
struct linect_iso_buf {
	void *data;
	int length;
	int read;
	struct urb *urb;
};


/**
 * @struct linect_frame_buf
 */
struct linect_frame_buf {
	int errors;
	void *data;
	volatile int filled;
	struct linect_frame_buf *next;
};


/**
 * @struct linect_image_buf
 */
struct linect_image_buf {
	unsigned long offset;				/**< Memory offset */
	int vma_use_count;					/**< VMA counter */
};


/**
 * @struct linect_coord
 */
struct linect_coord {
	int x;								/**< X-coordonate */
	int y;								/**< Y-coordonate */
};


/**
 * @struct linect_video
 */
struct linect_video {
	int brightness;						/**< Brightness setting */
	int depth;							/**< Depth colour setting */
	int palette;						/**< Palette setting */
};

typedef enum {
	LED_OFF    = 0,
	LED_GREEN  = 1,
	LED_RED    = 2,
	LED_YELLOW = 3,
	LED_BLINK_YELLOW = 4,
	LED_BLINK_GREEN = 5,
	LED_BLINK_RED_YELLOW = 6
} linect_led_options;

typedef struct {
	struct usb_linect *dev;
	uint8_t flag;
	int synced;
	uint8_t seq;
	int got_pkts;
	int pkt_num;
	int pkts_per_frame;
	int pkt_size;
	int valid_pkts;
	uint32_t last_timestamp;
	uint32_t timestamp;
} packet_stream;

typedef struct {
	struct usb_linect *dev;
	struct urb **xfers;
	uint8_t *buffer;
	int num_xfers;
	int pkts;
	int len;
	uint8_t type;
} fnusb_isoc_stream;

struct pkt_hdr {
	uint8_t magic[2];
	uint8_t pad;
	uint8_t flag;
	uint8_t unk1;
	uint8_t seq;
	uint8_t unk2;
	uint8_t unk3;
	uint32_t timestamp;
};


struct usb_linect {
	struct linect_cam *cam;
	struct usb_device *motor_udev;
	struct proc_dir_entry *proc_root;
	// Status
	int type;
	int index;
	// Config
	char freemotor;
	char freeled;
	
	// Motor
	char last_led_status;			/* Led last status for control status */
	short last_motor_status;		/* Motor last status for control status */
	struct mutex mutex_motor;		/* Motor USB dev access lock */
};

struct linect_cam {
	struct video_device *vdev;		/* VGA v4l device */
	struct video_device *depth_vdev;	/* Depth v4l device */
	struct usb_device *udev;		/* USB device */
	struct usb_interface *interface; 	/* USB Interface */
	struct mutex mutex_cam;		/* Cam USB dev access lock */

	int release;
	int webcam_model;

	unsigned char *int_in_buffer;
	size_t int_in_size;
	__u8 int_in_endpointAddr;

	size_t isoc_in_size;
	__u8 isoc_in_endpointAddr;

	int watchdog;

	struct linect_video vsettings;
	struct linect_video depth_vsettings;
	
	int error_status;

	int vopen_rgb;				/* VGA V4L opened device */
	int vopen_depth;			/* Depth V4L opened device */
	int visoc_errors;
	int vframes_error;
	int vframes_dumped;


	spinlock_t spinlock_rgb;
	spinlock_t spinlock_depth;
	struct semaphore mutex_rgb;
	struct semaphore mutex_depth;
	wait_queue_head_t wait_rgb_frame;
	wait_queue_head_t wait_depth_frame;
	struct mutex modlock_rgb;
	struct mutex modlock_depth;


	// 1: isoc
	char rgb_isoc_init_ok;
	char depth_isoc_init_ok;
	fnusb_isoc_stream depth_isoc;
	fnusb_isoc_stream rgb_isoc;
	int cam_tag;
	
	// 2: streams
	packet_stream depth_stream;
	packet_stream rgb_stream;

	// 3: frame rgb
	int frame_size;
	struct linect_frame_buf *framebuf;
	struct linect_frame_buf *empty_frames, *empty_frames_tail;
	struct linect_frame_buf *full_frames, *full_frames_tail;
	struct linect_frame_buf *fill_frame;
	struct linect_frame_buf *read_frame;
	
	// 3: frame depth
	int frame_size_depth;
	struct linect_frame_buf *framebuf_depth;
	struct linect_frame_buf *empty_frames_depth, *empty_frames_tail_depth;
	struct linect_frame_buf *full_frames_depth, *full_frames_tail_depth;
	struct linect_frame_buf *fill_frame_depth;
	struct linect_frame_buf *read_frame_depth;

	// 4: image rgb
	int view_size;
	int image_size;
	void *image_data;
	struct linect_image_buf images[LNT_MAX_IMAGES];
	int image_used[LNT_MAX_IMAGES];
	unsigned int nbuffers;
	unsigned int len_per_image;
	int image_read_pos;
	int fill_image;
	struct linect_coord view;
	struct linect_coord image;
	
	// 4: image depth
	int view_size_depth;
	int image_size_depth;
	void *image_data_depth;
	struct linect_image_buf images_depth[LNT_MAX_IMAGES];
	int image_used_depth[LNT_MAX_IMAGES];
	unsigned int nbuffers_depth;
	unsigned int len_per_image_depth;
	int image_read_pos_depth;
	int fill_image_depth;
	int resolution_depth;
	struct linect_coord view_depth;
	struct linect_coord image_depth;
	uint8_t *image_tmp;
	
	// Options
	char startupinit;
};


/**
 * @def LNT_PERCENT
 *   Calculate a value from a percent
 */
#define LNT_PERCENT(x,y) ( ((int)x * (int)y) / 100)


/**
 * @def to_linect_dev(d)
 * Cast a member of a structure out to the containing structure
 */
#define to_linect_dev(d) container_of(d, struct usb_linect, kref)


//extern const struct linect_coord linect_image_sizes[LNT_NBR_SIZES];

	
int usb_linect_write_registry(struct usb_linect *, __u16, __u16);
int usb_linect_read_registry(struct usb_linect *, __u16, int *);
int usb_linect_set_feature(struct usb_linect *, int);
int usb_linect_set_configuration(struct usb_linect *);
int usb_linect_rgb_isoc_init(struct usb_linect *);
int usb_linect_depth_isoc_init(struct usb_linect *);
void usb_linect_isoc_handler(struct urb *);
void usb_linect_rgb_isoc_cleanup(struct usb_linect *);
void usb_linect_depth_isoc_cleanup(struct usb_linect *);

int v4l_linect_select_video_mode(struct usb_linect *, int, int);
int v4l_linect_register_rgb_video_device(struct usb_linect *);
int v4l_linect_unregister_rgb_video_device(struct usb_linect *);
int v4l_linect_register_depth_video_device(struct usb_linect *);
int v4l_linect_unregister_depth_video_device(struct usb_linect *);

int linect_allocate_rgb_buffers(struct usb_linect *);
int linect_reset_rgb_buffers(struct usb_linect *);
int linect_clear_rgb_buffers(struct usb_linect *);
int linect_free_rgb_buffers(struct usb_linect *);
void linect_next_rgb_image(struct usb_linect *);
int linect_next_rgb_frame(struct usb_linect *);
int linect_handle_rgb_frame(struct usb_linect *);

int linect_allocate_depth_buffers(struct usb_linect *);
int linect_reset_depth_buffers(struct usb_linect *);
int linect_clear_depth_buffers(struct usb_linect *);
int linect_free_depth_buffers(struct usb_linect *);
void linect_next_depth_image(struct usb_linect *);
int linect_next_depth_frame(struct usb_linect *);
int linect_handle_depth_frame(struct usb_linect *);

int linect_rgb_decompress(struct usb_linect *);
int linect_depth_decompress(struct usb_linect *);

void * linect_rvmalloc(unsigned long size);
void linect_rvfree(void *mem, unsigned long size);

// Proc
void linect_proc_create(struct usb_linect *dev);
void linect_proc_destroy(struct usb_linect *dev);

// Cam
int linect_cam_init(struct usb_linect *dev);
int linect_cam_start_rgb(struct usb_linect *dev);
int linect_cam_stop_rgb(struct usb_linect *dev);
int linect_cam_start_depth(struct usb_linect *dev);
int linect_cam_stop_depth(struct usb_linect *dev);

// Motor
int linect_motor_set_led(struct usb_linect *dev, int led);
int linect_motor_set_tilt_degs(struct usb_linect *dev, int angle);
int linect_get_raw_accel(struct usb_linect *dev, int16_t* x, int16_t* y, int16_t* z);

#endif 
