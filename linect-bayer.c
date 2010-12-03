/** 
 * @file linect-bayer.c
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

#include "linect.h"


#define MAX(a,b)	((a)>(b)?(a):(b))
#define MIN(a,b)	((a)<(b)?(a):(b))
#define CLIP(a,low,high) MAX((low),MIN((high),(a)))


void linect_b2rgb24(uint8_t *, uint8_t *);
void linect_b2rgb32(uint8_t *, uint8_t *);
void linect_b2bgr24(uint8_t *, uint8_t *);
void linect_b2bgr32(uint8_t *, uint8_t *);

void linect_b2uyvy(uint8_t *, uint8_t *);
void linect_b2yuyv(uint8_t *, uint8_t *);

void linect_depth2rgb24(uint16_t *, uint8_t *);
void linect_depth2raw(uint16_t *, uint8_t *);


void linect_correct_brightness(uint8_t *, const int, const int,
		const int, int, int);


static uint16_t t_gamma[2048] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x4, 0x4, 0x4, 0x4, 0x4, 0x4, 0x4, 0x4, 0x4, 0x4, 0x4, 0x4, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x6, 0x6, 0x6, 0x6, 0x6, 0x6, 0x6, 0x6, 0x6, 0x7, 0x7, 0x7, 0x7, 0x7, 0x7, 0x7, 0x7, 0x7, 0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 0x9, 0x9, 0x9, 0x9, 0x9, 0x9, 0x9, 0xa, 0xa, 0xa, 0xa, 0xa, 0xa, 0xa, 0xb, 0xb, 0xb, 0xb, 0xb, 0xb, 0xc, 0xc, 0xc, 0xc, 0xc, 0xc, 0xd, 0xd, 0xd, 0xd, 0xd, 0xd, 0xe, 0xe, 0xe, 0xe, 0xe, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0x10, 0x10, 0x10, 0x10, 0x10, 0x11, 0x11, 0x11, 0x11, 0x12, 0x12, 0x12, 0x12, 0x12, 0x13, 0x13, 0x13, 0x13, 0x13, 0x14, 0x14, 0x14, 0x14, 0x15, 0x15, 0x15, 0x15, 0x16, 0x16, 0x16, 0x16, 0x17, 0x17, 0x17, 0x17, 0x18, 0x18, 0x18, 0x18, 0x19, 0x19, 0x19, 0x19, 0x1a, 0x1a, 0x1a, 0x1a, 0x1b, 0x1b, 0x1b, 0x1c, 0x1c, 0x1c, 0x1c, 0x1d, 0x1d, 0x1d, 0x1e, 0x1e, 0x1e, 0x1f, 0x1f, 0x1f, 0x1f, 0x20, 0x20, 0x20, 0x21, 0x21, 0x21, 0x22, 0x22, 0x22, 0x23, 0x23, 0x23, 0x24, 0x24, 0x24, 0x25, 0x25, 0x25, 0x26, 0x26, 0x26, 0x27, 0x27, 0x27, 0x28, 0x28, 0x29, 0x29, 0x29, 0x2a, 0x2a, 0x2a, 0x2b, 0x2b, 0x2c, 0x2c, 0x2c, 0x2d, 0x2d, 0x2d, 0x2e, 0x2e, 0x2f, 0x2f, 0x2f, 0x30, 0x30, 0x31, 0x31, 0x32, 0x32, 0x32, 0x33, 0x33, 0x34, 0x34, 0x35, 0x35, 0x35, 0x36, 0x36, 0x37, 0x37, 0x38, 0x38, 0x39, 0x39, 0x39, 0x3a, 0x3a, 0x3b, 0x3b, 0x3c, 0x3c, 0x3d, 0x3d, 0x3e, 0x3e, 0x3f, 0x3f, 0x40, 0x40, 0x41, 0x41, 0x42, 0x42, 0x43, 0x43, 0x44, 0x44, 0x45, 0x45, 0x46, 0x46, 0x47, 0x47, 0x48, 0x48, 0x49, 0x49, 0x4a, 0x4b, 0x4b, 0x4c, 0x4c, 0x4d, 0x4d, 0x4e, 0x4e, 0x4f, 0x50, 0x50, 0x51, 0x51, 0x52, 0x52, 0x53, 0x54, 0x54, 0x55, 0x55, 0x56, 0x57, 0x57, 0x58, 0x58, 0x59, 0x5a, 0x5a, 0x5b, 0x5c, 0x5c, 0x5d, 0x5d, 0x5e, 0x5f, 0x5f, 0x60, 0x61, 0x61, 0x62, 0x63, 0x63, 0x64, 0x65, 0x65, 0x66, 0x67, 0x67, 0x68, 0x69, 0x69, 0x6a, 0x6b, 0x6b, 0x6c, 0x6d, 0x6d, 0x6e, 0x6f, 0x70, 0x70, 0x71, 0x72, 0x72, 0x73, 0x74, 0x75, 0x75, 0x76, 0x77, 0x78, 0x78, 0x79, 0x7a, 0x7b, 0x7b, 0x7c, 0x7d, 0x7e, 0x7e, 0x7f, 0x80, 0x81, 0x82, 0x82, 0x83, 0x84, 0x85, 0x86, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0x9b, 0x9c, 0x9d, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf, 0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xcb, 0xcd, 0xce, 0xcf, 0xd0, 0xd1, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde, 0xdf, 0xe0, 0xe1, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xea, 0xeb, 0xec, 0xed, 0xee, 0xef, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf7, 0xf8, 0xf9, 0xfa, 0xfc, 0xfd, 0xfe, 0xff, 0x100, 0x102, 0x103, 0x104, 0x105, 0x107, 0x108, 0x109, 0x10a, 0x10c, 0x10d, 0x10e, 0x110, 0x111, 0x112, 0x114, 0x115, 0x116, 0x117, 0x119, 0x11a, 0x11b, 0x11d, 0x11e, 0x11f, 0x121, 0x122, 0x123, 0x125, 0x126, 0x128, 0x129, 0x12a, 0x12c, 0x12d, 0x12e, 0x130, 0x131, 0x133, 0x134, 0x135, 0x137, 0x138, 0x13a, 0x13b, 0x13c, 0x13e, 0x13f, 0x141, 0x142, 0x144, 0x145, 0x147, 0x148, 0x149, 0x14b, 0x14c, 0x14e, 0x14f, 0x151, 0x152, 0x154, 0x155, 0x157, 0x158, 0x15a, 0x15b, 0x15d, 0x15e, 0x160, 0x161, 0x163, 0x165, 0x166, 0x168, 0x169, 0x16b, 0x16c, 0x16e, 0x16f, 0x171, 0x173, 0x174, 0x176, 0x177, 0x179, 0x17b, 0x17c, 0x17e, 0x17f, 0x181, 0x183, 0x184, 0x186, 0x188, 0x189, 0x18b, 0x18d, 0x18e, 0x190, 0x192, 0x193, 0x195, 0x197, 0x198, 0x19a, 0x19c, 0x19d, 0x19f, 0x1a1, 0x1a3, 0x1a4, 0x1a6, 0x1a8, 0x1aa, 0x1ab, 0x1ad, 0x1af, 0x1b0, 0x1b2, 0x1b4, 0x1b6, 0x1b8, 0x1b9, 0x1bb, 0x1bd, 0x1bf, 0x1c1, 0x1c2, 0x1c4, 0x1c6, 0x1c8, 0x1ca, 0x1cb, 0x1cd, 0x1cf, 0x1d1, 0x1d3, 0x1d5, 0x1d6, 0x1d8, 0x1da, 0x1dc, 0x1de, 0x1e0, 0x1e2, 0x1e4, 0x1e6, 0x1e7, 0x1e9, 0x1eb, 0x1ed, 0x1ef, 0x1f1, 0x1f3, 0x1f5, 0x1f7, 0x1f9, 0x1fb, 0x1fd, 0x1ff, 0x201, 0x203, 0x205, 0x206, 0x208, 0x20a, 0x20c, 0x20e, 0x210, 0x212, 0x215, 0x217, 0x219, 0x21b, 0x21d, 0x21f, 0x221, 0x223, 0x225, 0x227, 0x229, 0x22b, 0x22d, 0x22f, 0x231, 0x233, 0x235, 0x238, 0x23a, 0x23c, 0x23e, 0x240, 0x242, 0x244, 0x246, 0x249, 0x24b, 0x24d, 0x24f, 0x251, 0x253, 0x256, 0x258, 0x25a, 0x25c, 0x25e, 0x261, 0x263, 0x265, 0x267, 0x269, 0x26c, 0x26e, 0x270, 0x272, 0x275, 0x277, 0x279, 0x27b, 0x27e, 0x280, 0x282, 0x285, 0x287, 0x289, 0x28b, 0x28e, 0x290, 0x292, 0x295, 0x297, 0x299, 0x29c, 0x29e, 0x2a0, 0x2a3, 0x2a5, 0x2a8, 0x2aa, 0x2ac, 0x2af, 0x2b1, 0x2b3, 0x2b6, 0x2b8, 0x2bb, 0x2bd, 0x2c0, 0x2c2, 0x2c4, 0x2c7, 0x2c9, 0x2cc, 0x2ce, 0x2d1, 0x2d3, 0x2d6, 0x2d8, 0x2db, 0x2dd, 0x2e0, 0x2e2, 0x2e5, 0x2e7, 0x2ea, 0x2ec, 0x2ef, 0x2f1, 0x2f4, 0x2f6, 0x2f9, 0x2fc, 0x2fe, 0x301, 0x303, 0x306, 0x308, 0x30b, 0x30e, 0x310, 0x313, 0x315, 0x318, 0x31b, 0x31d, 0x320, 0x323, 0x325, 0x328, 0x32b, 0x32d, 0x330, 0x333, 0x335, 0x338, 0x33b, 0x33e, 0x340, 0x343, 0x346, 0x348, 0x34b, 0x34e, 0x351, 0x353, 0x356, 0x359, 0x35c, 0x35e, 0x361, 0x364, 0x367, 0x36a, 0x36c, 0x36f, 0x372, 0x375, 0x378, 0x37b, 0x37d, 0x380, 0x383, 0x386, 0x389, 0x38c, 0x38f, 0x392, 0x394, 0x397, 0x39a, 0x39d, 0x3a0, 0x3a3, 0x3a6, 0x3a9, 0x3ac, 0x3af, 0x3b2, 0x3b5, 0x3b8, 0x3bb, 0x3be, 0x3c1, 0x3c4, 0x3c7, 0x3ca, 0x3cd, 0x3d0, 0x3d3, 0x3d6, 0x3d9, 0x3dc, 0x3df, 0x3e2, 0x3e5, 0x3e8, 0x3eb, 0x3ee, 0x3f1, 0x3f4, 0x3f7, 0x3fb, 0x3fe, 0x401, 0x404, 0x407, 0x40a, 0x40d, 0x411, 0x414, 0x417, 0x41a, 0x41d, 0x420, 0x424, 0x427, 0x42a, 0x42d, 0x430, 0x434, 0x437, 0x43a, 0x43d, 0x441, 0x444, 0x447, 0x44a, 0x44e, 0x451, 0x454, 0x457, 0x45b, 0x45e, 0x461, 0x465, 0x468, 0x46b, 0x46f, 0x472, 0x475, 0x479, 0x47c, 0x480, 0x483, 0x486, 0x48a, 0x48d, 0x490, 0x494, 0x497, 0x49b, 0x49e, 0x4a2, 0x4a5, 0x4a8, 0x4ac, 0x4af, 0x4b3, 0x4b6, 0x4ba, 0x4bd, 0x4c1, 0x4c4, 0x4c8, 0x4cb, 0x4cf, 0x4d2, 0x4d6, 0x4d9, 0x4dd, 0x4e1, 0x4e4, 0x4e8, 0x4eb, 0x4ef, 0x4f3, 0x4f6, 0x4fa, 0x4fd, 0x501, 0x505, 0x508, 0x50c, 0x50f, 0x513, 0x517, 0x51a, 0x51e, 0x522, 0x526, 0x529, 0x52d, 0x531, 0x534, 0x538, 0x53c, 0x540, 0x543, 0x547, 0x54b, 0x54f, 0x552, 0x556, 0x55a, 0x55e, 0x561, 0x565, 0x569, 0x56d, 0x571, 0x575, 0x578, 0x57c, 0x580, 0x584, 0x588, 0x58c, 0x590, 0x594, 0x597, 0x59b, 0x59f, 0x5a3, 0x5a7, 0x5ab, 0x5af, 0x5b3, 0x5b7, 0x5bb, 0x5bf, 0x5c3, 0x5c7, 0x5cb, 0x5cf, 0x5d3, 0x5d7, 0x5db, 0x5df, 0x5e3, 0x5e7, 0x5eb, 0x5ef, 0x5f3, 0x5f7, 0x5fb, 0x5ff, 0x603, 0x607, 0x60c, 0x610, 0x614, 0x618, 0x61c, 0x620, 0x624, 0x629, 0x62d, 0x631, 0x635, 0x639, 0x63d, 0x642, 0x646, 0x64a, 0x64e, 0x652, 0x657, 0x65b, 0x65f, 0x663, 0x668, 0x66c, 0x670, 0x675, 0x679, 0x67d, 0x682, 0x686, 0x68a, 0x68e, 0x693, 0x697, 0x69c, 0x6a0, 0x6a4, 0x6a9, 0x6ad, 0x6b1, 0x6b6, 0x6ba, 0x6bf, 0x6c3, 0x6c8, 0x6cc, 0x6d0, 0x6d5, 0x6d9, 0x6de, 0x6e2, 0x6e7, 0x6eb, 0x6f0, 0x6f4, 0x6f9, 0x6fd, 0x702, 0x706, 0x70b, 0x70f, 0x714, 0x719, 0x71d, 0x722, 0x726, 0x72b, 0x730, 0x734, 0x739, 0x73d, 0x742, 0x747, 0x74b, 0x750, 0x755, 0x759, 0x75e, 0x763, 0x767, 0x76c, 0x771, 0x776, 0x77a, 0x77f, 0x784, 0x789, 0x78d, 0x792, 0x797, 0x79c, 0x7a0, 0x7a5, 0x7aa, 0x7af, 0x7b4, 0x7b9, 0x7bd, 0x7c2, 0x7c7, 0x7cc, 0x7d1, 0x7d6, 0x7db, 0x7e0, 0x7e4, 0x7e9, 0x7ee, 0x7f3, 0x7f8, 0x7fd, 0x802, 0x807, 0x80c, 0x811, 0x816, 0x81b, 0x820, 0x825, 0x82a, 0x82f, 0x834, 0x839, 0x83e, 0x843, 0x848, 0x84d, 0x852, 0x857, 0x85d, 0x862, 0x867, 0x86c, 0x871, 0x876, 0x87b, 0x880, 0x886, 0x88b, 0x890, 0x895, 0x89a, 0x8a0, 0x8a5, 0x8aa, 0x8af, 0x8b4, 0x8ba, 0x8bf, 0x8c4, 0x8ca, 0x8cf, 0x8d4, 0x8d9, 0x8df, 0x8e4, 0x8e9, 0x8ef, 0x8f4, 0x8f9, 0x8ff, 0x904, 0x909, 0x90f, 0x914, 0x91a, 0x91f, 0x924, 0x92a, 0x92f, 0x935, 0x93a, 0x940, 0x945, 0x94a, 0x950, 0x955, 0x95b, 0x960, 0x966, 0x96b, 0x971, 0x977, 0x97c, 0x982, 0x987, 0x98d, 0x992, 0x998, 0x99d, 0x9a3, 0x9a9, 0x9ae, 0x9b4, 0x9ba, 0x9bf, 0x9c5, 0x9cb, 0x9d0, 0x9d6, 0x9dc, 0x9e1, 0x9e7, 0x9ed, 0x9f2, 0x9f8, 0x9fe, 0xa04, 0xa09, 0xa0f, 0xa15, 0xa1b, 0xa21, 0xa26, 0xa2c, 0xa32, 0xa38, 0xa3e, 0xa43, 0xa49, 0xa4f, 0xa55, 0xa5b, 0xa61, 0xa67, 0xa6d, 0xa73, 0xa78, 0xa7e, 0xa84, 0xa8a, 0xa90, 0xa96, 0xa9c, 0xaa2, 0xaa8, 0xaae, 0xab4, 0xaba, 0xac0, 0xac6, 0xacc, 0xad2, 0xad8, 0xade, 0xae5, 0xaeb, 0xaf1, 0xaf7, 0xafd, 0xb03, 0xb09, 0xb0f, 0xb16, 0xb1c, 0xb22, 0xb28, 0xb2e, 0xb34, 0xb3b, 0xb41, 0xb47, 0xb4d, 0xb54, 0xb5a, 0xb60, 0xb66, 0xb6d, 0xb73, 0xb79, 0xb7f, 0xb86, 0xb8c, 0xb92, 0xb99, 0xb9f, 0xba6, 0xbac, 0xbb2, 0xbb9, 0xbbf, 0xbc5, 0xbcc, 0xbd2, 0xbd9, 0xbdf, 0xbe6, 0xbec, 0xbf3, 0xbf9, 0xbff, 0xc06, 0xc0c, 0xc13, 0xc1a, 0xc20, 0xc27, 0xc2d, 0xc34, 0xc3a, 0xc41, 0xc47, 0xc4e, 0xc55, 0xc5b, 0xc62, 0xc68, 0xc6f, 0xc76, 0xc7c, 0xc83, 0xc8a, 0xc90, 0xc97, 0xc9e, 0xca5, 0xcab, 0xcb2, 0xcb9, 0xcc0, 0xcc6, 0xccd, 0xcd4, 0xcdb, 0xce1, 0xce8, 0xcef, 0xcf6, 0xcfd, 0xd04, 0xd0a, 0xd11, 0xd18, 0xd1f, 0xd26, 0xd2d, 0xd34, 0xd3b, 0xd42, 0xd49, 0xd50, 0xd56, 0xd5d, 0xd64, 0xd6b, 0xd72, 0xd79, 0xd80, 0xd87, 0xd8f, 0xd96, 0xd9d, 0xda4, 0xdab, 0xdb2, 0xdb9, 0xdc0, 0xdc7, 0xdce, 0xdd5, 0xddd, 0xde4, 0xdeb, 0xdf2, 0xdf9, 0xe00, 0xe08, 0xe0f, 0xe16, 0xe1d, 0xe24, 0xe2c, 0xe33, 0xe3a, 0xe42, 0xe49, 0xe50, 0xe57, 0xe5f, 0xe66, 0xe6d, 0xe75, 0xe7c, 0xe83, 0xe8b, 0xe92, 0xe9a, 0xea1, 0xea8, 0xeb0, 0xeb7, 0xebf, 0xec6, 0xece, 0xed5, 0xedd, 0xee4, 0xeec, 0xef3, 0xefb, 0xf02, 0xf0a, 0xf11, 0xf19, 0xf20, 0xf28, 0xf30, 0xf37, 0xf3f, 0xf46, 0xf4e, 0xf56, 0xf5d, 0xf65, 0xf6d, 0xf74, 0xf7c, 0xf84, 0xf8b, 0xf93, 0xf9b, 0xfa3, 0xfaa, 0xfb2, 0xfba, 0xfc2, 0xfc9, 0xfd1, 0xfd9, 0xfe1, 0xfe9, 0xff0, 0xff8, 0x1000, 0x1008, 0x1010, 0x1018, 0x1020, 0x1028, 0x1030, 0x1037, 0x103f, 0x1047, 0x104f, 0x1057, 0x105f, 0x1067, 0x106f, 0x1077, 0x107f, 0x1087, 0x108f, 0x1097, 0x109f, 0x10a8, 0x10b0, 0x10b8, 0x10c0, 0x10c8, 0x10d0, 0x10d8, 0x10e0, 0x10e8, 0x10f1, 0x10f9, 0x1101, 0x1109, 0x1111, 0x111a, 0x1122, 0x112a, 0x1132, 0x113b, 0x1143, 0x114b, 0x1153, 0x115c, 0x1164, 0x116c, 0x1175, 0x117d, 0x1185, 0x118e, 0x1196, 0x119e, 0x11a7, 0x11af, 0x11b8, 0x11c0, 0x11c8, 0x11d1, 0x11d9, 0x11e2, 0x11ea, 0x11f3, 0x11fb, 0x1204, 0x120c, 0x1215, 0x121d, 0x1226, 0x122e, 0x1237, 0x1240, 0x1248, 0x1251, 0x1259, 0x1262, 0x126b, 0x1273, 0x127c, 0x1285, 0x128d, 0x1296, 0x129f, 0x12a7, 0x12b0, 0x12b9, 0x12c2, 0x12ca, 0x12d3, 0x12dc, 0x12e5, 0x12ed, 0x12f6, 0x12ff, 0x1308, 0x1311, 0x1319, 0x1322, 0x132b, 0x1334, 0x133d, 0x1346, 0x134f, 0x1358, 0x1361, 0x136a, 0x1372, 0x137b, 0x1384, 0x138d, 0x1396, 0x139f, 0x13a8, 0x13b1, 0x13ba, 0x13c4, 0x13cd, 0x13d6, 0x13df, 0x13e8, 0x13f1, 0x13fa, 0x1403, 0x140c, 0x1415, 0x141f, 0x1428, 0x1431, 0x143a, 0x1443, 0x144d, 0x1456, 0x145f, 0x1468, 0x1471, 0x147b, 0x1484, 0x148d, 0x1497, 0x14a0, 0x14a9, 0x14b3, 0x14bc, 0x14c5, 0x14cf, 0x14d8, 0x14e1, 0x14eb, 0x14f4, 0x14fe, 0x1507, 0x1510, 0x151a, 0x1523, 0x152d, 0x1536, 0x1540, 0x1549, 0x1553, 0x155c, 0x1566, 0x156f, 0x1579, 0x1583, 0x158c, 0x1596, 0x159f, 0x15a9, 0x15b3, 0x15bc, 0x15c6, 0x15d0, 0x15d9, 0x15e3, 0x15ed, 0x15f6, 0x1600, 0x160a, 0x1613, 0x161d, 0x1627, 0x1631, 0x163b, 0x1644, 0x164e, 0x1658, 0x1662, 0x166c, 0x1675, 0x167f, 0x1689, 0x1693, 0x169d, 0x16a7, 0x16b1, 0x16bb, 0x16c5, 0x16cf, 0x16d9, 0x16e3, 0x16ed, 0x16f7, 0x1701, 0x170b, 0x1715, 0x171f, 0x1729, 0x1733, 0x173d, 0x1747, 0x1751, 0x175b, 0x1765, 0x176f, 0x177a, 0x1784, 0x178e, 0x1798, 0x17a2, 0x17ad, 0x17b7, 0x17c1, 0x17cb, 0x17d5, 0x17e0, 0x17ea, 0x17f4, 0x17ff, 0x1809, 0x1813, 0x181e, 0x1828, 0x1832, 0x183d, 0x1847, 0x1851, 0x185c, 0x1866, 0x1871, 0x187b, 0x1885, 0x1890, 0x189a, 0x18a5, 0x18af, 0x18ba, 0x18c4, 0x18cf, 0x18d9, 0x18e4, 0x18ef, 0x18f9, 0x1904, 0x190e, 0x1919, 0x1924, 0x192e, 0x1939, 0x1943, 0x194e, 0x1959, 0x1963, 0x196e, 0x1979, 0x1984, 0x198e, 0x1999, 0x19a4, 0x19af, 0x19b9, 0x19c4, 0x19cf, 0x19da, 0x19e5, 0x19f0, 0x19fa, 0x1a05, 0x1a10, 0x1a1b, 0x1a26, 0x1a31, 0x1a3c, 0x1a47, 0x1a52, 0x1a5d, 0x1a68, 0x1a73, 0x1a7e, 0x1a89, 0x1a94, 0x1a9f, 0x1aaa, 0x1ab5, 0x1ac0, 0x1acb, 0x1ad6, 0x1ae1, 0x1aec, 0x1af7, 0x1b02, 0x1b0e, 0x1b19, 0x1b24, 0x1b2f, 0x1b3a, 0x1b46, 0x1b51, 0x1b5c, 0x1b67, 0x1b73, 0x1b7e, 0x1b89, 0x1b94, 0x1ba0, 0x1bab, 0x1bb6, 0x1bc2, 0x1bcd, 0x1bd8, 0x1be4, 0x1bef, 0x1bfb, 0x1c06, 0x1c11, 0x1c1d, 0x1c28, 0x1c34, 0x1c3f, 0x1c4b, 0x1c56, 0x1c62, 0x1c6d, 0x1c79, 0x1c84, 0x1c90, 0x1c9c, 0x1ca7, 0x1cb3, 0x1cbe, 0x1cca, 0x1cd6, 0x1ce1, 0x1ced, 0x1cf9, 0x1d04, 0x1d10, 0x1d1c, 0x1d27, 0x1d33, 0x1d3f, 0x1d4b, 0x1d56, 0x1d62, 0x1d6e, 0x1d7a, 0x1d86, 0x1d92, 0x1d9d, 0x1da9, 0x1db5, 0x1dc1, 0x1dcd, 0x1dd9, 0x1de5, 0x1df1, 0x1dfd, 0x1e09, 0x1e15, 0x1e21, 0x1e2d, 0x1e39, 0x1e45, 0x1e51, 0x1e5d, 0x1e69, 0x1e75, 0x1e81, 0x1e8d, 0x1e99, 0x1ea5, 0x1eb1, 0x1ebd, 0x1eca, 0x1ed6, 0x1ee2, 0x1eee, 0x1efa, 0x1f07, 0x1f13, 0x1f1f, 0x1f2b, 0x1f38, 0x1f44, 0x1f50, 0x1f5c, 0x1f69, 0x1f75, 0x1f81, 0x1f8e, 0x1f9a, 0x1fa7, 0x1fb3, 0x1fbf, 0x1fcc, 0x1fd8, 0x1fe5, 0x1ff1, 0x1ffe, 0x200a, 0x2017, 0x2023, 0x2030, 0x203c, 0x2049, 0x2055, 0x2062, 0x206e, 0x207b, 0x2088, 0x2094, 0x20a1, 0x20ae, 0x20ba, 0x20c7, 0x20d4, 0x20e0, 0x20ed, 0x20fa, 0x2106, 0x2113, 0x2120, 0x212d, 0x213a, 0x2146, 0x2153, 0x2160, 0x216d, 0x217a, 0x2187, 0x2193, 0x21a0, 0x21ad, 0x21ba, 0x21c7, 0x21d4, 0x21e1, 0x21ee, 0x21fb, 0x2208, 0x2215, 0x2222, 0x222f, 0x223c, 0x2249, 0x2256, 0x2263, 0x2270, 0x227e, 0x228b, 0x2298, 0x22a5, 0x22b2, 0x22bf, 0x22cc, 0x22da, 0x22e7, 0x22f4, 0x2301, 0x230f, 0x231c, 0x2329, 0x2336, 0x2344, 0x2351, 0x235e, 0x236c, 0x2379, 0x2387, 0x2394, 0x23a1, 0x23af, 0x23bc, 0x23ca, 0x23d7, 0x23e5, 0x23f2};

static signed short linect_yuv_interp[256][8] = {
	{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0},{0,1,0,0,0,1,0,0},{0,1,0,0,0,1,-1,0},
	{1,2,0,0,-1,2,-1,0},{1,2,0,0,-1,2,-2,0},{1,3,0,-1,-1,3,-2,0},{2,3,0,-1,-2,3,-2,0},
	{2,4,0,-1,-2,4,-3,0},{2,5,1,-1,-2,4,-3,0},{2,5,1,-1,-3,5,-4,0},{3,6,1,-1,-3,5,-4,0},
	{3,6,1,-2,-3,6,-5,0},{3,7,1,-2,-4,6,-5,-1},{4,7,1,-2,-4,7,-5,-1},{4,8,1,-2,-4,7,-6,-1},
	{4,9,1,-2,-5,8,-6,-1},{5,9,1,-2,-5,8,-7,-1},{5,10,2,-3,-5,9,-7,-1},{5,10,2,-3,-6,9,-7,-1},
	{5,11,2,-3,-6,10,-8,-1},{6,11,2,-3,-6,10,-8,-1},{6,12,2,-3,-7,11,-9,-1},{6,13,2,-3,-7,11,-9,-1},
	{7,13,2,-4,-7,12,-10,-1},{7,14,2,-4,-8,12,-10,-2},{7,14,2,-4,-8,13,-10,-2},{8,15,3,-4,-8,13,-11,-2},
	{8,15,3,-4,-9,14,-11,-2},{8,16,3,-4,-9,14,-12,-2},{8,17,3,-5,-9,15,-12,-2},{9,17,3,-5,-10,15,-12,-2},
	{9,18,3,-5,-10,16,-13,-2},{9,18,3,-5,-10,16,-13,-2},{10,19,3,-5,-11,17,-14,-2},{10,19,3,-5,-11,17,-14,-2},
	{10,20,4,-6,-11,18,-15,-2},{11,20,4,-6,-12,18,-15,-3},{11,21,4,-6,-12,19,-15,-3},{11,22,4,-6,-12,19,-16,-3},
	{11,22,4,-6,-13,20,-16,-3},{12,23,4,-6,-13,20,-17,-3},{12,23,4,-7,-13,21,-17,-3},{12,24,4,-7,-14,21,-18,-3},
	{13,24,5,-7,-14,22,-18,-3},{13,25,5,-7,-14,22,-18,-3},{13,26,5,-7,-15,23,-19,-3},{14,26,5,-7,-15,23,-19,-3},
	{14,27,5,-8,-15,24,-20,-3},{14,27,5,-8,-16,24,-20,-3},{14,28,5,-8,-16,25,-20,-4},{15,28,5,-8,-16,25,-21,-4},
	{15,29,5,-8,-17,26,-21,-4},{15,30,6,-8,-17,26,-22,-4},{16,30,6,-9,-17,27,-22,-4},{16,31,6,-9,-18,27,-23,-4},
	{16,31,6,-9,-18,28,-23,-4},{17,32,6,-9,-18,28,-23,-4},{17,32,6,-9,-19,29,-24,-4},{17,33,6,-9,-19,29,-24,-4},
	{17,34,6,-10,-19,30,-25,-4},{18,34,6,-10,-20,30,-25,-4},{18,35,7,-10,-20,31,-25,-5},{18,35,7,-10,-20,31,-26,-5},
	{19,36,7,-10,-21,32,-26,-5},{19,36,7,-10,-21,32,-27,-5},{19,37,7,-11,-21,33,-27,-5},{20,37,7,-11,-22,33,-28,-5},
	{20,38,7,-11,-22,34,-28,-5},{20,39,7,-11,-22,34,-28,-5},{20,39,7,-11,-23,35,-29,-5},{21,40,8,-11,-23,35,-29,-5},
	{21,40,8,-12,-23,36,-30,-5},{21,41,8,-12,-24,36,-30,-5},{22,41,8,-12,-24,37,-30,-6},{22,42,8,-12,-24,37,-31,-6},
	{22,43,8,-12,-25,38,-31,-6},{23,43,8,-12,-25,38,-32,-6},{23,44,8,-13,-25,39,-32,-6},{23,44,9,-13,-26,39,-33,-6},
	{23,45,9,-13,-26,40,-33,-6},{24,45,9,-13,-26,40,-33,-6},{24,46,9,-13,-27,41,-34,-6},{24,47,9,-14,-27,41,-34,-6},
	{25,47,9,-14,-27,42,-35,-6},{25,48,9,-14,-28,42,-35,-6},{25,48,9,-14,-28,43,-36,-6},{26,49,9,-14,-28,43,-36,-7},
	{26,49,10,-14,-29,44,-36,-7},{26,50,10,-15,-29,44,-37,-7},{26,51,10,-15,-29,45,-37,-7},{27,51,10,-15,-30,45,-38,-7},
	{27,52,10,-15,-30,46,-38,-7},{27,52,10,-15,-30,46,-38,-7},{28,53,10,-15,-31,47,-39,-7},{28,53,10,-16,-31,47,-39,-7},
	{28,54,10,-16,-31,48,-40,-7},{29,54,11,-16,-32,48,-40,-7},{29,55,11,-16,-32,49,-41,-7},{29,56,11,-16,-32,49,-41,-8},
	{29,56,11,-16,-33,50,-41,-8},{30,57,11,-17,-33,50,-42,-8},{30,57,11,-17,-33,51,-42,-8},{30,58,11,-17,-34,51,-43,-8},
	{31,58,11,-17,-34,52,-43,-8},{31,59,11,-17,-34,52,-43,-8},{31,60,12,-17,-35,53,-44,-8},{31,60,12,-18,-35,53,-44,-8},
	{32,61,12,-18,-35,54,-45,-8},{32,61,12,-18,-36,54,-45,-8},{32,62,12,-18,-36,55,-46,-8},{33,62,12,-18,-36,55,-46,-9},
	{33,63,12,-18,-37,56,-46,-9},{33,64,12,-19,-37,56,-47,-9},{34,64,12,-19,-37,57,-47,-9},{34,65,13,-19,-38,57,-48,-9},
	{34,65,13,-19,-38,58,-48,-9},{34,66,13,-19,-38,58,-48,-9},{35,66,13,-19,-39,59,-49,-9},{35,67,13,-20,-39,59,-49,-9},
	{35,68,13,-20,-39,60,-50,-9},{36,68,13,-20,-40,60,-50,-9},{36,69,13,-20,-40,61,-51,-9},{36,69,14,-20,-40,61,-51,-9},
	{37,70,14,-20,-41,62,-51,-10},{37,70,14,-21,-41,62,-52,-10},{37,71,14,-21,-41,63,-52,-10},{37,72,14,-21,-42,63,-53,-10},
	{38,72,14,-21,-42,64,-53,-10},{38,73,14,-21,-42,64,-54,-10},{38,73,14,-21,-43,65,-54,-10},{39,74,14,-22,-43,65,-54,-10},
	{39,74,15,-22,-43,66,-55,-10},{39,75,15,-22,-44,66,-55,-10},{40,75,15,-22,-44,67,-56,-10},{40,76,15,-22,-44,67,-56,-10},
	{40,77,15,-22,-45,68,-56,-11},{40,77,15,-23,-45,68,-57,-11},{41,78,15,-23,-45,69,-57,-11},{41,78,15,-23,-46,69,-58,-11},
	{41,79,15,-23,-46,70,-58,-11},{42,79,16,-23,-46,70,-59,-11},{42,80,16,-23,-47,71,-59,-11},{42,81,16,-24,-47,71,-59,-11},
	{43,81,16,-24,-47,72,-60,-11},{43,82,16,-24,-48,72,-60,-11},{43,82,16,-24,-48,73,-61,-11},{43,83,16,-24,-48,73,-61,-11},
	{44,83,16,-24,-49,74,-61,-12},{44,84,16,-25,-49,74,-62,-12},{44,85,17,-25,-49,75,-62,-12},{45,85,17,-25,-50,75,-63,-12},
	{45,86,17,-25,-50,76,-63,-12},{45,86,17,-25,-50,76,-64,-12},{46,87,17,-25,-51,77,-64,-12},{46,87,17,-26,-51,77,-64,-12},
	{46,88,17,-26,-51,78,-65,-12},{46,89,17,-26,-52,78,-65,-12},{47,89,18,-26,-52,79,-66,-12},{47,90,18,-26,-52,79,-66,-12},
	{47,90,18,-26,-53,80,-66,-13},{48,91,18,-27,-53,80,-67,-13},{48,91,18,-27,-53,81,-67,-13},{48,92,18,-27,-54,81,-68,-13},
	{49,92,18,-27,-54,82,-68,-13},{49,93,18,-27,-54,82,-69,-13},{49,94,18,-28,-54,83,-69,-13},{49,94,19,-28,-55,83,-69,-13},
	{50,95,19,-28,-55,84,-70,-13},{50,95,19,-28,-55,84,-70,-13},{50,96,19,-28,-56,85,-71,-13},{51,96,19,-28,-56,85,-71,-13},
	{51,97,19,-29,-56,86,-72,-13},{51,98,19,-29,-57,86,-72,-14},{52,98,19,-29,-57,87,-72,-14},{52,99,19,-29,-57,87,-73,-14},
	{52,99,20,-29,-58,88,-73,-14},{52,100,20,-29,-58,88,-74,-14},{53,100,20,-30,-58,89,-74,-14},{53,101,20,-30,-59,89,-74,-14},
	{53,102,20,-30,-59,90,-75,-14},{54,102,20,-30,-59,90,-75,-14},{54,103,20,-30,-60,91,-76,-14},{54,103,20,-30,-60,91,-76,-14},
	{55,104,20,-31,-60,92,-77,-14},{55,104,21,-31,-61,92,-77,-15},{55,105,21,-31,-61,93,-77,-15},{55,106,21,-31,-61,93,-78,-15},
	{56,106,21,-31,-62,94,-78,-15},{56,107,21,-31,-62,94,-79,-15},{56,107,21,-32,-62,95,-79,-15},{57,108,21,-32,-63,95,-79,-15},
	{57,108,21,-32,-63,96,-80,-15},{57,109,22,-32,-63,96,-80,-15},{58,109,22,-32,-64,97,-81,-15},{58,110,22,-32,-64,97,-81,-15},
	{58,111,22,-33,-64,98,-82,-15},{58,111,22,-33,-65,98,-82,-16},{59,112,22,-33,-65,99,-82,-16},{59,112,22,-33,-65,99,-83,-16},
	{59,113,22,-33,-66,100,-83,-16},{60,113,22,-33,-66,100,-84,-16},{60,114,23,-34,-66,101,-84,-16},{60,115,23,-34,-67,101,-84,-16},
	{60,115,23,-34,-67,102,-85,-16},{61,116,23,-34,-67,102,-85,-16},{61,116,23,-34,-68,103,-86,-16},{61,117,23,-34,-68,103,-86,-16},
	{62,117,23,-35,-68,104,-87,-16},{62,118,23,-35,-69,104,-87,-16},{62,119,23,-35,-69,105,-87,-17},{63,119,24,-35,-69,105,-88,-17},
	{63,120,24,-35,-70,106,-88,-17},{63,120,24,-35,-70,106,-89,-17},{63,121,24,-36,-70,107,-89,-17},{64,121,24,-36,-71,107,-90,-17},
	{64,122,24,-36,-71,108,-90,-17},{64,123,24,-36,-71,108,-90,-17},{65,123,24,-36,-72,109,-91,-17},{65,124,24,-36,-72,109,-91,-17},
	{65,124,25,-37,-72,110,-92,-17},{66,125,25,-37,-73,110,-92,-17},{66,125,25,-37,-73,111,-92,-18},{66,126,25,-37,-73,111,-93,-18},
	{66,127,25,-37,-74,112,-93,-18},{67,127,25,-37,-74,112,-94,-18},{67,128,25,-38,-74,113,-94,-18},{67,128,25,-38,-75,113,-95,-18},
	{68,129,25,-38,-75,114,-95,-18},{68,129,26,-38,-75,114,-95,-18},{68,130,26,-38,-76,115,-96,-18},{69,130,26,-38,-76,115,-96,-18},
	{69,131,26,-39,-76,116,-97,-18},{69,132,26,-39,-77,116,-97,-18},{69,132,26,-39,-77,117,-97,-19},{70,133,26,-39,-77,117,-98,-19},
	{70,133,26,-39,-78,118,-98,-19},{70,134,27,-39,-78,118,-99,-19},{71,134,27,-40,-78,119,-99,-19},{71,135,27,-40,-79,119,-100,-19},
	{71,136,27,-40,-79,120,-100,-19},{72,136,27,-40,-79,120,-100,-19},{72,137,27,-40,-80,121,-101,-19},{72,137,27,-40,-80,121,-101,-19},
	{72,138,27,-41,-80,122,-102,-19},{73,138,27,-41,-81,122,-102,-19},{73,139,28,-41,-81,123,-103,-19},{73,140,28,-41,-81,123,-103,-20},
	{74,140,28,-41,-82,124,-103,-20},{74,141,28,-42,-82,124,-104,-20},{74,141,28,-42,-82,125,-104,-20},{75,142,28,-42,-83,125,-105,-20},
	{75,142,28,-42,-83,126,-105,-20},{75,143,28,-42,-83,126,-105,-20},{75,144,28,-42,-84,127,-106,-20},{76,144,29,-43,-84,127,-106,-20}
};


/** 
 * @brief Decompress a frame
 *
 * This function permits to decompress a frame from the video stream.
 *
 * @param dev Device structure
 * 
 * @returns 0 if all is OK
 */
int linect_rgb_decompress(struct usb_linect *dev)
{
	void *data;
	void *image;
	struct linect_frame_buf *framebuf;

	if (dev == NULL)
		return -EFAULT;

	framebuf = dev->cam->read_frame;

	if (framebuf == NULL)
		return -EFAULT;

	image  = dev->cam->image_data;
	image += dev->cam->images[dev->cam->fill_image].offset;

	data = framebuf->data;

	switch (dev->cam->vsettings.palette) {
		case LNT_PALETTE_RGB24:
			linect_b2rgb24(data, image);
			break;

		case LNT_PALETTE_RGB32:
			linect_b2rgb32(data, image);
			break;

		case LNT_PALETTE_BGR24:
			linect_b2bgr24(data, image);
			break;

		case LNT_PALETTE_BGR32:
			linect_b2bgr32(data, image);
			break;

		case LNT_PALETTE_UYVY:
			linect_b2uyvy(data, image);
			break;

		case LNT_PALETTE_YUYV:
			linect_b2yuyv(data, image);
			break;
	}

	linect_correct_brightness(image, 640, 480,
		dev->cam->vsettings.brightness, dev->cam->vsettings.palette, dev->cam->vsettings.depth); //0x7f00

	return 0;
}

int linect_depth_decompress(struct usb_linect *dev)
{
	uint8_t *data;
	uint8_t *image;
	uint16_t *image_tmp;
	struct linect_frame_buf *framebuf;
	int bitshift, idx, i, mask;
	uint32_t word;

	if (dev == NULL)
		return -EFAULT;

	framebuf = dev->cam->read_frame_depth;

	if (framebuf == NULL)
		return -EFAULT;

	image  = dev->cam->image_data_depth;
	image += dev->cam->images_depth[dev->cam->fill_image_depth].offset;

	data = framebuf->data;
	
	image_tmp = (uint16_t *) dev->cam->image_tmp;
	
	LNT_DEBUG("Decompress depth frame!!\n");

	// Always rgb24
	
	if (data == NULL) {
		LNT_DEBUG("BUG: Handle depth frame failed. Data = NULL.");
		
	}
	
	// Convert uint16
	
	mask = (1 << 11) - 1;
	bitshift = 0;
	for (i=0; i<(640*480); i++) {
		idx = (i*11)/8;
		word = (data[idx]<<(16)) | (data[idx+1]<<8) | data[idx+2];
		image_tmp[i] = ((word >> (((3*8)-11)-bitshift)) & mask);
		bitshift = (bitshift + 11) % 8;
	}
	
	switch (dev->cam->depth_vsettings.palette) {
		case LNT_PALETTE_RGB24:
			linect_depth2rgb24(image_tmp, image);
			break;
		case LNT_PALETTE_DEPTHRAW:
			linect_depth2raw(image_tmp, image);
			break;
	}

	return 0;
}

void linect_depth2rgb24(uint16_t *depth, uint8_t *image) {
	int pval, lb, i;

	for (i=0; i<640*480; i++) {
		pval = t_gamma[depth[i]];
		lb = pval & 0xff;
		switch (pval>>8) {
			case 0:
				image[3*i+0] = 255;
				image[3*i+1] = 255-lb;
				image[3*i+2] = 255-lb;
				break;
			case 1:
				image[3*i+0] = 255;
				image[3*i+1] = lb;
				image[3*i+2] = 0;
				break;
			case 2:
				image[3*i+0] = 255-lb;
				image[3*i+1] = 255;
				image[3*i+2] = 0;
				break;
			case 3:
				image[3*i+0] = 0;
				image[3*i+1] = 255;
				image[3*i+2] = lb;
				break;
			case 4:
				image[3*i+0] = 0;
				image[3*i+1] = 255-lb;
				image[3*i+2] = 255;
				break;
			case 5:
				image[3*i+0] = 0;
				image[3*i+1] = 0;
				image[3*i+2] = 255-lb;
				break;
			default:
				image[3*i+0] = 0;
				image[3*i+1] = 0;
				image[3*i+2] = 0;
				break;
		}
	}
}

void linect_depth2raw(uint16_t *depth, uint8_t *image) {
	
	memcpy(image, (uint8_t *)depth, 640*480*2);

}


/** 
 * @brief Correct the brightness of an image.
 *
 * This function permits to correct the brightness of an image.
 *
 * @param img Buffer to RGB/YUV data
 * @param width Width of frame
 * @param height Height of frame
 * @param brightness Brightness correction
 * @param depth Color depth
 *
 * @retval rgb Buffer to RGB/YUV data
 */
void linect_correct_brightness(uint8_t *img, const int width, const int height, 
		const int brightness, int palette, int depth)
{
	int i;
	int x;


	switch (palette) {
		case LNT_PALETTE_RGB24:
		case LNT_PALETTE_BGR24:
		case LNT_PALETTE_RGB32:
		case LNT_PALETTE_BGR32:
			depth = (depth == 24) ? 3 : 4;

			if (brightness >= 32767) {
				x = (brightness - 32767) / 256;

				for (i = 0; i < (width * height * depth); i++) {
					if ((*(img + i) + (unsigned char) x) > 255)
						*(img + i) = 255;
					else
						*(img + i) += (unsigned char) x;
				}
			}
			else {
				x = (32767 - brightness) / 256;
		
				for (i = 0; i < (width * height * depth); i++) {
					if ((unsigned char) x > *(img + i))
						*(img + i) = 0;
					else
						*(img + i) -= (unsigned char) x;
				}
			}

			break;

		case LNT_PALETTE_UYVY:
			depth = 2;

			if (brightness >= 32767) {
				x = (brightness - 32767) / 256;

				for (i = 1; i < (width * height * depth); i=i+depth) {
					if ((*(img + i) + (unsigned char) x) > 255)
						*(img + i) = 255;
					else
						*(img + i) += (unsigned char) x;
				}
			}
			else {
				x = (32767 - brightness) / 256;
		
				for (i = 1; i < (width * height * depth); i=i+depth) {
					if ((unsigned char) x > *(img + i))
						*(img + i) = 0;
					else
						*(img + i) -= (unsigned char) x;
				}
			}

			break;

		case LNT_PALETTE_YUYV:
			depth = 2;

			if (brightness >= 32767) {
				x = (brightness - 32767) / 256;

				for (i = 0; i < (width * height * depth); i=i+depth) {
					if ((*(img + i) + (unsigned char) x) > 255)
						*(img + i) = 255;
					else
						*(img + i) += (unsigned char) x;
				}
			}
			else {
				x = (32767 - brightness) / 256;
		
				for (i = 0; i < (width * height * depth); i=i+depth) {
					if ((unsigned char) x > *(img + i))
						*(img + i) = 0;
					else
						*(img + i) -= (unsigned char) x;
				}
			}

			break;
	}
}

#define R(x,y) pRGB24[0 + 3 * ((x) + 640 * (y))]
#define G(x,y) pRGB24[1 + 3 * ((x) + 640 * (y))]
#define B(x,y) pRGB24[2 + 3 * ((x) + 640 * (y))]

#define Bay(x,y) pBay[(x) + 640 * (y)]

void bayer_copy(u8 *pBay, u8 *pRGB24, int x, int y)
{

	G(x + 0, y + 0) = Bay(x + 0, y + 0);
	G(x + 1, y + 1) = Bay(x + 1, y + 1);
	G(x + 0, y + 1) = G(x + 1, y + 0) = ((u32)Bay(x + 0, y + 0) + (u32)Bay(x + 1, y + 1)) / 2;
	R(x + 0, y + 0) = R(x + 1, y + 0) = R(x + 1, y + 1) = R(x + 0, y + 1) = Bay(x + 0, y + 1);
	B(x + 1, y + 1) = B(x + 0, y + 0) = B(x + 0, y + 1) = B(x + 1, y + 0) = Bay(x + 1, y + 0);
}

void bayer_bilinear(u8 *pBay, u8 *pRGB24, int x, int y)
{
	R(x + 0, y + 0) = ((u32)Bay(x + 0, y + 1) + (u32)Bay(x + 0, y - 1)) / 2;
	G(x + 0, y + 0) = Bay(x + 0, y + 0);
	B(x + 0, y + 0) = ((u32)Bay(x - 1, y + 0) + (u32)Bay(x + 1, y + 0)) / 2;

	R(x + 0, y + 1) = Bay(x + 0, y + 1);
	G(x + 0, y + 1) = ((u32)Bay(x + 0, y + 0) + (u32)Bay(x + 0, y + 2)
			 + (u32)Bay(x - 1, y + 1) + (u32)Bay(x + 1, y + 1)) / 4;
	B(x + 0, y + 1) = ((u32)Bay(x + 1, y + 0) + (u32)Bay(x - 1, y + 0)
			 + (u32)Bay(x + 1, y + 2) + (u32)Bay(x - 1, y + 2)) / 4;

	R(x + 1, y + 0) = ((u32)Bay(x + 0, y + 1) + (u32)Bay(x + 2, y + 1)
			 + (u32)Bay(x + 0, y - 1) + (u32)Bay(x + 2, y - 1)) / 4;
	G(x + 1, y + 0) = ((u32)Bay(x + 0, y + 0) + (u32)Bay(x + 2, y + 0)
			 + (u32)Bay(x + 1, y - 1) + (u32)Bay(x + 1, y + 1)) / 4;
	B(x + 1, y + 0) = Bay(x + 1, y + 0);

	R(x + 1, y + 1) = ((u32)Bay(x + 0, y + 1) + (u32)Bay(x + 2, y + 1)) / 2;
	G(x + 1, y + 1) = Bay(x + 1, y + 1);
	B(x + 1, y + 1) = ((u32)Bay(x + 1, y + 0) + (u32)Bay(x + 1, y + 2)) / 2;
}

/** 
 * @brief This function permits to convert an image from bayer to RGB24
 *
 * @param bayer Buffer with the bayer data
 * @param image Size of image
 * @param view Size of view
 * @param hflip Horizontal flip
 * @param vflip Vertical flip
 * @param factor Factor of redimensioning
 *
 * @retval rgb Buffer with the RGB data
 */
void linect_b2rgb24(uint8_t *bayer, uint8_t *rgb) {

	int i, j; // Position in rgb image

	for (i = 1; i < 640; i += 2)
		for (j = 1; j < 480; j += 2)
			if (i == 0 || j == 0 || i == 640 - 2 || j == 480 - 2)
				bayer_copy(bayer, rgb, i, j);
			else
				bayer_bilinear(bayer, rgb, i, j);
	
}


/** 
 * @brief This function permits to convert an image from bayer to RGB32
 *
 * @param bayer Buffer with the bayer data
 * @param image Size of image
 * @param view Size of view
 * @param hflip Horizontal flip
 * @param vflip Vertical flip
 * @param factor Factor of redimensioning
 *
 * @retval rgb Buffer with the RGB data
 */
void linect_b2rgb32(uint8_t *bayer, uint8_t *rgb) {
	uint8_t *b;

	int x, y; // Position in bayer image
	int i, j; // Position in rgb image

	int width = 640;
	int height = 480;
	
	int factor = 1;
	
	int nwidth = width / factor;
	int nheight = height / factor;

	int offset;
	int startx, stepx;
	int starty, stepy;

	// Calculate the initial position (on Y axis)

		starty = 0;
		stepy = factor;


	// Calculate the initial position (on X axis)

		startx = 0;
		stepx = factor;
		offset = 1;


	// Skip the first line
	bayer += width;

	// To center vertically the image in the view
	rgb += ((480 - nheight) / 2) * 640 * 4;

	// To center horizontally the image in the view
	rgb += ((640 - nwidth) / 2) * 4;

	// Clean the first line
	memset(rgb, 0, nwidth * 4);
	rgb += nwidth * 4;


	// For each rgb line without the borders (first and last line)
	for (j=0, y=starty; j<nheight-2; j++, y=y+stepy) {
		// Go to the start of line
		b = bayer + y * width + offset;

		// Offset to center horizontally the image in the view
		rgb += (640 - nwidth) * 4;

		if (y & 0x1) {
			// Skip the first pixel
			*rgb++ = 0;
			*rgb++ = 0;
			*rgb++ = 0;
			*rgb++ = 0;

			// GBGBGB : Line process...
			for (i=0, x=startx; i<nwidth-2; i++, x=x+stepx) {
				if (x & 0x1) {
					*rgb++ = (*(b-width-1) + *(b-width+1) + *(b+width-1) + *(b+width+1)) >> 2;
					*rgb++ = (*(b-width) + *(b-1) + *(b+1) + *(b+width)) >> 2;
					*rgb++ = *b;
					*rgb++ = 0;
				}
				else {
					*rgb++ = (*(b-width) + *(b+width)) >> 1;
					*rgb++ = *b;
					*rgb++ = (*(b-1) + *(b+1)) >> 1;
					*rgb++ = 0;
				}

				b += stepx;
			}

			// Skip the last pixel
			*rgb++ = 0;
			*rgb++ = 0;
			*rgb++ = 0;
			*rgb++ = 0;
		}
		else {
			// Skip the first pixel
			*rgb++ = 0;
			*rgb++ = 0;
			*rgb++ = 0;
			*rgb++ = 0;

			// RGRGRG : Line process...
			for (i=0, x=startx; i<nwidth-2; i++, x=x+stepx) {
				if (x & 0x1) {
					*rgb++ = (*(b-1) + *(b+1)) >> 1;
					*rgb++ = *b;
					*rgb++ = (*(b-width) + *(b+width)) >> 1;
					*rgb++ = 0;
				}
				else {
					*rgb++ = *b;
					*rgb++ = (*(b-width) + *(b-1) + *(b+1) + *(b+width)) >> 2;
					*rgb++ = (*(b-width-1) + *(b-width+1) + *(b+width-1) + *(b+width+1)) >> 2;
					*rgb++ = 0;
				}
	
				b += stepx;
			}

			// Skip the last pixel
			*rgb++ = 0;
			*rgb++ = 0;
			*rgb++ = 0;
			*rgb++ = 0;
		}
	}

	// Clean the last line
	memset(rgb, 0, nwidth * 4);
}


/** 
 * @brief This function permits to convert an image from bayer to BGR24
 *
 * @param bayer Buffer with the bayer data
 * @param image Size of image
 * @param view Size of view
 * @param hflip Horizontal flip
 * @param vflip Vertical flip
 * @param factor Factor of redimensioning
 *
 * @retval bgr Buffer with the BGR data
 */
void linect_b2bgr24(uint8_t *bayer, uint8_t *bgr) {
	uint8_t *b;

	int x, y; // Position in bayer image
	int i, j; // Position in bgr image

	int width = 640;
	int height = 480;
int factor = 1;
	int nwidth = width / factor;
	int nheight = height / factor;

	int offset;
	int startx, stepx;
	int starty, stepy;
	
	


	// Calculate the initial position (on Y axis)

		starty = 0;
		stepy = factor;


	// Calculate the initial position (on X axis)

		startx = 0;
		stepx = factor;
		offset = 1;


	// Skip the first line
	bayer += width;

	// To center vertically the image in the view
	bgr += ((480- nheight) / 2) * 640 * 3;

	// To center horizontally the image in the view
	bgr += ((640 - nwidth) / 2) * 3;

	// Clean the first line
	memset(bgr, 0, nwidth * 3);
	bgr += nwidth * 3;


	// For each bgr line without the borders (first and last line)
	for (j=0, y=starty; j<nheight-2; j++, y=y+stepy) {
		// Go to the start of line
		b = bayer + y * width + offset;

		// Offset to center horizontally the image in the view
		bgr += (640 - nwidth) * 3;

		if (y & 0x1) {
			// Skip the first pixel
			*bgr++ = 0;
			*bgr++ = 0;
			*bgr++ = 0;

			// GBGBGB : Line process...
			for (i=0, x=startx; i<nwidth-2; i++, x=x+stepx) {
				if (x & 0x1) {
					*bgr++ = *b;
					*bgr++ = (*(b-width) + *(b-1) + *(b+1) + *(b+width)) >> 2;
					*bgr++ = (*(b-width-1) + *(b-width+1) + *(b+width-1) + *(b+width+1)) >> 2;
				}
				else {
					*bgr++ = (*(b-1) + *(b+1)) >> 1;
					*bgr++ = *b;
					*bgr++ = (*(b-width) + *(b+width)) >> 1;
				}

				b += stepx;
			}

			// Skip the last pixel
			*bgr++ = 0;
			*bgr++ = 0;
			*bgr++ = 0;
		}
		else {
			// Skip the first pixel
			*bgr++ = 0;
			*bgr++ = 0;
			*bgr++ = 0;

			// RGRGRG : Line process...
			for (i=0, x=startx; i<nwidth-2; i++, x=x+stepx) {
				if (x & 0x1) {
					*bgr++ = (*(b-width) + *(b+width)) >> 1;
					*bgr++ = *b;
					*bgr++ = (*(b-1) + *(b+1)) >> 1;
				}
				else {
					*bgr++ = (*(b-width-1) + *(b-width+1) + *(b+width-1) + *(b+width+1)) >> 2;
					*bgr++ = (*(b-width) + *(b-1) + *(b+1) + *(b+width)) >> 2;
					*bgr++ = *b;
				}
	
				b += stepx;
			}

			// Skip the last pixel
			*bgr++ = 0;
			*bgr++ = 0;
			*bgr++ = 0;
		}
	}

	// Clean the last line
	memset(bgr, 0, nwidth * 3);
}


/** 
 * @brief This function permits to convert an image from bayer to BGR32
 *
 * @param bayer Buffer with the bayer data
 * @param image Size of image
 * @param view Size of view
 * @param hflip Horizontal flip
 * @param vflip Vertical flip
 * @param factor Factor of redimensioning
 *
 * @retval bgr Buffer with the BGR data
 */
void linect_b2bgr32(uint8_t *bayer, uint8_t *bgr) {
	uint8_t *b;

	int x, y; // Position in bayer image
	int i, j; // Position in bgr image

	int width = 640;
	int height = 480;
int factor = 1;
	int nwidth = width / factor;
	int nheight = height / factor;

	int offset;
	int startx, stepx;
	int starty, stepy;
	
	


	// Calculate the initial position (on Y axis)

		starty = 0;
		stepy = factor;
	

	// Calculate the initial position (on X axis)

		startx = 0;
		stepx = factor;
		offset = 1;
	


	// Skip the first line
	bayer += width;

	// To center vertically the image in the view
	bgr += ((480 - nheight) / 2) * 640 * 4;

	// To center horizontally the image in the view
	bgr += ((640 - nwidth) / 2) * 4;

	// Clean the first line
	memset(bgr, 0, nwidth * 4);
	bgr += nwidth * 4;


	// For each bgr line without the borders (first and last line)
	for (j=0, y=starty; j<nheight-2; j++, y=y+stepy) {
		// Go to the start of line
		b = bayer + y * width + offset;

		// Offset to center horizontally the image in the view
		bgr += (640 - nwidth) * 4;

		if (y & 0x1) {
			// Skip the first pixel
			*bgr++ = 0;
			*bgr++ = 0;
			*bgr++ = 0;
			*bgr++ = 0;

			// GBGBGB : Line process...
			for (i=0, x=startx; i<nwidth-2; i++, x=x+stepx) {
				if (x & 0x1) {
					*bgr++ = *b;
					*bgr++ = (*(b-width) + *(b-1) + *(b+1) + *(b+width)) >> 2;
					*bgr++ = (*(b-width-1) + *(b-width+1) + *(b+width-1) + *(b+width+1)) >> 2;
					*bgr++ = 0;
				}
				else {
					*bgr++ = (*(b-1) + *(b+1)) >> 1;
					*bgr++ = *b;
					*bgr++ = (*(b-width) + *(b+width)) >> 1;
					*bgr++ = 0;
				}

				b += stepx;
			}

			// Skip the last pixel
			*bgr++ = 0;
			*bgr++ = 0;
			*bgr++ = 0;
			*bgr++ = 0;
		}
		else {
			// Skip the first pixel
			*bgr++ = 0;
			*bgr++ = 0;
			*bgr++ = 0;
			*bgr++ = 0;

			// RGRGRG : Line process...
			for (i=0, x=startx; i<nwidth-2; i++, x=x+stepx) {
				if (x & 0x1) {
					*bgr++ = (*(b-width) + *(b+width)) >> 1;
					*bgr++ = *b;
					*bgr++ = (*(b-1) + *(b+1)) >> 1;
					*bgr++ = 0;
				}
				else {
					*bgr++ = (*(b-width-1) + *(b-width+1) + *(b+width-1) + *(b+width+1)) >> 2;
					*bgr++ = (*(b-width) + *(b-1) + *(b+1) + *(b+width)) >> 2;
					*bgr++ = *b;
					*bgr++ = 0;
				}
	
				b += stepx;
			}

			// Skip the last pixel
			*bgr++ = 0;
			*bgr++ = 0;
			*bgr++ = 0;
			*bgr++ = 0;
		}
	}

	// Clean the last line
	memset(bgr, 0, nwidth * 4);
}


/** 
 * @brief This function permits to convert an image from bayer to YUV (UYVY)
 *
 * @param bayer Buffer with the bayer data
 * @param image Size of image
 * @param view Size of view
 * @param hflip Horizontal flip
 * @param vflip Vertical flip
 * @param factor Factor of redimensioning
 *
 * @retval yuv Buffer with the YUV data
 */
void linect_b2uyvy(uint8_t *bayer, uint8_t *yuv) {
	uint8_t *b;

	int x, y; // Position in bayer image
	int i, j; // Position in yuv image

	int pR, pG, pB;
	int pY, pU, pV;
int factor = 1;
	int width = 640;
	int height = 480;

	int nwidth = width / factor;
	int nheight = height / factor;

	int offset;
	int startx, stepx;
	int starty, stepy;
	
	


	// Calculate the initial position (on Y axis)

		starty = 0;
		stepy = factor;
	

	// Calculate the initial position (on X axis)

		startx = 0;
		stepx = factor;
		offset = 1;
	
	
	starty = 0;
	stepy = factor;
	startx = 0;
	stepx = factor;
	offset = 1;

	// Background color...
	memset(yuv, 16, width * 2);
	for (i=0; i<width*2; i=i+2, *(yuv+i)=128);
	for (i=1; i<height; i++)
		memcpy(yuv+i*width*2, yuv, width*2);

	// Skip the first line
	bayer += width;

	// To center vertically the image in the view
	yuv += ((480 - nheight) / 2) * 640 * 2;

	// To center horizontally the image in the view
	yuv += ((640 - nwidth) / 2) * 2;

	// Clean the first line
	memset(yuv, 16, nwidth * 2);
	for (i=0; i<nwidth*2; i=i+2, *(yuv+i)=128);
	yuv += nwidth * 2;


	// For each yuv line without the borders (first and last line)
	for (j=0, y=starty; j<nheight-2; j++, y=y+stepy) {
		// Go to the start of line
		b = bayer + y * width + offset;

		// Offset to center horizontally the image in the view
		yuv += (640 - nwidth) * 2;

		if (y & 0x1) {
			// Skip the first pixel
			*yuv++ = 128;
			*yuv++ = 16;

			// GBGBGB : Line process...
			for (i=0, x=startx; i<nwidth-2; i++, x=x+stepx) {
				if (!x & 0x1) {
					pR = (*(b-width-1) + *(b-width+1) + *(b+width-1) + *(b+width+1)) >> 2;
					pG = (*(b-width) + *(b-1) + *(b+1) + *(b+width)) >> 2;
					pB = *b;
				}
				else {
					pR = (*(b-width) + *(b+width)) >> 1;
					pG = *b;
					pB = (*(b-1) + *(b+1)) >> 1;
				}

				pY = linect_yuv_interp[pR][0] + linect_yuv_interp[pG][1] + linect_yuv_interp[pB][2];
				pU = linect_yuv_interp[pR][3] + linect_yuv_interp[pG][4] + linect_yuv_interp[pB][5];
				pV = linect_yuv_interp[pR][5] + linect_yuv_interp[pG][6] + linect_yuv_interp[pB][7];

				pY = CLIP(pY, 0,255);
				pU = CLIP(pU, -127,127);
				pV = CLIP(pV, -127,127);
	
				if (i % 2){
					*yuv++ = (112 * pU)/127 + 128; // U
					*yuv++ = (219 * pY)/255 + 16;  // Y
				}
				else {
					*yuv++ = (112 * pV)/127 + 128; // V
					*yuv++ = (219 * pY)/255 + 16;  // Y
				}

				b += stepx;
			}

			// Skip the last pixel
			*yuv++ = 128;
			*yuv++ = 16;
		}
		else {
			// Skip the first pixel
			*yuv++ = 128;
			*yuv++ = 16;

			// RGRGRG : Line process...
			for (i=0, x=startx; i<nwidth-2; i++, x=x+stepx) {
				if (!x & 0x1) {
					pR = (*(b-1) + *(b+1)) >> 1;
					pG = *b;
					pB = (*(b-width) + *(b+width)) >> 1;
				}
				else {
					pR = *b;
					pG = (*(b-width) + *(b-1) + *(b+1) + *(b+width)) >> 2;
					pB = (*(b-width-1) + *(b-width+1) + *(b+width-1) + *(b+width+1)) >> 2;
				}

				pY = linect_yuv_interp[pR][0] + linect_yuv_interp[pG][1] + linect_yuv_interp[pB][2];
				pU = linect_yuv_interp[pR][3] + linect_yuv_interp[pG][4] + linect_yuv_interp[pB][5];
				pV = linect_yuv_interp[pR][5] + linect_yuv_interp[pG][6] + linect_yuv_interp[pB][7];

				pY = CLIP(pY, 0,255);
				pU = CLIP(pU, -127,127);
				pV = CLIP(pV, -127,127);
	
				if (i % 2){
					*yuv++ = (112 * pU)/127 + 128; // U
					*yuv++ = (219 * pY)/255 + 16;  // Y
				}
				else {
					*yuv++ = (112 * pV)/127 + 128; // V
					*yuv++ = (219 * pY)/255 + 16;  // Y
				}

				b += stepx;
			}

			// Skip the last pixel
			*yuv++ = 128;
			*yuv++ = 16;
		}
	}

	// Clean the last line
	memset(yuv, 16, nwidth * 2);
	for (i=0; i<nwidth*2; i=i+2, *(yuv+i)=128);
}


/** 
 * @brief This function permits to convert an image from bayer to YUV (YUYV)
 *
 * @param bayer Buffer with the bayer data
 * @param image Size of image
 * @param view Size of view
 * @param hflip Horizontal flip
 * @param vflip Vertical flip
 * @param factor Factor of redimensioning
 *
 * @retval yuv Buffer with the YUV data
 */
void linect_b2yuyv(uint8_t *bayer, uint8_t *yuv) {
	uint8_t *b;

	int x, y; // Position in bayer image
	int i, j; // Position in yuv image

	int pR, pG, pB;
	int pY, pU, pV;
int factor = 1;
	int width = 640;
	int height = 480;

	int nwidth = width / factor;
	int nheight = height / factor;

	int offset;
	int startx, stepx;
	int starty, stepy;
	
	


	// Calculate the initial position (on Y axis)

		starty = 0;
		stepy = factor;
	

	// Calculate the initial position (on X axis)

		startx = 0;
		stepx = factor;
		offset = 1;
	

	// Background color...
	memset(yuv, 128, width * 2);
	for (i=0; i<width*2; i=i+2, *(yuv+i)=16);
	for (i=1; i<height; i++)
		memcpy(yuv+i*width*2, yuv, width*2);

	// Skip the first line
	bayer += width;

	// To center vertically the image in the view
	yuv += ((480 - nheight) / 2) * 640 * 2;

	// To center horizontally the image in the view
	yuv += ((640 - nwidth) / 2) * 2;

	// Clean the first line
	memset(yuv, 128, nwidth * 2);
	for (i=0; i<nwidth*2; i=i+2, *(yuv+i)=16);
	yuv += nwidth * 2;


	// For each yuv line without the borders (first and last line)
	for (j=0, y=starty; j<nheight-2; j++, y=y+stepy) {
		// Go to the start of line
		b = bayer + y * width + offset;

		// Offset to center horizontally the image in the view
		yuv += (640 - nwidth) * 2;

		if (y & 0x1) {
			// Skip the first pixel
			*yuv++ = 16;
			*yuv++ = 128;

			// GBGBGB : Line process...
			for (i=0, x=startx; i<nwidth-2; i++, x=x+stepx) {
				if (x & 0x1) {
					pR = (*(b-width-1) + *(b-width+1) + *(b+width-1) + *(b+width+1)) >> 2;
					pG = (*(b-width) + *(b-1) + *(b+1) + *(b+width)) >> 2;
					pB = *b;
				}
				else {
					pR = (*(b-width) + *(b+width)) >> 1;
					pG = *b;
					pB = (*(b-1) + *(b+1)) >> 1;
				}

				pY = linect_yuv_interp[pR][0] + linect_yuv_interp[pG][1] + linect_yuv_interp[pB][2];
				pU = linect_yuv_interp[pR][3] + linect_yuv_interp[pG][4] + linect_yuv_interp[pB][5];
				pV = linect_yuv_interp[pR][5] + linect_yuv_interp[pG][6] + linect_yuv_interp[pB][7];

				pY = CLIP(pY, 0,255);
				pU = CLIP(pU, -127,127);
				pV = CLIP(pV, -127,127);
	
				if (i % 2){
					*yuv++ = (219 * pY)/255 + 16;  // Y
					*yuv++ = (112 * pU)/127 + 128; // U
				}
				else {
					*yuv++ = (219 * pY)/255 + 16;  // Y
					*yuv++ = (112 * pV)/127 + 128; // V
				}

				b += stepx;
			}

			// Skip the last pixel
			*yuv++ = 16;
			*yuv++ = 128;
		}
		else {
			// Skip the first pixel
			*yuv++ = 16;
			*yuv++ = 128;

			// RGRGRG : Line process...
			for (i=0, x=startx; i<nwidth-2; i++, x=x+stepx) {
				if (x & 0x1) {
					pR = (*(b-1) + *(b+1)) >> 1;
					pG = *b;
					pB = (*(b-width) + *(b+width)) >> 1;
				}
				else {
					pR = *b;
					pG = (*(b-width) + *(b-1) + *(b+1) + *(b+width)) >> 2;
					pB = (*(b-width-1) + *(b-width+1) + *(b+width-1) + *(b+width+1)) >> 2;
				}

				pY = linect_yuv_interp[pR][0] + linect_yuv_interp[pG][1] + linect_yuv_interp[pB][2];
				pU = linect_yuv_interp[pR][3] + linect_yuv_interp[pG][4] + linect_yuv_interp[pB][5];
				pV = linect_yuv_interp[pR][5] + linect_yuv_interp[pG][6] + linect_yuv_interp[pB][7];

				pY = CLIP(pY, 0,255);
				pU = CLIP(pU, -127,127);
				pV = CLIP(pV, -127,127);
	
				if (i % 2){
					*yuv++ = (219 * pY)/255 + 16;  // Y
					*yuv++ = (112 * pU)/127 + 128; // U
				}
				else {
					*yuv++ = (219 * pY)/255 + 16;  // Y
					*yuv++ = (112 * pV)/127 + 128; // V
				}

				b += stepx;
			}

			// Skip the last pixel
			*yuv++ = 16;
			*yuv++ = 128;
		}
	}

	// Clean the last line
	memset(yuv, 128, nwidth * 2);
	for (i=0; i<nwidth*2; i=i+2, *(yuv+i)=16);
}


