/** 
 * @file linect_v4l_ctrl.h
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

#ifndef LINECT_V4L_CTRL_H
#define LINECT_V4L_CTRL_H

#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>

#define V4L2_CCID_MOTOR V4L2_CID_PRIVATE_BASE+0
#define V4L2_CCID_LED V4L2_CID_PRIVATE_BASE+1

#endif 
