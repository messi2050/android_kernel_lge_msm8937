/* mausb.h
 *
 * Copyright (C) 2015-2016 LGE Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2  of
 * the License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef __LINUX_MAUSB_H
#define __LINUX_MAUSB_H

#ifdef CONFIG_USB_MAUSB
void mausb_bind(void);
void mausb_unbind(void);
void check_mausb_enable(void);
void android_mausb_connect(int connect);
#else
static inline void mausb_bind(void) { }
static inline void mausb_ubind(void) { }
#endif

#endif /* __LINUX_MAUSB_H */
