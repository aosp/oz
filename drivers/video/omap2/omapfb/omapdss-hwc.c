/*
 * linux/drivers/video/omap2/omapdss-hwc.c
 *
 * OMAPDSS HWC support implementation
 *
 * Copyright (C) 2011 Texas Instruments, Inc
 * Author: Lajos Molnar <molnar@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/fb.h>
#include <linux/mm.h>

#include <mach/tiler.h>
#include <plat/display.h>
#include <plat/vrfb.h>

#include "omapfb.h"

/*
#undef DBG
#define DBG(fmt, ...) printk(KERN_ERR fmt, ##__VA_ARGS__)
*/

/* color formats supported - bitfield info is used for truncation logic */
static const struct color_info {
	int a_ix, a_bt;	/* bitfields */
	int r_ix, r_bt;
	int g_ix, g_bt;
	int b_ix, b_bt;
	int x_bt;
	enum omap_color_mode mode;
	const char *name;
} fmts[2][16] = { {
	{ 0,  0, 0,  0, 0,  0, 0, 0, 1, OMAP_DSS_COLOR_CLUT1, "BITMAP1" },
	{ 0,  0, 0,  0, 0,  0, 0, 0, 2, OMAP_DSS_COLOR_CLUT2, "BITMAP2" },
	{ 0,  0, 0,  0, 0,  0, 0, 0, 4, OMAP_DSS_COLOR_CLUT4, "BITMAP4" },
	{ 0,  0, 0,  0, 0,  0, 0, 0, 8, OMAP_DSS_COLOR_CLUT8, "BITMAP8" },
	{ 0,  0, 8,  4, 4,  4, 0, 4, 4, OMAP_DSS_COLOR_RGB12U, "xRGB12-4444" },
	{ 12, 4, 8,  4, 4,  4, 0, 4, 0, OMAP_DSS_COLOR_ARGB16, "ARGB16-4444" },
	{ 0,  0, 11, 5, 5,  6, 0, 5, 0, OMAP_DSS_COLOR_RGB16, "RGB16-565" },
	{ 15, 1, 10, 5, 5,  5, 0, 5, 0, OMAP_DSS_COLOR_ARGB16_1555,
								"ARGB16-1555" },
	{ 0,  0, 16, 8, 8,  8, 0, 8, 8, OMAP_DSS_COLOR_RGB24U, "xRGB24-8888" },
	{ 0,  0, 16, 8, 8,  8, 0, 8, 0, OMAP_DSS_COLOR_RGB24P, "RGB24-888" },
	{ 0,  0, 12, 4, 8,  4, 4, 4, 4, OMAP_DSS_COLOR_RGBX12, "RGBx12-4444" },
	{ 0,  4, 12, 4, 8,  4, 4, 4, 0, OMAP_DSS_COLOR_RGBA16, "RGBA16-4444" },
	{ 24, 8, 16, 8, 8,  8, 0, 8, 0, OMAP_DSS_COLOR_ARGB32, "ARGB32-8888" },
	{ 0,  8, 24, 8, 16, 8, 8, 8, 0, OMAP_DSS_COLOR_RGBA32, "RGBA32-8888" },
	{ 0,  0, 24, 8, 16, 8, 8, 8, 8, OMAP_DSS_COLOR_RGBX24, "RGBx24-8888" },
	{ 0,  0, 10, 5, 5,  5, 0, 5, 1, OMAP_DSS_COLOR_XRGB15, "xRGB15-1555" },
}, {
	{ 0,  0, 0,  0, 0,  0, 0, 0, 12, OMAP_DSS_COLOR_NV12, "NV12" },
	{ 0,  0, 12, 4, 8,  4, 4, 4, 4, OMAP_DSS_COLOR_RGBX12, "RGBx12-4444" },
	{ 0,  4, 12, 4, 8,  4, 4, 4, 0, OMAP_DSS_COLOR_RGBA16, "RGBA16-4444" },
	{ 0,  0, 0,  0, 0,  0, 0, 0, 0, 0, "invalid" },
	{ 0,  0, 8,  4, 4,  4, 0, 4, 4, OMAP_DSS_COLOR_RGB12U, "xRGB12-4444" },
	{ 12, 4, 8,  4, 4,  4, 0, 4, 0, OMAP_DSS_COLOR_ARGB16, "ARGB16-4444" },
	{ 0,  0, 11, 5, 5,  6, 0, 5, 0, OMAP_DSS_COLOR_RGB16, "RGB16-565" },
	{ 15, 1, 10, 5, 5,  5, 0, 5, 0, OMAP_DSS_COLOR_ARGB16_1555,
								"ARGB16-1555" },
	{ 0,  0, 16, 8, 8,  8, 0, 8, 8, OMAP_DSS_COLOR_RGB24U, "xRGB24-8888" },
	{ 0,  0, 16, 8, 8,  8, 0, 8, 0, OMAP_DSS_COLOR_RGB24P, "RGB24-888" },
	{ 0,  0, 0,  0, 0,  0, 0, 0, 16, OMAP_DSS_COLOR_YUV2, "YUYV" },
	{ 0,  0, 0,  0, 0,  0, 0, 0, 16, OMAP_DSS_COLOR_UYVY, "UYVY" },
	{ 24, 8, 16, 8, 8,  8, 0, 8, 0, OMAP_DSS_COLOR_ARGB32, "ARGB32-8888" },
	{ 0,  8, 24, 8, 16, 8, 8, 8, 0, OMAP_DSS_COLOR_RGBA32, "RGBA32-8888" },
	{ 0,  0, 24, 8, 16, 8, 8, 8, 8, OMAP_DSS_COLOR_RGBX24, "RGBx24-8888" },
	{ 0,  0, 10, 5, 5,  5, 0, 5, 1, OMAP_DSS_COLOR_XRGB15, "xRGB15-1555" },
} };

static const struct color_info *get_color_info(enum omap_color_mode mode)
{
	int i;
	for (i = 0; i < sizeof(fmts) / sizeof(fmts[0][0]); i++)
		if (fmts[0][i].mode == mode)
			return fmts[0] + i;
	return NULL;
}

int omapdss_hwc_set(struct dss_hwc_set_info *set)
{
	int i, r;
	struct omap_overlay_manager *mgr;
	struct omap_overlay *ovl;

	DBG("mgr(dis%d alpha=%d col=%08x ilace=%d)\n", set->mgr.ix,
		set->mgr.alpha_blending, set->mgr.default_color,
		set->mgr.interlaced);
#ifdef DEBUG
	for (i = 0; i < set->num_ovls; i++) {
		struct dss_hwc_ovl_info *o = set->ovls + i;
		const struct color_info *ci = get_color_info(o->color_mode);
		DBG("ovl%d(%s z%d %s%s *%d%% %d*%d:%d,%d+%d,%d rot%d%s => "
		    "%d,%d+%d,%d %x/%x|%d)\n",
			o->ix, o->enabled ? "ON" : "off", o->zorder,
			ci->name ? : "(none)",
			o->pre_mult_alpha ? " premult" : "",
			(o->global_alpha * 100 + 128) / 255,
			o->width, o->height, o->crop_x, o->crop_y,
			o->crop_w, o->crop_h,
			o->rotation, o->mirror ? "+mir" : "",
			o->pos_x, o->pos_y, o->out_width, o->out_height,
			o->handle, o->module, o->stride);
	}
#endif
	/* get manager */
	mgr = find_dss_mgr(set->mgr.ix);
	if (!mgr)
		return -EINVAL;

	/* set overlays' manager & info */
	for (i = 0; i < set->num_ovls; i++) {
		ovl = omap_dss_get_overlay(set->ovls[i].ix);
		if (!ovl)
			return -EINVAL;
		ovl->manager = mgr;
		r = set_dss_ovl_info(set->ovls + i);
		if (r)
			return r;
	}

	/* set manager's info */
	r = set_dss_mgr_info(&set->mgr);
	if (r)
		return r;

	/* call update */
	if (set->update) {
		struct omap_dss_device *dev = mgr->device;
		mgr->apply(mgr);

		if (dev && dev->driver &&
		    dssdev_manually_updated(dev)) {
			/* we only keep the last frame on manual-upd panels */
			if (!set->w)
				set->w = dev->panel.timings.x_res - set->x;
			if (!set->h)
				set->h = dev->panel.timings.y_res - set->y;

			/* sync to prevent frame loss */
			dev->driver->sync(dev);

			/* schedule update if supported to avoid delay */
			if (dev->driver->sched_update)
				return dev->driver->sched_update(dev, 0, 0,
					dev->panel.timings.x_res,
					dev->panel.timings.y_res);
			else if (dev->driver->update)
				return dev->driver->update(dev, 0, 0,
					dev->panel.timings.x_res,
					dev->panel.timings.y_res);
		}
	}
	return 0;
}

static u32 hwc_virt_to_phys(u32 arg)
{
	pmd_t *pmd;
	pte_t *ptep;

	pgd_t *pgd = pgd_offset(current->mm, arg);
	if (pgd_none(*pgd) || pgd_bad(*pgd))
		return 0;

	pmd = pmd_offset(pgd, arg);
	if (pmd_none(*pmd) || pmd_bad(*pmd))
		return 0;

	ptep = pte_offset_map(pmd, arg);
	if (ptep && pte_present(*ptep))
		return (PAGE_MASK & *ptep) | (~PAGE_MASK & arg);

	return 0;
}

static int color_mode_to_bpp(enum omap_color_mode color_mode)
{
	const struct color_info *ci = get_color_info(color_mode);
	BUG_ON(!ci);

	return ci->a_bt + ci->r_bt + ci->g_bt + ci->b_bt + ci->x_bt;
}

int set_dss_ovl_info(struct dss_hwc_ovl_info *oi)
{
	struct omap_overlay_info info;
	struct omap_overlay *ovl;
	struct crop_win {
		int x, y;
		int w, h;
	} crop;
	int c;

	/* check overlay number */
	if (!oi || oi->ix >= omap_dss_get_num_overlays())
		return -EINVAL;
	ovl = omap_dss_get_overlay(oi->ix);

	/* just in case there are new fields, we get the current info */
	ovl->get_overlay_info(ovl, &info);

	info.enabled = oi->enabled;
	if (!oi->enabled)
		goto done;

	/* copied params */
	info.global_alpha = oi->global_alpha;
	info.pre_mult_alpha = oi->pre_mult_alpha;
	info.zorder = oi->zorder;
	info.rotation = oi->rotation;
	info.mirror = oi->mirror;
	info.color_mode = oi->color_mode;

	/* crop to screen - same logic for X and Y directions */
#define IX(fld, i) (((typeof(fld) *) &fld)[i])

	/* align crop window with display coordinates */
	if (oi->rotation & 1) {
		crop.x = oi->crop_y + oi->crop_h;
		crop.y = oi->crop_x;
		crop.w = -oi->crop_h;
		crop.h = oi->crop_w;
	} else {
		crop.x = oi->crop_x;
		crop.y = oi->crop_y;
		crop.w = oi->crop_w;
		crop.h = oi->crop_h;
	}
	if (oi->rotation & 2) {
		crop.y += crop.h;
		crop.h = -crop.h;
	}
	if ((!oi->mirror) ^ !(oi->rotation & 2)) {
		crop.x += crop.w;
		crop.w = -crop.w;
	}

	for (c = 0; c < 2; c++) {
		/* convert everything to signed int (s32) */
		int scr_size = IX(ovl->manager->device->panel.timings.x_res, c);
		int pos = IX(oi->pos_x, c);
		int size = IX(oi->out_width, c);
		int vis_pos = IX(crop.x, c);
		int vis_size = IX(crop.w, c);

		/* see if complete buffer is outside the screen or it is
		   fully cropped */
		if (pos + size <= 0 || pos >= scr_size || !vis_size) {
			info.enabled = false;
			goto done;
		}

		/* crop left/top */
		if (pos < 0) {
			/* correction term */
			int a = -pos * vis_size / size;
			vis_pos += a;
			vis_size -= a;
			IX(crop.w, c) = vis_size;
			IX(crop.x, c) = vis_pos;
			size += pos;
			pos = 0;
		}
		/* crop right/bottom */
		if (pos + size > scr_size) {
			vis_size = vis_size * (scr_size - pos) / size;
			IX(crop.w, c) = vis_size;
			IX(crop.x, c) = vis_pos;
			size = scr_size - pos;
		}

		/* adjust crop to UV pixel boundaries */
		if (oi->color_mode & (OMAP_DSS_COLOR_NV12 |
		    (c ? 0 : (OMAP_DSS_COLOR_YUV2 | OMAP_DSS_COLOR_UYVY)))) {
			/* keep the output window to avoid trembling edges */
			vis_size += (vis_pos & 1);	/* round down start */
			vis_pos &= ~1;
			vis_size += (vis_size & 1);	/* round up end */

			/*
			 * Buffer is aligned on UV pixel boundaries, so no
			 * worries about extending crop region.
			 */
		}

		IX(info.pos_x, c) = pos;
		IX(info.out_width, c) = size;
		IX(crop.w, c) = vis_size;
		IX(crop.x, c) = vis_pos;
		if (!vis_size || !size) {
			info.enabled = false;
			goto done;
		}
	}
#undef IX

	/* realign crop window to buffer coordinates */
	if (oi->rotation & 2) {
		crop.y += crop.h;
		crop.h = -crop.h;
	}
	if ((!oi->mirror) ^ !(oi->rotation & 2)) {
		crop.x += crop.w;
		crop.w = -crop.w;
	}
	if (oi->rotation & 1) {
		c = crop.x;
		crop.x = crop.y;
		crop.y = c + crop.w;
		/* DISPC uses swapped height/width for 90/270 degrees */
		info.height = crop.h;
		info.width = -crop.w;
	} else {
		info.width = crop.w;
		info.height = crop.h;
	}

	/* calculate addresses and cropping */
	info.paddr = hwc_virt_to_phys(oi->handle);
	info.p_uv_addr = 0;
	info.vaddr = NULL;

	/* check for TILER 2D buffer */
	if (info.paddr >= 0x60000000 && info.paddr < 0x78000000) {
		int bpp = 1 << ((info.paddr >> 27) & 3);
		int offset;

		/* crop to top-left */

		/*
		 * DSS supports YUV422 on 32-bit mode, but its technically
		 * 2 bytes-per-pixel.
		 * Also RGB24-888 is 3 bytes-per-pixel even though no
		 * tiler pixel format matches this.
		 */
		if (oi->color_mode &
				(OMAP_DSS_COLOR_YUV2 | OMAP_DSS_COLOR_UYVY))
			bpp = 2;
		else if (oi->color_mode == OMAP_DSS_COLOR_RGB24P)
			bpp = 3;

		offset = crop.x * bpp + crop.y *
			tiler_stride(tiler_get_natural_addr(
						(void *) info.paddr));
		info.paddr += offset;
		info.rotation_type = OMAP_DSS_ROT_TILER;
		info.screen_width = 0;

		/* for NV12 format also crop NV12 */
		if (info.color_mode == OMAP_DSS_COLOR_NV12)
			info.p_uv_addr = hwc_virt_to_phys(oi->module) + offset;
	} else {
		/* program tiler 1D as SDMA */

		int bpp = color_mode_to_bpp(oi->color_mode);
		info.screen_width = oi->stride * 8 / (bpp == 12 ? 8 : bpp);
		info.paddr += crop.x * (bpp / 8) + crop.y * oi->stride;

		/* for NV12 format also crop NV12 */
		if (info.color_mode == OMAP_DSS_COLOR_NV12) {
			info.p_uv_addr = hwc_virt_to_phys(oi->module) +
				crop.x * (bpp / 8) +
				(crop.y >> 1) * oi->stride;
		}

		/* no rotation on DMA buffer */
		if (oi->rotation & 3 || oi->mirror)
			return -EINVAL;

		info.rotation_type = OMAP_DSS_ROT_DMA;
	}
	info.max_x_decim = 16;
	info.max_y_decim = 2;
	info.min_x_decim = info.min_y_decim = 1;
	info.pic_height = oi->height;

	info.field = 0;
	if (oi->ilace & OMAP_DSS_ILACE_SEQ)
		info.field |= OMAP_FLAG_IBUF;
	if (oi->ilace & OMAP_DSS_ILACE_SWAP)
		info.field |= OMAP_FLAG_ISWAP;
	/*
	 * Ignore OMAP_DSS_ILACE as there is no real support yet for
	 * interlaced interleaved vs progressive buffers
	 */
	if (ovl->manager &&
	    ovl->manager->device &&
	    !strcmp(ovl->manager->device->name, "hdmi") &&
	    is_hdmi_interlaced())
		info.field |= OMAP_FLAG_IDEV;

	info.out_wb = 0;

	/* :TODO: copy color conversion - this needs ovl support */

done:
	DBG("ovl%d: en=%d %x/%x (%dx%d|%d) => (%dx%d) @ (%d,%d) rot=%d mir=%d "
		"col=%x z=%d al=%02x prem=%d pich=%d ilace=%d\n",
		ovl->id, info.enabled, info.paddr, info.p_uv_addr, info.width,
		info.height, info.screen_width, info.out_width, info.out_height,
		info.pos_x, info.pos_y, info.rotation, info.mirror,
		info.color_mode, info.zorder, info.global_alpha,
		info.pre_mult_alpha, info.pic_height, info.field);

	/* set overlay info */
	return ovl->set_overlay_info(ovl, &info);
}
EXPORT_SYMBOL(set_dss_ovl_info);

struct omap_overlay_manager *find_dss_mgr(int display_ix)
{
	struct omap_overlay_manager *mgr;
	char name[32];
	int i;

	sprintf(name, "display%d", display_ix);

	for (i = 0; i < omap_dss_get_num_overlay_managers(); i++) {
		mgr = omap_dss_get_overlay_manager(i);
		if (mgr->device && !strcmp(name, dev_name(&mgr->device->dev)))
			return mgr;
	}
	return NULL;
}
EXPORT_SYMBOL(find_dss_mgr);

int set_dss_mgr_info(struct dss_hwc_mgr_info *mi)
{
	struct omap_overlay_manager_info info;
	struct omap_overlay_manager *mgr;

	if (!mi)
		return -EINVAL;
	mgr = find_dss_mgr(mi->ix);
	if (!mgr)
		return -EINVAL;

	/* just in case there are new fields, we get the current info */
	mgr->get_manager_info(mgr, &info);

	info.alpha_enabled = mi->alpha_blending;
	info.default_color = mi->default_color;
	info.trans_enabled = mi->trans_enabled && !mi->alpha_blending;
	info.trans_key = mi->trans_key;
	info.trans_key_type = mi->trans_key_type;

	return mgr->set_manager_info(mgr, &info);
}
EXPORT_SYMBOL(set_dss_mgr_info);
