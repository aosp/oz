/**********************************************************************
 *
 * Copyright(c) 2008 Imagination Technologies Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful but, except
 * as otherwise stated in writing, without any warranty; without even the
 * implied warranty of merchantability or fitness for a particular purpose.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 *
 * Contact Information:
 * Imagination Technologies Ltd. <gpl-support@imgtec.com>
 * Home Park Estate, Kings Langley, Herts, WD4 8LZ, UK
 *
 ******************************************************************************/

#ifndef AUTOCONF_INCLUDED
#include <linux/config.h>
#endif

#include <linux/version.h>
#include <linux/fb.h>
#include <plat/dma.h>
#include <mach/tiler.h>

#include <video/dsscomp.h>
#include <plat/dsscomp.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32))
#include <plat/vrfb.h>
#include <plat/display.h>
#else
#include <mach/vrfb.h>
#include <mach/display.h>
#endif

#ifdef RELEASE
#include <../drivers/video/omap2/omapfb/omapfb.h>
#undef DEBUG
#else
#undef DEBUG
#include <../drivers/video/omap2/omapfb/omapfb.h>
#endif

#if defined(CONFIG_OUTER_CACHE)  /* Kernel config option */
#include <asm/cacheflush.h>
#define HOST_PAGESIZE			(4096)
#define HOST_PAGEMASK			(~(HOST_PAGESIZE-1))
#define HOST_PAGEALIGN(addr)	(((addr)+HOST_PAGESIZE-1)&HOST_PAGEMASK)
#endif

#include "img_defs.h"
#include "servicesext.h"
#include "kerneldisplay.h"
#include "omaplfb.h"

static int g_use_dsscomp;

MODULE_SUPPORTED_DEVICE(DEVNAME);

#if defined(CONFIG_OUTER_CACHE)  /* Kernel config option */
#if defined(__arm__)
static void per_cpu_cache_flush_arm(void *arg)
{
    PVR_UNREFERENCED_PARAMETER(arg);
    flush_cache_all();
}
#endif
#endif

/*
 * Kernel malloc
 * in: ui32ByteSize
 */
void *OMAPLFBAllocKernelMem(unsigned long ui32ByteSize)
{
	void *p;

#if defined(CONFIG_OUTER_CACHE)  /* Kernel config option */
	IMG_VOID *pvPageAlignedCPUPAddr;
	IMG_VOID *pvPageAlignedCPUVAddr;
	IMG_UINT32 ui32PageOffset;
	IMG_UINT32 ui32PageCount;
#endif
	p = kmalloc(ui32ByteSize, GFP_KERNEL);

	if(!p)
		return 0;

#if defined(CONFIG_OUTER_CACHE)  /* Kernel config option */
	ui32PageOffset = (IMG_UINT32) p & (HOST_PAGESIZE - 1);
	ui32PageCount = HOST_PAGEALIGN(ui32ByteSize + ui32PageOffset) / HOST_PAGESIZE;

	pvPageAlignedCPUVAddr = (IMG_VOID *)((IMG_UINT8 *)p - ui32PageOffset);
	pvPageAlignedCPUPAddr = (IMG_VOID*) __pa(pvPageAlignedCPUVAddr);

#if defined(__arm__)
      on_each_cpu(per_cpu_cache_flush_arm, NULL, 1);
#endif
	outer_cache.flush_range((unsigned long) pvPageAlignedCPUPAddr, (unsigned long) ((pvPageAlignedCPUPAddr + HOST_PAGESIZE*ui32PageCount) - 1));
#endif
	return p;
}

/*
 * Kernel free
 * in: pvMem
 */
void OMAPLFBFreeKernelMem(void *pvMem)
{
	kfree(pvMem);
}

/*
 * Here we get the function pointer to get jump table from
 * services using an external function.
 * in: szFunctionName
 * out: ppfnFuncTable
 */
OMAP_ERROR OMAPLFBGetLibFuncAddr (char *szFunctionName,
	PFN_DC_GET_PVRJTABLE *ppfnFuncTable)
{
	if(strcmp("PVRGetDisplayClassJTable", szFunctionName) != 0)
	{
		ERROR_PRINTK("Unable to get function pointer for %s"
			" from services", szFunctionName);
		return OMAP_ERROR_INVALID_PARAMS;
	}
	*ppfnFuncTable = PVRGetDisplayClassJTable;

	return OMAP_OK;
}

void set_use_dsscomp(int use)
{
	g_use_dsscomp = use;
}

/* Must be called within framebuffer lock to prevent race conditions */
static dsscomp_t find_dsscomp_obj(struct omap_overlay_manager *manager)
{
	dsscomp_t comp = NULL;
	u32 sync_id;
	if (!manager)
		return NULL;
	sync_id = dsscomp_first_sync_id(manager);
	if (!sync_id)
		return NULL;
	comp = dsscomp_find(manager, sync_id);
	return comp;
}

static void omaplfb_dma_cb(int channel, u16 status, void *data)
{
	struct omaplfb_clone_data *clone_data =
		(struct omaplfb_clone_data *)data;

	if (!(status & OMAP_DMA_BLOCK_IRQ) && (status != 0))
		ERROR_PRINTK("DMA transfer failed, channel %d status %u",
			channel, status);
	clone_data->dma_transfer_done = 1;
	wake_up_interruptible(&clone_data->dma_waitq);
}

static int omaplfb_transfer_buf(OMAPLFB_DEVINFO *display_info,
	unsigned long src_addr, unsigned long dst_addr,
	unsigned long buf_size)
{
	int err;
	struct omaplfb_clone_data *clone_data = display_info->clone_data;

	/* DMA parameters */
	int channel, device_id, sync_mode, data_type, elem_in_frame, frame_num;
	int src_ei, src_fi, src_addr_mode, dst_ei, dst_fi, dst_addr_mode;
	int src_burst_mode, dst_burst_mode;

	clone_data->dma_transfer_done = 0;
	device_id = OMAP_DMA_NO_DEVICE;
	sync_mode = OMAP_DMA_SYNC_ELEMENT;
	data_type = OMAP_DMA_DATA_TYPE_S32;
	elem_in_frame = buf_size / 4; /* Divided by 4 because type is S32 */
	frame_num = 1; /* Destination buffer is Tiler1D, one shot transfer */

	err = omap_request_dma(device_id, "pvr_uiclone_dma", omaplfb_dma_cb,
		clone_data, &channel);

	if (err) {
		ERROR_PRINTK("Unable to get an available DMA channel");
		goto transfer_done;
	}

	omap_set_dma_transfer_params(channel, data_type, elem_in_frame,
		frame_num, sync_mode, device_id, 0x0);

	/* Source buffer parameters */

	/* Framebuffer and Tiler1D are physically contiguous, EI and FI are
	 * equal on both source and destination, addressing and burst modes too
	 * as well
	 */
	src_ei = src_fi = dst_ei = dst_fi = 1;
	src_addr_mode = dst_addr_mode = OMAP_DMA_AMODE_POST_INC;
	src_burst_mode = dst_burst_mode = OMAP_DMA_DATA_BURST_16;

	omap_set_dma_src_params(channel, 0, src_addr_mode, src_addr,
		src_ei, src_fi);
	omap_set_dma_src_data_pack(channel, 1);
	omap_set_dma_src_burst_mode(channel, src_burst_mode);

	omap_set_dma_dest_params(channel, 0, dst_addr_mode, dst_addr,
		dst_ei, dst_fi);
	omap_set_dma_dest_data_pack(channel, 1);
	omap_set_dma_dest_burst_mode(channel, dst_burst_mode);

	/* Transfer as soon as possible, high priority */
	omap_dma_set_prio_lch(channel, DMA_CH_PRIO_HIGH, DMA_CH_PRIO_HIGH);
	omap_dma_set_global_params(DMA_DEFAULT_ARB_RATE, 0xFF, 0);

	omap_start_dma(channel);

	/* Wait until the callback changes the status of the transfer */
	wait_event_interruptible_timeout(clone_data->dma_waitq,
		clone_data->dma_transfer_done, msecs_to_jiffies(30));

	omap_stop_dma(channel);
	omap_free_dma(channel);
transfer_done:
	return err;
}

static void omaplfb_clone_handler(struct work_struct *work)
{
	struct omaplfb_clone_work *clone_work =
		(struct omaplfb_clone_work *) work;
	OMAPLFB_DEVINFO *display_info = clone_work->display_info;
	struct omaplfb_clone_data *clone_data = display_info->clone_data;
	dsscomp_t comp = clone_work->comp;
	struct dss2_ovl_info dss2_ovl;
	int err;
	int dst_buff_idx;
	u32 dst_paddr;

	dst_buff_idx = (clone_data->active_buf_idx + 1) % clone_data->buff_num;

	dst_paddr = clone_data->buffs[dst_buff_idx];
	/* DMA transfer begins here, buffer size is already page aligned */
	err = omaplfb_transfer_buf(display_info, clone_work->src_buf_addr,
		dst_paddr, display_info->sSystemBuffer.ulBufferSize);

	/* Unlock any client waiting for the transfer to be done */
	clone_work->transfer_active = 0;
	wake_up_interruptible(&clone_data->transfer_waitq);

	/* If the transfer was not successful don't switch from the
	 * active buffer, we need to consume the composition anyway
	 */
	if (err)
		dst_paddr = clone_data->buffs[clone_data->active_buf_idx];
	else
		clone_data->active_buf_idx = dst_buff_idx;

	/* Apply the composition */
	err = dsscomp_get_first_ovl(comp, &dss2_ovl);
	if (err)
		ERROR_PRINTK("Overlay not found in comp %p", comp);
	else {
		dss2_ovl.ba = dst_paddr;
		dsscomp_set_ovl(comp, &dss2_ovl);
	}

	if (dsscomp_apply(comp))
		ERROR_PRINTK("DSSComp apply failed %p", comp);
}

/*
 * Presents the flip in the display with the DSSComp API
 */
static void OMAPLFBFlipDSSComp(OMAPLFB_DEVINFO *display_info,
	unsigned long phy_addr, dsscomp_t comp)
{
	struct dss2_ovl_info dss2_ovl;
	struct omaplfb_clone_data *clone_data = NULL;
	struct omaplfb_clone_work *work = NULL;
	dsscomp_t clone_comp;
	int queued_clone_work = 0;
	int mgr_id_dst;
	int r = 0;

	mutex_lock(&display_info->clone_lock);

	if (!display_info->cloning_enabled)
		goto clone_unlock;

	/* Look for compositions in the manager to clone */
	clone_data = display_info->clone_data;
	mgr_id_dst = clone_data->mgr_id_dst;
	clone_comp = find_dsscomp_obj(omap_dss_get_overlay_manager(mgr_id_dst));

	if (!clone_comp)
		goto clone_unlock;

	work = &clone_data->work;
	INIT_WORK((struct work_struct *)work, omaplfb_clone_handler);
	work->display_info = display_info;
	work->src_buf_addr = phy_addr;
	work->comp = clone_comp;
	work->transfer_active = 1;
	r = queue_work(clone_data->workqueue, (struct work_struct *)work);
	/* If queueing the work failed we need to consume the
	 * composition
	 */
	if (r == 0) {
		DEBUG_PRINTK("Failed to queue cloning work, "
			"droppping comp %p error %d", clone_comp, r);
		dsscomp_drop(clone_comp);
	} else
		queued_clone_work = 1;

clone_unlock:
	mutex_unlock(&display_info->clone_lock);

	r = dsscomp_get_first_ovl(comp, &dss2_ovl);

	if (r)
		ERROR_PRINTK("Overlay not found in comp %p", comp);
	else {
		dss2_ovl.ba = phy_addr;
		dsscomp_set_ovl(comp, &dss2_ovl);
	}

	if (dsscomp_apply(comp))
		ERROR_PRINTK("DSSComp apply failed %p", comp);

	/* We need to wait here until the transfer to the buffer used for
	 * cloning ends
	 */
	if (queued_clone_work)
		wait_event_interruptible_timeout(clone_data->transfer_waitq,
			!work->transfer_active, msecs_to_jiffies(25));
}

#if defined(FLIP_TECHNIQUE_FRAMEBUFFER)
/*
 * Presents the flip in the display with the framebuffer API
 * in: psSwapChain, aPhyAddr
 */
static void OMAPLFBFlipDSS(OMAPLFB_SWAPCHAIN *psSwapChain,
	unsigned long aPhyAddr)
{
	OMAPLFB_DEVINFO *psDevInfo = (OMAPLFB_DEVINFO *)psSwapChain->pvDevInfo;
	struct fb_info *framebuffer = psDevInfo->psLINFBInfo;

	/* Get the framebuffer physical address base */
	unsigned long fb_base_phyaddr =
		psDevInfo->sSystemBuffer.sSysAddr.uiAddr;

	/* Calculate the virtual Y to move in the framebuffer */
	framebuffer->var.yoffset =
		(aPhyAddr - fb_base_phyaddr) / framebuffer->fix.line_length;
	framebuffer->var.activate = FB_ACTIVATE_FORCE;
	fb_set_var(framebuffer, &framebuffer->var);
}

#elif defined(FLIP_TECHNIQUE_OVERLAY)

/*
 * Presents the flip in the display with the DSS2 overlay API
 * in: psSwapChain, aPhyAddr
 */
static void OMAPLFBFlipDSS(OMAPLFB_SWAPCHAIN *psSwapChain,
	unsigned long aPhyAddr)
{
	OMAPLFB_DEVINFO *psDevInfo = (OMAPLFB_DEVINFO *)psSwapChain->pvDevInfo;
	struct fb_info * framebuffer = psDevInfo->psLINFBInfo;
	struct omapfb_info *ofbi = FB2OFB(framebuffer);
	unsigned long fb_offset;
	int i;

	fb_offset = aPhyAddr - psDevInfo->sSystemBuffer.sSysAddr.uiAddr;

	for(i = 0; i < ofbi->num_overlays ; i++)
	{
		struct omap_dss_device *display = NULL;
		struct omap_dss_driver *driver = NULL;
		struct omap_overlay_manager *manager;
		struct omap_overlay *overlay;
		struct omap_overlay_info overlay_info;

		overlay = ofbi->overlays[i];
		manager = overlay->manager;
		overlay->get_overlay_info( overlay, &overlay_info );

		overlay_info.paddr = framebuffer->fix.smem_start + fb_offset;
		overlay_info.vaddr = framebuffer->screen_base + fb_offset;
		overlay->set_overlay_info(overlay, &overlay_info);

		if (manager) {
			display = manager->device;
			/* No display attached to this overlay, don't update */
			if (!display)
				continue;
			driver = display->driver;
			manager->apply(manager);
		}

		if (dss_ovl_manually_updated(overlay)) {
			if (driver->sched_update)
				driver->sched_update(display, 0, 0,
					overlay_info.width,
					overlay_info.height);
			else if (driver->update)
				driver->update(display, 0, 0,
					overlay_info.width,
					overlay_info.height);
		}
	}
}

#else
#error No flipping technique selected, please define \
	FLIP_TECHNIQUE_FRAMEBUFFER or FLIP_TECHNIQUE_OVERLAY
#endif

void OMAPLFBFlip(OMAPLFB_SWAPCHAIN *psSwapChain, unsigned long aPhyAddr)
{
	OMAPLFB_DEVINFO *psDevInfo = (OMAPLFB_DEVINFO *)psSwapChain->pvDevInfo;
	struct fb_info *framebuffer = psDevInfo->psLINFBInfo;
	struct omapfb_info *ofbi = FB2OFB(framebuffer);
	struct omapfb2_device *fbdev = ofbi->fbdev;
	struct omap_overlay_manager *manager;
	dsscomp_t comp = NULL;

	omapfb_lock(fbdev);

	/* Always get the first overlay for main LCD */
	if (ofbi->num_overlays > 0) {
		struct omap_overlay *overlay;
		overlay = ofbi->overlays[0];
		manager = overlay->manager;
		comp = find_dsscomp_obj(manager);
	}

	if (comp)
		OMAPLFBFlipDSSComp(psDevInfo, aPhyAddr, comp);
	else
		OMAPLFBFlipDSS(psSwapChain, aPhyAddr);

	omapfb_unlock(fbdev);
}

/*
 * Present frame and synchronize with the display to prevent tearing
 * On DSI panels the sync function is used to handle FRAMEDONE IRQ
 * On DPI panels the wait_for_vsync is used to handle VSYNC IRQ
 * in: psDevInfo
 */
void OMAPLFBPresentSync(OMAPLFB_DEVINFO *psDevInfo,
	OMAPLFB_FLIP_ITEM *psFlipItem)
{
	struct fb_info *framebuffer = psDevInfo->psLINFBInfo;
	struct omapfb_info *ofbi = FB2OFB(framebuffer);
	struct omapfb2_device *fbdev = ofbi->fbdev;
	struct omap_overlay_manager *manager;
	dsscomp_t comp = NULL;
	unsigned long aPhyAddr = (unsigned long)psFlipItem->sSysAddr->uiAddr;
	struct omap_dss_device *display;
	struct omap_dss_driver *driver;
	int err = 1;

	omapfb_lock(fbdev);

	/* DSSComp will do the flip */
	if (g_use_dsscomp) {
		/* Always get the first overlay for main display */
		if (ofbi->num_overlays > 0) {
			struct omap_overlay *overlay;
			overlay = ofbi->overlays[0];
			manager = overlay->manager;
			comp = find_dsscomp_obj(manager);
		}
		if (comp)
			OMAPLFBFlipDSSComp(psDevInfo, aPhyAddr, comp);
		goto exit_unlock;
	}

	/* OMAPLFB will do the flip */
	display = fb2display(framebuffer);
	/* The framebuffer doesn't have a display attached, just bail out */
	if (!display)
		goto exit_unlock;

	driver = display->driver;
	manager = display->manager;

	if (driver && driver->sync &&
		driver->get_update_mode(display) == OMAP_DSS_UPDATE_MANUAL) {
		/* Wait first for the DSI bus to be released then update */
		err = driver->sync(display);
		OMAPLFBFlipDSS(psDevInfo->psSwapChain, aPhyAddr);
	} else if (manager && manager->wait_for_vsync) {
		/*
		 * Update the video pipelines registers then wait until the
		 * frame is shown with a VSYNC
		 */
		OMAPLFBFlipDSS(psDevInfo->psSwapChain, aPhyAddr);
		err = manager->wait_for_vsync(manager);
	}

	if (err)
		DEBUG_PRINTK("Unable to sync with display %u!",
			psDevInfo->uDeviceID);

exit_unlock:
	omapfb_unlock(fbdev);
}

static int omaplfb_alloc_buf_cloning(OMAPLFB_DEVINFO *display_info)
{
	struct omaplfb_clone_data *clone_data = display_info->clone_data;
	int i, j, err;
	/* We already know the buffer size for each display is page aligned */
	unsigned long buff_size = display_info->sSystemBuffer.ulBufferSize;

	if (clone_data->buff_num < 0 ||
		clone_data->buff_num > OMAPLFB_CLONING_BUFFER_NUM)
		return -EINVAL;

	for (i = 0; i < clone_data->buff_num; i++) {
		/* Allocate Tiler1D buffer */
		err = tiler_alloc(TILFMT_PAGE, buff_size, 1,
			(u32 *) &clone_data->buffs[i]);
		if (err) {
			ERROR_PRINTK("Cloning buffer allocation failed %d",
				err);
			goto exit_free;
		}
		DEBUG_PRINTK("Cloning buffer allocated %p",
			(u32 *) clone_data->buffs[i]);
	}

	return 0;

exit_free:
	for (j = 0; j < i; j++)
		tiler_free(clone_data->buffs[j]);
	return err;
}

static int omaplfb_free_buf_cloning(OMAPLFB_DEVINFO *display_info)
{
	struct omaplfb_clone_data *clone_data = display_info->clone_data;
	int i, res = 0, err = 0;
	for (i = 0; i < clone_data->buff_num; i++) {
		DEBUG_PRINTK("Freeing cloning buffer %p",
			(u32 *)clone_data->buffs[i]);
		err = tiler_free(clone_data->buffs[i]);
		if (err) {
			ERROR_PRINTK("Unable to free cloning buffer %p",
				(u32 *)clone_data->buffs[i]);
			res = 1;
		}
	}
	clone_data->buff_num = 0;
	return res;
}

int omaplfb_enable_cloning(int mgr_id_src, int mgr_id_dst, int buff_num)
{
	int err = 0;
	OMAPLFB_DEVINFO *display_info = omaplfb_get_devinfo(mgr_id_src);
	struct omaplfb_clone_data *clone_data = NULL;

	if (!display_info)
		return -EINVAL;

	mutex_lock(&display_info->clone_lock);

	if (mgr_id_src == mgr_id_dst ||
		buff_num <= 0 || buff_num > OMAPLFB_CLONING_BUFFER_NUM) {
		err = -EINVAL;
		goto exit_unlock;
	}

	if (display_info->cloning_enabled) {
		err = -EBUSY;
		goto exit_unlock;
	}

	clone_data = kzalloc(sizeof(*clone_data), GFP_KERNEL);
	if (!clone_data) {
		err = -ENOMEM;
		goto exit_unlock;
	}

	display_info->clone_data = clone_data;

	clone_data->workqueue = create_singlethread_workqueue("pvr_wq_clone");
	if (!clone_data->workqueue) {
		WARNING_PRINTK("Unable to create workqueue for UI cloning");
		err = -EBUSY;
		goto exit_unlock;
	}

	clone_data->mgr_id_src = mgr_id_src;
	clone_data->mgr_id_dst = mgr_id_dst;
	clone_data->buff_num = buff_num;
	clone_data->active_buf_idx = 0;
	init_waitqueue_head(&clone_data->transfer_waitq);
	init_waitqueue_head(&clone_data->dma_waitq);

	if (omaplfb_alloc_buf_cloning(display_info)) {
		WARNING_PRINTK("Unable to allocate buffers for UI cloning");
		err = -ENOMEM;
		goto alloc_failed;
	}

	display_info->cloning_enabled = 1;
	DEBUG_PRINTK("Cloning enabled for display %d to %d",
		clone_data->mgr_id_src, clone_data->mgr_id_dst);

	goto exit_unlock;

alloc_failed:
	destroy_workqueue(clone_data->workqueue);
	kfree(display_info->clone_data);
	display_info->clone_data = NULL;
exit_unlock:
	mutex_unlock(&display_info->clone_lock);
	return err;
}

static int omaplfb_disable_cloning_disp(OMAPLFB_DEVINFO *display_info)
{
	struct omaplfb_clone_data *clone_data = display_info->clone_data;
	int err = 0;
	dsscomp_t comp;

	mutex_lock(&display_info->clone_lock);

	if (!display_info->cloning_enabled) {
		err = 0;
		goto exit_unlock;
	}

	err = omaplfb_free_buf_cloning(display_info);

	/* Wait for any pending DMA transfers and apply calls to the
	 * destination manager
	 */
	flush_workqueue(clone_data->workqueue);
	destroy_workqueue(clone_data->workqueue);

	/* Drop all pending compositions from the destination manager */
	do {
		comp = find_dsscomp_obj(
			omap_dss_get_overlay_manager(clone_data->mgr_id_dst));
		if (comp)
			dsscomp_drop(comp);
		else
			break;
	} while (true);

	display_info->cloning_enabled = 0;
	kfree(display_info->clone_data);
	display_info->clone_data = NULL;
	DEBUG_PRINTK("Cloning disabled for display %d", clone_data->mgr_id_src);

exit_unlock:
	mutex_unlock(&display_info->clone_lock);
	return err;
}

int omaplfb_disable_cloning(int mgr_id_src)
{
	OMAPLFB_DEVINFO *display_info = omaplfb_get_devinfo(mgr_id_src);
	if (!display_info)
		return -EINVAL;
	return omaplfb_disable_cloning_disp(display_info);
}

void omaplfb_disable_cloning_alldisp(void)
{
	OMAPLFB_DEVINFO *display_info;
	int i;
	for (i = 0; i < FRAMEBUFFER_COUNT; i++) {
		display_info = omaplfb_get_devinfo(i);
		if (display_info && display_info->cloning_enabled)
			omaplfb_disable_cloning_disp(display_info);
	}
}
