/*
 * Copyright (c) 2022, Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/cache.h>
#include <linux/mod_devicetable.h>
#include <linux/debugfs.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include <asm/cacheflush.h>

#include <crypto/md5.h>
#include <crypto/sha.h>
#include <crypto/sha3.h>
#include <crypto/aes.h>
#include <crypto/des.h>

#include "eip_priv.h"

struct eip_drv eip_drv_g;     /* Global Driver object */

static const struct of_device_id eip_dev_dt_id = { .compatible =  "qcom,eip" };

/*
 * eip_device_probe()
 *	Device probe to initialize HW engine & DMA.
 */
static int eip_device_probe(struct platform_device *pdev)
{
	struct device_node *np = of_node_get(pdev->dev.of_node);
	struct eip_drv *drv = &eip_drv_g;
	struct resource res = {0};
	struct eip_pdev *ep;
	uint32_t reg_offset;
	void __iomem *vaddr;
	phys_addr_t paddr;
	int err;

	if (of_property_read_u32(np, "reg_offset", &reg_offset) < 0) {
		pr_err("%px: Failed to read register offset\n", pdev);
		return -EINVAL;
	}

	/*
	 * Fetch Registers resource.
	 */
	if (of_address_to_resource(np, 0, &res) != 0) {
		pr_err("%px: Failed to read register resource\n", pdev);
		return -EINVAL;
	}

	/*
	 * remap the I/O addresses
	 */
	paddr = res.start + reg_offset;
	vaddr = ioremap_nocache(paddr, resource_size(&res));
	if (!vaddr) {
		pr_warn("%px: unable to remap crypto_addr(0x%px)\n", pdev, (void *)paddr);
		return -EIO;
	}

	/*
	 * Allocate EIP197 device node.
	 */
	ep = kzalloc(sizeof(struct eip_pdev), GFP_KERNEL);
	if (!ep) {
		pr_err("%px: Failed to allocate device node\n", pdev);
		err = -ENOMEM;
		goto fail_alloc;
	}

	/*
	 * Initialize the tr cache.
	 */
	ep->tr_cache = kmem_cache_create("eip_tr", EIP_TR_SIZE, 0,SLAB_HWCACHE_ALIGN, NULL);
	if (!ep->tr_cache) {
		pr_err("%px: Failed to allocate tr kmem cache\n", drv);
		err = -ENOMEM;
		goto fail_cache;
	}

	/*
	 * Create debugfs directory
	 * eip_hw_init() creates per ring file inside this.
	 */
	ep->dentry = debugfs_create_dir("eip197", drv->dentry);
	if (IS_ERR_OR_NULL(ep->dentry)) {
		pr_err("%px: Failed to create debugfs directory\n", pdev);
		err = -EBADF;
		goto fail_debugfs;
	}

	ep->dev_paddr = paddr;
	ep->dev_vaddr = vaddr;
	ep->pdev = pdev;

	/*
	 * Set platform data.
	 */
	platform_set_drvdata(pdev, ep);

	/*
	 * Initialize HW engine & DMA.
	 */
	err = eip_hw_init(pdev);
	if (err < 0) {
		pr_err("%px: Failed to initialize HW, err(%u)\n", pdev, err);
		goto fail_hw_init;
	}


	drv->pdev = pdev;
	return 0;

fail_hw_init:
	platform_set_drvdata(pdev, NULL);
	debugfs_remove_recursive(ep->dentry);
fail_debugfs:
	kmem_cache_destroy(ep->tr_cache);
fail_cache:
	kfree(ep);
fail_alloc:
	iounmap(vaddr);
	return err;
}

/*
 * eip_device_remove()
 *	Remove Device and free all linked resources.
 */
static int eip_device_remove(struct platform_device *pdev)
{
	struct eip_drv *drv = &eip_drv_g;
	struct eip_pdev *ep;

	/*
	 * Unset pdev in global object.
	 */
	drv->pdev = NULL;
	ep = platform_get_drvdata(pdev);

	/*
	 * Deinitialize device.
	 * TODO: We need to wait for all inflight transformation.
	 */
	eip_hw_deinit(pdev);
	debugfs_remove_recursive(ep->dentry);
	kmem_cache_destroy(ep->tr_cache);
	iounmap(ep->dev_vaddr);

	/*
	 * Free and unset private pointer in pdev to avoid any futher access.
	 */
	kfree(ep);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

/*
 * eip_device
 *	platform device instance
 */
static struct platform_driver eip_device = {
	.probe          = eip_device_probe,
	.remove         = eip_device_remove,
	.driver         = {
		.owner  = THIS_MODULE,
		.name   = "nss-eip-device",
		.of_match_table = of_match_ptr(&eip_dev_dt_id),
	}
};

/*
 * eip_init_module()
 *	Initialize all the resources.
 */
int __init eip_init_module(void)
{
	struct eip_drv *drv = &eip_drv_g;

	/*
	 * Check if crypto is enabled.
	 */
	if (!eip_is_enabled()) {
		pr_info("Crypto hardware is unavailable\n");
		return 0;
	}

	drv->dentry = debugfs_create_dir("qca-nss-eip", NULL);
	if (IS_ERR_OR_NULL(drv->dentry)) {
		pr_err("%px: Failed to create debugfs directory\n", drv);
		return -EBADF;
	}

	/*
	 * Register platform driver.
	 */
	if (platform_driver_register(&eip_device) < 0) {
		pr_err("%px: Failed to register driver\n", drv);
		debugfs_remove_recursive(drv->dentry);
		return -ESHUTDOWN;
	}

	return 0;
}

/*
 * eip_exit_module()
 *	Cleanup during module exit.
 */
void __exit eip_exit_module(void)
{
	struct eip_drv *drv = &eip_drv_g;
	platform_driver_unregister(&eip_device);
	debugfs_remove_recursive(drv->dentry);
}

MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("EIP197 driver");

module_init(eip_init_module);
module_exit(eip_exit_module);
