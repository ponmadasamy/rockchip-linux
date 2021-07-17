/*
 * ST Microelectronics MFD: dealcore's i2c client specific driver
 *
 * Copyright (C) ST-Ericsson SA 2010
 * Copyright (C) ST Microelectronics SA 2011
 *
 * License Terms: GNU General Public License, version 2
 * Author: Rabin Vincent <rabin.vincent@stericsson.com> for ST-Ericsson
 * Author: Viresh Kumar <vireshk@kernel.org> for ST Microelectronics
 */

#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/of_device.h>
#include "dealcore.h"

#ifdef CONFIG_PM
static int dealcore_suspend(struct device *dev)
{
	printk("DEALORE SUSPEND\n");

	return 0;
}

static int dealcore_resume(struct device *dev)
{
	printk("DEALORE RESUME\n");

	return 0;
}

const struct dev_pm_ops dealcore_dev_pm_ops = {
	.suspend	= dealcore_suspend,
	.resume		= dealcore_resume,
};
#endif

static const struct of_device_id dealcore_of_match[] = {
	{ .compatible = "duragon,mpu", .data = (void *)MPU, },
	{ .compatible = "duragon,mpu-v2", .data = (void *)MPU_V2, },
	{},
};
MODULE_DEVICE_TABLE(of, dealcore_of_match);

static int dealcore_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	printk("DEALCORE PROBE\n");
	return (0);
}

static int dealcore_i2c_remove(struct i2c_client *i2c)
{
	printk("DEALCORE REMOVE\n");
	return (0);
}

static const struct i2c_device_id dealcore_i2c_id[] = {
	{ "mpu", MPU },
	{ "mpu-v2", MPU_V2 },
	{ "drmc-32", DRMC_32 },
	{ "drmc-28", DRMC_28 },
	{ "drmc-48", DRMC_48 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, dealcore_i2c_id);

static struct i2c_driver dealcore_i2c_driver = {
	.driver = {
		.name = "dealcore-i2c",
#ifdef CONFIG_PM
		.pm = &dealcore_dev_pm_ops,
#endif
		.of_match_table = dealcore_of_match,
	},
	.probe		= dealcore_i2c_probe,
	.remove		= dealcore_i2c_remove,
	.id_table	= dealcore_i2c_id,
};

static int __init dealcore_init(void)
{
    printk("DEALCORE INIT\n");
	return i2c_add_driver(&dealcore_i2c_driver);
}
subsys_initcall(dealcore_init);

static void __exit dealcore_exit(void)
{
	printk("DEALCORE EXIT\n");
}
module_exit(dealcore_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("DealCore MFD Driver");
MODULE_AUTHOR("Ponmadasamy Muthuraj <ponmadasamy@live.com>");
