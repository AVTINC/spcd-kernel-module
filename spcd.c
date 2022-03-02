/*
    SPCD Kernel Driver
    Copyright (C) 2022 AVT, Inc.

    info@avtcare.com
    1125 N 13th St.
    Lafayette, IN 47904

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  US
*/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>

static int spcd_probe(struct platform_device *pdev) {
	pr_info("SPCD probe\n");

	return 0;
};

static int spcd_remove(struct platform_device *pdev) {
	pr_info("SPCD remove\n");

	return 0;
};

static struct platform_driver spcd_driver = {
	.probe = spcd_probe,
	.remove = spcd_remove,
	.driver = {
		.name = "spcd",
		.owner = THIS_MODULE,
	},
};
module_platform_driver(spcd_driver);

MODULE_AUTHOR("bryan.varner@e-gineering.com");
MODULE_DESCRIPTION("AVT SPCD Platform Driver.");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:spcd");
