/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <asm/arch/sys_proto.h>
#include <asm/errno.h>
#include <asm/io.h>
#include <stdbool.h>

static int check_mmc_autodetect(void)
{
	char *autodetect_str = getenv("mmcautodetect");
	
	printf("%s : %s\n", __FUNCTION__, autodetect_str);
	
	if ((autodetect_str != NULL) &&
		(strcmp(autodetect_str, "yes") == 0)) {
		return 1;
	}

	return 0;
}

/* This should be defined for each board */
__weak int mmc_map_to_kernel_blk(int dev_no)
{
	return dev_no;
}

void board_late_mmc_env_init(void)
{
	
	char cmd[32];
	char mmcblk[32];
	u32 dev_no = mmc_get_env_dev();
	
	printf("%s\n", __FUNCTION__);
	
	if (!check_mmc_autodetect())
	{
		printf("Check MMC autodetect failed\n");
		return;
	}
	printf("Check MMC autodetect success\n");
	setenv_ulong("mmcdev", dev_no);

	/* Set mmcblk env */
	printf("%s : Set mmcblk env\n", __FUNCTION__);
	sprintf(mmcblk, "/dev/mmcblk%dp2 rootwait rw",
		mmc_map_to_kernel_blk(dev_no));
	setenv("mmcroot", mmcblk);

	sprintf(cmd, "mmc dev %d", dev_no);
	printf("%s : run_command : %s\n", __FUNCTION__, cmd);
	run_command(cmd, 0);
	printf("%s : run_command finished\n", __FUNCTION__);
}
