/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2023 MediaTek Inc.
 */

#ifndef _UFS_MEDIATEK_PRIV_
#define _UFS_MEDIATEK_PRIV_

/*
 * MTK PATCH
 * For some devices, the performance is very low when TW buffer is fill full.
 * Mark this in order to optimize it in block layer
 */
#define UFS_DEVICE_QUIRK_WORSE_PERFORMANCE  (1 << 27)

/*
 * MTK PATCH
 * Some UFS need do urgent gc to resovle its performance
 * while the storage used in 80%+
 */
#define UFS_DEVICE_QUIRK_URGENT_GC  (1 << 28)


int ufs_mtk_auto_hibern8_disable(struct ufs_hba *hba);

#endif /* _UFS_MEDIATEK_PRIV_ */
