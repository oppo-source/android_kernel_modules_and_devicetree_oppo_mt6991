/* SPDX-License-Identifier: GPL-2.0 */
/*
 * mtk-afe-fe-dais.h  --  Mediatek afe fe dai operator definition
 *
 * Copyright (c) 2016 MediaTek Inc.
 * Author: Garlic Tseng <garlic.tseng@mediatek.com>
 */

#ifndef _MTK_AFE_FE_DAI_H_
#define _MTK_AFE_FE_DAI_H_

struct snd_soc_dai_ops;
struct mtk_base_afe;
struct mtk_base_afe_memif;
struct mtk_base_irq_data;

int mtk_get_channel_value(void);
int mtk_regmap_update_bits(struct regmap *map, int reg,
		       unsigned int mask,
		       unsigned int val, int shift);
int mtk_regmap_write(struct regmap *map, int reg,
		       unsigned int val);
int mtk_afe_fe_startup(struct snd_pcm_substream *substream,
		       struct snd_soc_dai *dai);
void mtk_afe_fe_shutdown(struct snd_pcm_substream *substream,
			 struct snd_soc_dai *dai);
int mtk_afe_fe_hw_params(struct snd_pcm_substream *substream,
			 struct snd_pcm_hw_params *params,
			 struct snd_soc_dai *dai);
int mtk_afe_fe_hw_free(struct snd_pcm_substream *substream,
		       struct snd_soc_dai *dai);
int mtk_afe_fe_prepare(struct snd_pcm_substream *substream,
		       struct snd_soc_dai *dai);
int mtk_afe_fe_trigger(struct snd_pcm_substream *substream, int cmd,
		       struct snd_soc_dai *dai);

extern const struct snd_soc_dai_ops mtk_afe_fe_ops;

int mtk_dynamic_irq_acquire(struct mtk_base_afe *afe);
int mtk_dynamic_irq_release(struct mtk_base_afe *afe, int irq_id);
int mtk_afe_suspend(struct snd_soc_component *component);
int mtk_afe_resume(struct snd_soc_component *component);

unsigned int is_afe_need_triggered(struct mtk_base_afe_memif *memif);
int mtk_memif_set_addr(struct mtk_base_afe *afe, int id,
		       unsigned char *dma_area,
		       dma_addr_t dma_addr,
		       size_t dma_bytes);
int mtk_memif_set_channel(struct mtk_base_afe *afe,
			  int id, unsigned int channel);
int mtk_memif_set_rate(struct mtk_base_afe *afe,
		       int id, unsigned int rate);
int mtk_memif_set_rate_substream(struct snd_pcm_substream *substream,
				 int id, unsigned int rate);
int mtk_memif_set_format(struct mtk_base_afe *afe,
			 int id, snd_pcm_format_t format);
int mtk_memif_set_pbuf_size(struct mtk_base_afe *afe,
			    int id, int pbuf_size);
int mtk_memif_set_min_max_len(struct mtk_base_afe *afe, int id,
			      int min_l, int max_l);

/* using samephore to ensure ap/dsp sync */
int mtk_memif_set_enable(struct mtk_base_afe *afe, int id);
int mtk_memif_set_disable(struct mtk_base_afe *afe, int id);
int mtk_irq_set_enable(struct mtk_base_afe *afe,
		       const struct mtk_base_irq_data *irq_data,
		       int afe_id);
int mtk_irq_set_disable(struct mtk_base_afe *afe,
			const struct mtk_base_irq_data *irq_data,
			int afe_id);

void register_is_vow_bargein_memif_callback(bool (*callback)(int));

#endif
