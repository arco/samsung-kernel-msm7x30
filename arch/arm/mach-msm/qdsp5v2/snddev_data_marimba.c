/* Copyright (c) 2009-2011, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/mfd/msm-adie-codec.h>
#include <linux/uaccess.h>
#include <mach/qdsp5v2/snddev_icodec.h>
#if defined(CONFIG_MACH_ARIESVE)
#include <mach/qdsp5v2/marimba_profile_ariesve.h>
#elif defined(CONFIG_MACH_ANCORA_TMO)
#include <mach/qdsp5v2/marimba_profile_ancora_tmo.h>
#elif defined(CONFIG_MACH_ANCORA)
#include <mach/qdsp5v2/marimba_profile_ancora.h>
#elif defined(CONFIG_MACH_APACHE)
#include <mach/qdsp5v2/marimba_profile_apache.h>
#else
#include <mach/qdsp5v2/marimba_profile.h>
#endif
#include <mach/qdsp5v2/aux_pcm.h>
#include <mach/qdsp5v2/snddev_ecodec.h>
#include <mach/qdsp5v2/audio_dev_ctl.h>
#include <mach/qdsp5v2/snddev_virtual.h>
#include <mach/board.h>
#include <asm/mach-types.h>
#include <mach/gpio.h>
#include <mach/qdsp5v2/snddev_mi2s.h>
#include <mach/qdsp5v2/mi2s.h>
#include <mach/qdsp5v2/audio_acdb_def.h>

#ifdef CONFIG_VP_A2220
#include <linux/a2220.h>
#endif

#define SAMSUNG_AUDIO_PATH 1

/* define the value for BT_SCO */
#define BT_SCO_PCM_CTL_VAL (PCM_CTL__RPCM_WIDTH__LINEAR_V |\
				PCM_CTL__TPCM_WIDTH__LINEAR_V)
#define BT_SCO_DATA_FORMAT_PADDING (DATA_FORMAT_PADDING_INFO__RPCM_FORMAT_V |\
				DATA_FORMAT_PADDING_INFO__TPCM_FORMAT_V)
#define BT_SCO_AUX_CODEC_INTF   AUX_CODEC_INTF_CTL__PCMINTF_DATA_EN_V

#ifdef CONFIG_DEBUG_FS
static struct dentry *debugfs_hsed_config;
static void snddev_hsed_config_modify_setting(int type);
static void snddev_hsed_config_restore_setting(void);
#endif

static struct adie_codec_action_unit iearpiece_48KHz_osr256_actions[] =
	HANDSET_RX_48000_OSR_256;

static struct adie_codec_hwsetting_entry iearpiece_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = iearpiece_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(iearpiece_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile iearpiece_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = iearpiece_settings,
	.setting_sz = ARRAY_SIZE(iearpiece_settings),
};

static struct snddev_icodec_data snddev_iearpiece_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "handset_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_SPKR,
	.profile = &iearpiece_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
	.max_voice_rx_vol[VOC_NB_INDEX] = -200,
	.min_voice_rx_vol[VOC_NB_INDEX] = -1700,
	.max_voice_rx_vol[VOC_WB_INDEX] = -200,
	.min_voice_rx_vol[VOC_WB_INDEX] = -1700
};

static struct platform_device msm_iearpiece_device = {
	.name = "snddev_icodec",
	.id = 0,
	.dev = { .platform_data = &snddev_iearpiece_data },
};

static struct adie_codec_action_unit imic_8KHz_osr256_actions[] =
	HANDSET_TX_8000_OSR_256;

static struct adie_codec_action_unit imic_16KHz_osr256_actions[] =
	HANDSET_TX_16000_OSR_256;

static struct adie_codec_action_unit imic_48KHz_osr256_actions[] =
	HANDSET_TX_48000_OSR_256;

static struct adie_codec_hwsetting_entry imic_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = imic_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(imic_8KHz_osr256_actions),
	},
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = imic_16KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(imic_16KHz_osr256_actions),
	},
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = imic_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(imic_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile imic_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = imic_settings,
	.setting_sz = ARRAY_SIZE(imic_settings),
};

static enum hsed_controller imic_pmctl_id[] = {PM_HSED_CONTROLLER_0};

static struct snddev_icodec_data snddev_imic_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_MIC,
	.profile = &imic_profile,
	.channel_mode = 1,
	.pmctl_id = imic_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(imic_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};

static struct platform_device msm_imic_device = {
	.name = "snddev_icodec",
	.id = 1,
	.dev = { .platform_data = &snddev_imic_data },
};

static struct adie_codec_action_unit ihs_stereo_rx_48KHz_osr256_actions[] =
	HEADSET_STEREO_RX_LEGACY_48000_OSR_256;

static struct adie_codec_hwsetting_entry ihs_stereo_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_stereo_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ihs_stereo_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ihs_stereo_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ihs_stereo_rx_settings,
	.setting_sz = ARRAY_SIZE(ihs_stereo_rx_settings),
};

static struct snddev_icodec_data snddev_ihs_stereo_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_stereo_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_SPKR_STEREO,
	.profile = &ihs_stereo_rx_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_poweramp_on_headset,
	.pamp_off = msm_snddev_poweramp_off_headset,
	.max_voice_rx_vol[VOC_NB_INDEX] = -700,
	.min_voice_rx_vol[VOC_NB_INDEX] = -2200,
	.max_voice_rx_vol[VOC_WB_INDEX] = -900,
	.min_voice_rx_vol[VOC_WB_INDEX] = -2400
};

static struct platform_device msm_ihs_stereo_rx_device = {
	.name = "snddev_icodec",
	.id = 2,
	.dev = { .platform_data = &snddev_ihs_stereo_rx_data },
};

static struct adie_codec_action_unit ihs_mono_rx_48KHz_osr256_actions[] =
	HEADSET_RX_LEGACY_48000_OSR_256;

static struct adie_codec_hwsetting_entry ihs_mono_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_mono_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ihs_mono_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ihs_mono_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ihs_mono_rx_settings,
	.setting_sz = ARRAY_SIZE(ihs_mono_rx_settings),
};

static struct snddev_icodec_data snddev_ihs_mono_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_mono_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_SPKR_MONO,
	.profile = &ihs_mono_rx_profile,
	.channel_mode = 1,
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
	.max_voice_rx_vol[VOC_NB_INDEX] = -700,
	.min_voice_rx_vol[VOC_NB_INDEX] = -2200,
	.max_voice_rx_vol[VOC_WB_INDEX] = -900,
	.min_voice_rx_vol[VOC_WB_INDEX] = -2400,

};

static struct platform_device msm_ihs_mono_rx_device = {
	.name = "snddev_icodec",
	.id = 3,
	.dev = { .platform_data = &snddev_ihs_mono_rx_data },
};

static struct adie_codec_action_unit ihs_ffa_stereo_rx_48KHz_osr256_actions[] =
	HEADSET_STEREO_RX_CAPLESS_48000_OSR_256;

static struct adie_codec_hwsetting_entry ihs_ffa_stereo_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_ffa_stereo_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ihs_ffa_stereo_rx_48KHz_osr256_actions),
	}
};

#ifdef CONFIG_DEBUG_FS
static struct adie_codec_action_unit
	ihs_ffa_stereo_rx_class_d_legacy_48KHz_osr256_actions[] =
	HEADSET_STEREO_RX_CLASS_D_LEGACY_48000_OSR_256;

static struct adie_codec_hwsetting_entry
	ihs_ffa_stereo_rx_class_d_legacy_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions =
		ihs_ffa_stereo_rx_class_d_legacy_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE
		(ihs_ffa_stereo_rx_class_d_legacy_48KHz_osr256_actions),
	}
};

static struct adie_codec_action_unit
	ihs_ffa_stereo_rx_class_ab_legacy_48KHz_osr256_actions[] =
	HEADSET_STEREO_RX_LEGACY_48000_OSR_256;

static struct adie_codec_hwsetting_entry
	ihs_ffa_stereo_rx_class_ab_legacy_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions =
		ihs_ffa_stereo_rx_class_ab_legacy_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE
		(ihs_ffa_stereo_rx_class_ab_legacy_48KHz_osr256_actions),
	}
};
#endif

static struct adie_codec_dev_profile ihs_ffa_stereo_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ihs_ffa_stereo_rx_settings,
	.setting_sz = ARRAY_SIZE(ihs_ffa_stereo_rx_settings),
};

static struct snddev_icodec_data snddev_ihs_ffa_stereo_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_stereo_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_SPKR_STEREO,
	.profile = &ihs_ffa_stereo_rx_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.voltage_on = msm_snddev_hsed_voltage_on,
	.voltage_off = msm_snddev_hsed_voltage_off,
	.max_voice_rx_vol[VOC_NB_INDEX] = -700,
	.min_voice_rx_vol[VOC_NB_INDEX] = -2200,
	.max_voice_rx_vol[VOC_WB_INDEX] = -900,
	.min_voice_rx_vol[VOC_WB_INDEX] = -2400,
};

static struct platform_device msm_ihs_ffa_stereo_rx_device = {
	.name = "snddev_icodec",
	.id = 4,
	.dev = { .platform_data = &snddev_ihs_ffa_stereo_rx_data },
};

static struct adie_codec_action_unit ihs_ffa_mono_rx_48KHz_osr256_actions[] =
	HEADSET_RX_CAPLESS_48000_OSR_256;

static struct adie_codec_hwsetting_entry ihs_ffa_mono_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_ffa_mono_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ihs_ffa_mono_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ihs_ffa_mono_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ihs_ffa_mono_rx_settings,
	.setting_sz = ARRAY_SIZE(ihs_ffa_mono_rx_settings),
};

static struct snddev_icodec_data snddev_ihs_ffa_mono_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_mono_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_SPKR_MONO,
	.profile = &ihs_ffa_mono_rx_profile,
	.channel_mode = 1,
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_hsed_voltage_on,
	.pamp_off = msm_snddev_hsed_voltage_off,
	.max_voice_rx_vol[VOC_NB_INDEX] = -700,
	.min_voice_rx_vol[VOC_NB_INDEX] = -2200,
	.max_voice_rx_vol[VOC_WB_INDEX] = -900,
	.min_voice_rx_vol[VOC_WB_INDEX] = -2400,
};

static struct platform_device msm_ihs_ffa_mono_rx_device = {
	.name = "snddev_icodec",
	.id = 5,
	.dev = { .platform_data = &snddev_ihs_ffa_mono_rx_data },
};

static struct adie_codec_action_unit ihs_mono_tx_8KHz_osr256_actions[] =
	HEADSET_MONO_TX_8000_OSR_256;

static struct adie_codec_action_unit ihs_mono_tx_16KHz_osr256_actions[] =
	HEADSET_MONO_TX_16000_OSR_256;

static struct adie_codec_action_unit ihs_mono_tx_48KHz_osr256_actions[] =
	HEADSET_MONO_TX_48000_OSR_256;

static struct adie_codec_hwsetting_entry ihs_mono_tx_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ihs_mono_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ihs_mono_tx_8KHz_osr256_actions),
	},
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = ihs_mono_tx_16KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ihs_mono_tx_16KHz_osr256_actions),
	},
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_mono_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ihs_mono_tx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ihs_mono_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = ihs_mono_tx_settings,
	.setting_sz = ARRAY_SIZE(ihs_mono_tx_settings),
};

static struct snddev_icodec_data snddev_ihs_mono_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "headset_mono_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_MIC,
	.profile = &ihs_mono_tx_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};

static struct platform_device msm_ihs_mono_tx_device = {
	.name = "snddev_icodec",
	.id = 6,
	.dev = { .platform_data = &snddev_ihs_mono_tx_data },
};

static struct adie_codec_action_unit ifmradio_handset_osr64_actions[] =
	FM_HANDSET_OSR_64;

static struct adie_codec_hwsetting_entry ifmradio_handset_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ifmradio_handset_osr64_actions,
		.action_sz = ARRAY_SIZE(ifmradio_handset_osr64_actions),
	}
};

static struct adie_codec_dev_profile ifmradio_handset_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ifmradio_handset_settings,
	.setting_sz = ARRAY_SIZE(ifmradio_handset_settings),
};

static struct snddev_icodec_data snddev_ifmradio_handset_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_FM),
	.name = "fmradio_handset_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_LP_FM_SPKR_PHONE_STEREO_RX,
	.profile = &ifmradio_handset_profile,
	.channel_mode = 1,
	.default_sample_rate = 8000,
	.pamp_on = NULL,
	.pamp_off = NULL,
	.dev_vol_type = SNDDEV_DEV_VOL_DIGITAL,
};

static struct platform_device msm_ifmradio_handset_device = {
	.name = "snddev_icodec",
	.id = 7,
	.dev = { .platform_data = &snddev_ifmradio_handset_data },
};


static struct adie_codec_action_unit ispeaker_rx_48KHz_osr256_actions[] =
   SPEAKER_STEREO_RX_48000_OSR_256;

static struct adie_codec_hwsetting_entry ispeaker_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ispeaker_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispeaker_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ispeaker_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ispeaker_rx_settings,
	.setting_sz = ARRAY_SIZE(ispeaker_rx_settings),
};

static struct snddev_icodec_data snddev_ispeaker_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "speaker_stereo_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_STEREO,
	.profile = &ispeaker_rx_profile,
	.channel_mode = 2,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = &msm_snddev_poweramp_on_speaker,
	.pamp_off = &msm_snddev_poweramp_off_speaker,
	.max_voice_rx_vol[VOC_NB_INDEX] = 1000,
	.min_voice_rx_vol[VOC_NB_INDEX] = -500,
	.max_voice_rx_vol[VOC_WB_INDEX] = 1000,
	.min_voice_rx_vol[VOC_WB_INDEX] = -500,
};

static struct platform_device msm_ispeaker_rx_device = {
	.name = "snddev_icodec",
	.id = 8,
	.dev = { .platform_data = &snddev_ispeaker_rx_data },

};

static struct adie_codec_action_unit ifmradio_speaker_osr64_actions[] =
	FM_SPEAKER_OSR_64;

static struct adie_codec_hwsetting_entry ifmradio_speaker_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ifmradio_speaker_osr64_actions,
		.action_sz = ARRAY_SIZE(ifmradio_speaker_osr64_actions),
	}
};

static struct adie_codec_dev_profile ifmradio_speaker_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ifmradio_speaker_settings,
	.setting_sz = ARRAY_SIZE(ifmradio_speaker_settings),
};

static struct snddev_icodec_data snddev_ifmradio_speaker_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_FM),
	.name = "fmradio_speaker_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_LP_FM_SPKR_PHONE_STEREO_RX,
	.profile = &ifmradio_speaker_profile,
	.channel_mode = 1,
	.default_sample_rate = 8000,
	.pamp_on = &msm_snddev_poweramp_on_speaker,
	.pamp_off = &msm_snddev_poweramp_off_speaker,
	.dev_vol_type = SNDDEV_DEV_VOL_DIGITAL,
};

static struct platform_device msm_ifmradio_speaker_device = {
	.name = "snddev_icodec",
	.id = 9,
	.dev = { .platform_data = &snddev_ifmradio_speaker_data },
};

static struct adie_codec_action_unit ifmradio_headset_osr64_actions[] =
	FM_HEADSET_STEREO_CLASS_D_LEGACY_OSR_64;

static struct adie_codec_hwsetting_entry ifmradio_headset_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ifmradio_headset_osr64_actions,
		.action_sz = ARRAY_SIZE(ifmradio_headset_osr64_actions),
	}
};

static struct adie_codec_dev_profile ifmradio_headset_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ifmradio_headset_settings,
	.setting_sz = ARRAY_SIZE(ifmradio_headset_settings),
};

static struct snddev_icodec_data snddev_ifmradio_headset_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_FM),
	.name = "fmradio_headset_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_LP_FM_HEADSET_SPKR_STEREO_RX,
	.profile = &ifmradio_headset_profile,
	.channel_mode = 1,
	.default_sample_rate = 8000,
	.pamp_on = NULL,
	.pamp_off = NULL,
	.dev_vol_type = SNDDEV_DEV_VOL_DIGITAL,
};

static struct platform_device msm_ifmradio_headset_device = {
	.name = "snddev_icodec",
	.id = 10,
	.dev = { .platform_data = &snddev_ifmradio_headset_data },
};


static struct adie_codec_action_unit ifmradio_ffa_headset_osr64_actions[] =
	FM_HEADSET_CLASS_AB_STEREO_CAPLESS_OSR_64;

static struct adie_codec_hwsetting_entry ifmradio_ffa_headset_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ifmradio_ffa_headset_osr64_actions,
		.action_sz = ARRAY_SIZE(ifmradio_ffa_headset_osr64_actions),
	}
};

static struct adie_codec_dev_profile ifmradio_ffa_headset_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ifmradio_ffa_headset_settings,
	.setting_sz = ARRAY_SIZE(ifmradio_ffa_headset_settings),
};

static struct snddev_icodec_data snddev_ifmradio_ffa_headset_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_FM),
	.name = "fmradio_headset_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_LP_FM_HEADSET_SPKR_STEREO_RX,
	.profile = &ifmradio_ffa_headset_profile,
	.channel_mode = 1,
	.default_sample_rate = 8000,
	.pamp_on = msm_snddev_hsed_voltage_on,
	.pamp_off = msm_snddev_hsed_voltage_off,
	.dev_vol_type = SNDDEV_DEV_VOL_DIGITAL,
};

static struct platform_device msm_ifmradio_ffa_headset_device = {
	.name = "snddev_icodec",
	.id = 11,
	.dev = { .platform_data = &snddev_ifmradio_ffa_headset_data },
};

static struct snddev_ecodec_data snddev_bt_sco_earpiece_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "bt_sco_rx",
	.copp_id = 1,
	.acdb_id = ACDB_ID_BT_SCO_SPKR,
	.channel_mode = 1,
	.conf_pcm_ctl_val = BT_SCO_PCM_CTL_VAL,
	.conf_aux_codec_intf = BT_SCO_AUX_CODEC_INTF,
	.conf_data_format_padding_val = BT_SCO_DATA_FORMAT_PADDING,
	.max_voice_rx_vol[VOC_NB_INDEX] = 400,
	.min_voice_rx_vol[VOC_NB_INDEX] = -1100,
	.max_voice_rx_vol[VOC_WB_INDEX] = 400,
	.min_voice_rx_vol[VOC_WB_INDEX] = -1100,
};

static struct snddev_ecodec_data snddev_bt_sco_mic_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "bt_sco_tx",
	.copp_id = 1,
	.acdb_id = ACDB_ID_BT_SCO_MIC,
	.channel_mode = 1,
	.conf_pcm_ctl_val = BT_SCO_PCM_CTL_VAL,
	.conf_aux_codec_intf = BT_SCO_AUX_CODEC_INTF,
	.conf_data_format_padding_val = BT_SCO_DATA_FORMAT_PADDING,
};

struct platform_device msm_bt_sco_earpiece_device = {
	.name = "msm_snddev_ecodec",
	.id = 0,
	.dev = { .platform_data = &snddev_bt_sco_earpiece_data },
};

struct platform_device msm_bt_sco_mic_device = {
	.name = "msm_snddev_ecodec",
	.id = 1,
	.dev = { .platform_data = &snddev_bt_sco_mic_data },
};

static struct adie_codec_action_unit idual_mic_endfire_8KHz_osr256_actions[] =
	MIC1_LEFT_LINE_IN_RIGHT_8000_OSR_256;

static struct adie_codec_hwsetting_entry idual_mic_endfire_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = idual_mic_endfire_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(idual_mic_endfire_8KHz_osr256_actions),
	}, /* 8KHz profile can be used for 16KHz */
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = idual_mic_endfire_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(idual_mic_endfire_8KHz_osr256_actions),
	}, /* 8KHz profile can be used for 48KHz */
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = idual_mic_endfire_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(idual_mic_endfire_8KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile idual_mic_endfire_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = idual_mic_endfire_settings,
	.setting_sz = ARRAY_SIZE(idual_mic_endfire_settings),
};

static enum hsed_controller idual_mic_endfire_pmctl_id[] = {
	PM_HSED_CONTROLLER_0, PM_HSED_CONTROLLER_2
};

static struct snddev_icodec_data snddev_idual_mic_endfire_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_dual_mic_endfire_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_MIC_ENDFIRE,
	.profile = &idual_mic_endfire_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pmctl_id = idual_mic_endfire_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(idual_mic_endfire_pmctl_id),
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_idual_mic_endfire_device = {
	.name = "snddev_icodec",
	.id = 12,
	.dev = { .platform_data = &snddev_idual_mic_endfire_data },
};


static struct snddev_icodec_data\
		snddev_idual_mic_endfire_real_stereo_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_dual_mic_endfire_tx_real_stereo",
	.copp_id = 0,
	.acdb_id = PSEUDO_ACDB_ID,
	.profile = &idual_mic_endfire_profile,
	.channel_mode = REAL_STEREO_CHANNEL_MODE,
	.default_sample_rate = 48000,
	.pmctl_id = idual_mic_endfire_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(idual_mic_endfire_pmctl_id),
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_real_stereo_tx_device = {
	.name = "snddev_icodec",
	.id = 26,
	.dev = { .platform_data =
			&snddev_idual_mic_endfire_real_stereo_data },
};

static struct adie_codec_action_unit idual_mic_bs_8KHz_osr256_actions[] =
	MIC1_LEFT_AUX_IN_RIGHT_8000_OSR_256;

static struct adie_codec_hwsetting_entry idual_mic_broadside_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = idual_mic_bs_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(idual_mic_bs_8KHz_osr256_actions),
	}, /* 8KHz profile can be used for 16KHz */
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = idual_mic_bs_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(idual_mic_bs_8KHz_osr256_actions),
	}, /* 8KHz profile can be used for 16KHz */
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = idual_mic_bs_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(idual_mic_bs_8KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile idual_mic_broadside_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = idual_mic_broadside_settings,
	.setting_sz = ARRAY_SIZE(idual_mic_broadside_settings),
};

static enum hsed_controller idual_mic_broadside_pmctl_id[] = {
	PM_HSED_CONTROLLER_0, PM_HSED_CONTROLLER_2
};

static struct snddev_icodec_data snddev_idual_mic_broadside_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_dual_mic_broadside_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_MIC_BROADSIDE,
	.profile = &idual_mic_broadside_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pmctl_id = idual_mic_broadside_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(idual_mic_broadside_pmctl_id),
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_idual_mic_broadside_device = {
	.name = "snddev_icodec",
	.id = 13,
	.dev = { .platform_data = &snddev_idual_mic_broadside_data },
};

static struct adie_codec_action_unit ispk_dual_mic_ef_8KHz_osr256_actions[] =
	SPEAKER_MIC1_LEFT_LINE_IN_RIGHT_8000_OSR_256;

static struct adie_codec_hwsetting_entry ispk_dual_mic_ef_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ispk_dual_mic_ef_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispk_dual_mic_ef_8KHz_osr256_actions),
	}, /* 8KHz profile can be used for 16Khz */
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = ispk_dual_mic_ef_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispk_dual_mic_ef_8KHz_osr256_actions),
	}, /* 8KHz profile can be used for 48KHz */
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ispk_dual_mic_ef_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispk_dual_mic_ef_8KHz_osr256_actions),
	},
};

static struct adie_codec_dev_profile ispk_dual_mic_ef_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = ispk_dual_mic_ef_settings,
	.setting_sz = ARRAY_SIZE(ispk_dual_mic_ef_settings),
};

static struct snddev_icodec_data snddev_spk_idual_mic_endfire_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "speaker_dual_mic_endfire_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_MIC_ENDFIRE,
	.profile = &ispk_dual_mic_ef_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pmctl_id = idual_mic_endfire_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(idual_mic_endfire_pmctl_id),
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_spk_idual_mic_endfire_device = {
	.name = "snddev_icodec",
	.id = 14,
	.dev = { .platform_data = &snddev_spk_idual_mic_endfire_data },
};

static struct adie_codec_action_unit ispk_dual_mic_bs_8KHz_osr256_actions[] =
	SPEAKER_MIC1_LEFT_AUX_IN_RIGHT_8000_OSR_256;

static struct adie_codec_hwsetting_entry ispk_dual_mic_bs_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ispk_dual_mic_bs_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispk_dual_mic_bs_8KHz_osr256_actions),
	}, /* 8KHz profile can be used for 16Khz */
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = ispk_dual_mic_bs_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispk_dual_mic_bs_8KHz_osr256_actions),
	}, /* 8KHz profile can be used for 48KHz */
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ispk_dual_mic_bs_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispk_dual_mic_bs_8KHz_osr256_actions),
	},
};

static struct adie_codec_dev_profile ispk_dual_mic_bs_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = ispk_dual_mic_bs_settings,
	.setting_sz = ARRAY_SIZE(ispk_dual_mic_bs_settings),
};
static struct snddev_icodec_data snddev_spk_idual_mic_broadside_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "speaker_dual_mic_broadside_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_MIC_BROADSIDE,
	.profile = &ispk_dual_mic_bs_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pmctl_id = idual_mic_broadside_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(idual_mic_broadside_pmctl_id),
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_spk_idual_mic_broadside_device = {
	.name = "snddev_icodec",
	.id = 15,
	.dev = { .platform_data = &snddev_spk_idual_mic_broadside_data },
};

static struct adie_codec_action_unit itty_hs_mono_tx_8KHz_osr256_actions[] =
	TTY_HEADSET_MONO_TX_8000_OSR_256;

static struct adie_codec_hwsetting_entry itty_hs_mono_tx_settings[] = {
	/* 8KHz, 16KHz, 48KHz TTY Tx devices can shared same set of actions */
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = itty_hs_mono_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(itty_hs_mono_tx_8KHz_osr256_actions),
	},
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = itty_hs_mono_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(itty_hs_mono_tx_8KHz_osr256_actions),
	},
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = itty_hs_mono_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(itty_hs_mono_tx_8KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile itty_hs_mono_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = itty_hs_mono_tx_settings,
	.setting_sz = ARRAY_SIZE(itty_hs_mono_tx_settings),
};

static struct snddev_icodec_data snddev_itty_hs_mono_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE | SNDDEV_CAP_TTY),
	.name = "tty_headset_mono_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_TTY_HEADSET_MIC,
	.profile = &itty_hs_mono_tx_profile,
	.channel_mode = 1,
	.default_sample_rate = 48000,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_itty_hs_mono_tx_device = {
	.name = "snddev_icodec",
	.id = 16,
	.dev = { .platform_data = &snddev_itty_hs_mono_tx_data },
};

static struct adie_codec_action_unit itty_hs_mono_rx_8KHz_osr256_actions[] =
	TTY_HEADSET_MONO_RX_CLASS_D_8000_OSR_256;

static struct adie_codec_action_unit itty_hs_mono_rx_16KHz_osr256_actions[] =
	TTY_HEADSET_MONO_RX_CLASS_D_16000_OSR_256;

static struct adie_codec_action_unit itty_hs_mono_rx_48KHz_osr256_actions[] =
	TTY_HEADSET_MONO_RX_CLASS_D_48000_OSR_256;

static struct adie_codec_hwsetting_entry itty_hs_mono_rx_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = itty_hs_mono_rx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(itty_hs_mono_rx_8KHz_osr256_actions),
	},
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = itty_hs_mono_rx_16KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(itty_hs_mono_rx_16KHz_osr256_actions),
	},
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = itty_hs_mono_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(itty_hs_mono_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile itty_hs_mono_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = itty_hs_mono_rx_settings,
	.setting_sz = ARRAY_SIZE(itty_hs_mono_rx_settings),
};

static struct snddev_icodec_data snddev_itty_hs_mono_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE | SNDDEV_CAP_TTY),
	.name = "tty_headset_mono_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_TTY_HEADSET_SPKR,
	.profile = &itty_hs_mono_rx_profile,
	.channel_mode = 1,
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_poweramp_on_tty,
	.pamp_off = msm_snddev_poweramp_off_tty,
	.max_voice_rx_vol[VOC_NB_INDEX] = 0,
	.min_voice_rx_vol[VOC_NB_INDEX] = 0,
	.max_voice_rx_vol[VOC_WB_INDEX] = 0,
	.min_voice_rx_vol[VOC_WB_INDEX] = 0,
};

static struct platform_device msm_itty_hs_mono_rx_device = {
	.name = "snddev_icodec",
	.id = 17,
	.dev = { .platform_data = &snddev_itty_hs_mono_rx_data },
};

static struct adie_codec_action_unit ispeaker_tx_8KHz_osr256_actions[] =
	SPEAKER_TX_8000_OSR_256;

static struct adie_codec_action_unit ispeaker_tx_48KHz_osr256_actions[] =
	SPEAKER_TX_48000_OSR_256;

static struct adie_codec_hwsetting_entry ispeaker_tx_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ispeaker_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispeaker_tx_8KHz_osr256_actions),
	},
	{ /* 8KHz profile is good for 16KHz */
		.freq_plan = 16000,
		.osr = 256,
		.actions = ispeaker_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispeaker_tx_8KHz_osr256_actions),
	},
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ispeaker_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispeaker_tx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ispeaker_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = ispeaker_tx_settings,
	.setting_sz = ARRAY_SIZE(ispeaker_tx_settings),
};

static enum hsed_controller ispk_pmctl_id[] = {PM_HSED_CONTROLLER_0};

static struct snddev_icodec_data snddev_ispeaker_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "speaker_mono_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_MIC,
	.profile = &ispeaker_tx_profile,
	.channel_mode = 1,
	.pmctl_id = ispk_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(ispk_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};

static struct platform_device msm_ispeaker_tx_device = {
	.name = "snddev_icodec",
	.id = 18,
	.dev = { .platform_data = &snddev_ispeaker_tx_data },
};

static struct adie_codec_action_unit iearpiece_ffa_48KHz_osr256_actions[] =
	HANDSET_RX_48000_OSR_256_FFA;

static struct adie_codec_hwsetting_entry iearpiece_ffa_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = iearpiece_ffa_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(iearpiece_ffa_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile iearpiece_ffa_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = iearpiece_ffa_settings,
	.setting_sz = ARRAY_SIZE(iearpiece_ffa_settings),
};

static struct snddev_icodec_data snddev_iearpiece_ffa_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "handset_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_SPKR,
	.profile = &iearpiece_ffa_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
	.max_voice_rx_vol[VOC_NB_INDEX] = -700,
	.min_voice_rx_vol[VOC_NB_INDEX] = -2200,
	.max_voice_rx_vol[VOC_WB_INDEX] = -1400,
	.min_voice_rx_vol[VOC_WB_INDEX] = -2900,
};

static struct platform_device msm_iearpiece_ffa_device = {
	.name = "snddev_icodec",
	.id = 19,
	.dev = { .platform_data = &snddev_iearpiece_ffa_data },
};

static struct adie_codec_action_unit imic_ffa_8KHz_osr256_actions[] =
	HANDSET_TX_8000_OSR_256_FFA;

static struct adie_codec_action_unit imic_ffa_16KHz_osr256_actions[] =
	HANDSET_TX_16000_OSR_256_FFA;

static struct adie_codec_action_unit imic_ffa_48KHz_osr256_actions[] =
	HANDSET_TX_48000_OSR_256_FFA;

static struct adie_codec_hwsetting_entry imic_ffa_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = imic_ffa_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(imic_ffa_8KHz_osr256_actions),
	},
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = imic_ffa_16KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(imic_ffa_16KHz_osr256_actions),
	},
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = imic_ffa_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(imic_ffa_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile imic_ffa_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = imic_ffa_settings,
	.setting_sz = ARRAY_SIZE(imic_ffa_settings),
};

static struct snddev_icodec_data snddev_imic_ffa_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_MIC,
	.profile = &imic_ffa_profile,
	.channel_mode = 1,
	.pmctl_id = imic_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(imic_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_imic_ffa_device = {
	.name = "snddev_icodec",
	.id = 20,
	.dev = { .platform_data = &snddev_imic_ffa_data },
};


static struct adie_codec_action_unit
	ihs_stereo_speaker_stereo_rx_48KHz_osr256_actions[] =
	HEADSET_STEREO_SPEAKER_STEREO_RX_CAPLESS_48000_OSR_256;


static struct adie_codec_hwsetting_entry
	ihs_stereo_speaker_stereo_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_stereo_speaker_stereo_rx_48KHz_osr256_actions,
		.action_sz =
		ARRAY_SIZE(ihs_stereo_speaker_stereo_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ihs_stereo_speaker_stereo_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ihs_stereo_speaker_stereo_rx_settings,
	.setting_sz = ARRAY_SIZE(ihs_stereo_speaker_stereo_rx_settings),
};

static struct snddev_icodec_data snddev_ihs_stereo_speaker_stereo_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_stereo_speaker_stereo_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_STEREO_PLUS_SPKR_STEREO_RX,
	.profile = &ihs_stereo_speaker_stereo_rx_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_poweramp_on_together,
	.pamp_off = msm_snddev_poweramp_off_together,
	.voltage_on = msm_snddev_hsed_voltage_on,
	.voltage_off = msm_snddev_hsed_voltage_off,
	.max_voice_rx_vol[VOC_NB_INDEX] = -500,
	.min_voice_rx_vol[VOC_NB_INDEX] = -2000,
	.max_voice_rx_vol[VOC_WB_INDEX] = -500,
	.min_voice_rx_vol[VOC_WB_INDEX] = -2000,
};

static struct platform_device msm_ihs_stereo_speaker_stereo_rx_device = {
	.name = "snddev_icodec",
	.id = 21,
	.dev = { .platform_data = &snddev_ihs_stereo_speaker_stereo_rx_data },
};

static struct snddev_mi2s_data snddev_mi2s_stereo_rx_data = {
	.capability = SNDDEV_CAP_RX ,
	.name = "hdmi_stereo_rx",
	.copp_id = 3,
	.acdb_id = ACDB_ID_HDMI,
	.channel_mode = 2,
	.sd_lines = MI2S_SD_0,
	.route = msm_snddev_tx_route_config,
	.deroute = msm_snddev_tx_route_deconfig,
	.default_sample_rate = 48000,
};

static struct platform_device msm_snddev_mi2s_stereo_rx_device = {
	.name = "snddev_mi2s",
	.id = 0,
	.dev = { .platform_data = &snddev_mi2s_stereo_rx_data },
};


static struct snddev_mi2s_data snddev_mi2s_fm_tx_data = {
	.capability = SNDDEV_CAP_TX ,
	.name = "fmradio_stereo_tx",
	.copp_id = 2,
	.acdb_id = ACDB_ID_FM_TX,
	.channel_mode = 2,
	.sd_lines = MI2S_SD_3,
	.route = NULL,
	.deroute = NULL,
	.default_sample_rate = 48000,
};

static struct platform_device  msm_snddev_mi2s_fm_tx_device = {
	.name = "snddev_mi2s",
	.id = 1,
	.dev = { .platform_data = &snddev_mi2s_fm_tx_data},
};

static struct snddev_icodec_data snddev_fluid_imic_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_MIC,
	.profile = &ispeaker_tx_profile,
	.channel_mode = 1,
	.pmctl_id = ispk_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(ispk_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};

static struct platform_device msm_fluid_imic_tx_device = {
	.name = "snddev_icodec",
	.id = 22,
	.dev = { .platform_data = &snddev_fluid_imic_tx_data },
};

static struct snddev_icodec_data snddev_fluid_iearpiece_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "handset_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_STEREO,
	.profile = &ispeaker_rx_profile,
	.channel_mode = 2,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = &msm_snddev_poweramp_on_speaker,
	.pamp_off = &msm_snddev_poweramp_off_speaker,
	.max_voice_rx_vol[VOC_NB_INDEX] = -500,
	.min_voice_rx_vol[VOC_NB_INDEX] = -1000,
	.max_voice_rx_vol[VOC_WB_INDEX] = -500,
	.min_voice_rx_vol[VOC_WB_INDEX] = -1000,
};

static struct platform_device msm_fluid_iearpeice_rx_device = {
	.name = "snddev_icodec",
	.id = 23,
	.dev = { .platform_data = &snddev_fluid_iearpiece_rx_data },
};

static struct adie_codec_action_unit fluid_idual_mic_ef_8KHz_osr256_actions[] =
	MIC1_LEFT_AUX_IN_RIGHT_8000_OSR_256;

static struct adie_codec_hwsetting_entry fluid_idual_mic_endfire_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = fluid_idual_mic_ef_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(fluid_idual_mic_ef_8KHz_osr256_actions),
	}, /* 8KHz profile can be used for 16KHz */
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = fluid_idual_mic_ef_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(fluid_idual_mic_ef_8KHz_osr256_actions),
	}, /* 8KHz profile can also be used for 48KHz */
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = fluid_idual_mic_ef_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(fluid_idual_mic_ef_8KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile fluid_idual_mic_endfire_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = fluid_idual_mic_endfire_settings,
	.setting_sz = ARRAY_SIZE(fluid_idual_mic_endfire_settings),
};

static enum hsed_controller fluid_idual_mic_endfire_pmctl_id[] = {
	PM_HSED_CONTROLLER_0, PM_HSED_CONTROLLER_2
};

static struct snddev_icodec_data snddev_fluid_idual_mic_endfire_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_dual_mic_endfire_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_MIC_ENDFIRE,
	.profile = &fluid_idual_mic_endfire_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pmctl_id = fluid_idual_mic_endfire_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(fluid_idual_mic_endfire_pmctl_id),
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};

static struct platform_device msm_fluid_idual_mic_endfire_device = {
	.name = "snddev_icodec",
	.id = 24,
	.dev = { .platform_data = &snddev_fluid_idual_mic_endfire_data },
};

static struct snddev_icodec_data snddev_fluid_spk_idual_mic_endfire_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "speaker_dual_mic_endfire_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_MIC_ENDFIRE,
	.profile = &fluid_idual_mic_endfire_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pmctl_id = fluid_idual_mic_endfire_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(fluid_idual_mic_endfire_pmctl_id),
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};

static struct platform_device msm_fluid_spk_idual_mic_endfire_device = {
	.name = "snddev_icodec",
	.id = 25,
	.dev = { .platform_data = &snddev_fluid_spk_idual_mic_endfire_data },
};

static struct snddev_virtual_data snddev_a2dp_tx_data = {
	.capability = SNDDEV_CAP_TX,
	.name = "a2dp_tx",
	.copp_id = 5,
	.acdb_id = PSEUDO_ACDB_ID,
};

static struct snddev_virtual_data snddev_a2dp_rx_data = {
	.capability = SNDDEV_CAP_RX,
	.name = "a2dp_rx",
	.copp_id = 2,
	.acdb_id = PSEUDO_ACDB_ID,
};

static struct platform_device msm_a2dp_rx_device = {
	.name = "snddev_virtual",
	.id = 0,
	.dev = { .platform_data = &snddev_a2dp_rx_data },
};

static struct platform_device msm_a2dp_tx_device = {
	.name = "snddev_virtual",
	.id = 1,
	.dev = { .platform_data = &snddev_a2dp_tx_data },
};

static struct snddev_virtual_data snddev_uplink_rx_data = {
	.capability = SNDDEV_CAP_RX,
	.name = "uplink_rx",
	.copp_id = 5,
	.acdb_id = PSEUDO_ACDB_ID,
};

static struct platform_device msm_uplink_rx_device = {
	.name = "snddev_virtual",
	.id = 2,
	.dev = { .platform_data = &snddev_uplink_rx_data },
};

/*********************************************************************************************
 *
 * 				DEVICE LIST
 *
 *********************************************************************************************/

static struct adie_codec_action_unit handset_rx_48KHz_osr256_actions[] =
	ADIE_HANDSET_RX_48000_256;
static struct adie_codec_action_unit handset_tx_48KHz_osr256_actions[] =
	ADIE_HANDSET_TX_48000_256;
static struct adie_codec_action_unit speaker_rx_48KHz_osr256_actions[] =
	ADIE_SPEAKER_RX_48000_256;
static struct adie_codec_action_unit speaker_tx_48KHz_osr256_actions[] =
	ADIE_SPEAKER_TX_48000_256;
static struct adie_codec_action_unit headset_rx_48KHz_osr256_actions[] =
	ADIE_HEADSET_RX_48000_256;
static struct adie_codec_action_unit headset_tx_48KHz_osr256_actions[] =
	ADIE_HEADSET_TX_48000_256;
static struct adie_codec_action_unit aux_dock_rx_48KHz_osr256_actions[] =
	ADIE_AUX_DOCK_RX_48000_256;
static struct adie_codec_action_unit speaker_headset_rx_48KHz_osr256_actions[] =
	ADIE_SPEAKER_HEADSET_RX_48000_256;
static struct adie_codec_action_unit speaker_dock_rx_48KHz_osr256_actions[] =
	ADIE_SPEAKER_DOCK_RX_48000_256;
static struct adie_codec_action_unit speaker_hdmi_rx_48KHz_osr256_actions[] =
	ADIE_SPEAKER_HDMI_RX_48000_256;
static struct adie_codec_action_unit handset_call_rx_48KHz_osr256_actions[] =
	ADIE_HANDSET_CALL_RX_48000_256;
static struct adie_codec_action_unit handset_call_tx_48KHz_osr256_actions[] =
	ADIE_HANDSET_CALL_TX_48000_256;
static struct adie_codec_action_unit speaker_call_rx_48KHz_osr256_actions[] =
	ADIE_SPEAKER_CALL_RX_48000_256;
static struct adie_codec_action_unit speaker_call_tx_48KHz_osr256_actions[] =
	ADIE_SPEAKER_CALL_TX_48000_256;
static struct adie_codec_action_unit headset_call_rx_48KHz_osr256_actions[] =
	ADIE_HEADSET_CALL_RX_48000_256;
static struct adie_codec_action_unit headset_call_tx_48KHz_osr256_actions[] =
	ADIE_HEADSET_CALL_TX_48000_256;
static struct adie_codec_action_unit tty_headset_mono_call_rx_48KHz_osr256_actions[] =
	ADIE_TTY_HEADSET_MONO_CALL_RX_48000_256;
static struct adie_codec_action_unit tty_headset_mono_call_tx_48KHz_osr256_actions[] =
	ADIE_TTY_HEADSET_MONO_CALL_TX_48000_256;
static struct adie_codec_action_unit dualmic_handset_call_tx_48KHz_osr256_actions[] =
	ADIE_DUALMIC_HANDSET_CALL_TX_48000_256;
static struct adie_codec_action_unit handset_fmradio_rx_8KHz_osr256_actions[] =
	ADIE_HANDSET_FMRADIO_RX_8000_256;
static struct adie_codec_action_unit headset_fmradio_rx_8KHz_osr256_actions[] =
	ADIE_HEADSET_FMRADIO_RX_8000_256;
static struct adie_codec_action_unit speaker_fmradio_rx_8KHz_osr256_actions[] =
	ADIE_SPEAKER_FMRADIO_RX_8000_256;
static struct adie_codec_action_unit speaker_voice_dialer_tx_48KHz_osr256_actions[] =
	ADIE_SPEAKER_VOICE_DIALER_TX_48000_256;
static struct adie_codec_action_unit headset_voice_dialer_tx_48KHz_osr256_actions[] =
	ADIE_HEADSET_VOICE_DIALER_TX_48000_256;
static struct adie_codec_action_unit speaker_voice_search_tx_48KHz_osr256_actions[] =
	ADIE_SPEAKER_VOICE_SEARCH_TX_48000_256;
static struct adie_codec_action_unit headset_voice_search_tx_48KHz_osr256_actions[] =
	ADIE_HEADSET_VOICE_SEARCH_TX_48000_256;
static struct adie_codec_action_unit headset_fmradio_only_rx_8KHz_osr256_actions[] =
	ADIE_HEADSET_FMRADIO_ONLY_RX_8000_256;
static struct adie_codec_action_unit speaker_fmradio_only_rx_8KHz_osr256_actions[] =
	ADIE_SPEAKER_FMRADIO_ONLY_RX_8000_256;
static struct adie_codec_action_unit handset_voip_rx_48KHz_osr256_actions[] =
	ADIE_HANDSET_VOIP_RX_48000_256;
static struct adie_codec_action_unit handset_voip_tx_48KHz_osr256_actions[] =
	ADIE_HANDSET_VOIP_TX_48000_256;
static struct adie_codec_action_unit speaker_voip_rx_48KHz_osr256_actions[] =
	ADIE_SPEAKER_VOIP_RX_48000_256;
static struct adie_codec_action_unit speaker_voip_tx_48KHz_osr256_actions[] =
	ADIE_SPEAKER_VOIP_TX_48000_256;
static struct adie_codec_action_unit headset_voip_rx_48KHz_osr256_actions[] =
	ADIE_HEADSET_VOIP_RX_48000_256;
static struct adie_codec_action_unit headset_voip_tx_48KHz_osr256_actions[] =
	ADIE_HEADSET_VOIP_TX_48000_256;
#ifdef CONFIG_MACH_ANCORA_TMO
static struct adie_codec_action_unit handset_call_hac_rx_48KHz_osr256_actions[] =
	ADIE_HANDSET_CALL_HAC_RX_48000_256;
static struct adie_codec_action_unit handset_call_hac_tx_48KHz_osr256_actions[] =
	ADIE_HANDSET_CALL_HAC_TX_48000_256;
#endif
#ifdef CONFIG_MACH_ANCORA_TMO
static struct adie_codec_action_unit handset_gan_rx_48KHz_osr256_actions[] =
	ADIE_HANDSET_GAN_RX_48000_256;
static struct adie_codec_action_unit handset_gan_tx_48KHz_osr256_actions[] =
	ADIE_HANDSET_GAN_TX_48000_256;
static struct adie_codec_action_unit speaker_gan_rx_48KHz_osr256_actions[] =
	ADIE_SPEAKER_GAN_RX_48000_256;
static struct adie_codec_action_unit speaker_gan_tx_48KHz_osr256_actions[] =
	ADIE_SPEAKER_GAN_TX_48000_256;
static struct adie_codec_action_unit headset_gan_rx_48KHz_osr256_actions[] =
	ADIE_HEADSET_GAN_RX_48000_256;
static struct adie_codec_action_unit headset_gan_tx_48KHz_osr256_actions[] =
	ADIE_HEADSET_GAN_TX_48000_256;
#endif
#if defined(CONFIG_MACH_ANCORA_TMO) || defined(CONFIG_MACH_APACHE)
static struct adie_codec_action_unit handset_gtalk_rx_48KHz_osr256_actions[] =
	ADIE_HANDSET_GTALK_RX_48000_256;
static struct adie_codec_action_unit handset_gtalk_tx_48KHz_osr256_actions[] =
	ADIE_HANDSET_GTALK_TX_48000_256;
static struct adie_codec_action_unit speaker_gtalk_rx_48KHz_osr256_actions[] =
	ADIE_SPEAKER_GTALK_RX_48000_256;
static struct adie_codec_action_unit speaker_gtalk_tx_48KHz_osr256_actions[] =
	ADIE_SPEAKER_GTALK_TX_48000_256;
static struct adie_codec_action_unit headset_gtalk_rx_48KHz_osr256_actions[] =
	ADIE_HEADSET_GTALK_RX_48000_256;
static struct adie_codec_action_unit headset_gtalk_tx_48KHz_osr256_actions[] =
	ADIE_HEADSET_GTALK_TX_48000_256;
#endif
#ifdef CONFIG_MACH_ANCORA_TMO
static struct adie_codec_action_unit speaker_loopback_rx_48KHz_osr256_actions[] =
	ADIE_SPEAKER_LOOPBACK_RX_48000_256;
static struct adie_codec_action_unit speaker_loopback_tx_48KHz_osr256_actions[] =
	ADIE_SPEAKER_LOOPBACK_TX_48000_256;
#endif

static struct adie_codec_hwsetting_entry handset_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = handset_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(handset_rx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry handset_tx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = handset_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(handset_tx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry speaker_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = speaker_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(speaker_rx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry speaker_tx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = speaker_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(speaker_tx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry headset_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = headset_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(headset_rx_48KHz_osr256_actions),
	}
 };
static struct adie_codec_hwsetting_entry headset_tx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = headset_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(headset_tx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry aux_dock_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = aux_dock_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(aux_dock_rx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry speaker_headset_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = speaker_headset_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(speaker_headset_rx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry speaker_dock_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = speaker_dock_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(speaker_dock_rx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry speaker_hdmi_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = speaker_hdmi_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(speaker_hdmi_rx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry handset_call_rx_settings[] = {
	{
#ifdef CONFIG_VP_A2220_16KHZ
		.freq_plan = 16000,
#else
		.freq_plan = 48000,
#endif
		.osr = 256,
		.actions = handset_call_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(handset_call_rx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry handset_call_tx_settings[] = {
#if 0
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = imic_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(imic_8KHz_osr256_actions),
	},
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = imic_16KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(imic_16KHz_osr256_actions),
	},
#endif
	{
#ifdef CONFIG_VP_A2220_16KHZ
		.freq_plan = 16000,
#else
		.freq_plan = 48000,
#endif
		.osr = 256,
		.actions = handset_call_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(handset_call_tx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry speaker_call_rx_settings[] = {
	{
#ifdef CONFIG_VP_A2220_16KHZ
		.freq_plan = 16000,
#else
		.freq_plan = 48000,
#endif
		.osr = 256,
		.actions = speaker_call_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(speaker_call_rx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry speaker_call_tx_settings[] = {
#if 0
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = speaker_call_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(speaker_call_tx_8KHz_osr256_actions),
	},
	{ /* 8KHz profile is good for 16KHz */
		.freq_plan = 16000,
		.osr = 256,
		.actions = speaker_call_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(speaker_call_tx_8KHz_osr256_actions),
	},
#endif
	{
#ifdef CONFIG_VP_A2220_16KHZ
		.freq_plan = 16000,
#else
		.freq_plan = 48000,
#endif
		.osr = 256,
		.actions = speaker_call_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(speaker_call_tx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry headset_call_rx_settings[] = {
	{
#ifdef CONFIG_VP_A2220_16KHZ
		.freq_plan = 16000,
#else
		.freq_plan = 48000,
#endif
		.osr = 256,
		.actions = headset_call_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(headset_call_rx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry headset_call_tx_settings[] = {
#if 0
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = headset_call_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(headset_call_tx_8KHz_osr256_actions),
	},
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = headset_call_tx_16KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(headset_call_tx_16KHz_osr256_actions),
	},
#endif
	{
#ifdef CONFIG_VP_A2220_16KHZ
		.freq_plan = 16000,
#else
		.freq_plan = 48000,
#endif
		.osr = 256,
		.actions = headset_call_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(headset_call_tx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry tty_headset_mono_call_rx_settings[] = {
#if 0
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = tty_headset_mono_call_rx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(tty_headset_mono_call_rx_8KHz_osr256_actions),
	},
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = tty_headset_mono_call_rx_16KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(tty_headset_mono_call_rx_16KHz_osr256_actions),
	},
#endif
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = tty_headset_mono_call_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(tty_headset_mono_call_rx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry tty_headset_mono_call_tx_settings[] = {
	/* 8KHz, 16KHz, 48KHz TTY Tx devices can shared same set of actions */
#if 0
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = tty_headset_mono_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(tty_headset_mono_tx_8KHz_osr256_actions),
	},
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = tty_headset_mono_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(tty_headset_mono_tx_8KHz_osr256_actions),
	},
#endif
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = tty_headset_mono_call_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(tty_headset_mono_call_tx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry dualmic_handset_call_tx_settings[] = {
#if 0
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = dualmic_handset_call_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(dualmic_handset_call_tx_8KHz_osr256_actions),
	}, /* 8KHz profile can be used for 16KHz */
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = dualmic_handset_call_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(dualmic_handset_call_tx_8KHz_osr256_actions),
	}, /* 8KHz profile can be used for 48KHz */
#endif
	{
#ifdef CONFIG_VP_A2220_16KHZ
		.freq_plan = 16000,
#else
		.freq_plan = 48000,
#endif
		.osr = 256,
		.actions = dualmic_handset_call_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(dualmic_handset_call_tx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry handset_fmradio_rx_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = handset_fmradio_rx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(handset_fmradio_rx_8KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry headset_fmradio_rx_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = headset_fmradio_rx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(headset_fmradio_rx_8KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry speaker_fmradio_rx_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = speaker_fmradio_rx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(speaker_fmradio_rx_8KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry headset_fmradio_only_rx_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = headset_fmradio_only_rx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(headset_fmradio_only_rx_8KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry speaker_fmradio_only_rx_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = speaker_fmradio_only_rx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(speaker_fmradio_only_rx_8KHz_osr256_actions),
	}
};

static struct adie_codec_hwsetting_entry speaker_voice_dialer_tx_settings[] = {
#if 0
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = speaker_voice_dialer_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(speaker_voice_dialer_tx_8KHz_osr256_actions),
	},
	{ /* 8KHz profile is good for 16KHz */
		.freq_plan = 16000,
		.osr = 256,
		.actions = speaker_voice_dialer_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(speaker_voice_dialer_tx_8KHz_osr256_actions),
	},
#endif
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = speaker_voice_dialer_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(speaker_voice_dialer_tx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry headset_voice_dialer_tx_settings[] = {
#if 0
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = headset_voice_dialer_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(headset_voice_dialer_tx_8KHz_osr256_actions),
	},
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = headset_voice_dialer_tx_16KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(headset_voice_dialer_tx_16KHz_osr256_actions),
	},
#endif
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = headset_voice_dialer_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(headset_voice_dialer_tx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry speaker_voice_search_tx_settings[] = {
#if 0
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = speaker_voice_search_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(speaker_voice_search_tx_8KHz_osr256_actions),
	},
	{ /* 8KHz profile is good for 16KHz */
		.freq_plan = 16000,
		.osr = 256,
		.actions = speaker_voice_search_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(speaker_voice_search_tx_8KHz_osr256_actions),
	},
#endif
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = speaker_voice_search_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(speaker_voice_search_tx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry headset_voice_search_tx_settings[] = {
#if 0
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = headset_voice_search_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(headset_voice_search_tx_8KHz_osr256_actions),
	},
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = headset_voice_search_tx_16KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(headset_voice_search_tx_16KHz_osr256_actions),
	},
#endif
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = headset_voice_search_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(headset_voice_search_tx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry handset_voip_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = handset_voip_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(handset_voip_rx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry handset_voip_tx_settings[] = {
#if 0
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = imic_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(imic_8KHz_osr256_actions),
	},
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = imic_16KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(imic_16KHz_osr256_actions),
	},
#endif
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = handset_voip_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(handset_voip_tx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry speaker_voip_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = speaker_voip_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(speaker_voip_rx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry speaker_voip_tx_settings[] = {
#if 0
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = speaker_voip_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(speaker_voip_tx_8KHz_osr256_actions),
	},
	{ /* 8KHz profile is good for 16KHz */
		.freq_plan = 16000,
		.osr = 256,
		.actions = speaker_voip_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(speaker_voip_tx_8KHz_osr256_actions),
	},
#endif
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = speaker_voip_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(speaker_voip_tx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry headset_voip_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = headset_voip_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(headset_voip_rx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry headset_voip_tx_settings[] = {
#if 0
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = headset_voip_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(headset_voip_tx_8KHz_osr256_actions),
	},
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = headset_voip_tx_16KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(headset_voip_tx_16KHz_osr256_actions),
	},
#endif
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = headset_voip_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(headset_voip_tx_48KHz_osr256_actions),
	}
};
#ifdef CONFIG_MACH_ANCORA_TMO
static struct adie_codec_hwsetting_entry handset_call_hac_rx_settings[] = {
	{
#ifdef CONFIG_VP_A2220_16KHZ
		.freq_plan = 16000,
#else
		.freq_plan = 48000,
#endif
		.osr = 256,
		.actions = handset_call_hac_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(handset_call_hac_rx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry handset_call_hac_tx_settings[] = {
	{
#ifdef CONFIG_VP_A2220_16KHZ
		.freq_plan = 16000,
#else
		.freq_plan = 48000,
#endif
		.osr = 256,
		.actions = handset_call_hac_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(handset_call_hac_tx_48KHz_osr256_actions),
	}
};

#endif

#ifdef CONFIG_MACH_ANCORA_TMO
static struct adie_codec_hwsetting_entry handset_gan_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = handset_gan_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(handset_gan_rx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry handset_gan_tx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = handset_gan_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(handset_gan_tx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry speaker_gan_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = speaker_gan_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(speaker_gan_rx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry speaker_gan_tx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = speaker_gan_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(speaker_gan_tx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry headset_gan_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = headset_gan_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(headset_gan_rx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry headset_gan_tx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = headset_gan_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(headset_gan_tx_48KHz_osr256_actions),
	}
};
#endif
#if defined(CONFIG_MACH_ANCORA_TMO) || defined(CONFIG_MACH_APACHE)
static struct adie_codec_hwsetting_entry handset_gtalk_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = handset_gtalk_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(handset_gtalk_rx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry handset_gtalk_tx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = handset_gtalk_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(handset_gtalk_tx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry speaker_gtalk_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = speaker_gtalk_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(speaker_gtalk_rx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry speaker_gtalk_tx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = speaker_gtalk_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(speaker_gtalk_tx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry headset_gtalk_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = headset_gtalk_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(headset_gtalk_rx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry headset_gtalk_tx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = headset_gtalk_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(headset_gtalk_tx_48KHz_osr256_actions),
	}
};
#endif

#ifdef CONFIG_MACH_ANCORA_TMO
static struct adie_codec_hwsetting_entry speaker_loopback_rx_settings[] = {
	{
#ifdef CONFIG_VP_A2220_16KHZ
		.freq_plan = 16000,
#else
		.freq_plan = 48000,
#endif
		.osr = 256,
		.actions = speaker_loopback_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(speaker_loopback_rx_48KHz_osr256_actions),
	}
};
static struct adie_codec_hwsetting_entry speaker_loopback_tx_settings[] = {
	{
#ifdef CONFIG_VP_A2220_16KHZ
		.freq_plan = 16000,
#else
		.freq_plan = 48000,
#endif
		.osr = 256,
		.actions = speaker_loopback_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(speaker_loopback_tx_48KHz_osr256_actions),
	}
};
#endif

static struct adie_codec_dev_profile handset_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = handset_rx_settings,
	.setting_sz = ARRAY_SIZE(handset_rx_settings),
};
static struct adie_codec_dev_profile handset_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = handset_tx_settings,
	.setting_sz = ARRAY_SIZE(handset_tx_settings),
};
static struct adie_codec_dev_profile speaker_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = speaker_rx_settings,
	.setting_sz = ARRAY_SIZE(speaker_rx_settings),
};
static struct adie_codec_dev_profile speaker_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = speaker_tx_settings,
	.setting_sz = ARRAY_SIZE(speaker_tx_settings),
};
static struct adie_codec_dev_profile headset_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = headset_rx_settings,
	.setting_sz = ARRAY_SIZE(headset_rx_settings),
};
static struct adie_codec_dev_profile headset_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = headset_tx_settings,
	.setting_sz = ARRAY_SIZE(headset_tx_settings),
};
static struct adie_codec_dev_profile aux_dock_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = aux_dock_rx_settings,
	.setting_sz = ARRAY_SIZE(aux_dock_rx_settings),
};
static struct adie_codec_dev_profile speaker_headset_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = speaker_headset_rx_settings,
	.setting_sz = ARRAY_SIZE(speaker_headset_rx_settings),
};
static struct adie_codec_dev_profile speaker_dock_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = speaker_dock_rx_settings,
	.setting_sz = ARRAY_SIZE(speaker_dock_rx_settings),
};
static struct adie_codec_dev_profile speaker_hdmi_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = speaker_hdmi_rx_settings,
	.setting_sz = ARRAY_SIZE(speaker_hdmi_rx_settings),
};
static struct adie_codec_dev_profile handset_call_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = handset_call_rx_settings,
	.setting_sz = ARRAY_SIZE(handset_call_rx_settings),
};
static struct adie_codec_dev_profile handset_call_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = handset_call_tx_settings,
	.setting_sz = ARRAY_SIZE(handset_call_tx_settings),
};
static struct adie_codec_dev_profile speaker_call_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = speaker_call_rx_settings,
	.setting_sz = ARRAY_SIZE(speaker_call_rx_settings),
};
static struct adie_codec_dev_profile speaker_call_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = speaker_call_tx_settings,
	.setting_sz = ARRAY_SIZE(speaker_call_tx_settings),
};
static struct adie_codec_dev_profile headset_call_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = headset_call_rx_settings,
	.setting_sz = ARRAY_SIZE(headset_call_rx_settings),
};
static struct adie_codec_dev_profile headset_call_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = headset_call_tx_settings,
	.setting_sz = ARRAY_SIZE(headset_call_tx_settings),
};
static struct adie_codec_dev_profile tty_headset_mono_call_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = tty_headset_mono_call_rx_settings,
	.setting_sz = ARRAY_SIZE(tty_headset_mono_call_rx_settings),
};
static struct adie_codec_dev_profile tty_headset_mono_call_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = tty_headset_mono_call_tx_settings,
	.setting_sz = ARRAY_SIZE(tty_headset_mono_call_tx_settings),
};
static struct adie_codec_dev_profile dualmic_handset_call_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = dualmic_handset_call_tx_settings,
	.setting_sz = ARRAY_SIZE(dualmic_handset_call_tx_settings),
};
static struct adie_codec_dev_profile handset_fmradio_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = handset_fmradio_rx_settings,
	.setting_sz = ARRAY_SIZE(handset_fmradio_rx_settings),
};
static struct adie_codec_dev_profile headset_fmradio_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = headset_fmradio_rx_settings,
	.setting_sz = ARRAY_SIZE(headset_fmradio_rx_settings),
};
static struct adie_codec_dev_profile speaker_fmradio_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = speaker_fmradio_rx_settings,
	.setting_sz = ARRAY_SIZE(speaker_fmradio_rx_settings),
};
static struct adie_codec_dev_profile headset_fmradio_only_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = headset_fmradio_only_rx_settings,
	.setting_sz = ARRAY_SIZE(headset_fmradio_only_rx_settings),
};
static struct adie_codec_dev_profile speaker_fmradio_only_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = speaker_fmradio_only_rx_settings,
	.setting_sz = ARRAY_SIZE(speaker_fmradio_only_rx_settings),
};
static struct adie_codec_dev_profile speaker_voice_dialer_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = speaker_voice_dialer_tx_settings,
	.setting_sz = ARRAY_SIZE(speaker_voice_dialer_tx_settings),
};
static struct adie_codec_dev_profile headset_voice_dialer_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = headset_voice_dialer_tx_settings,
	.setting_sz = ARRAY_SIZE(headset_voice_dialer_tx_settings),
};
static struct adie_codec_dev_profile speaker_voice_search_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = speaker_voice_search_tx_settings,
	.setting_sz = ARRAY_SIZE(speaker_voice_search_tx_settings),
};
static struct adie_codec_dev_profile headset_voice_search_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = headset_voice_search_tx_settings,
	.setting_sz = ARRAY_SIZE(headset_voice_search_tx_settings),
};
static struct adie_codec_dev_profile handset_voip_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = handset_voip_rx_settings,
	.setting_sz = ARRAY_SIZE(handset_voip_rx_settings),
};
static struct adie_codec_dev_profile handset_voip_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = handset_voip_tx_settings,
	.setting_sz = ARRAY_SIZE(handset_voip_tx_settings),
};
static struct adie_codec_dev_profile speaker_voip_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = speaker_voip_rx_settings,
	.setting_sz = ARRAY_SIZE(speaker_voip_rx_settings),
};
static struct adie_codec_dev_profile speaker_voip_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = speaker_voip_tx_settings,
	.setting_sz = ARRAY_SIZE(speaker_voip_tx_settings),
};
static struct adie_codec_dev_profile headset_voip_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = headset_voip_rx_settings,
	.setting_sz = ARRAY_SIZE(headset_voip_rx_settings),
};
static struct adie_codec_dev_profile headset_voip_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = headset_voip_tx_settings,
	.setting_sz = ARRAY_SIZE(headset_voip_tx_settings),
};
#ifdef CONFIG_MACH_ANCORA_TMO
static struct adie_codec_dev_profile handset_call_hac_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = handset_call_hac_rx_settings,
	.setting_sz = ARRAY_SIZE(handset_call_hac_rx_settings),
};
static struct adie_codec_dev_profile handset_call_hac_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = handset_call_hac_tx_settings,
	.setting_sz = ARRAY_SIZE(handset_call_hac_tx_settings),
};
#endif
#ifdef CONFIG_MACH_ANCORA_TMO
static struct adie_codec_dev_profile handset_gan_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = handset_gan_rx_settings,
	.setting_sz = ARRAY_SIZE(handset_gan_rx_settings),
};
static struct adie_codec_dev_profile handset_gan_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = handset_gan_tx_settings,
	.setting_sz = ARRAY_SIZE(handset_gan_tx_settings),
};
static struct adie_codec_dev_profile speaker_gan_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = speaker_gan_rx_settings,
	.setting_sz = ARRAY_SIZE(speaker_gan_rx_settings),
};
static struct adie_codec_dev_profile speaker_gan_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = speaker_gan_tx_settings,
	.setting_sz = ARRAY_SIZE(speaker_gan_tx_settings),
};
static struct adie_codec_dev_profile headset_gan_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = headset_gan_rx_settings,
	.setting_sz = ARRAY_SIZE(headset_gan_rx_settings),
};
static struct adie_codec_dev_profile headset_gan_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = headset_gan_tx_settings,
	.setting_sz = ARRAY_SIZE(headset_gan_tx_settings),
};
#endif
#if defined(CONFIG_MACH_ANCORA_TMO) || defined(CONFIG_MACH_APACHE)
static struct adie_codec_dev_profile handset_gtalk_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = handset_gtalk_rx_settings,
	.setting_sz = ARRAY_SIZE(handset_gtalk_rx_settings),
};
static struct adie_codec_dev_profile handset_gtalk_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = handset_gtalk_tx_settings,
	.setting_sz = ARRAY_SIZE(handset_gtalk_tx_settings),
};
static struct adie_codec_dev_profile speaker_gtalk_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = speaker_gtalk_rx_settings,
	.setting_sz = ARRAY_SIZE(speaker_gtalk_rx_settings),
};
static struct adie_codec_dev_profile speaker_gtalk_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = speaker_gtalk_tx_settings,
	.setting_sz = ARRAY_SIZE(speaker_gtalk_tx_settings),
};
static struct adie_codec_dev_profile headset_gtalk_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = headset_gtalk_rx_settings,
	.setting_sz = ARRAY_SIZE(headset_gtalk_rx_settings),
};
static struct adie_codec_dev_profile headset_gtalk_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = headset_gtalk_tx_settings,
	.setting_sz = ARRAY_SIZE(headset_gtalk_tx_settings),
};

#endif

#ifdef CONFIG_MACH_ANCORA_TMO
static struct adie_codec_dev_profile speaker_loopback_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = speaker_loopback_rx_settings,
	.setting_sz = ARRAY_SIZE(speaker_loopback_rx_settings),
};
static struct adie_codec_dev_profile speaker_loopback_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = speaker_loopback_tx_settings,
	.setting_sz = ARRAY_SIZE(speaker_loopback_tx_settings),
};
#endif


static struct snddev_icodec_data handset_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "handset_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_RX,
	.profile = &handset_rx_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
	.max_voice_rx_vol[VOC_NB_INDEX] = -200,
	.min_voice_rx_vol[VOC_NB_INDEX] = -1700,
	.max_voice_rx_vol[VOC_WB_INDEX] = -200,
	.min_voice_rx_vol[VOC_WB_INDEX] = -1700
};
static enum hsed_controller handset_tx_pmctl_id[] = {PM_HSED_CONTROLLER_0};
static struct snddev_icodec_data handset_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_TX,
	.profile = &handset_tx_profile,
	.channel_mode = 1,
	.pmctl_id = handset_tx_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(handset_tx_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};
static struct snddev_icodec_data speaker_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "speaker_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPEAKER_RX,
	.profile = &speaker_rx_profile,
	.channel_mode = 2,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = &msm_snddev_poweramp_on_speaker,
	.pamp_off = &msm_snddev_poweramp_off_speaker,
	.max_voice_rx_vol[VOC_NB_INDEX] = 1000,
	.min_voice_rx_vol[VOC_NB_INDEX] = -500,
	.max_voice_rx_vol[VOC_WB_INDEX] = 1000,
	.min_voice_rx_vol[VOC_WB_INDEX] = -500,
};
static enum hsed_controller spk_pmctl_id[] = {PM_HSED_CONTROLLER_0};
static struct snddev_icodec_data speaker_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "speaker_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPEAKER_TX,
	.profile = &speaker_tx_profile,
	.channel_mode = 1,
	.pmctl_id = spk_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(spk_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};
static struct snddev_icodec_data headset_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_RX,
	.profile = &headset_rx_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_poweramp_on_headset,
	.pamp_off = msm_snddev_poweramp_off_headset,
	.max_voice_rx_vol[VOC_NB_INDEX] = -700,
	.min_voice_rx_vol[VOC_NB_INDEX] = -2200,
	.max_voice_rx_vol[VOC_WB_INDEX] = -900,
	.min_voice_rx_vol[VOC_WB_INDEX] = -2400
};
static struct snddev_icodec_data headset_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "headset_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_TX,
	.profile = &headset_tx_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
#if defined(CONFIG_MACH_ANCORA) || defined(CONFIG_MACH_ANCORA_USA) || defined(CONFIG_MACH_APACHE)
	.pamp_on = msm_snddev_tx_ear_route_config,
	.pamp_off = msm_snddev_tx_ear_route_deconfig,
#else
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
#endif
};
static struct snddev_ecodec_data bt_sco_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "bt_sco_rx",
	.copp_id = 1,
	.acdb_id = ACDB_ID_BT_SCO_RX,
	.channel_mode = 1,
	.conf_pcm_ctl_val = BT_SCO_PCM_CTL_VAL,
	.conf_aux_codec_intf = BT_SCO_AUX_CODEC_INTF,
	.conf_data_format_padding_val = BT_SCO_DATA_FORMAT_PADDING,
	.max_voice_rx_vol[VOC_NB_INDEX] = 400,
	.min_voice_rx_vol[VOC_NB_INDEX] = -1100,
	.max_voice_rx_vol[VOC_WB_INDEX] = 400,
	.min_voice_rx_vol[VOC_WB_INDEX] = -1100,
};
static struct snddev_ecodec_data bt_sco_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "bt_sco_tx",
	.copp_id = 1,
	.acdb_id = ACDB_ID_BT_SCO_TX,
	.channel_mode = 1,
	.conf_pcm_ctl_val = BT_SCO_PCM_CTL_VAL,
	.conf_aux_codec_intf = BT_SCO_AUX_CODEC_INTF,
	.conf_data_format_padding_val = BT_SCO_DATA_FORMAT_PADDING,
};
static struct snddev_ecodec_data bt_sco_nrec_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "bt_sco_nrec_rx",
	.copp_id = 1,
	.acdb_id = ACDB_ID_BT_SCO_NREC_RX,
	.channel_mode = 1,
	.conf_pcm_ctl_val = BT_SCO_PCM_CTL_VAL,
	.conf_aux_codec_intf = BT_SCO_AUX_CODEC_INTF,
	.conf_data_format_padding_val = BT_SCO_DATA_FORMAT_PADDING,
	.max_voice_rx_vol[VOC_NB_INDEX] = 400,
	.min_voice_rx_vol[VOC_NB_INDEX] = -1100,
	.max_voice_rx_vol[VOC_WB_INDEX] = 400,
	.min_voice_rx_vol[VOC_WB_INDEX] = -1100,
};
static struct snddev_ecodec_data bt_sco_nrec_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "bt_sco_nrec_tx",
	.copp_id = 1,
	.acdb_id = ACDB_ID_BT_SCO_NREC_TX,
	.channel_mode = 1,
	.conf_pcm_ctl_val = BT_SCO_PCM_CTL_VAL,
	.conf_aux_codec_intf = BT_SCO_AUX_CODEC_INTF,
	.conf_data_format_padding_val = BT_SCO_DATA_FORMAT_PADDING,
};
static struct snddev_mi2s_data hdmi_stereo_rx_data = {
	.capability = SNDDEV_CAP_RX ,
	.name = "hdmi_stereo_rx",
	.copp_id = 3,
	.acdb_id = ACDB_ID_HDMI_STEREO_RX,
	.channel_mode = 2,
	.sd_lines = MI2S_SD_0,
	.route = msm_snddev_tx_route_config,
	.deroute = msm_snddev_tx_route_deconfig,
	.default_sample_rate = 48000,
};
static struct snddev_icodec_data aux_dock_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "aux_dock_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_AUX_DOCK_RX,
	.profile = &aux_dock_rx_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_poweramp_on_dock,
	.pamp_off = msm_snddev_poweramp_off_dock,
	.max_voice_rx_vol[VOC_NB_INDEX] = -700,
	.min_voice_rx_vol[VOC_NB_INDEX] = -2200,
	.max_voice_rx_vol[VOC_WB_INDEX] = -900,
	.min_voice_rx_vol[VOC_WB_INDEX] = -2400
};
static struct snddev_icodec_data speaker_headset_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "speaker_headset_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPEAKER_HEADSET_RX,
	.profile = &speaker_headset_rx_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_poweramp_on_together,
	.pamp_off = msm_snddev_poweramp_off_together,
	.voltage_on = msm_snddev_hsed_voltage_on,
	.voltage_off = msm_snddev_hsed_voltage_off,
	.max_voice_rx_vol[VOC_NB_INDEX] = -500,
	.min_voice_rx_vol[VOC_NB_INDEX] = -2000,
	.max_voice_rx_vol[VOC_WB_INDEX] = -500,
	.min_voice_rx_vol[VOC_WB_INDEX] = -2000,
};
static struct snddev_icodec_data speaker_dock_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "speaker_dock_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPEAKER_DOCK_RX,
	.profile = &speaker_dock_rx_profile,
	.channel_mode = 2,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = &msm_snddev_poweramp_on_speaker,
	.pamp_off = &msm_snddev_poweramp_off_speaker,
	.max_voice_rx_vol[VOC_NB_INDEX] = 1000,
	.min_voice_rx_vol[VOC_NB_INDEX] = -500,
	.max_voice_rx_vol[VOC_WB_INDEX] = 1000,
	.min_voice_rx_vol[VOC_WB_INDEX] = -500,
};
static struct snddev_icodec_data speaker_hdmi_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "speaker_hdmi_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPEAKER_HDMI_RX,
	.profile = &speaker_hdmi_rx_profile,
	.channel_mode = 2,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = &msm_snddev_poweramp_on_speaker,
	.pamp_off = &msm_snddev_poweramp_off_speaker,
	.max_voice_rx_vol[VOC_NB_INDEX] = 1000,
	.min_voice_rx_vol[VOC_NB_INDEX] = -500,
	.max_voice_rx_vol[VOC_WB_INDEX] = 1000,
	.min_voice_rx_vol[VOC_WB_INDEX] = -500,
};
static struct snddev_icodec_data handset_call_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "handset_call_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_CALL_RX,
	.profile = &handset_call_rx_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
#ifdef CONFIG_VP_A2220_16KHZ
	.default_sample_rate = 16000,
#else
	.default_sample_rate = 48000,
#endif
#ifdef CONFIG_VP_A2220
	.pamp_on = msm_snddev_setting_audience_call_connect,
	.pamp_off = msm_snddev_setting_audience_call_disconnect,
#else
	.pamp_on = NULL,
	.pamp_off = NULL,
#endif
//	.property = SIDE_TONE_MASK,
	.max_voice_rx_vol[VOC_NB_INDEX] = -200,
	.min_voice_rx_vol[VOC_NB_INDEX] = -1700,
	.max_voice_rx_vol[VOC_WB_INDEX] = -200,
	.min_voice_rx_vol[VOC_WB_INDEX] = -1700
};
static enum hsed_controller handset_call_tx_pmctl_id[] = {PM_HSED_CONTROLLER_0};
static struct snddev_icodec_data handset_call_tx_data = {
#ifdef CONFIG_VP_A2220
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_call_tx",
	.copp_id = 0,	//PRIMARY_I2S_TX,	// mdhwang_Test
	.acdb_id = ACDB_ID_HANDSET_CALL_TX,
	.profile = &dualmic_handset_call_tx_profile,
	.channel_mode = 2,
	.pmctl_id = handset_call_tx_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(handset_call_tx_pmctl_id),
#ifdef CONFIG_VP_A2220_16KHZ
	.default_sample_rate = 16000,
#else
	.default_sample_rate = 48000,
#endif
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
#else
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_call_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_CALL_TX,
	.profile = &handset_call_tx_profile,
	.channel_mode = 1,
	.pmctl_id = handset_call_tx_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(handset_call_tx_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
#endif
};
static struct snddev_icodec_data speaker_call_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "speaker_call_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPEAKER_CALL_RX,
	.profile = &speaker_call_rx_profile,
	.channel_mode = 2,
	.pmctl_id_sz = 0,
#ifdef CONFIG_VP_A2220_16KHZ
	.default_sample_rate = 16000,
#else
	.default_sample_rate = 48000,
#endif
	.pamp_on = &msm_snddev_poweramp_on_speaker_call,
	.pamp_off = &msm_snddev_poweramp_off_speaker_call,
//	.property = SIDE_TONE_MASK,
	.max_voice_rx_vol[VOC_NB_INDEX] = 1000,
	.min_voice_rx_vol[VOC_NB_INDEX] = -500,
	.max_voice_rx_vol[VOC_WB_INDEX] = 1000,
	.min_voice_rx_vol[VOC_WB_INDEX] = -500,
};
static enum hsed_controller spk_call_pmctl_id[] = {PM_HSED_CONTROLLER_0};
static struct snddev_icodec_data speaker_call_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "speaker_call_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPEAKER_CALL_TX,
	.profile = &speaker_call_tx_profile,
	.channel_mode = 1,
	.pmctl_id = spk_call_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(spk_call_pmctl_id),
#ifdef CONFIG_VP_A2220_16KHZ
	.default_sample_rate = 16000,
#else
	.default_sample_rate = 48000,
#endif
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};
static struct snddev_icodec_data headset_call_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_call_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_CALL_RX,
	.profile = &headset_call_rx_profile,
	.channel_mode = 2,
#ifdef CONFIG_VP_A2220_16KHZ
	.default_sample_rate = 16000,
#else
	.default_sample_rate = 48000,
#endif
	.pamp_on = msm_snddev_poweramp_on_headset_call,
	.pamp_off = msm_snddev_poweramp_off_headset_call,
//	.property = SIDE_TONE_MASK,
	.max_voice_rx_vol[VOC_NB_INDEX] = -700,
	.min_voice_rx_vol[VOC_NB_INDEX] = -2200,
	.max_voice_rx_vol[VOC_WB_INDEX] = -900,
	.min_voice_rx_vol[VOC_WB_INDEX] = -2400
};
static struct snddev_icodec_data headset_call_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "headset_call_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_CALL_TX,
	.profile = &headset_call_tx_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
#ifdef CONFIG_VP_A2220_16KHZ
	.default_sample_rate = 16000,
#else
	.default_sample_rate = 48000,
#endif
#if defined(CONFIG_MACH_ANCORA) || defined(CONFIG_MACH_ANCORA_TMO) || defined(CONFIG_MACH_APACHE)
	.pamp_on = msm_snddev_tx_ear_route_config,
	.pamp_off = msm_snddev_tx_ear_route_deconfig,
#else
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
#endif
};
static struct snddev_ecodec_data bt_sco_call_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "bt_sco_call_rx",
	.copp_id = 1,
	.acdb_id = ACDB_ID_BT_SCO_CALL_RX,
	.channel_mode = 1,
	.conf_pcm_ctl_val = BT_SCO_PCM_CTL_VAL,
	.conf_aux_codec_intf = BT_SCO_AUX_CODEC_INTF,
	.conf_data_format_padding_val = BT_SCO_DATA_FORMAT_PADDING,
	.max_voice_rx_vol[VOC_NB_INDEX] = 400,
	.min_voice_rx_vol[VOC_NB_INDEX] = -1100,
	.max_voice_rx_vol[VOC_WB_INDEX] = 400,
	.min_voice_rx_vol[VOC_WB_INDEX] = -1100,
};
static struct snddev_ecodec_data bt_sco_call_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "bt_sco_call_tx",
	.copp_id = 1,
	.acdb_id = ACDB_ID_BT_SCO_CALL_TX,
	.channel_mode = 1,
	.conf_pcm_ctl_val = BT_SCO_PCM_CTL_VAL,
	.conf_aux_codec_intf = BT_SCO_AUX_CODEC_INTF,
	.conf_data_format_padding_val = BT_SCO_DATA_FORMAT_PADDING,
};
static struct snddev_ecodec_data bt_sco_nrec_call_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "bt_sco_nrec_call_rx",
	.copp_id = 1,
	.acdb_id = ACDB_ID_BT_SCO_NREC_CALL_RX,
	.channel_mode = 1,
	.conf_pcm_ctl_val = BT_SCO_PCM_CTL_VAL,
	.conf_aux_codec_intf = BT_SCO_AUX_CODEC_INTF,
	.conf_data_format_padding_val = BT_SCO_DATA_FORMAT_PADDING,
	.max_voice_rx_vol[VOC_NB_INDEX] = 400,
	.min_voice_rx_vol[VOC_NB_INDEX] = -1100,
	.max_voice_rx_vol[VOC_WB_INDEX] = 400,
	.min_voice_rx_vol[VOC_WB_INDEX] = -1100,
};
static struct snddev_ecodec_data bt_sco_nrec_call_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "bt_sco_nrec_call_tx",
	.copp_id = 1,
	.acdb_id = ACDB_ID_BT_SCO_NREC_CALL_TX,
	.channel_mode = 1,
	.conf_pcm_ctl_val = BT_SCO_PCM_CTL_VAL,
	.conf_aux_codec_intf = BT_SCO_AUX_CODEC_INTF,
	.conf_data_format_padding_val = BT_SCO_DATA_FORMAT_PADDING,
};
static struct snddev_icodec_data tty_headset_mono_call_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE | SNDDEV_CAP_TTY),
	.name = "tty_headset_mono_call_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_TTY_HEADSET_MONO_CALL_RX,
	.profile = &tty_headset_mono_call_rx_profile,
	.channel_mode = 1,
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_poweramp_on_tty,
	.pamp_off = msm_snddev_poweramp_off_tty,
	.max_voice_rx_vol[VOC_NB_INDEX] = 0,
	.min_voice_rx_vol[VOC_NB_INDEX] = 0,
	.max_voice_rx_vol[VOC_WB_INDEX] = 0,
	.min_voice_rx_vol[VOC_WB_INDEX] = 0,
};
static struct snddev_icodec_data tty_headset_mono_call_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE | SNDDEV_CAP_TTY),
	.name = "tty_headset_mono_call_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_TTY_HEADSET_MONO_CALL_TX,
	.profile = &tty_headset_mono_call_tx_profile,
	.channel_mode = 1,
	.default_sample_rate = 48000,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.pamp_on = NULL,
	.pamp_off = NULL,
};
static enum hsed_controller dualmic_pmctl_id[] = {
	PM_HSED_CONTROLLER_0, PM_HSED_CONTROLLER_2
};
static struct snddev_icodec_data dualmic_handset_call_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "dualmic_handset_call_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_DUALMIC_HANDSET_CALL_TX,
	.profile = &dualmic_handset_call_tx_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pmctl_id = dualmic_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(dualmic_pmctl_id),
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};
static struct snddev_icodec_data handset_fmradio_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_FM),
	.name = "handset_fmradio_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_FMRADIO_RX,
	.profile = &handset_fmradio_rx_profile,
	.channel_mode = 1,
	.default_sample_rate = 8000,
	.pamp_on = NULL,
	.pamp_off = NULL,
	.max_voice_rx_vol[VOC_NB_INDEX] = -200,
	.min_voice_rx_vol[VOC_NB_INDEX] = -1700,
	.max_voice_rx_vol[VOC_WB_INDEX] = -200,
	.min_voice_rx_vol[VOC_WB_INDEX] = -1700
};
static struct snddev_icodec_data headset_fmradio_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_FM),
	.name = "headset_fmradio_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_FMRADIO_RX,
	.profile = &headset_fmradio_rx_profile,
	.channel_mode = 2,
	.default_sample_rate = 8000,
	.pamp_on = msm_snddev_poweramp_on_headset,
	.pamp_off = msm_snddev_poweramp_off_headset,
	.max_voice_rx_vol[VOC_NB_INDEX] = -700,
	.min_voice_rx_vol[VOC_NB_INDEX] = -2200,
	.max_voice_rx_vol[VOC_WB_INDEX] = -900,
	.min_voice_rx_vol[VOC_WB_INDEX] = -2400
};
static struct snddev_mi2s_data headset_fmradio_tx_data = {
	.capability = SNDDEV_CAP_TX ,
	.name = "headset_fmradio_tx",
	.copp_id = 2,
	.acdb_id = ACDB_ID_HEADSET_FMRADIO_TX,
	.channel_mode = 2,
	.sd_lines = MI2S_SD_3,
	.route = NULL,
	.deroute = NULL,
	.default_sample_rate = 48000,
};
static struct snddev_icodec_data headset_fmradio_only_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_FM),
	.name = "headset_fmradio_only_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_FMRADIO_ONLY_RX,
	.profile = &headset_fmradio_only_rx_profile,
	.channel_mode = 1,
	.default_sample_rate = 8000,
	.pamp_on = msm_snddev_poweramp_on_headset,
	.pamp_off = msm_snddev_poweramp_off_headset,
	.dev_vol_type = SNDDEV_DEV_VOL_DIGITAL,
};
static struct snddev_icodec_data speaker_fmradio_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_FM),
	.name = "speaker_fmradio_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPEAKER_FMRADIO_RX,
	.profile = &speaker_fmradio_rx_profile,
	.channel_mode = 1,
	.default_sample_rate = 8000,
	.pamp_on = &msm_snddev_poweramp_on_speaker,
	.pamp_off = &msm_snddev_poweramp_off_speaker,
	.max_voice_rx_vol[VOC_NB_INDEX] = 1000,
	.min_voice_rx_vol[VOC_NB_INDEX] = -500,
	.max_voice_rx_vol[VOC_WB_INDEX] = 1000,
	.min_voice_rx_vol[VOC_WB_INDEX] = -500,
};
static struct snddev_icodec_data speaker_fmradio_only_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_FM),
	.name = "speaker_fmradio_only_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPEAKER_FMRADIO_ONLY_RX,
	.profile = &speaker_fmradio_only_rx_profile,
	.channel_mode = 1,
	.default_sample_rate = 8000,
	.pamp_on = &msm_snddev_poweramp_on_speaker,
	.pamp_off = &msm_snddev_poweramp_off_speaker,
	.dev_vol_type = SNDDEV_DEV_VOL_DIGITAL,
};
static enum hsed_controller spk_dialer_pmctl_id[] = {PM_HSED_CONTROLLER_0};
static struct snddev_icodec_data speaker_voice_dialer_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "speaker_voice_dialer_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPEAKER_VOICE_DIALER_TX,
	.profile = &speaker_voice_dialer_tx_profile,
	.channel_mode = 1,
	.pmctl_id = spk_dialer_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(spk_dialer_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};
static struct snddev_icodec_data headset_voice_dialer_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "headset_voice_dialer_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_VOICE_DIALER_TX,
	.profile = &headset_voice_dialer_tx_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
#if defined(CONFIG_MACH_ANCORA) || defined(CONFIG_MACH_ANCORA_TMO) || defined(CONFIG_MACH_APACHE)
	.pamp_on = msm_snddev_tx_ear_route_config,
	.pamp_off = msm_snddev_tx_ear_route_deconfig,
#else
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
#endif
};
static struct snddev_ecodec_data bt_sco_voice_dialer_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "bt_sco_voice_dialer_tx",
	.copp_id = 1,
	.acdb_id = ACDB_ID_BT_SCO_VOICE_DIALER_TX,
	.channel_mode = 1,
	.conf_pcm_ctl_val = BT_SCO_PCM_CTL_VAL,
	.conf_aux_codec_intf = BT_SCO_AUX_CODEC_INTF,
	.conf_data_format_padding_val = BT_SCO_DATA_FORMAT_PADDING,
};
static struct snddev_ecodec_data bt_sco_nrec_voice_dialer_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "bt_sco_nrec_voice_dialer_tx",
	.copp_id = 1,
	.acdb_id = ACDB_ID_BT_SCO_NREC_VOICE_DIALER_TX,
	.channel_mode = 1,
	.conf_pcm_ctl_val = BT_SCO_PCM_CTL_VAL,
	.conf_aux_codec_intf = BT_SCO_AUX_CODEC_INTF,
	.conf_data_format_padding_val = BT_SCO_DATA_FORMAT_PADDING,
};
static enum hsed_controller spk_search_pmctl_id[] = {PM_HSED_CONTROLLER_0};
static struct snddev_icodec_data speaker_voice_search_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "speaker_voice_search_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPEAKER_VOICE_SEARCH_TX,
	.profile = &speaker_voice_search_tx_profile,
	.channel_mode = 1,
	.pmctl_id = spk_search_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(spk_search_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};
static struct snddev_icodec_data headset_voice_search_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "headset_voice_search_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_VOICE_SEARCH_TX,
	.profile = &headset_voice_search_tx_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
#if defined(CONFIG_MACH_ANCORA) || defined(CONFIG_MACH_ANCORA_TMO) || defined(CONFIG_MACH_APACHE)
	.pamp_on = msm_snddev_tx_ear_route_config,
	.pamp_off = msm_snddev_tx_ear_route_deconfig,
#else
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
#endif
};
static struct snddev_icodec_data handset_voip_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "handset_voip_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_VOIP_RX,
	.profile = &handset_voip_rx_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
	.max_voice_rx_vol[VOC_NB_INDEX] = -200,
	.min_voice_rx_vol[VOC_NB_INDEX] = -1700,
	.max_voice_rx_vol[VOC_WB_INDEX] = -200,
	.min_voice_rx_vol[VOC_WB_INDEX] = -1700
};
static enum hsed_controller handset_voip_tx_pmctl_id[] = {PM_HSED_CONTROLLER_0};
static struct snddev_icodec_data handset_voip_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_voip_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_VOIP_TX,
	.profile = &handset_voip_tx_profile,
	.channel_mode = 1,
	.pmctl_id = handset_voip_tx_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(handset_voip_tx_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};
static struct snddev_icodec_data speaker_voip_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "speaker_voip_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPEAKER_VOIP_RX,
	.profile = &speaker_voip_rx_profile,
	.channel_mode = 2,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = &msm_snddev_poweramp_on_speaker,
	.pamp_off = &msm_snddev_poweramp_off_speaker,
	.max_voice_rx_vol[VOC_NB_INDEX] = 1000,
	.min_voice_rx_vol[VOC_NB_INDEX] = -500,
	.max_voice_rx_vol[VOC_WB_INDEX] = 1000,
	.min_voice_rx_vol[VOC_WB_INDEX] = -500,
};
static enum hsed_controller spk_voip_pmctl_id[] = {PM_HSED_CONTROLLER_0};
static struct snddev_icodec_data speaker_voip_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "speaker_voip_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPEAKER_VOIP_TX,
	.profile = &speaker_voip_tx_profile,
	.channel_mode = 1,
	.pmctl_id = spk_voip_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(spk_voip_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};
static struct snddev_icodec_data headset_voip_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_voip_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_VOIP_RX,
	.profile = &headset_voip_rx_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_poweramp_on_headset,
	.pamp_off = msm_snddev_poweramp_off_headset,
	.max_voice_rx_vol[VOC_NB_INDEX] = -700,
	.min_voice_rx_vol[VOC_NB_INDEX] = -2200,
	.max_voice_rx_vol[VOC_WB_INDEX] = -900,
	.min_voice_rx_vol[VOC_WB_INDEX] = -2400
};
static struct snddev_icodec_data headset_voip_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "headset_voip_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_VOIP_TX,
	.profile = &headset_voip_tx_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
#if defined(CONFIG_MACH_ANCORA) || defined(CONFIG_MACH_ANCORA_USA) || defined(CONFIG_MACH_APACHE)
	.pamp_on = msm_snddev_tx_ear_route_config,
	.pamp_off = msm_snddev_tx_ear_route_deconfig,
#else
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
#endif
};
static struct snddev_ecodec_data bt_sco_voip_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "bt_sco_voip_rx",
	.copp_id = 1,
	.acdb_id = ACDB_ID_BT_SCO_VOIP_RX,
	.channel_mode = 1,
	.conf_pcm_ctl_val = BT_SCO_PCM_CTL_VAL,
	.conf_aux_codec_intf = BT_SCO_AUX_CODEC_INTF,
	.conf_data_format_padding_val = BT_SCO_DATA_FORMAT_PADDING,
	.max_voice_rx_vol[VOC_NB_INDEX] = 400,
	.min_voice_rx_vol[VOC_NB_INDEX] = -1100,
	.max_voice_rx_vol[VOC_WB_INDEX] = 400,
	.min_voice_rx_vol[VOC_WB_INDEX] = -1100,
};
static struct snddev_ecodec_data bt_sco_voip_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "bt_sco_voip_tx",
	.copp_id = 1,
	.acdb_id = ACDB_ID_BT_SCO_VOIP_TX,
	.channel_mode = 1,
	.conf_pcm_ctl_val = BT_SCO_PCM_CTL_VAL,
	.conf_aux_codec_intf = BT_SCO_AUX_CODEC_INTF,
	.conf_data_format_padding_val = BT_SCO_DATA_FORMAT_PADDING,
};
#ifdef CONFIG_MACH_ANCORA_TMO
static struct snddev_icodec_data handset_call_hac_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "handset_call_hac_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_CALL_HAC_RX,
	.profile = &handset_call_hac_rx_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
#ifdef CONFIG_VP_A2220_16KHZ
	.default_sample_rate = 16000,
#else
	.default_sample_rate = 48000,
#endif
#ifdef CONFIG_VP_A2220
	.pamp_on = msm_snddev_setting_audience_call_connect,
	.pamp_off = msm_snddev_setting_audience_call_disconnect,
#else
	.pamp_on = NULL,
	.pamp_off = NULL,
#endif
//	.property = SIDE_TONE_MASK,
	.max_voice_rx_vol[VOC_NB_INDEX] = -200,
	.min_voice_rx_vol[VOC_NB_INDEX] = -1700,
	.max_voice_rx_vol[VOC_WB_INDEX] = -200,
	.min_voice_rx_vol[VOC_WB_INDEX] = -1700
};
static struct snddev_icodec_data handset_call_hac_tx_data = {
#ifdef CONFIG_VP_A2220
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_call_hac_tx",
	.copp_id = 0,	//PRIMARY_I2S_TX,	// mdhwang_Test
	.acdb_id = ACDB_ID_HANDSET_CALL_HAC_TX,
	.profile = &dualmic_handset_call_tx_profile,
	.channel_mode = 2,
	.pmctl_id = handset_call_tx_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(handset_call_tx_pmctl_id),
#ifdef CONFIG_VP_A2220_16KHZ
	.default_sample_rate = 16000,
#else
	.default_sample_rate = 48000,
#endif
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
#else
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_call_hac_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_CALL_HAC_TX,
	.profile = &handset_call_hac_tx_profile,
	.channel_mode = 1,
	.pmctl_id = handset_call_tx_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(handset_call_tx_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
#endif
};
#endif

#ifdef CONFIG_MACH_ANCORA_TMO
static struct snddev_icodec_data handset_gan_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "handset_ganlite_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_GAN_RX,
	.profile = &handset_gan_rx_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
	.max_voice_rx_vol[VOC_NB_INDEX] = -200,
	.min_voice_rx_vol[VOC_NB_INDEX] = -1700,
	.max_voice_rx_vol[VOC_WB_INDEX] = -200,
	.min_voice_rx_vol[VOC_WB_INDEX] = -1700
};
static enum hsed_controller handset_gan_tx_pmctl_id[] = {PM_HSED_CONTROLLER_0};
static struct snddev_icodec_data handset_gan_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_ganlite_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_GAN_TX,
	.profile = &handset_gan_tx_profile,
	.channel_mode = 1,
	.pmctl_id = handset_gan_tx_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(handset_gan_tx_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};
static struct snddev_icodec_data speaker_gan_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "speaker_ganlite_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPEAKER_GAN_RX,
	.profile = &speaker_gan_rx_profile,
	.channel_mode = 2,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = &msm_snddev_poweramp_on_speaker,
	.pamp_off = &msm_snddev_poweramp_off_speaker,
	.max_voice_rx_vol[VOC_NB_INDEX] = 1000,
	.min_voice_rx_vol[VOC_NB_INDEX] = -500,
	.max_voice_rx_vol[VOC_WB_INDEX] = 1000,
	.min_voice_rx_vol[VOC_WB_INDEX] = -500,
};
static enum hsed_controller spk_gan_pmctl_id[] = {PM_HSED_CONTROLLER_0};
static struct snddev_icodec_data speaker_gan_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "speaker_ganlite_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPEAKER_GAN_TX,
	.profile = &speaker_gan_tx_profile,
	.channel_mode = 1,
	.pmctl_id = spk_gan_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(spk_gan_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};
static struct snddev_icodec_data headset_gan_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_ganlite_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_GAN_RX,
	.profile = &headset_gan_rx_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_poweramp_on_headset,
	.pamp_off = msm_snddev_poweramp_off_headset,
	.max_voice_rx_vol[VOC_NB_INDEX] = -700,
	.min_voice_rx_vol[VOC_NB_INDEX] = -2200,
	.max_voice_rx_vol[VOC_WB_INDEX] = -900,
	.min_voice_rx_vol[VOC_WB_INDEX] = -2400
};
static struct snddev_icodec_data headset_gan_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "headset_ganlite_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_GAN_TX,
	.profile = &headset_gan_tx_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
#if defined(CONFIG_MACH_ANCORA) || defined(CONFIG_MACH_ANCORA_USA) || defined(CONFIG_MACH_APACHE)
	.pamp_on = msm_snddev_tx_ear_route_config,
	.pamp_off = msm_snddev_tx_ear_route_deconfig,
#else
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
#endif
};
static struct snddev_ecodec_data bt_sco_gan_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "bt_sco_ganlite_rx",
	.copp_id = 1,
	.acdb_id = ACDB_ID_BT_SCO_GAN_RX,
	.channel_mode = 1,
	.conf_pcm_ctl_val = BT_SCO_PCM_CTL_VAL,
	.conf_aux_codec_intf = BT_SCO_AUX_CODEC_INTF,
	.conf_data_format_padding_val = BT_SCO_DATA_FORMAT_PADDING,
	.max_voice_rx_vol[VOC_NB_INDEX] = 400,
	.min_voice_rx_vol[VOC_NB_INDEX] = -1100,
	.max_voice_rx_vol[VOC_WB_INDEX] = 400,
	.min_voice_rx_vol[VOC_WB_INDEX] = -1100,
};
static struct snddev_ecodec_data bt_sco_gan_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "bt_sco_ganlite_tx",
	.copp_id = 1,
	.acdb_id = ACDB_ID_BT_SCO_GAN_TX,
	.channel_mode = 1,
	.conf_pcm_ctl_val = BT_SCO_PCM_CTL_VAL,
	.conf_aux_codec_intf = BT_SCO_AUX_CODEC_INTF,
	.conf_data_format_padding_val = BT_SCO_DATA_FORMAT_PADDING,
};
#endif
#if defined(CONFIG_MACH_ANCORA_TMO) || defined(CONFIG_MACH_APACHE)
static struct snddev_icodec_data handset_gtalk_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "handset_gtalk_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_GTALK_RX,
	.profile = &handset_gtalk_rx_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
	.max_voice_rx_vol[VOC_NB_INDEX] = -200,
	.min_voice_rx_vol[VOC_NB_INDEX] = -1700,
	.max_voice_rx_vol[VOC_WB_INDEX] = -200,
	.min_voice_rx_vol[VOC_WB_INDEX] = -1700
};
static enum hsed_controller handset_gtalk_tx_pmctl_id[] = {PM_HSED_CONTROLLER_0};
static struct snddev_icodec_data handset_gtalk_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_gtalk_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_GTALK_TX,
	.profile = &handset_gtalk_tx_profile,
	.channel_mode = 1,
	.pmctl_id = handset_gtalk_tx_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(handset_gtalk_tx_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};
static struct snddev_icodec_data speaker_gtalk_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "speaker_gtalk_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPEAKER_GTALK_RX,
	.profile = &speaker_gtalk_rx_profile,
	.channel_mode = 2,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = &msm_snddev_poweramp_on_speaker,
	.pamp_off = &msm_snddev_poweramp_off_speaker,
	.max_voice_rx_vol[VOC_NB_INDEX] = 1000,
	.min_voice_rx_vol[VOC_NB_INDEX] = -500,
	.max_voice_rx_vol[VOC_WB_INDEX] = 1000,
	.min_voice_rx_vol[VOC_WB_INDEX] = -500,
};
static enum hsed_controller spk_gtalk_pmctl_id[] = {PM_HSED_CONTROLLER_0};
static struct snddev_icodec_data speaker_gtalk_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "speaker_gtalk_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPEAKER_GTALK_TX,
	.profile = &speaker_gtalk_tx_profile,
	.channel_mode = 1,
	.pmctl_id = spk_gtalk_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(spk_gtalk_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};
static struct snddev_icodec_data headset_gtalk_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_gtalk_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_GTALK_RX,
	.profile = &headset_gtalk_rx_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_poweramp_on_headset,
	.pamp_off = msm_snddev_poweramp_off_headset,
	.max_voice_rx_vol[VOC_NB_INDEX] = -700,
	.min_voice_rx_vol[VOC_NB_INDEX] = -2200,
	.max_voice_rx_vol[VOC_WB_INDEX] = -900,
	.min_voice_rx_vol[VOC_WB_INDEX] = -2400
};
static struct snddev_icodec_data headset_gtalk_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "headset_gtalk_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_GTALK_TX,
	.profile = &headset_gtalk_tx_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
#if defined(CONFIG_MACH_ANCORA) || defined(CONFIG_MACH_ANCORA_USA) || defined(CONFIG_MACH_APACHE)
	.pamp_on = msm_snddev_tx_ear_route_config,
	.pamp_off = msm_snddev_tx_ear_route_deconfig,
#else
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
#endif
};
static struct snddev_ecodec_data bt_sco_gtalk_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "bt_sco_gtalk_rx",
	.copp_id = 1,
	.acdb_id = ACDB_ID_BT_SCO_GTALK_RX,
	.channel_mode = 1,
	.conf_pcm_ctl_val = BT_SCO_PCM_CTL_VAL,
	.conf_aux_codec_intf = BT_SCO_AUX_CODEC_INTF,
	.conf_data_format_padding_val = BT_SCO_DATA_FORMAT_PADDING,
	.max_voice_rx_vol[VOC_NB_INDEX] = 400,
	.min_voice_rx_vol[VOC_NB_INDEX] = -1100,
	.max_voice_rx_vol[VOC_WB_INDEX] = 400,
	.min_voice_rx_vol[VOC_WB_INDEX] = -1100,
};
static struct snddev_ecodec_data bt_sco_gtalk_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "bt_sco_gtalk_tx",
	.copp_id = 1,
	.acdb_id = ACDB_ID_BT_SCO_GTALK_TX,
	.channel_mode = 1,
	.conf_pcm_ctl_val = BT_SCO_PCM_CTL_VAL,
	.conf_aux_codec_intf = BT_SCO_AUX_CODEC_INTF,
	.conf_data_format_padding_val = BT_SCO_DATA_FORMAT_PADDING,
};
#endif

#ifdef CONFIG_MACH_ANCORA_TMO
static struct snddev_icodec_data speaker_loopback_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "speaker_loopback_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPEAKER_LOOPBACK_RX,
	.profile = &speaker_loopback_rx_profile,
	.channel_mode = 2,
	.pmctl_id_sz = 0,
#ifdef CONFIG_VP_A2220_16KHZ
	.default_sample_rate = 16000,
#else
	.default_sample_rate = 48000,
#endif
	.pamp_on = &msm_snddev_poweramp_on_speaker_call,
	.pamp_off = &msm_snddev_poweramp_off_speaker_call,
//	.property = SIDE_TONE_MASK,
	.max_voice_rx_vol[VOC_NB_INDEX] = 1000,
	.min_voice_rx_vol[VOC_NB_INDEX] = -500,
	.max_voice_rx_vol[VOC_WB_INDEX] = 1000,
	.min_voice_rx_vol[VOC_WB_INDEX] = -500,
};
static struct snddev_icodec_data speaker_loopback_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "speaker_loopback_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPEAKER_LOOPBACK_TX,
	.profile = &speaker_loopback_tx_profile,
	.channel_mode = 1,
	.pmctl_id = spk_call_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(spk_call_pmctl_id),
#ifdef CONFIG_VP_A2220_16KHZ
	.default_sample_rate = 16000,
#else
	.default_sample_rate = 48000,
#endif
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};
#endif

static struct platform_device device_handset_rx = {
	.name = "snddev_icodec",
	.id = 0,
	.dev = { .platform_data = &handset_rx_data },
};
static struct platform_device device_handset_tx = {
	.name = "snddev_icodec",
	.id = 1,
	.dev = { .platform_data = &handset_tx_data },
};
static struct platform_device device_speaker_rx = {
	.name = "snddev_icodec",
	.id = 2,
	.dev = { .platform_data = &speaker_rx_data },
};
static struct platform_device device_speaker_tx = {
	.name = "snddev_icodec",
	.id = 3,
	.dev = { .platform_data = &speaker_tx_data },
};
static struct platform_device device_headset_rx = {
	.name = "snddev_icodec",
	.id = 4,
	.dev = { .platform_data = &headset_rx_data },
};
static struct platform_device device_headset_tx = {
	.name = "snddev_icodec",
	.id = 5,
	.dev = { .platform_data = &headset_tx_data },
};
static struct platform_device device_bt_sco_rx = {
	.name = "msm_snddev_ecodec",
	.id = 6,
	.dev = { .platform_data = &bt_sco_rx_data },
};
static struct platform_device device_bt_sco_tx = {
	.name = "msm_snddev_ecodec",
	.id = 7,
	.dev = { .platform_data = &bt_sco_tx_data },
};
static struct platform_device device_bt_sco_nrec_rx = {
	.name = "msm_snddev_ecodec",
	.id = 8,
	.dev = { .platform_data = &bt_sco_nrec_rx_data },
};
static struct platform_device device_bt_sco_nrec_tx = {
	.name = "msm_snddev_ecodec",
	.id = 9,
	.dev = { .platform_data = &bt_sco_nrec_tx_data },
};
static struct platform_device device_hdmi_stereo_rx = {
	.name = "snddev_mi2s",
	.id = 30,
	.dev = { .platform_data = &hdmi_stereo_rx_data },
};
static struct platform_device device_aux_dock_rx = {
	.name = "snddev_icodec",
	.id = 31,
	.dev = { .platform_data = &aux_dock_rx_data },
};
static struct platform_device device_speaker_headset_rx = {
	.name = "snddev_icodec",
	.id = 32,
	.dev = { .platform_data = &speaker_headset_rx_data },
};
static struct platform_device device_speaker_dock_rx = {
	.name = "snddev_icodec",
	.id = 33,
	.dev = { .platform_data = &speaker_dock_rx_data },
};
static struct platform_device device_speaker_hdmi_rx = {
	.name = "snddev_icodec",
	.id = 34,
	.dev = { .platform_data = &speaker_hdmi_rx_data },
};
static struct platform_device device_handset_call_rx = {
	.name = "snddev_icodec",
	.id = 50,
	.dev = { .platform_data = &handset_call_rx_data },
};
static struct platform_device device_handset_call_tx = {
	.name = "snddev_icodec",
	.id = 51,
	.dev = { .platform_data = &handset_call_tx_data },
};
static struct platform_device device_speaker_call_rx = {
	.name = "snddev_icodec",
	.id = 52,
	.dev = { .platform_data = &speaker_call_rx_data },
};
static struct platform_device device_speaker_call_tx = {
	.name = "snddev_icodec",
	.id = 53,
	.dev = { .platform_data = &speaker_call_tx_data },
};
static struct platform_device device_headset_call_rx = {
	.name = "snddev_icodec",
	.id = 54,
	.dev = { .platform_data = &headset_call_rx_data },
};
static struct platform_device device_headset_call_tx = {
	.name = "snddev_icodec",
	.id = 55,
	.dev = { .platform_data = &headset_call_tx_data },
};
static struct platform_device device_bt_sco_call_rx = {
	.name = "msm_snddev_ecodec",
	.id = 56,
	.dev = { .platform_data = &bt_sco_call_rx_data },
};
static struct platform_device device_bt_sco_call_tx = {
	.name = "msm_snddev_ecodec",
	.id = 57,
	.dev = { .platform_data = &bt_sco_call_tx_data },
};
static struct platform_device device_bt_sco_nrec_call_rx = {
	.name = "msm_snddev_ecodec",
	.id = 58,
	.dev = { .platform_data = &bt_sco_nrec_call_rx_data },
};
static struct platform_device device_bt_sco_nrec_call_tx = {
	.name = "msm_snddev_ecodec",
	.id = 59,
	.dev = { .platform_data = &bt_sco_nrec_call_tx_data },
};
static struct platform_device device_tty_headset_mono_call_rx = {
	.name = "snddev_icodec",
	.id = 78,
	.dev = { .platform_data = &tty_headset_mono_call_rx_data },
};
static struct platform_device device_tty_headset_mono_call_tx = {
	.name = "snddev_icodec",
	.id = 79,
	.dev = { .platform_data = &tty_headset_mono_call_tx_data },
};
static struct platform_device device_dualmic_handset_call_tx = {
	.name = "snddev_icodec",
	.id = 99,
	.dev = { .platform_data = &dualmic_handset_call_tx_data },
};
static struct platform_device device_handset_fmradio_rx = {
	.name = "snddev_icodec",
	.id = 100,
	.dev = { .platform_data = &handset_fmradio_rx_data },
};
static struct platform_device device_headset_fmradio_rx = {
	.name = "snddev_icodec",
	.id = 102,
	.dev = { .platform_data = &headset_fmradio_rx_data },
};
static struct platform_device device_headset_fmradio_tx = {
	.name = "snddev_mi2s",
	.id = 103,
	.dev = { .platform_data = &headset_fmradio_tx_data},
};
static struct platform_device device_speaker_fmradio_rx = {
	.name = "snddev_icodec",
	.id = 104,
	.dev = { .platform_data = &speaker_fmradio_rx_data },
};
static struct platform_device device_headset_fmradio_only_rx = {
	.name = "snddev_icodec",
	.id = 105,
	.dev = { .platform_data = &headset_fmradio_only_rx_data },
};
static struct platform_device device_speaker_fmradio_only_rx = {
	.name = "snddev_icodec",
	.id = 106,
	.dev = { .platform_data = &speaker_fmradio_only_rx_data },
};
static struct platform_device device_speaker_voice_dialer_tx = {
	.name = "snddev_icodec",
	.id = 110,
	.dev = { .platform_data = &speaker_voice_dialer_tx_data },
};
static struct platform_device device_headset_voice_dialer_tx = {
	.name = "snddev_icodec",
	.id = 111,
	.dev = { .platform_data = &headset_voice_dialer_tx_data },
};
static struct platform_device device_bt_sco_voice_dialer_tx = {
	.name = "msm_snddev_ecodec",
	.id = 112,
	.dev = { .platform_data = &bt_sco_voice_dialer_tx_data },
};
static struct platform_device device_bt_sco_nrec_voice_dialer_tx = {
	.name = "msm_snddev_ecodec",
	.id = 113,
	.dev = { .platform_data = &bt_sco_nrec_voice_dialer_tx_data },
};
static struct platform_device device_speaker_voice_search_tx = {
	.name = "snddev_icodec",
	.id = 115,
	.dev = { .platform_data = &speaker_voice_search_tx_data },
};
static struct platform_device device_headset_voice_search_tx = {
	.name = "snddev_icodec",
	.id = 116,
	.dev = { .platform_data = &headset_voice_search_tx_data },
};

/* [jseob.kim] VOIP call path */
static struct platform_device device_handset_voip_rx = {
	.name = "snddev_icodec",
	.id = 121,
	.dev = { .platform_data = &handset_voip_rx_data },
};
static struct platform_device device_handset_voip_tx = {
	.name = "snddev_icodec",
	.id = 122,
	.dev = { .platform_data = &handset_voip_tx_data },
};
static struct platform_device device_speaker_voip_rx = {
	.name = "snddev_icodec",
	.id = 123,
	.dev = { .platform_data = &speaker_voip_rx_data },
};
static struct platform_device device_speaker_voip_tx = {
	.name = "snddev_icodec",
	.id = 124,
	.dev = { .platform_data = &speaker_voip_tx_data },
};
static struct platform_device device_headset_voip_rx = {
	.name = "snddev_icodec",
	.id = 125,
	.dev = { .platform_data = &headset_voip_rx_data },
};
static struct platform_device device_headset_voip_tx = {
	.name = "snddev_icodec",
	.id = 126,
	.dev = { .platform_data = &headset_voip_tx_data },
};
static struct platform_device device_bt_sco_voip_rx = {
	.name = "msm_snddev_ecodec",
	.id = 127,
	.dev = { .platform_data = &bt_sco_voip_rx_data },
};
static struct platform_device device_bt_sco_voip_tx = {
	.name = "msm_snddev_ecodec",
	.id = 128,
	.dev = { .platform_data = &bt_sco_voip_tx_data },
};

#ifdef CONFIG_MACH_ANCORA_TMO
static struct platform_device device_handset_gan_rx = {
	.name = "snddev_icodec",
	.id = 131,
	.dev = { .platform_data = &handset_gan_rx_data },
};
static struct platform_device device_handset_gan_tx = {
	.name = "snddev_icodec",
	.id = 132,
	.dev = { .platform_data = &handset_gan_tx_data },
};
static struct platform_device device_speaker_gan_rx = {
	.name = "snddev_icodec",
	.id = 133,
	.dev = { .platform_data = &speaker_gan_rx_data },
};
static struct platform_device device_speaker_gan_tx = {
	.name = "snddev_icodec",
	.id = 134,
	.dev = { .platform_data = &speaker_gan_tx_data },
};
static struct platform_device device_headset_gan_rx = {
	.name = "snddev_icodec",
	.id = 135,
	.dev = { .platform_data = &headset_gan_rx_data },
};
static struct platform_device device_headset_gan_tx = {
	.name = "snddev_icodec",
	.id = 136,
	.dev = { .platform_data = &headset_gan_tx_data },
};
static struct platform_device device_bt_sco_gan_rx = {
	.name = "msm_snddev_ecodec",
	.id = 137,
	.dev = { .platform_data = &bt_sco_gan_rx_data },
};
static struct platform_device device_bt_sco_gan_tx = {
	.name = "msm_snddev_ecodec",
	.id = 138,
	.dev = { .platform_data = &bt_sco_gan_tx_data },
};
#endif
#if defined(CONFIG_MACH_ANCORA_TMO) || defined(CONFIG_MACH_APACHE)
static struct platform_device device_handset_gtalk_rx = {
	.name = "snddev_icodec",
	.id = 141,
	.dev = { .platform_data = &handset_gtalk_rx_data },
};
static struct platform_device device_handset_gtalk_tx = {
	.name = "snddev_icodec",
	.id = 142,
	.dev = { .platform_data = &handset_gtalk_tx_data },
};
static struct platform_device device_speaker_gtalk_rx = {
	.name = "snddev_icodec",
	.id = 143,
	.dev = { .platform_data = &speaker_gtalk_rx_data },
};
static struct platform_device device_speaker_gtalk_tx = {
	.name = "snddev_icodec",
	.id = 144,
	.dev = { .platform_data = &speaker_gtalk_tx_data },
};
static struct platform_device device_headset_gtalk_rx = {
	.name = "snddev_icodec",
	.id = 145,
	.dev = { .platform_data = &headset_gtalk_rx_data },
};
static struct platform_device device_headset_gtalk_tx = {
	.name = "snddev_icodec",
	.id = 146,
	.dev = { .platform_data = &headset_gtalk_tx_data },
};
static struct platform_device device_bt_sco_gtalk_rx = {
	.name = "msm_snddev_ecodec",
	.id = 147,
	.dev = { .platform_data = &bt_sco_gtalk_rx_data },
};
static struct platform_device device_bt_sco_gtalk_tx = {
	.name = "msm_snddev_ecodec",
	.id = 148,
	.dev = { .platform_data = &bt_sco_gtalk_tx_data },
};
#endif
#ifdef CONFIG_MACH_ANCORA_TMO
static struct platform_device device_handset_hac_call_rx = {
	.name = "snddev_icodec",
	.id = 151,
	.dev = { .platform_data = &handset_call_hac_rx_data },
};
static struct platform_device device_handset_hac_call_tx = {
	.name = "snddev_icodec",
	.id = 152,
	.dev = { .platform_data = &handset_call_hac_tx_data },
};
#endif
#ifdef CONFIG_MACH_ANCORA_TMO
static struct platform_device device_speaker_loopback_rx = {
	.name = "snddev_icodec",
	.id = 153,
	.dev = { .platform_data = &speaker_loopback_rx_data },
};
static struct platform_device device_speaker_loopback_tx = {
	.name = "snddev_icodec",
	.id = 154,
	.dev = { .platform_data = &speaker_loopback_tx_data },
};
#endif

static struct platform_device *snd_devices_ffa[] __initdata = {
        &msm_iearpiece_ffa_device,
        &msm_imic_ffa_device,
        &msm_ifmradio_handset_device,
        &msm_ihs_ffa_stereo_rx_device,
        &msm_ihs_ffa_mono_rx_device,
        &msm_ihs_mono_tx_device,
        &msm_bt_sco_earpiece_device,
        &msm_bt_sco_mic_device,
        &msm_ispeaker_rx_device,
        &msm_ifmradio_speaker_device,
        &msm_ifmradio_ffa_headset_device,
        &msm_idual_mic_endfire_device,
        &msm_idual_mic_broadside_device,
        &msm_spk_idual_mic_endfire_device,
        &msm_spk_idual_mic_broadside_device,
        &msm_itty_hs_mono_tx_device,
        &msm_itty_hs_mono_rx_device,
        &msm_ispeaker_tx_device,
        &msm_ihs_stereo_speaker_stereo_rx_device,
        &msm_a2dp_rx_device,
        &msm_a2dp_tx_device,
        &msm_snddev_mi2s_stereo_rx_device,
        &msm_snddev_mi2s_fm_tx_device,
        &msm_uplink_rx_device,
        &msm_real_stereo_tx_device,
};

static struct platform_device *snd_devices_surf[] __initdata = {
        &msm_iearpiece_device,
        &msm_imic_device,
        &msm_ihs_stereo_rx_device,
        &msm_ihs_mono_rx_device,
        &msm_ihs_mono_tx_device,
        &msm_bt_sco_earpiece_device,
        &msm_bt_sco_mic_device,
        &msm_ifmradio_handset_device,
        &msm_ispeaker_rx_device,
        &msm_ifmradio_speaker_device,
        &msm_ifmradio_headset_device,
        &msm_itty_hs_mono_tx_device,
        &msm_itty_hs_mono_rx_device,
        &msm_ispeaker_tx_device,
        &msm_ihs_stereo_speaker_stereo_rx_device,
        &msm_a2dp_rx_device,
        &msm_a2dp_tx_device,
        &msm_snddev_mi2s_stereo_rx_device,
        &msm_snddev_mi2s_fm_tx_device,
        &msm_uplink_rx_device,
};

static struct platform_device *snd_devices_fluid[] __initdata = {
        &msm_ihs_stereo_rx_device,
        &msm_ihs_mono_rx_device,
        &msm_ihs_mono_tx_device,
        &msm_ispeaker_rx_device,
        &msm_ispeaker_tx_device,
        &msm_fluid_imic_tx_device,
        &msm_fluid_iearpeice_rx_device,
        &msm_fluid_idual_mic_endfire_device,
        &msm_fluid_spk_idual_mic_endfire_device,
        &msm_a2dp_rx_device,
        &msm_a2dp_tx_device,
        &msm_snddev_mi2s_stereo_rx_device,
        &msm_uplink_rx_device,
        &msm_ifmradio_speaker_device,
        &msm_ifmradio_headset_device,
};

#ifdef CONFIG_MACH_ARIESVE
static struct platform_device *snd_devices_ariesve[] __initdata = {
	&device_handset_rx,
	&device_handset_tx,
	&device_speaker_rx,
	&device_speaker_tx,
	&device_headset_rx,
	&device_headset_tx,
	&device_bt_sco_rx,
	&device_bt_sco_tx,
	&device_bt_sco_nrec_rx,
	&device_bt_sco_nrec_tx,
	&device_hdmi_stereo_rx,
	&device_aux_dock_rx,
	&device_speaker_headset_rx,
	&device_speaker_dock_rx,
	&device_speaker_hdmi_rx,
	&device_handset_call_rx,
	&device_handset_call_tx,
	&device_speaker_call_rx,
	&device_speaker_call_tx,
	&device_headset_call_rx,
	&device_headset_call_tx,
	&device_bt_sco_call_rx,
	&device_bt_sco_call_tx,
	&device_bt_sco_nrec_call_rx,
	&device_bt_sco_nrec_call_tx,
	&device_tty_headset_mono_call_rx,
	&device_tty_headset_mono_call_tx,
	&device_dualmic_handset_call_tx,
	&device_handset_fmradio_rx,
	&device_headset_fmradio_rx,
	&device_headset_fmradio_tx,
	&device_speaker_fmradio_rx,
	&device_speaker_voice_dialer_tx,
	&device_headset_voice_dialer_tx,
	&device_bt_sco_voice_dialer_tx,
	&device_bt_sco_nrec_voice_dialer_tx,
	&device_speaker_voice_search_tx,
	&device_headset_voice_search_tx,
	&device_headset_fmradio_only_rx,
	&device_speaker_fmradio_only_rx,
        &device_handset_voip_rx,
        &device_handset_voip_tx,
        &device_speaker_voip_rx,
        &device_speaker_voip_tx,
        &device_headset_voip_rx,
        &device_headset_voip_tx,
        &device_bt_sco_voip_rx,
        &device_bt_sco_voip_tx,
};
#endif

#if defined(CONFIG_MACH_ANCORA)
static struct platform_device *snd_devices_ancora[] __initdata = {
	&device_handset_rx,
	&device_handset_tx,
	&device_speaker_rx,
	&device_speaker_tx,
	&device_headset_rx,
	&device_headset_tx,
	&device_bt_sco_rx,
	&device_bt_sco_tx,
	&device_bt_sco_nrec_rx,
	&device_bt_sco_nrec_tx,
	&device_hdmi_stereo_rx,
	&device_aux_dock_rx,
	&device_speaker_headset_rx,
	&device_speaker_dock_rx,
	&device_speaker_hdmi_rx,
	&device_handset_call_rx,
	&device_handset_call_tx,
	&device_speaker_call_rx,
	&device_speaker_call_tx,
	&device_headset_call_rx,
	&device_headset_call_tx,
	&device_bt_sco_call_rx,
	&device_bt_sco_call_tx,
	&device_bt_sco_nrec_call_rx,
	&device_bt_sco_nrec_call_tx,
	&device_tty_headset_mono_call_rx,
	&device_tty_headset_mono_call_tx,
	&device_dualmic_handset_call_tx,
	&device_handset_fmradio_rx,
	&device_headset_fmradio_rx,
	&device_headset_fmradio_tx,
	&device_speaker_fmradio_rx,
	&device_speaker_voice_dialer_tx,
	&device_headset_voice_dialer_tx,
	&device_bt_sco_voice_dialer_tx,
	&device_bt_sco_nrec_voice_dialer_tx,
	&device_speaker_voice_search_tx,
	&device_headset_voice_search_tx,
	&device_headset_fmradio_only_rx,
	&device_speaker_fmradio_only_rx,
	&device_handset_voip_rx,
	&device_handset_voip_tx,
	&device_speaker_voip_rx,
	&device_speaker_voip_tx,
	&device_headset_voip_rx,
	&device_headset_voip_tx,
	&device_bt_sco_voip_rx,
	&device_bt_sco_voip_tx,
};
#endif

#if defined(CONFIG_MACH_APACHE)
static struct platform_device *snd_devices_ancora[] __initdata = {
	&device_handset_rx,
	&device_handset_tx,
	&device_speaker_rx,
	&device_speaker_tx,
	&device_headset_rx,
	&device_headset_tx,
	&device_bt_sco_rx,
	&device_bt_sco_tx,
	&device_bt_sco_nrec_rx,
	&device_bt_sco_nrec_tx,
	&device_hdmi_stereo_rx,
	&device_aux_dock_rx,
	&device_speaker_headset_rx,
	&device_speaker_dock_rx,
	&device_speaker_hdmi_rx,
	&device_handset_call_rx,
	&device_handset_call_tx,
	&device_speaker_call_rx,
	&device_speaker_call_tx,
	&device_headset_call_rx,
	&device_headset_call_tx,
	&device_bt_sco_call_rx,
	&device_bt_sco_call_tx,
	&device_bt_sco_nrec_call_rx,
	&device_bt_sco_nrec_call_tx,
	&device_tty_headset_mono_call_rx,
	&device_tty_headset_mono_call_tx,
	&device_dualmic_handset_call_tx,
	&device_handset_fmradio_rx,
	&device_headset_fmradio_rx,
	&device_headset_fmradio_tx,
	&device_speaker_fmradio_rx,
	&device_speaker_voice_dialer_tx,
	&device_headset_voice_dialer_tx,
	&device_bt_sco_voice_dialer_tx,
	&device_bt_sco_nrec_voice_dialer_tx,
	&device_speaker_voice_search_tx,
	&device_headset_voice_search_tx,
	&device_headset_fmradio_only_rx,
	&device_speaker_fmradio_only_rx,
	&device_handset_voip_rx,
	&device_handset_voip_tx,
	&device_speaker_voip_rx,
	&device_speaker_voip_tx,
	&device_headset_voip_rx,
	&device_headset_voip_tx,
	&device_bt_sco_voip_rx,
	&device_bt_sco_voip_tx,
	&device_handset_gtalk_rx,
	&device_handset_gtalk_tx,
	&device_speaker_gtalk_rx,
	&device_speaker_gtalk_tx,
	&device_headset_gtalk_rx,
	&device_headset_gtalk_tx,
	&device_bt_sco_gtalk_rx,
	&device_bt_sco_gtalk_tx,
};
#endif

#if defined(CONFIG_MACH_ANCORA_TMO)
static struct platform_device *snd_devices_ancora[] __initdata = {
	&device_handset_rx,
	&device_handset_tx,
	&device_speaker_rx,
	&device_speaker_tx,
	&device_headset_rx,
	&device_headset_tx,
	&device_bt_sco_rx,
	&device_bt_sco_tx,
	&device_bt_sco_nrec_rx,
	&device_bt_sco_nrec_tx,
	&device_hdmi_stereo_rx,
	&device_aux_dock_rx,
	&device_speaker_headset_rx,
	&device_speaker_dock_rx,
	&device_speaker_hdmi_rx,
	&device_handset_call_rx,
	&device_handset_call_tx,
	&device_speaker_call_rx,
	&device_speaker_call_tx,
	&device_headset_call_rx,
	&device_headset_call_tx,
	&device_bt_sco_call_rx,
	&device_bt_sco_call_tx,
	&device_bt_sco_nrec_call_rx,
	&device_bt_sco_nrec_call_tx,
	&device_tty_headset_mono_call_rx,
	&device_tty_headset_mono_call_tx,
	&device_dualmic_handset_call_tx,
	&device_handset_fmradio_rx,
	&device_headset_fmradio_rx,
	&device_headset_fmradio_tx,
	&device_speaker_fmradio_rx,
	&device_speaker_voice_dialer_tx,
	&device_headset_voice_dialer_tx,
	&device_bt_sco_voice_dialer_tx,
	&device_bt_sco_nrec_voice_dialer_tx,
	&device_speaker_voice_search_tx,
	&device_headset_voice_search_tx,
	&device_headset_fmradio_only_rx,
	&device_speaker_fmradio_only_rx,
	&device_handset_voip_rx,
	&device_handset_voip_tx,
	&device_speaker_voip_rx,
	&device_speaker_voip_tx,
	&device_headset_voip_rx,
	&device_headset_voip_tx,
	&device_bt_sco_voip_rx,
	&device_bt_sco_voip_tx,
	&device_handset_gan_rx,
	&device_handset_gan_tx,
	&device_speaker_gan_rx,
	&device_speaker_gan_tx,
	&device_headset_gan_rx,
	&device_headset_gan_tx,
	&device_bt_sco_gan_rx,
	&device_bt_sco_gan_tx,
	&device_handset_gtalk_rx,
	&device_handset_gtalk_tx,
	&device_speaker_gtalk_rx,
	&device_speaker_gtalk_tx,
	&device_headset_gtalk_rx,
	&device_headset_gtalk_tx,
	&device_bt_sco_gtalk_rx,
	&device_bt_sco_gtalk_tx,
	&device_handset_hac_call_rx,
	&device_handset_hac_call_tx,
	&device_speaker_loopback_rx,
	&device_speaker_loopback_tx,
};
#endif

#ifdef CONFIG_DEBUG_FS
static void snddev_hsed_config_modify_setting(int type)
{
        struct platform_device *device;
        struct snddev_icodec_data *icodec_data;

        device = &msm_ihs_ffa_stereo_rx_device;
        icodec_data = (struct snddev_icodec_data *)device->dev.platform_data;

        if (icodec_data) {
                if (type == 1) {
                        icodec_data->voltage_on = NULL;
                        icodec_data->voltage_off = NULL;
                        icodec_data->profile->settings =
                                ihs_ffa_stereo_rx_class_d_legacy_settings;
                        icodec_data->profile->setting_sz =
                        ARRAY_SIZE(ihs_ffa_stereo_rx_class_d_legacy_settings);
                } else if (type == 2) {
                        icodec_data->voltage_on = NULL;
                        icodec_data->voltage_off = NULL;
                        icodec_data->profile->settings =
                                ihs_ffa_stereo_rx_class_ab_legacy_settings;
                        icodec_data->profile->setting_sz =
                        ARRAY_SIZE(ihs_ffa_stereo_rx_class_ab_legacy_settings);
                }
        }
}

static void snddev_hsed_config_restore_setting(void)
{
	struct platform_device *device;
	struct snddev_icodec_data *icodec_data;

	device = &msm_ihs_ffa_stereo_rx_device;
	icodec_data = (struct snddev_icodec_data *)device->dev.platform_data;

	if (icodec_data) {
		icodec_data->voltage_on = msm_snddev_hsed_voltage_on;
		icodec_data->voltage_off = msm_snddev_hsed_voltage_off;
		icodec_data->profile->settings = ihs_ffa_stereo_rx_settings;
		icodec_data->profile->setting_sz =
			ARRAY_SIZE(ihs_ffa_stereo_rx_settings);
	}
}

static ssize_t snddev_hsed_config_debug_write(struct file *filp,
	const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	char *lb_str = filp->private_data;
	char cmd;

	if (get_user(cmd, ubuf))
		return -EFAULT;

	if (!strcmp(lb_str, "msm_hsed_config")) {
		switch (cmd) {
		case '0':
			snddev_hsed_config_restore_setting();
			break;

		case '1':
			snddev_hsed_config_modify_setting(1);
			break;

		case '2':
			snddev_hsed_config_modify_setting(2);
			break;

		default:
			break;
		}
	}
	return cnt;
}

static int snddev_hsed_config_debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static const struct file_operations snddev_hsed_config_debug_fops = {
	.open = snddev_hsed_config_debug_open,
	.write = snddev_hsed_config_debug_write
};
#endif

#ifdef CONFIG_VP_A2220
//#define A1026_DEVICE_NAME "dev/audience_a1026"
extern int a2220_ioctl2(unsigned int cmd , unsigned long arg);
#endif

void __ref msm_snddev_init(void)
{
	if (machine_is_msm7x30_ffa() || machine_is_msm8x55_ffa() ||
		machine_is_msm8x55_svlte_ffa()) {
		platform_add_devices(snd_devices_ffa,
		ARRAY_SIZE(snd_devices_ffa));
#ifdef CONFIG_DEBUG_FS
		debugfs_hsed_config = debugfs_create_file("msm_hsed_config",
					S_IFREG | S_IRUGO, NULL,
		(void *) "msm_hsed_config", &snddev_hsed_config_debug_fops);
#endif
	} else if (machine_is_msm7x30_surf() || machine_is_msm8x55_surf() ||
		machine_is_msm8x55_svlte_surf())
		platform_add_devices(snd_devices_surf,
		ARRAY_SIZE(snd_devices_surf));
	else if (machine_is_msm7x30_fluid())
		platform_add_devices(snd_devices_fluid,
		ARRAY_SIZE(snd_devices_fluid));
	else
#if SAMSUNG_AUDIO_PATH
#ifdef CONFIG_MACH_ARIESVE
   		platform_add_devices(snd_devices_ariesve,
		ARRAY_SIZE(snd_devices_ariesve));
#elif defined(CONFIG_MACH_ANCORA)
   		platform_add_devices(snd_devices_ancora,
		ARRAY_SIZE(snd_devices_ancora));
#elif defined(CONFIG_MACH_ANCORA_TMO)
   		platform_add_devices(snd_devices_ancora,
		ARRAY_SIZE(snd_devices_ancora));
#elif defined(CONFIG_MACH_APACHE)
   		platform_add_devices(snd_devices_ancora,
		ARRAY_SIZE(snd_devices_ancora));
#endif

#else
		platform_add_devices(snd_devices_surf,
		ARRAY_SIZE(snd_devices_surf));
#endif

#ifdef CONFIG_VP_A2220
	printk("msm_snddev_init() : a2220_ioctl2() :  A2220_BOOTUP_INIT\n");
	a2220_ioctl2(A2220_BOOTUP_INIT , 0);
	printk("msm_snddev_init() : end\n");
#endif

//		pr_err("%s: Unknown machine type\n", __func__);
}
