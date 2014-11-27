#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/skbuff.h>
#include <linux/wlan_plat.h>
#include <linux/mmc/host.h>

#include "devices.h"

#define WLAN_HOST_WAKE	111
#define WLAN_RESET	127
#define WLAN_EN_GPIO	144
#define GPIO_BT_RESET	146

#define WLAN_STATIC_SCAN_BUF0		5
#define WLAN_STATIC_SCAN_BUF1		6
#define PREALLOC_WLAN_SEC_NUM		4
#define PREALLOC_WLAN_BUF_NUM		160
#define PREALLOC_WLAN_SECTION_HEADER	24

#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_BUF_NUM * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_BUF_NUM * 1024)

#define DHD_SKB_HDRSIZE		336
#define DHD_SKB_1PAGE_BUFSIZE	((PAGE_SIZE*1)-DHD_SKB_HDRSIZE)
#define DHD_SKB_2PAGE_BUFSIZE	((PAGE_SIZE*2)-DHD_SKB_HDRSIZE)
#define DHD_SKB_4PAGE_BUFSIZE	((PAGE_SIZE*4)-DHD_SKB_HDRSIZE)

#define WLAN_SKB_BUF_NUM	17

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

struct wlan_mem_prealloc {
	void *mem_ptr;
	unsigned long size;
};

static struct wlan_mem_prealloc wlan_mem_array[PREALLOC_WLAN_SEC_NUM] = {
	{NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER)}
};

void *wlan_static_scan_buf0;
void *wlan_static_scan_buf1;
static void *brcm_wlan_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_SEC_NUM)
		return wlan_static_skb;
	if (section == WLAN_STATIC_SCAN_BUF0)
		return wlan_static_scan_buf0;
	if (section == WLAN_STATIC_SCAN_BUF1)
		return wlan_static_scan_buf1;
	if ((section < 0) || (section > PREALLOC_WLAN_SEC_NUM))
		return NULL;

	if (wlan_mem_array[section].size < size)
		return NULL;

	return wlan_mem_array[section].mem_ptr;
}

static int brcm_init_wlan_mem(void)
{
	int i;
	int j;

	for (i = 0; i < 8; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_1PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}

	for (; i < 16; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_2PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}

	wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_4PAGE_BUFSIZE);
	if (!wlan_static_skb[i])
		goto err_skb_alloc;

	for (i = 0 ; i < PREALLOC_WLAN_SEC_NUM ; i++) {
		wlan_mem_array[i].mem_ptr =
				kmalloc(wlan_mem_array[i].size, GFP_KERNEL);

		if (!wlan_mem_array[i].mem_ptr)
			goto err_mem_alloc;
	}
	wlan_static_scan_buf0 = kmalloc(65536, GFP_KERNEL);
	if (!wlan_static_scan_buf0)
		goto err_mem_alloc;
	wlan_static_scan_buf1 = kmalloc(65536, GFP_KERNEL);
	if (!wlan_static_scan_buf1)
		goto err_mem_alloc;

	printk(KERN_INFO "%s: WIFI MEM Allocated\n", __func__);
	return 0;

 err_mem_alloc:
	pr_err("Failed to mem_alloc for WLAN\n");
	for (j = 0 ; j < i ; j++)
		kfree(wlan_mem_array[j].mem_ptr);

	i = WLAN_SKB_BUF_NUM;

 err_skb_alloc:
	pr_err("Failed to skb_alloc for WLAN\n");
	for (j = 0 ; j < i ; j++)
		dev_kfree_skb(wlan_static_skb[j]);

	return -ENOMEM;
}

/* Customized Locale table : OPTIONAL feature */
#define COUNTRY_BUF_SZ        4
struct cntry_locales_custom {
	char iso_abbrev[COUNTRY_BUF_SZ];
	char custom_locale[COUNTRY_BUF_SZ];
	int  custom_locale_rev;
};

static struct cntry_locales_custom brcm_wlan_translate_custom_table[] = {
/* Table should be filled out based on custom platform regulatory requirement */
	{"",   "XV", 17},	/* Universal if Country code is unknown or empty */
	{"IR", "XV", 17},	/* Universal if Country code is IRAN, (ISLAMIC REPUBLIC OF) */
	{"SD", "XV", 17},	/* Universal if Country code is SUDAN */
	{"SY", "XV", 17},	/* Universal if Country code is SYRIAN ARAB REPUBLIC */
	{"GL", "XV", 17},	/* Universal if Country code is GREENLAND */
	{"PS", "XV", 17},	/* Universal if Country code is PALESTINE */
	{"TL", "XV", 17},	/* Universal if Country code is TIMOR-LESTE (EAST TIMOR) */
	{"MH", "XV", 17},	/* Universal if Country code is MARSHALL ISLANDS */
	{"PK", "XV", 17},	/* Universal if Country code is PAKISTAN */
	{"CK", "XV", 17},	/* Universal if Country code is Cook Island (13.4.27)*/
	{"CU", "XV", 17},	/* Universal if Country code is Cuba (13.4.27)*/
	{"FK", "XV", 17},	/* Universal if Country code is Falkland Island (13.4.27)*/
	{"FO", "XV", 17},	/* Universal if Country code is Faroe Island (13.4.27)*/
	{"GI", "XV", 17},	/* Universal if Country code is Gibraltar (13.4.27)*/
	{"IM", "XV", 17},	/* Universal if Country code is Isle of Man (13.4.27)*/
	{"CI", "XV", 17},	/* Universal if Country code is Ivory Coast (13.4.27)*/
	{"JE", "XV", 17},	/* Universal if Country code is Jersey (13.4.27)*/
	{"KP", "XV", 17},	/* Universal if Country code is North Korea (13.4.27)*/
	{"FM", "XV", 17},	/* Universal if Country code is Micronesia (13.4.27)*/
	{"MM", "XV", 17},	/* Universal if Country code is Myanmar (13.4.27)*/
	{"NU", "XV", 17},	/* Universal if Country code is Niue (13.4.27)*/
	{"NF", "XV", 17},	/* Universal if Country code is Norfolk Island (13.4.27)*/
	{"PN", "XV", 17},	/* Universal if Country code is Pitcairn Islands (13.4.27)*/
	{"PM", "XV", 17},	/* Universal if Country code is Saint Pierre and Miquelon (13.4.27)*/
	{"SS", "XV", 17},	/* Universal if Country code is South_Sudan (13.4.27)*/
	{"AL", "AL", 2},
	{"DZ", "DZ", 1},
	{"AS", "AS", 12},	/* changed 2 -> 12*/
	{"AI", "AI", 1},
	{"AG", "AG", 2},
	{"AR", "AR", 21},
	{"AW", "AW", 2},
	{"AU", "AU", 6},
	{"AT", "AT", 4},
	{"AZ", "AZ", 2},
	{"BS", "BS", 2},
	{"BH", "BH", 4},	/* changed 24 -> 4*/
	{"BD", "BD", 2},
	{"BY", "BY", 3},
	{"BE", "BE", 4},
	{"BM", "BM", 12},
	{"BA", "BA", 2},
	{"BR", "BR", 4},
	{"VG", "VG", 2},
	{"BN", "BN", 4},
	{"BG", "BG", 4},
	{"KH", "KH", 2},
	{"CA", "CA", 31},
	{"KY", "KY", 3},
	{"CN", "CN", 24},
	{"CO", "CO", 17},
	{"CR", "CR", 17},
	{"HR", "HR", 4},
	{"CY", "CY", 4},
	{"CZ", "CZ", 4},
	{"DK", "DK", 4},
	{"EE", "EE", 4},
	{"ET", "ET", 2},
	{"FI", "FI", 4},
	{"FR", "FR", 5},
	{"GF", "GF", 2},
	{"DE", "DE", 7},
	{"GR", "GR", 4},
	{"GD", "GD", 2},
	{"GP", "GP", 2},
	{"GU", "GU", 12},
	{"HK", "HK", 2},
	{"HU", "HU", 4},
	{"IS", "IS", 4},
	{"IN", "IN", 3},
	{"ID", "KR", 25},	/* ID/1 -> KR/24 */
	{"IE", "IE", 5},
	{"IL", "BO", 0},	/* IL/7 -> BO/0 */
	{"IT", "IT", 4},
	{"JP", "JP", 58},
	{"JO", "JO", 3},
	{"KW", "KW", 5},
	{"LA", "LA", 2},
	{"LV", "LV", 4},
	{"LB", "LB", 5},
	{"LS", "LS", 2},
	{"LI", "LI", 4},
	{"LT", "LT", 4},
	{"LU", "LU", 3},
	{"MO", "MO", 2},
	{"MK", "MK", 2},
	{"MW", "MW", 1},
	{"MY", "MY", 3},
	{"MV", "MV", 3},
	{"MT", "MT", 4},
	{"MQ", "MQ", 2},
	{"MR", "MR", 2},
	{"MU", "MU", 2},
	{"YT", "YT", 2},
	{"MX", "MX", 20},
	{"MD", "MD", 2},
	{"MC", "MC", 1},
	{"ME", "ME", 2},
	{"MA", "MA", 2},
	{"NP", "NP", 3},
	{"NL", "NL", 4},
	{"AN", "AN", 2},
	{"NZ", "NZ", 4},
	{"NO", "NO", 4},
	{"OM", "OM", 4},
	{"PA", "PA", 17},
	{"PG", "PG", 2},
	{"PY", "PY", 2},
	{"PE", "PE", 20},
	{"PH", "PH", 5},
	{"PL", "PL", 4},
	{"PT", "PT", 4},
	{"PR", "PR", 20},
	{"RE", "RE", 2},
	{"RO", "RO", 4},
	{"SN", "SN", 2},
	{"RS", "RS", 2},
	{"SG", "SG", 4},
	{"SK", "SK", 4},
	{"SI", "SI", 4},
	{"ES", "ES", 4},
	{"LK", "LK", 1},
	{"SE", "SE", 4},
	{"CH", "CH", 4},
	{"TW", "TW", 1},
	{"TH", "TH", 5},
	{"TT", "TT", 3},
	{"TR", "TR", 7},
	{"AE", "AE", 6},
	{"UG", "UG", 2},
	{"GB", "GB", 6},
	{"UY", "UY", 1},
	{"VI", "VI", 13},
	{"VA", "VA", 2},
	{"VE", "VE", 3},
	{"VN", "VN", 4},
	{"MA", "MA", 1},
	{"ZM", "ZM", 2},
	{"EC", "EC", 21},
	{"SV", "SV", 19},
	{"KR", "KR", 57},
	{"RU", "RU", 13},
	{"UA", "UA", 8},
	{"GT", "GT", 1},
	{"MN", "MN", 1},
	{"NI", "NI", 2},
	{"US", "Q2", 57},
};

static void *brcm_wlan_get_country_code(char *ccode)
{
	int size, i;
	static struct cntry_locales_custom country_code;

	size = ARRAY_SIZE(brcm_wlan_translate_custom_table);

	if ((size == 0) || (ccode == NULL))
		return NULL;

	for (i = 0; i < size; i++) {
		if (!strcmp(ccode, brcm_wlan_translate_custom_table[i].iso_abbrev))
			return &brcm_wlan_translate_custom_table[i];
	}

	memset(&country_code, 0, sizeof(struct cntry_locales_custom));
	strlcpy(country_code.custom_locale, ccode, COUNTRY_BUF_SZ);

	return &country_code;
}

static int wlan_wifi_cd = 0;
static void (*wlan_status_notify_cb)(int card_present, void *dev_id);
static void *wlan_status_notify_cb_devid;
int wlan_register_status_notify(void (*callback)(int, void *),
	void *dev_id)
{
	wlan_status_notify_cb = callback;
	wlan_status_notify_cb_devid = dev_id;
	return 0;
}

unsigned int wlan_status(struct device *dev)
{
	printk("wlan_status called....\n");
	return wlan_wifi_cd;
}

int __init brcm_wifi_init_gpio(void)
{
	printk(KERN_ERR "%s: msm_wlan_gpio_init\n", __func__);
	if (gpio_tlmm_config (GPIO_CFG(WLAN_EN_GPIO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), GPIO_CFG_ENABLE))
	{
		printk (KERN_ERR "%s: Unable to configure WLAN_EN_GPIO\n", __func__);
		return -EIO;
	}
	if (gpio_request (WLAN_EN_GPIO, "wlan_en"))
	{
		printk (KERN_ERR "%s: Unable to request WLAN_EN_GPIO", __func__);
		return -EINVAL;
	}

	if (gpio_tlmm_config (GPIO_CFG(WLAN_HOST_WAKE, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA), GPIO_CFG_ENABLE))
	{
		printk (KERN_ERR "%s: Unable to configure WLAN_WAKEUP \n", __func__);
		return -EIO;
	}
	if (gpio_request (WLAN_HOST_WAKE, "wlan_wakeup"))
	{
		printk (KERN_ERR "%s: Unable to request WLAN_WAKEUP ", __func__);
		return -EINVAL;
	}

	gpio_set_value (WLAN_EN_GPIO, 0);
	gpio_set_value (WLAN_RESET, 0);

	return 0;
}

int brcm_wlan_power(int onoff)
{
	printk ("Before %s: WLAN_EN_GPIO value before set is %d on=%d WLAN_RESET=%d \n", __func__, gpio_get_value (WLAN_EN_GPIO),onoff,gpio_get_value (WLAN_RESET));
	if(onoff)
	{
		if (gpio_tlmm_config (GPIO_CFG(WLAN_EN_GPIO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), GPIO_CFG_ENABLE))
		{
				printk (KERN_ERR "%s: Unable configure WLAN_EN_GPIO\n", __func__);
				return -1;
		}

		gpio_set_value (WLAN_EN_GPIO, onoff);
	}
	else
	{
		if(!gpio_get_value(GPIO_BT_RESET))
		{
			printk("Switching OFF Enable pin \n");
			if (gpio_tlmm_config (GPIO_CFG(WLAN_EN_GPIO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), GPIO_CFG_ENABLE))
			{
					printk (KERN_ERR "%s: Unable configure WLAN_EN_GPIO\n", __func__);
					return -1;
			}

			gpio_set_value (WLAN_EN_GPIO, onoff);
		}
		else
		{
			printk("BT is Enabled\n");
		}
	}
	printk ("%s: WLAN_EN_GPIO value before set is %d on=%d \n", __func__, gpio_get_value (WLAN_EN_GPIO),onoff);
	if (gpio_tlmm_config (GPIO_CFG(WLAN_RESET, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), GPIO_CFG_ENABLE))
	{
			printk (KERN_ERR "%s: Unable configure WLAN_RESET \n", __func__);
			return -1;
	}

	mdelay (100);
	gpio_set_value (WLAN_RESET, 0);
	mdelay (100);
	printk("Just Resetting WIFI ONCE,before POWER ON\n");
	gpio_set_value (WLAN_RESET, onoff);
	mdelay (10);
	printk ("After %s: WLAN_EN_GPIO value before set is %d on=%d WLAN_RESET=%d \n", __func__, gpio_get_value (WLAN_EN_GPIO),onoff,gpio_get_value (WLAN_RESET));
	return 0;
}

int brcm_wlan_set_carddetect(int val)
{
	wlan_wifi_cd = val;
	if (wlan_status_notify_cb) {
		printk ("%s: calling detect change\n", __func__);
		wlan_status_notify_cb(val,
			wlan_status_notify_cb_devid);
	}
	return 0;
}

int brcm_wlan_reset(int onoff)
{
	printk("Reseting WLAN...\n");
	if (gpio_tlmm_config (GPIO_CFG(WLAN_RESET, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), GPIO_CFG_ENABLE))
	{
		printk (KERN_ERR "%s: Unable configure WLAN_RESET \n", __func__);
		return -1;
	}

	gpio_set_value (WLAN_RESET, onoff);
	return 0;
}

static struct resource brcm_wlan_resources[] = {
	[0] = {
		.name   = "bcmdhd_wlan_irq",
		.start  = MSM_GPIO_TO_INT(WLAN_HOST_WAKE),
		.end    = MSM_GPIO_TO_INT(WLAN_HOST_WAKE),
		.flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE,
	},
};

static struct wifi_platform_data brcm_wlan_control = {
	.set_power		= brcm_wlan_power,
	.set_reset		= brcm_wlan_reset,
	.set_carddetect		= brcm_wlan_set_carddetect,
#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
	.mem_prealloc		= brcm_wlan_mem_prealloc,
#endif
	.get_country_code	= brcm_wlan_get_country_code,
};

static struct platform_device brcm_device_wlan = {
	.name			= "bcmdhd_wlan",
	.id			= 1,
	.num_resources		= ARRAY_SIZE(brcm_wlan_resources),
	.resource		= brcm_wlan_resources,
	.dev			= {
		.platform_data = &brcm_wlan_control,
	},
};

int __init brcm_wlan_init(void)
{
	printk(KERN_INFO "%s: start\n", __func__);

	brcm_wifi_init_gpio();
#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
	brcm_init_wlan_mem();
#endif

	return platform_device_register(&brcm_device_wlan);
}
