#include <typedefs.h>
#include <osl.h>

#include <epivers.h>
#include <bcmutils.h>

#include <bcmendian.h>
#include <dngl_stats.h>
#include <dhd.h>

#include <dhd_dbg.h>

extern int dhdcdc_set_ioctl(dhd_pub_t *dhd, int ifidx, uint cmd, void *buf, uint len);
#ifdef CONFIG_CONTROL_PM
extern bool g_PMcontrol;
#endif

void sec_dhd_config_pm(dhd_pub_t *dhd, uint power_mode)
{
#ifdef CONFIG_CONTROL_PM
	struct file *fp      = NULL;
	char* filepath       = "/data/.psm.info";
	char iovbuf[WL_EVENTING_MASK_LEN + 12];

	/* Set PowerSave mode */
	fp = filp_open(filepath, O_RDONLY, 0);
	if(IS_ERR(fp))// the file is not exist
	{
		/* Set PowerSave mode */
		dhdcdc_set_ioctl(dhd, 0, WLC_SET_PM, (char *)&power_mode, sizeof(power_mode));

		fp = filp_open(filepath, O_RDWR | O_CREAT, 0666);
		if(IS_ERR(fp)||(fp==NULL))
		{
			DHD_ERROR(("[WIFI] %s: File open error\n", filepath));
		}
		else
		{
			char buffer[2]   = {1};
			if(fp->f_mode & FMODE_WRITE)
			{
				sprintf(buffer,"1\n");
				fp->f_op->write(fp, (const char *)buffer, sizeof(buffer), &fp->f_pos);
			}
		}
	}
	else
	{
		char buffer[1]   = {0};
		kernel_read(fp, fp->f_pos, buffer, 1);
		if(strncmp(buffer, "1",1)==0)
		{
			/* Set PowerSave mode */
			dhdcdc_set_ioctl(dhd, 0, WLC_SET_PM, (char *)&power_mode, sizeof(power_mode));
		}
		else
		{
			/*Disable Power save features for CERTIFICATION*/
			power_mode = 0;

			g_PMcontrol = TRUE;

			//disable roam for TIS
			dhd_roam = 1;
		 
			/* Set PowerSave mode */
			dhdcdc_set_ioctl(dhd, 0, WLC_SET_PM, (char *)&power_mode, sizeof(power_mode));
		 
			/* Disable MPC */    
			bcm_mkiovar("mpc", (char *)&power_mode, 4, iovbuf, sizeof(iovbuf));
			dhdcdc_set_ioctl(dhd, 0, WLC_SET_VAR, iovbuf, sizeof(iovbuf));

			fp = filp_open(filepath, O_RDWR | O_CREAT, 0666);
			if(IS_ERR(fp)||(fp==NULL))
			{
				DHD_ERROR(("[WIFI] %s: File open error\n", filepath));
			}
			else
			{
				char buffer[2]   = {1};
				if(fp->f_mode & FMODE_WRITE)
				{
					sprintf(buffer,"1\n");
					fp->f_op->write(fp, (const char *)buffer, sizeof(buffer), &fp->f_pos);
				}
			}
		}
	}

	if(fp)
		filp_close(fp, NULL);
#else
	/* Set PowerSave mode */
	dhdcdc_set_ioctl(dhd, 0, WLC_SET_PM, (char *)&power_mode, sizeof(power_mode));
#endif
}

#ifdef WRITE_MACADDR
int
dhd_write_macaddr(char *addr)
{
    struct file *fp   = NULL;
    char filepath[40] = {0};
    char macbuffer[18]= {0};
    int ret           = 0;
    mm_segment_t oldfs= {0};


    strcpy(filepath, "/data/.mac.info");

    fp = filp_open(filepath, O_RDONLY, 0);
    if(IS_ERR(fp))
    {
	/* File Doesn't Exist. Create and write mac addr.*/
	fp = filp_open(filepath, O_RDWR | O_CREAT, 0666);
    	if(IS_ERR(fp))
	{
		fp = NULL;
			DHD_ERROR(("[WIFI] %s: File open error \n", filepath));
		return -1;
	}

	oldfs = get_fs();
	set_fs(get_ds());

	sprintf(macbuffer,"%02X:%02X:%02X:%02X:%02X:%02X\n",
		addr[0],addr[1],addr[2],addr[3],addr[4],addr[5]);

	if(fp->f_mode & FMODE_WRITE)
	{
	   ret = fp->f_op->write(fp, (const char *)macbuffer, sizeof(macbuffer), &fp->f_pos);
		   DHD_INFO(("[WIFI] Mac address [%s] written into File:%s \n", macbuffer, filepath));
	}

	set_fs(oldfs);
    }

    if(fp)
	filp_close(fp, NULL);
    
    return 0;
}
#endif /* WRITE_MACADDR */

