#include <typedefs.h>
#include <osl.h>

#include <epivers.h>
#include <bcmutils.h>

#include <bcmendian.h>
#include <dngl_stats.h>
#include <dhd.h>

#include <dhd_dbg.h>

extern int dhdcdc_set_ioctl(dhd_pub_t *dhd, int ifidx, uint cmd, void *buf, uint len);

int sec_dhd_config_pm(dhd_pub_t *dhd, uint power_mode)
{
	struct file *fp      = NULL;
	char* filepath       = "/data/.psm.info";
	char iovbuf[WL_EVENTING_MASK_LEN + 12];
	int ret = 0;
	
	/* Set PowerSave mode */
	fp = filp_open(filepath, O_RDONLY, 0);
	if(IS_ERR(fp))// the file is not exist
	{
		/* Set PowerSave mode */
		ret = dhdcdc_set_ioctl(dhd, 0, WLC_SET_PM, (char *)&power_mode, sizeof(power_mode));
		if (ret < 0)
			goto err_out;

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
			if(fp)
    	 			filp_close(fp, NULL);
		}
		return;	
	}
	else
	{
		char buffer[1]   = {0};
		kernel_read(fp, fp->f_pos, buffer, 1);
		if(strncmp(buffer, "1",1)==0)
		{
			/* Set PowerSave mode */
			ret = dhdcdc_set_ioctl(dhd, 0, WLC_SET_PM, (char *)&power_mode, sizeof(power_mode));
			if (ret < 0)
				goto err_out;
		}
		else
		{
			/*Disable Power save features for CERTIFICATION*/
			power_mode = 0;
		 
			/* Set PowerSave mode */
			ret = dhdcdc_set_ioctl(dhd, 0, WLC_SET_PM, (char *)&power_mode, sizeof(power_mode));
			if (ret < 0)
				goto err_out;
		 
			/* Disable MPC */    
			bcm_mkiovar("mpc", (char *)&power_mode, 4, iovbuf, sizeof(iovbuf));
			ret = dhdcdc_set_ioctl(dhd, 0, WLC_SET_VAR, iovbuf, sizeof(iovbuf));
			if (ret < 0)
				goto err_out;

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
				if(fp)
    	 				filp_close(fp, NULL);
			}
			return;	
			
		}
	}
err_out:
	if(fp)
		filp_close(fp, NULL);
	return ret;
}
#if 0
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

#ifdef READ_MACADDR

int
dhd_read_macaddr(dhd_info_t *dhd)
{
    struct file *fp      = NULL;
    struct file *fpnv      = NULL;
    char macbuffer[18]   = {0};
    mm_segment_t oldfs   = {0};
	char randommac[3]    = {0};
	char buf[18]         = {0};
	char* filepath       = "/data/.mac.info";
    char* nvfilepath       = "/data/.nvmac.info";
	int ret;

	//MAC address copied from nv
	fpnv = filp_open(nvfilepath, O_RDONLY, 0);
	if (IS_ERR(fpnv)) {
start_readmac:
		fpnv = NULL;
		fp = filp_open(filepath, O_RDONLY, 0);
		if (IS_ERR(fp)) {
			/* File Doesn't Exist. Create and write mac addr.*/
			fp = filp_open(filepath, O_RDWR | O_CREAT, 0666);
			if(IS_ERR(fp)) {
				DHD_ERROR(("[WIFI] %s: File open error\n", filepath));
				return -1;
			}

			oldfs = get_fs();
			set_fs(get_ds());

		/* Generating the Random Bytes for 3 last octects of the MAC address */
		get_random_bytes(randommac, 3);
		
		sprintf(macbuffer,"%02X:%02X:%02X:%02X:%02X:%02X\n",
			0x60,0xd0,0xa9,randommac[0],randommac[1],randommac[2]);
			DHD_INFO(("[WIFI] The Random Generated MAC ID : %s\n", macbuffer));
			printk("[WIFI] The Random Generated MAC ID : %s\n", macbuffer);

			if(fp->f_mode & FMODE_WRITE) {			
				ret = fp->f_op->write(fp, (const char *)macbuffer, sizeof(macbuffer), &fp->f_pos);
				if(ret < 0)
					DHD_ERROR(("[WIFI] Mac address [%s] Failed to write into File: %s\n", macbuffer, filepath));
				else
					DHD_INFO(("[WIFI] Mac address [%s] written into File: %s\n", macbuffer, filepath));
			}
			set_fs(oldfs);
		}
		/* Reading the MAC Address from .mac.info file( the existed file or just created file)*/
		//rtn_value=kernel_read(fp, fp->f_pos, buf, 18);	
		ret = kernel_read(fp, 0, buf, 18);	
	}
	else {
		/* Reading the MAC Address from .nvmac.info file( the existed file or just created file)*/
		ret = kernel_read(fpnv, 0, buf, 18);
		buf[17] ='\0';   // to prevent abnormal string display when mac address is displayed on the screen. 
		printk("Read MAC : [%s] [%d] \r\n" , buf, strncmp(buf , "00:00:00:00:00:00" , 17));
		if(strncmp(buf , "00:00:00:00:00:00" , 17) == 0) {
			filp_close(fpnv, NULL);
			goto start_readmac;
		}

		fp = filp_open(filepath, O_RDONLY, 0);
		if (IS_ERR(fp))   // If you want to write MAC address to /data/.mac.info once, 
		{
			fp = filp_open(filepath, O_RDWR | O_CREAT, 0666);
			if(IS_ERR(fp)) {
				DHD_ERROR(("[WIFI] %s: File open error\n", filepath));
				return -1;
			}

			oldfs = get_fs();
			set_fs(get_ds());
			
			if(fp->f_mode & FMODE_WRITE) {
				ret = fp->f_op->write(fp, (const char *)buf, sizeof(buf), &fp->f_pos);
				if(ret < 0)
					DHD_ERROR(("[WIFI] Mac address [%s] Failed to write into File: %s\n", buf, filepath));
				else
					DHD_INFO(("[WIFI] Mac address [%s] written into File: %s\n", buf, filepath));
			}       
			set_fs(oldfs);
		} 
		ret = kernel_read(fp, 0, buf, 18);	
	}

	if(ret)
		sscanf(buf,"%02X:%02X:%02X:%02X:%02X:%02X",
			   &dhd->pub.mac.octet[0], &dhd->pub.mac.octet[1], &dhd->pub.mac.octet[2], 
			   &dhd->pub.mac.octet[3], &dhd->pub.mac.octet[4], &dhd->pub.mac.octet[5]);
	else
		DHD_ERROR(("dhd_bus_start: Reading from the '%s' returns 0 bytes\n", filepath));

	if (fp)
		filp_close(fp, NULL);
	if (fpnv)
		filp_close(fpnv, NULL);    	

	/* Writing Newly generated MAC ID to the Dongle */
	if (0 == _dhd_set_mac_address(dhd, 0, &dhd->pub.mac))
		DHD_INFO(("dhd_bus_start: MACID is overwritten\n"));
	else
		DHD_ERROR(("dhd_bus_start: _dhd_set_mac_address() failed\n"));

    return 0;
}

#endif /* READ_MACADDR */
#endif

