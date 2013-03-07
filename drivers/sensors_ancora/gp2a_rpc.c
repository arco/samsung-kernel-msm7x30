
#include <mach/msm_rpcrouter.h>
#include <linux/atomic.h>
#include <linux/err.h>

#if defined(CONFIG_MACH_ARIESVE) || defined(CONFIG_MACH_ANCORA) || defined(CONFIG_MACH_GODART) || defined(CONFIG_MACH_ANCORA_TMO) || defined(CONFIG_MACH_APACHE)
#define MSM_LIGHTSENSOR_ADC_READ 
#endif

#ifdef MSM_LIGHTSENSOR_ADC_READ

#define LIGHTSENSOR_RPC_PROG  0x30000089
#define LIGHTSENSOR_READ_PROC 17
#define LIGHTSENSOR_RPC_TIMEOUT 5000	/* 5 sec */
#define LIGHTSENSOR_CB_TYPE_PROC 		1

#define BATTERY_RPC_PROG	0x30000089
#define BATTERY_RPC_VER_1_1	0x00010001
#define BATTERY_RPC_VER_2_1	0x00020001
#define BATTERY_RPC_VER_4_1     0x00040001
#define BATTERY_RPC_VER_5_1	0x00050001

struct msm_lightsensor_get_adc_ret_data {
	u32 lightsensor_adc;
};


struct msm_rpc_client *light_client;
u32 batt_api_version;

static int msm_lightsensor_get_adc_ret_func(struct msm_rpc_client *client,
				       void *buf, void *data)
{
	struct msm_lightsensor_get_adc_ret_data *data_ptr, *buf_ptr;

	data_ptr = (struct msm_lightsensor_get_adc_ret_data *)data;
	buf_ptr = (struct msm_lightsensor_get_adc_ret_data *)buf;

	data_ptr->lightsensor_adc = be32_to_cpu(buf_ptr->lightsensor_adc);

	return 0;
}


u32 lightsensor_get_adc(void)
{
	int rc;
    static int cnt = 0; 

    //printk("%s : cnt(%d)\n", __func__, cnt);

    cnt = cnt + 1;
    
	struct msm_lightsensor_get_adc_ret_data rep;

	rc = msm_rpc_client_req(light_client,
			LIGHTSENSOR_READ_PROC,
			NULL, NULL,
			msm_lightsensor_get_adc_ret_func, &rep,
			msecs_to_jiffies(LIGHTSENSOR_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s: FAIL: mpp4 get adc. rc=%d\n", __func__, rc);
		return 0;
	}

    //printk("%s : return(%d)\n", __func__, rep.lightsensor_adc);
    
	return rep.lightsensor_adc;
}
EXPORT_SYMBOL(lightsensor_get_adc);

static int msm_lightsensor_cb_func(struct msm_rpc_client *client,
			    void *buffer, int in_size)
{
	int rc = 0;
	struct rpc_request_hdr *req;
	u32 procedure;
	u32 accept_status;

	req = (struct rpc_request_hdr *)buffer;
	procedure = be32_to_cpu(req->procedure);

	switch (procedure) {
	case LIGHTSENSOR_CB_TYPE_PROC:
		accept_status = RPC_ACCEPTSTAT_SUCCESS;
		break;

	default:
		accept_status = RPC_ACCEPTSTAT_PROC_UNAVAIL;
		pr_err("%s: ERROR. procedure (%d) not supported\n",
		       __func__, procedure);
		break;
	}

	msm_rpc_start_accepted_reply(light_client,
			be32_to_cpu(req->xid), accept_status);

	rc = msm_rpc_send_accepted_reply(light_client, 0);
	if (rc)
		pr_err("%s: FAIL: sending reply. rc=%d\n", __func__, rc);

	/*if (accept_status == RPC_ACCEPTSTAT_SUCCESS)
		msm_batt_update_psy_status();*/

	return rc;
}


void msm_lightsensor_cleanup(void)
{
	if (light_client)
		msm_rpc_unregister_client(light_client);
}
EXPORT_SYMBOL(msm_lightsensor_cleanup);

int __devinit msm_lightsensor_init_rpc(void)
{
  	int rc = 0;

	light_client =
		msm_rpc_register_client("lightsensor", LIGHTSENSOR_RPC_PROG,
					BATTERY_RPC_VER_4_1,
					1, msm_lightsensor_cb_func);

	if (light_client == NULL) {
		pr_err("%s: FAIL: rpc_register_client. light_client=NULL\n",
		       __func__);
		return -ENODEV;
	} else if (IS_ERR(light_client)) {
		light_client =
			msm_rpc_register_client("lightsensor", LIGHTSENSOR_RPC_PROG,
						BATTERY_RPC_VER_1_1,
						1, msm_lightsensor_cb_func);
		batt_api_version =  BATTERY_RPC_VER_1_1;
	} else {
		light_client =
	      	msm_rpc_register_client("lightsensor", LIGHTSENSOR_RPC_PROG,
		      			BATTERY_RPC_VER_2_1,
			      		1, msm_lightsensor_cb_func);	
		batt_api_version =  BATTERY_RPC_VER_2_1;
	}

  	if (IS_ERR(light_client)) {
		light_client=
			msm_rpc_register_client("lightsensor", LIGHTSENSOR_RPC_PROG,
						BATTERY_RPC_VER_5_1,
						1, msm_lightsensor_cb_func);
		batt_api_version =  BATTERY_RPC_VER_5_1;
	}

	if (IS_ERR(light_client)) {
		rc = PTR_ERR(light_client);
		pr_err("%s: ERROR: rpc_register_client: rc = %d\n ",
		       __func__, rc);
		light_client = NULL;
		return rc;
	}

  return rc;
}
EXPORT_SYMBOL(msm_lightsensor_init_rpc);

#endif
