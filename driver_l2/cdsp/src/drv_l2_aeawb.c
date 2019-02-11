#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "application.h"
#include "drv_l1_cache.h"
#include "drv_l1_cdsp.h"
#include "drv_l1_front.h"
#include "drv_l2_cdsp.h"
#include "drv_l2_sensor.h"
#include "project.h"
#include "drv_l2_aeawb.h"

/********************************************************************************************************
 *                           C O N S T A N T S                                                          *
 ********************************************************************************************************/
#define C_SENSOR_AE_PROCESS_STACK_SIZE	2048
#define C_SENSOR_AE_Q_ACCEPT_MAX		4

#define C_AWB_PROCESS_STACK_SIZE	    2048
#define C_AE_PROCESS_STACK_SIZE	        4096//(2048+1024)
#define C_AEAWB_Q_ACCEPT_MAX			4

#define C_WDR_INTP_NUM                  8


/********************************************************************************************************
 *                              M A C R O S                                                             *
 ********************************************************************************************************/


/********************************************************************************************************
 *                          D A T A    T Y P E S                                                        *
 ********************************************************************************************************/


/********************************************************************************************************
 *                         E X T E R N    D A T A                                                       *
 ********************************************************************************************************/
extern gpCdspDev_t *CdspDev;

extern const INT8U g_wdr_table[];
extern const INT8U g_wdr_table_1[];
extern const INT8U g_wdr_table_2[];
extern const INT8U g_wdr_table_3[];
extern const INT8U g_wdr_table_4[];
extern const INT8U g_wdr_table_5[];

extern const INT8U g_bf_table[];
extern const INT8U g_bf_table_3[];

static INT32U cv_aeawb_open_flag = 1;

/********************************************************************************************************
 *                         E X T E R N    F U N C T I O N                                               *
 ********************************************************************************************************/
extern void sensor_get_ae_info(sensor_exposure_t *si);

/********************************************************************************************************
 *                         G L O B A L    D A T A                                                       *
 ********************************************************************************************************/
static ALIGN4 INT8U g_wdr_table_intp[C_WDR_INTP_NUM][256];
static int g_wdr_thr[C_WDR_INTP_NUM+1];
static int g_wdr_idx = 127;
static INT8U *p_wdr_table_start, *p_wdr_table_end;

osMessageQId gp_ae_msg_q;
osMessageQId gp_awb_msg_q;
osMessageQId gp_sensor_ae_msg_q;

static osThreadId gp_ae_thread_id;
static osThreadId gp_awb_thread_id;
static osThreadId gp_sensor_ae_thread_id;
static osSemaphoreId gp_aeawb_sem, gp_ae_sem;

static unsigned int iq_status_cnt[4];

static unsigned int center_weighted_tbl[64] =
{
    (int)(0.35 * 1024.0), (int)(0.35 * 1024.0), (int)(0.35 * 1024.0), (int)(0.35 * 1024.0), (int)(0.35 * 1024.0), (int)(0.35 * 1024.0), (int)(0.35 * 1024.0), (int)(0.35 * 1024.0),
    (int)(0.35 * 1024.0), (int)(0.35 * 1024.0), (int)(0.35 * 1024.0), (int)(0.35 * 1024.0), (int)(0.35 * 1024.0), (int)(0.35 * 1024.0), (int)(0.35 * 1024.0), (int)(0.35 * 1024.0),
    (int)(0.35 * 1024.0), (int)(0.35 * 1024.0), (int)(0.65 * 1024.0), (int)(0.65 * 1024.0), (int)(0.65 * 1024.0), (int)(0.65 * 1024.0), (int)(0.35 * 1024.0), (int)(0.35 * 1024.0),
    (int)(0.35 * 1024.0), (int)(0.35 * 1024.0), (int)(0.65 * 1024.0), (int)(0.65 * 1024.0), (int)(0.65 * 1024.0), (int)(0.65 * 1024.0), (int)(0.35 * 1024.0), (int)(0.35 * 1024.0),
    (int)(0.35 * 1024.0), (int)(0.35 * 1024.0), (int)(0.65 * 1024.0), (int)(0.65 * 1024.0), (int)(0.65 * 1024.0), (int)(0.65 * 1024.0), (int)(0.35 * 1024.0), (int)(0.35 * 1024.0),
    (int)(0.35 * 1024.0), (int)(0.35 * 1024.0), (int)(0.65 * 1024.0), (int)(0.65 * 1024.0), (int)(0.65 * 1024.0), (int)(0.65 * 1024.0), (int)(0.35 * 1024.0), (int)(0.35 * 1024.0),
    (int)(0.35 * 1024.0), (int)(0.35 * 1024.0), (int)(0.35 * 1024.0), (int)(0.35 * 1024.0), (int)(0.35 * 1024.0), (int)(0.35 * 1024.0), (int)(0.35 * 1024.0), (int)(0.35 * 1024.0),
    (int)(0.35 * 1024.0), (int)(0.35 * 1024.0), (int)(0.35 * 1024.0), (int)(0.35 * 1024.0), (int)(0.35 * 1024.0), (int)(0.35 * 1024.0), (int)(0.35 * 1024.0), (int)(0.35 * 1024.0),
};

static float center_weighted_tbl_f[64] =
{
   0.35, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35,
    0.35, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35,
    0.35, 0.35, 0.65, 0.65, 0.65, 0.65, 0.35, 0.35,
    0.35, 0.35, 0.65, 0.65, 0.65, 0.65, 0.35, 0.35,
    0.35, 0.35, 0.65, 0.65, 0.65, 0.65, 0.35, 0.35,
    0.35, 0.35, 0.65, 0.65, 0.65, 0.65, 0.35, 0.35,
    0.35, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35,
    0.35, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35,
};


static int uv_div[4][6] =
{
	{ 4,  8, 12, 16, 20, 24},
	{ 4,  8, 16, 20, 24, 36},
	{ 4,  8, 16, 24, 32, 48},
	{ 4,  8, 20, 32, 40, 56}
};


/********************************************************************************************************
 *               F U N C T I O N    D E C L A R A T I O N S                                             *
 ********************************************************************************************************/

int cv_aeawb_flag_lock()
{
	cv_aeawb_open_flag = 0;
	
	return 0;	
}
int cv_aeawb_flag_unlock()
{
		cv_aeawb_open_flag = 1;
	
	return 0;	
}

void drv_l2_ae_awb_lock(void)
{
    osSemaphoreWait(gp_aeawb_sem, osWaitForever);
}

void drv_l2_ae_awb_unlock(void)
{
    osSemaphoreRelease(gp_aeawb_sem);
}

void show_ae_result(void)
{
    int i;
    int norm;
    unsigned int *p_ae;

    DBG_PRINT("\r\n ==== %s ==== \r\n", __FUNCTION__);

    p_ae = CdspDev->cdsp3aresult.ae_win;
    norm = CdspDev->ae_win_w * CdspDev->ae_win_h;
    i = 64;
    do {
        unsigned int val = *p_ae++;
        if((i & 7) == 0) DBG_PRINT("\r\n");
        DBG_PRINT(" %d,", val/norm);
        i--;
    } while(i != 0);

    DBG_PRINT("\r\n");
}


void show_awb_result(void)
{
    unsigned long t1, t2;
	long t3;
	AWB_RESULT_t *awb_result;

    DBG_PRINT("\r\n ==== %s ==== \r\n", __FUNCTION__);

    awb_result = &CdspDev->cdsp3aresult.awb_result;

	DBG_PRINT("AWB cnt: %d, %d, %d\r\n", awb_result->sumcnt[0],awb_result->sumcnt[1],awb_result->sumcnt[2]);

	t1 = awb_result->sumg[0] & 0x0ffffffff;
	t2 = awb_result->sumg[0] >> 32;
	DBG_PRINT("SumG = 0x%x%x,", t2, t1);
	t1 = awb_result->sumg[1] & 0x0ffffffff;
	t2 = awb_result->sumg[1] >> 32;
	DBG_PRINT(" 0x%x%x, ", t2, t1);
	t1 = awb_result->sumg[2] & 0x0ffffffff;
	t2 = awb_result->sumg[2] >> 32;
	DBG_PRINT(" 0x%x%x\r\n", t2, t1);

	t1 = awb_result->sumrg[0] & 0x0ffffffff;
	t3 = awb_result->sumrg[0] >> 32;
	DBG_PRINT("SumRG = 0x%x%.8x,", t3, t1);
	t1 = awb_result->sumrg[1] & 0x0ffffffff;
	t3 = awb_result->sumrg[1] >> 32;
	DBG_PRINT(" 0x%x%.8x, ", t3, t1);
	t1 = awb_result->sumrg[2] & 0x0ffffffff;
	t3 = awb_result->sumrg[2] >> 32;
	DBG_PRINT(" 0x%x%.8x\r\n", t3, t1);

	t1 = awb_result->sumbg[0] & 0x0ffffffff;
	t3 = awb_result->sumbg[0] >> 32;
	DBG_PRINT("SumBG = 0x%x%.8x,", t3, t1);
	t1 = awb_result->sumbg[1] & 0x0ffffffff;
	t3 = awb_result->sumbg[1] >> 32;
	DBG_PRINT(" 0x%x%.8x, ", t3, t1);
	t1 = awb_result->sumbg[2] & 0x0ffffffff;
	t3 = awb_result->sumbg[2] >> 32;
	DBG_PRINT(" 0x%x%.8x\r\n", t3, t1);
	DBG_PRINT("\r\n================\r\n");
}


void show_histogram_result(void)
{
    int i;
    unsigned int total;
    unsigned int *p_hist;

    DBG_PRINT("\r\n ==== %s ==== \r\n", __FUNCTION__);

    p_hist = CdspDev->cdsp3aresult.histogram;
    total = 0;
    i = 64;
    do {
        unsigned int val = *p_hist++;
        total += val;
        if((i & 7) == 0) DBG_PRINT("\r\n");
        DBG_PRINT(" %d,", val);
        i--;
    } while(i != 0);

    DBG_PRINT("\r\nhistogram total = %d, hist_cnt x 256 = %d\r\n\r\n", total, total * 256);
}


void gp_awb_calc_gain_test(void)
{
    int rval, gval, bval;
    AWB_RESULT_t *awb_result;

    //DBG_PRINT("\r\n ==== %s ==== \r\n", __FUNCTION__);

    awb_result = &CdspDev->cdsp3aresult.awb_result;

    if(awb_result->sumcnt[0] + awb_result->sumcnt[1] + awb_result->sumcnt[2] > 3500)
    {
        rval = (awb_result->sumrg[0] + awb_result->sumg[0])/awb_result->sumcnt[0];
        rval += (awb_result->sumrg[1] + awb_result->sumg[1])/awb_result->sumcnt[1];
        rval += (awb_result->sumrg[2] + awb_result->sumg[2])/awb_result->sumcnt[2];

        bval = (awb_result->sumbg[0] + awb_result->sumg[0])/awb_result->sumcnt[0];
        bval += (awb_result->sumbg[1] + awb_result->sumg[1])/awb_result->sumcnt[1];
        bval += (awb_result->sumbg[2] + awb_result->sumg[2])/awb_result->sumcnt[2];

        gval = awb_result->sumg[0]/awb_result->sumcnt[0];
        gval += awb_result->sumg[1]/awb_result->sumcnt[1];
        gval += awb_result->sumg[2]/awb_result->sumcnt[2];

        CdspDev->rgain = (gval*64)/rval;
        CdspDev->bgain = (gval*64)/bval;
        //DBG_PRINT("rgain = %d, bgain = %d\r\n", CdspDev->rgain, CdspDev->bgain);
    }
}


int gp_ae_calc_gain_test(int ae_target, unsigned int *weight)
{
    int ae_val;
    int i;
    int norm;
    unsigned int *p_ae;
    unsigned int *p_w;
    unsigned int www;

    //DBG_PRINT("\r\n ==== %s ==== \r\n", __FUNCTION__);

    p_ae = CdspDev->cdsp3aresult.ae_win;
    p_w = weight;
    norm = CdspDev->ae_win_w * CdspDev->ae_win_h;
    www = 0;
    ae_val = 0;
    i = 64;
    do {
        unsigned int val = *p_ae++;
        unsigned int w = *p_w++;

        www += w;
        val = val/norm;

        ae_val += (val*w);
        i--;
    } while(i != 0);

    ae_val /= (www);
   // DBG_PRINT("w = %d\r\n", www);

    if(ae_target >= ae_val*2)
        i = 8;
    else if(ae_target >= ae_val + 10)
        i = 4;
    else if(ae_target >= ae_val + 4)
        i = 1;
    else if(ae_target <= ae_val/2)
        i = -8;
    else if(ae_target <= ae_val - 10)
        i = -4;
    else if(ae_target <= ae_val - 4)
        i = -1;
    else
        i = 0;

    //DBG_PRINT("AE val = %d, ae_target = %d, evStep = %d\r\n", ae_val, ae_target, i);
    return i;
}



static void wdr_thr_set(int thr1, int thr2)
{
    int i, step;

    g_wdr_thr[0] = thr1;
    g_wdr_thr[C_WDR_INTP_NUM] = thr2 - 4;

	step = (g_wdr_thr[C_WDR_INTP_NUM] - g_wdr_thr[0] + 1) / (C_WDR_INTP_NUM);
	if(step < 1) step = 1;
    for(i=1;i<C_WDR_INTP_NUM;i++)
    {
        g_wdr_thr[i] = g_wdr_thr[i-1] + step;
        if(g_wdr_thr[i] > g_wdr_thr[C_WDR_INTP_NUM]) g_wdr_thr[i] = g_wdr_thr[C_WDR_INTP_NUM];
    }


    //DBG_PRINT("\r\n\r\ng_wdr_thr = ");
    //for(i=0;i<C_WDR_INTP_NUM+1;i++) DBG_PRINT(" %d,", g_wdr_thr[i]);
}



static void wdr_intp(INT8U *ptable1, INT8U *ptable2)
{
    int i, j, k;

    for(i=0;i<C_WDR_INTP_NUM;i++)
    {
        unsigned int w1, w2;
        INT8U *pt = g_wdr_table_intp[i];

        w2 = ((i+1) << 15) / (C_WDR_INTP_NUM+1);
        w1 = 32768 - w2;
        //DBG_PRINT("\r\nw1 = %d, w2 = %d\r\nwdr%d = \r\n", w1, w2, i);
        for(j = 0; j < 8 ; j++)
        {
            INT8U *pt1, *pt2, *pt_intp;
            pt1 = ptable1 + j*32;
            pt2 = ptable2 + j*32;
            pt_intp = pt + j*32;
            for(k = 0; k < 32 ; k++)
            {
                INT8U t1, t2, intp;
                t1 = *pt1++;
                t2 = *pt2++;

                intp = (INT8U) ((((INT32U)t1 * w1) + ((INT32U)t2 * w2)) >> 15);
                *pt++ = intp;

                //DBG_PRINT(" %d,", intp);
            }
            //DBG_PRINT("\r\n");
        }
    }
}



static void gp_wdr_switch(int sensor_ev_idx)
{
    if(sensor_ev_idx <= g_wdr_thr[0] && g_wdr_idx != -1)
    {
        g_wdr_idx = -1;
        if((CdspDev->wdr_status & 0x0800) != 0)
        {
            int idx = CdspDev->wdr_status & 0x0f;
            drv_l1_CdspSetWDR_Table((INT32U*)p_wdr_table_start, idx);

            idx++;
            idx &= 1;
            CdspDev->wdr_status &= 0xfff0;
            CdspDev->wdr_status = CdspDev->wdr_status  | (0x10 | idx);
        }
    }
    else if(sensor_ev_idx > g_wdr_thr[C_WDR_INTP_NUM] && g_wdr_idx != C_WDR_INTP_NUM)
    {
        g_wdr_idx = C_WDR_INTP_NUM;
        if((CdspDev->wdr_status & 0x0800) != 0)
        {
            int idx = CdspDev->wdr_status & 0x0f;
            drv_l1_CdspSetWDR_Table((INT32U*)p_wdr_table_end, idx);

            idx++;
            idx &= 1;
            CdspDev->wdr_status &= 0xfff0;
            CdspDev->wdr_status = CdspDev->wdr_status  | (0x10 | idx);
        }
    }
    else
    {
        int i;
        for(i = 0; i <C_WDR_INTP_NUM ; i++)
        {
            if(sensor_ev_idx > g_wdr_thr[i] && sensor_ev_idx <= g_wdr_thr[i+1] && g_wdr_idx != i)
            {
                g_wdr_idx = i;
                if((CdspDev->wdr_status & 0x0800) != 0)
                {
                    int idx = CdspDev->wdr_status & 0x0f;
                    drv_l1_CdspSetWDR_Table((INT32U*)g_wdr_table_intp[g_wdr_idx], idx);

                    idx++;
                    idx &= 1;
                    CdspDev->wdr_status &= 0xfff0;
                    CdspDev->wdr_status = CdspDev->wdr_status  | (0x10 | idx);
                }
                break;
            }
        }
    }
}


/////////////////////////////////////////////////////////////////////////
// Set Sensor Exp/Gain
/////////////////////////////////////////////////////////////////////////
static void gp_sensor_ae_process(void const *parm)
{
    osEvent evt;
    int set_ae_cnt = 0;
    int ev_step = 0;

    DBG_PRINT("\r\n ==== %s ==== \r\n", __FUNCTION__);

    CdspDev->low_lum_switch_cnt = 0;
    CdspDev->high_lum_switch_cnt = 0;

    p_wdr_table_start = (INT8U*)g_wdr_table_1;
    p_wdr_table_end =  (INT8U*)g_wdr_table_4;
    wdr_intp(p_wdr_table_start, p_wdr_table_end);

    sensor_get_ae_info(&CdspDev->si);
    wdr_thr_set(CdspDev->si.night_ev_idx, CdspDev->si.max_ev_idx);


    while(1)
    {
        INT32U msg;

        evt = osMessageGet(gp_sensor_ae_msg_q, osWaitForever); // wait for message
        if (evt.status != osEventMessage ) {
            DBG_PRINT("%s: Err msg status: 0x%x\r\n", __FUNCTION__, evt.status);
            break;
        }

        msg = evt.value.v;
        if(msg == C_SENSOR_SET_AE_STOP) {
            DBG_PRINT("%s: stop msg: 0x%x\r\n", __FUNCTION__, msg);
            break;
        }
        else if(msg == C_SENSOR_SET_AE) {
        #if 1
            if((CdspDev->ae_awb_flag & CDSP_AE_UPDATE) != 0) {
                //DBG_PRINT("xx set Sensor AE ev step = %d\r\n", CdspDev->si.ae_ev_step);
                if(CdspDev->si.ae_ev_step != 0)
                {  CdspDev->sensor_set_exptime(CdspDev->si.ae_ev_step);}

                CdspDev->si.ae_ev_step = 0;
                //set_ae_cnt++;

                sensor_get_ae_info(&CdspDev->si);
                gp_wdr_switch(CdspDev->si.sensor_ev_idx);

                CdspDev->ae_awb_flag &= (~(CDSP_AE_UPDATE | CDSP_AE_CTRL_EN | CDSP_HIST_CTRL_EN));

                gp_wdr_switch(CdspDev->si.sensor_ev_idx);
            }
        #else
            osSemaphoreWait(gp_ae_sem, osWaitForever);
            if(CdspDev->si.ae_ev_step != 0x7ff7)
            {
                ev_step = CdspDev->si.ae_ev_step;
                CdspDev->si.ae_ev_step = 0x7ff7;
            }
            osSemaphoreRelease(gp_ae_sem);


            //DBG_PRINT("%s: ev_step = %d\r\n", __FUNCTION__, ev_step);
            if(ev_step != 0)
            {
                int t;

                t = ev_step;
                if(t > 4) {
                    t = t >> 1;
                    if(t < 4) t = 4;
                }
                else if(t < -3) {
                    t = t >> 1;
                    if(t > -3) t = -3;
                }

                CdspDev->sensor_set_exptime(t);
                ev_step -= t;

                sensor_get_ae_info(&CdspDev->si);
                gp_wdr_switch(CdspDev->si.sensor_ev_idx);
            }

            CdspDev->ae_awb_flag &= (~(CDSP_AE_UPDATE | CDSP_AE_CTRL_EN | CDSP_HIST_CTRL_EN));

        #endif
        }
        else {
            DBG_PRINT("%s: Unknow msg: 0x%x\r\n", __FUNCTION__, msg);
        }
    }

    DBG_PRINT("SENSOR AE THREAD EXIT!\r\n");
    osDelay(5000);
}


static void awb_uv_thr_adj(int offset)
{
    awb_uv_thr_t UVthr;

    UVthr.UL1N1 = CdspDev->sensor_cdsp.awb_thr[7]  - offset;
    UVthr.UL1P1 = CdspDev->sensor_cdsp.awb_thr[8]  + offset;
    UVthr.VL1N1	= CdspDev->sensor_cdsp.awb_thr[9]  - offset;
    UVthr.VL1P1 = CdspDev->sensor_cdsp.awb_thr[10] + offset;

    UVthr.UL1N2 = CdspDev->sensor_cdsp.awb_thr[11] - offset;
    UVthr.UL1P2 = CdspDev->sensor_cdsp.awb_thr[12] + offset;
    UVthr.VL1N2	= CdspDev->sensor_cdsp.awb_thr[13] - offset;
    UVthr.VL1P2 = CdspDev->sensor_cdsp.awb_thr[14] + offset;

    UVthr.UL1N3 = CdspDev->sensor_cdsp.awb_thr[15] - offset;
    UVthr.UL1P3 = CdspDev->sensor_cdsp.awb_thr[16] + offset;
    UVthr.VL1N3	= CdspDev->sensor_cdsp.awb_thr[17] - offset;
    UVthr.VL1P3 = CdspDev->sensor_cdsp.awb_thr[18] + offset;

    drv_l1_CdspSetAwbUVThr(&UVthr);
}



static void awb_y_thr_adj(int y_flag)
{
    int Ythr0, Ythr1, Ythr2, Ythr3;

    Ythr0 = CdspDev->sensor_cdsp.awb_thr[3];
    Ythr1 = CdspDev->sensor_cdsp.awb_thr[4];
    Ythr2 = CdspDev->sensor_cdsp.awb_thr[5];
    Ythr3 = CdspDev->sensor_cdsp.awb_thr[6];

    if(y_flag == 1)
    {
        Ythr0 -= 5;     if(Ythr0 < 35) Ythr0 = 35;
        Ythr1 -= 10;
        Ythr2 -= 10;
        Ythr3 -= 15;

    }
    else if(y_flag == 2)
    {
        Ythr0 -= 5;    if(Ythr0 < 35) Ythr0 = 35;
        Ythr3 += 5;    if(Ythr3 > 200) Ythr3 = 200;
    }

    drv_l1_CdspSetAwbYThr(Ythr0, Ythr1, Ythr2, Ythr3);
}



/////////////////////////////////////////////////////////////////////////
// AWB PROCESS
/////////////////////////////////////////////////////////////////////////
static void gp_awb_process(void const *parm)
{
    osEvent evt;
    unsigned char *awb;
    AWB_RESULT_t *awb_result;
    int awb_low_lum_cnt;
	int awb_fail_cnt;
	int pre_awb_ct;
	int awb_first_cnt;

    DBG_PRINT("\r\n ==== %s ==== \r\n", __FUNCTION__);

    awb = CdspDev->awb_workmem;
    awb_result = &CdspDev->cdsp3aresult.awb_result;

    gp_cdsp_awb_set_mode(awb, AWB_AUTO_FIRST_SEARCH);
    DBG_PRINT("AWB mode = AWB_AUTO_FIRST_SEARCH\r\n");

    awb_fail_cnt = 0;
    awb_low_lum_cnt = 0;
    awb_first_cnt = 0;
	pre_awb_ct = 50;

	gp_cdsp_awb_autoset_r_b_gain_boundary(awb, CdspDev->sensor_cdsp.wb_gain);

    CdspDev->ae_awb_flag = 0;
    CdspDev->awb_fr_thr = 2;
    while(1)
    {
        evt = osMessageGet(gp_awb_msg_q, osWaitForever); // wait for message
        if (evt.status != osEventMessage ) {
            DBG_PRINT("%s: Err msg status: 0x%x\r\n", __FUNCTION__, evt.status);
            break;
        }

        if(evt.value.v == C_GP_DO_AWB_STOP) {
            DBG_PRINT("%s: msg stop: 0x%x\r\n", __FUNCTION__);
            break;
        }

        if((CdspDev->ae_awb_flag & CDSP_AWB_CTRL_EN) != 0)
        {
          if(cv_aeawb_open_flag == 1)
          {
            int awb_ret;
            int ev_step;
            int awbmode = gp_cdsp_awb_get_mode(awb);
            sensor_exposure_t seInfo;

            drv_l1_CdspSetAWBEn(DISABLE, ENABLE);

            awb_ret = gp_cdsp_awb_calc_gain(awb, awb_result, CdspDev->sensor_cdsp.wb_gain);

            //if(awb_ret == AWB_FAIL) DBG_PRINT("\r\n ** AWB Failed!!! **\r\n");
/*
            if((awb_ret == AWB_FAIL) && ((CdspDev->ae_awb_flag & CDSP_LOW_LUM) != 0) && (CdspDev->sat_contr_idx >= 2))
            {
                awb_low_lum_cnt++;
                if(awb_low_lum_cnt >= 16)
                {
                    if(pre_awb_ct > 45)
                    {
                        // could be night)
                        //DBG_PRINT("Reset CT = 40\r\n");
                        gp_cdsp_awb_reset_wb_gain(awb, 40, CdspDev->sensor_cdsp.wb_gain);

                        CdspDev->ae_awb_flag |= CDSP_AWB_SET_GAIN;

                        awb_low_lum_cnt = -512;
                        pre_awb_ct = 40;
                    }
                    else
                    {
                        awb_low_lum_cnt = 0;
                    }
                }
            }
            else if((awb_ret != AWB_FAIL) && (awb_ret != AWB_RET) && (awb_low_lum_cnt > 0))
            {
                awb_low_lum_cnt = 0;
            }*/

            if(awb_ret == AWB_FAIL)
            {
                if(awbmode != AWB_AUTO_FIRST_SEARCH)
                {
                    int y_flag, uv_offset;

                    sensor_get_ae_info(&CdspDev->si);

                    y_flag = uv_offset = 0;
                    if(CdspDev->si.sensor_ev_idx <= (CdspDev->si.daylight_ev_idx+10)) {
                        y_flag = 1;
                        uv_offset = -1;
                    }
                    else if(CdspDev->si.sensor_ev_idx > CdspDev->si.night_ev_idx) {
                        y_flag = 2;
                        uv_offset = 1;
                    }

                    awb_y_thr_adj(y_flag);
                    awb_uv_thr_adj(uv_offset);
                }
                else
                    awb_y_thr_adj(2);
            }
            else if(awb_ret != AWB_RET)
            {
                CdspDev->rgain = gp_cdsp_awb_get_r_gain(awb);
                CdspDev->bgain = gp_cdsp_awb_get_b_gain(awb);

                CdspDev->ae_awb_flag |= CDSP_AWB_SET_GAIN;
                //DBG_PRINT("awb_ret = %d, awbmode = 0x%x, rgain = %d, bgain = %d\r\n", awb_ret, awbmode, CdspDev->rgain, CdspDev->bgain);
            }


            sensor_get_ae_info(&seInfo);
            ev_step = gp_cdsp_ae_get_result_ev(CdspDev->ae_workmem);
            if((awb_ret == AWB_SUCCESS_CVR) && (awbmode == AWB_AUTO_FIRST_SEARCH))
            {
                if((abs(ev_step) <= 3) || (seInfo.sensor_ev_idx >= (seInfo.max_ev_idx - 4)))
                {
                    //DBG_PRINT("awb_ret = %d, awbmode = 0x%x, rgain = %d, bgain = %d\r\n", awb_ret, awbmode, CdspDev->rgain, CdspDev->bgain);
                    awb_first_cnt++;
                    if(awb_first_cnt >= 3)
                    {
                        awb_uv_thr_adj(0);
                        gp_cdsp_awb_set_mode(awb, AWB_AUTO_CVR2);
                        awbmode = gp_cdsp_awb_get_mode(awb);
                        //DBG_PRINT("AWB mode = AWB_AUTO_CVR2\r\n");
                    }
                }
            }


            if(awb_ret != AWB_RET)
            {
                if((awbmode == AWB_AUTO_CVR) || (awbmode == AWB_AUTO_CVR_DAYLIGHT) || (awbmode == AWB_AUTO_CVR_NIGHT))
                {
                    awbmode = AWB_AUTO_CVR;
                    gp_cdsp_awb_set_mode(awb, awbmode);
                    gp_cdsp_awb_pre_calc_gain(awb, awb_result, CdspDev->sensor_cdsp.wb_gain);
                }
            }

            if(awbmode != AWB_AUTO_FIRST_SEARCH) {
                INT16U rgain2 = gp_cdsp_awb_get_r_gain2(awb);
                INT16U bgain2 = gp_cdsp_awb_get_b_gain2(awb);

                drv_l1_CdspSetWb_RB_Gain2(rgain2, bgain2);
            }

                CdspDev->ae_awb_flag &= (~CDSP_AWB_CTRL_EN);
                drv_l1_CdspSetAWBEn(ENABLE, DISABLE);

            }

        }
    }

    DBG_PRINT("AWB THREAD EXIT!\r\n");
    osDelay(5000);
}




static void gp_iq_tuning(int sensor_ev_idx)
{
    int night_ev_idx = CdspDev->si.night_ev_idx;
    //DBG_PRINT("\r\n ==== %s ==== \r\n", __FUNCTION__);

    if(sensor_ev_idx >= night_ev_idx)
    {
        CdspDev->high_lum_switch_cnt = 0;

        if((CdspDev->ae_awb_flag & CDSP_LOW_LUM) == 0)
        {
            CdspDev->low_lum_switch_cnt++;
            if(CdspDev->low_lum_switch_cnt >= 10) {
                CdspDev->ae_awb_flag &=( ~(CDSP_LUM_STATUS | CDSP_HIGH_LUM | CDSP_LOW_LUM));
                CdspDev->ae_awb_flag |= (CDSP_LUM_STATUS | CDSP_LOW_LUM);
                CdspDev->low_lum_switch_cnt = 0;
                //DBG_PRINT("DENOISE En\r\n");
            }
        }
    }
    else if(sensor_ev_idx <= (night_ev_idx-18))
    { 	// Daylight
        CdspDev->low_lum_switch_cnt = 0;
        if((CdspDev->ae_awb_flag & CDSP_HIGH_LUM) == 0)
        {
            CdspDev->high_lum_switch_cnt++;
            if(CdspDev->high_lum_switch_cnt >= 10)
            {
                CdspDev->ae_awb_flag &= (~(CDSP_LUM_STATUS | CDSP_HIGH_LUM | CDSP_LOW_LUM));
                CdspDev->ae_awb_flag |= (CDSP_LUM_STATUS | CDSP_HIGH_LUM);
                CdspDev->high_lum_switch_cnt = 0;
                //DBG_PRINT("DENOISE Dis\r\n");
            }
        }
    }
    else
    {
        CdspDev->low_lum_switch_cnt--;
        if(CdspDev->low_lum_switch_cnt < 0) CdspDev->low_lum_switch_cnt = 0;
        CdspDev->high_lum_switch_cnt--;
        if(CdspDev->high_lum_switch_cnt < 0) CdspDev->high_lum_switch_cnt = 0;
    }

    if(sensor_ev_idx >= CdspDev->sat_yuv_thr[3] && CdspDev->sat_contr_idx != 3)
    {
        iq_status_cnt[3]++;
        if(iq_status_cnt[3] >= 4)
        {
        ///raw_enable, w1,w2,w3, yuv_enable,y_mode,uv_mode
            unsigned char denoise_para[] = {1,1,2,3,1,3,1, 0, 0, 0};

            if(CdspDev->cv_para.cv_denoise_para.is_open_cv_denoise == 1)
            {
                denoise_para[0] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[4].raw_enable;
                denoise_para[1] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[4].w1;
                denoise_para[2] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[4].w2;
                denoise_para[3] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[4].w3;
                denoise_para[4] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[4].yuv_enable;
                denoise_para[5] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[4].y_mode;
                denoise_para[6] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[4].uv_mode;
                denoise_para[7] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[4].HAvg_Y_mode;
                denoise_para[8] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[4].HAvg_U_mode;
                denoise_para[9] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[4].HAvg_V_mode;
            }

            iq_status_cnt[3] = 0;
            CdspDev->sat_contr_idx = 3;
            CdspDev->UVDivide.YT1 = uv_div[3][0];
            CdspDev->UVDivide.YT2 = uv_div[3][1];
            CdspDev->UVDivide.YT3 = uv_div[3][2];
            CdspDev->UVDivide.YT4 = uv_div[3][3];
            CdspDev->UVDivide.YT5 = uv_div[3][4];
            CdspDev->UVDivide.YT6 = uv_div[3][5];
            CdspDev->ae_awb_flag |= CDSP_SAT_SWITCH;

            drv_l1_CdspSetIntplThr(160, 240);
            drv_l1_CdspSetBadPixel(40,40,40);
            drv_l1_CdspSetYuvHAvg(3, denoise_para[7], denoise_para[8], denoise_para[9]);

            drv_l1_CdspSetUvDivideEn(ENABLE);
            //drv_l1_CdspSetHbNr_Table((INT32U*)g_bf_table_3);

            drv_l1_CdspSetHbNrAJ_en(ENABLE);
            if(denoise_para[4] == 1)
            {
            drv_l1_CdspSetHbNrEn(ENABLE);
                drv_l1_CdspSetHbNrFilter_size(denoise_para[5], denoise_para[6]);
            }
            else
            {
                drv_l1_CdspSetHbNrEn(DISABLE);
            }

            drv_l1_CdspSetEdgeEn(ENABLE);
            if(denoise_para[0] == 1)
            {
                drv_l1_CdspSetDenoiseEn(ENABLE);
                drv_l1_CdspSetDenoiseWeight(denoise_para[1], denoise_para[2], denoise_para[3]);
            }
            else
            {
                drv_l1_CdspSetDenoiseEn(DISABLE);
            }

            drv_l1_CdspSetDpcRcvModeSelThr(1, 10, 20, 4);
            drv_l1_CdspSetSharpenS(CdspDev->edge_level[3]);

      /*      if((CdspDev->wdr_status & 0x0800) != 0)
            {
                //drv_l1_CdspSetWDRUVEn(DISABLE);
                //drv_l1_CdspSetWDREn(DISABLE);
                //CdspDev->wdr_status &= (~0x0010);

                int idx = CdspDev->wdr_status & 0x0f;
                drv_l1_CdspSetWDR_Table((INT32U*)g_wdr_table_3, idx);
                //drv_l1_CdspSetWDRUVEn(ENABLE);
                //drv_l1_CdspSetWDREn(ENABLE);

                idx++;
                idx &= 1;
                CdspDev->wdr_status &= 0xfff0;
                CdspDev->wdr_status = CdspDev->wdr_status  | (0x10 | idx);
            }*/

            //DBG_PRINT("sat_contr_idx = 3\r\n");
        }
    }
    else if(sensor_ev_idx < CdspDev->sat_yuv_thr[3] && sensor_ev_idx >= CdspDev->sat_yuv_thr[2] && CdspDev->sat_contr_idx != 2)
    {
        iq_status_cnt[2]++;
        if(iq_status_cnt[2] >= 6)
        {
            ///raw_enable, w1,w2,w3, yuv_enable,y_mode,uv_mode
            unsigned char denoise_para[] = {1,1,2,3,1,2,1, 0, 0, 0};

            if(CdspDev->cv_para.cv_denoise_para.is_open_cv_denoise == 1)
            {
                denoise_para[0] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[3].raw_enable;
                denoise_para[1] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[3].w1;
                denoise_para[2] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[3].w2;
                denoise_para[3] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[3].w3;
                denoise_para[4] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[3].yuv_enable;
                denoise_para[5] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[3].y_mode;
                denoise_para[6] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[3].uv_mode;
                denoise_para[7] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[3].HAvg_Y_mode;
                denoise_para[8] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[3].HAvg_U_mode;
                denoise_para[9] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[3].HAvg_V_mode;
            }
            iq_status_cnt[2] = 0;
            CdspDev->sat_contr_idx = 2;
            CdspDev->UVDivide.YT1 = uv_div[2][0];
            CdspDev->UVDivide.YT2 = uv_div[2][1];
            CdspDev->UVDivide.YT3 = uv_div[2][2];
            CdspDev->UVDivide.YT4 = uv_div[2][3];
            CdspDev->UVDivide.YT5 = uv_div[2][4];
            CdspDev->UVDivide.YT6 = uv_div[2][5];
            CdspDev->ae_awb_flag |= CDSP_SAT_SWITCH;

            drv_l1_CdspSetIntplThr(160, 240);
            drv_l1_CdspSetBadPixel(80,80,80);
            drv_l1_CdspSetYuvHAvg(3, denoise_para[7], denoise_para[8], denoise_para[9]);

            drv_l1_CdspSetUvDivideEn(ENABLE);
            //drv_l1_CdspSetHbNr_Table((INT32U*)g_bf_table);
            drv_l1_CdspSetHbNrAJ_en(ENABLE);

            drv_l1_CdspSetEdgeEn(ENABLE);

            drv_l1_CdspSetDpcRcvModeSelThr(1, 10, 25, 4);
            drv_l1_CdspSetSharpenS(CdspDev->edge_level[2]);

            if(denoise_para[4] == 1)
            {
                drv_l1_CdspSetHbNrEn(ENABLE);
                drv_l1_CdspSetHbNrFilter_size(denoise_para[5], denoise_para[6]);
            }
            else
            {
                drv_l1_CdspSetHbNrEn(DISABLE);
            }

            if(denoise_para[0] == 1)
            {
                drv_l1_CdspSetDenoiseEn(ENABLE);
                drv_l1_CdspSetDenoiseWeight(denoise_para[1], denoise_para[2], denoise_para[3]);
            }
            else
            {
                drv_l1_CdspSetDenoiseEn(DISABLE);
            }

       /*     if((CdspDev->wdr_status & 0x0800) != 0)
            {
                //drv_l1_CdspSetWDRUVEn(DISABLE);
                //drv_l1_CdspSetWDREn(DISABLE);
               // CdspDev->wdr_status &= (~0x0010);

                int idx = CdspDev->wdr_status & 0x0f;
                drv_l1_CdspSetWDR_Table((INT32U*)g_wdr_table_3, idx);
                //drv_l1_CdspSetWDRUVEn(ENABLE);
                //drv_l1_CdspSetWDREn(ENABLE);

                idx++;
                idx &= 1;
                CdspDev->wdr_status &= 0xfff0;
                CdspDev->wdr_status = CdspDev->wdr_status  | (0x10 | idx);
            }*/
           // DBG_PRINT("sat_contr_idx = 2\r\n");
        }
    }
    else if(sensor_ev_idx < CdspDev->sat_yuv_thr[2] && sensor_ev_idx >= CdspDev->sat_yuv_thr[1] && CdspDev->sat_contr_idx != 1)
    {
        iq_status_cnt[1]++;
        if(iq_status_cnt[1] >= 6)
        {
            ///raw_enable, w1,w2,w3, yuv_enable,y_mode,uv_mode
            unsigned char denoise_para[] = {1,1,2,3,1,1,1, 0, 0, 0};

            if(CdspDev->cv_para.cv_denoise_para.is_open_cv_denoise == 1)
            {
                denoise_para[0] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[2].raw_enable;
                denoise_para[1] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[2].w1;
                denoise_para[2] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[2].w2;
                denoise_para[3] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[2].w3;
                denoise_para[4] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[2].yuv_enable;
                denoise_para[5] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[2].y_mode;
                denoise_para[6] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[2].uv_mode;
                denoise_para[7] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[2].HAvg_Y_mode;
                denoise_para[8] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[2].HAvg_U_mode;
                denoise_para[9] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[2].HAvg_V_mode;
            }

            iq_status_cnt[1] = 0;
            CdspDev->sat_contr_idx = 1;
            CdspDev->UVDivide.YT1 = uv_div[1][0];
            CdspDev->UVDivide.YT2 = uv_div[1][1];
            CdspDev->UVDivide.YT3 = uv_div[1][2];
            CdspDev->UVDivide.YT4 = uv_div[1][3];
            CdspDev->UVDivide.YT5 = uv_div[1][4];
            CdspDev->UVDivide.YT6 = uv_div[1][5];
            CdspDev->ae_awb_flag |= CDSP_SAT_SWITCH;

            drv_l1_CdspSetIntplThr(120, 240);
            drv_l1_CdspSetBadPixel(120,120,120);
            drv_l1_CdspSetYuvHAvg(3, denoise_para[7], denoise_para[8], denoise_para[9]);
            drv_l1_CdspSetUvDivideEn(ENABLE);
            //drv_l1_CdspSetHbNr_Table((INT32U*)g_bf_table);

            drv_l1_CdspSetHbNrAJ_en(ENABLE);
            drv_l1_CdspSetEdgeEn(ENABLE);
            drv_l1_CdspSetDpcRcvModeSelThr(1, 10, 40, 4);
            drv_l1_CdspSetSharpenS(CdspDev->edge_level[1]);

            if(denoise_para[4] == 1)
            {
                drv_l1_CdspSetHbNrEn(ENABLE);
                drv_l1_CdspSetHbNrFilter_size(denoise_para[5], denoise_para[6]);
            }
            else
            {
                drv_l1_CdspSetHbNrEn(DISABLE);
            }

            if(denoise_para[0] == 1)
            {
                drv_l1_CdspSetDenoiseEn(ENABLE);
                drv_l1_CdspSetDenoiseWeight(denoise_para[1], denoise_para[2], denoise_para[3]);
            }
            else
            {
                drv_l1_CdspSetDenoiseEn(DISABLE);
            }

        /*    if((CdspDev->wdr_status & 0x0800) != 0)
            {
                int idx = CdspDev->wdr_status & 0x0f;
                drv_l1_CdspSetWDR_Table((INT32U*)g_wdr_table_2, idx);
               // drv_l1_CdspSetWDRUVEn(ENABLE);
               // drv_l1_CdspSetWDREn(ENABLE);


                idx++;
                idx &= 1;
                CdspDev->wdr_status &= 0xfff0;
                CdspDev->wdr_status = CdspDev->wdr_status  | (0x10 | idx);
            }*/
            //DBG_PRINT("sat_contr_idx = 1\r\n");
        }
    }
    else if(sensor_ev_idx < CdspDev->sat_yuv_thr[1] && sensor_ev_idx >= CdspDev->sat_yuv_thr[0] &&  CdspDev->sat_contr_idx != 0)
    {
        iq_status_cnt[0]++;
        if(iq_status_cnt[0] >= 6)
        {
            //raw_enable, w1,w2,w3, yuv_enable,y_mode,uv_mode
            unsigned char denoise_para[] = {1,0,1,2,0,1,1, 0, 1, 1};

            if(CdspDev->cv_para.cv_denoise_para.is_open_cv_denoise == 1)
            {
                denoise_para[0] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[1].raw_enable;
                denoise_para[1] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[1].w1;
                denoise_para[2] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[1].w2;
                denoise_para[3] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[1].w3;
                denoise_para[4] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[1].yuv_enable;
                denoise_para[5] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[1].y_mode;
                denoise_para[6] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[1].uv_mode;
                denoise_para[7] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[1].HAvg_Y_mode;
                denoise_para[8] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[1].HAvg_U_mode;
                denoise_para[9] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[1].HAvg_V_mode;
            }

            iq_status_cnt[0] = 0;
            CdspDev->sat_contr_idx = 0;
            CdspDev->UVDivide.YT1 = uv_div[0][0];
            CdspDev->UVDivide.YT2 = uv_div[0][1];
            CdspDev->UVDivide.YT3 = uv_div[0][2];
            CdspDev->UVDivide.YT4 = uv_div[0][3];
            CdspDev->UVDivide.YT5 = uv_div[0][4];
            CdspDev->UVDivide.YT6 = uv_div[0][5];
            CdspDev->ae_awb_flag |= CDSP_SAT_SWITCH;

            drv_l1_CdspSetIntplThr(CdspDev->int_low_thr, CdspDev->int_hi_thr);
            drv_l1_CdspSetBadPixel(160,160,160);
            drv_l1_CdspSetYuvHAvg(3, denoise_para[7], denoise_para[8], denoise_para[9]);
            drv_l1_CdspSetUvDivideEn(DISABLE);

            //drv_l1_CdspSetHbNr_Table((INT32U*)g_bf_table);
            drv_l1_CdspSetHbNrAJ_en(DISABLE);

            drv_l1_CdspSetEdgeEn(DISABLE);
            drv_l1_CdspSetDpcRcvModeSelThr(1, 15, 55, 4);
            drv_l1_CdspSetSharpenS(CdspDev->edge_level[0]);

            if(denoise_para[4] == 1)
            {
                drv_l1_CdspSetHbNrEn(ENABLE);
                drv_l1_CdspSetHbNrFilter_size(denoise_para[5], denoise_para[6]);
            }
            else
            {
                drv_l1_CdspSetHbNrEn(DISABLE);
            }

            if(denoise_para[0] == 1)
            {
                drv_l1_CdspSetDenoiseEn(ENABLE);
                drv_l1_CdspSetDenoiseWeight(denoise_para[1], denoise_para[2], denoise_para[3]);
            }
            else
            {
                drv_l1_CdspSetDenoiseEn(DISABLE);
            }
/*
            if((CdspDev->wdr_status & 0x0800) != 0)
            {
                int idx = CdspDev->wdr_status & 0x0f;
                drv_l1_CdspSetWDR_Table((INT32U*)g_wdr_table_2, idx);
               // drv_l1_CdspSetWDRUVEn(ENABLE);
               // drv_l1_CdspSetWDREn(ENABLE);

                idx++;
                idx &= 1;
                CdspDev->wdr_status &= 0xfff0;
                CdspDev->wdr_status = CdspDev->wdr_status  | (0x10 | idx);
            }*/

		//DBG_PRINT("sat_contr_idx = 0\r\n");
        }
    }
    else if(sensor_ev_idx < CdspDev->sat_yuv_thr[0] && CdspDev->sat_contr_idx != -1)
    {
        ///raw_enable, w1,w2,w3, yuv_enable,y_mode,uv_mode
        unsigned char denoise_para[] = {1,2,3,3,0,0,0,0,0,0};//default
        //unsigned char denoise_para[] = {1,0,1,2,1,2,3,0,0,0};

        if(CdspDev->cv_para.cv_denoise_para.is_open_cv_denoise == 1)
        {
            denoise_para[0] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[0].raw_enable;
            denoise_para[1] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[0].w1;
            denoise_para[2] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[0].w2;
            denoise_para[3] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[0].w3;
            denoise_para[4] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[0].yuv_enable;
            denoise_para[5] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[0].y_mode;
            denoise_para[6] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[0].uv_mode;
            denoise_para[7] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[0].HAvg_Y_mode;
            denoise_para[8] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[0].HAvg_U_mode;
            denoise_para[9] = CdspDev->cv_para.cv_denoise_para.cv_denoise_para[0].HAvg_V_mode;
        }

        CdspDev->sat_contr_idx = -1;
        CdspDev->ae_awb_flag |= CDSP_SAT_SWITCH;
        drv_l1_CdspSetIntplThr(CdspDev->int_low_thr, CdspDev->int_hi_thr);
        drv_l1_CdspSetBadPixel(160,160,160);
        drv_l1_CdspSetYuvHAvg(3, denoise_para[7], denoise_para[8], denoise_para[9]);
        drv_l1_CdspSetUvDivideEn(DISABLE);
        drv_l1_CdspSetEdgeEn(DISABLE);
        drv_l1_CdspSetDpcRcvModeSelThr(1, 20, 70, 4);
        drv_l1_CdspSetSharpenS(CdspDev->edge_level[0]);

         if(denoise_para[4] == 1)
        {
            drv_l1_CdspSetHbNrEn(ENABLE);
            drv_l1_CdspSetHbNrFilter_size(denoise_para[5], denoise_para[6]);
        }
        else
        {
            drv_l1_CdspSetHbNrEn(DISABLE);
        }

        if(denoise_para[0] == 1)
        {
            drv_l1_CdspSetDenoiseEn(ENABLE);
            drv_l1_CdspSetDenoiseWeight(denoise_para[1], denoise_para[2], denoise_para[3]);
        }
        else
        {
            drv_l1_CdspSetDenoiseEn(DISABLE);
        }
		//DBG_PRINT("sat_contr_idx = -1\r\n");
    }
}

/////////////////////////////////////////////////////////////////////////
// AE PROCESS
/////////////////////////////////////////////////////////////////////////
static void gp_ae_process(void const *parm)
{
    osEvent evt;
    unsigned char *ae;
    unsigned int *ae_win, *histogram;

    DBG_PRINT("\r\n ==== %s ==== \r\n", __FUNCTION__);

    iq_status_cnt[0] = iq_status_cnt[1] = iq_status_cnt[2] = iq_status_cnt[3] = 0;


    ae = CdspDev->ae_workmem;
    ae_win = CdspDev->cdsp3aresult.ae_win;
    CdspDev->global_gain = 0x20;

    histogram = CdspDev->cdsp3aresult.histogram;
    CdspDev->ae_awb_flag = CDSP_HIGH_LUM;

    gp_cdsp_ae_set_meter(CDSP_AE_METER_USER_WEIGHT, ae);
    gp_cdsp_ae_set_user_weight(center_weighted_tbl_f, ae);

	gp_cdsp_ae_set_global_gain(ae, CdspDev->global_gain);

    while(1)
    {
        evt = osMessageGet(gp_ae_msg_q, osWaitForever); // wait for message
        if (evt.status != osEventMessage ) {
            DBG_PRINT("%s: Err msg status: 0x%x\r\n", __FUNCTION__, evt.status);
            break;
        }

        if(evt.value.v == C_GP_DO_AE_STOP) {
            DBG_PRINT("%s: msg stop: 0x%x\r\n", __FUNCTION__);
            break;
        }

        //if((CdspDev->ae_awb_flag & CDSP_AE_CTRL_EN) != 0 && (CdspDev->ae_awb_flag & CDSP_AE_UPDATE) == 0 && (CdspDev->ae_awb_flag & CDSP_HIST_CTRL_EN) != 0)
        if(cv_aeawb_open_flag == 1)
        {
            int ret, ev_step;

            drv_l1_CdspSetHistgmEn(DISABLE, ENABLE);
            //show_histogram_result();
            drv_l1_CdspSetAEEn(DISABLE, ENABLE);

            sensor_get_ae_info(&CdspDev->si);

#if 0
            //show_ae_result();
            ev_step = gp_ae_calc_gain_test(36, cv_control_weighted_tbl);
#else
            ret = gp_cdsp_ae_calc_exp(ae, ae_win, &CdspDev->si, histogram);
            ev_step = gp_cdsp_ae_get_result_ev(ae);
#endif
           // DBG_PRINT("org sensor_ev_idx = %d\r\n", ev_step);

             if( (CdspDev->cv_para.cv_ev_control.is_ev_control == 1) && (ev_step != 0))
            {
                if(abs(ev_step) < CdspDev->cv_para.cv_ev_control.ev_control_threshold)
                {
                    if(ev_step > 0)
                    {
                        ev_step = 1;
                    }
                    else
                    {
                        ev_step = -1;
                    }
                }
            }


            //DBG_PRINT("ev_idx only = %d\r\n", ev_step);

            CdspDev->ae_awb_flag &= (~CDSP_AE_CTRL_EN);
            CdspDev->ae_awb_flag &= (~CDSP_HIST_CTRL_EN);
            drv_l1_CdspSetAEEn(ENABLE, DISABLE);
            drv_l1_CdspSetHistgmEn(ENABLE, DISABLE);


            if(ev_step != 0) {
                CdspDev->ae_fr_thr = 2;

                osSemaphoreWait(gp_ae_sem, osWaitForever);
                CdspDev->si.ae_ev_step = ev_step;
                osSemaphoreRelease(gp_ae_sem);

                CdspDev->ae_awb_flag |= CDSP_AE_UPDATE;
            }
            else
                CdspDev->ae_fr_thr = 1;



            ev_step += CdspDev->si.sensor_ev_idx;
            if(ev_step > CdspDev->si.max_ev_idx) ev_step = CdspDev->si.max_ev_idx;
            //DBG_PRINT("sensor_ev_idx = %d\r\n", ev_step);
            gp_iq_tuning(ev_step);
        }
    }

    DBG_PRINT("AE THREAD EXIT!\r\n");
    osDelay(5000);
}


INT32S gp_aeawb_thread_del(void)
{
	INT32U nRet = STATUS_OK;
	INT32U msg;
	osStatus status;

    DBG_PRINT("\r\n%s\r\n", __FUNCTION__);
    // MSG Q process

    msg = C_SENSOR_SET_AE_STOP;
    status = osMessagePut(gp_sensor_ae_msg_q, (INT32U)&msg, 30*1000);

    msg = C_GP_DO_AE_STOP;
    status = osMessagePut(gp_ae_msg_q, (INT32U)&msg, 30*1000);

    msg = C_GP_DO_AWB_STOP;
    status = osMessagePut(gp_awb_msg_q, (INT32U)&msg, 30*1000);



    // del thread
    status = osThreadTerminate(gp_ae_thread_id);
    if (status == osOK) {
    // Thread was terminated successfully
        DBG_PRINT("AE Thread is terminated successfully.\r\n");
    }
    else {
        // Failed to terminate a thread
        DBG_PRINT("Failed to terminate AE  Thread!!!\r\n");
    }

    // del thread
    status = osThreadTerminate(gp_awb_thread_id);
    if (status == osOK) {
    // Thread was terminated successfully
        DBG_PRINT("AWB Thread is terminated successfully.\r\n");
    }
    else {
        // Failed to terminate a thread
        DBG_PRINT("Failed to terminate AWB Thread!!!\r\n");
    }


    // del thread
    status = osThreadTerminate(gp_sensor_ae_thread_id);
    if (status == osOK) {
    // Thread was terminated successfully
        DBG_PRINT("SENSOR AE control Thread is terminated successfully.\r\n");
    }
    else {
        // Failed to terminate a thread
        DBG_PRINT("Failed to terminate SENSOR AE control Thread!!!\r\n");
    }


    // semaphore release
		status = osSemaphoreRelease(gp_aeawb_sem);
	  status = osSemaphoreRelease(gp_ae_sem);
	  vSemaphoreDelete(gp_aeawb_sem);
	  vSemaphoreDelete(gp_ae_sem);
	  gp_aeawb_sem = gp_ae_sem = 0;

	  // Delete Queue
		vQueueDelete(gp_ae_msg_q);
	  vQueueDelete(gp_awb_msg_q);
	  vQueueDelete(gp_sensor_ae_msg_q);
	  gp_ae_msg_q = gp_awb_msg_q = gp_sensor_ae_msg_q = 0;

    gp_free(CdspDev->ae_workmem);
    gp_free(CdspDev->awb_workmem);

	return nRet;
}


// function
INT32S gp_aeawb_thread_create(INT32U task_create_en)
{
	INT32S nRet;
	INT32U size;

	osThreadDef_t ae_thread_def = { "gp_ae_task", gp_ae_process, osPriorityAboveNormal, 1, C_AE_PROCESS_STACK_SIZE };
	osThreadDef_t awb_thread_def = { "gp_awb_task", gp_awb_process, osPriorityNormal, 1, C_AWB_PROCESS_STACK_SIZE };
	osThreadDef_t sensor_ae_thread_def = { "gp_sensor_ae_task", gp_sensor_ae_process, osPriorityAboveNormal, 1, C_SENSOR_AE_PROCESS_STACK_SIZE };
	osSemaphoreDef_t aeawb_sem_def = {0};
	osSemaphoreDef_t ae_sem_def = {0};

    size = gp_cdsp_ae_get_workmem_size();
	CdspDev->ae_workmem = (INT8U*)gp_malloc_align(size, 4);
	if(CdspDev->ae_workmem == 0) {
        DBG_PRINT("AE workmem alloc failed!\r\n");
        goto Return;
	}

	size = gp_cdsp_awb_get_workmem_size();
	CdspDev->awb_workmem = (INT8U*)gp_malloc_align(size, 4);
	if(CdspDev->ae_workmem == 0) {
        DBG_PRINT("AWB workmem alloc failed!\r\n");
        goto Return;
	}

	gp_cdsp_aeawb_init(CdspDev->ae_workmem, CdspDev->awb_workmem, AWB_AUTO_CVR);
	CdspDev->ae_awb_flag = 0;


	if(gp_ae_msg_q == 0) {
		osMessageQDef_t ae_q_def = { C_AEAWB_Q_ACCEPT_MAX, sizeof(INT32U)*4, 0 };

		gp_ae_msg_q = osMessageCreate(&ae_q_def, NULL);
		if(gp_ae_msg_q == 0) {
            DBG_PRINT("Err : create gp_ae_msg_q\r\n");
			nRet = STATUS_FAIL;
			goto Return;
		}
	}


    if(gp_awb_msg_q == 0) {
		osMessageQDef_t awb_q_def = { C_AEAWB_Q_ACCEPT_MAX, sizeof(INT32U)*4, 0 };

		gp_awb_msg_q = osMessageCreate(&awb_q_def, NULL);
		if(gp_awb_msg_q == 0) {
            DBG_PRINT("Err : create gp_awb_msg_q\r\n");
			nRet = STATUS_FAIL;
			goto Return;
		}
	}


	if(gp_sensor_ae_msg_q == 0) {
		osMessageQDef_t sensor_ae_q_def = { C_SENSOR_AE_Q_ACCEPT_MAX, sizeof(INT32U)*4, 0 };

		gp_sensor_ae_msg_q = osMessageCreate(&sensor_ae_q_def, NULL);
		if(gp_sensor_ae_msg_q == 0) {
            DBG_PRINT("Err : create gp_sensor_ae_msg_q\r\n");
			nRet = STATUS_FAIL;
			goto Return;
		}
	}

	gp_aeawb_sem = osSemaphoreCreate(&aeawb_sem_def, 1);
    if(gp_aeawb_sem == 0) {
        DBG_PRINT("Err : create aeawb_sem!\r\n");
        goto Return;
    }


    gp_ae_sem = osSemaphoreCreate(&ae_sem_def, 1);
    if(gp_ae_sem == 0) {
        DBG_PRINT("Err : create ae_sem!\r\n");
        goto Return;
    }

    if(task_create_en == ENABLE){
        gp_awb_thread_id = osThreadCreate(&awb_thread_def, (void *)NULL);
        if(gp_awb_thread_id == 0) {
            DBG_PRINT("Err : create gp_awb_thread_id\r\n");
            nRet = STATUS_FAIL;
            goto Return;
        }


        gp_ae_thread_id = osThreadCreate(&ae_thread_def, (void *)NULL);
        if(gp_ae_thread_id == 0) {
            DBG_PRINT("Err : create gp_ae_thread_id\r\n");
            nRet = STATUS_FAIL;
            goto Return;
        }


        gp_sensor_ae_thread_id = osThreadCreate(&sensor_ae_thread_def, (void *)NULL);
        if(gp_sensor_ae_thread_id == 0) {
            DBG_PRINT("Err : create gp_sensor_ae_thread_id\r\n");
            nRet = STATUS_FAIL;
            goto Return;
        }
    }

	nRet = STATUS_OK;
Return:
    return nRet;
}



int gp_aeawb_create(int task_en)
{
    return gp_aeawb_thread_create(task_en);
}



int gp_aeawb_del(void)
{
    return gp_aeawb_thread_del();
}
