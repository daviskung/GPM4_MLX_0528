#ifndef __DRV_L2_AEAWB_H__


#define CDSP_AE_CTRL_EN		0x01
#define CDSP_AWB_CTRL_EN	0x02
#define CDSP_AF_CTRL_EN		0x04
#define CDSP_HIST_CTRL_EN	0x08
#define CDSP_LUM_STATUS		0x10
#define CDSP_LOW_LUM		0x20
#define CDSP_HIGH_LUM		0x40
#define CDSP_AWB_SET_GAIN	0x80
#define CDSP_SAT_SWITCH		0x100
#define CDSP_AE_UPDATE		0x200


#define C_SENSOR_SET_AE         0x123456dd
#define C_SENSOR_SET_AE_STOP    0x3456dd88

#define C_GP_DO_AE          0x33445566
#define C_GP_DO_AE_STOP     0x88779965
#define C_GP_DO_AWB         0x44556677
#define C_GP_DO_AWB_STOP    0x54687691



int gp_aeawb_create(int arg);
int gp_aeawb_del(void);

int cv_aeawb_flag_lock();
int cv_aeawb_flag_unlock();

#endif // __DRV_L2_AEAWB_H__
