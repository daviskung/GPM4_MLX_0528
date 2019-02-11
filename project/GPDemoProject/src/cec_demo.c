#include "drv_l1_cec.h"
#include "drv_l2_hdmi.h"
#include "drv_l1_ext_int.h"
#include "board_config.h"
#include "application.h"
#include "drv_l2_ad_key_scan.h"

#define HDMI_CEC_CMDCODE_ACTIVE_SOURCE                       0x82
#define HDMI_CEC_CMDCODE_IMAGE_VIEW_ON                       0x04
#define HDMI_CEC_CMDCODE_TEXT_VIEW_ON                        0x0D
#define HDMI_CEC_CMDCODE_INACTIVE_SOURCE                     0x9D
#define HDMI_CEC_CMDCODE_REQUEST_ACTIVE_SOURCE               0x85
#define HDMI_CEC_CMDCODE_ROUTING_CHANGE                      0x80
#define HDMI_CEC_CMDCODE_ROUTING_INFORMATION                 0x81
#define HDMI_CEC_CMDCODE_SET_STREAM_PATH                     0x86
#define HDMI_CEC_CMDCODE_STANDBY                             0x36
#define HDMI_CEC_CMDCODE_RECORD_OFF                          0x0B
#define HDMI_CEC_CMDCODE_RECORD_ON                           0x09
#define HDMI_CEC_CMDCODE_RECORD_STATUS                       0x0A
#define HDMI_CEC_CMDCODE_RECORD_TV_SCREEN                    0x0F
#define HDMI_CEC_CMDCODE_CLEAR_ANALOGUE_TIMER                0x33
#define HDMI_CEC_CMDCODE_CLEAR_DIGITAL_TIMER                 0x99
#define HDMI_CEC_CMDCODE_CLEAR_EXTERNAL_TIMER                0xA1
#define HDMI_CEC_CMDCODE_SET_ANALOGUE_TIMER                  0x34
#define HDMI_CEC_CMDCODE_SET_DIGITAL_TIMER                   0x97
#define HDMI_CEC_CMDCODE_SET_EXTERNAL_TIMER                  0xA2
#define HDMI_CEC_CMDCODE_SET_TIMER_PROGRAM_TITLE             0x67
#define HDMI_CEC_CMDCODE_TIMER_CLEARED_STATUS                0x43
#define HDMI_CEC_CMDCODE_TIMER_STATUS                        0x35
#define HDMI_CEC_CMDCODE_CEC_VERSION                         0x9E
#define HDMI_CEC_CMDCODE_GET_CEC_VERSION                     0x9F
#define HDMI_CEC_CMDCODE_GIVE_PHYSICAL_ADDRESS               0x83
#define HDMI_CEC_CMDCODE_GET_MENU_LANGUAGE                   0x91
#define HDMI_CEC_CMDCODE_REPORT_PHYSICAL_ADDRESS             0x84
#define HDMI_CEC_CMDCODE_SET_MENU_LANGUAGE                   0x32
#define HDMI_CEC_CMDCODE_DECK_CONTROL                        0x42
#define HDMI_CEC_CMDCODE_DECK_STATUS                         0x1B
#define HDMI_CEC_CMDCODE_GIVE_DECK_STATUS                    0x1A
#define HDMI_CEC_CMDCODE_PLAY                                0x41
#define HDMI_CEC_CMDCODE_GIVE_TUNER_DEVICE_STATUS            0x08
#define HDMI_CEC_CMDCODE_SELECT_ANALOGUE_SERVICE             0x92
#define HDMI_CEC_CMDCODE_SELECT_DIGITAL_SERVICE              0x93
#define HDMI_CEC_CMDCODE_TUNER_DEVICE_STATUS                 0x07
#define HDMI_CEC_CMDCODE_TUNER_STEP_DECREMENT                0x06
#define HDMI_CEC_CMDCODE_TUNER_STEP_INCREMENT                0x05
#define HDMI_CEC_CMDCODE_DEVICE_VENDOR_ID                    0x87
#define HDMI_CEC_CMDCODE_GIVE_DEVICE_VENDOR_ID               0x8C
#define HDMI_CEC_CMDCODE_VENDOR_COMMAND                      0x89
#define HDMI_CEC_CMDCODE_VENDOR_COMMAND_WITH_ID              0xA0
#define HDMI_CEC_CMDCODE_VENDOR_REMOTE_BUTTON_DOWN           0x8A
#define HDMI_CEC_CMDCODE_VENDOR_REMOTE_BUTTON_UP             0x8B
#define HDMI_CEC_CMDCODE_SET_OSD_STRING                      0x64
#define HDMI_CEC_CMDCODE_GIVE_OSD_NAME                       0x46
#define HDMI_CEC_CMDCODE_SET_OSD_NAME                        0x47
#define HDMI_CEC_CMDCODE_MENU_REQUEST                        0x8D
#define HDMI_CEC_CMDCODE_MENU_STATUS                         0x8E
#define HDMI_CEC_CMDCODE_USER_CONTROL_PRESSED                0x44
#define HDMI_CEC_CMDCODE_USER_CONTROL_RELEASED               0x45
#define HDMI_CEC_CMDCODE_GIVE_DEVICE_POWER_STATUS            0x8F
#define HDMI_CEC_CMDCODE_REPORT_POWER_STATUS                 0x90
#define HDMI_CEC_CMDCODE_FEATURE_ABORT                       0x00
#define HDMI_CEC_CMDCODE_ABORT                               0xFF
#define HDMI_CEC_CMDCODE_GIVE_AUDIO_STATUS                   0x71
#define HDMI_CEC_CMDCODE_GIVE_SYSTEM_AUDIO_MODE_STATUS       0x7D
#define HDMI_CEC_CMDCODE_REPORT_AUDIO_STATUS                 0x7A
#define HDMI_CEC_CMDCODE_SET_SYSTEM_AUDIO_MODE               0x72
#define HDMI_CEC_CMDCODE_SYSTEM_AUDIO_MODE_REQUEST           0x70
#define HDMI_CEC_CMDCODE_SYSTEM_AUDIO_MODE_STATUS            0x7E
#define HDMI_CEC_CMDCODE_SET_AUDIO_RATE                      0x9A
#define HDMI_CEC_CMDCODE_NONE                                0x1000

#define DEVICE_BOARDCAST    0xF
#define EXTA_INT 0x40
#define CEC_QUEUE_MAX  16
#define CEC_PARA_MAX_LEN  sizeof(INT32U)
#define C_CEC_TASK_STACK_SIZE 1024
#define CEC_PRIORITY 20
#define CEC_KEY_PRIORITY 18
#define C_CEC_Q_ACCEPT_MAX 5
#define MAX_LOGIC_TYPE 5
#define RX_LEN 30

typedef enum {
	CEC_TYPE_TV = 0,
	CEC_TYPE_RECORD,
	CEC_TYPE_TUNER,
	CEC_TYPE_PLAYBACK,
	CEC_TYPE_AUDIO
}CEC_DEVICE_TYPE_ENUM;

void HDMI_CEC_Active_Source(INT16U phy_addr);
void HDMI_CEC_Feature_Abort(INT8U dest);
void cec_task_init(void);
INT32S HDMI_CEC_Polling_Message(INT8U dest);
void HDMI_CEC_Report_device_vendor_id(INT32U vendor_id);
INT32S HDMI_CEC_Send_Command(INT8U initiator, INT8U destination, INT8U* data ,INT8U length);
void HDMI_CEC_Set_OSD_name(INT8U dest, INT8U *p, INT8U length);

MSG_Q_ID CEC_TaskQ;
osMessageQId cec_q;
INT8U cec_para[CEC_PARA_MAX_LEN];

const INT8U logic_address_table[MAX_LOGIC_TYPE][5] = {
   {0x0,0xF,0xF,0xF,0xF},//TV
   {0x1,0x2,0x9,0xF,0xF},//Recording device
   {0x3,0x6,0x7,0xA,0xF},//Tuner
   {0x4,0x8,0xB,0xF,0xF},//PlayBack device
   {0x5,0xF,0xF,0xF,0xF}//Audio system
};

const INT8U logic_address_cnt_table[5] = {0x1,0x3,0x4,0x3,0x1};

static INT8U *cec_rx_buf;
static INT8U hpd_status;
static INT8U rx_msg_id;
INT8U flag_hpd_addressing_step;
INT8U OSD_NAME[3] = {0x47, 0x47, 0x50};
static own_physcial_address;
static own_logic_address;
static own_logic_type = CEC_TYPE_PLAYBACK;
static polling_msg_en;

void TX_polling_msg_ack_err_isr()
{
    INT32U msg = 0x2000;
    INT8S err;

    if(polling_msg_en == 1)
    {
        err = osMessagePut(cec_q, (INT32U)&msg, 0);
    }
}

void TX_polling_msg_get_ack_isr()
{
    INT32U msg = 0x3000;
    INT8S err;

    if(polling_msg_en == 1)
    {
        err = osMessagePut(cec_q, (INT32U)&msg, 0);
    }
}

void set_logic_address(INT8U index)
{
    own_logic_address = logic_address_table[own_logic_type][index];
    drv_l1_cec_own_address_config(0x1<<own_logic_address);
    DBG_PRINT("device set %d\r\n",own_logic_address);
}

void RX_get_data_isr()
{
    INT32U idx=0;
    INT32U opcode_cmd = 0;
    INT32U rcnt = drv_l1_cec_get_rx_count();;
    DBG_PRINT("CEC RX[%d]",rcnt);
    while(rcnt > 0)
    {
        DBG_PRINT("%02x ",*(cec_rx_buf+idx));
        idx++;
        rcnt--;
    }
	DBG_PRINT("\r\n");
    opcode_cmd = HDMI_CEC_CMDCODE_NONE;
    rx_msg_id = *(cec_rx_buf);
	switch((*(cec_rx_buf+1)))
	{
		case HDMI_CEC_CMDCODE_GIVE_PHYSICAL_ADDRESS:
			opcode_cmd = HDMI_CEC_CMDCODE_REPORT_PHYSICAL_ADDRESS;
		break;
		case HDMI_CEC_CMDCODE_REQUEST_ACTIVE_SOURCE:
			opcode_cmd = HDMI_CEC_CMDCODE_ACTIVE_SOURCE;
		break;
		case HDMI_CEC_CMDCODE_SET_STREAM_PATH:
		break;
		case HDMI_CEC_CMDCODE_STANDBY:
            R_IOA_O_DATA &= ~(0x1<<6);
            R_SYSTEM_POWER_CTRL0 &= ~0x1;
		break;
		case HDMI_CEC_CMDCODE_GIVE_OSD_NAME:
			opcode_cmd = HDMI_CEC_CMDCODE_SET_OSD_NAME;
		break;
		case HDMI_CEC_CMDCODE_ABORT:
			opcode_cmd = HDMI_CEC_CMDCODE_FEATURE_ABORT;
		break;
		case HDMI_CEC_CMDCODE_GIVE_DEVICE_VENDOR_ID:
			opcode_cmd = HDMI_CEC_CMDCODE_DEVICE_VENDOR_ID;
		break;
		default:
		break;
	}
	if(opcode_cmd != HDMI_CEC_CMDCODE_NONE)
	{
        R_IOA_O_DATA |= (0x1<<6);
        msgQSendFromISR(CEC_TaskQ, opcode_cmd, (void *)&opcode_cmd, CEC_PARA_MAX_LEN, MSG_PRI_NORMAL);
    }
}

void cec_key_task_entry(void *p_arg)
{
    INT8U hdmi_out;
    INT32U temp_data;
    INT8U hdmi_resolution;

    adc_key_scan_init();
    hdmi_out = 0;

	while(1)
	{
		adc_key_scan();

		if(ADKEY_IO1) {
            DBG_PRINT("KEY1 \r\n");
            own_logic_type++;
            if(own_logic_type >= MAX_LOGIC_TYPE)
            {
                own_logic_type = 0;
            }
            switch(own_logic_type)
            {
                case 0: DBG_PRINT("TV(not support)\r\n");break;
                case 1: DBG_PRINT("Recording Device\r\n");break;
                case 2: DBG_PRINT("Tuner\r\n");break;
                case 3: DBG_PRINT("Playback device\r\n");break;
                case 4: DBG_PRINT("Audio system\r\n");break;
                default:break;
            }
            set_logic_address(0);
		}
		else if(ADKEY_IO2) {
            DBG_PRINT("KEY2 \r\n");

            if(own_logic_type > 0)
            {
                own_logic_type--;
            }
            else
                own_logic_type = MAX_LOGIC_TYPE - 1;
            switch(own_logic_type)
            {
                case 0: DBG_PRINT("TV(not support)\r\n");break;
                case 1: DBG_PRINT("Recording Device\r\n");break;
                case 2: DBG_PRINT("Tuner\r\n");break;
                case 3: DBG_PRINT("Playback device\r\n");break;
                case 4: DBG_PRINT("Audio system\r\n");break;
                default:break;
            }
            set_logic_address(0);
		}
		else if(ADKEY_IO3) {
            DBG_PRINT("KEY3 One-Touch Play\r\n");
            if(hdmi_out == 0)
            {
                //diag_hdmi_40(0);
                hdmi_out = 1;
                //R_IOA_O_DATA |= 0x1<<4;
            }
            temp_data = own_physcial_address;
            msgQSend(CEC_TaskQ, HDMI_CEC_CMDCODE_IMAGE_VIEW_ON, (void *)0x00, CEC_PARA_MAX_LEN, MSG_PRI_NORMAL);
            osDelay(35);
            msgQSend(CEC_TaskQ, HDMI_CEC_CMDCODE_ACTIVE_SOURCE, (void *)&temp_data, CEC_PARA_MAX_LEN, MSG_PRI_NORMAL);
			osDelay(35);
			temp_data = 0;
            msgQSend(CEC_TaskQ, HDMI_CEC_CMDCODE_MENU_STATUS, (void *)&temp_data, CEC_PARA_MAX_LEN, MSG_PRI_NORMAL);
		}
		else if(ADKEY_IO4) {
            DBG_PRINT("KEY4 Power off\r\n");

            temp_data = own_physcial_address;
            msgQSend(CEC_TaskQ, HDMI_CEC_CMDCODE_INACTIVE_SOURCE, (void *)&temp_data, CEC_PARA_MAX_LEN, MSG_PRI_NORMAL);
			osDelay(35);
			temp_data = 1;
            msgQSend(CEC_TaskQ, HDMI_CEC_CMDCODE_MENU_STATUS, (void *)&temp_data, CEC_PARA_MAX_LEN, MSG_PRI_NORMAL);
            osDelay(35);
            if(hdmi_out == 1)
            {
                hdmi_out = 0;
                //R_IOA_O_DATA &= ~(0x1<<4);
            }
		}
		else if(ADKEY_IO5) {
		}
		else if(ADKEY_IO6) {
		}
		else if(ADKEY_IO7) {
		}
		else if(ADKEY_IO8) {
		}
    }
}

INT32S HDMI_CEC_Send_Command(INT8U initiator, INT8U destination, INT8U* data ,INT8U length)
{
    INT8U i;
    INT32S ret;
    DBG_PRINT("CEC TX[%d] ",length+1);
    for(i=0;i<(length+1);i++)
    {

        if(i==0)
        {
            DBG_PRINT("%x ",(initiator<<4)|destination);
            if(length == 0)
                ret = drv_l1_cec_tx((initiator<<4)|destination, 1,1);
            else
                ret = drv_l1_cec_tx((initiator<<4)|destination, 0,1);

            if(ret!=0)
            {
                DBG_PRINT("e1\r\n");
                return -1;
            }

            //DBG_PRINT("S");
        }
        else if(i==(length))
        {
            DBG_PRINT("%x ",*(data+i-1));
            ret = drv_l1_cec_tx(*(data+i-1), 1,1);

            if(ret!=0)
            {
                DBG_PRINT("e2\r\n");
                return -2;
            }
            //DBG_PRINT("E");
            DBG_PRINT("\r\n");
        }
        else
        {
            DBG_PRINT("%x ",*(data+i-1));
            ret = drv_l1_cec_tx(*(data+i-1), 0,1);
            if(ret!=0)
            {
                DBG_PRINT("e3\r\n");
                return -3;
            }
            //DBG_PRINT("O");

        }
    }
    return 0;
}

void HDMI_CEC_Report_PHY_ADDR(INT8U phya1, INT8U phya2)
{
    INT8U data[4] = {HDMI_CEC_CMDCODE_REPORT_PHYSICAL_ADDRESS,phya1,phya2,0x4};
    HDMI_CEC_Send_Command(own_logic_address,DEVICE_BOARDCAST,data,4);
}

//OSD name data first byte is 0x47 and length +1
void HDMI_CEC_Set_OSD_name(INT8U dest, INT8U *p, INT8U length)
{
    HDMI_CEC_Send_Command(own_logic_address,dest,p,length);
}

void HDMI_CEC_Report_device_vendor_id(INT32U vendor_id)
{
    INT8U data[4] = {HDMI_CEC_CMDCODE_DEVICE_VENDOR_ID,0x00,0x00,0x00};
    data[1] = vendor_id & 0xFF;
    data[2] = (vendor_id >> 8) & 0xFF;
    data[3] = (vendor_id >> 16) & 0xFF;
    HDMI_CEC_Send_Command(own_logic_address,DEVICE_BOARDCAST,data,4);
}

INT32S HDMI_CEC_Polling_Message(INT8U dest)
{
	return HDMI_CEC_Send_Command(own_logic_address,dest,0x00,0);
}

void HDMI_CEC_Active_Source(INT16U phy_addr)
{
    INT8U data[3] = {HDMI_CEC_CMDCODE_ACTIVE_SOURCE,0x00,0x00};
    data[1] = phy_addr>>8;
    data[2] = phy_addr;
    HDMI_CEC_Send_Command(own_logic_address,DEVICE_BOARDCAST,data,3);
}

void HDMI_CEC_Inactive_Source(INT16U phy_addr)
{
    INT8U data[3] = {HDMI_CEC_CMDCODE_INACTIVE_SOURCE,0x00,0x00};
    data[1] = phy_addr>>8;
    data[2] = phy_addr;
    HDMI_CEC_Send_Command(own_logic_address,0x00,data,3);
}

void HDMI_CEC_image_view_on(void)
{
    INT8U data[1] = {HDMI_CEC_CMDCODE_IMAGE_VIEW_ON};
    HDMI_CEC_Send_Command(own_logic_address,0x00,data,1);
}

void HDMI_CEC_menu_status(INT8U st)
{
    INT8U data[2] = {HDMI_CEC_CMDCODE_MENU_STATUS,0x00};
	data[1] = st;
    HDMI_CEC_Send_Command(own_logic_address,0x00,data,2);
}

void HDMI_CEC_text_view_on()
{

}

void HDMI_CEC_Feature_Abort(INT8U dest)
{
    INT8U data[3] = {0x00,0xFF,0x00};
    HDMI_CEC_Send_Command(own_logic_address,dest,data,3);
}

void EXTA_isr_callback(void)
{
    DBG_PRINT("EXTA\r\n");
    flag_hpd_addressing_step = 1;
}

void cec_task_init(void)
{
    osMessageQDef_t cec_q_def = {C_CEC_Q_ACCEPT_MAX, sizeof(void*),NULL};

    CEC_TaskQ = msgQCreate(CEC_QUEUE_MAX, CEC_QUEUE_MAX, CEC_PARA_MAX_LEN);

    cec_q = osMessageCreate(&cec_q_def, NULL);
	if(!cec_q) {
		DBG_PRINT("cec q err\r\n");
	}
}

void cec_task_entry(void *p_arg)
{
    INT32S  ret;
    INT32U  msg_id;
	INT32U  *CEC_TaskPara;
	INT32U  temp;
	cec_task_init();
    DBG_PRINT("%s\r\n",__func__);
    own_physcial_address = 0x1000;
    while (1)
	{
	    /* Pend task message */
	    ret = msgQReceive(CEC_TaskQ, &msg_id, (void*)cec_para, CEC_PARA_MAX_LEN);
        if(ret < 0) {
            continue;
        }

        CEC_TaskPara = (INT32U*)cec_para;
		switch(msg_id) {
            case HDMI_CEC_CMDCODE_REPORT_PHYSICAL_ADDRESS:
                temp = *CEC_TaskPara;
                DBG_PRINT("phy addr 0x%x\r\n",temp);
                HDMI_CEC_Report_PHY_ADDR((temp>>8)&0xFF,temp&0xFF);
			break;
			case HDMI_CEC_CMDCODE_DEVICE_VENDOR_ID:
                HDMI_CEC_Report_device_vendor_id(0x00100048);
			break;
			case HDMI_CEC_CMDCODE_ACTIVE_SOURCE:
                temp = *CEC_TaskPara;
                HDMI_CEC_Active_Source(temp);
			break;
            case HDMI_CEC_CMDCODE_INACTIVE_SOURCE:
                temp = *CEC_TaskPara;
                HDMI_CEC_Inactive_Source(temp);
            break;
			case HDMI_CEC_CMDCODE_FEATURE_ABORT:
                HDMI_CEC_Feature_Abort((rx_msg_id>>4)&0xF);
			break;
			case HDMI_CEC_CMDCODE_SET_OSD_NAME:
                HDMI_CEC_Set_OSD_name(((rx_msg_id>>4)&0xF),OSD_NAME,3);
			break;
			case HDMI_CEC_CMDCODE_IMAGE_VIEW_ON:
                HDMI_CEC_image_view_on();
			break;
			case HDMI_CEC_CMDCODE_MENU_STATUS:
			temp = *CEC_TaskPara;
				HDMI_CEC_menu_status(temp);
			break;
			case 0x100:
                ret = HDMI_CEC_Polling_Message(own_logic_address);
            break;
			default:
			break;
		}
		osDelay(10);
    }
}

void cec_sample_demo(void)
{
    INT32S ret;
    int i;
    INT16U cnt_addressing;
    INT8U edid_buf[128];
    INT8U dev_idx, dev_init_cnt;
    INT8U extend_block[128];
    INT32U temp_data;
    DBG_PRINT("CEC demo\r\n");
    INT32U cnt_dly_hpd;
    INT8U DTD_offset;
    INT8U find_offset;
    INT8U block_tag;
    INT8U block_length;
    osEvent event;

    //R_IOA_ATT |= 0x050;
    //R_IOA_DIR |= 0x050;
    //R_IOA_O_DATA = 0;

	//external interrupt
    R_IOB_ATT |= 0x100;
    R_IOB_DIR &= ~0x100;

    drv_l1_ext_user_isr_set(EXTA,EXTA_isr_callback);
    drv_l1_ext_edge_set(EXTA, RISING);
    drv_l1_ext_enable_set(EXTA, TRUE);
    //NVIC_EnableIRQ(EXTA_IRQn);
    //NVIC_SetPriority(EXTA_IRQn, 5);

    //R_INT_KECON |= 0x40; // clear int flag
    //R_INT_KECON &= ~0x08;
    //R_INT_KECON |= 0x1<<3; //rising edge
    //R_INT_KECON |= 0x1; //enable ext int A

    hpd_status = 0;
    cec_pinmux_set(MUX0);
    drv_l1_cec_init();
    drv_l1_cec_interrupt_enable(0x1FFF);
    //need pull high resister
    //enable CEC module, auto switch mux, but sensor priority  high need disable it
    dev_idx = 0;
    set_logic_address(dev_idx);

    drv_l1_cec_error_bit_config(BRE_STP | BRE_GEN | LBPE_GEN);
    cec_rx_buf = (INT8U*)gp_malloc(sizeof(INT8U)*RX_LEN);
    drv_l1_cec_assign_rx_buffer(cec_rx_buf,RX_LEN);
    drv_l1_cec_register_handler(CEC_ISR_RXEND,RX_get_data_isr);
    drv_l1_cec_register_handler(CEC_ISR_TXACKE,TX_polling_msg_ack_err_isr);
    drv_l1_cec_register_handler(CEC_ISR_TXEND,TX_polling_msg_get_ack_isr);

    cnt_dly_hpd = 0;
    ddc_sccb_open();
    cnt_addressing = 0;
    ret = xTaskCreate(cec_task_entry, "CEC_Task", C_CEC_TASK_STACK_SIZE, NULL, CEC_PRIORITY, NULL);
    if(ret == pdPASS) {
		DBG_PRINT("CEC Task Create[%d]\r\n", CEC_PRIORITY);
	} else {
		DBG_PRINT("CEC Task Create Fail[%d]\r\n", CEC_PRIORITY);
	}

	ret = xTaskCreate(cec_key_task_entry, "CEC_key_Task", C_CEC_TASK_STACK_SIZE, NULL, CEC_KEY_PRIORITY, NULL);
    if(ret == pdPASS) {
		DBG_PRINT("CEC KEY Task Create[%d]\r\n", CEC_KEY_PRIORITY);
	} else {
		DBG_PRINT("CEC KEY Task Create Fail[%d]\r\n", CEC_KEY_PRIORITY);
	}

    polling_msg_en = 0;
    dev_init_cnt = 0;
    while(1)
    {

        if((R_IOB_I_DATA&0x100) == 0x0 )
        {
            if(hpd_status==1)
            {
                cnt_dly_hpd++;
                if(cnt_dly_hpd==3)
                {
                    hpd_status = 0;
                    cnt_dly_hpd = 0;
                }
            }
        }
        else
        {
            if(hpd_status==0)
            {
                cnt_dly_hpd++;
                if(cnt_dly_hpd==3)
                {
                    hpd_status = 1;
                    cnt_dly_hpd = 0;
                }
            }
        }

        switch(flag_hpd_addressing_step)
        {
            case 1:
            case 2:
                //get EDID table
                if(flag_hpd_addressing_step==1)
                {
                    dev_init_cnt = 0;
                    flag_hpd_addressing_step = 2;
                    hdmi_read_EDID(edid_buf);
/*
                    DBG_PRINT("   00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\r\n");
                    for(i=0;i<128;i++)
                    {
                        if((i%16)==0)
                            DBG_PRINT("\r\n%2x ",i/16);
                        DBG_PRINT("%2x ",edid_buf[i]);
                    }
                    DBG_PRINT("\r\n----\r\n");
*/
                    if(edid_buf[126]!=0)//check extend block table
                    {
                        osDelay(1);
                        hdmi_read_block(0x80,extend_block,128);
                        DTD_offset = extend_block[2];
                        find_offset = 4;//first block
                        while(find_offset < DTD_offset)
                        {
                            block_tag = ((extend_block[find_offset]&0xE0)>>5);
                            block_length = (extend_block[find_offset]&0x1F)+1;
                            DBG_PRINT("[%d][%x %x %x]",find_offset,extend_block[find_offset],block_tag,block_length);

                            switch(block_tag)
                            {
                                case 2: // video data block
                                    DBG_PRINT("video DB\r\n");
                                break;
                                case 1: // Audio data block
                                    DBG_PRINT("Audio DB\r\n");
                                break;
                                case 4: // speaker allocatn data block
                                    DBG_PRINT("speaker DB\r\n");
                                break;
                                case 3: // VSDB
                                    DBG_PRINT("VSDB\r\n");
                                    temp_data = (extend_block[find_offset+4]<<8) | (extend_block[find_offset+5]);//physical address
                                break;
                                case 7: // colorimetry/video cap
                                    DBG_PRINT("COL/VIDCAP\r\n");
                                break;
                                default:
                                    DBG_PRINT("unknown\r\n");
                                break;
                            }
                            find_offset += block_length;
                        }
                    }
/*
                    DBG_PRINT("   00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\r\n");

                    for(i=0;i<128;i++)
                    {
                        if((i%16)==0)
                            DBG_PRINT("\r\n%2x ",i/16);
                        DBG_PRINT("%2x ",extend_block[i]);
                    }
                    DBG_PRINT("\r\n----\r\n");
*/
                    own_physcial_address = temp_data;
                }

                xQueueReset(cec_q);
                polling_msg_en = 1;
                msgQSend(CEC_TaskQ, 0x100, (void *)0, CEC_PARA_MAX_LEN, MSG_PRI_NORMAL);
                event = osMessageGet(cec_q, osWaitForever);
                if (event.status != osEventMessage) {
                    continue;
                }
                switch(event.value.v)
                {
                    case 0x2000: //not ack, logical address usable
                        flag_hpd_addressing_step = 3;
                        polling_msg_en = 0;
                    break;
                    case 0x3000: //acked, polling next logical address
                        if(dev_init_cnt < logic_address_cnt_table[own_logic_type])
                        {
                            dev_idx++;
                            own_logic_address = logic_address_table[own_logic_type][dev_idx];
                            if(own_logic_address==0xF)
                            {
                                dev_idx = 0;
                                own_logic_address = logic_address_table[own_logic_type][dev_idx];
                            }
                            dev_init_cnt++;
                        }
                        else
                        {
                            dev_idx = logic_address_cnt_table[own_logic_type];
                            polling_msg_en = 0;
                            flag_hpd_addressing_step = 4;
                        }
                        set_logic_address(dev_idx);

                    break;
                    default:break;
                }

            break;
            case 3: // check again
                xQueueReset(cec_q);
                polling_msg_en = 1;
                msgQSend(CEC_TaskQ, 0x100, (void *)0, CEC_PARA_MAX_LEN, MSG_PRI_NORMAL);
                event = osMessageGet(cec_q, osWaitForever);
                if (event.status != osEventMessage) {
                    continue;
                }
                switch(event.value.v)
                {
                    case 0x2000:
                        flag_hpd_addressing_step = 4;
                        polling_msg_en = 0;
                    break;
                    case 0x3000:
                        if(dev_init_cnt < logic_address_cnt_table[own_logic_type])
                        {
                            dev_idx++;
                            own_logic_address = logic_address_table[own_logic_type][dev_idx];
                            if(own_logic_address == 0xF)
                            {
                                dev_idx = 0;
                                own_logic_address = logic_address_table[own_logic_type][dev_idx];
                            }
                            dev_init_cnt++;
                        }
                        else
                        {
                            dev_idx = logic_address_cnt_table[own_logic_type];
                            polling_msg_en = 0;
                            flag_hpd_addressing_step = 4;
                        }
                        set_logic_address(dev_idx);
                    break;
                    default:break;
                }
            break;
            case 4:
                msgQSend(CEC_TaskQ, HDMI_CEC_CMDCODE_REPORT_PHYSICAL_ADDRESS, (void *)&temp_data, CEC_PARA_MAX_LEN, MSG_PRI_NORMAL);
                flag_hpd_addressing_step++;
            break;
            case 5:
                msgQSend(CEC_TaskQ, HDMI_CEC_CMDCODE_DEVICE_VENDOR_ID, (void *)0, CEC_PARA_MAX_LEN, MSG_PRI_NORMAL);
                flag_hpd_addressing_step=0;
                xQueueReset(cec_q);
            break;
            default:break;
        }

		osDelay(35);
    }
}

