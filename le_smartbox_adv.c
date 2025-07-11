/*********************************************************************************************
    *   Filename        : le_smartbox_adv.c

    *   Description     :

    *   Author          :

    *   Email           : zh-jieli.com

    *   Last modifiled  : 2022-06-08 11:14

    *   Copyright:(c)JIELI  2011-2022  @ , All Rights Reserved.
*********************************************************************************************/
// *****************************************************************************
#include "system/app_core.h"
#include "system/includes.h"
#include "smartbox/config.h"

#include "app_action.h"

#include "btstack/btstack_task.h"
#include "btstack/bluetooth.h"
#include "user_cfg.h"
#include "vm.h"
#include "app_power_manage.h"
#include "btcontroller_modules.h"
#include "bt_common.h"
#include "3th_profile_api.h"
#include "btstack/avctp_user.h"
#include "app_chargestore.h"

#include "btcrypt.h"
#include "custom_cfg.h"
#include "smartbox_music_info_setting.h"
#include "classic/tws_api.h"
#include "le_smartbox_module.h"
#include "smartbox_rcsp_manage.h"
#include "bt/bt_tws.h"

#include "asm/charge.h"
#include "ute/log/ute_module_log.h"
#include "debug.h"
#include "ui/ui_sys_param.h"
#include "app_ble_spp_api.h"

#if (TCFG_BLE_DEMO_SELECT == DEF_BLE_DEMO_RCSP_DEMO)

extern void *le_smartbox_ble_hdl;

#if TCFG_USER_BLE_CTRL_BREDR_EN
#define VER_FLAG_BLE_CTRL_BREDR					(BIT(0)) // 先连接ble再连接edr
#else /* #if TCFG_USER_BLE_CTRL_BREDR_EN */
#define VER_FLAG_BLE_CTRL_BREDR					(0)
#endif /* #if TCFG_USER_BLE_CTRL_BREDR_EN */

#define VER_FLAG_IOS_BLE_LINK_BREDR				BIT(1) // ios一次连接

#if 1
#define log_info(x, ...)       printf("[LE-SMART-ADV]" x " ", ## __VA_ARGS__)
#define log_info_hexdump       put_buf
#else
#define log_info(...)
#define log_info_hexdump(...)
#endif

static u8 adv_data_len;
static u8 adv_data[ADV_RSP_PACKET_MAX];//max is 31
static u8 scan_rsp_data_len;
static u8 scan_rsp_data[ADV_RSP_PACKET_MAX];//max is 31

#if RCSP_SMARTBOX_ADV_EN
static u32 ble_timer_handle = 0;
u16 sibling_ver_info = 0;

#define BLE_TIMER_SET (500)

struct _bt_adv_handle {

    u8 seq_rand;
    u8 connect_flag;

    u8 bat_charge_L;
    u8 bat_charge_R;
    u8 bat_charge_C;
    u8 bat_percent_L;
    u8 bat_percent_R;
    u8 bat_percent_C;

    u8 modify_flag;
    u8 adv_disable_cnt;

    u8 ble_adv_notify;
} bt_adv_handle = {0};

#define __this (&bt_adv_handle)

extern u8 get_tws_sibling_bat_persent(void);

u8 get_adv_notify_state(void)
{
    return __this->ble_adv_notify;
}

void set_adv_notify_state(u8 notify)
{
    __this->ble_adv_notify  = notify;
}

static void rcsp_adv_fill_mac_addr(u8 *mac_addr_buf)
{
#if (MUTIl_CHARGING_BOX_EN)
    extern u8 *get_chargebox_adv_addr(void);
    u8 *mac_addr = get_chargebox_adv_addr();
    if (mac_addr) {
        swapX(mac_addr, mac_addr_buf, 6);
    }

    /* log_info("mac_addr:"); */
    /* log_info_buf(mac_addr_buf, 6); */

#else
    swapX(bt_get_mac_addr(), mac_addr_buf, 6);
#endif
}

int smartbox_make_set_adv_data(void)
{
    u8 i;
    u8 *buf = adv_data;
    buf[0] = 0x1E;
    buf[1] = 0xFF;

    buf[2] = 0xD6;	// JL ID
    buf[3] = 0x05;

#if 1
    u16 vid = get_vid_pid_ver_from_cfg_file(GET_VID_FROM_EX_CFG);
    buf[4] = vid & 0xFF;
    buf[5] = vid >> 8;

    u16 pid = get_vid_pid_ver_from_cfg_file(GET_PID_FROM_EX_CFG);
    buf[6] = pid & 0xFF;
    buf[7] = pid >> 8;
#else
    buf[4] = 0x02;	// VID
    buf[5] = 0x00;

    buf[6] = 0x33;	// PID
    buf[7] = 0x00;
#endif

#if (MUTIl_CHARGING_BOX_EN)
    buf[8] = 0x01;	//   0:音箱类型   |  protocol verson
#else
    buf[8] = 0x0;	//   0:音箱类型   |  protocol verson
    if (RCSP_SPP == get_defalut_bt_channel_sel()) {
        buf[8] |= 2;
    }
#endif

    rcsp_adv_fill_mac_addr(&buf[9]);
    buf[15] = __this->connect_flag;

    /* buf[16] = 0xC1; */
    /* buf[17] = 0x40; */
    /* buf[18] = 0x30; */
    buf[16] = __this->bat_percent_L ? (((!!__this->bat_charge_L) << 7) | (__this->bat_percent_L & 0x7F)) : 0;
    buf[17] = __this->bat_percent_R ? (((!!__this->bat_charge_R) << 7) | (__this->bat_percent_R & 0x7F)) : 0;
    buf[18] = __this->bat_percent_C ? (((!!__this->bat_charge_C) << 7) | (__this->bat_percent_C & 0x7F)) : 0;

    memset(&buf[19], 0x00, 4);							// reserve

    buf[19] = __this->seq_rand;

#if (MUTIl_CHARGING_BOX_EN)
    buf[20] = 1;
#else
    if (RCSP_SPP == get_defalut_bt_channel_sel()) {
        buf[20] = 1;
    }
#endif

    u8 t_buf[16];
    btcon_hash(&buf[2], 16, &buf[15], 4, t_buf);		// hash

    for (i = 0; i < 8; i++) {								// single byte
        buf[23 + i] = t_buf[2 * i + 1];
    }

    __this->modify_flag = 0;
    adv_data_len = 31;

    /* ble_op_set_adv_data(31, buf); */
    app_ble_adv_data_set(le_smartbox_ble_hdl, buf, offset);

    UTE_MODULE_LOG(UTE_LOG_SYSTEM_LVL,"ADV data():");
    log_info_hexdump(buf, 31);
    return 0;
}

int smartbox_make_set_rsp_data(void)
{
    u8 offset = 0;
    u8 *buf = scan_rsp_data;
    u8 *edr_name = bt_get_local_name();

    offset += make_eir_packet_val(&buf[offset], offset, HCI_EIR_DATATYPE_FLAGS, 0x06, 1);
    offset += make_eir_packet_data(&buf[offset], offset, HCI_EIR_DATATYPE_COMPLETE_LOCAL_NAME, (void *)edr_name, strlen(edr_name));
    /* log_info("RSP data(%d):,offset"); */
    /* log_info_hexdump(buf, offset); */

    scan_rsp_data_len = 31;

    /* ble_op_set_rsp_data(31, buf); */
    app_ble_rsp_data_set(le_smartbox_ble_hdl, buf, offset);

    return 0;
}


static int update_adv_data(u8 *buf)
{
    u8 i;
    buf[1] = 0xD6;	// JL ID
    buf[0] = 0x05;

#if 1
    u16 vid = get_vid_pid_ver_from_cfg_file(GET_VID_FROM_EX_CFG);
    buf[3] = vid & 0xFF;
    buf[2] = vid >> 8;

    u16 pid = get_vid_pid_ver_from_cfg_file(GET_PID_FROM_EX_CFG);
    buf[5] = pid & 0xFF;
    buf[4] = pid >> 8;
#else
    buf[3] = 0x02;	// VID
    buf[2] = 0x00;

    buf[5] = 0x33;	// PID
    buf[4] = 0x00;
#endif

    buf[6] = 0x0;	//   2:音箱类型   |  protocol verson
    if (RCSP_SPP == get_defalut_bt_channel_sel()) {
        buf[6] |= 2;
    }

    swapX(bt_get_mac_addr(), &buf[7], 6);

    buf[13] = __this->connect_flag;

    buf[14] = __this->bat_percent_L ? (((!!__this->bat_charge_L) << 7) | (__this->bat_percent_L & 0x7F)) : 0;
    buf[15] = __this->bat_percent_R ? (((!!__this->bat_charge_R) << 7) | (__this->bat_percent_R & 0x7F)) : 0;
    buf[16] = __this->bat_percent_C ? (((!!__this->bat_charge_C) << 7) | (__this->bat_percent_C & 0x7F)) : 0;

    buf[17] = __this->seq_rand;

    return 0;
}


u8 *ble_get_scan_rsp_ptr(u16 *len)
{
    if (len) {
        *len = scan_rsp_data_len;
    }
#if 0//RCSP_UPDATE_EN
    u8 offset = 0;
    offset = make_eir_packet_val(&scan_rsp_data[offset], offset, HCI_EIR_DATATYPE_FLAGS, 0x02, 1);
#endif
    return scan_rsp_data;
}

u8 *ble_get_adv_data_ptr(u16 *len)
{
    if (len) {
        *len = adv_data_len;
    }
#if RCSP_UPDATE_EN
    adv_data[15] = 1;
    adv_data[20] = 0;
    u8 t_buf[16];
    btcon_hash(&adv_data[2], 16, &adv_data[15], 4, t_buf);
    for (u8 i = 0; i < 8; i++) {
        adv_data[23 + i] = t_buf[2 * i + 1];
    }
#endif
    return adv_data;
}





static u8 update_dev_battery_level(void)
{

    u8 master_bat = 0;
    u8 master_charge = 0;
    u8 slave_bat = 0;
    u8 slave_charge = 0;
    u8 box_bat = 0;
    u8 box_charge = 0;

//	Master bat
#if CONFIG_DISPLAY_DETAIL_BAT
    master_bat = get_vbat_percent();
#else
    master_bat = get_self_battery_level() * 10 + 10;
#endif
    if (master_bat > 100) {
        master_bat = 100;
    }
    master_charge = get_charge_online_flag();


// Slave bat

#if	TCFG_USER_TWS_ENABLE
    slave_bat = get_tws_sibling_bat_persent();
#if TCFG_CHARGESTORE_ENABLE
    if (slave_bat == 0xff) {
        /* log_info("--update_bat_01\n"); */
        if (get_bt_tws_connect_status()) {
            slave_bat = chargestore_get_sibling_power_level();
        }
    }
#endif

    if (slave_bat == 0xff) {
        /*  log_info("--update_bat_02\n"); */
        slave_bat = 0x00;
    }

    slave_charge = !!(slave_bat & 0x80);
    slave_bat &= 0x7F;
#else
    slave_charge = 0;
    slave_bat = 0;
#endif

// Box bat
    if ((master_charge && (master_bat != 0))
        || (slave_charge && (slave_bat != 0))) {
        //earphone in charge
#if TCFG_CHARGESTORE_ENABLE
        box_bat = chargestore_get_power_level();
        if (box_bat == 0xFF) {
            box_bat = 0;
        }
#else
        box_bat = 0;
#endif
    } else {
        //all not in charge
        box_bat = 0;
    }
    box_charge = !!(box_bat & 0x80);
    box_bat &= 0x7F;
// check if master is L

    u8 tbat_charge_L = 0;
    u8 tbat_charge_R = 0;
    u8 tbat_percent_L = 0;
    u8 tbat_percent_R = 0;
    u8 tbat_percent_C = box_bat;
    u8 tbat_charge_C = box_charge;
#if TCFG_USER_TWS_ENABLE
    if (get_bt_tws_connect_status()) {
        if ('L' == tws_api_get_local_channel()) {
            tbat_charge_L = master_charge;
            tbat_charge_R = slave_charge;
            tbat_percent_L = master_bat;
            tbat_percent_R = slave_bat;
        } else if ('R' == tws_api_get_local_channel()) {
            tbat_charge_L = slave_charge;
            tbat_charge_R = master_charge;
            tbat_percent_L = slave_bat;
            tbat_percent_R = master_bat;
        }
    } else {
        switch (tws_api_get_local_channel()) {
        case 'R':
            tbat_charge_R = master_charge;
            tbat_percent_R = master_bat;
            break;
        case 'L':
        default:
            tbat_charge_L = master_charge;
            tbat_percent_L = master_bat;
            break;
        }
    }
#else
    tbat_charge_L = master_charge;
    tbat_percent_L = master_bat;
#endif

#if 0//TCFG_CHARGESTORE_ENABLE
    u8 tpercent = 0;
    u8 tcharge = 0;
    if (chargestore_get_earphone_pos() == 'L') {
        //switch position
        UTE_MODULE_LOG(UTE_LOG_SYSTEM_LVL,"is L pos\n");
        tpercent = tbat_percent_R;
        tcharge = tbat_charge_R;
        tbat_percent_R = tbat_percent_L;
        tbat_charge_R = tbat_charge_L;
        tbat_percent_L = tpercent;
        tbat_charge_L = tcharge;
    } else {
        UTE_MODULE_LOG(UTE_LOG_SYSTEM_LVL,"is R pos\n");
    }
#endif

    if ((!!__this->bat_charge_L) || (!!__this->bat_charge_R) || (!!__this->bat_charge_C)) {
        if (((__this->bat_percent_C + 1) & 0x7F) <= 0x2) {
            __this->bat_percent_C = 0x2;
        }
    }
// check if change
    u8 tchange = 0;
    if ((tbat_charge_L  != __this->bat_charge_L)
        || (tbat_charge_R  != __this->bat_charge_R)
        || (tbat_charge_C  != __this->bat_charge_C)
        || (tbat_percent_L != __this->bat_percent_L)
        || (tbat_percent_R != __this->bat_percent_R)
        || (tbat_percent_C != __this->bat_percent_C)) {
        tchange = 1;
    }

    __this->bat_charge_L  = tbat_charge_L;
    __this->bat_charge_R  = tbat_charge_R;
    __this->bat_charge_C  = tbat_charge_C;
    __this->bat_percent_L = tbat_percent_L;
    __this->bat_percent_R = tbat_percent_R;
    __this->bat_percent_C = tbat_percent_C;

    /* u8 *buf = adv_data; */
    /* buf[16] = __this->bat_percent_L ? (((!!__this->bat_charge_L) << 7) | (__this->bat_percent_L & 0x7F)):0; */
    /* buf[17] = __this->bat_percent_R ? (((!!__this->bat_charge_R) << 7) | (__this->bat_percent_R & 0x7F)):0; */
    /* buf[18] = __this->bat_percent_C ? (((!!__this->bat_charge_C) << 7) | (__this->bat_percent_C & 0x7F)):0; */

    return tchange;
}


static u8 update_dev_connect_flag(void)
{
    u8 old_flag = __this->connect_flag;


    extern u8 get_bt_connect_status(void);
    if (get_bt_connect_status() ==  BT_STATUS_WAITINT_CONN) {
        __this->connect_flag = 1;
    } else {
        __this->connect_flag = 2;
    }

    if (old_flag != __this->connect_flag) {
        return 1;
    }
    return 0;
}

static void bt_ble_rcsp_adv_enable_do(void)
{
#if(TCFG_CHARGE_BOX_ENABLE)
    extern u8 get_tws_ear_status(void);
    if (!get_tws_ear_status()) {
        return;
    }
#endif

#if TCFG_USER_TWS_ENABLE
    if (tws_api_get_role() == TWS_ROLE_SLAVE) {
        if (!(get_rcsp_connect_status() && RCSP_SPP == bt_3th_get_cur_bt_channel_sel())) {
            return;
        }
    }
#endif


    ble_state_e cur_state =  smartbox_get_ble_work_state();
    // battery
    if (update_dev_battery_level()) {
        __this->modify_flag = 1;
    }

    // connect flag
    /* if (update_dev_connect_flag()) { */
    /* __this->modify_flag = 1; */
    /* } */

    extern u8 adv_info_notify(u8 * buf, u8 len);
    extern u8 JL_rcsp_get_auth_flag(void);
#if (!MUTIl_CHARGING_BOX_EN) && (RCSP_ADV_EN) && (TCFG_USER_TWS_ENABLE)
    if ((smartbox_get_con_handle() || get_rcsp_connect_status()) && JL_rcsp_get_auth_flag()) {
        /* extern void deal_adv_setting_gain_time_stamp(void); */
        /* deal_adv_setting_gain_time_stamp(); */
    }
#endif
    if ((smartbox_get_con_handle() || get_rcsp_connect_status()) && JL_rcsp_get_auth_flag() && (__this->ble_adv_notify || __this->modify_flag)) {
        UTE_MODULE_LOG(UTE_LOG_SYSTEM_LVL,"adv con_handle %d\n", smartbox_get_con_handle());
        /* u8 data[5]; */
        /* data[0] = __this->seq_rand; */
        /* data[1] = __this->connect_flag; */
        /* data[2] = __this->bat_percent_L ? (((!!__this->bat_charge_L) << 7) | (__this->bat_percent_L & 0x7F)):0; */
        /* data[3] = __this->bat_percent_R ? (((!!__this->bat_charge_R) << 7) | (__this->bat_percent_R & 0x7F)):0; */
        /* data[4] = __this->bat_percent_C ? (((!!__this->bat_charge_C) << 7) | (__this->bat_percent_C & 0x7F)):0; */
        /* adv_info_notify(data, 5); */
        u8 data[18];
        update_adv_data(data);
        adv_info_notify(data, 18);
        __this->modify_flag = 0;
        return;
    }

#if (!MUTIl_CHARGING_BOX_EN)
    if (cur_state != BLE_ST_ADV) {
        return;
    }
#endif

    if (__this->modify_flag == 0) {
        return;
    }

    log_info("adv modify!!!!!!\n");
    le_smartbox_bt_ble_adv_enable(0);
    smartbox_make_set_adv_data();
    smartbox_make_set_rsp_data();
    le_smartbox_bt_ble_adv_enable(1);
}

void bt_ble_rcsp_adv_enable(void)
{
#if (MUTIl_CHARGING_BOX_EN)
#if (TCFG_CHARGE_BOX_ENABLE)
    extern u8 get_chgbox_lid_status(void);
    if (0 == get_chgbox_lid_status()) {
        return;
    }
#endif
    __this->modify_flag = 1;
    bt_ble_rcsp_adv_enable_do();
#endif
    ble_timer_handle = sys_timer_add(NULL, bt_ble_rcsp_adv_enable_do, BLE_TIMER_SET);
}


void bt_ble_rcsp_adv_disable(void)
{
    if (ble_timer_handle) {
        sys_timer_del(ble_timer_handle);
        ble_timer_handle = 0;
    }
    ble_app_disconnect();
    smartbox_set_adv_enable(0, 0);
    bt_adv_seq_change();
}

// 同步seq到对端
#define TWS_FUNC_ID_SEQ_RAND_SYNC	(('S' << (3 * 8)) | ('E' << (2 * 8)) | ('Q' << (1 * 8)) | ('\0'))
void tws_conn_switch_role();
static void deal_sibling_seq_rand_sync(void *data, u16 len, bool rx)
{
    log_info_hexdump(data, len);
    if (rx) {
        switch (((u8 *)data)[0]) {
        case TWS_ADV_SEQ_CHANGE:
            __this->seq_rand = ((u8 *) data)[1];
            JL_rcsp_event_to_user(DEVICE_EVENT_FROM_RCSP, MSG_JL_UPDATE_SEQ, NULL, 0);
            break;
        case TWS_VERSON_INFO:
            UTE_MODULE_LOG(UTE_LOG_SYSTEM_LVL,"get sibling version:0x%x\n", ((u8 *)data)[1] | ((u8 *)data)[2] << 8);
            sibling_ver_info  = ((u8 *)data)[1] | ((u8 *)data)[2] << 8;
            u16 cur_ver_info  = get_vid_pid_ver_from_cfg_file(GET_VER_FROM_EX_CFG);
            if ((cur_ver_info > sibling_ver_info) && (tws_api_get_role() == TWS_ROLE_MASTER)) { //如果主机版本号比从机高需要进行role_switch
                //tws_conn_switch_role();
            }
            break;

        }
    }
}
REGISTER_TWS_FUNC_STUB(adv_seq_rand_sync) = {
    .func_id = TWS_FUNC_ID_SEQ_RAND_SYNC,
    .func    = deal_sibling_seq_rand_sync,
};
void adv_seq_vaule_sync(void)
{
    syscfg_write(ADV_SEQ_RAND, &__this->seq_rand, sizeof(__this->seq_rand));
}
void bt_adv_seq_change(void)
{
    u8 trand;
    u8 data[2];
    syscfg_read(ADV_SEQ_RAND, &trand, 1);
    UTE_MODULE_LOG(UTE_LOG_SYSTEM_LVL,"adv seq read: %x\n", trand);
    trand++;
    syscfg_write(ADV_SEQ_RAND, &trand, 1);
    UTE_MODULE_LOG(UTE_LOG_SYSTEM_LVL,"adv seq write: %x\n", trand);
    __this->seq_rand = trand;
    data[0] = TWS_ADV_SEQ_CHANGE;
    data[1] = __this->seq_rand;
#if TCFG_USER_TWS_ENABLE
    if (get_bt_tws_connect_status() && tws_api_get_role() == TWS_ROLE_MASTER) {
        tws_api_send_data_to_sibling(data, sizeof(data), TWS_FUNC_ID_SEQ_RAND_SYNC);
    }
#endif
}

u8 get_ble_adv_notify(void)
{
    return __this->ble_adv_notify;
}

void set_ble_adv_notify(u8 en)
{

    __this->ble_adv_notify = en;
}

u8 get_connect_flag(void)
{
    return __this->connect_flag;
}

void set_connect_flag(u8 value)
{
    __this->connect_flag = 	value;
}

void bt_ble_init_do(void)
{
    UTE_MODULE_LOG(UTE_LOG_SYSTEM_LVL,"bt_ble_test_tag_do\n");
    //初始化三个电量值
    /* swapX(bt_get_mac_addr(), &user_tag_data[7], 6); */

    __this->connect_flag = 0;
    __this->bat_charge_L = 1;
    __this->bat_charge_R = 1;
    __this->bat_charge_C = 1;
    __this->bat_percent_L = 100;
    __this->bat_percent_R = 100;
    __this->bat_percent_C = 100;
    __this->modify_flag = 0;
    __this->ble_adv_notify = 0;
    update_dev_battery_level();
    bt_adv_seq_change();
    //update_dev_connect_flag();
}

int bt_ble_adv_ioctl(u32 cmd, u32 priv, u8 mode)
{
    UTE_MODULE_LOG(UTE_LOG_SYSTEM_LVL," bt_ble_adv_ioctl %d %d %d\n", cmd, priv, mode);
    ble_state_e cur_state =  smartbox_get_ble_work_state();
    if (mode) {			// set
        switch (cmd) {
        case BT_ADV_ENABLE:
            break;
        case BT_ADV_DISABLE:
            if (cur_state == BLE_ST_ADV) {
                log_info("ADV DISABLE !!!\n");
                le_smartbox_bt_ble_adv_enable(0);
                return 0;
            }
            __this->connect_flag = SECNE_DISMISS;
            break;
        case BT_ADV_SET_EDR_CON_FLAG:
            __this->connect_flag = priv;
            if (priv == SECNE_UNCONNECTED) {
                bt_adv_seq_change();
            }
#if (RCSP_ADV_MUSIC_INFO_ENABLE)
            extern void rcsp_adv_music_info_set_state(u8 state, u32 time);
            if (priv == SECNE_UNCONNECTED) {
                rcsp_adv_music_info_set_state(0, 1);
            } else if (priv == SECNE_CONNECTED) {
                rcsp_adv_music_info_set_state(1, 1000);
            }
#endif
            break;
        case BT_ADV_SET_BAT_CHARGE_L:
            __this->bat_charge_L = priv;
            break;
        case BT_ADV_SET_BAT_CHARGE_R:
            __this->bat_charge_R = priv;
            break;
        case BT_ADV_SET_BAT_CHARGE_C:
            __this->bat_charge_C = priv;
            break;
        case BT_ADV_SET_BAT_PERCENT_L:
            __this->bat_percent_L = priv;
            break;
        case BT_ADV_SET_BAT_PERCENT_R:
            __this->bat_percent_R = priv;
            break;
        case BT_ADV_SET_BAT_PERCENT_C:
            __this->bat_percent_C = priv;
            break;
        case BT_ADV_SET_NOTIFY_EN:
            __this->ble_adv_notify = priv;
            break;
        default:
            return -1;
        }

        __this->modify_flag = 1;
    } else {			// get
        switch (cmd) {
        case BT_ADV_ENABLE:
        case BT_ADV_DISABLE:
            break;
        case BT_ADV_SET_EDR_CON_FLAG:
            break;
        case BT_ADV_SET_BAT_CHARGE_L:
            break;
        case BT_ADV_SET_BAT_CHARGE_R:
            break;
        case BT_ADV_SET_BAT_CHARGE_C:
            break;
        case BT_ADV_SET_BAT_PERCENT_L:
            break;
        case BT_ADV_SET_BAT_PERCENT_R:
            break;
        case BT_ADV_SET_BAT_PERCENT_C:
            break;
        default:
            break;
        }
    }
    return 0;
}





void bt_adv_get_bat(u8 *buf)
{
    buf[0] = __this->bat_percent_L ? (((!!__this->bat_charge_L) << 7) | (__this->bat_percent_L & 0x7F)) : 0;
    buf[1] = __this->bat_percent_R ? (((!!__this->bat_charge_R) << 7) | (__this->bat_percent_R & 0x7F)) : 0;
    buf[2] = __this->bat_percent_C ? (((!!__this->bat_charge_C) << 7) | (__this->bat_percent_C & 0x7F)) : 0;
}

extern u8 jl_app_init_setting(void);
u8 btstck_app_init_setting(void)
{
    jl_app_init_setting();
    return 1;
}

#define TWS_FUNC_ID_ADV_STATE_INFO_SYNC	(('A' << (3 * 8)) | ('S' << (2 * 8)) | ('S' << (1 * 8)) | ('Y'))
static void deal_sibling_adv_state_info_sync(void *data, u16 len, bool rx)
{
    if (rx) {
        JL_rcsp_event_to_user(DEVICE_EVENT_FROM_RCSP, MSG_JL_UPDAET_ADV_STATE_INFO, data, len);
    }
}
REGISTER_TWS_FUNC_STUB(adv_state_sync) = {
    .func_id = TWS_FUNC_ID_ADV_STATE_INFO_SYNC,
    .func    = deal_sibling_adv_state_info_sync,
};
static void adv_state_info_sync(void)
{
#if TCFG_USER_TWS_ENABLE
    if (get_bt_tws_connect_status()) {
        u8 param[2] = {0};
        param[0] = JL_rcsp_get_auth_flag();
        param[1] = __this->ble_adv_notify;
        tws_api_send_data_to_sibling(param, sizeof(param), TWS_FUNC_ID_ADV_STATE_INFO_SYNC);
    }
#endif
}

// type : 0 - 正常关机检查		// 1 - 低电关机检测
u8 adv_tws_both_in_charge_box(u8 type)
{
    if (0 == ((!!__this->bat_percent_L) & (!!__this->bat_percent_R))) {
        return 0;
    }
    static u8 ret = 0;
    if (ret) {
        goto check_changes;
    }

    if (type) {
#if TCFG_USER_TWS_ENABLE
        if (TWS_ROLE_MASTER == tws_api_get_role()) {
            ret = 1;
        }
#endif
        return ret;
    }

    ret = ((!!__this->bat_charge_L) ^ (!!__this->bat_charge_R));
check_changes:
    if (smartbox_get_con_handle()) {
        return ret;
    }
    if (get_rcsp_connect_status() && ret) {
        adv_state_info_sync();
    }
    return 0;
}

// 切换后触发
void adv_role_switch_handle(void)
{
#if TCFG_USER_TWS_ENABLE
    if (tws_api_get_tws_state()) {
        if (RCSP_BLE == bt_3th_get_cur_bt_channel_sel()) {
            if (tws_api_get_role() == TWS_ROLE_MASTER) {
                ble_module_enable(1);
            } else {
                u8 adv_cmd = 0x3;
                extern u8 adv_info_device_request(u8 * buf, u16 len);
                adv_info_device_request(&adv_cmd, sizeof(adv_cmd));
                ble_module_enable(0);
            }
        }
    }
#endif
}

void send_version_to_sibling(void)
{
    u8 data[3] = {0};
    u16 ver = 0;
    ver =  get_vid_pid_ver_from_cfg_file(GET_VER_FROM_EX_CFG);
    UTE_MODULE_LOG(UTE_LOG_SYSTEM_LVL,"%s---verson:%x\n", ver);
    data[0] = TWS_VERSON_INFO;
    data[1] = ver;
    data[2] = ver >> 8;
    tws_api_send_data_to_sibling(data, sizeof(data), TWS_FUNC_ID_SEQ_RAND_SYNC);
}

u16 rebuild_adv_rcsp_info(u8 *adv_rsp_buf, u16 buf_size, u8 type)
{
    return 0;
}


#else // RCSP_SMARTBOX_ADV_EN

extern void swapX(const uint8_t *src, uint8_t *dst, int len);
extern const u8 *bt_get_mac_addr();
static void rcsp_adv_fill_mac_addr(u8 *mac_addr_buf)
{
    u32 rets;
    __asm__ volatile("%0 = rets":"=r"(rets));
    UTE_MODULE_LOG(UTE_LOG_SYSTEM_LVL,"__func__ %s %x\n", __func__, rets);
    swapX(bt_get_mac_addr(), mac_addr_buf, 6);
    UTE_MODULE_LOG(UTE_LOG_SYSTEM_LVL,"adv");
    put_buf(mac_addr_buf,sizeof(mac_addr_buf));
}


int smartbox_make_set_rsp_data(void)
{
    u8 offset = 0;
#if TCFG_USR_DIVIDE_EDR_BLE_NAME
    u8 *temp1 = BT_BLE_DEV_NAME;//bt_get_local_name();
    u8 str_len = strlen(temp1);
    u8 temp2[LOCAL_NAME_LEN];
    memset(temp2,0,LOCAL_NAME_LEN);
    memcpy(temp2,temp1,str_len);
    u8 *edr_name = (const char*)temp2;
    temp1 = NULL;
#else
    u8 *edr_name = (u8 *)bt_get_local_name();
#endif
    u8 *buf = scan_rsp_data;

#if 0//DOUBLE_BT_SAME_MAC/*支付宝测试*/
    offset += make_eir_packet_val(&buf[offset], offset, HCI_EIR_DATATYPE_FLAGS, 0x0A, 1);//单模、0a是双模，06单模，
#else
    offset += make_eir_packet_val(&buf[offset], offset, HCI_EIR_DATATYPE_FLAGS, 0x06, 1);
#endif

    u8 name_len = strlen(edr_name) + 1;
    // 升级前可能需要在蓝牙名后添加_ota这四个字符的关键字
    if (offset + 2 + name_len > ADV_RSP_PACKET_MAX) {
         log_info("ble name too long!!!\n");/*支付宝测试*/
         name_len = ADV_RSP_PACKET_MAX - (offset + 2);
        //puts("***rsp_data overflow!!!!!!\n");
        //return -1;
    }
    offset += make_eir_packet_data(&buf[offset], offset, HCI_EIR_DATATYPE_COMPLETE_LOCAL_NAME, (void *)edr_name, name_len);

    u8 temp_buf[8] ={0x00,0x00};//u8 temp_buf[] = {0x6E,0x49,0x00,0x3E,0x00,0x5A,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

    u8 tmp_data[6];
    le_controller_get_mac(tmp_data);

    //mac
	temp_buf[2] = tmp_data[5];	
	temp_buf[3] = tmp_data[4];
	temp_buf[4] = tmp_data[3];
	temp_buf[5] = tmp_data[2];
	temp_buf[6] = tmp_data[1];
	temp_buf[7] = tmp_data[0];
    offset += make_eir_packet_data(&buf[offset], offset, HCI_EIR_DATATYPE_MANUFACTURER_SPECIFIC_DATA, temp_buf, sizeof(temp_buf));
    
    scan_rsp_data_len = offset;
    UTE_MODULE_LOG(UTE_LOG_SYSTEM_LVL,"adv_data(%d):", offset);
    put_buf(buf, offset);
    //ble_op_set_rsp_data(offset, buf);/*fixed 31bytes*/
    app_ble_rsp_data_set(le_smartbox_ble_hdl, buf, offset);
    return 0;
}

int smartbox_make_set_adv_data(void)
{
    u8 offset = 0;
    u8 *buf = adv_data;
	
#if UTE_BLE_APP_USER_PROFILE

	offset += make_eir_packet_val(&buf[offset], offset, HCI_EIR_DATATYPE_FLAGS, 0x06, 1);

	u8 service_data[6] = {0x36, 0x55, 0x22, 0x22,0x55,0x55};
    offset += make_eir_packet_data(&buf[offset], offset, HCI_EIR_DATATYPE_COMPLETE_16BIT_SERVICE_UUIDS, service_data, 6);

    // u8 *temp1 = BT_BLE_DEV_NAME;//bt_get_local_name();
    // offset += make_eir_packet_data(&buf[offset], offset, HCI_EIR_DATATYPE_COMPLETE_LOCAL_NAME, temp1, strlen(temp1));

	
    //log_info("8888888888888888888888  adv_data(%d):", offset);

#else

    buf[offset++] = 0x1E;
    buf[offset++] = 0xFF;

    buf[offset++] = 0xD6; // JL ID
    buf[offset++] = 0x05;

#if 1
    u16 vid = get_vid_pid_ver_from_cfg_file(GET_VID_FROM_EX_CFG);
    buf[offset++] = vid & 0xFF;
    buf[offset++] = vid >> 8;

    u16 pid = get_vid_pid_ver_from_cfg_file(GET_PID_FROM_EX_CFG);
    buf[offset++] = pid & 0xFF;
    buf[offset++] = pid >> 8;
#else
    buf[offset++] = 0x02;	// VID
    buf[offset++] = 0x00;

    buf[offset++] = 0x0F;	// PID
    buf[offset++] = 0x00;
#endif

    buf[offset++] = 0x50 | VER_FLAG_BLE_CTRL_BREDR;	// 手表类型

    rcsp_adv_fill_mac_addr(buf + offset);
    offset += 6;

    if (RCSP_SPP == get_defalut_bt_channel_sel()) {
        log_info("sel rcsp_spp\n");
        buf[offset++] = BIT(7);
    }

#endif

    if (offset > ADV_RSP_PACKET_MAX) {
        puts("***adv_data overflow!!!!!!\n");
        return -1;
    }
    offset = 31;//fixed
    UTE_MODULE_LOG(UTE_LOG_SYSTEM_LVL,"adv_data(%d):", offset);
    put_buf(buf, offset);
    adv_data_len = offset;
    /* ble_op_set_adv_data(offset, buf); */
    app_ble_adv_data_set(le_smartbox_ble_hdl, buf, offset);
    return 0;
}




#if TCFG_PAY_ALIOS_ENABLE
int upay_ble_adv_data_set(void)
{
    u8 offset = 0;
    u8 *buf = adv_data;
    u8 *edr_name = bt_get_local_name();
    u8 service_data[8];
    u8 tmp_data[6];

    offset += make_eir_packet_val(&buf[offset], offset, HCI_EIR_DATATYPE_FLAGS, 0x06, 1);
    service_data[0] = 0x02;
    service_data[1] = 0x38;
    le_controller_get_mac(tmp_data);
    swapX(tmp_data, &service_data[2], 6);
    offset += make_eir_packet_data(&buf[offset], offset, HCI_EIR_DATATYPE_SERVICE_DATA, service_data, 8);
    adv_data_len = offset;
    app_ble_adv_data_set(le_smartbox_ble_hdl, buf, offset);

    log_info("upay ADV data(%d):", adv_data_len);
    log_info_hexdump(buf, offset);

    buf = scan_rsp_data;
    offset = 0;
    offset += make_eir_packet_data(&buf[offset], offset, HCI_EIR_DATATYPE_COMPLETE_LOCAL_NAME, (void *)edr_name, strlen(edr_name));

    scan_rsp_data_len = offset;
    app_ble_rsp_data_set(le_smartbox_ble_hdl, buf, offset);

    UTE_MODULE_LOG(UTE_LOG_SYSTEM_LVL,"upay RSP data(%d):", scan_rsp_data_len);
    log_info_hexdump(buf, offset);

    return 0;
}
#endif


#if TCFG_PAY_ALIPAY_ENABLE
int alipay_ble_adv_data_set(void)
{
    u8 offset = 0;
    u8 *buf = adv_data;
    u8 *edr_name = BT_BLE_DEV_NAME;// bt_get_local_name();
    u8 service_data[8];
    u8 tmp_data[6];

    offset += make_eir_packet_val(&buf[offset], offset, HCI_EIR_DATATYPE_FLAGS, 0x06, 1);//都可以方                

    u8 service_data1[6] = {0x36, 0x55, 0x22, 0x22,0x55,0x55};//pcy
    offset += make_eir_packet_data(&buf[offset], offset, HCI_EIR_DATATYPE_COMPLETE_16BIT_SERVICE_UUIDS, service_data1, 6);

    service_data[0] = 0x02;
    service_data[1] = 0x38;
    le_controller_get_mac(tmp_data);
    swapX(tmp_data, &service_data[2], 6);
    offset += make_eir_packet_data(&buf[offset], offset, HCI_EIR_DATATYPE_SERVICE_DATA, service_data, 8);

    adv_data_len = offset;
    ble_op_set_adv_data(offset, buf);

    UTE_MODULE_LOG(UTE_LOG_SYSTEM_LVL,"alipay ADV data(%d):", adv_data_len);
    log_info_hexdump(buf, offset);

    buf = scan_rsp_data;
    offset = 0;
    offset += make_eir_packet_data(&buf[offset], offset, HCI_EIR_DATATYPE_COMPLETE_LOCAL_NAME, (void *)edr_name, strlen(edr_name));


    u8 alipay_temp_buf[8] ={0x00,0x00};//u8 temp_buf[] = {0x6E,0x49,0x00,0x3E,0x00,0x5A,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

    u8 alipay_tmp_data[6]; //pcy
    le_controller_get_mac(alipay_tmp_data);
    //mac
	alipay_temp_buf[2] = alipay_tmp_data[5];	
	alipay_temp_buf[3] = alipay_tmp_data[4];
	alipay_temp_buf[4] = alipay_tmp_data[3];
	alipay_temp_buf[5] = alipay_tmp_data[2];
	alipay_temp_buf[6] = alipay_tmp_data[1];
	alipay_temp_buf[7] = alipay_tmp_data[0];
    offset += make_eir_packet_data(&buf[offset], offset, HCI_EIR_DATATYPE_MANUFACTURER_SPECIFIC_DATA, alipay_temp_buf, sizeof(alipay_temp_buf));
    
    scan_rsp_data_len = offset;
    ble_op_set_rsp_data(offset, buf);

    UTE_MODULE_LOG(UTE_LOG_SYSTEM_LVL,"alipay RSP data(%d):", scan_rsp_data_len);
    UTE_MODULE_LOG(UTE_LOG_SYSTEM_LVL,"adv");
    put_buf(buf, offset);

    return 0;
}
#endif

void bt_ble_init_do(void)
{

}

int bt_ble_adv_ioctl(u32 cmd, u32 priv, u8 mode)
{
    return 0;
}

void bt_ble_rcsp_adv_enable(void)
{

}

u8 *ble_get_scan_rsp_ptr(u16 *len)
{
    if (len) {
#if 0//RCSP_UPDATE_EN
        u8 *ptr = bt_get_local_name();
        for (u8 i = 0, *ptr_str = NULL; i < scan_rsp_data_len; i += strlen(scan_rsp_data + i) + 1) {
            ptr_str = strstr(scan_rsp_data + i, ptr);
            if (ptr_str) {
                ptr = ptr_str;
                break;
            }
        }
        scan_rsp_data[(u32)ptr - (u32)scan_rsp_data - 2] += 4;
        scan_rsp_data[scan_rsp_data_len - 1 + 4] = scan_rsp_data[scan_rsp_data_len - 1];
        memcpy(scan_rsp_data + scan_rsp_data_len - 1, "_ota", 4);
        *len = scan_rsp_data_len + 4;
#else
        *len = scan_rsp_data_len;
#endif
    }
    return scan_rsp_data;
}

u8 *ble_get_adv_data_ptr(u16 *len)
{
    if (len) {
        *len = adv_data_len;
    }
    return adv_data;
}

u16 rebuild_adv_rcsp_info(u8 *adv_rsp_buf, u16 buf_size, u8 type, u8 *addr)
{
    struct excfg_rsp_payload rsp_payload;
    u16 item_len = 0;
    u8 *item_data = NULL;
    u8 i, rsp_adv_len = 0;

    switch (type) {
    case CFG_ITEM_ADV_IND:
        UTE_MODULE_LOG(UTE_LOG_SYSTEM_LVL/* UTE_LOG_SYSTEM_LVL */,"[make new adv data]\n");
        rsp_payload.vid = 0x05D6;
        memcpy(rsp_payload.logo, "JLOTA", sizeof("JLOTA"));
        for (i = 0; i < sizeof(rsp_payload.logo) / 2; i++) {
            rsp_payload.logo[i] ^= rsp_payload.logo[sizeof(rsp_payload.logo) - i - 1];
            rsp_payload.logo[sizeof(rsp_payload.logo) - i - 1] ^= rsp_payload.logo[i];
            rsp_payload.logo[i] ^= rsp_payload.logo[sizeof(rsp_payload.logo) - i - 1];
        }
        rsp_payload.version = 1;

        memcpy(rsp_payload.addr, addr, 6);

        if (rsp_adv_len + sizeof(struct excfg_rsp_payload) + 2 + 6 > buf_size) {
            UTE_MODULE_LOG(UTE_LOG_SYSTEM_LVL/* UTE_LOG_SYSTEM_LVL */,"adv data overflow!!!\n");
        } else {
            // *(adv_rsp_buf + rsp_adv_len) = sizeof(struct excfg_rsp_payload) + 1 + 6;        //fill jlpayload
            // *(adv_rsp_buf + rsp_adv_len + 1) = 0xff;                                    // HCI_EIR_DATATYPE_MANUFACTURER_SPECIFIC_DATA
            // memcpy(adv_rsp_buf + rsp_adv_len + 2, &rsp_payload, sizeof(struct excfg_rsp_payload));
            // rsp_adv_len += (2 + sizeof(struct excfg_rsp_payload));
            // addr[0] += 1;                                                        //修改地址，让手机重新发现服务, 这里地址的修改规则可以用户自行设置
            // // vid (2byte)
            // u16 vid = get_vid_pid_ver_from_cfg_file(GET_VID_FROM_EX_CFG);
            // adv_rsp_buf[rsp_adv_len++] = vid & 0xFF;
            // adv_rsp_buf[rsp_adv_len++] = vid >> 8;
            // // pid (2byte)
            // u16 pid = get_vid_pid_ver_from_cfg_file(GET_PID_FROM_EX_CFG);
            // adv_rsp_buf[rsp_adv_len++] = pid & 0xFF;
            // adv_rsp_buf[rsp_adv_len++] = pid >> 8;
            // // type + version (1byte)
            // adv_rsp_buf[rsp_adv_len++] = 0x50 | VER_FLAG_BLE_CTRL_BREDR;	// 手表类型
            // // vbat (1byte)
            // adv_rsp_buf[rsp_adv_len++] = 0;
            // UTE_MODULE_LOG(UTE_LOG_SYSTEM_LVL/* UTE_LOG_SYSTEM_LVL */,"new adv_data:\n");
            // put_buf(adv_rsp_buf, rsp_adv_len);


            rsp_adv_len += make_eir_packet_val(&adv_rsp_buf[rsp_adv_len], rsp_adv_len, HCI_EIR_DATATYPE_FLAGS, 0x02, 1);

            u8 service_data[8] = {0x36, 0x55, 0x22, 0x22,0x55,0x55,0x34,0x12};
            rsp_adv_len += make_eir_packet_data(&adv_rsp_buf[rsp_adv_len], rsp_adv_len, HCI_EIR_DATATYPE_COMPLETE_16BIT_SERVICE_UUIDS, service_data, sizeof(service_data));
            rsp_adv_len += make_eir_packet_val(&adv_rsp_buf[rsp_adv_len], rsp_adv_len, HCI_EIR_DATATYPE_FLAGS, 0x06, 1);
            char *edr_name = BT_BLE_DEV_NAME;//bt_get_local_name();
            rsp_adv_len += make_eir_packet_data(&adv_rsp_buf[rsp_adv_len], rsp_adv_len, HCI_EIR_DATATYPE_COMPLETE_LOCAL_NAME, (void *)edr_name, strlen(edr_name) > 31 -10 ? 31 -10 : strlen(edr_name));
            UTE_MODULE_LOG(UTE_LOG_SYSTEM_LVL,"%s, strlen(edr_name):%d", edr_name, (int)strlen(edr_name));
            
            UTE_MODULE_LOG(UTE_LOG_SYSTEM_LVL/* UTE_LOG_SYSTEM_LVL */,"new adv_data:\n");
            put_buf(adv_rsp_buf, rsp_adv_len);
        }
        break;
    case CFG_ITEM_SCAN_RSP:
	
        UTE_MODULE_LOG(UTE_LOG_SYSTEM_LVL/* UTE_LOG_SYSTEM_LVL */,"[make new rsp data]\n");
        u8 temp_buf[8] ={0x00,0x00};//u8 temp_buf[] = {0x6E,0x49,0x00,0x3E,0x00,0x5A,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

        u8 tmp_data[6];
        // u8 *temp1 = UTE_SW_VERSION;//bt_get_local_name();
        // u8 str_len = strlen(temp1);
        le_controller_get_mac(tmp_data);

        //mac
        temp_buf[2] = tmp_data[5];	
        temp_buf[3] = tmp_data[4];
        temp_buf[4] = tmp_data[3];
        temp_buf[5] = tmp_data[2];
        temp_buf[6] = tmp_data[1];
        temp_buf[7] = tmp_data[0];
        puts("ble_mac_addr:\n");
        put_buf(temp_buf, sizeof(temp_buf));    
        // rsp_adv_len += make_eir_packet_data(&adv_rsp_buf[rsp_adv_len], rsp_adv_len, HCI_EIR_DATATYPE_SECURITY_MANAGER_TK_VALUE, (void *)temp1, str_len);
        
        rsp_adv_len += make_eir_packet_data(&adv_rsp_buf[rsp_adv_len], rsp_adv_len, HCI_EIR_DATATYPE_MANUFACTURER_SPECIFIC_DATA, temp_buf, sizeof(temp_buf));
#if !UTE_CLEAR_ALL_DATA_AFTER_THE_UPGRADE_IS_COMPLETE
        TemporaryUpgradeEraseDataMethod();
#endif
        UTE_MODULE_LOG(UTE_LOG_SYSTEM_LVL/* UTE_LOG_SYSTEM_LVL */,"new rsp_data:\n");
        put_buf(adv_rsp_buf, rsp_adv_len);
        // log_info("[make new rsp data]\n");
        // rsp_adv_len += make_eir_packet_val(&adv_rsp_buf[rsp_adv_len], rsp_adv_len, HCI_EIR_DATATYPE_FLAGS, 0x06, 1);
        // char *edr_name = bt_get_local_name();
        // rsp_adv_len += make_eir_packet_data(&adv_rsp_buf[rsp_adv_len], rsp_adv_len, HCI_EIR_DATATYPE_COMPLETE_LOCAL_NAME, (void *)edr_name, strlen(edr_name) > 26 ? 26 : strlen(edr_name));
        // printf("%s, strlen(edr_name):%d", edr_name, (int)strlen(edr_name));
        // log_info("new rsp_data(%d):\n", rsp_adv_len);
        // log_info_hexdump(adv_rsp_buf, rsp_adv_len);
        break;
    }
    return rsp_adv_len;
}

void set_adv_notify_state(u8 notify)
{
    //
}

u8 get_adv_notify_state(void)
{
    //
    return 0;
}

void set_connect_flag(u8 value)
{
    //
}

#endif // RCSP_SMARTBOX_ADV_EN

#endif // (TCFG_BLE_DEMO_SELECT == DEF_BLE_DEMO_RCSP_DEMO)
