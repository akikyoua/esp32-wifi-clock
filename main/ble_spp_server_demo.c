/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "driver/uart.h"
#include "string.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "ble_spp_server_demo.h"

#include "net.h"
#include "display.h"

#define GATTS_TABLE_TAG  "GATTS_SPP_DEMO"

#define SPP_PROFILE_NUM             1
#define SPP_PROFILE_APP_IDX         0
#define ESP_SPP_APP_ID              0x56
//#define SAMPLE_DEVICE_NAME          "WIFI_CLOCK_SERVER"
#define SPP_SVC_INST_ID	            0

uint8_t bleDevName[33] = "wifi-clock";

nvs_handle_t my_handle;

/// SPP Service
static const uint16_t spp_service_uuid = 0xABF0;
/// Characteristic UUID
#define ESP_GATT_UUID_SPP_DATA_RECEIVE      0xABF1
#define ESP_GATT_UUID_SPP_DATA_NOTIFY       0xABF2

static uint8_t spp_adv_data[42] = {
    0x02,0x01,0x06,
    0x03,0x03,0xF0,0xAB,
    0x0B,0x09,'w','i','f','i','-','C','l','o','c','k'
};

static uint16_t spp_mtu_size = 23;
static uint16_t spp_conn_id = 0xffff;
static esp_gatt_if_t spp_gatts_if = 0xff;
QueueHandle_t spp_uart_queue = NULL;
static xQueueHandle cmd_cmd_queue = NULL;


static bool enable_data_ntf = false;
static bool is_connected = false;
static esp_bd_addr_t spp_remote_bda = {0x0,};

static uint16_t spp_handle_table[SPP_IDX_NB];

static esp_ble_adv_params_t spp_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

typedef struct spp_receive_data_node{
    int32_t len;
    uint8_t * node_buff;
    struct spp_receive_data_node * next_node;
}spp_receive_data_node_t;

static spp_receive_data_node_t * temp_spp_recv_data_node_p1 = NULL;
static spp_receive_data_node_t * temp_spp_recv_data_node_p2 = NULL;

typedef struct spp_receive_data_buff{
    int32_t node_num;
    int32_t buff_size;
    spp_receive_data_node_t * first_node;
}spp_receive_data_buff_t;

static spp_receive_data_buff_t SppRecvDataBuff = {
    .node_num   = 0,
    .buff_size  = 0,
    .first_node = NULL
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst spp_profile_tab[SPP_PROFILE_NUM] = {
    [SPP_PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/*
 *  SPP PROFILE ATTRIBUTES
 ****************************************************************************************
 */

#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE_NR|ESP_GATT_CHAR_PROP_BIT_READ;

#ifdef SUPPORT_HEARTBEAT
static const uint8_t char_prop_read_write_notify = ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_WRITE_NR|ESP_GATT_CHAR_PROP_BIT_NOTIFY;
#endif

///SPP Service - data receive characteristic, read&write without response
static const uint16_t spp_data_receive_uuid = ESP_GATT_UUID_SPP_DATA_RECEIVE;
static const uint8_t  spp_data_receive_val[20] = {0x00};

///SPP Service - data notify characteristic, notify&read
static const uint16_t spp_data_notify_uuid = ESP_GATT_UUID_SPP_DATA_NOTIFY;
static const uint8_t  spp_data_notify_val[20] = {0x00};
static const uint8_t  spp_data_notify_ccc[2] = {0x00, 0x00};


///Full HRS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t spp_gatt_db[SPP_IDX_NB] =
{
    //SPP -  Service Declaration
    [SPP_IDX_SVC]                      	=
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
    sizeof(spp_service_uuid), sizeof(spp_service_uuid), (uint8_t *)&spp_service_uuid}},

    //SPP -  data receive characteristic Declaration
    [SPP_IDX_SPP_DATA_RECV_CHAR]            =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    //SPP -  data receive characteristic Value
    [SPP_IDX_SPP_DATA_RECV_VAL]             	=
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_receive_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    SPP_DATA_MAX_LEN,sizeof(spp_data_receive_val), (uint8_t *)spp_data_receive_val}},

    //SPP -  data notify characteristic Declaration
    [SPP_IDX_SPP_DATA_NOTIFY_CHAR]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    //SPP -  data notify characteristic Value
    [SPP_IDX_SPP_DATA_NTY_VAL]   =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_notify_uuid, ESP_GATT_PERM_READ,
    SPP_DATA_MAX_LEN, sizeof(spp_data_notify_val), (uint8_t *)spp_data_notify_val}},

    //SPP -  data notify characteristic - Client Characteristic Configuration Descriptor
    [SPP_IDX_SPP_DATA_NTF_CFG]         =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(uint16_t),sizeof(spp_data_notify_ccc), (uint8_t *)spp_data_notify_ccc}},

};

void receDeal(void);
void paraSSIDSave(void);
void bleNameSave(void);


/*****************通信协议部分，包含拼包处理，拆包发送****************/
uint8_t packBuffer[256];     //数据包最大256字节
uint8_t packLength = 0;
uint8_t packlist[16];
uint8_t sendPackID = 0;
uint8_t recePackID = 0;
//数据包格式0-总数，1-当前个数，2-数据长度， 3-包id，包id每次自增，区分个个包

void packetBuffer(uint8_t* buffer)
{
    static uint8_t err = 0,i;
    if(buffer[1] == 0)
    {
        packLength = 0;                 //第一包清空长度
        recePackID = buffer[3];         //第一包设置包id
        memset((void*)packlist,0,16);          //清空标记
    }
    if(buffer[3] == recePackID)
    {
        if(buffer[0] == buffer[1])
        {
            memcpy((void*)&packBuffer[packLength],(void*)&buffer[4],buffer[2]);
            packLength += buffer[2];
            packlist[buffer[1]] = 1;
            err = 0;
            for(i=0; i<=buffer[0]; i++)
            {
                if(packlist[i] == 0)
                {
                    ESP_LOGI(GATTS_TABLE_TAG, "pack error");
                    err = 1;
                    break;
                }
            }
            if(err != 1)
            {
                ESP_LOGI(GATTS_TABLE_TAG, "pack correct");
                receDeal();
            }
        }
        else
        {
            memcpy((void*)&packBuffer[packLength],(void*)&buffer[4],buffer[2]);
            packLength += buffer[2];
            packlist[buffer[1]] = 1;
        }
        
    }
    
}


void sendbleBuffer(uint8_t* buffer, uint16_t length)
{
    uint8_t arr[20];
    uint16_t i,n;
    uint16_t packlastlen = length % 16;
    uint16_t packcnt = length / 16;
    uint16_t totalpack = 0;
    uint16_t currentpack = 0;
    if(packlastlen != 0)
      totalpack = packcnt + 1;
    else
      totalpack = packcnt;
    for(i = 0; i< packcnt;i++)
    {
      arr[0] = totalpack-1;
      arr[1] = i;
      arr[2] = 16;
      arr[3] = 0;
      for(n=0;n<16;n++)
      {
        arr[4+n] = buffer[i*16+n];
      }
      esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL],20, arr, false);
    }
    if(packlastlen != 0)
    {
      arr[0] = totalpack-1;
      arr[1] = totalpack-1;
      arr[2] = packlastlen;
      arr[3] = 0;
      for(n=0;n<packlastlen;n++)
      {
        arr[4+n] = buffer[packcnt*16+n];
      }
      esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL],20, arr, false);
    }
}



void receDeal(void)
{
    uint16_t index;
    uint8_t temp[8];
    if(packBuffer[0] == 1)         //命令2是发送过来的wifi名称
    {
        index = 1;
        while((index < packLength) && ((packBuffer[index] != 0)))
        {
            myssid[index-1] = packBuffer[index];
            index++;
        }
        myssid[index-1] = 0;
        ESP_LOGI(GATTS_TABLE_TAG, "RECSSID= %s\n", myssid);
    }
    if(packBuffer[0] == 2)         //命令2是发送过来的wifi密码，最多两个包构成，一个包18个字节
    {
        index = 1;
        while((index < packLength) && ((packBuffer[index] != 0)))
        {
            mypass[index-1] = packBuffer[index];
            index++;
        }
        mypass[index-1] = 0;
        paraSSIDSave();
        ssidReady = 1;
        ESP_LOGI(GATTS_TABLE_TAG, "RECPASS= %s\n", mypass);
        netWorkStatus = WIFI_LINK;//发送更新时间命令
    }
    if(packBuffer[0] == 3)         //命令3设置时钟显示状态等参数
    {
        secondMode =  packBuffer[1];  
        daylightness =  packBuffer[2]; 
        nightMode = packBuffer[3]; 
        nightlightness = packBuffer[4];
        nightStart = packBuffer[5];
        nightEnd = packBuffer[6];                  
    }
    if(packBuffer[0] == 4)     //命令4开始扫描wifi
    {
        temp[0] = 4;
        ESP_LOGI(GATTS_TABLE_TAG, "rec scan cmd");
        sendbleBuffer(temp, 1);
        ESP_ERROR_CHECK(esp_wifi_stop());
        netWorkStatus = WIFI_SCAN;
    }
    if(packBuffer[0] == 6)         //命令2是发送过来的wifi密码，最多两个包构成，一个包18个字节
    {

        index = 1;
        while((index < packLength) && ((packBuffer[index] != 0)))
        {
            bleDevName[index-1] = packBuffer[index];
            index++;
        }
        bleDevName[index-1] = 0;
        ESP_LOGI(GATTS_TABLE_TAG, "recname= %s\n", bleDevName);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        bleNameSave();
        temp[0] = 6;
        sendbleBuffer(temp, 1);
        
    }
}

static uint8_t find_char_and_desr_index(uint16_t handle)
{
    uint8_t error = 0xff;
    for(int i = 0; i < SPP_IDX_NB ; i++){
        if(handle == spp_handle_table[i]){
            return i;
        }
    }
    return error;
}

static bool store_wr_buffer(esp_ble_gatts_cb_param_t *p_data)
{
    temp_spp_recv_data_node_p1 = (spp_receive_data_node_t *)malloc(sizeof(spp_receive_data_node_t));
    if(temp_spp_recv_data_node_p1 == NULL){
        ESP_LOGI(GATTS_TABLE_TAG, "malloc error %s %d\n", __func__, __LINE__);
        return false;
    }
    if(temp_spp_recv_data_node_p2 != NULL){
        temp_spp_recv_data_node_p2->next_node = temp_spp_recv_data_node_p1;
    }
    temp_spp_recv_data_node_p1->len = p_data->write.len;
    SppRecvDataBuff.buff_size += p_data->write.len;
    temp_spp_recv_data_node_p1->next_node = NULL;
    temp_spp_recv_data_node_p1->node_buff = (uint8_t *)malloc(p_data->write.len);
    temp_spp_recv_data_node_p2 = temp_spp_recv_data_node_p1;
    memcpy(temp_spp_recv_data_node_p1->node_buff,p_data->write.value,p_data->write.len);
    if(SppRecvDataBuff.node_num == 0){
        SppRecvDataBuff.first_node = temp_spp_recv_data_node_p1;
        SppRecvDataBuff.node_num++;
    }else{
        SppRecvDataBuff.node_num++;
    }
    return true;
}

static void free_write_buffer(void)
{
    temp_spp_recv_data_node_p1 = SppRecvDataBuff.first_node;

    while(temp_spp_recv_data_node_p1 != NULL){
        temp_spp_recv_data_node_p2 = temp_spp_recv_data_node_p1->next_node;
        free(temp_spp_recv_data_node_p1->node_buff);
        free(temp_spp_recv_data_node_p1);
        temp_spp_recv_data_node_p1 = temp_spp_recv_data_node_p2;
    }
    SppRecvDataBuff.node_num = 0;
    SppRecvDataBuff.buff_size = 0;
    SppRecvDataBuff.first_node = NULL;
}

static void print_write_buffer(void)
{
    temp_spp_recv_data_node_p1 = SppRecvDataBuff.first_node;

    while(temp_spp_recv_data_node_p1 != NULL){
        uart_write_bytes(UART_NUM_0, (char *)(temp_spp_recv_data_node_p1->node_buff), temp_spp_recv_data_node_p1->len);
        temp_spp_recv_data_node_p1 = temp_spp_recv_data_node_p1->next_node;
    }
}

void uart_task(void *pvParameters)
{
    uart_event_t event;
    for (;;) {
        //Waiting for UART event.
        if (xQueueReceive(spp_uart_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            switch (event.type) {
            //Event of UART receving data
            case UART_DATA:
                if (event.size) {
                    uint8_t * temp = NULL;
                    temp = (uint8_t *)malloc(sizeof(uint8_t)*event.size);
                    if(temp == NULL){
                        ESP_LOGE(GATTS_TABLE_TAG, "%s malloc.1 failed\n", __func__);
                        break;
                    }
                    memset(temp,0x0,event.size);
                    uart_read_bytes(UART_NUM_0,temp,event.size,portMAX_DELAY);
                    if(event.size <= (spp_mtu_size - 3)){
                        esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL],event.size, temp, false);
                    }
                    free(temp);
                }
                break;
            default:
                break;
            }
        }
    }
    vTaskDelete(NULL);
}

static void spp_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_RTS,
        .rx_flow_ctrl_thresh = 122,
    };

    //Set UART parameters
    uart_param_config(UART_NUM_0, &uart_config);
    //Set UART pins
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    //Install UART driver, and get the queue.
    uart_driver_install(UART_NUM_0, 4096, 8192, 10,&spp_uart_queue,0);
    xTaskCreate(uart_task, "uTask", 2048, (void*)UART_NUM_0, 8, NULL);
}

void bleSendSSID(char *ssid)
{
    unsigned char buffer[128];
    uint16_t len = 0;
    buffer[0] = 1;
    while(ssid[len] != 0)
    {
        buffer[len+1] = ssid[len];
        len++; 
    }
    sendbleBuffer(buffer, len+1);
}

void bleSendSearchSSID(char *ssid)
{
    unsigned char buffer[128];
    uint16_t len = 0;
    buffer[0] = 5;
    while(ssid[len] != 0)
    {
        buffer[len+1] = ssid[len];
        len++; 
    }
    sendbleBuffer(buffer, len+1);
}

void bleSendStat(void)
{
    unsigned char buffer[20];
    buffer[0] = 3;           //命令3设置时钟显示状态等参数
    buffer[1] = secondMode;  
    buffer[2] = daylightness; 
    buffer[3] = nightMode; 
    buffer[4] = nightlightness;
    buffer[5] = nightStart;
    buffer[6] = nightEnd;  
    sendbleBuffer(buffer, 7);
}


void spp_cmd_task(void * arg)
{
    uint16_t statusCnt = 20;
    uint16_t n;
    for(;;){
        if(bleConn == 1)
        {
            statusCnt++;
            if(statusCnt >= 20)
            {
                statusCnt = 0;
                bleSendStat();
                vTaskDelay(50 / portTICK_PERIOD_MS);
                bleSendSSID(myssid);
            }
            if(ap_count > 0)
            {
                vTaskDelay(500 / portTICK_PERIOD_MS);  //延时500ms等待apinfo复制完整
                for(n=0;n<ap_count;n++)
                {
                    bleSendSearchSSID((char*)ap_info[n].ssid);
                    vTaskDelay(50 / portTICK_PERIOD_MS);
                }
                ap_count = 0;
            }
        }
        else
        {
            statusCnt = 20;
        }
        
        vTaskDelay(50 / portTICK_PERIOD_MS);
        //esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL],event.size, temp, false);
    }
    vTaskDelete(NULL);
}

static void boot_task_init(void)
{
    spp_uart_init();

    displayInit();

    xTaskCreate(displayTask, "display_task", 2048, NULL, 10, NULL);

    xTaskCreate(network_task, "udp_client", 4096, NULL, 10, NULL);

    cmd_cmd_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(spp_cmd_task, "spp_cmd_task", 2048, NULL, 10, NULL);

    
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;
    //ESP_LOGE(GATTS_TABLE_TAG, "GAP_EVT, event %d\n", event);

    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            esp_ble_gap_start_advertising(&spp_adv_params);
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            //advertising start complete event to indicate advertising start successfully or failed
            if((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "Advertising start failed: %s\n", esp_err_to_name(err));
            }
            break;
        default:
            break;
    }
}


static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t *) param;
    uint8_t res = 0xff;

    //ESP_LOGI(GATTS_TABLE_TAG, "event = %x\n",event);
    switch (event) {
    	case ESP_GATTS_REG_EVT:
    	    //ESP_LOGI(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
        	esp_ble_gap_set_device_name((const char*)bleDevName);
        	//ESP_LOGI(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
        	esp_ble_gap_config_adv_data_raw((uint8_t *)spp_adv_data, strlen((const char*)spp_adv_data));
        	//ESP_LOGI(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
        	esp_ble_gatts_create_attr_tab(spp_gatt_db, gatts_if, SPP_IDX_NB, SPP_SVC_INST_ID);
       	break;
    	case ESP_GATTS_READ_EVT:
            res = find_char_and_desr_index(p_data->read.handle);
            //if(res == SPP_IDX_SPP_STATUS_VAL){
                //TODO:client read the status characteristic
            //}
       	 break;
    	case ESP_GATTS_WRITE_EVT: {
    	    res = find_char_and_desr_index(p_data->write.handle);
            if(p_data->write.is_prep == false){
                ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_WRITE_EVT : handle = %d\n", res);
                if(res == SPP_IDX_SPP_DATA_NTF_CFG){
                    if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x01)&&(p_data->write.value[1] == 0x00)){
                        bleConn = 1;
                        enable_data_ntf = true;
                    }else if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x00)&&(p_data->write.value[1] == 0x00)){
                        enable_data_ntf = false;
                    }
                }
                else if(res == SPP_IDX_SPP_DATA_RECV_VAL){
                    uart_write_bytes(UART_NUM_0, (char *)(p_data->write.value), p_data->write.len);
                    packetBuffer(p_data->write.value);
                }else{
                    //TODO:
                }
            }else if((p_data->write.is_prep == true)&&(res == SPP_IDX_SPP_DATA_RECV_VAL)){
                ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_PREP_WRITE_EVT : handle = %d\n", res);
                store_wr_buffer(p_data);
            }
      	 	break;
    	}
    	case ESP_GATTS_EXEC_WRITE_EVT:{
    	    ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT\n");
    	    if(p_data->exec_write.exec_write_flag){
    	        print_write_buffer();
    	        free_write_buffer();
    	    }
    	    break;
    	}
    	case ESP_GATTS_MTU_EVT:
    	    spp_mtu_size = p_data->mtu.mtu;
    	    break;
    	case ESP_GATTS_CONF_EVT:
    	    break;
    	case ESP_GATTS_UNREG_EVT:
        	break;
    	case ESP_GATTS_DELETE_EVT:
        	break;
    	case ESP_GATTS_START_EVT:
        	break;
    	case ESP_GATTS_STOP_EVT:
        	break;
    	case ESP_GATTS_CONNECT_EVT:                         //蓝牙连接成功
    	    spp_conn_id = p_data->connect.conn_id;
    	    spp_gatts_if = gatts_if;
    	    is_connected = true;
    	    memcpy(&spp_remote_bda,&p_data->connect.remote_bda,sizeof(esp_bd_addr_t));
        	break;
    	case ESP_GATTS_DISCONNECT_EVT:                      ////蓝牙断开连接
            bleConn = 0;
    	    is_connected = false;
    	    enable_data_ntf = false;
            paraStatusSave();
    	    esp_ble_gap_start_advertising(&spp_adv_params);
    	    break;
    	case ESP_GATTS_OPEN_EVT:
    	    break;
    	case ESP_GATTS_CANCEL_OPEN_EVT:
    	    break;
    	case ESP_GATTS_CLOSE_EVT:
    	    break;
    	case ESP_GATTS_LISTEN_EVT:
    	    break;
    	case ESP_GATTS_CONGEST_EVT:
    	    break;
    	case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
    	    ESP_LOGI(GATTS_TABLE_TAG, "The number handle =%x\n",param->add_attr_tab.num_handle);
    	    if (param->add_attr_tab.status != ESP_GATT_OK){
    	        ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table failed, error code=0x%x", param->add_attr_tab.status);
    	    }
    	    else if (param->add_attr_tab.num_handle != SPP_IDX_NB){
    	        ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table abnormally, num_handle (%d) doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, SPP_IDX_NB);
    	    }
    	    else {
    	        memcpy(spp_handle_table, param->add_attr_tab.handles, sizeof(spp_handle_table));
    	        esp_ble_gatts_start_service(spp_handle_table[SPP_IDX_SVC]);
    	    }
    	    break;
    	}
    	default:
    	    break;
    }
}


static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    //ESP_LOGI(GATTS_TABLE_TAG, "EVT %d, gatts if %d\n", event, gatts_if);

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            spp_profile_tab[SPP_PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGI(GATTS_TABLE_TAG, "Reg app failed, app_id %04x, status %d\n",param->reg.app_id, param->reg.status);
            return;
        }
    }

    do {
        int idx;
        for (idx = 0; idx < SPP_PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == spp_profile_tab[idx].gatts_if) {
                if (spp_profile_tab[idx].gatts_cb) {
                    spp_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}


void paraStatusSave(void)
{
    nvs_set_u8(my_handle, "secondMode", secondMode);
    nvs_set_u8(my_handle, "daylightness", daylightness);
    nvs_set_u8(my_handle, "nightlightness", nightlightness);
    nvs_set_u8(my_handle, "nightMode", nightMode);
    nvs_set_u8(my_handle, "nightStart", nightStart);
    nvs_set_u8(my_handle, "nightEnd", nightEnd);
    nvs_commit(my_handle);
    ESP_LOGE(GATTS_TABLE_TAG, "para save success\n");
}

void paraSSIDSave(void)
{
    nvs_set_str(my_handle, "myssid", myssid);
    nvs_set_str(my_handle, "mypass", mypass);
    nvs_commit(my_handle);
    ESP_LOGE(GATTS_TABLE_TAG, "para save success\n");
}

void bleNameSave(void)
{
    nvs_set_str(my_handle, "myblename", (char*)bleDevName);
    nvs_commit(my_handle);
    ESP_LOGE(GATTS_TABLE_TAG, "name save success\n");
}

void bleadvInit(void)
{
    int n = 0;
    while(bleDevName[n] != '\0')
    {
        spp_adv_data[9+n] = bleDevName[n];
        n++;
    }
    spp_adv_data[9+n] = 0x00;
    spp_adv_data[7] = n+1;
}

void app_main()
{
    esp_err_t ret;
    size_t length = 0;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();


    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    
    ESP_ERROR_CHECK( ret );

    ret = nvs_open("clockpara", NVS_READWRITE, &my_handle);
    if (ret != ESP_OK) {        //参数读取失败，全部使用默认参数
        myssid[0] = 0;
        mypass[0] = 0;
        secondMode = 0;
        daylightness = 6;
        nightlightness = 1;
        nightMode = 0;
        nightStart = 138;
        nightEnd = 42;
        ESP_LOGE(GATTS_TABLE_TAG, "para read failed use default para\n");
    }else {
        length = 36;
        nvs_get_str(my_handle, "myssid", myssid, &length);
        length = 36;
        nvs_get_str(my_handle, "mypass", mypass, &length);
        length = 33;
        nvs_get_str(my_handle, "myblename", (char*)bleDevName, &length);
        nvs_get_u8(my_handle, "secondMode", &secondMode);
        nvs_get_u8(my_handle, "daylightness", &daylightness);
        nvs_get_u8(my_handle, "nightlightness", &nightlightness);
        nvs_get_u8(my_handle, "nightMode", &nightMode);
        nvs_get_u8(my_handle, "nightStart", &nightStart);
        nvs_get_u8(my_handle, "nightEnd", &nightEnd);
        if(myssid[0] != 0)
            ssidReady = 1;
        //ESP_LOGE(GATTS_TABLE_TAG, "para read success\n");
    }
    bleadvInit();

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        //ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        //ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    //ESP_LOGI(GATTS_TABLE_TAG, "%s init bluetooth\n", __func__);
    ret = esp_bluedroid_init();
    if (ret) {
        //ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        //ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(ESP_SPP_APP_ID);

    boot_task_init();

    return;
}
