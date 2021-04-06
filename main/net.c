#include "net.h"
#include "ble_spp_server_demo.h"


static const char *TAG = "WIFI_CLOCK";

char myssid[36] = {0};
char mypass[36] = {0};
char ssidReady = 0;
char ntpSuccess = 0;

uint16_t ap_number = 20;
uint16_t ap_count = 0;
wifi_ap_record_t ap_info[20];

//WIFI连接相关参数
//char defaultSSID[] ="LLJ_ROOM";
//char defaultPASS[] = "87654321a";
#define EXAMPLE_ESP_MAXIMUM_RETRY  3
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

wifi_config_t wifi_config = {
        .sta = {
            .ssid = {},
            .password = {},
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };

static const char *ntpServerName = "ntp4.aliyun.com";

NETWORKSTATUS netWorkStatus = WAIT_SET;
int my_hour;
int my_minute;
int my_second;
unsigned char wifi_sta_start = 0;

unsigned char udp_buffer[NTP_PACKET_SIZE];

static void display_scan_result(void)
{
    memset(ap_info, 0, sizeof(ap_info));
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_number, ap_info));
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
    ESP_LOGI(TAG, "Total APs scanned = %u", ap_count);
    for (int i = 0; i < ap_count; i++) {
        ESP_LOGI(TAG, "SSID \t\t%s", ap_info[i].ssid);
        //ESP_LOGI(TAG, "RSSI \t\t%d", ap_info[i].rssi);
    }
}

//WIFI连接处理回调
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    static int s_retry_num = 0;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        wifi_sta_start = 1;
        //esp_wifi_connect();
        ESP_LOGI(TAG,"wifi_sta_start = 1");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        netWorkStatus = WIFI_LINK;
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:%s",ip4addr_ntoa(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_SCAN_DONE) {
        ESP_LOGI(TAG,"wifi scan event got...");
        display_scan_result();
    }
}

void wifi_init_sta()
{
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );

    //ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );

    //ESP_ERROR_CHECK(esp_wifi_start());
}

//连接WIFI
int wifi_sta_startScan()
{
    //重启wifi部分
    ESP_ERROR_CHECK(esp_wifi_stop());
    ESP_ERROR_CHECK(esp_wifi_start());
    //开始扫描wifi操作
    ESP_LOGE(TAG, "deal wifi scan task....");
    esp_wifi_scan_start(NULL, true);
    return 0;
}

//连接WIFI
int wifi_sta_connect()
{
    int wifiStatus = 0;
    uint16_t n = 0;
    s_wifi_event_group = xEventGroupCreate();

    //重启wifi部分
    ESP_ERROR_CHECK(esp_wifi_stop());
    ESP_ERROR_CHECK(esp_wifi_start());
    //设置wifi连接参数
    n = 0;
    while(myssid[n] != 0){
        wifi_config.sta.ssid[n] = myssid[n];
        n++;
    }
    wifi_config.sta.ssid[n] = 0;   
    n = 0;
    while(mypass[n] != 0){
        wifi_config.sta.password[n] = mypass[n];
        n++;
    }
    wifi_config.sta.password[n] = 0;
    ESP_LOGI(TAG, "paraset and restart");
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );

    esp_wifi_connect();
    ESP_LOGI(TAG, "wait connectted");
    
    //等待连接中
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,pdFALSE,pdFALSE,5000 / portTICK_PERIOD_MS);
    //判断连接结果
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s--",myssid, mypass);
        wifiStatus = 1;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",myssid, mypass);
    }
    
    vEventGroupDelete(s_wifi_event_group);

    return wifiStatus;
}


//通过域名获取IP
int url2ip(char* buf)
{
    int ip_status = 1;
    // 1、定义一个hints结构体，用来设置函数的getaddrinfo()的使用方式
    const struct addrinfo hints = {
        .ai_family = AF_INET,           /* 指定返回地址的协议簇，AF_INET(IPv4)、AF_INET6(IPv6)、AF_UNSPEC(IPv4 and IPv6)*/
        .ai_socktype = SOCK_STREAM,     /* 设定返回地址的socket类型，流式套接字 */     
    };
    struct addrinfo *result;
    int err;
    // 3、将获取到的信息打印出来
    struct sockaddr_in  *ipv4 = NULL;   /* IPv4地址结构体指针 */
    err = getaddrinfo(ntpServerName, "123", &hints, &result);
    if(err != 0)        /* 返回值不为0，函数执行失败*/
        ESP_LOGI(TAG, "getaddrinfo err: %d \n", err);
    else
    {
        if(result->ai_family == AF_INET) 
        {
            ipv4 = (struct sockaddr_in *)result->ai_addr;
            inet_ntop(result->ai_family, &ipv4->sin_addr, buf, 32);
            ESP_LOGI(TAG, "IPv4= %s port=%d \n",buf,ntohs(ipv4->sin_port));
            ip_status = 0;
        }
        else
        {
            ESP_LOGI(TAG, "got IPv4 err !!!\n");
            ip_status = 1;
        }
    }
    freeaddrinfo(result);
    return ip_status;
}

int get_time(void)
{
    static char ip_str[32];
    static int addr_family;
    static int ip_protocol;
    static int64_t secsSince1900;
    static int64_t seventyYears  = 2208988800LL;
    static int64_t epoch;

    struct sockaddr_in dest_addr;

    if(url2ip(ip_str) == 0)
    {
        dest_addr.sin_addr.s_addr = inet_addr(ip_str);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(123);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        }
        // 设置超时
        struct timeval timeout;
        timeout.tv_sec = 5;//秒
        timeout.tv_usec = 0;//微秒
        if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == -1) {
            ESP_LOGE(TAG, "setsockopt failed");
        }
        //构建UDP数据包
        memset(udp_buffer, 0, NTP_PACKET_SIZE);
        // Initialize values needed to form NTP request
        // (see URL above for details on the packets)
        udp_buffer[0] = 0b11100011;   // LI, Version, Mode
        udp_buffer[1] = 0;     // Stratum, or type of clock
        udp_buffer[2] = 6;     // Polling Interval
        udp_buffer[3] = 0xEC;  // Peer Clock Precision
        // 8 bytes of zero for Root Delay & Root Dispersion
        udp_buffer[12]  = 49;
        udp_buffer[13]  = 0x4E;
        udp_buffer[14]  = 49;
        udp_buffer[15]  = 52;
        //发送数据
        int err = sendto(sock, udp_buffer, NTP_PACKET_SIZE, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
            return 1;
        }
        //接收数据
        struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
        socklen_t socklen = sizeof(source_addr);
        int len = recvfrom(sock, udp_buffer, NTP_PACKET_SIZE, 0, (struct sockaddr *)&source_addr, &socklen);

        // Error occurred during receiving
        if (len < 0) {
            ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
            return 1;
        }
        // Data received
        else {
            secsSince1900 = (unsigned int)udp_buffer[40]*65536*256 + (unsigned int)udp_buffer[41]*65536 
                                        + (unsigned int)udp_buffer[42]*256 + (unsigned int)udp_buffer[43];
            ESP_LOGI(TAG, "Recrive my_second=%lld ", secsSince1900);
            // subtract seventy years:
            epoch = secsSince1900 - seventyYears;
            ESP_LOGI(TAG, "Recrive epoch=%lld ", epoch);
            epoch = epoch  % 86400LL;
            ESP_LOGI(TAG, "Recrive epoch=%lld ", epoch);
            //int my_hour = epoch / 3600 + 8;
            //if(my_hour >= 24) my_hour = my_hour - 24;
            //ESP_LOGI(TAG, "Recrive my_hour=%d ", my_hour);
            //int my_minute = my_minute = (epoch % 3600) / 60;
            //ESP_LOGI(TAG, "Recrive my_minute=%d ", my_minute);
            my_hour = epoch / 3600 + 8;
            if(my_hour >= 24) my_hour = my_hour - 24;
            my_minute = (epoch % 3600) / 60;
            my_second = (epoch % 60) * 2;
            ESP_LOGI(TAG, "Received times=%d:%d:%d:", my_hour,my_minute,my_second);
        }
        //完成后关闭socket
        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    else
    {
        return 1;
    }
    return 0;
}


void network_task(void *pvParameters)
{
    int count = 0;
    wifi_init_sta();
    while (1) {
        //WIFI名和密码还未配置，等待配置状态
        if(netWorkStatus == WAIT_SET)
        {
            ntpSuccess = 0;
            vTaskDelay(500 / portTICK_PERIOD_MS);
            if(ssidReady == 1)
            {
                paraSSIDSave();
                vTaskDelay(500 / portTICK_PERIOD_MS);
                netWorkStatus = WIFI_LINK;
            }
        }
        //开始一次wifi扫描操作
        if(netWorkStatus == WIFI_SCAN)
        {
            //ssidReady = 0;
            ntpSuccess = 0;
            wifi_sta_startScan();
            netWorkStatus = WAIT_SET;
        }
        if(netWorkStatus == WIFI_LINK)
        {
            ntpSuccess = 0;
            ESP_LOGE(TAG, "deal wifi link task....");
            if(wifi_sta_connect() == 1)
                netWorkStatus = TIME_GET;
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }
        if(netWorkStatus == TIME_GET)
        {
            if(get_time() == 0)
            {
                ntpSuccess = 1;
                vTaskDelay(5000 / portTICK_PERIOD_MS);
                netWorkStatus = TIME_OK;
            }
            else
            {
                netWorkStatus = WIFI_LINK;
            } 
        }
        if(netWorkStatus == TIME_OK)
        {
            count++;
            if(count > 1000)
            {
                count = 0;
                if(get_time() != 0)
                {
                    ntpSuccess = 0;
                    netWorkStatus = WIFI_LINK;
                }     
            }
            vTaskDelay(5000 / portTICK_PERIOD_MS);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

