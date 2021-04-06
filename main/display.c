#include "display.h"
#include "font.h"
#include "net.h"

unsigned char bleConn = 0;

unsigned char secondMode = 1;
unsigned char daylightness = 6;
unsigned char nightMode = 0;
unsigned char nightlightness = 1;
unsigned char nightStart = 138;
unsigned char nightEnd = 42;

unsigned char lightnessSet = 4 ;

unsigned char disram[8][32] = {0};//8行32列
unsigned char pix[32]={0};

void start1()
{
    SCK1H();
    DIN1H();
    DIN1L();
    SCK1L();
}

void start2()
{
    SCK2H();
    DIN2H();
    DIN2L();
    SCK2L();
}

void stop1()
{
    SCK1L();
    DIN1L();
    SCK1H();
    DIN1H();
}

void stop2()
{
    SCK2L();
    DIN2L();
    SCK2H();
    DIN2H();
}

void sendCommd1(unsigned char cmd)
{
    unsigned char  i;
    for(i = 0; i < 8; i++)
    {
        SCK1L();
        if(cmd & 0x01)
            DIN1H();
        else
            DIN1L();
        SCK1H();
        cmd >>= 1;
    }
    SCK1L();
}

void sendCommd2(unsigned char cmd)
{
    unsigned char  i;
    for(i = 0; i < 8; i++)
    {
        SCK2L();
        if(cmd & 0x01)
            DIN2H();
        else
            DIN2L();
        SCK2H();
        cmd >>= 1;
    }
    SCK2L();
}

void Init_Display()
{
    start1();
    sendCommd1(DISPLAY_DIS);         // 关显示
    stop1();
    start1();
    sendCommd1(WRITE_DATA_MODE_Z);   // 自动地址
    stop1();
    start1();
    sendCommd1(0x8F);                // 开显示
    stop1();

    start2();
    sendCommd2(DISPLAY_DIS);         // 关显示
    stop2();
    start2();
    sendCommd2(WRITE_DATA_MODE_Z);   // 自动地址
    stop2();
    start2();
    sendCommd2(0x8F);                // 开显示
    stop2();
}

void setlightness(unsigned char lightness)
{
    if(lightness == 0)
    {
        start1();
        sendCommd1(0x80);                // 关显示
        stop1();
        start2();
        sendCommd2(0x80);                // 关显示
        stop2();
    }
    else if(lightness <= 8)
    {
        lightness = 0x88 + lightness-1; 
        start1();
        sendCommd1(lightness);                // 开显示
        stop1();
        start2();
        sendCommd2(lightness);                // 开显示
        stop2();
    }

    
}

void Update_Display(void)
{
    unsigned char i,j;
    for(i=0;i<32;i++)                   //清空显示
        pix[i]=0;

    if(ntpSuccess == 0)
        disram[7][0] = 1;
    else
        disram[7][0] = 0;

    if(bleConn == 1)
        disram[0][31] = 1;
    else
        disram[0][31] = 0;
    
    //共阴数码管数据梳理
    if(DISM() == 0)
    {
        for(i=0;i<4;i++)
        {
            for(j=0; j<8; j++)
            {
                if(disram[j][8*i+0]) pix[8*i+j] |= 0x01;
                if(disram[j][8*i+1]) pix[8*i+j] |= 0x02;
                if(disram[j][8*i+2]) pix[8*i+j] |= 0x04;
                if(disram[j][8*i+3]) pix[8*i+j] |= 0x08;
                if(disram[j][8*i+4]) pix[8*i+j] |= 0x10;
                if(disram[j][8*i+5]) pix[8*i+j] |= 0x20;
                if(disram[j][8*i+6]) pix[8*i+j] |= 0x40;
                if(disram[j][8*i+7]) pix[8*i+j] |= 0x80;
            }
        }
    }
    else    //共阳数码管数据处理
    {
        for(i=0;i<32;i++)
        {
            if(disram[0][i]) pix[i] |= 0x01;
            if(disram[1][i]) pix[i] |= 0x02;
            if(disram[2][i]) pix[i] |= 0x04;
            if(disram[3][i]) pix[i] |= 0x08;
            if(disram[4][i]) pix[i] |= 0x10;
            if(disram[5][i]) pix[i] |= 0x20;
            if(disram[6][i]) pix[i] |= 0x40;
            if(disram[7][i]) pix[i] |= 0x80;
        }
    }


    start1();
    sendCommd1(START_DATA);              //起始地址
    for(i=0;i<16;i++)                   //送16位数
    {
        sendCommd1(pix[i]);
    }
    stop1();
    start2();
    sendCommd2(START_DATA);              //起始地址
    for(i=16;i<32;i++)                   //送16位数
    {
        sendCommd2(pix[i]);
    }
    stop2();
}

void displaybmp(unsigned char row,unsigned char col,unsigned char w,unsigned char h, const unsigned char *bmp)
{
    unsigned char i,j,n = 0;
    for(i=row; i<row+h; i++)
    {
        for(j=col; j<col+w; j++)
        {
            disram[i][j] = bmp[n];
            n++;
        }
    }
}

void displayClear(void)
{
    unsigned char i,j;
    for(i=0; i<8; i++)
    {
        for(j=0; j<32; j++)
        {
            disram[i][j] = 0;
        }
    }
}

void displayNum5x8(unsigned char row,unsigned char col,unsigned char num)
{
    displaybmp(row,col,5,8,&nums[num*8][0]);   
}

void displayNum4x8(unsigned char row,unsigned char col,unsigned char num)
{
    displaybmp(row,col,4,8,&num4x8[num*8][0]);   
}

void displayNum3x6(unsigned char row,unsigned char col,unsigned char num)
{
    displaybmp(row,col,3,6,&num3x6[num*6][0]);   
}

void showTime(void)
{
    displayNum5x8(0,2,my_hour/10);
    displayNum5x8(0,9,my_hour%10);
    displayNum5x8(0,18,my_minute/10);
    displayNum5x8(0,25,my_minute%10);
    if(my_second%2 == 0)
    {
        disram[2][15] = 1;
        disram[2][16] = 1;
        disram[5][15] = 1;
        disram[5][16] = 1;
    }
    else
    {
        disram[2][15] = 0;
        disram[2][16] = 0;
        disram[5][15] = 0;
        disram[5][16] = 0;
    }
    Update_Display();
}

void showTimeWithSec(void)
{
    displayNum4x8(0,1,my_hour/10);
    displayNum4x8(0,6,my_hour%10);
    displayNum4x8(0,13,my_minute/10);
    displayNum4x8(0,18,my_minute%10);
    displayNum3x6(2,25,my_second/2/10);
    displayNum3x6(2,29,my_second/2%10);
    disram[2][11] = 1;
    disram[5][11] = 1;
    disram[2][23] = 1;
    disram[5][23] = 1;
    Update_Display();
}

void displayTask(void* arg)
{
    int Toggle = 0;
    int tick = 0;
    int lastTick = 0;
    int oldSecond = 0;
    unsigned char lastSecondMode = 0;
    unsigned char onNightMode = 0;
    unsigned char timetip;
    tick = xTaskGetTickCount()/(500/portTICK_RATE_MS);
    lastTick = tick;
    //校时成功前的静态显示
    for(;;) {
        vTaskDelay(10 / portTICK_RATE_MS);
        if(ntpSuccess == 0)
        {
            tick = xTaskGetTickCount()/(500/portTICK_RATE_MS);
            if(lastTick != tick)
            {
                lastTick = tick;
                if(Toggle == 0)
                {
                    Toggle = 1;
                    memcpy((void*)disram,(void*)openlog1,256);
                }
                else
                {
                    Toggle = 0;
                    memcpy((void*)disram,(void*)openlog2,256);
                }
                Update_Display();
            }
        }
        else
        {
            break;           //校时成功退出循环
        }
    }
    for(;;) {
        vTaskDelay(10 / portTICK_RATE_MS);
        tick = xTaskGetTickCount()/(500/portTICK_RATE_MS);
        my_second = my_second + (tick - lastTick);
        lastTick = tick;
        if(oldSecond != my_second)
        {
            oldSecond = my_second; 
            //ESP_LOGI("TAG", "Received mysecond=%d:", my_second);
            if(my_second >= 120)
            {
                my_second = 0;
                my_minute++;
                if(my_minute >= 60)
                {
                    my_minute = 0;
                    my_hour++;
                    if(my_hour >= 24)
                        my_hour = 0;
                }
            } 
            if(lastSecondMode != secondMode)
            {
                lastSecondMode = secondMode;
                displayClear();
            }
            if(secondMode == 1)
            {
                showTimeWithSec();
            }
            else
            {
                showTime();
            }
        }
        if(nightMode == 1)
        {
            timetip = my_hour*6 + my_minute/10;
            onNightMode = 0;
            if(nightEnd > nightStart)
            {
                if((timetip >= nightStart) && (timetip < nightEnd))
                    onNightMode = 1;
            }
            else
            {
                if((timetip < nightEnd) || (timetip >= nightStart))
                    onNightMode = 1;
            }
            if(onNightMode == 1)
            {
                if(lightnessSet != nightlightness)        //进白天模式，白天亮度调整
                {
                    lightnessSet = nightlightness;
                    setlightness(lightnessSet);
                }
            }
            else
            {
                if(lightnessSet != daylightness)        //进白天模式，白天亮度调整
                {
                    lightnessSet = daylightness;
                    setlightness(lightnessSet);
                }
            }
        }
        else if(lightnessSet != daylightness)        //进白天模式，白天亮度调整
        {
            lightnessSet = daylightness;
            setlightness(lightnessSet);
        }
    }
}

void displayInit(void)
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //disable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    Init_Display();
    //start gpio task
    //xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);
}

