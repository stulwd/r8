#include "HardwareTimer.h"
#include "High_Temp.h"
//#include "Wire.h"
#include "I2Cdev.h"
#include "MPU9250.h"

#include <SPI.h>
#include "SdFat.h"
#include "FreeStack.h"

#include <Arduino.h>
#include <Servo.h>
#include "SHT31.h"
#include "SoftSerialSTM32.h"
//#include <avr/wdt.h>


//Camera vary
#define PIC_PKT_LEN    128
#define PIC_FMT_VGA    7
#define PIC_FMT_CIF    5
#define PIC_FMT_OCIF   3 
#define CAM_ADDR       0
#define CAM_SERIAL     Serial2
#define PIC_FMT        PIC_FMT_VGA

#define SERVO_PIN      PA0
#define SERVO1_PIN     PB9

#define sample_num_mdate  5000

#define HEATER_BAND_PIN 8
#define GPS_SERIAL SwSerial
#define RF_SERIAL Serial3
#define DebugSR Serial1
#define HEADTING_DUTY_SETTING 250//40  // 加热板占空比设置（0~255），注意占空比不要设置过高，温度过高危险
#define HEATER_ON true
#define HEATER_OFF false

#define L_SERVOR_DOWN 150
#define L_SERVOR_UP   70
#define R_SERVOR_DOWN 80
#define R_SERVOR_UP   160


// Hardware timer

// SD card shield
File myFile;
SdFat SD(2);

// RF port
bool RF_busy = false;

const byte cameraAddr = (CAM_ADDR << 5);  // addr
unsigned long picTotalLen = 0;            // picture length

//GPS vary
String GPS_data = "";

//GYRO vray
MPU9250 accelgyro;
I2Cdev I2C_M;  //接受的data命令序号;

float Axyz[3]; //三轴加速度计
float Gxyz[3]; //三轴陀螺仪
float Mxyz[3]; //三轴地磁计

volatile float mx_sample[3];
volatile float my_sample[3];
volatile float mz_sample[3];

static float mx_centre = 0;
static float my_centre = 0;
static float mz_centre = 0;

volatile int mx_max =0;
volatile int my_max =0;
volatile int mz_max =0;

volatile int mx_min =0;
volatile int my_min =0;
volatile int mz_min =0;

volatile bool heater_state = false;
volatile uint8_t operation_index = 0;      //任务序号
// Servo
Servo myservo,myservo1;         //太阳能电池板舵机
SoftSerialSTM32 SwSerial(PA15, PC10);   //TX:PC10  RX:PA15
// temp control
// High Tempature Sensor vary
HighTemp ht(PA2, PA1);    //读取模拟值，不依赖库

// SHT31 Temperature&Huminity Sensor
SHT31 sht31 = SHT31();          //IIC协议传感器，依赖wire总线，需改为模拟IIC

// sys_data
struct SYS_DATA{
    // 传感器数据
    int heating_panel_temp = 0;     //加热板温度
    float inside_temp = 0.0;        //温度
    float inside_humi = 0.0;        //湿度
    float posture_x = 0.0;          //xyz坐标
    float posture_y = 0.0;
    float posture_z = 0.0;
    float heading = 0.0;            
    float gps_latitude = 110.988356;    //gps经纬度
    float gps_longitude = 19.646334;
    float gps_time = 0;
    // 执行器状态
    bool is_solar_panel_on = false;     //太阳能电池板闭合状态
}sys_data;


enum communicate {                              //接受的data命令序号，匹配下面的任务处理序号
    COMM_SOH = 0x01,
    COMM_EOT = 0x04,
    COMM_ACK = 0x06,
    COMM_NAK = 0x15,
    COMM_CAN = 0x18,
    COMM_FAK = 0xFF,

    // command
    COMM_REQUEST_SAT_DATA = 254,
    COMM_OPEN_SOLAR_PANEL = 253,
    COMM_CLOSE_SOLAR_PANEL = 252,
    COMM_RESET_SYS = 251,
    COMM_TURN_ON_HEATER = 250,
    COMM_TURN_OFF_HEATER = 249,
    // COMM_TAKE_PHOTO = 248,
    COMM_PRE_CAPTURE = 247,
    COMM_GET_PIC_LEN = 246,
    COMM_SEND_PIC_DATA = 245,
    COMM_RESET = 244,
};

enum Option_Index {                                 //任务处理序号
    // comm execute index
    opt_Pre_Capture = 0,
    opt_Get_Pic_Len,
    opt_Send_Pic_data,
    opt_Turn_On_Heater,
    opt_Turn_Off_Heater,
    opt_Open_Solar_Panel,
    opt_Close_Solar_Panel,
    opt_Request_Sat_Data,
    opt_Reset,
    opt_default,
};

bool is_opt_busy = false;


void comm_run(HardwareSerial* port);        
void RFSerial_ISR();
void set_heater(bool state);
void set_heater_on();
void set_heater_off();
void set_solar_panel_up();
void set_solar_panel_down();
void RF_Serial_test();
bool RF_data_update(void);
String getGPSdata();
byte crc8(byte *data, byte len) ;
float getHeading(void);
float getTiltHeading(void);
void Mxyz_init_calibrated ();
void get_calibration_Data ();
void get_one_sample_date_mxyz();
void getAccel_Data(void);
void getGyro_Data(void);
void getCompass_Data(void);
void getCompassDate_calibrated ();
void clearRxBuf();
void cam_sendCmd(char cmd[], int cmd_len);
void camera_initialize();
int preCapture(void);
int Capture();
int GetData(void);
int sendData(void);
void serialEvent2(void);
bool MAG3110_Init(void);

volatile uint8_t cnt = 0;

void comm_run(HardwareSerial* port)         //中断服务函数5ms调用一次
{
    char data;
    
    if(port->available()) {         //检查端口是否已有上位机发来命令数据，若有则接受
        data = port->read();        
        //port->write(data);
        //DebugSR.print("Received data: 0x");
        //DebugSR.println(data, HEX);      //打印在调试端

        if(is_opt_busy == true) {       //如果舵机任务繁忙则停止通信
            return;
        } 
        
        switch((uint8_t)data) {
            case COMM_SOH:
                operation_index = opt_default;
                break;

            case COMM_EOT:  // End of transmit
                break;

            case COMM_ACK:

                break;

            case COMM_NAK:
                break;

            case COMM_CAN:  // Cancle anyway
                break;

            case COMM_REQUEST_SAT_DATA:         //参数请求命令
                operation_index = opt_Request_Sat_Data;
                break;

            case COMM_OPEN_SOLAR_PANEL:         //打开太阳能板命令
                operation_index = opt_Open_Solar_Panel;
                break;
            
            case COMM_CLOSE_SOLAR_PANEL:        //闭合太阳能板命令
                operation_index = opt_Close_Solar_Panel;

                break;

            case COMM_TURN_ON_HEATER:           //开启加热板命令
                operation_index = opt_Turn_On_Heater;
                break;

            case COMM_TURN_OFF_HEATER:          //关闭加热板命令
                operation_index = opt_Turn_Off_Heater;
                break;

            case COMM_PRE_CAPTURE:              //摄像头预捕获命令
                operation_index = opt_Pre_Capture;
                break;

            case COMM_GET_PIC_LEN:              //获取图像数据长度
                operation_index = opt_Get_Pic_Len;
                break;

            case COMM_SEND_PIC_DATA:            //传图命令
                operation_index = opt_Send_Pic_data;
                break;

            case COMM_RESET:                    //复位命令
                operation_index = opt_Reset;
                break;

            default: break;
        }
    }
}


void setup() {
    //DebugSR init
    DebugSR.begin(115200);  // Sreial0 init
    DebugSR.println("init....");

    //SD carc shield init
    const uint8_t SD2_CS = PB12;   // chip select for sd2
    pinMode(SD2_CS,OUTPUT);          // CS pin of SD Card Shield
    if (!SD.begin(SD2_CS))
    {
        DebugSR.println("sd init failed\n");
        // return;
    } else {
      DebugSR.println("sd init done.\n");
    }
    
    // RF init
    RF_SERIAL.begin(9600);
    DebugSR.println("init....");

    // Camera init
    CAM_SERIAL.begin(38400);  
    camera_initialize();
    
    // GPS init
    GPS_SERIAL.begin(9600);  
    DebugSR.println("GPS init succeed");

    //High Temperature Sensor init
    ht.begin();
    DebugSR.println("High Temperature Sensor init suceed");

    //gyro init
    Wire.begin();           //使用硬件IIC
    accelgyro.initialize();
    DebugSR.println(accelgyro.testConnection() ? "MPU9250 connection successful" : "MPU9250 connection failed");

    //MAG3110
    DebugSR.println(MAG3110_Init() ? "MAG3110 Init successed" : "MAG3110 Init failed");

    //servo init
    myservo.attach(SERVO_PIN);
    myservo1.attach(SERVO1_PIN);
    myservo.write(R_SERVOR_DOWN);
    myservo1.write(L_SERVOR_DOWN);
    sys_data.is_solar_panel_on = false;

    // Temperature and humidity sensor init
    sht31.begin();          //使用硬件IIC

    //temp control
    pinMode(HEATER_BAND_PIN, OUTPUT);
    analogWrite(HEATER_BAND_PIN, 0);

    // Timer1.initialize(5000);  // 定时中断 us
    // Timer1.attachInterrupt( RFSerial_ISR );
    Timer1.setPeriod(5000);         //定时器中断5ms一次
    Timer1.attachInterrupt(0, RFSerial_ISR);
    Timer1.resume();
    // heater_state = true;  // 加热板状态 - 开

    RF_SERIAL.setTimeout(1000); //rf_serial接受数据最长等待时间1ms

    operation_index = opt_default;      //当前任务为默认
}

/**
 * @函数：void RFSerial_ISR()
 * @说明： 无线串口定时中断服务, 中断里面时间是静止的，
 *        millis()不会得到更新，delay() 函数用不了。
 *        
 */
void RFSerial_ISR() {
    
    // if(RF_SERIAL.available()) {
    //     DebugSR.println("Enter timer1 interrupt!");
        comm_run(&RF_SERIAL);       //此函数只是用来获取指令，匹配任务序号。
        // RF_Serial_test();
    // }
}

void set_heater(bool state)             //设置加热板
{
    if (HEATER_ON == state) {
        analogWrite(HEATER_BAND_PIN, HEADTING_DUTY_SETTING);
    }else {
        analogWrite(HEATER_BAND_PIN, 0);
    }
}

void set_heater_off()
{
    heater_state = false;
}

void set_heater_on()
{
    heater_state = true;
}

/**
 * @函数：void set_solar_panel_on()
 */
void set_solar_panel_up()
{
    if(sys_data.is_solar_panel_on == false) {
        sys_data.is_solar_panel_on = true;
         for(int l = L_SERVOR_DOWN, r = R_SERVOR_DOWN; (l >= L_SERVOR_UP) || (r <=R_SERVOR_UP); l--, r++) {
            myservo.write(r);
            myservo1.write(l);
            delay(40);
        }
    }

}

/**
 * @函数：void set_solar_panel_off()
 */
void set_solar_panel_down()
{    
    if(sys_data.is_solar_panel_on == true) {
        sys_data.is_solar_panel_on = false;
        for(int l = L_SERVOR_UP, r = R_SERVOR_UP; (l <= L_SERVOR_DOWN) && (r >= R_SERVOR_DOWN); l++, r--) {
            myservo.write(r);
            myservo1.write(l);
            delay(40);    
        }      
    }
}

void loop() {

      //static volatile uint32_t timer_start = millis();            //millis 声明在wirish.h里
      //uint32_t timer_start = 1;
     static volatile uint32 timer_start = 0;

     char *p;
     char buf[512] = {};
     long temp = 0;
    //DebugSR.println("test loop");

    serialEvent2();
    if (1000 < millis() - timer_start) {                    //任务处理函数，确保前后两次处理时间差大于1s
        DebugSR.println("fresh sensor data!");

        timer_start = millis();
        
        set_heater(HEATER_OFF);                         //加热板关闭
        delay(5);                                       //延时5毫秒

        for(int i = 0; i < 50; i++) {                   //测试加热板温度50次，计算平均值
            temp += ht.getThmc();
        }
        temp = temp / 50;
        sys_data.heating_panel_temp = temp;             //保存加热板温度数据

        set_heater(heater_state);                       //执行加热

        // get Temperature and humiduty
        sys_data.inside_temp = sht31.getTemperature();  //读取温度并保存

        sys_data.inside_humi = sht31.getHumidity();     //读取湿度并保存
        DebugSR.print("temp & humi:\n");
        DebugSR.print(sys_data.inside_temp);
        DebugSR.print("\n");
        DebugSR.print(sys_data.inside_humi);
        DebugSR.print("\n");

        // 9DOF posture
        getAccel_Data();                                //读取加速度数据
        sys_data.posture_x = Axyz[0];                   //保存三轴加速度数据
        sys_data.posture_y = Axyz[1];
        sys_data.posture_z = Axyz[2];
        
        DebugSR.print("Axyz:\n");
        DebugSR.print(sys_data.posture_x);
        DebugSR.print("\n");
        DebugSR.print(sys_data.posture_y);
        DebugSR.print("\n");
        DebugSR.print(sys_data.posture_z);
        DebugSR.print("\n");        

        // Gyro
        getGyro_Data();
        DebugSR.print("Gxyz:\n");
        DebugSR.print(Gxyz[0]);
        DebugSR.print("\n");
        DebugSR.print(Gxyz[1]);
        DebugSR.print("\n");
        DebugSR.print(Gxyz[2]);
        DebugSR.print("\n");  
        // compass heading
        sys_data.heading = getHeading();                //获取地磁航向，使用IICdev协议
        DebugSR.print("Heading:\n");
        DebugSR.print(sys_data.heading);
        DebugSR.print("\n");
        
        if (GPS_data.length() > 0) {
           // DebugSR.print(GPS_data);
            GPS_data.toCharArray(buf, GPS_data.length());
            p = strtok(buf, ",");
            p = strtok(NULL, ",");
            sys_data.gps_time = atof(p);
            p = strtok(NULL, ",");
            DebugSR.print("GPS TIME:");
            DebugSR.print(sys_data.gps_time, 4);

            sys_data.gps_longitude = atof(p);
            DebugSR.print("GPS longitude: ");
            DebugSR.print(sys_data.gps_longitude, 4);
            
            p = strtok(NULL, ",");
            p = strtok(NULL, ",");
            sys_data.gps_latitude = atof(p);
            DebugSR.print("GPS latitude: ");
            DebugSR.println(sys_data.gps_latitude, 4);
        } 
    }

    switch(operation_index) {

        case opt_Pre_Capture:
            RF_SERIAL.write(COMM_PRE_CAPTURE);
            if(-1 == preCapture()) {            //预捕捉任务，成功后直接下一步捕捉
                operation_index == opt_default;
                break;
            }
            if(-1 == Capture()) {               //捕捉任务，成功后直接获取图像至SD卡
                operation_index == opt_default;
                break;
            }
            
            if (0 == GetData()) {               //获取图像至SD卡
                DebugSR.println("GetData Succeed!");
            } else {
                DebugSR.println("GetData Error!");
            }

            operation_index = opt_default;

            break;

        case opt_Get_Pic_Len:                      //获取图像长度信息
            operation_index = opt_default;         //没有找到赋值的地方
            RF_SERIAL.println(picTotalLen);
            picTotalLen = 0;
            break;

        case opt_Send_Pic_data:                     //发送摄像头数据
            RF_SERIAL.write(COMM_SEND_PIC_DATA);    //发送状态
            sendData();
            break;

        case opt_Turn_On_Heater:                    //开启加热板
            operation_index = opt_default;  
            set_heater_on();                        
            RF_SERIAL.write(COMM_TURN_ON_HEATER);   //发送状态
            heater_state = true;                    //貌似多余
            break;

        case opt_Turn_Off_Heater:                   //关闭加热板
            operation_index = opt_default;  
            set_heater_off();                       
            RF_SERIAL.write(COMM_TURN_OFF_HEATER);  //发送状态
            heater_state = false;                   //貌似多余
            break;

        case opt_Open_Solar_Panel:                  //打开太阳能电池板
            operation_index = opt_default;  
            is_opt_busy = true;                     //置位处理繁忙，舵机任务不能打断？

            RF_SERIAL.write(COMM_OPEN_SOLAR_PANEL); //发送状态
            //RF_SERIAL.write(COMM_FAK);

            set_solar_panel_up();                   //开启太阳能电池板
            is_opt_busy = false;                    //置位闲置

            break;

        case opt_Close_Solar_Panel:                 //闭合太阳能电池板
            operation_index = opt_default;  
            is_opt_busy = true;                     //置繁忙

            RF_SERIAL.write(COMM_CLOSE_SOLAR_PANEL);//发送状态
            //RF_SERIAL.write(COMM_FAK);

            set_solar_panel_down();                 
            is_opt_busy = false;                    //闲置
            break;

        case opt_Request_Sat_Data:                  //读取卫星参数
            operation_index = opt_default;
            RF_data_update();                       //参数更新并发送
            break;

        case opt_Reset:                             //复位
            operation_index = opt_default;
            RF_SERIAL.write(COMM_RESET);            //发送复位
            //RF_SERIAL.write(COMM_FAK);
            break;



        default: break;
    }

    /* 加热板功率过高，在发射RF信号和检测温度时禁用了
       加热，这里恢复禁用前的加热板状态 */
    set_heater(heater_state);                       //设置加热板
}


/**
 * @function: RF_Serial_test()
 */
void RF_Serial_test()
{
    if(RF_SERIAL.available()) {
        char chr = RF_SERIAL.read();
        RF_SERIAL.write(chr);
        DebugSR.print(chr, HEX);
    }
}



/**
 * @function：RF_data_update()
 * @数据格式： “传感器名:数值;校验值”
 * @校验值: 与一个特定的数字进行和校验，这里让传感器数值加“1”
 */
bool RF_data_update(void)
{
    int checkValue = 1;

    RF_SERIAL.print("{");
    // 加热板温度
    RF_SERIAL.print("\"HEAT_TMP\":");
    RF_SERIAL.print("{");
    RF_SERIAL.print("\"value\": ");

    RF_SERIAL.print(sys_data.inside_temp);
    RF_SERIAL.print(",\"check\": ");
    RF_SERIAL.print(sys_data.inside_temp + checkValue);

    // RF_SERIAL.print(sys_data.heating_panel_temp);
    // RF_SERIAL.print(",\"check\": ");
    // RF_SERIAL.print(sys_data.heating_panel_temp + checkValue);
    RF_SERIAL.print("},");
    // string.toFloat
    // 内温度
    RF_SERIAL.print("\"IN_TMP\":");
    RF_SERIAL.print("{");
    RF_SERIAL.print("\"value\": ");
    RF_SERIAL.print(sys_data.inside_temp);
    RF_SERIAL.print(",\"check\": ");
    RF_SERIAL.print(sys_data.inside_temp + checkValue);
    RF_SERIAL.print("},");

    // 内湿度
    RF_SERIAL.print("\"IN_HUMI\":");
    RF_SERIAL.print("{");
    RF_SERIAL.print("\"value\": ");
    RF_SERIAL.print(sys_data.inside_humi);
    RF_SERIAL.print(",\"check\": ");
    RF_SERIAL.print(sys_data.inside_humi + checkValue);
    RF_SERIAL.print("},");

    // 姿态X轴
    RF_SERIAL.print("\"POS_X\":");
    RF_SERIAL.print("{");
    RF_SERIAL.print("\"value\": ");
    RF_SERIAL.print(sys_data.posture_x);
    RF_SERIAL.print(",\"check\": ");
    RF_SERIAL.print(sys_data.posture_x + checkValue);
    RF_SERIAL.print("},");

    // 姿态Y轴
    RF_SERIAL.print("\"POS_Y\":");
    RF_SERIAL.print("{");
    RF_SERIAL.print("\"value\": ");
    RF_SERIAL.print(sys_data.posture_y);
    RF_SERIAL.print(",\"check\": ");
    RF_SERIAL.print(sys_data.posture_y + checkValue);
    RF_SERIAL.print("},");

    // 姿态Z轴
    RF_SERIAL.print("\"POS_Z\":");
    RF_SERIAL.print("{");
    RF_SERIAL.print("\"value\": ");
    RF_SERIAL.print(sys_data.posture_z);
    RF_SERIAL.print(",\"check\": ");
    RF_SERIAL.print(sys_data.posture_z + checkValue);
    RF_SERIAL.print("},");

    // 地磁航向
    RF_SERIAL.print("\"HEAD\":");
    RF_SERIAL.print("{");
    RF_SERIAL.print("\"value\": ");
    RF_SERIAL.print(sys_data.heading);
    RF_SERIAL.print(",\"check\": ");
    RF_SERIAL.print(sys_data.heading + checkValue);
    RF_SERIAL.print("},");

    // GPS 纬度
    RF_SERIAL.print("\"GPS_LAT\":");
    RF_SERIAL.print("{");
    RF_SERIAL.print("\"value\": ");
    RF_SERIAL.print(sys_data.gps_latitude, 4);
    RF_SERIAL.print(",\"check\": ");
    RF_SERIAL.print(sys_data.gps_latitude + checkValue, 4);
    RF_SERIAL.print("},");

    // GPS 经度
    RF_SERIAL.print("\"GPS_LON\":");
    RF_SERIAL.print("{");
    RF_SERIAL.print("\"value\": ");
    RF_SERIAL.print(sys_data.gps_longitude, 4);
    RF_SERIAL.print(",\"check\": ");
    RF_SERIAL.print(sys_data.gps_longitude + checkValue, 4);
    RF_SERIAL.print("},");

    RF_SERIAL.print("}");
    RF_SERIAL.println("");
    //RF_SERIAL.print(0xFF);


    return true;
}

/*
 * @brief Use seiral2 to reveive GPS data   
 */
void serialEvent2()
//String getGPSdata()
{
    String inString = "";
    //if (!RF_busy) {
        //DebugSR.println("GPS data actived...");
        if (GPS_SERIAL.available()) {
            char inChar = GPS_SERIAL.read();

            if(inChar == '$'){             //'$' means a new line of data
                
                GPS_SERIAL.setTimeout(100);
                String inString = GPS_SERIAL.readStringUntil('\n');
                if(inString.substring(0,5) == "GPGGA"){     //"GPGGA" is a line of message which include the message of possition, then send it with DebugSR
                    GPS_data = inString;
                    // DebugSR.println(GPS_data);
                }
            }
        }
        //return inString;
    //}
}

/*
 * @brief a simple crc8
 */
byte crc8(byte *data, byte len) {
  byte crc = 0x00;

  while (len--) {
    byte extract = *data++;
    for (byte bit = 8; bit; bit--) {
      byte sum = (crc ^ extract) & 0x01;
      crc >>= 1;
      if (sum) {
        crc ^= 0x8C;
      }
      extract >>= 1;
    }
  }
  return crc;
}

/**
* Gyro functions
*/

float getHeading(void) {
    float heading;

    getCompassDate_calibrated(); // compass data has been calibrated here
    heading=180*atan2(Mxyz[1],Mxyz[0])/PI;
    if(heading <0) heading +=360;

    return heading;
}

float getTiltHeading(void)
{
    float tiltheading;
    float pitch = asin(-Axyz[0]);
    float roll = asin(Axyz[1]/cos(pitch));

    getCompassDate_calibrated(); // compass data has been calibrated here
    float xh = Mxyz[0] * cos(pitch) + Mxyz[2] * sin(pitch);
    float yh = Mxyz[0] * sin(roll) * sin(pitch) + Mxyz[1] * cos(roll) - Mxyz[2] * sin(roll) * cos(pitch);
    float zh = -Mxyz[0] * cos(roll) * sin(pitch) + Mxyz[1] * sin(roll) + Mxyz[2] * cos(roll) * cos(pitch);
    tiltheading = 180 * atan2(yh, xh)/PI;
    if(yh<0)    tiltheading +=360;

    return tiltheading;
}

void Mxyz_init_calibrated () {

    DebugSR.println(F("Before using 9DOF,we need to calibrate the compass frist,It will takes about 2 minutes."));
    DebugSR.print("  ");
    DebugSR.println(F("During  calibratting ,you should rotate and turn the 9DOF all the time within 2 minutes."));
    DebugSR.print("  ");
    DebugSR.println(F("If you are ready ,please sent a command data 'ready' to start sample and calibrate."));
    while(!DebugSR.find("ready"));
    DebugSR.println("  ");
    DebugSR.println("ready");
    DebugSR.println("Sample starting......");
    DebugSR.println("waiting ......");

    get_calibration_Data ();

    DebugSR.println("     ");
    DebugSR.println("compass calibration parameter ");
    DebugSR.print(mx_centre);
    DebugSR.print("     ");
    DebugSR.print(my_centre);
    DebugSR.print("     ");
    DebugSR.println(mz_centre);
    DebugSR.println("    ");
}


void get_calibration_Data () {
    for (int i=0; i<sample_num_mdate;i++)
    {
        get_one_sample_date_mxyz();

        if (mx_sample[2]>=mx_sample[1])mx_sample[1] = mx_sample[2];
        if (my_sample[2]>=my_sample[1])my_sample[1] = my_sample[2]; //find max value
        if (mz_sample[2]>=mz_sample[1])mz_sample[1] = mz_sample[2];

        if (mx_sample[2]<=mx_sample[0])mx_sample[0] = mx_sample[2];
        if (my_sample[2]<=my_sample[0])my_sample[0] = my_sample[2];//find min value
        if (mz_sample[2]<=mz_sample[0])mz_sample[0] = mz_sample[2];

    }

    mx_max = mx_sample[1];
    my_max = my_sample[1];
    mz_max = mz_sample[1];

    mx_min = mx_sample[0];
    my_min = my_sample[0];
    mz_min = mz_sample[0];

    mx_centre = (mx_max + mx_min)/2;
    my_centre = (my_max + my_min)/2;
    mz_centre = (mz_max + mz_min)/2;

}

void get_one_sample_date_mxyz() {
    getCompass_Data();
    mx_sample[2] = Mxyz[0];
    my_sample[2] = Mxyz[1];
    mz_sample[2] = Mxyz[2];
}

void getAccel_Data(void)                //获取九轴加速度数据
{
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t   mx, my, mz;

    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Axyz[0] = (double) ax / 16384;//16384  LSB/g
    Axyz[1] = (double) ay / 16384;
    Axyz[2] = (double) az / 16384;
}

void getGyro_Data(void)
{
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t   mx, my, mz;

    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Gxyz[0] = (double) gx * 250 / 32768; //131 LSB(????/s)
    Gxyz[1] = (double) gy * 250 / 32768;
    Gxyz[2] = (double) gz * 250 / 32768;
}

void MAG3110_Standby(void)
{
    byte n;
    I2C_M.readByte(0x0E, 0x10, &n); 
    I2C_M.writeByte(0x0E, 0x10, n&0xFC|0x00);
}

void MAG3110_Active(void)
{
    byte n; 
    I2C_M.readByte(0x0E, 0x10, &n); 
    I2C_M.writeByte(0x0E, 0x10, n&0xFC|0x01);
}

bool MAG3110_Init(void)
{
    byte n;
    MAG3110_Standby();
    I2C_M.writeByte(0x0E, 0x10, 0X40);
    MAG3110_Active();
    I2C_M.readByte(0x0E, 0x07, &n);
    return n == 0xC4;
}
void getCompass_Data(void)
{
    uint8_t i;
    int16_t   mx, my, mz;
    uint8_t buffer_m[6];
    uint8_t count;
    I2C_M.readByte(0x0E, 0x00, &i);
    if( i & 0x08 )
    {
        I2C_M.writeByte(/*0x0C*/0x0E, 0x0A, 0x01); //enable the magnetometer
        I2C_M.readBytes(/*0x0C*/0x0E, 0x01, 6, buffer_m);
    }
    mx = ((int16_t)(buffer_m[1]) << 8) | buffer_m[0];
    my = ((int16_t)(buffer_m[3]) << 8) | buffer_m[2];
    mz = ((int16_t)(buffer_m[5]) << 8) | buffer_m[4];

    Mxyz[0] = (double) mx * 4800 / 8192;
    Mxyz[1] = (double) my * 4800 / 8192;
    Mxyz[2] = (double) mz * 4800 / 8192;
}

void getCompassDate_calibrated ()
{
    getCompass_Data();
    Mxyz[0] = Mxyz[0] - mx_centre;
    Mxyz[1] = Mxyz[1] - my_centre;
    Mxyz[2] = Mxyz[2] - mz_centre;
}


/**
* camera functions
*/
void clearRxBuf()
{
    while (CAM_SERIAL.available())
    {
        CAM_SERIAL.read();
    }
}

void cam_sendCmd(char cmd[], int cmd_len)
{
    for (char i = 0; i < cmd_len; i++) CAM_SERIAL.print(cmd[i]);
}

void camera_initialize()
{
    char cmd[] = {0xaa,0x0d|cameraAddr,0x00,0x00,0x00,0x00} ;
    unsigned char resp[6];
    int init_cnt = 100;
    bool init_state = false;

    CAM_SERIAL.setTimeout(500);
    while (1)
    {

        init_cnt --;
        if(0 == init_cnt) {
            break;
        }

        //clearRxBuf();
        cam_sendCmd(cmd,6);
        if (CAM_SERIAL.readBytes((char *)resp, 6) != 6)     //读取6字节，若不是6个字节则重试，最多试5次放弃
        {
            continue;
        }
        if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x0d && resp[4] == 0 && resp[5] == 0)
        {
            if (CAM_SERIAL.readBytes((char *)resp, 6) != 6) continue;
            if (resp[0] == 0xaa && resp[1] == (0x0d | cameraAddr) && resp[2] == 0 && resp[3] == 0 && resp[4] == 0 && resp[5] == 0) 
            {init_state = true; break;} //再读取6字节判断是否初始化成功
        }
    }
    cmd[1] = 0x0e | cameraAddr;
    cmd[2] = 0x0d;
    cam_sendCmd(cmd, 6);
    if(init_state) {
        DebugSR.println("\nCamera initialization done.\n");
    } else {
        DebugSR.println("\nCamera initialize failed...\n");
    }
}

/**
 * @函数：int preCapture(void)
 * @说明：摄像头拍照，照片数据存在摄像头模块中
 * @返回值：成功返回0， 失败返回-1
 */
int preCapture(void)
{
    char cmd[] = { 0xaa, 0x01 | cameraAddr, 0x00, 0x07, 0x00, PIC_FMT };
    unsigned char resp[6];
    int err_cnt = 0;

    CAM_SERIAL.setTimeout(100);
    while (err_cnt < 10)
    {
        clearRxBuf();
        cam_sendCmd(cmd, 6);
        if (CAM_SERIAL.readBytes((char *)resp, 6) != 6) continue;
        if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x01 && resp[4] == 0 && resp[5] == 0) 
        {
            return 0;
        }
        err_cnt ++;
    }

    return -1;
}

/**
 * @函数：int Capture(void)
 * @说明：拍照并储存照片数据到Serial Camera 模块
 * @返回：成功返回0， 失败返回-1
 */

int Capture(void)
{
    char cmd[] = { 0xaa, 0x06 | cameraAddr, 0x08, PIC_PKT_LEN & 0xff, (PIC_PKT_LEN>>8) & 0xff ,0};
    unsigned char resp[6];
    int err_cnt = 0;

    CAM_SERIAL.setTimeout(100);
    while (err_cnt < 10)
    {
        clearRxBuf();
        cam_sendCmd(cmd, 6);
        if (CAM_SERIAL.readBytes((char *)resp, 6) != 6) continue;
        if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x06 && resp[4] == 0 && resp[5] == 0) break;
        err_cnt ++;

    }
    cmd[1] = 0x05 | cameraAddr;
    cmd[2] = 0;
    cmd[3] = 0;
    cmd[4] = 0;
    cmd[5] = 0;

    err_cnt = 0;
    while (err_cnt < 10)
    {
        clearRxBuf();
        cam_sendCmd(cmd, 6);
        if (CAM_SERIAL.readBytes((char *)resp, 6) != 6) continue;
        if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x05 && resp[4] == 0 && resp[5] == 0) break;
        err_cnt ++;

    }
    cmd[1] = 0x04 | cameraAddr;
    cmd[2] = 0x1;
    err_cnt = 0;

    while (err_cnt < 10)
    {
        clearRxBuf();
        cam_sendCmd(cmd, 6);
        if (CAM_SERIAL.readBytes((char *)resp, 6) != 6) continue;
        if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x04 && resp[4] == 0 && resp[5] == 0)
        {
            CAM_SERIAL.setTimeout(1000);
            if (CAM_SERIAL.readBytes((char *)resp, 6) != 6)
            {
                continue;
            }
            if (resp[0] == 0xaa && resp[1] == (0x0a | cameraAddr) && resp[2] == 0x01)
            {
                picTotalLen = (resp[3]) | (resp[4] << 8) | (resp[5] << 16);
                break;
            }
        }
        err_cnt ++;
    }

    return 0;

}

int GetData(void)
{
    unsigned int pktCnt = (picTotalLen) / (PIC_PKT_LEN - 6);
    if ((picTotalLen % (PIC_PKT_LEN-6)) != 0) pktCnt += 1;

    char cmd[] = { 0xaa, 0x0e | cameraAddr, 0x00, 0x00, 0x00, 0x00 };
    unsigned char pkt[PIC_PKT_LEN];

    char picName[] = "pic.jpg";     //make a jpg file named "pic.jpg", the file will be removed if it has already exist.

    if (SD.exists(picName)) {
        SD.remove(picName);
    }

    myFile = SD.open(picName, FILE_WRITE);
    if(!myFile) {
        DebugSR.print("Error: ["); 
        DebugSR.print(__LINE__);
        DebugSR.print("]: ");
        DebugSR.println("Open file error!");
        return -1;
    }

    CAM_SERIAL.setTimeout(1000);
    for (unsigned int i = 0; i < pktCnt; i++)
    {
        cmd[4] = i & 0xff;
        cmd[5] = (i >> 8) & 0xff;

        int retry_cnt = 0;
        retry:
        delay(10);
        clearRxBuf();
        cam_sendCmd(cmd, 6);
        uint16_t cnt = CAM_SERIAL.readBytes((char *)pkt, PIC_PKT_LEN);

        unsigned char sum = 0;
        for (int y = 0; y < cnt - 2; y++)
        {
            sum += pkt[y];
        }
        if (sum != pkt[cnt-2])
        {
            if (++retry_cnt < 100) goto retry;
            else break;
        }

        myFile.write((const uint8_t *)&pkt[4], cnt-6);  //write the date getted from camera.
    }
    cmd[4] = 0xf0;
    cmd[5] = 0xf0;
    cam_sendCmd(cmd, 6);

    myFile.close();

    return 0;
}

/**
 * @函数：int sendData(void)
 * @说明：发送图片数据
 * @返回值：int， 成功返回0， 异常返回（-1 ， -2 收到是控制台退出命令）
 */
int sendData(void){

    const int dataSize = 127;
    byte dataBuffer[dataSize];
    int tail = 0;
    // uint32_t time_cnt = 0;

    delay(1000);
    File photoFile = SD.open("pic.jpg");

    if(!photoFile) {
        DebugSR.print("Error: ["); 
        DebugSR.print(__LINE__);
        DebugSR.print("]: ");        
        DebugSR.println("Open photoFile error！");
        return -1;
    }

    while (photoFile.position() < photoFile.size()) {   //do when there is bytes in jpg file.

        // if(0 == (tail % dataSize)){
        //     time_cnt = millis();
        // }

        dataBuffer[tail++] = photoFile.read();   //fullfill the databuffer
        


        if(tail == dataSize){          //if already get dataSize byte from jpg file
            int val;
            // DebugSR.print("Read photoFile by 127 bytes: ");
            // DebugSR.println(millis() - time_cnt);

            do{
                // time_cnt = millis();
                int sum = 0;
                for(int i=0; i< dataSize; i++){
                    RF_SERIAL.write(dataBuffer[i]); // send the data in buffer
                    sum += dataBuffer[i];
                    sum = sum % 0xFF;           //calc the check byte.
                }
                RF_SERIAL.write(sum);            //send the check byte.
                tail = 0;

                // DebugSR.print("Send dataSize + 1 bytes: ");
                // DebugSR.println(millis() - time_cnt);

                // 等待监控台发来命令
                while(RF_SERIAL.available() <= 0){
                }

                val = RF_SERIAL.read();


                if(val == COMM_CAN ){
                    DebugSR.println("Received cancle!");
                    photoFile.close();
                    return -2;
                } else if(val == COMM_ACK) {
                    break;
                } else if(val != COMM_NAK) { // 收到指令，既不退出也不是重传，则退出。
                    DebugSR.println("Received rubbish!");
                    photoFile.close();
                    return -3;
                }
            } while (val == COMM_NAK);


        }
    }
    if(tail > 0){
        int val;
        do{
            int sum = 0;
            for(int i=0; i< dataSize; i++){
                if(i < tail){
                    RF_SERIAL.write(dataBuffer[i]);
                    sum += dataBuffer[i];
                    sum = sum % 0xFF;
                }else{
                    RF_SERIAL.write(0);    //if there are no bytes in jpg file, fill it with 0x00;
                }
            }
            DebugSR.print("[");
            DebugSR.print(__LINE__);
            DebugSR.print("]: ");
            DebugSR.println("Sent last pic data!");
            RF_SERIAL.write(sum);
            // 等待监控台发来命令
            while(RF_SERIAL.available() <= 0){
            }

            val = RF_SERIAL.read();
            if(val == COMM_CAN ){
                DebugSR.println("Received cancle!");
                photoFile.close();
                return -2;
            } else if(val == COMM_ACK) {
                break;
            } else if(val != COMM_NAK) { // 收到指令，既不退出也不是重传，则退出。
                DebugSR.println("Received rubbish!");
                photoFile.close();
                return -2;
            }
        } while (val == COMM_NAK);
    } /*End of "while (photoFile.position() < photoFile.size())"" */

    photoFile.close();

    return 0;
}


// END FILE





