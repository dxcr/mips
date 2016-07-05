//echo.hpp:只使用通用DISP
//echo1.cpp:和MS WINDOWS有关的绘图(hdc,...)
//TODO:1.use asdb.cpp 开发雷达产品,用saveAsBmp()观看结果.
//     2.bisheng.hpp中英文字符用BC BGI的*.chr文件?or MSC6的*.fon文件?(Y!"PCDict大全"P299.X!NE格式)or小孙的eclib文件?(Y,256char*4b(16x16dots)).

//Todo:asdb.exe:generate V##MDDHH.MMr
//todo:R## and V## has all true data,so used for 海 forced!
//Todo:all next hail identity and calc must use R##*.??? file data.

#include <stdio.h>
#include <string.h>


// Constants rounded for 21 decimals. (from BC31 math.h)
#define M_E         2.71828182845904523536
#define M_LOG2E     1.44269504088896340736
#define M_LOG10E    0.434294481903251827651
#define M_LN2       0.693147180559945309417
#define M_LN10      2.30258509299404568402
#define M_PI        3.14159265358979323846
#define M_PI_2      1.57079632679489661923
#define M_PI_4      0.785398163397448309616
#define M_1_PI      0.318309886183790671538
#define M_2_PI      0.636619772367581343076
#define M_1_SQRTPI  0.564189583547756286948
#define M_2_SQRTPI  1.12837916709551257390
#define M_SQRT2     1.41421356237309504880
#define M_SQRT_2    0.707106781186547524401


/* 定义图像类型 */
// 为了与常规资料保持兼容, 仅在常规图像类型字的最高位置1

  #define D_CAPPI       0x8000          /* CAPPI */
  #define D_PPI         0x8001          /* PPI */
  #define D_RHI         0x8002          /* RHI */
  #define D_VSCAN       0x8003          /* 体扫 */
  #define D_ZPPI        0x8004          /* ZPPI */
  #define D_ETPPI       0x8005          /* ETPPI */
  #define D_CS          0x8006          /* 垂直剖面 */
  #define D_MUL         0x8007          /* 多幅图 */
  #define D_CLMAX       0x8008          /* 柱内最在回波 */
  #define D_AR          0x80c9          /* 累积降雨量 */
  #define D_VIL         0x80cb          /* VIL(垂直积分液态含水量) */
  #define D_SECTOR      0x80f0          /* 扇扫 */
  #define D_VSCAN8      0x8013          /* 8仰角CAPPI */
  #define D_VSCAN4      0x8023          /* 4仰角CAPPI */

  #define D_IMAGESAVED          0x0001  // 图像存贮
  #define D_DIRINTEGRATED       0x0002  // 方位积分
  #define D_HARD_R_CORRECTION   0x0004  // 距离订正(硬件)
  #define D_DOPPLERMODE         0x0008  // 多普勒模式
  #define D_DPLINTENSITY        0x0010  // 多普勒强度资料
  #define D_DPLVELOCITY         0x0020  // 多普勒速度资料
  #define D_DPLSPECTRUM         0x0030  // 多普勒谱宽资料
  #define D_POLAR               0x0040  // 极坐标
  #define D_480X400             0x0080  // 480*400图像
  #define D_APREDUCED           0x0100  // 杂波抑制
  #define D_STC                 0x0200
  #define D_STCREVISED          0x0400
  #define D_DEALIASED           0x0800
  #define D_DBZCONVERTED        0x1000
  #define D_SOFT_R_CORRECTION   0x2000
  #define D_RAW_DATA            0x4000  // 未作任何软件处理的雷达原始资料

/* 多普勒资料文件头结构 */
#pragma pack (1)
  typedef struct {
    char flags[8];              // 固定标志, 必须设定为"NUDPL(R)"          |  0
    unsigned short headlength;        // 文件头长度(block数, 即512字节的倍数)    |  8
    char reserved0[2];          // 保留2字节                               | 10
    char station[26];           // 台站名(最多23个字符)                    | 12
    char observer[26];          // 观测员姓名(最多23个字符)                | 38
    unsigned char year;         // 1900年以来的年数                        | 64
    unsigned char month;        // 月                                      | 65
    unsigned char day;          // 日                                      | 66
    unsigned char hour;         // 时                                      | 67
    unsigned char minute;       // 分                                      | 68
    unsigned char second;       // 秒                                      | 69
    unsigned short radar;             // 雷达型号, 38所多普勒雷达为3824          | 70
    unsigned short attribute;         // 位映象(属性位)                          | 72
    unsigned short observemode;       // 观测模式(0-强度,1-速度FFT,2-速度PPP)    | 74
    unsigned short datatype;          // 数据类型(PPI,CAPPI等)                   | 76
    unsigned short range;             // 量程                                    | 78
    unsigned short resolution;        // 分辨率(距离库(公里数)*16)               | 80
    unsigned short weathercode;       // 观测时天气码(由观测员给出)              | 82
    float eleangle;             // PPI仰角                                 | 84
    float dirangle;             // RHI方位                                 | 88
    float height;               // CAPPI高度                               | 92
    float sectorstartingdir;    // 扇扫开始方位                            | 96
    float sectorendingdir;      // 扇扫终止方位                            |100
    char filtertype;            // 滤波器类型(1,2,3,4)                     |104
    char solveuncertaintyrate;  // 解模糊双频比(0x43-4:3,0x32-3:2)         |105
    char DVIPshortegrationmode;   // DVIP积分次数(0-255)                     |106
    char PPPmode;               // PPP方式(1,2)                            |107
    short Vshortegrationmode;       // 速度积分次数(0)                         |108
    short FFTposhorts;              // FFT点数(32,64,128,256,512,1024)         |110
    float WaveFreq;             // 电波频率(HZ)                            |112
    float Gain;                 // 天线增益(dB)                            |116
    float hBeamWidth;           // 水平波瓣宽度(度)                        |120
    float vBeamWidth;           // 垂直波瓣宽度(度)                        |124
    float r0;                   // 距离订正起始距离(公里)                  |128
    float PeakPower;            // 峰值功率(kw)                            |132
    float MinRecPower;          // 最小可辨功率(dB)                        |136
    short RadiusRevisionStartingV;// 距离订正起始值(dBZ)                     |140
    short PPPPulses;              // (0-255)                                 |142
    short PPPPorts;               // (0-64)                                  |144
    short PPPInterval;            // (0-7)                                   |146
    short FrequencySet;           // (0-SingleF,1-DoubleF,2-DoublePulses)    |148
    short Frequency;              // PRF                                      |150
    long lastingtime;           // lasting time, used by D_AR              |152
    float Z_R_A;                //                                         |156
    float Z_R_B;                //                                         |160
    char reserved1[92];         // 保留92字节                              |164
    unsigned short eleaglrspct;       // 最大可辨仰角数(每圈8192)                |256
    unsigned short diraglrspct;       // 最大可辨径向数(每圈8192)                |258
    unsigned short eleaglnum;         // 体扫或CAPPI,ZPPI仰角数,最大为30)        |260
    float eleagls[30];          // 体扫或CAPPI,ZPPI仰角                    |262
    char reserved2[130];        // 保留130字节                             |382
  } DHEAD; //length=512Bytes

#pragma pack ()

/*
      本系统产生的文件资料格式如下：
      文件头：
      偏移(字节)  长度    意义
        0          8b   标志（固定为"NUDPL(R)"）
        8          1w   文件头长度（单位：块数）
                        1块=512字节
       10          2b   保留
       12         26b   台站名，以空字符结束
       48         26b   观测员姓名，以空字符结束
       64          1b   1900年以来的年数
       65          1b   月
       66          1b   日
       67          1b   时
       68          1b   分
       69          1b   秒
       70          1w   雷达型号，本系统用3824
       72          1w   属性
                        位      意义
                         0      图像存贮
                         1      方位积分
                         2      硬件距离订正
                         3      多普勒扫描方式
                         4-5 01 强度资料
                             10 速度资料
                             11 谱宽资料
                         6      极坐标
                                (0为直角坐标)
                         7      若为直角坐标, 则
                                1 - 480*400 图
                                0 - 480*480 图
                         8      杂波抑制
                         9      STC
                         10     STC校验
                         11     解模糊
                         12     转换成dBZ
                         13     软件距离订正
                         14-15  保留
       74          1w   观测模式
                        0       强度
                        1       速度FFT
                        2       速度PPP
       76          1w   数据类型
                        0x8000  CAPPI
                        0x8001  PPI
                        0x8002  RHI
                        0x8003  体积扫描
                        0x8004  ZPPI
                        0x8005  ETPPI
                        0x8006  垂直剖面
                        0x8007  多幅图
                        0x8008  柱内最大回波
                        0x80c9  累积降雨量
                        0x80cb  VIL
                        0x80f0  扇扫
       78          1w   量程
       80          1w   距离库长（分辨率），
                        为实际距离库长*16
       82          1w   观测时天气码
       84          1d   浮点，PPI/扇扫仰角
       88          1d   浮点，RHI方位
       92          1d   浮点，CAPPI高度
       96          1d   浮点，扇扫起始方位
      100          1d   浮点，扇扫终止方位
      104          1b   滤波器形式(1,2,3,4)
      105          1b   解波糊比值
                        0x43    4:3
                        0x32    3:2
      106          1b   DVIP积分次数(0-255)
      107          1b   PPP工作方式
                        1       PPP1
                        2       PPP2
      108          1w   速度积分次数
      110          1w   FFT点数
                        取32,64,128,256,512,1024
      112          1d   浮点，雷达射频(MHZ)
      116          1d   浮点，天线增益(dB)
      120          1d   浮点，水平波瓣宽度(?
      124          1d   浮点，垂直波瓣宽度(?
      128          1d   浮点，距离订正起始距离
                        (km)
      132          1d   浮点，峰值功率(kw)
      136          1d   浮点，最小可辨功率(dB)
      140          1w   整型，距离订正起始值，
                        (dBZ)
      142          1w   PPP脉冲数(0-255)
      144          1w   PPP滑窗数(0-63)
      146          1w   PPP间隔点数(0-7)
      148          1w   频率方式
                        0       单频
                        1       双频
                        2       双脉冲
      150          1w   重复频率(上限)
      152          1d   持续时间(累积降雨量用)
      156          100b 保留
      256          1w   仰角分辨率
                        （仰角最大可能径向数）
      258          1w   方位分辨率
                        （方位最大可能径向数）
      260          1w   体扫包含PPI数
      262       30*1d   浮点，体扫包含PPI仰角序列
      382        130b   保留
      文件头以下为数据，如不是体扫资料，则只有一
  块（一个扫描单位，如PPI，RHI等），一块中有很多
  径向，一个径向的数据中的数据从最后面（离开雷达
  最远）开始减去前一个数值，最前面一个不作变化。
      一块的格式如下：
      1w  本块包含径向数----------------------|
      1d  本块数据长度（不包含径向数和长度）  |
      1w  噪声电平------------------|         |
      1w  方位码(1/8192)            |         |
      1w  仰角码(1/8192)            |         |
      1b  被压缩数据                |         |
      1b  开始偏移（0-239)-|        |         |
      1b  连续数据长度n    |-一节   |-一个径向|
      nb  连续数据 --------|        |         |
        . -----------------|        |         |
        .                  |-其他节 |         |
        . -----------------|        |         |
      1b  0xff         径向结束标志-|         |
        . -----------------|                  |
        .                  |-其他径向         |
        . -----------------|                  |
      ----------------------------------------|
                   一       块
      如果是体扫资料，则有20个这样的块。
      读资料后必须解码，解完之后每个径向从须从前
  面开始，逐个加上前面的数值。
      在强度资料中记录值单位为dBZ， 速度资料中记
  录的为速度值，补码表示，最后一个为小数（精确到
  0.5m/s)。谱宽资料同速度资料。

*/
//v:电压/电平:0-255,1km sampled from DVIP(xxxm?(1000m/150m)对功率积分?(X)average(Pr),how?(X,dont care it!))
//volt:来自对数中放,ie.dbmW?(X,调幅!)
//Q:电压/电平=>P功率?(X,接收到调幅!)
//fill volt2db[] (dbz???)Z:反射率因子?,有效照射体积内?(X,r.to:分辨率,独立采样!)
//Q:volt value(0-255) is true(天线入口的) echo 幅度(伏特),isn't it?(X,极微小!超外差极限放大,检波后得0-5V)What is it?
//A:DVIP:<<HY>>P5:0-5v A/D,8bit(0-255!)(from 对中)在"开关板"上反对数成16bit线性,在"DVIP板"算术平均(....)
//  the value:in db:10log(x/y):基数=1V.?!(X! 基数不是1伏!最小可辩功率=0db!?(X,in head= ?)最小可辩功率=0dbZ?
//  What is 最小可辩功率?
//乘以雷达常数C?
//Q:ndr running now 下redisplay volt2db?DistCorrection?see binc\src,NDRINI.EXE.


class RadFile
{

//991210 in CMA:Radar Engineer from USA
float Pt;//KW: at R: Pt/4*pi*R*R
float sigma; //scatter squares:at R:return sigma*(Pt/4*pi*R*R)
float Ae; //atenna e_squrares
float phi; //atenna phi = 4.3m
float Gain; //atenna gainning: = 45db
float minPower; //min determin signal; = Noise volt?
float minPowerDbz; // = 105dbz (important!)
float sita; //beam width: = 0.96 degree
float Pav; //W;average power; = 
float PRF; //HZ,
float tau; //1e-6 seconds,puls width
float PRF2rate; //PRF1/PRF2
float wFreq;// GHZ  : 电波频率 = 
float labda; //wave length; = 5cm

DHEAD head;

public:
    short openFile() 
    { 
	char fname[255];
	printf("\n Input file name: ");
	scanf("%s",fname);
	return openFile(fname);
	}
    short openFile(char *fname);
    int  forPPI(char *file);
};

// 实时采集的数据显示时应做距离订正 !
// 距离订正:P->R.球面发散原因。
//
int  RadFile::forPPI(char *file) //see radfile.cpp
{
  FILE *fp;

  fp=fopen(file,"rb");

  fread(&head,sizeof(DHEAD),1,fp);
  
  short dirs;
  fread(&dirs,sizeof(short),1,fp); //directions本块径向数
  
  int thiselelength; //long thiselelength;=>in bc31 is 32bits.
  fread(&thiselelength,sizeof(int),1,fp); //本块长度
  printf("total dirs = %d,long = %d\n",dirs,thiselelength);
  //991130 test OK:2+4+512+thislength=file length!
  
  fclose(fp);
  
  if(!(head.attribute & D_SOFT_R_CORRECTION))
    //SelectRangeCorrection();
	printf("soft r-correction\n");

    if((head.attribute & D_DPLSPECTRUM) == D_DPLINTENSITY) 
        {
        if(head.attribute & D_RAW_DATA) 
            { // 是最原始资料
            printf("raw of raw\n");
	        // short Resolution: 分辨率(0-0.25km,1-0.5km,2-1.0 km)
	        }
	    }

    if((head.attribute&D_DPLSPECTRUM) == D_DPLINTENSITY)
    {
      printf("if rModificationLater ==1,todo DistCorrection()\n");
      if(head.range==240)
	    printf("240km\n");
      else if(head.range==120)
	    printf("120km\n");
      if(head.attribute & D_SOFT_R_CORRECTION)
	    printf("Do Range Correction !\n");
    }

  switch(head.attribute & D_DPLSPECTRUM) { //0x0030
    case D_DPLINTENSITY: //0x0010
      //MakeLevelColorTable(dbzs,intensitycolors[0],VPort1Colors,YES);
	    printf("intensity data\n");
      // ModifyValueColor(VPort1Colors,intensitycolors[0],0);
        return 1;
      break;
    case D_DPLVELOCITY: //0x0020
      //MakeVelocityColorTable(velocitycolors[0],VPort1Colors,&head);
	    printf("velocity data\n");
	    return 2;
        break;
    case D_DPLSPECTRUM:
      //MakeSpectrumColorTable(spectrumcolors[0],VPort1Colors);
	    printf("spectrum data\n");
	    return 3;
        break;
  }

  return 0;
}

short RadFile::openFile(char *fname)
{
  FILE *fp;

  fp=fopen(fname,"rb");
  if(!fp)
    {
    printf("Can not open %s.\n",fname);
    return -1;
    }

  fread(&head,sizeof(DHEAD),1,fp);
  if(memcmp(head.flags,"NUDPL(R)",8)!=0) {
    fclose(fp);
    printf("Not doppler radar file.\n");
    return -1;
  }
  fclose(fp);

  if(head.attribute&D_IMAGESAVED) //off=72,len=1w &0x0001 = 1
	{
     //dDispImageSaved(vport,file);
     printf("%x=Disp ImageSaved.\n",head.attribute);
	}
  else 
    {
    switch(head.datatype) {
      case D_RHI:
	    //dDispRHI(vport,file);
	    printf("RHI\n");
	    break;
      case D_AR:
	    //dDispAR(vport,file);
	    printf("AR\n");
	    break;
      case D_CAPPI:
      case D_PPI:
	    //dDispPPI(vport,file);
	    printf("PPI\n");
	    // dDispPPI(fname);
	    return 1;
	    break;
      case D_VSCAN:
	    //__PostError("体积扫描资料不能显示\n");
	    printf("Volume\n");
	    return -1;
      default:
	    printf("Invalid data\n");
	    //__PostError("非法数据类型, 若是多普勒资料, 则应为\n"
	    //          "  CAPPI,RHI,PPI\n");
	return 0;
    }
  }

  return 0;

}


#include "arraydis.hpp"
#include "arraybis.hpp"
#include "echo.hpp"
#include "vecho.hpp"

void main(int argc,char *argv[])
{
	unsigned char *bmpbuf = new char[640*480];
	
	arrayDisp *ad = new arrayDisp(bmpbuf,640,480);
    arrayBishengE *ab = new arrayBishengE(ad);

    char fname[255];    
	if(argc <= 1) 
	    {
	        printf("Input file name: ");
	        scanf("%s",fname);
	    }
	else strcpy(fname,argv[1]);
    
    //now judg what is it?
    RadFile rf;
    int tp = rf.openFile(fname);
    
    if(tp == 1) // ppi,cappi:judg i,v,s
        tp = rf.forPPI(fname);
        printf("tp = %d\n",tp);
        tp=1;
        if(tp == 1) //intensity
            {
            echo3824 *p1 = new echo3824(fname,ad,ab);
            if(p1->getMsg() == 0) 
	            {
		        delete ad;
		        delete ab;
		        delete p1;
		        return;
		        }
	        else 
		        {
		        p1->LoadFile();
                p1->SaveAsBmp(); //also new arrayDisp,arrayBishengE.
		        }
		    printf("saved intensity R##\n");
            return;
        }
        else if(tp == 2)  //velocity
        {
            VEcho *p1 = new VEcho(fname,ad,ab);
            if(p1->getMsg() == 0) 
	            {
		        delete ad;
		        delete ab;
		        delete p1;
		        return;
		        }
	        else 
		        {
		        p1->LoadFile();
                p1->SaveAsBmp2(); //also new arrayDisp,arrayBishengE.
		        }
            printf("saved volecity V##\n");
            return;
        }
        else if(tp == 3)  //spectrum
        {
            SEcho *p1 = new SEcho(fname,ad,ab);
            if(p1->getMsg() == 0) 
	            {
		        delete ad;
		        delete ab;
		        delete p1;
		        return;
		        }
	        else 
		        {
		        p1->LoadFile();
                p1->SaveAsBmp2(); //also new arrayDisp,arrayBishengE.
		        }
            printf("saved spectrum S##\n");
            return;
        }
        
}

