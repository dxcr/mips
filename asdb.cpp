//echo.hpp:ֻʹ��ͨ��DISP
//echo1.cpp:��MS WINDOWS�йصĻ�ͼ(hdc,...)
//TODO:1.use asdb.cpp �����״��Ʒ,��saveAsBmp()�ۿ����.
//     2.bisheng.hpp��Ӣ���ַ���BC BGI��*.chr�ļ�?or MSC6��*.fon�ļ�?(Y!"PCDict��ȫ"P299.X!NE��ʽ)orС���eclib�ļ�?(Y,256char*4b(16x16dots)).

//Todo:asdb.exe:generate V##MDDHH.MMr
//todo:R## and V## has all true data,so used for �� forced!
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


/* ����ͼ������ */
// Ϊ���볣�����ϱ��ּ���, ���ڳ���ͼ�������ֵ����λ��1

  #define D_CAPPI       0x8000          /* CAPPI */
  #define D_PPI         0x8001          /* PPI */
  #define D_RHI         0x8002          /* RHI */
  #define D_VSCAN       0x8003          /* ��ɨ */
  #define D_ZPPI        0x8004          /* ZPPI */
  #define D_ETPPI       0x8005          /* ETPPI */
  #define D_CS          0x8006          /* ��ֱ���� */
  #define D_MUL         0x8007          /* ���ͼ */
  #define D_CLMAX       0x8008          /* �������ڻز� */
  #define D_AR          0x80c9          /* �ۻ������� */
  #define D_VIL         0x80cb          /* VIL(��ֱ����Һ̬��ˮ��) */
  #define D_SECTOR      0x80f0          /* ��ɨ */
  #define D_VSCAN8      0x8013          /* 8����CAPPI */
  #define D_VSCAN4      0x8023          /* 4����CAPPI */

  #define D_IMAGESAVED          0x0001  // ͼ�����
  #define D_DIRINTEGRATED       0x0002  // ��λ����
  #define D_HARD_R_CORRECTION   0x0004  // ���붩��(Ӳ��)
  #define D_DOPPLERMODE         0x0008  // ������ģʽ
  #define D_DPLINTENSITY        0x0010  // ������ǿ������
  #define D_DPLVELOCITY         0x0020  // �������ٶ�����
  #define D_DPLSPECTRUM         0x0030  // �������׿�����
  #define D_POLAR               0x0040  // ������
  #define D_480X400             0x0080  // 480*400ͼ��
  #define D_APREDUCED           0x0100  // �Ӳ�����
  #define D_STC                 0x0200
  #define D_STCREVISED          0x0400
  #define D_DEALIASED           0x0800
  #define D_DBZCONVERTED        0x1000
  #define D_SOFT_R_CORRECTION   0x2000
  #define D_RAW_DATA            0x4000  // δ���κ����������״�ԭʼ����

/* �����������ļ�ͷ�ṹ */
#pragma pack (1)
  typedef struct {
    char flags[8];              // �̶���־, �����趨Ϊ"NUDPL(R)"          |  0
    unsigned short headlength;        // �ļ�ͷ����(block��, ��512�ֽڵı���)    |  8
    char reserved0[2];          // ����2�ֽ�                               | 10
    char station[26];           // ̨վ��(���23���ַ�)                    | 12
    char observer[26];          // �۲�Ա����(���23���ַ�)                | 38
    unsigned char year;         // 1900������������                        | 64
    unsigned char month;        // ��                                      | 65
    unsigned char day;          // ��                                      | 66
    unsigned char hour;         // ʱ                                      | 67
    unsigned char minute;       // ��                                      | 68
    unsigned char second;       // ��                                      | 69
    unsigned short radar;             // �״��ͺ�, 38���������״�Ϊ3824          | 70
    unsigned short attribute;         // λӳ��(����λ)                          | 72
    unsigned short observemode;       // �۲�ģʽ(0-ǿ��,1-�ٶ�FFT,2-�ٶ�PPP)    | 74
    unsigned short datatype;          // ��������(PPI,CAPPI��)                   | 76
    unsigned short range;             // ����                                    | 78
    unsigned short resolution;        // �ֱ���(�����(������)*16)               | 80
    unsigned short weathercode;       // �۲�ʱ������(�ɹ۲�Ա����)              | 82
    float eleangle;             // PPI����                                 | 84
    float dirangle;             // RHI��λ                                 | 88
    float height;               // CAPPI�߶�                               | 92
    float sectorstartingdir;    // ��ɨ��ʼ��λ                            | 96
    float sectorendingdir;      // ��ɨ��ֹ��λ                            |100
    char filtertype;            // �˲�������(1,2,3,4)                     |104
    char solveuncertaintyrate;  // ��ģ��˫Ƶ��(0x43-4:3,0x32-3:2)         |105
    char DVIPshortegrationmode;   // DVIP���ִ���(0-255)                     |106
    char PPPmode;               // PPP��ʽ(1,2)                            |107
    short Vshortegrationmode;       // �ٶȻ��ִ���(0)                         |108
    short FFTposhorts;              // FFT����(32,64,128,256,512,1024)         |110
    float WaveFreq;             // �粨Ƶ��(HZ)                            |112
    float Gain;                 // ��������(dB)                            |116
    float hBeamWidth;           // ˮƽ������(��)                        |120
    float vBeamWidth;           // ��ֱ������(��)                        |124
    float r0;                   // ���붩����ʼ����(����)                  |128
    float PeakPower;            // ��ֵ����(kw)                            |132
    float MinRecPower;          // ��С�ɱ湦��(dB)                        |136
    short RadiusRevisionStartingV;// ���붩����ʼֵ(dBZ)                     |140
    short PPPPulses;              // (0-255)                                 |142
    short PPPPorts;               // (0-64)                                  |144
    short PPPInterval;            // (0-7)                                   |146
    short FrequencySet;           // (0-SingleF,1-DoubleF,2-DoublePulses)    |148
    short Frequency;              // PRF                                      |150
    long lastingtime;           // lasting time, used by D_AR              |152
    float Z_R_A;                //                                         |156
    float Z_R_B;                //                                         |160
    char reserved1[92];         // ����92�ֽ�                              |164
    unsigned short eleaglrspct;       // ���ɱ�������(ÿȦ8192)                |256
    unsigned short diraglrspct;       // ���ɱ澶����(ÿȦ8192)                |258
    unsigned short eleaglnum;         // ��ɨ��CAPPI,ZPPI������,���Ϊ30)        |260
    float eleagls[30];          // ��ɨ��CAPPI,ZPPI����                    |262
    char reserved2[130];        // ����130�ֽ�                             |382
  } DHEAD; //length=512Bytes

#pragma pack ()

/*
      ��ϵͳ�������ļ����ϸ�ʽ���£�
      �ļ�ͷ��
      ƫ��(�ֽ�)  ����    ����
        0          8b   ��־���̶�Ϊ"NUDPL(R)"��
        8          1w   �ļ�ͷ���ȣ���λ��������
                        1��=512�ֽ�
       10          2b   ����
       12         26b   ̨վ�����Կ��ַ�����
       48         26b   �۲�Ա�������Կ��ַ�����
       64          1b   1900������������
       65          1b   ��
       66          1b   ��
       67          1b   ʱ
       68          1b   ��
       69          1b   ��
       70          1w   �״��ͺţ���ϵͳ��3824
       72          1w   ����
                        λ      ����
                         0      ͼ�����
                         1      ��λ����
                         2      Ӳ�����붩��
                         3      ������ɨ�跽ʽ
                         4-5 01 ǿ������
                             10 �ٶ�����
                             11 �׿�����
                         6      ������
                                (0Ϊֱ������)
                         7      ��Ϊֱ������, ��
                                1 - 480*400 ͼ
                                0 - 480*480 ͼ
                         8      �Ӳ�����
                         9      STC
                         10     STCУ��
                         11     ��ģ��
                         12     ת����dBZ
                         13     ������붩��
                         14-15  ����
       74          1w   �۲�ģʽ
                        0       ǿ��
                        1       �ٶ�FFT
                        2       �ٶ�PPP
       76          1w   ��������
                        0x8000  CAPPI
                        0x8001  PPI
                        0x8002  RHI
                        0x8003  ���ɨ��
                        0x8004  ZPPI
                        0x8005  ETPPI
                        0x8006  ��ֱ����
                        0x8007  ���ͼ
                        0x8008  �������ز�
                        0x80c9  �ۻ�������
                        0x80cb  VIL
                        0x80f0  ��ɨ
       78          1w   ����
       80          1w   ����ⳤ���ֱ��ʣ���
                        Ϊʵ�ʾ���ⳤ*16
       82          1w   �۲�ʱ������
       84          1d   ���㣬PPI/��ɨ����
       88          1d   ���㣬RHI��λ
       92          1d   ���㣬CAPPI�߶�
       96          1d   ���㣬��ɨ��ʼ��λ
      100          1d   ���㣬��ɨ��ֹ��λ
      104          1b   �˲�����ʽ(1,2,3,4)
      105          1b   �Ⲩ����ֵ
                        0x43    4:3
                        0x32    3:2
      106          1b   DVIP���ִ���(0-255)
      107          1b   PPP������ʽ
                        1       PPP1
                        2       PPP2
      108          1w   �ٶȻ��ִ���
      110          1w   FFT����
                        ȡ32,64,128,256,512,1024
      112          1d   ���㣬�״���Ƶ(MHZ)
      116          1d   ���㣬��������(dB)
      120          1d   ���㣬ˮƽ������(?
      124          1d   ���㣬��ֱ������(?
      128          1d   ���㣬���붩����ʼ����
                        (km)
      132          1d   ���㣬��ֵ����(kw)
      136          1d   ���㣬��С�ɱ湦��(dB)
      140          1w   ���ͣ����붩����ʼֵ��
                        (dBZ)
      142          1w   PPP������(0-255)
      144          1w   PPP������(0-63)
      146          1w   PPP�������(0-7)
      148          1w   Ƶ�ʷ�ʽ
                        0       ��Ƶ
                        1       ˫Ƶ
                        2       ˫����
      150          1w   �ظ�Ƶ��(����)
      152          1d   ����ʱ��(�ۻ���������)
      156          100b ����
      256          1w   ���Ƿֱ���
                        �����������ܾ�������
      258          1w   ��λ�ֱ���
                        ����λ�����ܾ�������
      260          1w   ��ɨ����PPI��
      262       30*1d   ���㣬��ɨ����PPI��������
      382        130b   ����
      �ļ�ͷ����Ϊ���ݣ��粻����ɨ���ϣ���ֻ��һ
  �飨һ��ɨ�赥λ����PPI��RHI�ȣ���һ�����кܶ�
  ����һ������������е����ݴ�����棨�뿪�״�
  ��Զ����ʼ��ȥǰһ����ֵ����ǰ��һ�������仯��
      һ��ĸ�ʽ���£�
      1w  �������������----------------------|
      1d  �������ݳ��ȣ��������������ͳ��ȣ�  |
      1w  ������ƽ------------------|         |
      1w  ��λ��(1/8192)            |         |
      1w  ������(1/8192)            |         |
      1b  ��ѹ������                |         |
      1b  ��ʼƫ�ƣ�0-239)-|        |         |
      1b  �������ݳ���n    |-һ��   |-һ������|
      nb  �������� --------|        |         |
        . -----------------|        |         |
        .                  |-������ |         |
        . -----------------|        |         |
      1b  0xff         ���������־-|         |
        . -----------------|                  |
        .                  |-��������         |
        . -----------------|                  |
      ----------------------------------------|
                   һ       ��
      �������ɨ���ϣ�����20�������Ŀ顣
      �����Ϻ������룬����֮��ÿ����������ǰ
  �濪ʼ���������ǰ�����ֵ��
      ��ǿ�������м�¼ֵ��λΪdBZ�� �ٶ������м�
  ¼��Ϊ�ٶ�ֵ�������ʾ�����һ��ΪС������ȷ��
  0.5m/s)���׿�����ͬ�ٶ����ϡ�

*/
//v:��ѹ/��ƽ:0-255,1km sampled from DVIP(xxxm?(1000m/150m)�Թ��ʻ���?(X)average(Pr),how?(X,dont care it!))
//volt:���Զ����з�,ie.dbmW?(X,����!)
//Q:��ѹ/��ƽ=>P����?(X,���յ�����!)
//fill volt2db[] (dbz???)Z:����������?,��Ч���������?(X,r.to:�ֱ���,��������!)
//Q:volt value(0-255) is true(������ڵ�) echo ����(����),isn't it?(X,��΢С!�����޷Ŵ�,�첨���0-5V)What is it?
//A:DVIP:<<HY>>P5:0-5v A/D,8bit(0-255!)(from ����)��"���ذ�"�Ϸ�������16bit����,��"DVIP��"����ƽ��(....)
//  the value:in db:10log(x/y):����=1V.?!(X! ��������1��!��С�ɱ繦��=0db!?(X,in head= ?)��С�ɱ繦��=0dbZ?
//  What is ��С�ɱ繦��?
//�����״ﳣ��C?
//Q:ndr running now ��redisplay volt2db?DistCorrection?see binc\src,NDRINI.EXE.


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
float wFreq;// GHZ  : �粨Ƶ�� = 
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

// ʵʱ�ɼ���������ʾʱӦ�����붩�� !
// ���붩��:P->R.���淢ɢԭ��
//
int  RadFile::forPPI(char *file) //see radfile.cpp
{
  FILE *fp;

  fp=fopen(file,"rb");

  fread(&head,sizeof(DHEAD),1,fp);
  
  short dirs;
  fread(&dirs,sizeof(short),1,fp); //directions���龶����
  
  int thiselelength; //long thiselelength;=>in bc31 is 32bits.
  fread(&thiselelength,sizeof(int),1,fp); //���鳤��
  printf("total dirs = %d,long = %d\n",dirs,thiselelength);
  //991130 test OK:2+4+512+thislength=file length!
  
  fclose(fp);
  
  if(!(head.attribute & D_SOFT_R_CORRECTION))
    //SelectRangeCorrection();
	printf("soft r-correction\n");

    if((head.attribute & D_DPLSPECTRUM) == D_DPLINTENSITY) 
        {
        if(head.attribute & D_RAW_DATA) 
            { // ����ԭʼ����
            printf("raw of raw\n");
	        // short Resolution: �ֱ���(0-0.25km,1-0.5km,2-1.0 km)
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
	    //__PostError("���ɨ�����ϲ�����ʾ\n");
	    printf("Volume\n");
	    return -1;
      default:
	    printf("Invalid data\n");
	    //__PostError("�Ƿ���������, ���Ƕ���������, ��ӦΪ\n"
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

