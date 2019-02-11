
#ifndef _QR_FUNCTIONS_H_
#define _QR_FUNCTIONS_H_

#ifdef __cplusplus
extern "C"{
#endif


#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

	//********* findpattern.h **************
#define LINE_NODE_SIZE      36

#define MAX_LINE			1000
#define MAX_NEIGHBOR		100
#define MAX_HORIZONTAL		100
#define MAX_VERTICAL		100

#define NEIGHBOR_OFFSET     LINE_NODE_SIZE*MAX_LINE
#define CROSSH_OFFSET       LINE_NODE_SIZE*(MAX_LINE+MAX_NEIGHBOR)
#define CROSSV_OFFSET       LINE_NODE_SIZE*(MAX_LINE+MAX_NEIGHBOR+MAX_HORIZONTAL)
	//**************************************
	//********** ImageProcess ***********
	//#define QR_HOUGH_NUMANGLE 1800		//mark 121228		(會產生64Mbuf)
#define QR_HOUGH_NUMANGLE 180			//modify 121228		(會產生6.4Mbuf)

#define QR_PERSPECTIVE_MATRIX_SIZE 8*8*8+32
#define NUMRBO_MAXIMUM  2241//((QR_WIDTH + QR_HEIGHT) * 2 + 1)	//add 121225
#ifdef QR_V1_USED	//bypass to decode v1, while not define "QR_V1_USED"
	int accum[ ( QR_HOUGH_NUMANGLE + 2 )*( NUMRBO_MAXIMUM + 2 ) ];			//add 121225
#endif

#ifdef QR_TEST
#include "QR_test.h"
#endif
	// *************************************

	//************* QR_Code ****************
	//========================================================*/
	// Error Code Number //
#define QR_OK					0
#define QR_ERR_FINDERPATTERN	-1
#define QR_ERR_ALIGNPATTERN		-2
#define QR_ERR_IMAGE_SIZE       -3
#define QR_ERR_IMAGE_BPP        -4
#define QR_ERR_MEMORY           -5
#define QR_ERR_VERSION          -6
#define QR_ECC_FAILUR			-7
#define QR_ERR_TOO_MUCH_LINE    -8
#define QR_REDO_FINDERPATTERN	-9			//add 130715

	// Image Type //
#define IMAGETYPE_YUYV 			0x01	// Big-endian YUYV image (YUYV)
#define IMAGETYPE_UYVY 			0x02	// Little-endian YUYV image (UYVY)
#define IMAGETYPE_Y0Y1			0x04	// Big-endian Y image (Y0Y1 Y2Y3..)(Y only image)
#define IMAGETYPE_Y1Y0			0x08	// Little-endian Y image (Y1Y0 Y3Y2..)(Y only image)
#define IMAGETYPE_COLORINVERSE	0x10	// for white/black inverse pattern
	//<<for simulator DBG
#ifdef __CC_ARM
#ifdef ARMUL
#define DBG_READ_BMP
#endif
#endif

#ifndef __CC_ARM
#ifdef WIN32
#define DBG_READ_BMP
#endif
#endif
	//for simulator DBG>>

	//const char version[] = { "GP_QRdec$1004" };	//FixPoint

#ifdef QR_TEST
#include "QR_test.h"
#endif


	// ********** QR_Kernel ****************
	//========================================
	//bypass to decode v1, while not define "QR_V1_USED"
	//========================================
	//#define QR_V1_USED


	//========================================
	//Special Case for Quick Response while no pattern in figure
	// remove repeat process of calibration & decode for increase dec-time while no pattern be find
	// request by Nick.Fu (150108)
	//========================================
	//#define QR_SimpleDecFlow_USED


	//========================================
	// dump data for DBG
	//========================================
	//#define QR_TEST
#ifdef QR_TEST
#include "QR_test.h"
#endif

#ifdef QR_V1_USED //bypass to decode v1, while not define "QR_V1_USED"
	int contour_mem[ 3424 ];					//add 121225
	//int contour_mem[673424];				//add 121225

#endif

#define OTSU

	//#define NUM_SQRT_AREA	16
#define NUM_SQRT_CONS	16

#define BOXWIDTH 160  //80 //40
#define BOXHEIGHT 120 //60 //30
#define WTOT      (BOXWIDTH*BOXHEIGHT)

#define QR_LOCAL_MEMORY_SIZE 50000//16384
	//#define QR_WIDTH 		848		//640	(mdy 150204)for WIFI cam WideResolution
#define QR_WIDTH 		640
#define QR_HEIGHT		480
	//#define QR_WIDTH 		640
	//#define QR_HEIGHT		480
#define QR_MAX_VERSION	21  //This is the max version supported under 640*480 resolution without rotate
#define QR_MAXMATRIX	(21+(QR_MAX_VERSION-1)*4)
#define QR_MAX_CODEWORD 1258
#define QR_RESULT_SIZE	1024//for v21 and below
#define QR_MAX_LINE_T	1000
#define QR_MAX_LINE_N	100
#define QR_MAX_LINE_H	100
#define QR_MAX_LINE_V	100

#define READ_HORIZONTAL	0
#define READ_VERTICAL	1
#define READ_UP			1
#define READ_DOWN		0

#define UL				0
#define UR				1
#define DL				2

#define ULH				0
#define ULV				1
#define URH				2
#define URV				3
#define DLH				4
#define DLV				5

#define QR_HORIZONTAL   1
#define QR_VERTICAL     0

#define POINT_DARK		0
#define POINT_LIGHT		255

#define LOGIC_X			10
#define LOGIC_Y			10

#define LOGIC_MODULSIZE  2
#define VIRTUAL_MODULSIZE  2

#define PATTERN_FIND_RANGE   10

#define QR_TRUE		1
#define QR_FALSE	0
#define QR_NULL     ((void*)0)

#define QR_PI		3.1415926
#define QR_SIN45	46341//Q16, the Q0 value is 0.70710678

#define QR_ABS(x)	(((x)>0)?(x):(-(x)))
#define max(a,b)    (((a) > (b)) ? (a) : (b))
#define min(a,b)    (((a) < (b)) ? (a) : (b))

	/*------------------Error Num-----------------*/
#define QR_OK					0
#define QR_ERR_FINDERPATTERN	-1
#define QR_ERR_ALIGNPATTERN		-2
#define QR_ERR_IMAGE_SIZE       -3
#define QR_ERR_IMAGE_BPP        -4
#define QR_ERR_MEMORY           -5
#define QR_ERR_VERSION          -6
#define QR_ECC_FAILUR			-7
#define QR_ERR_TOO_MUCH_LINE    -8

	/*--------------------------------------------*/

	/*------------------Image Type-----------------*/
#define IMAGETYPE_YUYV 			0x01	//for YUYV
#define IMAGETYPE_UYVY 			0x02	//for UYVY
#define IMAGETYPE_Y0Y1			0x04	//for (Y0Y1 Y2Y3..)(Y only image)
#define IMAGETYPE_Y1Y0			0x08	//for (Y1Y0 Y3Y2..)(Y only image)
#define IMAGETYPE_COLORINVERSE	0x10	//for white/black inverse pattern


	// ************ RSDecode **************************
#define MAX_DATACAPACITY	1156

#define RS_PERM_ERROR	-1
#define RS_CORRECT_ERROR -2



#define QR_recong_ROI_WH_ratio  168 //224   // (256+192)/2


	// ************** Type **********************

#define INT64 long long



	// ********* CPoint *********************
	typedef struct {
		//	double x;
		//	double y;
		long x;
		long y;
	}CPoint;

	typedef struct {
		long id;
		long num_black;
	}COrder;

	typedef struct {
		CPoint p_point;
		long length_h;	//used in getLinesCenter() only, it is the length of the line that generate this point
		long length_v;	//used in getLinesCenter() only, it is the length of the line that generate this point
		//	double length_h;				//used in getLinesCenter() only, it is the length of the line that generate this point
		//	double length_v;				//used in getLinesCenter() only, it is the length of the line that generate this point
		void* p_next;		//address of the next node
	}CPointNode;

	typedef struct {
		CPointNode *header;
		CPointNode *cur;
		long total_node;					//number of node in the chain
	}CPointsChain;

	//CPoint Point(double x, double y);
	//CPoint setPoint(CPoint p, double x, double y);
	//CPoint pointTranslate(CPoint p, double dx, double dy);
	CPoint Point( long x, long y );
	CPoint pointTranslate( CPoint p, long dx, long dy );
	//double getPointX(CPoint point);
	//double getPointY(CPoint point);
	long getPointX( CPoint point );
	long getPointY( CPoint point );
	void addPointElement( CPointsChain *points, CPointNode* node, CPoint point, long length_h, long length_v );
	void removePointElement( CPointsChain *points, CPoint point );

	// *********** CLine********************
	typedef struct {
		CPoint p1;
		CPoint p2;
	}CLine;

	typedef struct {
		CLine line;
		void* p_next;		//address of the next node
	}CLineNode;

	typedef struct {
		CLineNode *header;
		CLineNode *cur;
		long length;				//size of these lines
	}CLinesChain;

	// *************** QR_Kernel *********************
	typedef struct {
		long left;
		long right;
		long top;
		long bottom;
	}RECT_QR;

	typedef struct {
		CPoint center[ 7 ][ 7 ];
		long   matrix_length;
		long   side_length;
	}CAlignmentPattern;



	typedef struct {
		CPoint center[ 4 ];
		long   version;
		long   sincos[ 2 ];
		long   moduleSize[ 6 ];
		//	double sincos[2];
		//	double moduleSize[6];
	}CFinderPattern;

	typedef struct
	{
		int ret;
		int QR_center_X;
		int QR_center_Y;
		int QR_P_X[3];
		int QR_P_Y[3];
		int QR_recong_ROI_W;
		int QR_recong_ROI_H;

	}QR_RESULT;



	typedef struct
	{
		unsigned char *graybmp;//[ QR_WIDTH*QR_HEIGHT ];
		unsigned char bitmap[ QR_WIDTH*QR_HEIGHT ];
		unsigned char local_memory[ QR_LOCAL_MEMORY_SIZE ];
		unsigned char* qrcode_matrix;				//pointer of qrcode matrix
		unsigned char* result_array;				//pointer of out put buffer

		CFinderPattern    finder_pattern;
		CAlignmentPattern alignment_pattern;
		//CQRCodeSymbol     qrcode_symbol;

		//long version;
		long width;
		long height;
		long matrix_size;
		long error_num;
		long image_type;		//0:YUYV, 1:Yonly	(add121227)

	}QRCode_MEMORY_BLOCK;


	// ********** ImageProcessing ****************
	typedef struct {
		double ang;
		double rho;
	}HoughLine;




	CPoint getLineP1( CLine line );
	CPoint getLineP2( CLine line );
	CPoint getLineCenter( CLine line );
	//CPoint axisTranslate(CAxis axis, CPoint origin, double moveX, double moveY);
	CLine Line( CPoint p1, CPoint p2 );
	CLine getLinesLongest( CLine lines[ 3 ] );
	void addLineElement( CLinesChain* plist, CLineNode* node, CLine line );
	//void translateLine(CLine *line, double dx, double dy);
	void removeLineElement( CLinesChain *lines_candidate, CLinesChain* lines_neighbor );
	long isLineCross( CLine, CLine );
	long isHorizontal( CLine l );
	long isNeighbor( CLine line1, CLine line2, long direction );
	long getLineLength( CLine line );
	static void calcSyndrome( short y[ 15 ], short *s );
	void detectErrorBitPosition( short s[ 5 ], short *e );
	void correctErrorBit( short y[ 15 ], short errorPos[ 4 ], short *output );



	// ********* BCH15_5 *******************
	void correct( short recieveData[ 15 ], short *output );
	// ********* bitmap ********************
	long ReadBitmap( FILE* f_bitmap, unsigned char** image, long* width, long* height );
	long Read_Yonly_RAWdata( FILE* f_bitmap, unsigned char** image, long* width, long* height );		//add 121227
	long Read_YUYV_RAWdata( FILE* f_bitmap, unsigned char** image, long* width, long* height );

	//********* Galois *********************
	int galois_div( int a, int b );
	int galois_divExp( int a, int b );
	int galois_toPos( int length, int a );
	int galois_mul( int a, int b );
	int galois_mulExp( int a, int b );
	void galois_mulPoly( short *seki, short *a, short *b, short npar );

	//int calcSyndrome( short* data, short length, short *syn, long syn_length );

	// ********* ImageProcess **************
	long qr_grayscale_BMP( QRCode_MEMORY_BLOCK* qr, unsigned char* image );
	long qr_grayscale( QRCode_MEMORY_BLOCK* qr, unsigned char* image );
	long qr_threshold( QRCode_MEMORY_BLOCK* qr, long flag );
	//void qr_rotate(QRCode_MEMORY_BLOCK *qr,double newxcenteroffset,double newycenteroffset);
	void qr_rotate( QRCode_MEMORY_BLOCK *qr, long newxcenteroffset, long newycenteroffset );
	long qr_calibration( QRCode_MEMORY_BLOCK* qr, HoughLine Lh, HoughLine Lv );
	long qr_Projection( QRCode_MEMORY_BLOCK* qr, CPoint p1, CPoint p2, int* pgrid );
	HoughLine qr_hough( QRCode_MEMORY_BLOCK* qr, CPoint p1, CPoint p2 );
	HoughLine qr_hough2( QRCode_MEMORY_BLOCK* qr, CPoint* points, int p_num );
	void qr_crla( QRCode_MEMORY_BLOCK *qr, int ct );
	void qr_find_contour( QRCode_MEMORY_BLOCK *qr, CPoint* contour, CPoint p1, CPoint p2, int direction );
	void qr_erode( QRCode_MEMORY_BLOCK *qr, int ksize );
	void qr_dilate( QRCode_MEMORY_BLOCK *qr, int ksize );
	void qr_lightbalancing( QRCode_MEMORY_BLOCK *qr );
	void qr_contrast( QRCode_MEMORY_BLOCK *qr );

	//[test 131121]
	long qr_Image_Inverse( QRCode_MEMORY_BLOCK* qr );

	// ************ MathFunc **********************
	long Divide_LL_Func( long x, long y );

	double F_ABS_d( double indata );
	long F_ABS_L( long indata );
	double Sqrt_Func( double indata );
	long Sqrt_Func_L( long indata );
	double Sin_Func_d( double ang );
	double Tan_Func_d( double ang );
	double Cos_Func_d( double ang );

	//float Divide_Func_f( float x, float y );
	double Divide_Func_dd( double x, double y );

	//long F_Reminder_LL( long x, long y );
	short F_Reminder_SS( short x, short y );
	//float Sqrt_Func(float indata);
	//float Divide_Func(float x, float y);

	// ************* mem_op ************************
	void QR_memset( void *s1, int s2, int n );
	void QR_memcpy( void *s1, const void *s2, int n );

	// *************QR_Kernel **********************
	QR_RESULT qr_finderpattern( QRCode_MEMORY_BLOCK* qr , unsigned char* inputGrayImage);
	long qr_findFinderPattern( QRCode_MEMORY_BLOCK* qr );
	long qr_findAlignmentPattern( QRCode_MEMORY_BLOCK *qr );
	long qr_findlastalignment( QRCode_MEMORY_BLOCK *qr );
	long qr_get_symbol( QRCode_MEMORY_BLOCK* qr );
	long qr_decode_symbol( QRCode_MEMORY_BLOCK* qr );
	long qr_SymbolInitialize( QRCode_MEMORY_BLOCK *qrcode );
	void qr_getBlocks( QRCode_MEMORY_BLOCK* qr, short *codeWords );
	void qr_correctDataBlocks( QRCode_MEMORY_BLOCK* qr, short* blocks );
	long qr_getDecodedByteArray( QRCode_MEMORY_BLOCK* qr, short *blocks );
	//long qr_calcExactVersion( QRCode_MEMORY_BLOCK* qr );
	long qr_getLogicalSeed( int version, int patternNumber );

	//************ QR_Test *************************
	void test_output_bmp( unsigned char* pimage, long height, long width, char* outname );
	void test_output_bmp_s( QRCode_MEMORY_BLOCK* qr, char* outname );
	void test_grid( int* grid, int size );

	//************ QR_Code *************************
	/**************************************************************************/
	// QRcode Decoder Head File
	// v1004(150520)
	/**************************************************************************/

	//========================================================
	// Function Name :  QR_GetErrorNum
	// Syntax : long QR_GetErrorNum(unsigned char *qr_working_memory);
	// Purpose :  get number of error code
	// Parameters : unsigned char *qr_working_memory: working memory pointer
	// Return : error code number
	//========================================================
	long QR_GetErrorNum( unsigned char *qr_working_memory );

	//========================================================
	// Function Name :  QR_GetVersion
	// Syntax : long QR_GetVersion(unsigned char *qr_working_memory);
	// Purpose :  get version of QRcode-pattern
	// Parameters : unsigned char *qr_working_memory: working memory pointer
	// Return : version of QRcode-pattern
	//========================================================
	//long QR_GetVersion( unsigned char *qr_working_memory );	//Version of QRcode

	//========================================================
	// Function Name :  QR_GetECCLevel
	// Syntax : long QR_GetECCLevel(unsigned char *qr_working_memory);
	// Purpose :  get Ecc-level of QRcode-pattern
	// Parameters : unsigned char *qr_working_memory: working memory pointer
	// Return : ECC-level of QRcode-pattern
	//========================================================
	long QR_GetECCLevel( unsigned char *qr_working_memory );

	//========================================================
	// Function Name :  QR_GetWorkingMemorySize
	// Syntax : long QR_GetWorkingMemorySize(void);
	// Purpose :  get working memory size
	// Parameters : none
	// Return : size of working memory
	//========================================================
	long QR_GetWorkingMemorySize( void );

	//========================================================
	// Function Name :  QR_Init
	// Syntax : long QR_Init(unsigned char *qr_working_memory,long width, long height,unsigned char// image, unsigned char image_type, unsigned char* resultstream);
	// Purpose : kernel initial process
	// Parameters : unsigned char *qr_working_memory : working memory pointer
	//              long width : width of image(pixels)
	//              long height : height of image(pixels)
	//              unsigned char* image : image data pointer
	//              unsigned char image_type : reference "Image Type"
	//              unsigned char* resultstream : decode result
	// Return : 0 is ok; other vaule is failed. (Check error code)
	//========================================================
	long QR_Init( unsigned char *qr_working_memory, long width, long height, unsigned char image_type, unsigned char* resultstream );

	//========================================================
	// Function Name :  QR_Decode
	// Syntax : unsigned char* QR_Decode(unsigned char *qr_working_memory);
	// Purpose : main decode process of QRcode
	// Parameters : unsigned char *qr_working_memory: working memory pointer
	// Return : decode result
	//========================================================
	unsigned char* QR_Decode( unsigned char *qr_working_memory );

	//========================================================
	// Function Name :  QR_GetVersion
	// Syntax : const char* QR_Version(void);
	// Purpose :  get library version
	// Parameters : none
	// Return : version of QRcode decoder library
	//========================================================
	//const char* QR_Version( void );							//version of Decoder



	// **************** RSDecode ******************
	short calcSigmaMBM( short *sigma, short *omega, short syn[ 8 ], short npar );
	void arraycopy( short *array1, short pos1, short *array2, short pos2, short length );
	//int RsDecode( short* data, short length, short npar, short* memory );

	//************** QRCodeSymbol ***************

	static short getWidth( QRCode_MEMORY_BLOCK *qrcode );
	static short getHeight( QRCode_MEMORY_BLOCK *qrcode );
	short *getLogicalSeeds( int version );

#ifdef __cplusplus
}
#endif

#endif
