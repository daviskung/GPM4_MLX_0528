#ifndef __GP_GC1014_CALIBRATION_H__
#define __GP_GC1014_CALIBRATION_H__

#include "..\gp_LensDef.h"


static const short g_ob[15] =
{
	0, // obautoen
	0, // ob_type
	0, // obHOffset
	0, // obVOffset

	1, // obmanuen
	8, // maunob

	1, // wboffseten
	0, // wbo_r
	0, // wbo_gr
	0, // wbo_gb
	0, // wbo_b

	1, // badpixen
	160, // bprthr
	160, // bpgthr
	160, // bpbthr
};

static const unsigned int g_gamma[] =
{
0x000000, 0x000000, 0x044401, 0x111405, 0x111109, 0x11110d, 0x111111, 0x111115,
0x041119, 0x04441d, 0x110421, 0x011124, 0x044428, 0x01112b, 0x10442f, 0x041132,
0x111036, 0x104439, 0x04413c, 0x04413f, 0x041142, 0x041145, 0x041148, 0x10414b,
0x10414e, 0x010451, 0x041054, 0x104156, 0x010459, 0x04105c, 0x01015e, 0x041061,
0x010463, 0x104066, 0x041068, 0x01046a, 0x00416c, 0x10406f, 0x101071, 0x041073,
0x040475, 0x040477, 0x040479, 0x10047b, 0x10107d, 0x00107f, 0x004081, 0x010083,
0x040184, 0x101086, 0x004088, 0x010189, 0x10048b, 0x00408d, 0x04018e, 0x001090,
0x040191, 0x001093, 0x040095, 0x001096, 0x040197, 0x004099, 0x10049a, 0x01009c,
0x00109d, 0x10019e, 0x0100a0, 0x0010a1, 0x1001a2, 0x0400a4, 0x0040a5, 0x0004a6,
0x1001a7, 0x0100a9, 0x0040aa, 0x0004ab, 0x1001ac, 0x0400ae, 0x0100af, 0x0010b0,
0x0004b1, 0x1001b2, 0x0400b4, 0x0100b5, 0x0010b6, 0x0004b7, 0x1000b9, 0x0400ba,
0x0040bb, 0x0010bc, 0x0001bd, 0x0400bf, 0x0100c0, 0x0010c1, 0x0001c2, 0x0400c4,
0x0040c5, 0x0004c6, 0x0400c8, 0x0040c9, 0x0004ca, 0x0400cc, 0x0040cd, 0x1001ce,
0x0100d0, 0x0004d1, 0x0100d3, 0x0010d4, 0x0100d6, 0x1004d7, 0x0100d9, 0x1004da,
0x0040dc, 0x0100de, 0x1004df, 0x0010e1, 0x0100e3, 0x0401e4, 0x0404e6, 0x1010e8,
0x0010ea, 0x0040ec, 0x0040ee, 0x0040f0, 0x0040f2, 0x0040f4, 0x1010f6, 0x0410f8
/*
	//H22 ,此Table清晰度較好
0x04510d, 0x051112, 0x111417, 0x14451b, 0x111120, 0x044425, 0x111129, 0x05112d,
0x044432, 0x044436, 0x04443a, 0x04443e, 0x044442, 0x110446, 0x111149, 0x04444d,
0x111051, 0x044154, 0x111058, 0x10445b, 0x04415e, 0x011062, 0x011065, 0x010468,
0x01046b, 0x01046e, 0x041071, 0x041074, 0x104176, 0x010479, 0x04107c, 0x00417e,
0x041081, 0x010183, 0x101086, 0x040488, 0x01018a, 0x10408d, 0x10108f, 0x041091,
0x040493, 0x040495, 0x040497, 0x040499, 0x10109b, 0x00109d, 0x00409f, 0x0101a0,
0x1004a2, 0x0010a4, 0x0100a6, 0x1004a7, 0x0040a9, 0x1001aa, 0x0040ac, 0x1001ad,
0x0100af, 0x0004b0, 0x0400b2, 0x0040b3, 0x0004b4, 0x0400b6, 0x0100b7, 0x0010b8,
0x0004b9, 0x1000bb, 0x0400bc, 0x0100bd, 0x0040be, 0x0010bf, 0x0004c0, 0x0004c1,
0x0001c2, 0x0001c3, 0x0000c5, 0x0000c6, 0x0000c7, 0x0000c8, 0x0000c9, 0x0000ca,
0x0000cb, 0x0000cc, 0x0000cd, 0x0000ce, 0x0000cf, 0x0000d0, 0x0001d0, 0x0001d1,
0x0001d2, 0x0001d3, 0x0001d4, 0x0001d5, 0x0001d6, 0x0001d7, 0x0001d8, 0x0001d9,
0x0000db, 0x0000dc, 0x1000dd, 0x0400de, 0x0400df, 0x0100e0, 0x0040e1, 0x0010e2,
0x0004e3, 0x1000e5, 0x0100e6, 0x0040e7, 0x0004e8, 0x1000ea, 0x0100eb, 0x0004ec,
0x0400ee, 0x0040ef, 0x1001f0, 0x0040f2, 0x1004f3, 0x0040f5, 0x0401f6, 0x0010f8,
0x0100fa, 0x0404fb, 0x1010fd, 0x0000ff, 0x0000ff, 0x0000ff, 0x0000ff, 0x0000ff
*/
//u2.2016.01.05 H42 此對比度變高但暗處太暗
/*
	0x000000, 0x000000, 0x000000, 0x000000, 0x110000, 0x111102, 0x045106, 0x04440b,
	0x04440f, 0x044413, 0x044417, 0x04441b, 0x11041f, 0x111122, 0x044126, 0x10442a,
	0x01112d, 0x104431, 0x041134, 0x110438, 0x04443b, 0x04113e, 0x011042, 0x010445,
	0x110448, 0x11044b, 0x01044e, 0x010451, 0x011054, 0x041057, 0x104159, 0x01045c,
	0x04105f, 0x104161, 0x010464, 0x104067, 0x010469, 0x10416b, 0x04106e, 0x010170,
	0x104073, 0x041075, 0x040477, 0x010179, 0x01017b, 0x00407e, 0x004080, 0x004082,
	0x004084, 0x004086, 0x004088, 0x010189, 0x01018b, 0x04048d, 0x10108f, 0x004091,
	0x010093, 0x040194, 0x001096, 0x004098, 0x040199, 0x00109b, 0x01009d, 0x10049e,
	0x0100a0, 0x1004a1, 0x0040a3, 0x1004a4, 0x0100a6, 0x0004a7, 0x0100a9, 0x0010aa,
	0x0400ac, 0x0040ad, 0x1001ae, 0x0100b0, 0x0010b1, 0x0401b2, 0x0040b4, 0x0004b5,
	0x0400b7, 0x0040b8, 0x0004b9, 0x0400bb, 0x0040bc, 0x0004bd, 0x0400bf, 0x0040c0,
	0x0004c1, 0x1001c2, 0x0100c4, 0x0010c5, 0x1001c6, 0x0100c8, 0x0004c9, 0x0400cb,
	0x0040cc, 0x0004cd, 0x0400cf, 0x0010d0, 0x1001d1, 0x0040d3, 0x1004d4, 0x0100d6,
	0x0004d7, 0x0100d9, 0x0010da, 0x0100dc, 0x0004dd, 0x0100df, 0x1004e0, 0x0040e2,
	0x0401e3, 0x0010e5, 0x0040e7, 0x0401e8, 0x1004ea, 0x0040ec, 0x0100ee, 0x0401ef,
	0x0404f1, 0x1010f3, 0x1010f5, 0x0040f7, 0x0040f9, 0x0040fb, 0x0040fd, 0x0000ff,
*/
};


static const unsigned short g_wb_gain[61][2] =
{	/*2000K~8000K*/
	{27 ,133},
	{31 ,131},
	{34 ,129},
	{38 ,126},
	{41 ,124},
	{43 ,122},
	{46 ,120},
	{49 ,118},
	{51 ,117},
	{53 ,115},
	{55 ,113},
	{57 ,111},
	{59 ,109},
	{61 ,108},
	{62 ,106},
	{64 ,104},
	{65 ,103},
	{66 ,101},
	{67 ,100},
	{68 ,98 },
	{69 ,97 },//40
	{70 ,95 },
	{71 ,94 },
	{72 ,92 },
	{73 ,91 },
	{74 ,90 },
	{74 ,89 },
	{75 ,87 },
	{76 ,86 },
	{77 ,85 },
	{78 ,84 },//50
	{78 ,83 },
	{79 ,82 },
	{80 ,81 },
	{81 ,80 },
	{82 ,79 },
	{83 ,78 },
	{84 ,77 },
	{85 ,76 },
	{86 ,76 },
	{88 ,75 },//60
	{89 ,74 },
	{90 ,74 },
	{92 ,73 },
	{94 ,72 },
	{96 ,72 },
	{98 ,71 },
	{100,71 },
	{102,70 },
	{104,70 },
	{107,70 },//70
	{110,69 },
	{113,69 },
	{116,69 },
	{119,68 },
	{123,68 },
	{127,68 },
	{131,68 },
	{135,68 },
	{139,68 },
	{144,68 } //80

};

static const short g_awb_thr[31] =
{
	200, // awbwinthr

	0*64, // sindata
	1*64, // cosdata

	 30, // Ythr0
	 90, // Ythr1
	140, // Ythr2
	200, // Ythr3

	// wb thr
	-4, //UL1N1
	 4, //UL1P1
	-3, //VL1N1
	 3, //VL1P1

	 -5, //UL1N2
	  5, //UL1P2
	 -4, //VL1N2
	  4, //VL1P2

	 -6, //UL1N3
	  6, //UL1P3
	 -4, //VL1N3
	  4, //VL1P3
	// without wb thr
	-21, // UL1N1
	-2, // UL1P1
	-14, // VL1N1
	 20, // VL1P1

	-31, //UL1N2
	-2, //UL1P2
	-22, //VL1N2
	 29, // VL1P2

	-54, // UL1N3
	-2, //UL1P3
	-32, // VL1N3
	 35, //VL1P3
};


static const short g_color_matrix[9] =
{
	(short)(1.35710021313892340000*64),
    (short)(-0.5530298693042127200*64),
    (short)(0.19592965616528946000*64),
    (short)(-0.1364870632631542100*64),
    (short)(1.26799618774913840000*64),
    (short)(-0.1315091244859840100*64),
    (short)(0.11474140695766549000*64),
    (short)(-1.0694119814917977000*64),
    (short)(1.95467057453413220000*64)

};


const INT8U LiTable_rgb[48]=
{	//R*16,Gb/Gr*16,B*16
	0x1e,
	0x2e,
	0x3e,
	0x4e,
	0x5e,
	0x6e,
	0x7e,
	0x8e,
	0x9e,
	0xae,
	0xbe,
	0xce,
	0xde,
	0xee,
	0xfe,
	0xff,
	0x0f,
	0x1f,
	0x2f,
	0x3f,
	0x4f,
	0x5f,
	0x6f,
	0x7f,
	0x8f,
	0x9f,
	0xaf,
	0xbf,
	0xcf,
	0xdf,
	0xef,
	0xff,
	0x07,
	0x17,
	0x27,
	0x37,
	0x47,
	0x57,
	0x67,
	0x77,
	0x87,
	0x97,
	0xa7,
	0xb7,
	0xc7,
	0xd7,
	0xe7,
	0xf7
};

const INT16U MaxTan8[32]=
{
	0x0C0,
	0x155,
	0x154,
	0x0C1,
	0x0BF,
	0x156,
	0x153,
	0x0C2,
	0x0BF,
	0x154,
	0x153,
	0x0C0,
	0x0BE,
	0x155,
	0x152,
	0x0C1,
	0x0BE,
	0x153,
	0x152,
	0x0BF,
	0x0BD,
	0x154,
	0x151,
	0x0C0,
	0x0BD,
	0x152,
	0x151,
	0x0BE,
	0x0BC,
	0x153,
	0x150,
	0x0BF
};

const INT16U Slope4[16]=
{
	0x155,
	0x154,
	0x153,
	0x152,
	0x151,
	0x152,
	0x153,
	0x154,
	0x155,
	0x154,
	0x153,
	0x152,
	0x151,
	0x152,
	0x153,
	0x154
};

const INT16U CLPoint[8]=		//Sensor Center is Weight/2, Hight/2 for RGB
{
	#if 0	//for VGA
		//R Center
		0x140,//X=320
		0x0F0,//Y=
		//Gr Center
		0x13F,//
		0x0F0,//
		0x140,
		0x0EF,
		0x13F,
		0x0EF
	#else	//For XQGA
		0x400,
		0x300,
		0x400,
		0x300,
		0x400,
		0x300,
		0x400,
		0x300
	#endif
};

const INT16U Radius_File_0[512]=
{
	0x100,
	0x101,
	0x102,
	0x103,
	0x104,
	0x105,
	0x106,
	0x107,
	0x102,
	0x103,
	0x104,
	0x105,
	0x106,
	0x107,
	0x108,
	0x109,
	0x104,
	0x105,
	0x106,
	0x107,
	0x108,
	0x109,
	0x10a,
	0x10b,
	0x106,
	0x107,
	0x108,
	0x109,
	0x10a,
	0x10b,
	0x10c,
	0x10d,
	0x108,
	0x109,
	0x10a,
	0x10b,
	0x10c,
	0x10d,
	0x10e,
	0x10f,
	0x10a,
	0x10b,
	0x10c,
	0x10d,
	0x10e,
	0x10f,
	0x110,
	0x111,
	0x10c,
	0x10d,
	0x10e,
	0x10f,
	0x110,
	0x111,
	0x112,
	0x113,
	0x10e,
	0x10f,
	0x110,
	0x111,
	0x112,
	0x113,
	0x114,
	0x115,
	0x110,
	0x111,
	0x112,
	0x113,
	0x114,
	0x115,
	0x116,
	0x117,
	0x112,
	0x113,
	0x114,
	0x115,
	0x116,
	0x117,
	0x118,
	0x119,
	0x114,
	0x115,
	0x116,
	0x117,
	0x118,
	0x119,
	0x11a,
	0x11b,
	0x116,
	0x117,
	0x118,
	0x119,
	0x11a,
	0x11b,
	0x11c,
	0x11d,
	0x118,
	0x119,
	0x11a,
	0x11b,
	0x11c,
	0x11d,
	0x11e,
	0x11f,
	0x11a,
	0x11b,
	0x11c,
	0x11d,
	0x11e,
	0x11f,
	0x120,
	0x121,
	0x11c,
	0x11d,
	0x11e,
	0x11f,
	0x120,
	0x121,
	0x122,
	0x123,
	0x11e,
	0x11f,
	0x120,
	0x121,
	0x122,
	0x123,
	0x124,
	0x125,
	0x102,
	0x103,
	0x104,
	0x105,
	0x106,
	0x107,
	0x108,
	0x109,
	0x104,
	0x105,
	0x106,
	0x107,
	0x108,
	0x109,
	0x10a,
	0x10b,
	0x106,
	0x107,
	0x108,
	0x109,
	0x10a,
	0x10b,
	0x10c,
	0x10d,
	0x108,
	0x109,
	0x10a,
	0x10b,
	0x10c,
	0x10d,
	0x10e,
	0x10f,
	0x10a,
	0x10b,
	0x10c,
	0x10d,
	0x10e,
	0x10f,
	0x110,
	0x111,
	0x10c,
	0x10d,
	0x10e,
	0x10f,
	0x110,
	0x111,
	0x112,
	0x113,
	0x10e,
	0x10f,
	0x110,
	0x111,
	0x112,
	0x113,
	0x114,
	0x115,
	0x110,
	0x111,
	0x112,
	0x113,
	0x114,
	0x115,
	0x116,
	0x117,
	0x112,
	0x113,
	0x114,
	0x115,
	0x116,
	0x117,
	0x118,
	0x119,
	0x114,
	0x115,
	0x116,
	0x117,
	0x118,
	0x119,
	0x11a,
	0x11b,
	0x116,
	0x117,
	0x118,
	0x119,
	0x11a,
	0x11b,
	0x11c,
	0x11d,
	0x118,
	0x119,
	0x11a,
	0x11b,
	0x11c,
	0x11d,
	0x11e,
	0x11f,
	0x11a,
	0x11b,
	0x11c,
	0x11d,
	0x11e,
	0x11f,
	0x120,
	0x121,
	0x11c,
	0x11d,
	0x11e,
	0x11f,
	0x120,
	0x121,
	0x122,
	0x123,
	0x11e,
	0x11f,
	0x120,
	0x121,
	0x122,
	0x123,
	0x124,
	0x125,
	0x120,
	0x121,
	0x122,
	0x123,
	0x124,
	0x125,
	0x126,
	0x127,
	0x104,
	0x105,
	0x106,
	0x107,
	0x108,
	0x109,
	0x10a,
	0x10b,
	0x106,
	0x107,
	0x108,
	0x109,
	0x10a,
	0x10b,
	0x10c,
	0x10d,
	0x108,
	0x109,
	0x10a,
	0x10b,
	0x10c,
	0x10d,
	0x10e,
	0x10f,
	0x10a,
	0x10b,
	0x10c,
	0x10d,
	0x10e,
	0x10f,
	0x110,
	0x111,
	0x10c,
	0x10d,
	0x10e,
	0x10f,
	0x110,
	0x111,
	0x112,
	0x113,
	0x10e,
	0x10f,
	0x110,
	0x111,
	0x112,
	0x113,
	0x114,
	0x115,
	0x110,
	0x111,
	0x112,
	0x113,
	0x114,
	0x115,
	0x116,
	0x117,
	0x112,
	0x113,
	0x114,
	0x115,
	0x116,
	0x117,
	0x118,
	0x119,
	0x114,
	0x115,
	0x116,
	0x117,
	0x118,
	0x119,
	0x11a,
	0x11b,
	0x116,
	0x117,
	0x118,
	0x119,
	0x11a,
	0x11b,
	0x11c,
	0x11d,
	0x118,
	0x119,
	0x11a,
	0x11b,
	0x11c,
	0x11d,
	0x11e,
	0x11f,
	0x11a,
	0x11b,
	0x11c,
	0x11d,
	0x11e,
	0x11f,
	0x120,
	0x121,
	0x11c,
	0x11d,
	0x11e,
	0x11f,
	0x120,
	0x121,
	0x122,
	0x123,
	0x11e,
	0x11f,
	0x120,
	0x121,
	0x122,
	0x123,
	0x124,
	0x125,
	0x120,
	0x121,
	0x122,
	0x123,
	0x124,
	0x125,
	0x126,
	0x127,
	0x122,
	0x123,
	0x124,
	0x125,
	0x126,
	0x127,
	0x128,
	0x129,
	0x106,
	0x107,
	0x108,
	0x109,
	0x10a,
	0x10b,
	0x10c,
	0x10d,
	0x108,
	0x109,
	0x10a,
	0x10b,
	0x10c,
	0x10d,
	0x10e,
	0x10f,
	0x10a,
	0x10b,
	0x10c,
	0x10d,
	0x10e,
	0x10f,
	0x110,
	0x111,
	0x10c,
	0x10d,
	0x10e,
	0x10f,
	0x110,
	0x111,
	0x112,
	0x113,
	0x10e,
	0x10f,
	0x110,
	0x111,
	0x112,
	0x113,
	0x114,
	0x115,
	0x110,
	0x111,
	0x112,
	0x113,
	0x114,
	0x115,
	0x116,
	0x117,
	0x112,
	0x113,
	0x114,
	0x115,
	0x116,
	0x117,
	0x118,
	0x119,
	0x114,
	0x115,
	0x116,
	0x117,
	0x118,
	0x119,
	0x11a,
	0x11b,
	0x116,
	0x117,
	0x118,
	0x119,
	0x11a,
	0x11b,
	0x11c,
	0x11d,
	0x118,
	0x119,
	0x11a,
	0x11b,
	0x11c,
	0x11d,
	0x11e,
	0x11f,
	0x11a,
	0x11b,
	0x11c,
	0x11d,
	0x11e,
	0x11f,
	0x120,
	0x121,
	0x11c,
	0x11d,
	0x11e,
	0x11f,
	0x120,
	0x121,
	0x122,
	0x123,
	0x11e,
	0x11f,
	0x120,
	0x121,
	0x122,
	0x123,
	0x124,
	0x125,
	0x120,
	0x121,
	0x122,
	0x123,
	0x124,
	0x125,
	0x126,
	0x127,
	0x122,
	0x123,
	0x124,
	0x125,
	0x126,
	0x127,
	0x128,
	0x129,
	0x124,
	0x125,
	0x126,
	0x127,
	0x128,
	0x129,
	0x12a,
	0x12b
};

const INT16U Radius_File_1[512]=
{
	0x101,
	0x102,
	0x103,
	0x104,
	0x105,
	0x106,
	0x107,
	0x108,
	0x103,
	0x104,
	0x105,
	0x106,
	0x107,
	0x108,
	0x109,
	0x10a,
	0x105,
	0x106,
	0x107,
	0x108,
	0x109,
	0x10a,
	0x10b,
	0x10c,
	0x107,
	0x108,
	0x109,
	0x10a,
	0x10b,
	0x10c,
	0x10d,
	0x10e,
	0x109,
	0x10a,
	0x10b,
	0x10c,
	0x10d,
	0x10e,
	0x10f,
	0x110,
	0x10b,
	0x10c,
	0x10d,
	0x10e,
	0x10f,
	0x110,
	0x111,
	0x112,
	0x10d,
	0x10e,
	0x10f,
	0x110,
	0x111,
	0x112,
	0x113,
	0x114,
	0x10f,
	0x110,
	0x111,
	0x112,
	0x113,
	0x114,
	0x115,
	0x116,
	0x111,
	0x112,
	0x113,
	0x114,
	0x115,
	0x116,
	0x117,
	0x118,
	0x113,
	0x114,
	0x115,
	0x116,
	0x117,
	0x118,
	0x119,
	0x11a,
	0x115,
	0x116,
	0x117,
	0x118,
	0x119,
	0x11a,
	0x11b,
	0x11c,
	0x117,
	0x118,
	0x119,
	0x11a,
	0x11b,
	0x11c,
	0x11d,
	0x11e,
	0x119,
	0x11a,
	0x11b,
	0x11c,
	0x11d,
	0x11e,
	0x11f,
	0x120,
	0x11b,
	0x11c,
	0x11d,
	0x11e,
	0x11f,
	0x120,
	0x121,
	0x122,
	0x11d,
	0x11e,
	0x11f,
	0x120,
	0x121,
	0x122,
	0x123,
	0x124,
	0x11f,
	0x120,
	0x121,
	0x122,
	0x123,
	0x124,
	0x125,
	0x126,
	0x103,
	0x104,
	0x105,
	0x106,
	0x107,
	0x108,
	0x109,
	0x10a,
	0x105,
	0x106,
	0x107,
	0x108,
	0x109,
	0x10a,
	0x10b,
	0x10c,
	0x107,
	0x108,
	0x109,
	0x10a,
	0x10b,
	0x10c,
	0x10d,
	0x10e,
	0x109,
	0x10a,
	0x10b,
	0x10c,
	0x10d,
	0x10e,
	0x10f,
	0x110,
	0x10b,
	0x10c,
	0x10d,
	0x10e,
	0x10f,
	0x110,
	0x111,
	0x112,
	0x10d,
	0x10e,
	0x10f,
	0x110,
	0x111,
	0x112,
	0x113,
	0x114,
	0x10f,
	0x110,
	0x111,
	0x112,
	0x113,
	0x114,
	0x115,
	0x116,
	0x111,
	0x112,
	0x113,
	0x114,
	0x115,
	0x116,
	0x117,
	0x118,
	0x113,
	0x114,
	0x115,
	0x116,
	0x117,
	0x118,
	0x119,
	0x11a,
	0x115,
	0x116,
	0x117,
	0x118,
	0x119,
	0x11a,
	0x11b,
	0x11c,
	0x117,
	0x118,
	0x119,
	0x11a,
	0x11b,
	0x11c,
	0x11d,
	0x11e,
	0x119,
	0x11a,
	0x11b,
	0x11c,
	0x11d,
	0x11e,
	0x11f,
	0x120,
	0x11b,
	0x11c,
	0x11d,
	0x11e,
	0x11f,
	0x120,
	0x121,
	0x122,
	0x11d,
	0x11e,
	0x11f,
	0x120,
	0x121,
	0x122,
	0x123,
	0x124,
	0x11f,
	0x120,
	0x121,
	0x122,
	0x123,
	0x124,
	0x125,
	0x126,
	0x121,
	0x122,
	0x123,
	0x124,
	0x125,
	0x126,
	0x127,
	0x128,
	0x105,
	0x106,
	0x107,
	0x108,
	0x109,
	0x10a,
	0x10b,
	0x10c,
	0x107,
	0x108,
	0x109,
	0x10a,
	0x10b,
	0x10c,
	0x10d,
	0x10e,
	0x109,
	0x10a,
	0x10b,
	0x10c,
	0x10d,
	0x10e,
	0x10f,
	0x110,
	0x10b,
	0x10c,
	0x10d,
	0x10e,
	0x10f,
	0x110,
	0x111,
	0x112,
	0x10d,
	0x10e,
	0x10f,
	0x110,
	0x111,
	0x112,
	0x113,
	0x114,
	0x10f,
	0x110,
	0x111,
	0x112,
	0x113,
	0x114,
	0x115,
	0x116,
	0x111,
	0x112,
	0x113,
	0x114,
	0x115,
	0x116,
	0x117,
	0x118,
	0x113,
	0x114,
	0x115,
	0x116,
	0x117,
	0x118,
	0x119,
	0x11a,
	0x115,
	0x116,
	0x117,
	0x118,
	0x119,
	0x11a,
	0x11b,
	0x11c,
	0x117,
	0x118,
	0x119,
	0x11a,
	0x11b,
	0x11c,
	0x11d,
	0x11e,
	0x119,
	0x11a,
	0x11b,
	0x11c,
	0x11d,
	0x11e,
	0x11f,
	0x120,
	0x11b,
	0x11c,
	0x11d,
	0x11e,
	0x11f,
	0x120,
	0x121,
	0x122,
	0x11d,
	0x11e,
	0x11f,
	0x120,
	0x121,
	0x122,
	0x123,
	0x124,
	0x11f,
	0x120,
	0x121,
	0x122,
	0x123,
	0x124,
	0x125,
	0x126,
	0x121,
	0x122,
	0x123,
	0x124,
	0x125,
	0x126,
	0x127,
	0x128,
	0x123,
	0x124,
	0x125,
	0x126,
	0x127,
	0x128,
	0x129,
	0x12a,
	0x107,
	0x108,
	0x109,
	0x10a,
	0x10b,
	0x10c,
	0x10d,
	0x10e,
	0x109,
	0x10a,
	0x10b,
	0x10c,
	0x10d,
	0x10e,
	0x10f,
	0x110,
	0x10b,
	0x10c,
	0x10d,
	0x10e,
	0x10f,
	0x110,
	0x111,
	0x112,
	0x10d,
	0x10e,
	0x10f,
	0x110,
	0x111,
	0x112,
	0x113,
	0x114,
	0x10f,
	0x110,
	0x111,
	0x112,
	0x113,
	0x114,
	0x115,
	0x116,
	0x111,
	0x112,
	0x113,
	0x114,
	0x115,
	0x116,
	0x117,
	0x118,
	0x113,
	0x114,
	0x115,
	0x116,
	0x117,
	0x118,
	0x119,
	0x11a,
	0x115,
	0x116,
	0x117,
	0x118,
	0x119,
	0x11a,
	0x11b,
	0x11c,
	0x117,
	0x118,
	0x119,
	0x11a,
	0x11b,
	0x11c,
	0x11d,
	0x11e,
	0x119,
	0x11a,
	0x11b,
	0x11c,
	0x11d,
	0x11e,
	0x11f,
	0x120,
	0x11b,
	0x11c,
	0x11d,
	0x11e,
	0x11f,
	0x120,
	0x121,
	0x122,
	0x11d,
	0x11e,
	0x11f,
	0x120,
	0x121,
	0x122,
	0x123,
	0x124,
	0x11f,
	0x120,
	0x121,
	0x122,
	0x123,
	0x124,
	0x125,
	0x126,
	0x121,
	0x122,
	0x123,
	0x124,
	0x125,
	0x126,
	0x127,
	0x128,
	0x123,
	0x124,
	0x125,
	0x126,
	0x127,
	0x128,
	0x129,
	0x12a,
	0x125,
	0x126,
	0x127,
	0x128,
	0x129,
	0x12a,
	0x12b,
	0x12c
};



const gpCisCali_t g_cali = {
	.ob = (short *)g_ob,
	.linearity = 0,
	.radius0 = (INT16U *) Radius_File_0,
	.radius1 = (INT16U *) Radius_File_1,
	.clpoint = (INT16U *) CLPoint,
	.maxtan = (INT16U *) MaxTan8,
	.slope = (INT16U *) Slope4,
	.segR = 20,                         /*0x194[4:0]*/
	.gamma = (unsigned int *)g_gamma,
	.awb_thr = (signed short *)g_awb_thr,
	.wb_gain = (unsigned short (*)[2])g_wb_gain,
	.color_matrix = (short *)g_color_matrix
};



#endif
