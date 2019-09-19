#include "avi_encoder_scaler_jpeg.h"

#if (defined AVI_ENCODE_SHOW_TIME && AVI_ENCODE_SHOW_TIME == 1)
const INT8U acFontHZArial01100030[14] = {	/* 0 013000*/
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x70,//01110000,
0x88,//10001000,
0x88,//10001000,
0x88,//10001000,
0x88,//10001000,
0x88,//10001000,
0x88,//10001000,
0x70,//01110000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
};

const unsigned char acFontHZArial01100031[14] = {	/* 1 013100*/
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x20,//00100000,
0x60,//01100000,
0xa0,//10100000,
0x20,//00100000,
0x20,//00100000,
0x20,//00100000,
0x20,//00100000,
0x20,//00100000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
};

const unsigned char acFontHZArial01100032[14] = {	/* 2 013200*/
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x70,//01110000,
0x88,//10001000,
0x08,//00001000,
0x08,//00001000,
0x10,//00010000,
0x20,//00100000,
0x40,//01000000,
0xf8,//11111000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
};

const unsigned char acFontHZArial01100033[14] = {	/* 3 013300*/
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x70,//01110000,
0x88,//10001000,
0x08,//00001000,
0x30,//00110000,
0x08,//00001000,
0x08,//00001000,
0x88,//10001000,
0x70,//01110000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
};

const unsigned char acFontHZArial01100034[14] = {	/* 4 013400*/
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x10,//00010000,
0x30,//00110000,
0x50,//01010000,
0x50,//01010000,
0x90,//10010000,
0xf8,//11111000,
0x10,//00010000,
0x10,//00010000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
};

const unsigned char acFontHZArial01100035[14] = {	/* 5 013500*/
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x78,//01111000,
0x40,//01000000,
0x80,//10000000,
0xf0,//11110000,
0x08,//00001000,
0x08,//00001000,
0x88,//10001000,
0x70,//01110000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
};

const unsigned char acFontHZArial01100036[14] = {	/* 6 013600*/
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x70,//01110000,
0x88,//10001000,
0x80,//10000000,
0xf0,//11110000,
0x88,//10001000,
0x88,//10001000,
0x88,//10001000,
0x70,//01110000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
};

const unsigned char acFontHZArial01100037[14] = {	/* 7 013700*/
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0xf8,//11111000,
0x10,//00010000,
0x10,//00010000,
0x20,//00100000,
0x20,//00100000,
0x40,//01000000,
0x40,//01000000,
0x40,//01000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
};

const unsigned char acFontHZArial01100038[14] = {	/* 8 013800*/
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x70,//01110000,
0x88,//10001000,
0x88,//10001000,
0x70,//01110000,
0x88,//10001000,
0x88,//10001000,
0x88,//10001000,
0x70,//01110000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
};

const unsigned char acFontHZArial01100039[14] = {	/* 9 013900*/
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x70,//01110000,
0x88,//10001000,
0x88,//10001000,
0x88,//10001000,
0x78,//01111000,
0x08,//00001000,
0x88,//10001000,
0x70,//01110000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
};
const unsigned char acFontHZArialCommon[14] = {	/* 9 013900*/
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x60,//01100000,
0x60,//01100000,
0x00,//00000000,
0x00,//00000000,
0x60,//01100000,
0x60,//01100000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
};
const unsigned char acFontHZArialDot[14] = {	/* . 012e00*/
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x60,//01100000,
0x60,//01100000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
};

const unsigned char acFontHZArialSlash[14] = {	/* / 012f00*/
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x20,//00100000,
0x20,//00100000,
0x40,//01000000,
0x40,//01000000,
0x40,//01000000,
0x40,//01000000,
0x80,//10000000,
0x80,//10000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
};


const unsigned char acFontHZArial_d[14] = {	/* / 012f00*/
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x01,//00000001,
0x01,//00000001,
0x01,//00000001,
0xff,//11111111,
0x01,//00000001,
0x81,//10000001,
0x81,//10000001,
0xff,//11111111,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
};


const unsigned char acFontHZArial_u[14] = {
0x00,//00000000,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x42,//01000010,
0x7e,//01111110,
};

const unsigned char acFontHZArial_o[14] = {
0x00,//00000000,
0x7e,//01111110,
0x42,//01000010,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x42,//01000010,
0x7e,//01111110,
};



const unsigned char acFontHZArial017Dot[19] = {
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x20,//00100000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
};

const unsigned char acFontHZArial017Slash[19] = {
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0X02,//0x08,//00001000,
0X04,//0x10,//00010000,
0X04,//0x10,//00010000,
0X04,//0x10,//00010000,
0X08,//0x20,//00100000,
0X08,//0x20,//00100000,
0X08,//0x20,//00100000,
0X10,//0x40,//01000000,
0X10,//0x40,//01000000,
0X10,//0x40,//01000000,
0X20,//0x80,//10000000,
0X20,//0x80,//10000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
};



const unsigned char acFontHZArial017_H[19] = {
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0xff,//11111111,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
};


const unsigned char acFontHZArial017_U[19] = {
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x42,//01000010,
0x7e,//01111110,
};

const unsigned char acFontHZArial017_O[19] = {
0x00,//00000000,
0x00,//00000000,
0x7e,//01111110,
0x42,//01000010,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x42,//01000010,
0x7e,//01111110,
};



const unsigned char acFontHZArial017_L[19] = {
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x80,//10000000,
0x80,//10000000,
0x80,//10000000,
0x80,//10000000,
0x80,//10000000,
0x80,//10000000,
0x80,//10000000,
0x80,//10000000,
0x80,//10000000,
0x80,//10000000,
0x80,//10000000,
0x80,//10000000,
0x80,//10000000,
0x80,//10000000,
0xff,//11111111,
};

const unsigned char acFontHZArial017_M[19] = {
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x81,//10000001,
0xc3,//11000011,
0xa5,//10100101,
0xa5,//10100101,
0xa5,//10100101,
0x99,//10011001,
0x99,//10011001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
};



const unsigned char acFontHZArial017_E[19] = {
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0xff,//11111111,
0x80,//10000000,
0x80,//10000000,
0x80,//10000000,
0x80,//10000000,
0x80,//10000000,
0xff,//11111111,
0x80,//10000000,
0x80,//10000000,
0x80,//10000000,
0x80,//10000000,
0x80,//10000000,
0x80,//10000000,
0x80,//10000000,
0xff,//11111111,
};

const unsigned char acFontHZArial017_G[19] = {
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x1c,//00011100,
0x22,//00100010,
0x41,//01000001,
0x80,//10000000,
0x80,//10000000,
0x80,//10000000,
0xff,//11111111,
0x85,//10000101,
0x82,//10000010,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x42,//01000010,
0x24,//00100100,
0x18,//00011000,
};


const unsigned char acFontHZArial017_Y[19] = {
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x42,//01000010,
0x42,//01000010,
0x42,//01000010,
0x3c,//00111100,
0x10,//00010000,
0x10,//00010000,
0x10,//00010000,
0x10,//00010000,
0x10,//00010000,
0x10,//00010000,
0x10,//00010000,
0x10,//00010000,
};


const unsigned char acFontHZArial017_B[19] = {
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x80,//10000000,
0x80,//10000000,
0x80,//10000000,
0x80,//10000000,
0x80,//10000000,
0x80,//10000000,
0xf8,//11111000,
0x84,//10000100,
0x82,//10000010,
0x81,//10000001,
0x81,//10000001,
0x81,//10000001,
0x82,//10000010,
0x84,//10000100,
0xf8,//11111000,
};


const unsigned char acFontHZArial01700030[19] = {
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x1c,//00011100,
0x22,//00100010,
0x41,//01000001,
0x41,//01000001,
0x41,//01000001,
0x41,//01000001,
0x41,//01000001,
0x41,//01000001,
0x41,//01000001,
0x41,//01000001,
0x22,//00100010,
0x1c,//00011100,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
};

const unsigned char acFontHZArial01700031[19] = {
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x04,//00000100,
0x0c,//00001100,
0x14,//00010100,
0x24,//00100100,
0x04,//00000100,
0x04,//00000100,
0x04,//00000100,
0x04,//00000100,
0x04,//00000100,
0x04,//00000100,
0x04,//00000100,
0x04,//00000100,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
};

const unsigned char acFontHZArial01700032[19] = {
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x1e,//00011110,
0x22,//00100010,
0x41,//01000001,
0x01,//00000001,
0x01,//00000001,
0x02,//00000010,
0x02,//00000010,
0x04,//00000100,
0x08,//00001000,
0x10,//00010000,
0x20,//00100000,
0x7f,//01111111,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
};

const unsigned char acFontHZArial01700033[19] = {
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x1c,//00011100,
0x22,//00100010,
0x22,//01000010,
0x02,//00000010,
0x04,//00000100,
0x1c,//00011100,
0x02,//00000010,
0x01,//00000001,
0x01,//00000001,
0x41,//01000001,
0x62,//01100010,
0x1c,//00011100,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
};

const unsigned char acFontHZArial01700034[19] = {
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x04,//00000100,
0x0c,//00001100,
0x14,//00010100,
0x14,//00010100,
0x24,//00100100,
0x24,//00100100,
0x44,//01000100,
0x84,//10000100,
0xff,//11111111,
0x04,//00000100,
0x04,//00000100,
0x04,//00000100,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
};

const unsigned char acFontHZArial01700035[19] = {
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x3f,//00111111,
0x20,//00100000,
0x20,//00100000,
0x40,//01000000,
0x78,//01111100,
0x42,//01000010,
0x01,//00000001,
0x01,//00000001,
0x01,//00000001,
0x41,//01000001,
0x22,//00100010,
0x1c,//00011100,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
};

const unsigned char acFontHZArial01700036[19] = {
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x1c,//00011100,
0x23,//00100011,
0x41,//01000001,
0x40,//01000000,
0x5c,//01011100,
0x62,//01100010,
0x41,//01000001,
0x41,//01000001,
0x41,//01000001,
0x41,//01000001,
0x22,//00100010,
0x1c,//00011100,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
};

const unsigned char acFontHZArial01700037[19] = {
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x7f,//01111111,
0x02,//00000010,
0x02,//00000010,
0x04,//00000100,
0x04,//00000100,
0x08,//00001000,
0x08,//00001000,
0x08,//00001000,
0x08,//00001000,
0x10,//00010000,
0x10,//00010000,
0x10,//00010000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
};

const unsigned char acFontHZArial01700038[19] = {
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x1c,//00011100,
0x22,//00100010,
0x21,//01000001,
0x21,//01000001,
0x22,//00100010,
0x1c,//00011100,
0x22,//00100010,
0x41,//01000001,
0x41,//01000001,
0x41,//01000001,
0x22,//00100010,
0x1c,//00011100,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
};

const unsigned char acFontHZArial01700039[19] = {
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x1c,//00011100,
0x22,//00100010,
0x21,//01000001,
0x21,//01000001,
0x21,//01000001,
0x21,//01000001,
0x23,//00100011,
0x1d,//00011101,
0x01,//00000001,
0x41,//01000001,
0x22,//00100010,
0x3c,//00111100,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
};

const unsigned char acFontHZArial017Comma[19] = {
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x08,//0x20,//00100000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x08,//0x20,//00100000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
0x00,//00000000,
};
#endif
