#include "sprite.h"

const INT16U ALIGN4 _r16x16_CellIdx[]={ 
	   2,	SP_ROTATE0,	   0,	SP_ZOOM0,	   0,	SP_PBANK0,	SPBLEND_DISABLE,	SP_DEPTH1,	SP_PALETTE0,	SP_VSIZE16,	SP_HSIZE16,	SPVFLIP_DISABLE,	SPHFLIP_DISABLE,	SP_COLOR16,	SP_MOSAIC0,	SPBLEND_LEVEL0,	0,	SpriteNoGroupID, SPLarge_DISABLE,  SPInterpolation_DISABLE,
	0xffff, 0xffff };
const SPRITE ALIGN4 Sprite0016_SP[]={ 
 { 0, 0, 0, 16, 16, 0, 0, 0, 0, 0, 0, 0, 0, (const INT32U *)_r16x16_CellIdx } 
};
const INT16U ALIGN4 _r8x8_CellIdx[]={ 
	   1,	SP_ROTATE0,	   0,	SP_ZOOM0,	   0,	SP_PBANK0,	SPBLEND_DISABLE,	SP_DEPTH1,	SP_PALETTE1,	SP_VSIZE8,	SP_HSIZE8,	SPVFLIP_DISABLE,	SPHFLIP_DISABLE,	SP_COLOR16,	SP_MOSAIC0,	SPBLEND_LEVEL0,	0,	SpriteNoGroupID, SPLarge_DISABLE,  SPInterpolation_DISABLE,
	0xffff, 0xffff };
const SPRITE ALIGN4 Sprite0017_SP[]={ 
 { 0, 0, 0, 8, 8, 0, 0, 0, 0, 0, 0, 0, 0, (const INT32U *)_r8x8_CellIdx } 
};
const SPRITE* SpritesPool[]={ 
	Sprite0016_SP,
	Sprite0017_SP
};
