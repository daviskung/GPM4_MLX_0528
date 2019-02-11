/*******************************************************************
	gplib_spu_drv.c
	For GPL32300A,GPL32500A, MAX Channel num is 32
  	Edited and Created by Ray

	Support 16Bits/8Bits/ADPCM/ADPCM36 modes
	Expression/Pedal/PitchBend event included

	History:
2008-05-21:
    modify can not play ADPCM36. add SPU_ClearADCPM36_Mode(uiSPUChannel)
    at SPU_PlayNote(), SPU_PlayTone and SPU_PlayDrum().

2008-08-08:
	modify the register name, make register same as programing guide.

2008-08-11
	add SPU_PlaySFX_FixCH(), it is to play the sound effect with no-envelop

2008-08-18
	1. change SPU_PlaySFX_FixCH to SPU_PlayPCM_NoEnv_FixCH
	2. fix the volume different when use SPU_PlaySFX_FixCH and play midi
	3. add SPU_Get_BeatIRQ_Enable_Flag
2008-10-27
	1. add SPU_MIDI_SetDataEntryEventCallback(void (*DataEntryEvent_callback)(void));
	2. add SPU_MIDI_GetControlEventInfo
	3. the two function is to handle the data entry event.
2008-10-28
	1. add SPU_MIdI_GetCurDt
	2. add SPU_MIDI_PlayDt
	3. add SPU_MIDI_SetPlatDtEndCallback
	4. the three function is for play A to B.
	5. modify the spu_read_a_sector_from sd_card for add mode 2
2008-10-31
	1. modify the F_GetSeqCmd
	2. modify the programing control event
2009-03-24
	1.add gmd file support
2009-05-27
	1. modify the adpcm/adpcm36 bug of conny
	2. add resume channel operation in stop channel
2010-07-26
	1. add function SPU_get_driver_version() and SPU_PlayPCM_NoEnv()
	2. add read_a_sector/read_two_sector from sdram
2010-08-18
	1. modify FindEmptyChannel() and SPU_StopChannel()
2010-10-05
	1. add protect in function read_a_sector and read_two_sector
********************************************************************/
#include "drv_l1_spu.h"
#include "gplib_spu_driver.h"

const INT32U T_BitEnable[]={
	0x00000001, 0x00000002, 0x00000004, 0x00000008,
	0x00000010, 0x00000020, 0x00000040, 0x00000080,
	0x00000100, 0x00000200, 0x00000400, 0x00000800,
	0x00001000, 0x00002000, 0x00004000, 0x00008000,
	0x00010000, 0x00020000, 0x00040000, 0x00080000,
	0x00100000, 0x00200000, 0x00400000, 0x00800000,
	0x01000000, 0x02000000, 0x04000000, 0x08000000,
	0x10000000, 0x20000000, 0x40000000, 0x80000000
};

const INT32U T_BitDisable[]={
	~0x00000001, ~0x00000002, ~0x00000004, ~0x00000008,
	~0x00000010, ~0x00000020, ~0x00000040, ~0x00000080,
	~0x00000100, ~0x00000200, ~0x00000400, ~0x00000800,
	~0x00001000, ~0x00002000, ~0x00004000, ~0x00008000,
	~0x00010000, ~0x00020000, ~0x00040000, ~0x00080000,
	~0x00100000, ~0x00200000, ~0x00400000, ~0x00800000,
	~0x01000000, ~0x02000000, ~0x04000000, ~0x08000000,
	~0x10000000, ~0x20000000, ~0x40000000, ~0x80000000
};

#if PB_8bit
const INT32U T_VarPhaseTableBottom[]={
	15, 16, 17, 18, 19, 21, 22, 23,
	25, 26, 28, 29, 31, 33, 35, 37,
	39, 42, 44, 47, 50, 53, 56, 59,
	63, 66, 70, 75, 79, 84, 89, 94,
	100, 106, 112, 119, 126, 133, 141, 150,
	159, 168, 178, 189, 200, 212, 225, 238,
	252, 267, 283, 300, 318, 337, 357, 378,
	401, 425, 450, 477, 505, 535, 567, 601,
	637, 674, 715, 757, 802, 850, 900, 954,
	1011, 1071, 1135, 1202, 1274, 1349, 1430, 1515,
	1605, 1700, 1801, 1908, 2022, 2142, 2270, 2405,
	2548, 2699, 2860, 3030, 3210, 3401, 3603, 3817,
	4044, 4285, 4540, 4810, 5096, 5399, 5720, 6060,
	6420, 6802, 7206,
//};

//const INT32U T_VarPhaseTable[]={
	D_BASEPHASE, 505, 535, 567, 601, 637, 674, 715,
	757, 802, 850, 900, 954, 1011, 1071, 1135,
	1202, 1274, 1349, 1430, 1515, 1605, 1700, 1801,
	1908, 2022, 2142, 2270, 2405, 2548, 2699, 2860,
	3030, 3210, 3401, 3603, 3817, 4044, 4285, 4540,
	4810, 5096, 5399, 5720, 6060, 6420, 6802, 7207,
	7635, 8089, 8570, 9080, 9620, 10192, 10798, 11440,
	12121, 12841, 13605, 14414, 15271, 16179, 17141, 18161,
	19240, 20385, 21597, 22881, 24242, 25683, 27210, 28828,
	30543, 32359, 34283, 36322, 38481, 40770, 43194, 45762,
	48484, 51367, 54421, 57657, 61086, 64718
};
#endif

#if PB_16bit
const INT32U T_VarPhaseTableBottom[]={		//for step 65536
	//2^12 * 2^19 / (281250)
	//2^12 * 2^19 / SERVICE_RATE
	252, 267, 283, 300, 318, 337, 357, 378 ,
	401, 425, 450, 477, 505, 535, 567, 601,
	637, 674, 715, 757, 802, 850, 900, 954,
	1011, 1071, 1135, 1202, 1274, 1349, 1430, 1515,
	1605, 1700, 1801, 1908, 2022, 2142, 2270, 2405,
	2548, 2699, 2860, 3030, 3210, 3401, 3603, 3817,
	4044, 4285, 4540, 4810, 5096, 5399, 5720, 6060,
	6420, 6802, 7206, 7635, 8089, 8570, 9080, 9620,
	10192, 10798, 11440, 12120, 12841, 13604, 14413, 15270,
	16179, 17141, 18160, 19240, 20384, 21596, 22880, 24241,
	25682, 27209, 28827, 30541, 32358, 34282, 36320, 38480,
	40768, 43192, 45761, 48482, 51365, 54419, 57655, 61083,
	64716, 68564, 72641, 76960, 81537, 86385, 91522, 96964,
	102730, 108839, 115311,
//};

//const FP32 T_VarPhaseTable[]={
//	65536*2^19/281250*2^(0/12),
//	65536*2^19/281250*2^(1/12),
	122167, 129432, 137128, 145283, 153921, 163074, 172771, 183045,
	193929, 205461, 217678, 230622, 244335, 258864, 274257, 290566,
	307843, 326149, 345543, 366090, 387859, 410922, 435357, 461244,
	488671, 517729, 548515, 581132, 615687, 652298, 691086, 732180,
	775718, 821844, 870714, 922489, 977343, 1035459, 1097031, 1162264,
	1231375, 1304597, 1382172, 1464360, 1551436, 1643689, 1741428, 1844979,
	1954687, 2070919, 2194062, 2324528, 2462751, 2609194, 2764345, 2928721,
	3102872, 3287379, 3482856, 3689958, 3909374, 4141838, 4388124, 4649056,
	4925503, 5218389, 5528690, 5857443, 6205745, 6574758, 6965713, 7379916,
	7818749, 8283676, 8776249, 9298112, 9851006, 10436778, 11057381, 11714887,
	12411490, 13149516, 13931427, 14759833, 15637498, 16567352
};
#endif

const INT32U T_TempoRefTable[]={
	36045, 35166, 34328, 33530, 32768, 32040, 31343, 30676,
	30037, 29424, 28836, 28270, 27727, 27204, 26700, 26214,
	25746, 25295, 24858, 24437, 24030, 23636, 23255, 22886,
	22528, 22181, 21845, 21519, 21203, 20896, 20597, 20307,
	20025, 19751, 19484, 19224, 18971, 18725, 18485, 18251,
	18022, 17800, 17583, 17371, 17164, 16962, 16765, 16572,
	16384, 16200, 16020, 15844, 15672, 15503, 15338, 15177,
	15019, 14864, 14712, 14564, 14418, 14275, 14135, 13998,
	13863, 13731, 13602, 13475, 13350, 13227, 13107, 12989,
	12873, 12759, 12647, 12537, 12429, 12323, 12219, 12116,
	12015, 11916, 11818, 11722, 11627, 11534, 11443, 11353,
	11264, 11177, 11091, 11006, 10923, 10841, 10760, 10680,
	10601, 10524, 10448, 10373, 10299, 10225, 10153, 10082,
	10012,  9943,  9875,  9808,  9742,  9676,  9612,  9548,
	 9485,  9423,  9362,  9302,  9242,  9183,  9125,  9068,
	 9011,  8955,  8900,  8845,  8791,  8738,  8685,  8633,
	 8582,  8531,  8481,  8432,  8383,  8334,  8286,  8239,
	 8192,  8146,  8100,  8055,  8010,  7966,  7922,  7879,
	 7836,  7793,  7752,  7710,  7669,  7629,  7588,  7549,
	 7509,  7470,  7432,  7394,  7356,  7319,  7282,  7245,
	 7209,  7173,  7138,  7102,  7068,  7033,  6999,  6965,
	 6932,  6899,  6866,  6833,  6801,  6769,  6737,  6706,
	 6675,  6644,  6614,  6584,  6554,  6524,  6495,  6465,
	 6437,  6408,  6380,  6352,  6324,  6296,  6269,  6242,
	 6215,  6188,  6162,  6135,  6109,  6084,  6058,  6033,
	 6007
};

const INT32U T_TempoDivide[]={
	102, 100, 98, 95, 93, 91, 89, 87,
	85, 84, 82, 80, 79, 77, 76, 74,
	73, 72, 71, 69, 68, 67, 66, 65,
	64, 63, 62, 61, 60, 59, 59, 58,
	57, 56, 55, 55, 54, 53, 53, 52,
	51, 51, 50, 49, 49, 48, 48, 47,
	47, 46, 46, 45, 45, 44, 44, 43,
	43, 42, 42, 41, 41, 41, 40, 40,
	39, 39, 39, 38, 38, 38, 37, 37,
	37, 36, 36, 36, 35, 35, 35, 34,
	34, 34, 34, 33, 33, 33, 33, 32,
	32, 32, 32, 31, 31, 31, 31, 30,
	30, 30, 30, 29, 29, 29, 29, 29,
	28, 28, 28, 28, 28, 27, 27, 27,
	27, 27, 27, 26, 26, 26, 26, 26,
	26, 25, 25, 25, 25, 25, 25, 25,
	24, 24, 24, 24, 24, 24, 24, 23,
	23, 23, 23, 23, 23, 23, 23, 22,
	22, 22, 22, 22, 22, 22, 22, 21,
	21, 21, 21, 21, 21, 21, 21, 21,
	20, 20, 20, 20, 20, 20, 20, 20,
	20, 20, 20, 19, 19, 19, 19, 19,
	19, 19, 19, 19, 19, 19, 18, 18,
	18, 18, 18, 18, 18, 18, 18, 18,
	18, 18, 18, 17, 17, 17, 17, 17,
	17
};

//*****************************************************************************
//							Pitch Bend Table
//*****************************************************************************
const INT32U T_PicthWheelTable[]={
	15464, 15478, 15492, 15506, 15520, 15534, 15548, 15562,
	15576, 15590, 15604, 15618, 15632, 15646, 15661, 15675,
	15689, 15703, 15717, 15731, 15746, 15760, 15774, 15788,
	15803, 15817, 15831, 15845, 15860, 15874, 15888, 15903,
	15917, 15931, 15946, 15960, 15975, 15989, 16004, 16018,
	16032, 16047, 16061, 16076, 16090, 16105, 16119, 16134,
	16149, 16163, 16178, 16192, 16207, 16222, 16236, 16251,
	16266, 16280, 16295, 16310, 16324, 16339, 16354, 16369,
	16384, 16398, 16413, 16428, 16443, 16458, 16472, 16487,
	16502, 16517, 16532, 16547, 16562, 16577, 16592, 16607,
	16622, 16637, 16652, 16667, 16682, 16697, 16712, 16727,
	16742, 16757, 16773, 16788, 16803, 16818, 16833, 16848,
	16864, 16879, 16894, 16909, 16925, 16940, 16955, 16970,
	16986, 17001, 17016, 17032, 17047, 17063, 17078, 17093,
	17109, 17124, 17140, 17155, 17171, 17186, 17202, 17217,
	17233, 17248, 17264, 17280, 17295, 17311, 17326, 17342,
	17358
};
//==================================================================

const INT32U T_PicthWheelTable_TWO[]={
	14596, 14623, 14649, 14676, 14702, 14729, 14755, 14782,
	14809, 14836, 14862, 14889, 14916, 14943, 14970, 14997,
	15024, 15051, 15079, 15106, 15133, 15160, 15188, 15215,
	15243, 15270, 15298, 15325, 15353, 15381, 15409, 15437,
	15464, 15492, 15520, 15548, 15576, 15605, 15633, 15661,
	15689, 15718, 15746, 15775, 15803, 15832, 15860, 15889,
	15918, 15946, 15975, 16004, 16033, 16062, 16091, 16120,
	16149, 16178, 16208, 16237, 16266, 16296, 16325, 16354,
	16384, 16414, 16443, 16473, 16503, 16533, 16562, 16592,
	16622, 16652, 16682, 16713, 16743, 16773, 16803, 16834,
	16864, 16895, 16925, 16956, 16986, 17017, 17048, 17079,
	17109, 17140, 17171, 17202, 17233, 17264, 17296, 17327,
	17358, 17390, 17421, 17452, 17484, 17516, 17547, 17579,
	17611, 17643, 17674, 17706, 17738, 17770, 17802, 17835,
	17867, 17899, 17931, 17964, 17996, 18029, 18061, 18094,
	18127, 18160, 18192, 18225, 18258, 18291, 18324, 18357,
	18390
};
//==================================================================
const INT32U T_PicthWheelTable_THREE[]={
	13777, 13815, 13852, 13890, 13927, 13965, 14003, 14041,
	14079, 14117, 14155, 14194, 14232, 14271, 14310, 14348,
	14387, 14426, 14465, 14505, 14544, 14583, 14623, 14663,
	14702, 14742, 14782, 14822, 14862, 14903, 14943, 14984,
	15024, 15065, 15106, 15147, 15188, 15229, 15270, 15312,
	15353, 15395, 15437, 15478, 15520, 15562, 15605, 15647,
	15689, 15732, 15775, 15817, 15860, 15903, 15946, 15990,
	16033, 16076, 16120, 16164, 16208, 16251, 16296, 16340,
	16384, 16428, 16473, 16518, 16562, 16607, 16652, 16697,
	16743, 16788, 16834, 16879, 16925, 16971, 17017, 17063,
	17109, 17156, 17202, 17249, 17296, 17343, 17390, 17437,
	17484, 17531, 17579, 17627, 17674, 17722, 17770, 17819,
	17867, 17915, 17964, 18013, 18061, 18110, 18160, 18209,
	18258, 18308, 18357, 18407, 18457, 18507, 18557, 18607,
	18658, 18708, 18759, 18810, 18861, 18912, 18963, 19015,
	19066, 19118, 19170, 19222, 19274, 19326, 19379, 19431,
	19484
};
//==================================================================

const INT32U T_PicthWheelTable_FOUR[]={
	13004, 13051, 13098, 13146, 13193, 13241, 13289, 13337,
	13385, 13433, 13482, 13531, 13580, 13629, 13678, 13728,
	13777, 13827, 13877, 13927, 13978, 14028, 14079, 14130,
	14181, 14232, 14284, 14335, 14387, 14439, 14491, 14544,
	14596, 14649, 14702, 14755, 14809, 14862, 14916, 14970,
	15024, 15079, 15133, 15188, 15243, 15298, 15353, 15409,
	15464, 15520, 15576, 15633, 15689, 15746, 15803, 15860,
	15918, 15975, 16033, 16091, 16149, 16208, 16266, 16325,
	16384, 16443, 16503, 16562, 16622, 16682, 16743, 16803,
	16864, 16925, 16986, 17048, 17109, 17171, 17233, 17296,
	17358, 17421, 17484, 17547, 17611, 17674, 17738, 17802,
	17867, 17931, 17996, 18061, 18127, 18192, 18258, 18324,
	18390, 18457, 18524, 18591, 18658, 18725, 18793, 18861,
	18929, 18998, 19066, 19135, 19205, 19274, 19344, 19414,
	19484, 19554, 19625, 19696, 19767, 19839, 19911, 19983,
	20055, 20127, 20200, 20273, 20347, 20420, 20494, 20568,
	20643
};
//==================================================================

const INT32U T_PicthWheelTable_FIVE[]={
	12274, 12330, 12385, 12441, 12498, 12554, 12611, 12668,
	12725, 12783, 12841, 12899, 12957, 13016, 13075, 13134,
	13193, 13253, 13313, 13373, 13433, 13494, 13555, 13617,
	13678, 13740, 13802, 13865, 13927, 13990, 14054, 14117,
	14181, 14245, 14310, 14374, 14439, 14505, 14570, 14636,
	14702, 14769, 14836, 14903, 14970, 15038, 15106, 15174,
	15243, 15312, 15381, 15450, 15520, 15591, 15661, 15732,
	15803, 15875, 15946, 16018, 16091, 16164, 16237, 16310,
	16384, 16458, 16533, 16607, 16682, 16758, 16834, 16910,
	16986, 17063, 17140, 17218, 17296, 17374, 17452, 17531,
	17611, 17690, 17770, 17851, 17931, 18013, 18094, 18176,
	18258, 18341, 18424, 18507, 18591, 18675, 18759, 18844,
	18929, 19015, 19101, 19187, 19274, 19361, 19449, 19537,
	19625, 19714, 19803, 19893, 19983, 20073, 20164, 20255,
	20347, 20439, 20531, 20624, 20717, 20811, 20905, 21000,
	21095, 21190, 21286, 21382, 21479, 21576, 21674, 21772,
	21870
};
//==================================================================

const INT32U T_PicthWheelTable_SIX[]={
	11585, 11648, 11711, 11775, 11839, 11903, 11968, 12033,
	12098, 12164, 12230, 12296, 12363, 12430, 12498, 12566,
	12634, 12702, 12771, 12841, 12910, 12981, 13051, 13122,
	13193, 13265, 13337, 13409, 13482, 13555, 13629, 13703,
	13777, 13852, 13927, 14003, 14079, 14155, 14232, 14310,
	14387, 14465, 14544, 14623, 14702, 14782, 14862, 14943,
	15024, 15106, 15188, 15270, 15353, 15437, 15520, 15605,
	15689, 15775, 15860, 15946, 16033, 16120, 16208, 16296,
	16384, 16473, 16562, 16652, 16743, 16834, 16925, 17017,
	17109, 17202, 17296, 17390, 17484, 17579, 17674, 17770,
	17867, 17964, 18061, 18160, 18258, 18357, 18457, 18557,
	18658, 18759, 18861, 18963, 19066, 19170, 19274, 19379,
	19484, 19590, 19696, 19803, 19911, 20019, 20127, 20237,
	20347, 20457, 20568, 20680, 20792, 20905, 21019, 21133,
	21247, 21363, 21479, 21595, 21713, 21831, 21949, 22068,
	22188, 22309, 22430, 22552, 22674, 22797, 22921, 23045,
	23170
};
//==================================================================

const INT32U T_PicthWheelTable_SEVEN[]={
	10935, 11004, 11074, 11144, 11215, 11286, 11357, 11429,
	11502, 11575, 11648, 11722, 11796, 11871, 11946, 12022,
	12098, 12175, 12252, 12330, 12408, 12486, 12566, 12645,
	12725, 12806, 12887, 12969, 13051, 13134, 13217, 13301,
	13385, 13470, 13555, 13641, 13728, 13815, 13902, 13990,
	14079, 14168, 14258, 14348, 14439, 14531, 14623, 14716,
	14809, 14903, 14997, 15092, 15188, 15284, 15381, 15478,
	15576, 15675, 15775, 15875, 15975, 16076, 16178, 16281,
	16384, 16488, 16592, 16697, 16803, 16910, 17017, 17125,
	17233, 17343, 17452, 17563, 17674, 17786, 17899, 18013,
	18127, 18242, 18357, 18474, 18591, 18708, 18827, 18946,
	19066, 19187, 19309, 19431, 19554, 19678, 19803, 19929,
	20055, 20182, 20310, 20439, 20568, 20699, 20830, 20962,
	21095, 21228, 21363, 21498, 21634, 21772, 21910, 22048,
	22188, 22329, 22470, 22613, 22756, 22900, 23045, 23191,
	23338, 23486, 23635, 23785, 23936, 24087, 24240, 24394,
	24548
};
//==================================================================

const INT32U T_PicthWheelTable_EIGHT[]={
	10321, 10396, 10471, 10547, 10624, 10701, 10778, 10856,
	10935, 11014, 11094, 11174, 11255, 11337, 11419, 11502,
	11585, 11669, 11754, 11839, 11925, 12011, 12098, 12186,
	12274, 12363, 12453, 12543, 12634, 12725, 12818, 12910,
	13004, 13098, 13193, 13289, 13385, 13482, 13580, 13678,
	13777, 13877, 13978, 14079, 14181, 14284, 14387, 14491,
	14596, 14702, 14809, 14916, 15024, 15133, 15243, 15353,
	15464, 15576, 15689, 15803, 15918, 16033, 16149, 16266,
	16384, 16503, 16622, 16743, 16864, 16986, 17109, 17233,
	17358, 17484, 17611, 17738, 17867, 17996, 18127, 18258,
	18390, 18524, 18658, 18793, 18929, 19066, 19205, 19344,
	19484, 19625, 19767, 19911, 20055, 20200, 20347, 20494,
	20643, 20792, 20943, 21095, 21247, 21401, 21556, 21713,
	21870, 22028, 22188, 22349, 22511, 22674, 22838, 23004,
	23170, 23338, 23507, 23678, 23849, 24022, 24196, 24372,
	24548, 24726, 24905, 25086, 25268, 25451, 25635, 25821,
	26008
};
//==================================================================
const INT32U T_PicthWheelTable_NINE[]={
	9742, 9821, 9902, 9982, 10064, 10146, 10229, 10312,
	10396, 10481, 10566, 10653, 10739, 10827, 10915, 11004,
	11094, 11185, 11276, 11368, 11460, 11554, 11648, 11743,
	11839, 11935, 12033, 12131, 12230, 12330, 12430, 12532,
	12634, 12737, 12841, 12945, 13051, 13157, 13265, 13373,
	13482, 13592, 13703, 13815, 13927, 14041, 14155, 14271,
	14387, 14505, 14623, 14742, 14862, 14984, 15106, 15229,
	15353, 15478, 15605, 15732, 15860, 15990, 16120, 16251,
	16384, 16518, 16652, 16788, 16925, 17063, 17202, 17343,
	17484, 17627, 17770, 17915, 18061, 18209, 18357, 18507,
	18658, 18810, 18963, 19118, 19274, 19431, 19590, 19750,
	19911, 20073, 20237, 20402, 20568, 20736, 20905, 21076,
	21247, 21421, 21595, 21772, 21949, 22128, 22309, 22491,
	22674, 22859, 23045, 23233, 23423, 23614, 23806, 24001,
	24196, 24394, 24593, 24793, 24995, 25199, 25405, 25612,
	25821, 26031, 26244, 26458, 26674, 26891, 27110, 27332,
	27554
};
//==================================================================

const INT32U T_PicthWheelTable_TEN[]={
	9195, 9279, 9363, 9448, 9533, 9620, 9707, 9795,
	9884, 9973, 10064, 10155, 10247, 10340, 10434, 10528,
	10624, 10720, 10817, 10915, 11014, 11114, 11215, 11317,
	11419, 11523, 11627, 11733, 11839, 11946, 12055, 12164,
	12274, 12385, 12498, 12611, 12725, 12841, 12957, 13075,
	13193, 13313, 13433, 13555, 13678, 13802, 13927, 14054,
	14181, 14310, 14439, 14570, 14702, 14836, 14970, 15106,
	15243, 15381, 15520, 15661, 15803, 15946, 16091, 16237,
	16384, 16533, 16682, 16834, 16986, 17140, 17296, 17452,
	17611, 17770, 17931, 18094, 18258, 18424, 18591, 18759,
	18929, 19101, 19274, 19449, 19625, 19803, 19983, 20164,
	20347, 20531, 20717, 20905, 21095, 21286, 21479, 21674,
	21870, 22068, 22268, 22470, 22674, 22880, 23087, 23296,
	23507, 23721, 23936, 24153, 24372, 24593, 24816, 25041,
	25268, 25497, 25728, 25961, 26196, 26434, 26674, 26915,
	27159, 27406, 27654, 27905, 28158, 28413, 28671, 28931,
	29193
};
//==================================================================

const INT32U T_PicthWheelTable_ELEVEN[]={
	8679, 8766, 8853, 8942, 9031, 9121, 9212, 9304,
	9397, 9490, 9585, 9681, 9777, 9875, 9973, 10073,
	10173, 10275, 10377, 10481, 10585, 10691, 10798, 10905,
	11014, 11124, 11235, 11347, 11460, 11575, 11690, 11807,
	11925, 12044, 12164, 12285, 12408, 12532, 12657, 12783,
	12910, 13039, 13169, 13301, 13433, 13567, 13703, 13840,
	13978, 14117, 14258, 14400, 14544, 14689, 14836, 14984,
	15133, 15284, 15437, 15591, 15746, 15903, 16062, 16222,
	16384, 16547, 16713, 16879, 17048, 17218, 17390, 17563,
	17738, 17915, 18094, 18275, 18457, 18641, 18827, 19015,
	19205, 19396, 19590, 19785, 19983, 20182, 20383, 20587,
	20792, 21000, 21209, 21421, 21634, 21850, 22068, 22288,
	22511, 22735, 22962, 23191, 23423, 23656, 23893, 24131,
	24372, 24615, 24860, 25108, 25359, 25612, 25868, 26126,
	26386, 26650, 26915, 27184, 27455, 27729, 28006, 28285,
	28567, 28852, 29140, 29431, 29725, 30021, 30321, 30623,
	30929
};
//==================================================================

const INT32U T_PicthWheelTable_TWELVE[]={
	8192, 8281, 8371, 8463, 8555, 8648, 8742, 8837,
	8933, 9031, 9129, 9228, 9329, 9431, 9533, 9637,
	9742, 9848, 9955, 10064, 10173, 10284, 10396, 10509,
	10624, 10739, 10856, 10975, 11094, 11215, 11337, 11460,
	11585, 11711, 11839, 11968, 12098, 12230, 12363, 12498,
	12634, 12771, 12910, 13051, 13193, 13337, 13482, 13629,
	13777, 13927, 14079, 14232, 14387, 14544, 14702, 14862,
	15024, 15188, 15353, 15520, 15689, 15860, 16033, 16208,
	16384, 16562, 16743, 16925, 17109, 17296, 17484, 17674,
	17867, 18061, 18258, 18457, 18658, 18861, 19066, 19274,
	19484, 19696, 19911, 20127, 20347, 20568, 20792, 21019,
	21247, 21479, 21713, 21949, 22188, 22430, 22674, 22921,
	23170, 23423, 23678, 23936, 24196, 24460, 24726, 24995,
	25268, 25543, 25821, 26102, 26386, 26674, 26964, 27258,
	27554, 27855, 28158, 28464, 28774, 29088, 29405, 29725,
	30048, 30376, 30706, 31041, 31379, 31720, 32066, 32415,
	32768
};

const INT32U T_PicthWheelTable_THIRTEEN[]={
	7732, 7823, 7916, 8009, 8104, 8199, 8296, 8394,
	8493, 8593, 8695, 8797, 8901, 9006, 9113, 9220,
	9329, 9439, 9550, 9663, 9777, 9893, 10009, 10127,
	10247, 10368, 10490, 10614, 10739, 10866, 10994, 11124,
	11255, 11388, 11523, 11659, 11796, 11935, 12076, 12219,
	12363, 12509, 12657, 12806, 12957, 13110, 13265, 13421,
	13580, 13740, 13902, 14066, 14232, 14400, 14570, 14742,
	14916, 15092, 15270, 15450, 15633, 15817, 16004, 16193,
	16384, 16577, 16773, 16971, 17171, 17374, 17579, 17786,
	17996, 18209, 18424, 18641, 18861, 19084, 19309, 19537,
	19767, 20001, 20237, 20476, 20717, 20962, 21209, 21459,
	21713, 21969, 22228, 22491, 22756, 23025, 23296, 23571,
	23849, 24131, 24416, 24704, 24995, 25290, 25589, 25891,
	26196, 26506, 26818, 27135, 27455, 27779, 28107, 28439,
	28774, 29114, 29458, 29805, 30157, 30513, 30873, 31237,
	31606, 31979, 32357, 32738, 33125, 33516, 33911, 34312,
	34716
};

#define USER_DEFINE_MALLOC_EN       1//1
/* golbal varaible */
volatile INT32U MIDI_PlayFlag;
volatile INT32U MIDI_Tempo;
volatile INT8U R_MIDI_Volume;		//080710 tangqt
volatile INT8U R_MIDI_Pan;			//080710 tangqt
volatile ALP_Header_Struct ALP_Header;
volatile MIDI_ChannelInfoStruct MIDI_ChannelInfo[MIDI_ChannelNumber];
volatile SPU_ChannelInfoStruct SPU_ChannelInfo[SPU_ChannelNumber];
volatile INT32U R_DUR_Tone[SPU_ChannelNumber + 1];

volatile INT32U uiAddr;
volatile INT32U uiData;
volatile INT32U uiPhaseLow;
volatile INT32U uiPhaseMiddle1;
volatile INT32U uiPhaseMiddle2;
volatile INT32U uiPhaseHigh;
volatile INT32U *pTableAddr;
volatile unsigned long ulData;

volatile INT32U R_channel_original_phase[SPU_ChannelNumber];//用于记录各个通道暂停前的phase值 

volatile INT32U R_Total_Voice;
volatile INT8U *pMIDI_StartAddr;
volatile INT8U *pMIDI_DataPointer;
volatile MIDI_ChannelInfoStruct *pMIDI_ChInfo;
volatile SPU_ChannelInfoStruct *pSPU_ChInfo;
//volatile union MIDI_EventStruct MIDI_Event;
volatile MIDI_EventStruct MIDI_Event;
volatile INT32U R_MIDI_EndFlag;
volatile INT32U R_CH_NoteOff;
volatile INT32U EventIndex;
//volatile INT32U R_CH_NoteOff;
volatile INT32U R_PlayChannel;
volatile INT32U R_CH_OneBeat;
volatile INT32U R_MIDI_CH_MASK;
volatile INT32U R_SourceMIDIMask;
volatile INT32U R_SourceMIDI_ChangeColor_Mask;//对应的位为1:该MIDI通道需要切换音色081105
volatile INT32U R_NoteOnHistPtr;
volatile INT32U R_Avail_Voice;
void (*MIDI_StopCallBack)(void);
void (*MIDI_DataEntryEventCallBack)(void);
void (*MIDI_PlayDtStopCallBack)(void);

INT8U	MIDI_Control_Event[4];
INT32U	MIDI_Current_Dt;
INT32U  MIDI_Stop_Dt;
INT8U	MIDI_Skip_Flag;
INT8U User_FIQ_Flag;
void (*SPU_User_FIQ_ISR[C_MAX_FIQ])(void);
//--------------------------------------------------------------------------080912
INT16S static_fd_idi;
INT8U SPU_load_data_mode;//081107
INT32U total_inst=0,total_drum=0;
INT32U idi_offset_addr=0;//记录idi数据在nandflash中的偏移地址或是在FS文件中的偏移地址 
INT32U currt_midi_data_offset;//记录当前读到的midi数据在整个idi文件中的偏移； 
INT32U static_midi_offset, static_midi_length, remain_midi_length;//当前这首midi的数据的偏移地址，midi数据字节数; 
INT32U midi_ring_buffer_addr;
INT32U midi_ring_buffer_ri, midi_ring_buffer_wi;
INT32U static_midi_ring_buffer_ri = 0;//记录midi数据的开始部分在ring buffer中的地址 
INT32U static_midi_ring_buffer_wi = 0;//记录midi数据的最后部分在ring buffer中的地址 
INT32U adpcm_comb_offset_addr=0;//adpcm comb数据在文件中或NANDFLASH中的地址 
INT32U adpcm_ram_buffer_addr=0, flag_malloc_adpcm_ram_buffer=1;//adpcm 数据存放的临时ram地址 
INT32U adpcm_data_temp_buffer_addr = 0;//用来标记当前播放的ADPCM数据的地址；若为0，则表示还未赋值 
INT32U inst_start_addr,lib_start_addr,midi_start_addr;//081230
//--------------------------------------------------------------------------
INT32U T_InstrumentStartSection[129];
INT32U T_InstrumentPitchTable[500];
INT32U *T_InstrumentSectionAddr[500];
//---------------------------------------------------------------------//切换音色和播音符用 
INT32U T_InstrumentStartSection_1[129];//切换音色和播音符用 
INT32U T_InstrumentPitchTable_1[500];//切换音色和播音符用 
INT32U *T_InstrumentSectionAddr_1[500];//切换音色和播音符用 
//---------------------------------------------------------------------
INT32U *T_DrumAddr[128];

static INT32S (*pfunc_user_memory_malloc)(INT32U size);
static INT32S (*pfunc_user_memory_free)(void *ptr);

/* external function */
extern void NVRAM_OS_LOCK(void);
extern void NVRAM_OS_UNLOCK(void);

#define NAND_LOCK	NVRAM_OS_LOCK()
#define NAND_UNLOCK	NVRAM_OS_LOCK()

// fpga test
//INT32U spu_test_ram_buffer[0x4000];
//INT32U spu_512_byte_per_sector_buffer[512*2];

//--------------------------------------------------------------------------

#ifndef __CS_COMPILER__
    const char  C_SPU_driver_version_string[] =	"GLB_GP-S2_10A_SPU-GPLIB-ADS_1.0.3  ";//library檔名 
#else
    const char  C_SPU_driver_version_string[] =	"GLB_GP-S2_10A_SPU-GPLIB-CS_1.0.3  ";
#endif

//1.0.1.1 与1.0.1相比，增加了从sdram总播midi和其它speech的功能//091225修改 
//1.0.1.2与1.0.1.1相比，将user_memory_malloc()和user_memory_free()放到spu driver外面//100531修改 

char* SPU_get_driver_version(void)
{
	char *p;
	p = (char *)&C_SPU_driver_version_string;
	return(p);
}

void SPU_err(void)
{
	//INT32U err_temp=1;
	//while(err_temp)
	//	;
}

void SPU_CLEAR_CHANNEL_ATT_SRAM(INT8U ChannelIndex) //090527
{
        INT32U  temp;

        temp = 0x10*ChannelIndex;

        *((INT32U *)0xd0401000 + temp) = 0;
        *((INT32U *)0xd0401004 + temp) = 0;
        *((INT32U *)0xd0401008 + temp) = 0;
        *((INT32U *)0xd040100c + temp) = 0;
        *((INT32U *)0xd0401010 + temp) = 0;
        *((INT32U *)0xd0401014 + temp) = 0;
        *((INT32U *)0xd0401018 + temp) = 0;
        *((INT32U *)0xd040101c + temp) = 0;
        *((INT32U *)0xd0401020 + temp) = 0;
        *((INT32U *)0xd0401024 + temp) = 0;
        *((INT32U *)0xd0401028 + temp) = 0;
        *((INT32U *)0xd040102c + temp) = 0;
        *((INT32U *)0xd0401034 + temp) = 0;
        *((INT32U *)0xd0401038 + temp) = 0;
        *((INT32U *)0xd040103c + temp) = 0;

        *((INT32U *)0xd0401800 + temp) &= 0x0007;//= 0;
        *((INT32U *)0xd0401804 + temp) = 0;
        *((INT32U *)0xd0401808 + temp) = 0;
        *((INT32U *)0xd040180c + temp) = 0;
        //*((INT32U *)0xd0401810 + temp) = 0; //20110520 mask
        *((INT32U *)0xd0401814 + temp) = 0;
        *((INT32U *)0xd0401818 + temp) = 0;
        *((INT32U *)0xd040181c + temp) = 0;
}

void F_StopExpiredCH(void)
{
	INT32U Temp;
	INT32U Ch;
	Temp = SPU_Get_EnvRampDown();
	Temp |= R_CH_NoteOff;
	SPU_Set_EnvRampDownMultiChannel(Temp);

	Temp = R_CH_NoteOff;
	for(Ch = 0; Ch < SPU_ChannelNumber; Ch++)
	{
		if(Temp & 0x0001)
		{
			(pSPU_ChInfo + Ch)->R_MIDI_CH_MAP = 0xFFFF;
			(pSPU_ChInfo + Ch)->R_NOTE_PITCH = 0;//081105
		}
		Temp = Temp >> 1;
	}
}

INT32U Calculate_TempPan(INT32U Pan1, INT32U Pan2)
{
	INT32U ret;

	if(Pan1 == 0)
	{
		if(Pan2 == 127)
			ret = 64;
		else
			ret = 0;
		return ret;
	}
	if(Pan2 == 0)
	{
		if(Pan1 == 127)
			ret = 64;
		else
			ret = 0;
		return ret;
	}
	if(Pan1 == 127)
	{
		ret = 127;
		return ret;
	}
	if(Pan2 == 127)
	{
		ret = 127;
		return ret;
	}
	if(Pan1 < 64)
	{
		if(Pan2 < 64)
		{
			ret = (Pan1 * Pan2) >> 6;
		}
		else
		{
			ret = 127 - (((127 - Pan1) * (127 - Pan2)) >> 6);
		}
	}
	else
	{
		if(Pan2 < 64)
		{
			ret = 127 - (((127 - Pan1) * (127 - Pan2)) >> 6);
		}
		else
		{
			ret = (Pan1 * Pan2) >> 6;
		}
	}
	return ret;
}

INT32U Calculate_Pan(INT32U SPU_Ch)
{
	INT32U uiPan, uiVelo;
	INT32U MIDI_Ch;
	INT32U Temp1, Temp2;

	MIDI_Ch = (pSPU_ChInfo + SPU_Ch)->R_MIDI_CH_MAP;
	if(MIDI_Ch > 16)//081010
		MIDI_Ch = 0;
	Temp1 = (pMIDI_ChInfo + MIDI_Ch)->R_MIDI_CH_PAN;
	Temp2 = (pSPU_ChInfo + SPU_Ch)->R_MIDI_ToneLib_PAN;
	//--------------------------------------------------------------------------080808
	Temp1 = Calculate_TempPan(Temp1,Temp2);
	uiPan = Calculate_TempPan(Temp1,R_MIDI_Pan);
	//--------------------------------------------------------------------------
	Temp1 = (pMIDI_ChInfo + MIDI_Ch)->R_MIDI_CH_EXPRESSION;
	Temp2 = (pMIDI_ChInfo + MIDI_Ch)->R_MIDI_CH_VOLUME;
	Temp1 = (Temp1 * Temp2) >> 7;
	Temp2 = (pSPU_ChInfo + SPU_Ch)->R_NOTE_VELOCITY;
	Temp1 = (Temp1 * Temp2) >> 7;
	Temp2 = (pSPU_ChInfo + SPU_Ch)->R_MIDI_ToneLib_Volume;
	//--------------------------------------------------------------------------
	Temp1 = (Temp1 * R_MIDI_Volume) >> 7;//080710 tangqt
	//--------------------------------------------------------------------------
	uiVelo = ((Temp1 * Temp2) >> 7) & 0x007f;
	return ((uiPan << 8) | uiVelo);
}
//------------------------------------------------------------------------------100817
INT8U T_channel[SPU_ChannelNumber];

INT8U find_channel(INT8U ch)//查找数组中是否有ch这个值 
{
	INT8U i;
	for(i=0;i<SPU_ChannelNumber;i++)
	{
		if(T_channel[i] == ch)
			break;
	}
	if(i<SPU_ChannelNumber)
		return i;
	else
		return 0xff;
}

void delete_channel(INT8U index)//将数组中第index个单元删除，且将后面的单元往前移，最后一个单元补0xff值 
{
	INT8U i;
	for(i=index;i<SPU_ChannelNumber-1;i++)
	{
		T_channel[i] = T_channel[i+1];
	}
	T_channel[SPU_ChannelNumber-1] = 0xff;
}

void insert_channel(INT8U ch)//在数组的最开始处插入一个通道 
{
	INT8U temp,i;
	temp = find_channel(ch);
	if(temp != 0xff)//在数组中有该channel
		delete_channel(temp);//删除数组中的第temp个单元 
	for(i=SPU_ChannelNumber-1;i>0;i--)
	{
		T_channel[i] = T_channel[i-1];//将数组往后移一个单元，将第一个单元空出来 
	}
	T_channel[0] = ch;
}

INT8U get_oldest_channel(void)
{
	INT8S ch;
	for(ch=SPU_ChannelNumber-1;ch>=0;ch--)
	{
		if(T_channel[ch] != 0xff)
			break;
	}
	if(ch >= 0)
		return T_channel[ch];
	else
		return 0;//数组全为空，强制返回通道0 
}

INT32U FindEmptyChannel(void)
{
	INT32U Temp;
	INT32U ChannelIndex;
	INT32U SPU_IdleChannel;

	SPU_IdleChannel = SPU_GetChannelStatus();
	SPU_IdleChannel |= R_CH_OneBeat;
	SPU_IdleChannel ^= 0xFFFFFFFF;
	SPU_IdleChannel &= R_MIDI_CH_MASK;

	if(SPU_IdleChannel)
	{
		Temp = 0x00000001;
		for(ChannelIndex = 0; ChannelIndex < SPU_ChannelNumber; ChannelIndex++)
		{
			if(SPU_IdleChannel & Temp)
				break;
			else
			{
				Temp = Temp << 1;
			}
		}
		insert_channel((INT8U)ChannelIndex);
	}
	else	// all channels are busy
	{
		ChannelIndex = get_oldest_channel();
		insert_channel((INT8U)ChannelIndex);
	}
	SPU_StopChannel(ChannelIndex);
	return ChannelIndex;
}
//------------------------------------------------------------------------------
#if 1 // williamyeo
// sampleRate: sample rate of spu file. Ex 48K, then value is 48000
INT32U SPU_CalculateI2SPhase(INT32U sampleRate)
{
    INT32U phase;
    INT32U i2s_sampleRate = SPU_GetI2SSyncSampleRate();
    INT32U spu_FrameRate = 48000; // GP22 is 48K ? According to 1KHz sine wave measurement, it is 48K.
                                  // then spu clock should 6*48K = 288k, not 281.25.

    // 524288UL mean 2^19
    // 288 is assume SPU apply 48K frame rate when sync with I2S, old is 281.25
    // divide 1000.0 is to avoid overflow after multiply with 2^19
    phase = (INT32U)((((sampleRate/(double)1000.0) * 524288UL)/(double)288.0)*spu_FrameRate/i2s_sampleRate);

    DBG_PRINT("I2S play rate[%u]K, file sample rate %u, then phase is %u, 0x%08x\r\n", \
          i2s_sampleRate, sampleRate, phase, phase);

    return phase;
}
#endif

void Load_ALP_Header(INT32U *pAddr)
{
	volatile INT32U uiALP_HeaderData;

	if((INT32U)pAddr & 0x00000001)
	{
		SPU_err();
	}
	ALP_Header.ALP_ID[0] = *pAddr;
	if(ALP_Header.ALP_ID[0] != 0x32465053)
	{
		SPU_err();
	}
	ALP_Header.ALP_ID[1] = *(pAddr + 1);
	ALP_Header.ALP_ID[2] = *(pAddr + 2);
	ALP_Header.ALP_ID[3] = *(pAddr + 3);
	ALP_Header.SampleRate = *(pAddr + 4);
	ALP_Header.SampleLength = *(pAddr + 5);
	ALP_Header.LoopStartAddr = *(pAddr + 6);
	ALP_Header.EnvelopStartAddr = *(pAddr + 7);
	uiALP_HeaderData = *(pAddr + 8);
	ALP_Header.WaveType = uiALP_HeaderData & 0x000000FF;
	ALP_Header.BasePitch = (uiALP_HeaderData & 0x0000FF00) >> 8;
	ALP_Header.MaxPitch = (uiALP_HeaderData & 0x00FF0000) >> 16;
	ALP_Header.MinPitch = (uiALP_HeaderData & 0xFF000000) >> 24;
	uiALP_HeaderData = *(pAddr + 9);
	ALP_Header.RampDownClock = uiALP_HeaderData & 0x000000FF;
	ALP_Header.RampDownStep = (uiALP_HeaderData & 0x0000FF00) >> 8;
	ALP_Header.Pan = (uiALP_HeaderData & 0x00FF0000) >> 16;
	ALP_Header.Velocity = (uiALP_HeaderData & 0xFF000000) >> 24;
}

void SPU_PlayNote(INT8U uiPlayPitch, INT32U *pAddr, INT8U uiPan, INT8U uiVelocity, INT8U uiSPUChannel)
{
	INT32U MIDI_Channel;
	INT32U PitchBendIndex, PitchBendValue;
	INT32U NewPhase;
	INT32U *pAddrPB;
	INT16U Temp;
	INT16U *pStartAddr;

	SPU_CLEAR_CHANNEL_ATT_SRAM(uiSPUChannel);//090527
	Load_ALP_Header(pAddr);
	SPU_EnvelopeAutoMode(uiSPUChannel);
	uiAddr = ((INT32U)(pAddr + 10) / 2);
	SPU_SetStartAddress(uiAddr, uiSPUChannel);
	SPU_ClearADCPM36_Mode(uiSPUChannel);	//Roy 2008-0521

	if(ALP_Header.WaveType & 0x0040)		// ADPCM mode
	{
		// ADPCM mode, check!
		SPU_SetADPCM_Mode(uiSPUChannel);	// set ADPCM mode
		if(ALP_Header.WaveType & 0x0080)	// ADPCM36 mode
		{
			SPU_SetWaveData_0(0x8000, uiSPUChannel);	//Roy 2008-0521
			SPU_SelectADPCM36_Mode(uiSPUChannel);		// set ADPCM36 mode
			SPU_SetADPCM_PointNumber(31, uiSPUChannel); // set point number = 31
			SPU_SetToneColorMode(3, uiSPUChannel);		// HW auto-repeat mode 1
		}
		else
		{
			SPU_SelectADPCM_Mode(uiSPUChannel);		// set ADPCM mode
			SPU_SetADPCM_PointNumber(0, uiSPUChannel);
			SPU_SetToneColorMode(2, uiSPUChannel);
			uiAddr += 1;
			SPU_SetStartAddress(uiAddr, uiSPUChannel);
		}
		if(ALP_Header.WaveType & 0x0010)		// check loop mode
		{
			SPU_Set_16bit_Mode(uiSPUChannel);	// set 16-bit PCM mode
		}
		else
			SPU_Set_8bit_Mode(uiSPUChannel);	// set 8-bit PCM mode
	}
	else	// PCM mode
	{
		SPU_SetPCM_Mode(uiSPUChannel);			// set PCM mode
		if(ALP_Header.WaveType & 0x0010)		// 16-bit PCM mode
		{
			SPU_Set_16bit_Mode(uiSPUChannel);	// set 16-bit PCM mode
		}
		else
			SPU_Set_8bit_Mode(uiSPUChannel);	// set 8-bit PCM mode
		SPU_SetToneColorMode(2, uiSPUChannel);	// HW auto-repeat mode
	}
	SPU_SetWaveData_0(0x8000, uiSPUChannel);
	SPU_SetWaveData(*(pAddr + 10), uiSPUChannel);	// tone color first sample

	uiAddr = ((INT32U)pAddr)/2 + ALP_Header.LoopStartAddr;
	SPU_SetLoopAddress(uiAddr, uiSPUChannel);

	SPU_SetVelocity(uiVelocity, uiSPUChannel);
	SPU_SetPan(uiPan, uiSPUChannel);


	uiAddr = (INT32U)pAddr + ALP_Header.EnvelopStartAddr * 2;
	pTableAddr = (INT32U *)uiAddr;
	pStartAddr = (INT16U *)uiAddr;
	Temp = *pStartAddr;
	SPU_SetEnvelope_0(Temp, uiSPUChannel);
	pStartAddr += 1;
	Temp = *pStartAddr;
	SPU_SetEnvelope_1(Temp, uiSPUChannel);
	SPU_SetEnvelopeData(0x0000, uiSPUChannel);
	SPU_SetEnvelopeRampDownOffset(ALP_Header.RampDownStep, uiSPUChannel);
	SPU_SetEnvelopeRepeatAddrOffset(0x0000, uiSPUChannel);

	uiAddr = uiAddr / 2;
	SPU_SetEnvelopeAddress(uiAddr, uiSPUChannel);
	SPU_SetWaveData(0x8000, uiSPUChannel);

	pTableAddr = (INT32U *)T_VarPhaseTable;
	uiData = *(pTableAddr + uiPlayPitch - ALP_Header.BasePitch);
	uiPhaseLow = (uiData & 0xFFFF) * (ALP_Header.SampleRate & 0xFFFF);
	uiPhaseMiddle1 = (uiData & 0xFFFF) * (ALP_Header.SampleRate >> 16);
	uiPhaseMiddle2 = (uiData >> 16) * (ALP_Header.SampleRate & 0xFFFF);
	uiPhaseHigh = (uiData >> 16) * (ALP_Header.SampleRate >> 16);
	uiPhaseLow = (uiPhaseLow + 0x00008000) >> 16;
	uiPhaseMiddle1 += uiPhaseMiddle2;
	uiPhaseMiddle1 += uiPhaseLow;
	uiPhaseHigh = uiPhaseHigh << 16;
	uiPhaseHigh += uiPhaseMiddle1;

	(pSPU_ChInfo + uiSPUChannel)->R_PB_PhaseRecord = uiPhaseHigh;
	MIDI_Channel = MIDI_Event.NoteEvent.ChannelNumber;
	PitchBendIndex = (pMIDI_ChInfo + MIDI_Channel)->R_MIDI_CH_PitchBend;
	pAddrPB = (INT32U *)((pMIDI_ChInfo + MIDI_Channel)->R_PB_TABLE_Addr + PitchBendIndex);
	PitchBendValue = *pAddrPB;
	NewPhase = (PitchBendValue * uiPhaseHigh) >> 14;

	SPU_SetPhase(NewPhase, uiSPUChannel);
	SPU_SetPhaseAccumulator(0x0000, uiSPUChannel);
	SPU_SetTargetPhase(0x0000, uiSPUChannel);
	SPU_SetPhaseOffset(0x0000, uiSPUChannel);
	SPU_SetPhaseIncrease(uiSPUChannel);
	SPU_SetPhaseTimeStep(0x0000, uiSPUChannel);
	SPU_SetRampDownClock(ALP_Header.RampDownClock, uiSPUChannel);
}

void SPU_PlayTone1(INT8U uiPitch, INT8U uiPan, INT8U uiVelocity, INT8U uiSPUChannel)
{
	SPU_CLEAR_CHANNEL_ATT_SRAM(uiSPUChannel);//090527
	SPU_EnvelopeManualMode(uiSPUChannel);			//set Manual Mode

	SPU_SetStartAddress(0, uiSPUChannel);
	SPU_ClearADCPM36_Mode(uiSPUChannel);

	SPU_SetPCM_Mode(uiSPUChannel);					// set PCM mode
	SPU_Set_16bit_Mode(uiSPUChannel);				// set 16-bit PCM mode
	SPU_SetToneColorMode(1, uiSPUChannel);			// HW auto-repeat mode
	SPU_SetWaveData_0(0x8000, uiSPUChannel);
	SPU_SetWaveData(0x8000, uiSPUChannel);			// tone color first sample

	SPU_SetLoopAddress(0x0000, uiSPUChannel);
	SPU_SetVelocity(uiVelocity, uiSPUChannel);
	SPU_SetPan(uiPan, uiSPUChannel);

	// Play With No Envelop
	SPU_SetEnvelope_0(0, uiSPUChannel);
	SPU_SetEnvelope_1(0, uiSPUChannel);
	SPU_SetEnvelopeData(0x007F, uiSPUChannel);
	SPU_SetEnvelopeRampDownOffset(0x15, uiSPUChannel);
	SPU_SetEnvelopeRepeatAddrOffset(0x0000, uiSPUChannel);

	SPU_SetEnvelopeAddress(0, uiSPUChannel);
	//SPU_SetWaveData(0x8000, uiSPUChannel);

	uiPhaseHigh = (0x1DD * 44100 ) >> 8;	//sample rate is 44.1K
	SPU_SetPhase(uiPhaseHigh , uiSPUChannel);
	SPU_SetPhaseAccumulator(0x0000, uiSPUChannel);
	SPU_SetTargetPhase(0x0000, uiSPUChannel);
	SPU_SetPhaseOffset(0x0000, uiSPUChannel);
	SPU_SetPhaseIncrease(uiSPUChannel);
	SPU_SetPhaseTimeStep(0x0000, uiSPUChannel);
	SPU_SetRampDownClock(0x01, uiSPUChannel);
}

void SPU_PlayTone(INT8U uiPitch, INT32U *pAddr, INT8U uiPan, INT8U uiVelocity, INT8U uiSPUChannel)
{
	INT16U Temp;
	INT16U *pStartAddr;

	SPU_CLEAR_CHANNEL_ATT_SRAM(uiSPUChannel);//090527
	Load_ALP_Header(pAddr);
	if(uiPitch == 0xFF)
		uiPitch = ALP_Header.BasePitch;

	SPU_EnvelopeAutoMode(uiSPUChannel);
	uiAddr = ((INT32U)(pAddr + 10) / 2);
	SPU_SetStartAddress(uiAddr, uiSPUChannel);
	SPU_ClearADCPM36_Mode(uiSPUChannel);	// 20080521 Roy

	if(ALP_Header.WaveType & 0x0040)		// ADPCM mode
	{
		// ADPCM mode, check!
		SPU_SetADPCM_Mode(uiSPUChannel);	// set ADPCM mode
		if(ALP_Header.WaveType & 0x0080)	// ADPCM36 mode
		{
			SPU_SetWaveData_0(0x8000, uiSPUChannel);	// 20080521 Roy
			SPU_SelectADPCM36_Mode(uiSPUChannel);		// set ADPCM36 mode
			SPU_SetADPCM_PointNumber(31, uiSPUChannel); // set point number = 3
			SPU_SetToneColorMode(3, uiSPUChannel);		// HW auto-repeat mode 1
		}
		else
		{
			SPU_SelectADPCM_Mode(uiSPUChannel);			// set ADPCM mode
			SPU_SetToneColorMode(2, uiSPUChannel);
			uiAddr += 1;
			SPU_SetStartAddress(uiAddr, uiSPUChannel);
		}
		if(ALP_Header.WaveType & 0x0010)		// check loop mode
		{
			SPU_Set_16bit_Mode(uiSPUChannel);	// set 16-bit PCM mode
		}
		else
			SPU_Set_8bit_Mode(uiSPUChannel);	// set 8-bit PCM mode
	}
	else	// PCM mode
	{
		SPU_SetPCM_Mode(uiSPUChannel);			// set PCM mode
		if(ALP_Header.WaveType & 0x0010)		// 16-bit PCM mode
		{
			SPU_Set_16bit_Mode(uiSPUChannel);	// set 16-bit PCM mode
		}
		else
			SPU_Set_8bit_Mode(uiSPUChannel);	// set 8-bit PCM mode
		SPU_SetToneColorMode(2, uiSPUChannel);	// HW auto-repeat mode
	}
	SPU_SetWaveData_0(0x8000, uiSPUChannel);
	SPU_SetWaveData(*(pAddr + 10), uiSPUChannel);	// tone color first sample

	uiAddr = ((INT32U)pAddr)/2 + ALP_Header.LoopStartAddr;
	SPU_SetLoopAddress(uiAddr, uiSPUChannel);
	SPU_SetVelocity(uiVelocity, uiSPUChannel);
	SPU_SetPan(uiPan, uiSPUChannel);

	uiAddr = (INT32U)pAddr + ALP_Header.EnvelopStartAddr * 2;
	pTableAddr = (INT32U *)uiAddr;
	pStartAddr = (INT16U *)uiAddr;
	Temp = *pStartAddr;
	SPU_SetEnvelope_0(Temp, uiSPUChannel);
	pStartAddr += 1;
	Temp = *pStartAddr;
	SPU_SetEnvelope_1(Temp, uiSPUChannel);
	SPU_SetEnvelopeData(0x0000, uiSPUChannel);
	SPU_SetEnvelopeRampDownOffset(ALP_Header.RampDownStep, uiSPUChannel);
	SPU_SetEnvelopeRepeatAddrOffset(0x0000, uiSPUChannel);

	uiAddr = uiAddr / 2;
	SPU_SetEnvelopeAddress(uiAddr, uiSPUChannel);
	SPU_SetWaveData(0x8000, uiSPUChannel);

	pTableAddr = (INT32U *)T_VarPhaseTable;
	uiData = *(pTableAddr + uiPitch - ALP_Header.BasePitch);
	uiPhaseLow = (uiData & 0xFFFF) * (ALP_Header.SampleRate & 0xFFFF);
	uiPhaseMiddle1 = (uiData & 0xFFFF) * (ALP_Header.SampleRate >> 16);
	uiPhaseMiddle2 = (uiData >> 16) * (ALP_Header.SampleRate & 0xFFFF);
	uiPhaseHigh = (uiData >> 16) * (ALP_Header.SampleRate >> 16);
	uiPhaseLow = (uiPhaseLow + 0x00008000) >> 16;
	uiPhaseMiddle1 += uiPhaseMiddle2;
	uiPhaseMiddle1 += uiPhaseLow;
	uiPhaseHigh = uiPhaseHigh << 16;
	uiPhaseHigh += uiPhaseMiddle1;

	SPU_SetPhase(uiPhaseHigh, uiSPUChannel);
	SPU_SetPhaseAccumulator(0x0000, uiSPUChannel);
	SPU_SetTargetPhase(0x0000, uiSPUChannel);
	SPU_SetPhaseOffset(0x0000, uiSPUChannel);
	SPU_SetPhaseIncrease(uiSPUChannel);
	SPU_SetPhaseTimeStep(0x0000, uiSPUChannel);
	SPU_SetRampDownClock(ALP_Header.RampDownClock, uiSPUChannel);

	SPU_Enable_Channel(uiSPUChannel);		//080710 tangqt
	SPU_Clear_Ch_StopFlag(uiSPUChannel);	//080710 tangqt
}

void SPU_PlayDrum(INT8U uiDrumIndex, INT32U *pAddr, INT8U uiPan, INT8U uiVelocity, INT8U uiSPUChannel)
{
	INT16U Temp;
	INT16U *pStartAddr;

	SPU_CLEAR_CHANNEL_ATT_SRAM(uiSPUChannel);//090527
	Load_ALP_Header(pAddr);
	SPU_EnvelopeAutoMode(uiSPUChannel);
	uiAddr = ((INT32U)(pAddr + 10) / 2);
	SPU_SetStartAddress(uiAddr, uiSPUChannel);
	SPU_ClearADCPM36_Mode(uiSPUChannel);	// 20080521 Roy

	if(ALP_Header.WaveType & 0x0040)		// ADPCM mode
	{
		// ADPCM mode, check!
		SPU_SetADPCM_Mode(uiSPUChannel);	// set ADPCM mode
		if(ALP_Header.WaveType & 0x0080)	// ADPCM36 mode
		{
			SPU_SetWaveData_0(0x8000, uiSPUChannel);	// 20080521 Roy
			SPU_SelectADPCM36_Mode(uiSPUChannel);		// set ADPCM36 mode
			SPU_SetADPCM_PointNumber(31, uiSPUChannel); // set point number = 31
			SPU_SetToneColorMode(1, uiSPUChannel);		// HW auto-repeat mode 1
		}
		else
		{
			SPU_SelectADPCM_Mode(uiSPUChannel);			// set ADPCM mode
			SPU_SetADPCM_PointNumber(0, uiSPUChannel);
			SPU_SetToneColorMode(1, uiSPUChannel);
			uiAddr += 1;
			SPU_SetStartAddress(uiAddr, uiSPUChannel);
		}
		if(ALP_Header.WaveType & 0x0010)		// check loop mode
		{
			SPU_Set_16bit_Mode(uiSPUChannel);	// set 16-bit PCM mode
		}
		else
			SPU_Set_8bit_Mode(uiSPUChannel);	// set 8-bit PCM mode
	}
	else	// PCM mode
	{
		SPU_SetPCM_Mode(uiSPUChannel);			// set PCM mode
		if(ALP_Header.WaveType & 0x0010)		// 16-bit PCM mode
		{
			SPU_Set_16bit_Mode(uiSPUChannel);	// set 16-bit PCM mode
		}
		else
			SPU_Set_8bit_Mode(uiSPUChannel);	// set 8-bit PCM mode
		SPU_SetToneColorMode(1, uiSPUChannel);	// HW auto-repeat mode
	}
	SPU_SetWaveData_0(0x8000, uiSPUChannel);
	SPU_SetWaveData(*(pAddr + 10), uiSPUChannel);	// tone color first sample

	SPU_SetLoopAddress(0x0000, uiSPUChannel);
	SPU_SetVelocity(uiVelocity, uiSPUChannel);
	SPU_SetPan(uiPan, uiSPUChannel);

	uiAddr = (INT32U)pAddr + ALP_Header.EnvelopStartAddr * 2;
	pTableAddr = (INT32U *)uiAddr;
	pStartAddr = (INT16U *)uiAddr;
	Temp = *pStartAddr;
	SPU_SetEnvelope_0(Temp, uiSPUChannel);
	pStartAddr += 1;
	Temp = *pStartAddr;
	SPU_SetEnvelope_1(Temp, uiSPUChannel);
	SPU_SetEnvelopeData(0x0000, uiSPUChannel);
	SPU_SetEnvelopeRampDownOffset(ALP_Header.RampDownStep, uiSPUChannel);
	SPU_SetEnvelopeRepeatAddrOffset(0x0000, uiSPUChannel);

	uiAddr = uiAddr / 2;
	SPU_SetEnvelopeAddress(uiAddr, uiSPUChannel);
	SPU_SetWaveData(0x8000, uiSPUChannel);

#if 1
	pTableAddr = (INT32U *)T_VarPhaseTable;
	uiData = *pTableAddr;
	uiPhaseLow = (uiData & 0xFFFF) * (ALP_Header.SampleRate & 0xFFFF);
	uiPhaseMiddle1 = (uiData & 0xFFFF) * (ALP_Header.SampleRate >> 16);
	uiPhaseMiddle2 = (uiData >> 16) * (ALP_Header.SampleRate & 0xFFFF);
	uiPhaseHigh = (uiData >> 16) * (ALP_Header.SampleRate >> 16);
	uiPhaseLow = (uiPhaseLow + 0x00008000) >> 16;
	uiPhaseMiddle1 += uiPhaseMiddle2;
	uiPhaseMiddle1 += uiPhaseLow;
	uiPhaseHigh = uiPhaseHigh << 16;
	uiPhaseHigh += uiPhaseMiddle1;
#else
	uiData = 0x1DD;
	uiPhaseHigh = (uiData * ALP_Header.SampleRate) >> 8;
#endif

	SPU_SetPhase(uiPhaseHigh, uiSPUChannel);
	SPU_SetPhaseAccumulator(0x0000, uiSPUChannel);
	SPU_SetTargetPhase(0x0000, uiSPUChannel);
	SPU_SetPhaseOffset(0x0000, uiSPUChannel);
	SPU_SetPhaseIncrease(uiSPUChannel);
	SPU_SetPhaseTimeStep(0x0000, uiSPUChannel);
	SPU_SetRampDownClock(ALP_Header.RampDownClock, uiSPUChannel);

	(pSPU_ChInfo + uiSPUChannel)->R_PB_PhaseRecord = uiPhaseHigh;
}

INT8U SPU_PlayPCM_NoEnv(INT32U *pAddr, INT8U uiPan, INT8U uiVelocity)//auto find empty channel
{
	INT8U  temp_BeatIRQ_Enable_Flag;
	INT8U	uiSPUChannel;

	temp_BeatIRQ_Enable_Flag = SPU_Get_BeatIRQ_Enable_Flag();	//080805 tangqt
	SPU_Disable_BeatIRQ();		//080805 tangqt

	uiSPUChannel = FindEmptyChannel();

	SPU_CLEAR_CHANNEL_ATT_SRAM(uiSPUChannel);//090527
	Load_ALP_Header(pAddr);
	SPU_EnvelopeManualMode((INT8U)uiSPUChannel);	//set Manual Mode
	uiAddr = ((INT32U)(pAddr + 10) / 2);
	SPU_SetStartAddress(uiAddr, uiSPUChannel);
	SPU_ClearADCPM36_Mode(uiSPUChannel);			// 20080521 Roy

	if(ALP_Header.WaveType & 0x0040)				// ADPCM mode
	{
		// ADPCM mode, check!
		SPU_SetADPCM_Mode(uiSPUChannel);			// set ADPCM mode
		if(ALP_Header.WaveType & 0x0080)			// ADPCM36 mode
		{
			SPU_SetWaveData_0(0x8000, uiSPUChannel);	// 20080521 Roy
			SPU_SelectADPCM36_Mode(uiSPUChannel);		// set ADPCM36 mode
			SPU_SetADPCM_PointNumber(31, uiSPUChannel); // set point number = 31
			SPU_SetToneColorMode(1, uiSPUChannel);		// HW auto-repeat mode 1
		}
		else
		{
			SPU_SelectADPCM_Mode(uiSPUChannel);			// set ADPCM mode
			SPU_SetADPCM_PointNumber(0, uiSPUChannel);
			SPU_SetToneColorMode(1, uiSPUChannel);
			uiAddr += 1;
			SPU_SetStartAddress(uiAddr, uiSPUChannel);
		}
		if(ALP_Header.WaveType & 0x0010)				// check loop mode
		{
			SPU_Set_16bit_Mode(uiSPUChannel);			// set 16-bit PCM mode
		}
		else
			SPU_Set_8bit_Mode(uiSPUChannel);			// set 8-bit PCM mode
	}
	else	// PCM mode
	{
		SPU_SetPCM_Mode(uiSPUChannel);					// set PCM mode
		if(ALP_Header.WaveType & 0x0010)				// 16-bit PCM mode
		{
			SPU_Set_16bit_Mode(uiSPUChannel);			// set 16-bit PCM mode
		}
		else
			SPU_Set_8bit_Mode(uiSPUChannel);			// set 8-bit PCM mode
		SPU_SetToneColorMode(1, uiSPUChannel);			// HW auto-end mode
		//SPU_SetToneColorMode(2, uiSPUChannel);//// HW auto-repeat mode
	}
	SPU_SetWaveData_0(0x8000, uiSPUChannel);
	SPU_SetWaveData(*(pAddr + 10), uiSPUChannel);		// tone color first sample

	SPU_SetLoopAddress(0x0000, uiSPUChannel);
	SPU_SetVelocity(uiVelocity, uiSPUChannel);
	SPU_SetPan(uiPan, uiSPUChannel);

	// Play With No Envelop
	SPU_SetEnvelope_0(0, uiSPUChannel);
	SPU_SetEnvelope_1(0, uiSPUChannel);
	SPU_SetEnvelopeData(0x007F, uiSPUChannel);
	SPU_SetEnvelopeRampDownOffset(ALP_Header.RampDownStep, uiSPUChannel);
	SPU_SetEnvelopeRepeatAddrOffset(0x0000, uiSPUChannel);

	SPU_SetEnvelopeAddress(0, uiSPUChannel);
	SPU_SetWaveData(0x8000, uiSPUChannel);

	pTableAddr = (INT32U *)T_VarPhaseTable;
	uiData = *pTableAddr;
	uiPhaseLow = (uiData & 0xFFFF) * (ALP_Header.SampleRate & 0xFFFF);
	uiPhaseMiddle1 = (uiData & 0xFFFF) * (ALP_Header.SampleRate >> 16);
	uiPhaseMiddle2 = (uiData >> 16) * (ALP_Header.SampleRate & 0xFFFF);
	uiPhaseHigh = (uiData >> 16) * (ALP_Header.SampleRate >> 16);
	uiPhaseLow = (uiPhaseLow + 0x00008000) >> 16;
	uiPhaseMiddle1 += uiPhaseMiddle2;
	uiPhaseMiddle1 += uiPhaseLow;
	uiPhaseHigh = uiPhaseHigh << 16;
	uiPhaseHigh += uiPhaseMiddle1;

	SPU_SetPhase(uiPhaseHigh, uiSPUChannel);
	SPU_SetPhaseAccumulator(0x0000, uiSPUChannel);
	SPU_SetTargetPhase(0x0000, uiSPUChannel);
	SPU_SetPhaseOffset(0x0000, uiSPUChannel);
	SPU_SetPhaseIncrease(uiSPUChannel);
	SPU_SetPhaseTimeStep(0x0000, uiSPUChannel);
	SPU_SetRampDownClock(ALP_Header.RampDownClock, uiSPUChannel);

	SPU_Enable_Channel(uiSPUChannel);
	SPU_Clear_Ch_StopFlag(uiSPUChannel);

	if(temp_BeatIRQ_Enable_Flag)	//080805 tangqt
			SPU_Enable_BeatIRQ();
	return(uiSPUChannel);
}

void SPU_PlayPCM_NoEnv_FixCH(INT32U *pAddr, INT8U uiPan, INT8U uiVelocity, INT8U uiSPUChannel)
{
	INT8U  temp_BeatIRQ_Enable_Flag;

	if(uiSPUChannel >= SPU_ChannelNumber)
		return;

	if(SPU_Get_SingleChannel_Status(uiSPUChannel))	//080804
		SPU_StopChannel(uiSPUChannel);

	temp_BeatIRQ_Enable_Flag = SPU_Get_BeatIRQ_Enable_Flag();	//080805 tangqt
	SPU_Disable_BeatIRQ();		//080805 tangqt

	SPU_CLEAR_CHANNEL_ATT_SRAM(uiSPUChannel);//090527
	Load_ALP_Header(pAddr);
	SPU_EnvelopeManualMode((INT8U)uiSPUChannel);	//set Manual Mode
	//uiAddr = ((INT32U)(pAddr + 10) / 2);
        uiAddr = ((INT32U)(pAddr + 10));
        uiAddr = uiAddr / 2;
	SPU_SetStartAddress(uiAddr, uiSPUChannel);
	SPU_ClearADCPM36_Mode(uiSPUChannel);			// 20080521 Roy

	if(ALP_Header.WaveType & 0x0040)				// ADPCM mode
	{
		// ADPCM mode, check!
		SPU_SetADPCM_Mode(uiSPUChannel);			// set ADPCM mode
		if(ALP_Header.WaveType & 0x0080)			// ADPCM36 mode
		{
			SPU_SetWaveData_0(0x8000, uiSPUChannel);	// 20080521 Roy
			SPU_SelectADPCM36_Mode(uiSPUChannel);		// set ADPCM36 mode
			SPU_SetADPCM_PointNumber(31, uiSPUChannel);     // set point number = 31
			SPU_SetToneColorMode(1, uiSPUChannel);		// HW auto-repeat mode 1
		}
		else
		{
			SPU_SelectADPCM_Mode(uiSPUChannel);			// set ADPCM mode
			SPU_SetADPCM_PointNumber(0, uiSPUChannel);
			SPU_SetToneColorMode(1, uiSPUChannel);
			uiAddr += 1;
			SPU_SetStartAddress(uiAddr, uiSPUChannel);
		}
		if(ALP_Header.WaveType & 0x0010)				// check loop mode
		{
			SPU_Set_16bit_Mode(uiSPUChannel);			// set 16-bit PCM mode
		}
		else
			SPU_Set_8bit_Mode(uiSPUChannel);			// set 8-bit PCM mode
	}
	else	// PCM mode
	{
		SPU_SetPCM_Mode(uiSPUChannel);					// set PCM mode
		if(ALP_Header.WaveType & 0x0010)				// 16-bit PCM mode
		{
			SPU_Set_16bit_Mode(uiSPUChannel);			// set 16-bit PCM mode
		}
		else
			SPU_Set_8bit_Mode(uiSPUChannel);			// set 8-bit PCM mode
		SPU_SetToneColorMode(1, uiSPUChannel);			// HW auto-end mode
		//SPU_SetToneColorMode(2, uiSPUChannel);//// HW auto-repeat mode
	}
	SPU_SetWaveData_0(0x8000, uiSPUChannel);
	SPU_SetWaveData(*(pAddr + 10), uiSPUChannel);		// tone color first sample

	SPU_SetLoopAddress(0x0000, uiSPUChannel);
	SPU_SetVelocity(uiVelocity, uiSPUChannel);
	SPU_SetPan(uiPan, uiSPUChannel);

	// Play With No Envelop
	SPU_SetEnvelope_0(0, uiSPUChannel);
	SPU_SetEnvelope_1(0, uiSPUChannel);
	SPU_SetEnvelopeData(0x007F, uiSPUChannel);
	SPU_SetEnvelopeRampDownOffset(ALP_Header.RampDownStep, uiSPUChannel);
	SPU_SetEnvelopeRepeatAddrOffset(0x0000, uiSPUChannel);

	SPU_SetEnvelopeAddress(0, uiSPUChannel);
	SPU_SetWaveData(0x8000, uiSPUChannel);

	pTableAddr = (INT32U *)T_VarPhaseTable;
	uiData = *pTableAddr;
	uiPhaseLow = (uiData & 0xFFFF) * (ALP_Header.SampleRate & 0xFFFF);
	uiPhaseMiddle1 = (uiData & 0xFFFF) * (ALP_Header.SampleRate >> 16);
	uiPhaseMiddle2 = (uiData >> 16) * (ALP_Header.SampleRate & 0xFFFF);
	uiPhaseHigh = (uiData >> 16) * (ALP_Header.SampleRate >> 16);
	uiPhaseLow = (uiPhaseLow + 0x00008000) >> 16;
	uiPhaseMiddle1 += uiPhaseMiddle2;
	uiPhaseMiddle1 += uiPhaseLow;
	uiPhaseHigh = uiPhaseHigh << 16;
	uiPhaseHigh += uiPhaseMiddle1;

        #if 1// williamyeoadded, patch the phase to sync with I2S
        if (SPU_GetPostWaveI2SSync())
          uiPhaseHigh = SPU_CalculateI2SPhase(ALP_Header.SampleRate);
        #endif

        // uiPhaseHigh = (uiPhaseHigh * 480)/441; // TEST

	SPU_SetPhase(uiPhaseHigh, uiSPUChannel);
	SPU_SetPhaseAccumulator(0x0000, uiSPUChannel);
	SPU_SetTargetPhase(0x0000, uiSPUChannel);
	SPU_SetPhaseOffset(0x0000, uiSPUChannel);
	SPU_SetPhaseIncrease(uiSPUChannel);
	SPU_SetPhaseTimeStep(0x0000, uiSPUChannel);
	SPU_SetRampDownClock(ALP_Header.RampDownClock, uiSPUChannel);

        SPU_Enable_FIQ_Channel(uiSPUChannel);
	SPU_Enable_Channel(uiSPUChannel);
	SPU_Clear_Ch_StopFlag(uiSPUChannel);

 	if(temp_BeatIRQ_Enable_Flag)	//080805 tangqt
            SPU_Enable_BeatIRQ();
}

void SPU_pause_channel(INT8U spu_channel)
{
	INT32U temp;

	if(spu_channel >= SPU_ChannelNumber)
		return;

	temp = SPU_ReadPhase(spu_channel);
	if(temp != 0)
	{
		R_channel_original_phase[spu_channel] = temp;
		SPU_SetPhase(0, spu_channel);
	}
}
void SPU_resume_channel(INT8U spu_channel)
{
	INT32U temp;

	if(spu_channel >= SPU_ChannelNumber)
		return;

	temp = SPU_ReadPhase(spu_channel);
	if((temp == 0) && (R_channel_original_phase[spu_channel]!=0))
	{
		SPU_SetPhase(R_channel_original_phase[spu_channel], spu_channel);
		R_channel_original_phase[spu_channel] = 0;
	}
}

void MidiSRAM_Initial(void)
{
	INT32S i;
	User_FIQ_Flag &= ~B_SPU_BEAT_FIQ;//081125
	R_Total_Voice = SPU_ChannelNumber;
	R_MIDI_EndFlag = 0x0000;
	R_CH_NoteOff = 0x00000000;
	EventIndex = 0x0000;
	R_PlayChannel = 0x0000;
	R_CH_OneBeat = 0x0000;
	R_NoteOnHistPtr = 0x0000;
	R_Avail_Voice = SPU_ChannelNumber;

	for(i=0;i<SPU_ChannelNumber;i++)//100817
	{
		T_channel[i] = 0xff;
	}
}

void MidiPlay_Initialize()
{
	INT32U ChannelIndex;

	MidiSRAM_Initial();
	MIDI_Tempo = C_DefaultTempo;
	pMIDI_ChInfo = (MIDI_ChannelInfoStruct *)MIDI_ChannelInfo;
	pSPU_ChInfo = (SPU_ChannelInfoStruct *)SPU_ChannelInfo;
	for(ChannelIndex = 0; ChannelIndex < MIDI_ChannelNumber; ChannelIndex++)
	{
		(pMIDI_ChInfo + ChannelIndex)->R_MIDI_CH_PAN = 64;
		(pMIDI_ChInfo + ChannelIndex)->R_MIDI_CH_VOLUME = 127;
		(pMIDI_ChInfo + ChannelIndex)->R_MIDI_CH_EXPRESSION = 127;
		(pMIDI_ChInfo + ChannelIndex)->R_MIDI_CH_PitchBend = 64;
		(pMIDI_ChInfo + ChannelIndex)->R_CH_SMFTrack = 0x0000;
		(pMIDI_ChInfo + ChannelIndex)->R_ChannelInst &= 0xffff0000;//081029
		(pMIDI_ChInfo + ChannelIndex)->R_PB_TABLE_Addr = (INT32U *)T_PicthWheelTable_TWO;
		(pMIDI_ChInfo + ChannelIndex)->R_RPN_ReceiveFlag = 0x0000;
		(pMIDI_ChInfo + ChannelIndex)->R_RPN_DATA = 0x0000;
	}

	for(ChannelIndex = 0; ChannelIndex < SPU_ChannelNumber; ChannelIndex++)
	{
		(pSPU_ChInfo + ChannelIndex)->R_NOTE_PITCH = 0;//081108
		(pSPU_ChInfo + ChannelIndex)->R_NOTE_VELOCITY = 127;
		(pSPU_ChInfo + ChannelIndex)->R_MIDI_ToneLib_PAN = 64;
		(pSPU_ChInfo + ChannelIndex)->R_MIDI_ToneLib_Volume = 127;
		(pSPU_ChInfo + ChannelIndex)->R_MIDI_CH_MAP = 0xFFFF;
		(pSPU_ChInfo + ChannelIndex)->R_NoteOnHist = 0x0000;
		(pSPU_ChInfo + ChannelIndex)->R_PB_PhaseRecord = 0x0000;
	}
	for(ChannelIndex = 0; ChannelIndex < SPU_ChannelNumber + 1; ChannelIndex++)
	{
		R_DUR_Tone[ChannelIndex] = 0x0000;
	}
	//---------------------------------------------------------------------------090527
	for(ChannelIndex=0; ChannelIndex<SPU_ChannelNumber; ChannelIndex++)
	{
		if(R_MIDI_CH_MASK & (1<<ChannelIndex))
		{
			SPU_CLEAR_CHANNEL_ATT_SRAM(ChannelIndex);//090527
		}
	}
	//---------------------------------------------------------------------------
	R_MIDI_EndFlag = 0x0000;
}

void SPU_MIDI_Play(INT8U repeat_en)//repeat_en: 1 repeat;   0 no repeat
{
	SPU_MIDI_Stop();	//080709 tangqt
	MIDI_PlayFlag &= ~STS_MIDI_PAUSE_ON;
	MIDI_PlayFlag |= STS_MIDI_PLAY;
	if(repeat_en)
		MIDI_PlayFlag |= STS_MIDI_REPEAT_ON;
	else
		MIDI_PlayFlag &= ~STS_MIDI_REPEAT_ON;
	pMIDI_DataPointer = (INT8U*)(midi_ring_buffer_addr + midi_ring_buffer_ri);//081105
	pMIDI_StartAddr = (INT8U*)midi_ring_buffer_addr;
	SPU_Set_BeatBaseCounter(C_DefaultBeatBase);
	MidiPlay_Initialize();
	SPU_Set_BeatCounter(0x30);
	SPU_Clear_BeatIRQ_Flag();
	SPU_Enable_BeatIRQ();
	SPU_EnableBeatCountFIQ();
}


void SPU_MIDI_Stop(void)
{
	INT32U ChannelIndex;
	INT32U Temp;

	MIDI_Current_Dt = 0;  	//20081028 Roy
	MIDI_Stop_Dt = 0;     	//20081028 Roy

	SPU_Clear_BeatIRQ_Flag();
	SPU_Disable_BeatIRQ();
	SPU_Set_BeatCounter(0x00);
	MIDI_PlayFlag &= ~(STS_MIDI_PLAY | STS_MIDI_REPEAT_ON | STS_MIDI_PAUSE_ON);//080709 tangqt

	while( SPU_Get_EnvRampDown() !=0);

	R_CH_NoteOff = 0x00000000;//0x0000;
	Temp = R_MIDI_CH_MASK;
	for(ChannelIndex = 0; ChannelIndex < SPU_ChannelNumber; ChannelIndex++)
	{
		if(Temp & 0x0001)
		{
			SPU_StopChannel(ChannelIndex);
		}
		Temp = Temp >> 1;
	}
}

//--------------------------------------------------------------------------------------------
void SPU_MIDI_Pause(void)//080709 tangqt
{
	INT32U SPU_ChannelIndex;
	INT32U Temp;

	if(MIDI_PlayFlag & STS_MIDI_PLAY)
	{
		SPU_Disable_BeatIRQ();
		SPU_Clear_BeatIRQ_Flag();
		SPU_Set_BeatCounter(0x00);
		R_CH_NoteOff = 0x00000000;//0x0000;
		Temp = R_MIDI_CH_MASK;
		for(SPU_ChannelIndex = 0; SPU_ChannelIndex < SPU_ChannelNumber; SPU_ChannelIndex++)
		{
			if(Temp & 0x0001)
			{
				SPU_StopChannel(SPU_ChannelIndex);
			}
			Temp = Temp >> 1;
		}
		MIDI_PlayFlag |= STS_MIDI_PAUSE_ON;
	}
}

void SPU_MIDI_Resume(void)//080709 tangqt
{
	if(MIDI_PlayFlag & STS_MIDI_PAUSE_ON)
	{
		MIDI_PlayFlag &= ~STS_MIDI_PAUSE_ON;
		if(MIDI_PlayFlag & STS_MIDI_PLAY)
		{
			SPU_Clear_BeatIRQ_Flag();
			SPU_Set_BeatCounter(0x00);
			SPU_Enable_BeatIRQ();
		}
	}
}

void SPU_Initial(void)
{
	INT32S i;
	SPU_Init();//layer 1
	for(i=0;i<SPU_ChannelNumber;i++)
	{
		R_channel_original_phase[i] = 0;
	}

	SPU_MIDI_Set_SPU_Channel_Mask(0xFFFC);	//16 channel: CH0 and CH1 for softchannel of background audio

	MidiPlay_Initialize();
	SPU_MIDI_SetMidiMask(0x0000FFFF);
	SPU_MIDI_SetMidiCH_ChangeColor_Mask(0x00000000);
	SPU_Set_MainVolume(127);//set spu global volume
	SPU_MIDI_Set_MIDI_Volume(0x3f);//080709
	SPU_MIDI_Set_MIDI_Pan(63);//080808
	//SPU_SignedPCM16ON();
        SPU_Free_Midi();

	for(i = 0; i<500; i++)
	{
		T_InstrumentSectionAddr_1[i] = 0;
	}

        //SPU_EnableChannelFIQ();
        //SPU_FIQ_Enable(VIC_SPU_FIQ);
}

void SPU_MIDI_Set_SPU_Channel_Mask(INT32U SPU_CH_MASK)//080709 tangqt
{
	R_MIDI_CH_MASK = SPU_CH_MASK;
}

void SPU_MIDI_Set_MIDI_Volume(INT32U MIDI_Volume)//080709 tangqt
{
	INT32U temp,spu_channel;
	INT32U PanWord;

	R_MIDI_Volume = (INT8U)MIDI_Volume;
	if(R_MIDI_Volume > 127)
		R_MIDI_Volume = 127;
	for(temp = 0x0001,spu_channel = 0; spu_channel < SPU_ChannelNumber; spu_channel++,temp <<= 1)
	{
		if(temp & R_MIDI_CH_MASK)
		{
			PanWord = Calculate_Pan(spu_channel);
			SPU_SetVelocity( (INT8U)(PanWord&0x007f), (INT8U)spu_channel );
			SPU_SetPan((INT8U)((PanWord>>8)&0x007f), (INT8U)spu_channel );
		}
	}
}

void SPU_MIDI_Set_MIDI_Pan(INT32U MIDI_Pan)//080808 tangqt
{
	INT32U temp,spu_channel;
	INT32U PanWord;


	R_MIDI_Pan = (INT8U)MIDI_Pan;
	if(R_MIDI_Pan > 127)
		R_MIDI_Pan = 127;
	for(temp = 0x0001,spu_channel = 0; spu_channel < SPU_ChannelNumber; spu_channel++,temp <<= 1)
	{
		if(temp & R_MIDI_CH_MASK)
		{
			PanWord = Calculate_Pan(spu_channel);
			SPU_SetVelocity( (INT8U)(PanWord&0x007f), (INT8U)spu_channel );
			SPU_SetPan((INT8U)((PanWord>>8)&0x007f), (INT8U)spu_channel );
		}
	}
}

INT32U SPU_MIDI_Get_Status(void)
{
	return(MIDI_PlayFlag);
}

INT8U SPU_Get_SingleChannel_Status(INT8U SPU_Channel)
{
	INT32U temp;
	temp = SPU_GetChannelStatus();
	if(temp & (0x00000001 << SPU_Channel))
		return(0x01);
	else
		return(0x00);
}
//--------------------------------------------------------------------------------------------
void SPU_MIDI_SetMidiMask(INT32U MIDI_Ch_Mask)
{
	MIDI_Ch_Mask &= 0xFFFF;
	R_SourceMIDIMask = MIDI_Ch_Mask;
}

void SPU_MIDI_SetMidiCH_ChangeColor_Mask(INT32U MIDI_Ch_Mask)
{
	MIDI_Ch_Mask &= 0x0000FFFF;
	R_SourceMIDI_ChangeColor_Mask = MIDI_Ch_Mask;
}

void SPU_MIDI_SetMidiCH_ChangeColor(INT32U MIDI_CH, INT32U color_index)
{
	MIDI_ChannelInfoStruct *p_MIDI_ChInfo;
	INT32U temp;

	p_MIDI_ChInfo = (MIDI_ChannelInfoStruct *)MIDI_ChannelInfo;
	temp = (p_MIDI_ChInfo + MIDI_CH)->R_ChannelInst;
	temp &= 0x0000ffff;
	temp |= color_index << 16;
	(p_MIDI_ChInfo + MIDI_CH)->R_ChannelInst = temp;
}

void SPU_MIDI_Service(void)
{
	F_StopExpiredCH();
	if(R_DUR_Tone[SPU_ChannelNumber] != 0)
	{
		F_CheckDuration();
	}
	else
	{
		F_GetSeqCmd();
	}
	SPU_Clear_BeatIRQ_Flag();
	SPU_Enable_BeatIRQ();
}

void F_CheckDuration(void)
{
	INT16U MinDuration;

	INT32U Ch;
	INT32S Temp;
	INT32U *pAddr;
	MinDuration = 0x0000;

	for(Ch = 0; Ch < SPU_ChannelNumber + 1; Ch++)
	{
		if(R_DUR_Tone[Ch] != 0x0000)
		{
			if(MinDuration == 0x0000)
				MinDuration = R_DUR_Tone[Ch];
			else
			{
				if(R_DUR_Tone[Ch] < MinDuration)
					MinDuration = R_DUR_Tone[Ch];
			}
		}
	}
	R_CH_NoteOff = 0x00000000;//0x0000;
	for(Ch = 0; Ch < SPU_ChannelNumber + 1; Ch++)
	{
		if(R_DUR_Tone[Ch] != 0x0000)
		{
			Temp = (INT32S)(R_DUR_Tone[Ch] - MinDuration);
			if(Temp <= 0)
			{
				if(Ch < SPU_ChannelNumber)
				{
					Temp = 0;
					pAddr = (INT32U *)T_BitEnable;
					R_CH_NoteOff |= *(pAddr + Ch);
				}
			}
			R_DUR_Tone[Ch] = Temp;
		}
	}
	R_CH_NoteOff &= R_MIDI_CH_MASK;
	SPU_Set_BeatCounter(MinDuration);
}

void SPU_FIQ_ISR(void)
{
	if(User_FIQ_Flag & B_SPU_FIQ)
	{
		(*SPU_User_FIQ_ISR[C_SPU_FIQ])();
	}
}

void SPU_ENV_ISR(void)
{
	if(User_FIQ_Flag & B_SPU_ENV_FIQ)
	{
		(*SPU_User_FIQ_ISR[C_SPU_ENV_FIQ])();
	}
}

void SPU_BEAT_ISR(void)
{
	INT8U  bCallback;
	//INT32U Temp,ChannelIndex;

	if((MIDI_Current_Dt >= MIDI_Stop_Dt) && (MIDI_Stop_Dt != 0))	//20081028 Roy
	{
		bCallback = 1;
		MIDI_PlayFlag = 0;
		R_MIDI_EndFlag = 0xFFFF;
	}
	else
		bCallback = 0;

	if(MIDI_PlayFlag & STS_MIDI_PLAY)
	{
		SPU_Disable_BeatIRQ();
		SPU_MIDI_Service();
	}
	else
	{
		if(R_MIDI_EndFlag == 0xFFFF)
		{
			R_MIDI_EndFlag = 0x0000;
			SPU_Disable_BeatIRQ();
			SPU_Clear_BeatIRQ_Flag();

			MIDI_PlayFlag &= ~(STS_MIDI_PLAY | STS_MIDI_REPEAT_ON | STS_MIDI_PAUSE_ON);//080709 tangqt
/*
			R_CH_NoteOff = 0x00000000;
			Temp = R_MIDI_CH_MASK;
			for(ChannelIndex = 0; ChannelIndex < SPU_ChannelNumber; ChannelIndex++)
			{
				if(Temp & 0x0001)
				{
					SPU_StopChannel(ChannelIndex);
				}
				Temp = Temp >> 1;
			}
*/
		}
	}

	if(User_FIQ_Flag & B_SPU_BEAT_FIQ)
		(*SPU_User_FIQ_ISR[C_SPU_BEAT_FIQ])();

	if(bCallback && MIDI_PlayDtStopCallBack != NULL)	//20081028 Roy
		(*MIDI_PlayDtStopCallBack)();

}

void SPU_PW_ISR(void)
{
	if(User_FIQ_Flag & B_SPU_PW_FIQ)
	{
		(*SPU_User_FIQ_ISR[C_SPU_PW_FIQ])();
	}
}

void SPUFIQ_IRQHandler(void)
{
	SPU_FIQ_ISR();
}

void SPUENV_IRQHandler(void)
{
	SPU_ENV_ISR();
}

void SPUPW_IRQHandler(void)
{
	SPU_PW_ISR();
}

void SPUBEAT_IRQHandler(void)
{
	SPU_BEAT_ISR();
}

void SPU_EnableChannelFIQ(void)
{
	SPU_FIQ_Register(C_SPU_FIQ, SPU_FIQ_ISR);
	SPU_FIQ_Enable(C_SPU_FIQ);
}

void SPU_DisableChannelFIQ(void)
{
	SPU_FIQ_Enable(C_SPU_FIQ);
}

void SPU_EnableEnvelopeFIQ(void)
{
	SPU_FIQ_Register(C_SPU_ENV_FIQ, SPU_ENV_ISR);
	SPU_FIQ_Enable(C_SPU_ENV_FIQ);
}

void SPU_DisableEnvelopeFIQ(void)
{
	SPU_FIQ_Disable(C_SPU_ENV_FIQ);
}

void SPU_EnableBeatCountFIQ(void)
{
	SPU_FIQ_Register(C_SPU_BEAT_FIQ, SPU_BEAT_ISR);
	SPU_FIQ_Enable(C_SPU_BEAT_FIQ);
}

void SPU_DisableBeatCountFIQ(void)
{
	SPU_FIQ_Disable(C_SPU_BEAT_FIQ);
}

void SPU_EnablePostWaveFIQ(void)
{
	SPU_FIQ_Register(C_SPU_PW_FIQ, SPU_PW_ISR);
	SPU_FIQ_Enable(C_SPU_PW_FIQ);
}

void SPU_DisablePostWaveFIQ(void)
{
	SPU_FIQ_Disable(C_SPU_PW_FIQ);
}


//-------------------------------------------------------------080910
void TM_PlayTone(INT32U uiMidiCh, INT8U uiPitch, INT8U uiPan, INT8U uiVelocity)
{
	INT32U MIDI_Ch, SPU_Ch;
	INT32U R_NoteVolume;
	INT32U R_NotePitch;
	INT32U InstrumentIndex;
	INT32U MaxPitch, R_BasePitch;
	INT32U SectionLow, SectionHigh, SectionIndex;
	INT32U *pAddr;
	INT32U PanWord;
	INT32U ChannelMask;
	INT32U temp_BeatIRQ_Enable_Flag;

	MIDI_Ch = uiMidiCh;
	R_NoteVolume = uiVelocity;
	R_NotePitch = uiPitch;
	pAddr = (INT32U *)T_BitEnable;
	ChannelMask = *(pAddr + MIDI_Ch);

	temp_BeatIRQ_Enable_Flag = SPU_Get_BeatIRQ_Enable_Flag();//080805 tangqt
	SPU_Disable_BeatIRQ();//080805 tangqt

			SPU_Ch = FindEmptyChannel();
			(pMIDI_ChInfo + MIDI_Ch)->R_CH_SMFTrack = SPU_Ch;
			(pSPU_ChInfo + SPU_Ch)->R_MIDI_CH_MAP = MIDI_Ch;
			(pSPU_ChInfo + SPU_Ch)->R_NOTE_VELOCITY = uiVelocity;
			(pSPU_ChInfo + SPU_Ch)->R_NOTE_PITCH = uiPitch;//081028
			R_DUR_Tone[SPU_Ch] = 0x0000ffff;//MIDI_Event.NoteEvent.Duration;

				InstrumentIndex = ((pMIDI_ChInfo + MIDI_Ch)->R_ChannelInst) >> 16;
				pAddr = (INT32U *)T_InstrumentStartSection_1;
				SectionLow = *(pAddr + InstrumentIndex) & 0x0000ffff;
				SectionHigh = *(pAddr + InstrumentIndex + 1) & 0x0000ffff;
				for(SectionIndex = SectionLow; SectionIndex < SectionHigh; SectionIndex++)
				{
					pAddr = (INT32U *)T_InstrumentPitchTable_1;
					MaxPitch = *(pAddr + SectionIndex) & 0x00FF;
					if(R_NotePitch <= MaxPitch)
						break;
				}
				if(SectionIndex >= SectionHigh)
				{
					SectionIndex = SectionHigh - 1;
				}
				R_BasePitch = *(pAddr + SectionIndex) >> 8;
				pAddr = (INT32U *)(T_InstrumentSectionAddr_1 + SectionIndex);

			Load_ALP_Header((INT32U *)(*pAddr));
			(pSPU_ChInfo + SPU_Ch)->R_MIDI_ToneLib_PAN = ALP_Header.Pan;
			(pSPU_ChInfo + SPU_Ch)->R_MIDI_ToneLib_Volume = ALP_Header.Velocity;
			PanWord = (uiPan<<8) | uiVelocity;
			MIDI_Event.NoteEvent.ChannelNumber = MIDI_Ch;//081028
			SPU_PlayNote(uiPitch, (INT32U *)(*pAddr), uiPan, uiVelocity, SPU_Ch);

			SPU_Clear_Ch_StopFlag(SPU_Ch);
			SPU_Enable_Channel(SPU_Ch);

	if(temp_BeatIRQ_Enable_Flag)//080805 tangqt
		SPU_Enable_BeatIRQ();
}

void TM_StopTone(INT32U uiMidiCh, INT8U uiPitch)
{
	INT32U MIDI_Ch, SPU_Ch;
	INT32U R_NotePitch;
	INT32U *pAddr;
	INT32U ChannelMask;
	INT32U temp;

	MIDI_Ch = uiMidiCh;
	R_NotePitch = uiPitch;
	pAddr = (INT32U *)T_BitEnable;

	for(SPU_Ch=0; SPU_Ch<SPU_ChannelNumber; SPU_Ch++)
	{
		if((pSPU_ChInfo + SPU_Ch)->R_NOTE_PITCH == uiPitch)
		{
			if((pSPU_ChInfo + SPU_Ch)->R_MIDI_CH_MAP == uiMidiCh)
			{
				ChannelMask = *(pAddr + SPU_Ch);
				temp = SPU_Get_EnvRampDown();
				temp |= ChannelMask;
				SPU_Set_EnvRampDownMultiChannel(temp);
				(pSPU_ChInfo + SPU_Ch)->R_MIDI_CH_MAP = 0xFFFF;
				(pSPU_ChInfo + SPU_Ch)->R_NOTE_PITCH = 0;//081028
				R_DUR_Tone[SPU_Ch] = 0;
				break;
			}
		}
	}
}

static INT32U SPU_get_midi_ring_buffer_ri(void)
{
	INT32U temp;
	temp = (INT32U)(pMIDI_DataPointer) - (INT32U)midi_ring_buffer_addr;
	return temp;
}

static INT16U get_a_word(void)
{
	INT16U data_temp;
	INT8U temp,temp1;

	temp = *pMIDI_DataPointer++;//low 8 bit//081031
	if( (INT32U)pMIDI_DataPointer >= ((INT32U)midi_ring_buffer_addr + MIDI_RING_BUFFER_SIZE) )//081031
		pMIDI_DataPointer = (INT8U *)midi_ring_buffer_addr;//081031
	temp1 = *pMIDI_DataPointer++;//high 8 bit//081031
	if( (INT32U)pMIDI_DataPointer >= ((INT32U)midi_ring_buffer_addr + MIDI_RING_BUFFER_SIZE) )//081031
		pMIDI_DataPointer = (INT8U *)midi_ring_buffer_addr;//081031
	data_temp = (INT16U)temp | (((INT16U)temp1)<<8);//081031
	return data_temp;

}
//-------------------------------------------------------------
void F_GetSeqCmd(void)
{
	INT16U MIDI_Data, i;

	MIDI_PlayFlag &= ~C_MIDI_Delay;
	do
	{
		if(MIDI_Skip_Flag)	//20081031
			SPU_check_fill_midi_ring_buffer();

		MIDI_Data = get_a_word();//*pMIDI_DataPointer++;
		EventIndex = (MIDI_Data & 0xF000) >> 12;
		switch(EventIndex)
		{
			case C_NoteEvent:
				MIDI_Event.NoteEvent.ChannelNumber = MIDI_Data & 0x000F;
				MIDI_Data = get_a_word();//*pMIDI_DataPointer++;
				MIDI_Event.NoteEvent.Pitch = (MIDI_Data & 0x7F00) >> 8;
				MIDI_Event.NoteEvent.Velocity = MIDI_Data & 0x007F;
				MIDI_Data = get_a_word();//*pMIDI_DataPointer++;
				MIDI_Event.NoteEvent.Duration = MIDI_Data;

				if(MIDI_Skip_Flag == 0)		//20081028 Roy
					ProcessNoteEvent();

				break;

			case C_BeatCountEvent:
				MIDI_Event.BeatCountEvent.BeatCountValue = MIDI_Data & 0x7FF;
				if(MIDI_Data & 0x0800)
				{
					MIDI_Data = get_a_word();//*pMIDI_DataPointer++;
					MIDI_Event.BeatCountEvent.BeatCountValue |= ((MIDI_Data &= 0x0007) << 11);
				}

				MIDI_Current_Dt += MIDI_Event.BeatCountEvent.BeatCountValue;	//20081028 Roy
				if(MIDI_Skip_Flag == 0)											//20081028 Roy
					ProcessBeatCountEvent();
				MIDI_PlayFlag |= C_MIDI_Delay;
				break;

			case C_PitchBendEvent:
				MIDI_Event.PitchBendEvent.ChannelNumber = (MIDI_Data & 0x0F00) >> 8;
				MIDI_Event.PitchBendEvent.PitchBendValue = MIDI_Data & 0x007F;
				MIDI_Data = get_a_word();//*pMIDI_DataPointer++;
				ProcessPitchBendEvent();
				break;

			case C_ControlEvent:
				MIDI_Event.ControlEvent.ChannelNumber = (MIDI_Data & 0x0F00) >> 8;
				MIDI_Event.ControlEvent.ControlNumber = MIDI_Data & 0x007F;
				MIDI_Data = get_a_word();//*pMIDI_DataPointer++;
				MIDI_Event.ControlEvent.ControlValue = MIDI_Data & 0x007F;
				ProcessControlEvent();
				break;

			case C_ProgramChangeEvent:
				MIDI_Event.ProgramChangeEvent.ChannelNumber = (MIDI_Data & 0x0F00) >> 8;
				MIDI_Event.ProgramChangeEvent.InstrumentIndex = MIDI_Data & 0x007F;
				ProcessProgramChangeEvent();
				break;

			case C_TempoEvent:
				MIDI_Event.TempoEvent.TempoValue = MIDI_Data & 0x00FF;
				break;

			case C_MIDI_EndEvent:
				ProcessEndEvent();
				break;

			case C_LyricEvent:
				MIDI_Event.LyricEvent.ChannelNumber = (MIDI_Data & 0x0F00) >> 8;
				MIDI_Event.LyricEvent.LyricWordCount = MIDI_Data & 0x007F;
				MIDI_Data = get_a_word();//*pMIDI_DataPointer++;
				MIDI_Event.LyricEvent.Duration = MIDI_Data;
				for(i = MIDI_Event.LyricEvent.LyricWordCount; i > 0; i--)
				{
					get_a_word();
				}
				break;

			case C_TextEvent:
				MIDI_Event.TextEvent.TextType = (MIDI_Data & 0x0F00) >> 8;
				MIDI_Event.TextEvent.LyricWordCount = MIDI_Data & 0x007F;
				break;

			default:
				break;
		};
	}while((!(MIDI_PlayFlag & C_MIDI_Delay)) && (MIDI_PlayFlag & STS_MIDI_PLAY));
}

void ProcessNoteEvent(void)
{
	INT32U MIDI_Ch, SPU_Ch;
	INT32U R_NoteVolume;
	INT32U InstrumentIndex;
	INT32U MaxPitch, R_BasePitch;
	INT32U SectionLow, SectionHigh, SectionIndex;
	INT32U *pAddr;
	INT32U PanWord;
	INT32U ChannelMask;

	MIDI_Ch = MIDI_Event.NoteEvent.ChannelNumber;
	R_NoteVolume = ((pMIDI_ChInfo + MIDI_Ch)->R_MIDI_CH_VOLUME * (pMIDI_ChInfo + MIDI_Ch)->R_MIDI_CH_EXPRESSION) / 128;
	pAddr = (INT32U *)T_BitEnable;
	ChannelMask = *(pAddr + MIDI_Ch);
	if(ChannelMask & R_SourceMIDIMask)
	{
		//Volume=CH_Volume * CH_Expression / 128
		if(MIDI_Ch == 9)
		{
			// play drum
			SPU_Ch = FindEmptyChannel();

			(pMIDI_ChInfo + MIDI_Ch)->R_CH_SMFTrack = SPU_Ch; // ??
			(pSPU_ChInfo + SPU_Ch)->R_MIDI_CH_MAP = MIDI_Ch; // ??
			(pSPU_ChInfo + SPU_Ch)->R_NOTE_VELOCITY = MIDI_Event.NoteEvent.Velocity;
			(pSPU_ChInfo + SPU_Ch)->R_NOTE_PITCH = MIDI_Event.NoteEvent.Pitch;//081024
			pAddr = (INT32U *)(T_DrumAddr + MIDI_Event.NoteEvent.Pitch);
			Load_ALP_Header((INT32U *)(*pAddr));
			(pSPU_ChInfo + SPU_Ch)->R_MIDI_ToneLib_PAN = ALP_Header.Pan;
			(pSPU_ChInfo + SPU_Ch)->R_MIDI_ToneLib_Volume = ALP_Header.Velocity;
			PanWord = Calculate_Pan(SPU_Ch);
			SPU_PlayDrum(MIDI_Event.NoteEvent.Pitch, (INT32U *)(*pAddr),
						(PanWord >> 8), (PanWord & 0x00FF), SPU_Ch);
			pAddr = (INT32U *)(T_BitEnable + SPU_Ch);
			R_CH_OneBeat |= *pAddr;
		}
		else	// not drum
		{
			//----------------------------------------------------------------------------//080710
			if(MIDI_Event.NoteEvent.Duration == 0)
				return;
			//----------------------------------------------------------------------------
			SPU_Ch = FindEmptyChannel();
			if(SPU_Ch == 0xffffffff)
				return;
			(pMIDI_ChInfo + MIDI_Ch)->R_CH_SMFTrack = SPU_Ch;
			(pSPU_ChInfo + SPU_Ch)->R_MIDI_CH_MAP = MIDI_Ch;
			(pSPU_ChInfo + SPU_Ch)->R_NOTE_VELOCITY = MIDI_Event.NoteEvent.Velocity;
			(pSPU_ChInfo + SPU_Ch)->R_NOTE_PITCH = MIDI_Event.NoteEvent.Pitch;//081024
			R_DUR_Tone[SPU_Ch] = MIDI_Event.NoteEvent.Duration;
			//for debug
			//SendString("\n\r  duration = ");
			//SendWord(R_DUR_Tone[SPU_Ch]);
			//end debug
			//----------------------------------------------------------------------------
			if(ChannelMask & R_SourceMIDI_ChangeColor_Mask)//切换音色 
			{
				InstrumentIndex = ((pMIDI_ChInfo + MIDI_Ch)->R_ChannelInst) >> 16;
				pAddr = (INT32U *)T_InstrumentStartSection_1;
				SectionLow = *(pAddr + InstrumentIndex) & 0x0000ffff;
				SectionHigh = *(pAddr + InstrumentIndex + 1) & 0x0000ffff;
				for(SectionIndex = SectionLow; SectionIndex < SectionHigh; SectionIndex++)
				{
					pAddr = (INT32U *)T_InstrumentPitchTable_1;
					MaxPitch = *(pAddr + SectionIndex) & 0x00FF;
					if(MIDI_Event.NoteEvent.Pitch <= MaxPitch)
						break;
				}
				if(SectionIndex >= SectionHigh)
				{
					SectionIndex = SectionHigh - 1;
				}
				R_BasePitch = *(pAddr + SectionIndex) >> 8;
				pAddr = (INT32U *)(T_InstrumentSectionAddr_1 + SectionIndex);
			}
			else
			{
				InstrumentIndex = ((pMIDI_ChInfo + MIDI_Ch)->R_ChannelInst) & 0x0000ffff;
				pAddr = (INT32U *)T_InstrumentStartSection;
				SectionLow = *(pAddr + InstrumentIndex);
				SectionHigh = *(pAddr + InstrumentIndex + 1);
				for(SectionIndex = SectionLow; SectionIndex < SectionHigh; SectionIndex++)
				{
					pAddr = (INT32U *)T_InstrumentPitchTable;
					MaxPitch = *(pAddr + SectionIndex) & 0x00FF;
					if(MIDI_Event.NoteEvent.Pitch <= MaxPitch)
						break;
				}
				if(SectionIndex >= SectionHigh)
				{
					SectionIndex = SectionHigh - 1;
				}
				R_BasePitch = *(pAddr + SectionIndex) >> 8;
				pAddr = (INT32U *)(T_InstrumentSectionAddr + SectionIndex);
			}
			Load_ALP_Header((INT32U *)(*pAddr));
			(pSPU_ChInfo + SPU_Ch)->R_MIDI_ToneLib_PAN = ALP_Header.Pan;
			(pSPU_ChInfo + SPU_Ch)->R_MIDI_ToneLib_Volume = ALP_Header.Velocity;
			PanWord = Calculate_Pan(SPU_Ch);

			SPU_PlayNote(MIDI_Event.NoteEvent.Pitch, (INT32U *)(*pAddr),
						(PanWord >> 8), (PanWord & 0x00FF), SPU_Ch);
			pAddr = (INT32U *)(T_BitEnable + SPU_Ch);
			R_CH_OneBeat |= *pAddr;
		}
	}
}

void ProcessControlEvent(void)
{
	INT32U ChannelNumber, ControlNumber, ControlValue;
	INT32U ChannelIndex;
	INT32U PanWord;
	INT32U *pAddr;

	ChannelNumber = MIDI_Event.ControlEvent.ChannelNumber;
	ControlNumber = MIDI_Event.ControlEvent.ControlNumber;
	ControlValue = MIDI_Event.ControlEvent.ControlValue;
	switch(ControlNumber)
	{
		case C_VolumeEvent:
			(pMIDI_ChInfo + ChannelNumber)->R_MIDI_CH_VOLUME = ControlValue;
			for(ChannelIndex = 0; ChannelIndex < SPU_ChannelNumber; ChannelIndex++)
			{
				if((pSPU_ChInfo + ChannelIndex)->R_MIDI_CH_MAP == ChannelNumber)
				{
					PanWord = Calculate_Pan(ChannelIndex);
					SPU_SetVelocity(PanWord & 0x00FF, ChannelIndex);
					SPU_SetPan(PanWord >> 8, ChannelIndex);
				}
			}
			break;

		case C_PanEvent:
			(pMIDI_ChInfo + ChannelNumber)->R_MIDI_CH_PAN = ControlValue;
			for(ChannelIndex = 0; ChannelIndex < SPU_ChannelNumber; ChannelIndex++)
			{
				if((pSPU_ChInfo + ChannelIndex)->R_MIDI_CH_MAP == ChannelNumber)
				{
					PanWord = Calculate_Pan(ChannelIndex);
					SPU_SetVelocity(PanWord & 0x00FF, ChannelIndex);
					SPU_SetPan(PanWord >> 8, ChannelIndex);
				}
			}
			break;

		case C_ExpressionEvent:
			(pMIDI_ChInfo + ChannelNumber)->R_MIDI_CH_EXPRESSION = ControlValue;
			for(ChannelIndex = 0; ChannelIndex < SPU_ChannelNumber; ChannelIndex++)
			{
				if((pSPU_ChInfo + ChannelIndex)->R_MIDI_CH_MAP == ChannelNumber)
				{
					PanWord = Calculate_Pan(ChannelIndex);
					SPU_SetVelocity(PanWord & 0x00FF, ChannelIndex);
					SPU_SetPan(PanWord >> 8, ChannelIndex);
				}
			}
			break;

		case C_RPN_LSB_Event:
			(pMIDI_ChInfo + ChannelNumber)->R_RPN_ReceiveFlag |= 0x0F00;
			(pMIDI_ChInfo + ChannelNumber)->R_RPN_DATA &= 0xFF00;
			(pMIDI_ChInfo + ChannelNumber)->R_RPN_DATA |= ControlValue;
			break;

		case C_RPN_MSB_Event:
			(pMIDI_ChInfo + ChannelNumber)->R_RPN_ReceiveFlag |= 0xF000;
			(pMIDI_ChInfo + ChannelNumber)->R_RPN_DATA &= 0x00FF;
			(pMIDI_ChInfo + ChannelNumber)->R_RPN_DATA |= ControlValue;
			break;

		case C_DataEntryEvent:
			if((MIDI_Skip_Flag == 0)&&(MIDI_DataEntryEventCallBack != NULL))	//20081031	Roy
			{
				MIDI_Control_Event[0] = ChannelNumber;
				MIDI_Control_Event[1] = ControlValue;
				MIDI_Control_Event[2] = 0;
				MIDI_Control_Event[3] = 0;
				(*MIDI_DataEntryEventCallBack)();
			}

			if((pMIDI_ChInfo + ChannelNumber)->R_RPN_ReceiveFlag == 0xFF00)
			{
				(pMIDI_ChInfo + ChannelNumber)->R_RPN_ReceiveFlag = 0x0000;
				if((pMIDI_ChInfo + ChannelNumber)->R_RPN_DATA == 0x0000)
				{
					if(ControlValue <= D_PITCH_BEND_TABLE_TOTAL)
					{
						switch(ControlValue)
						{
							case 1:
								pAddr = (INT32U *)T_PicthWheelTable;
								break;
							case 2:
								pAddr = (INT32U *)T_PicthWheelTable_TWO;
								break;
							case 3:
								pAddr = (INT32U *)T_PicthWheelTable_THREE;
								break;
							case 4:
								pAddr = (INT32U *)T_PicthWheelTable_FOUR;
								break;
							case 5:
								pAddr = (INT32U *)T_PicthWheelTable_FIVE;
								break;
							case 6:
								pAddr = (INT32U *)T_PicthWheelTable_SIX;
								break;
							case 7:
								pAddr = (INT32U *)T_PicthWheelTable_SEVEN;
								break;
							case 8:
								pAddr = (INT32U *)T_PicthWheelTable_EIGHT;
								break;
							case 9:
								pAddr = (INT32U *)T_PicthWheelTable_NINE;
								break;
							case 10:
								pAddr = (INT32U *)T_PicthWheelTable_TEN;
								break;
							case 11:
								pAddr = (INT32U *)T_PicthWheelTable_ELEVEN;
								break;
							case 12:
								pAddr = (INT32U *)T_PicthWheelTable_TWELVE;
								break;
							case 13:
								pAddr = (INT32U *)T_PicthWheelTable_THIRTEEN;
								break;
							default:
								pAddr = (INT32U *)T_PicthWheelTable_TWO;
								break;
						}
						(pMIDI_ChInfo + ChannelNumber)->R_PB_TABLE_Addr = pAddr;
					}
				}
			}
			break;
	};
}

void ProcessPitchBendEvent(void)
{
	INT32U MIDIChannel, PitchBendIndex;
	INT32U PitchBendValue;
	INT32U ChannelIndex;
	INT32U NewPhase;
	INT32U *pAddr;

	MIDIChannel = MIDI_Event.PitchBendEvent.ChannelNumber;
	PitchBendIndex = MIDI_Event.PitchBendEvent.PitchBendValue;
	(pMIDI_ChInfo + MIDIChannel)->R_MIDI_CH_PitchBend = PitchBendIndex;
	pAddr = (INT32U *)(pMIDI_ChInfo + MIDIChannel)->R_PB_TABLE_Addr;
	PitchBendValue = *(pAddr + PitchBendIndex);
	for(ChannelIndex = 0; ChannelIndex < SPU_ChannelNumber; ChannelIndex++)
	{
		if((pSPU_ChInfo + ChannelIndex)->R_MIDI_CH_MAP == MIDIChannel)
		{
			NewPhase = (PitchBendValue * (pSPU_ChInfo + ChannelIndex)->R_PB_PhaseRecord) >> 14;
			SPU_SetPhase(NewPhase, ChannelIndex);
		}
	}
}

void ProcessProgramChangeEvent(void)
{
	INT32U ChannelNumber,temp;
	ChannelNumber = MIDI_Event.ProgramChangeEvent.ChannelNumber;
	temp = (pMIDI_ChInfo + ChannelNumber)->R_ChannelInst;
	temp &= 0xffff0000;
	temp |= MIDI_Event.ProgramChangeEvent.InstrumentIndex & 0x0000ffff;
	(pMIDI_ChInfo + ChannelNumber)->R_ChannelInst = temp;
}

void ProcessBeatCountEvent(void)
{
	INT32U MinDuration;
	MinDuration = 0x0000;
	R_DUR_Tone[SPU_ChannelNumber] = MIDI_Event.BeatCountEvent.BeatCountValue;

	F_CheckDuration();

	SPU_Enable_MultiChannel(R_CH_OneBeat);
	SPU_Clear_MultiCh_StopFlag(R_CH_OneBeat);
	R_CH_OneBeat = 0x0000;
}

void ProcessTempoEvent(void)
{
	INT16U Temp;
	Temp = C_DefaultBeatBase;
	SPU_Set_BeatBaseCounter(Temp);
}

void SPU_MIDI_Set_Tempo(INT8U tempo)
{
	INT16U Temp;
	Temp = (C_DefaultBeatBase * C_DefaultTempo) / tempo;//080812 tangqt
	if(Temp > 0x07ff)
		Temp = 0x07ff;
	SPU_Set_BeatBaseCounter(Temp);
}

void SPU_MIDI_Repeat(INT8U repeat_en)
{
	if(repeat_en)
		MIDI_PlayFlag |= STS_MIDI_REPEAT_ON;
	else
		MIDI_PlayFlag &= ~STS_MIDI_REPEAT_ON;
}

void SPU_StopChannel(INT32U StopChannel)
{
	INT32U temp;
	SPU_resume_channel((INT8U)StopChannel);//081225
	//-----------------------------------------------------------------
	temp = SPU_ReadPhase(StopChannel);//100817 避免通道无法停止的问题 
	if(temp == 0x00000000)
	{
		SPU_SetPhase(0x00010000,(INT8U)StopChannel);
	}
	//-----------------------------------------------------------------
	R_DUR_Tone[StopChannel] = 0x0000;
	SPU_Set_EnvRampDown(StopChannel);
	SPU_SetEnvelope_0(0x0000, StopChannel);
	SPU_SetEnvelope_1(0x0000, StopChannel);
	SPU_SetEnvelopeData(0x0000, StopChannel);
	SPU_SetEnvelopeCounter(0x0000, StopChannel);
	SPU_SetWaveData(0x8000, StopChannel);
	SPU_Disable_Channel(StopChannel);
}

void ProcessEndEvent(void)
{
	INT32S ChannelIndex;
	INT32U Temp;

//	MidiPlay_Initialize();
	if(MIDI_PlayFlag & STS_MIDI_CALLBACK)
		(*MIDI_StopCallBack)();
	if(MIDI_PlayFlag & STS_MIDI_REPEAT_ON)
	{
		pMIDI_DataPointer = (INT8U*)(midi_ring_buffer_addr + static_midi_ring_buffer_ri);//081105

		Temp = R_MIDI_CH_MASK;
		for(ChannelIndex = 0; ChannelIndex < SPU_ChannelNumber; ChannelIndex++)
		{
			if(Temp & 0x0001)
			{
				SPU_StopChannel(ChannelIndex);
			}
			Temp = Temp >> 1;
		}
		MidiPlay_Initialize();
		SPU_Set_BeatCounter(0x0000);
		SPU_Clear_BeatIRQ_Flag();
		SPU_Enable_BeatIRQ();
	}
	else
	{
		MIDI_PlayFlag &= ~STS_MIDI_PLAY;
		R_MIDI_EndFlag = 0xFFFF;
		SPU_Set_BeatCounter(0x0100);
		SPU_Clear_BeatIRQ_Flag();
		SPU_Enable_BeatIRQ();
		SPU_Set_BeatCounter(0x0000);//080904
	}
}

void SPU_MIDI_SetStopCallBackFunction(void (*StopCallBack)(void))
{
	MIDI_PlayFlag |= STS_MIDI_CALLBACK;
	MIDI_StopCallBack = StopCallBack;
}
//----------------------------------------------------------- 20081027 Roy
void SPU_MIDI_SetDataEntryEventCallback(void (*DataEntryEvent_callback)(void))
{
	MIDI_DataEntryEventCallBack = DataEntryEvent_callback;
}

void SPU_MIDI_GetControlEventInfo(INT8U *para)
{
	*(para++) = MIDI_Control_Event[0];
	*(para++) = MIDI_Control_Event[1];
	//*(para++) = MIDI_Control_Event[2];
	//*(para) = MIDI_Control_Event[3];
}

//-----------------------------------------------------------20081028 Roy
INT32U SPU_MIDI_GetCurDt(void)
{
	return MIDI_Current_Dt;
}

void SPU_MIDI_PlayDt(INT32U MidiStartDt, INT32U MidiStopDt)
{
	SPU_MIDI_Stop();

	MIDI_Current_Dt = 0;
	MIDI_Stop_Dt = MidiStopDt;

	MIDI_PlayFlag &= ~STS_MIDI_PAUSE_ON;
	MIDI_PlayFlag |= STS_MIDI_PLAY;
	MIDI_PlayFlag &= ~STS_MIDI_REPEAT_ON;
	pMIDI_DataPointer = (INT8U*)(midi_ring_buffer_addr + midi_ring_buffer_ri);
	pMIDI_StartAddr = (INT8U*)midi_ring_buffer_addr;
	MidiPlay_Initialize();

	MIDI_Skip_Flag = 1;
	while(MidiStartDt > MIDI_Current_Dt)
		F_GetSeqCmd();
	MIDI_Skip_Flag = 0;

	SPU_Set_BeatBaseCounter(C_DefaultBeatBase);
	SPU_Set_BeatCounter(0x30);
	SPU_Clear_BeatIRQ_Flag();
	SPU_Enable_BeatIRQ();
	SPU_EnableBeatCountFIQ();
}

void SPU_MIDI_SetPlayDtEndCallback(void (*PlayDtStopCallBack)(void))
{
	MIDI_PlayDtStopCallBack =  PlayDtStopCallBack;
}

//-----------------------------------------------------------

void SPU_AttachISR(INT8U FIQ_ID, void (*ISR)(void))
{
	switch(FIQ_ID)
	{
		case C_SPU_PW_FIQ:
			User_FIQ_Flag |= B_SPU_PW_FIQ;
			SPU_EnablePostWaveFIQ();
			SPU_User_FIQ_ISR[C_SPU_PW_FIQ] = ISR;
			break;
		case C_SPU_BEAT_FIQ:
			User_FIQ_Flag |= B_SPU_BEAT_FIQ;
			SPU_EnableBeatCountFIQ();
			SPU_User_FIQ_ISR[C_SPU_BEAT_FIQ] = ISR;
			break;
		case C_SPU_ENV_FIQ:
			User_FIQ_Flag |= B_SPU_ENV_FIQ;
			SPU_EnableEnvelopeFIQ();
			SPU_User_FIQ_ISR[C_SPU_ENV_FIQ] = ISR;
			break;
		case C_SPU_FIQ:
			User_FIQ_Flag |= B_SPU_FIQ;
			SPU_EnableChannelFIQ();
			SPU_User_FIQ_ISR[C_SPU_FIQ] = ISR;
			break;
	}
}

void SPU_ReleaseISR(INT8U FIQ_ID)
{
	switch(FIQ_ID)
	{
		case C_SPU_PW_FIQ:
			User_FIQ_Flag &= ~B_SPU_PW_FIQ;
			SPU_User_FIQ_ISR[C_SPU_PW_FIQ] = 0;
			break;
		case C_SPU_BEAT_FIQ:
			User_FIQ_Flag &= ~B_SPU_BEAT_FIQ;
			SPU_User_FIQ_ISR[C_SPU_BEAT_FIQ] = 0;
			break;
		case C_SPU_ENV_FIQ:
			User_FIQ_Flag &= ~B_SPU_ENV_FIQ;
			SPU_User_FIQ_ISR[C_SPU_ENV_FIQ] = 0;
			break;
		case C_SPU_FIQ:
			User_FIQ_Flag &= ~B_SPU_FIQ;
			SPU_User_FIQ_ISR[C_SPU_FIQ] = 0;
			break;
	}
}
//-----------------------------------------------------------080911
INT32S SPU_FreeLib()
{
#if 1
	int i;
	for(i = 0; i<total_inst; i++)
	{
		//DBG_PRINT("free memory for tone library[%d] = 0x%x ",i,T_InstrumentSectionAddr[i]);
		if(T_InstrumentSectionAddr[i])
			user_memory_free((void *)T_InstrumentSectionAddr[i]);
	}
	total_inst = 0;
	for(i = 0; i<total_drum; i++)
	{
		//DBG_PRINT("free memory for drum library[%d] = 0x%x ",i,T_DrumAddr[i]);
		if(T_DrumAddr[i])
			user_memory_free((void *)T_DrumAddr[i]);
	}
	total_drum = 0;
#endif
        return 0;
}

INT32S	SPU_Free_Midi(void)
{
	INT32S nRet;

	nRet = SPU_FreeLib();
	if(nRet<0)
		return -1;
	return 0;
}

INT32S SPU_Free_ToneColor_1(void)//free 第二组音色 
{
#if 1
        INT32S i;
	for(i = 0; i<500; i++)
	{
		if(T_InstrumentSectionAddr_1[i])//地址不为0，则该乐器已载入 
		{
			user_memory_free((void *)T_InstrumentSectionAddr_1[i]);
			T_InstrumentSectionAddr_1[i] = 0;
		}
	}
#endif
	return 0;
}

INT32S SPU_Free_adpcm_data_temp_buffer(void)//释放用于存放ADPCM数据的内存 
{
#if 1
        if(flag_malloc_adpcm_ram_buffer)//1：需要申请RAM
	{
		if(adpcm_data_temp_buffer_addr)
		{
			user_memory_free((void *)adpcm_data_temp_buffer_addr);
			adpcm_data_temp_buffer_addr = 0;
			return 0;
		}
		else
		{
			return -1;//表示原来没有adpcm数据在内存中 
		}
	}
	else
		return -1;
#else
        return 0;
#endif
}
//---------------------------------------------------------080912

void SPU_Set_midi_ring_buffer_addr(INT32U ring_buffer_addr)
{
	midi_ring_buffer_addr = ring_buffer_addr;
}

void SPU_Set_idi_addr(INT32U idi_addr)//设置idi数据在Nand flash中的起始地址或是在FS文件中的偏移地址 
{
	idi_offset_addr = idi_addr;
}
void SPU_Set_adpcm_comb_addr(INT32U adpcm_comb_addr, INT32U ram_buffer_addr)//设置adpcm_comb数据在Nand flash中的起始地址或是在FS文件中的偏移地址 
//以及存放ADPCM的临时RAM地址；ram_buffer_addr=0:调用user_memory_malloc()分配地址，其它，需要提供足够大的RAM区域的地址 
{
	adpcm_comb_offset_addr = adpcm_comb_addr;
	adpcm_ram_buffer_addr = ram_buffer_addr;
	if(ram_buffer_addr == 0)
		flag_malloc_adpcm_ram_buffer = 1;//需要申请RAM区域 
	else
		flag_malloc_adpcm_ram_buffer = 0;//0:使用预先设定的ram区域 
}

#if 0//SUPPORT_MIDI_LOAD_FROM_NVRAM == 1
typedef INT32S (*fp_resource_read)(INT32U,  INT32U *, INT32U);
fp_resource_read fp_rs_read;

void SPU_resource_read_register(void* rs_read)
{
	fp_rs_read = (fp_resource_read) rs_read;
}
#endif

INT32S read_a_sector(INT16S fd, INT32U data_offset_addr, INT32U buffer_addr, INT8U mode)
{
	INT32S ret;
#if 0//SUPPORT_MIDI_LOAD_FROM_NVRAM == 1
	if(mode == 1)	//nv gprs load.
	{
		if (fp_rs_read == 0) {
			return -1;
		}
		ret = fp_rs_read(data_offset_addr/512,  (INT32U*)buffer_addr, 1);
	}
	else if (mode == 2) {
		ret = (INT32S)nv_lseek(fd, data_offset_addr, SEEK_SET);
		ret = (INT32S)nv_read(fd, (INT32U)buffer_addr, 512);
	}
	else if (mode == 3) {
		ret = (INT32S)nvp_lseek(fd, data_offset_addr);
		ret = (INT32S)nvp_fread(fd, (INT32U)buffer_addr, 512);
	}
	else//FS data load.
	{
		ret = (INT32S)lseek(fd, data_offset_addr, SEEK_SET);
		ret = (INT32S)read(fd, (INT32U)buffer_addr, 512);
	}
	return ret;
#else
	INT32U temp_addr,i;
	INT8U *p_s,*p_t;
	switch(mode)
	{
		case 0://NAND flash logic area
		#if 0//(defined _DRV_L2_NAND) && (_DRV_L2_NAND == 1)
			NAND_LOCK;
			ret = (INT32S)DrvNand_read_sector(((INT32U)data_offset_addr>>9), 1, (INT32U)buffer_addr);
			NAND_UNLOCK;
		#else
			ret = 0;
		#endif
			break;
		case 2://NAND flash app area
		#if 1//(defined _DRV_L2_NAND) && (_DRV_L2_NAND == 1)
			if(buffer_addr & 0x03)
			{
				temp_addr = (INT32U)user_memory_malloc(512);
				if((temp_addr==0) || (temp_addr&0x80000000))
					return -1;

				//NAND_LOCK;
				ret = (INT32S)NandBootReadSector(((INT32U)data_offset_addr>>9), 1, (INT32U)temp_addr);
				//NAND_UNLOCK;
				p_s = (INT8U*)temp_addr;
				p_t = (INT8U*)buffer_addr;
				for(i=0; i<512; i++)
				{
					*p_t++ = *p_s++;
				}
				user_memory_free((void*)temp_addr);
			}
			else
			{
				//NAND_LOCK;
				ret = (INT32S)NandBootReadSector(((INT32U)data_offset_addr>>9), 1, (INT32U)buffer_addr);
				//NAND_UNLOCK;
			}
		#else
			ret = 0;
		#endif
			break;
		case 3://from file system
			ret = (INT32S)lseek(fd, (data_offset_addr&0xfffffe00), SEEK_SET);
			ret = (INT32S)fs_read(fd, (INT32U)buffer_addr, 512);
			break;
		case 4://nv_read
			//ret = (INT32S)nv_lseek(fd, data_offset_addr, SEEK_SET);
			//ret = (INT32S)nv_read(fd, (INT32U)buffer_addr, 512);
			break;
		case 5://nvp_read
			//ret = (INT32S)nvp_lseek(fd, data_offset_addr);
			//ret = (INT32S)nvp_fread(fd, (INT32U)buffer_addr, 512);
			break;
		case 6://from sdram without fs
			p_s = (INT8U*)(data_offset_addr&0xfffffe00);
			p_t = (INT8U*)buffer_addr;
			for(i=0; i<512; i++)
				*p_t++ = *p_s++;

			ret = i;
			break;
		default:break;
	}
	return ret;
#endif
}

INT32S read_two_sector(INT16S fd, INT32U data_offset_addr, INT32U buffer_addr, INT8U mode)
{
	INT32S ret;
#if 0//SUPPORT_MIDI_LOAD_FROM_NVRAM == 1
	if(mode == 1)	//nv gprs load.
	{
		if (fp_rs_read == 0) {
			return -1;
		}
		ret = fp_rs_read(data_offset_addr/512,  (INT32U*)buffer_addr, 1);
	}
	else if (mode == 2) {
		ret = (INT32S)nv_lseek(fd, data_offset_addr, SEEK_SET);
		ret = (INT32S)nv_read(fd, (INT32U)buffer_addr, 512*2);
	}
	else if (mode == 3) {
		ret = (INT32S)nvp_lseek(fd, data_offset_addr);
		ret = (INT32S)nvp_fread(fd, (INT32U)buffer_addr, 512*2);
	}
	else//FS data load.
	{
		ret = (INT32S)lseek(fd, data_offset_addr, SEEK_SET);
		ret = (INT32S)read(fd, (INT32U)buffer_addr, 512*2);
	}
	return ret;
#else
	INT32U temp_addr,i;
	INT8U *p_s,*p_t;
	switch(mode)
	{
		case 0://NAND flash logic area
		#if 0//(defined _DRV_L2_NAND) && (_DRV_L2_NAND == 1)
			NAND_LOCK;
			ret = (INT32S)DrvNand_read_sector(((INT32U)data_offset_addr>>9), 2, (INT32U)buffer_addr);
			NAND_UNLOCK;
		#else
			ret = 0;
		#endif
			break;
		case 2://NAND flash app area
		#if 1//(defined _DRV_L2_NAND) && (_DRV_L2_NAND == 1)
			if(buffer_addr & 0x03)
			{
				temp_addr = (INT32U)user_memory_malloc(512*2);
				if((temp_addr==0) || (temp_addr&0x80000000))
					return -1;

				//NAND_LOCK;
				ret = (INT32S)NandBootReadSector(((INT32U)data_offset_addr>>9), 2, (INT32U)temp_addr);
				//NAND_UNLOCK;
				p_s = (INT8U*)temp_addr;
				p_t = (INT8U*)buffer_addr;
				for(i=0; i<(512*2); i++)
				{
					*p_t++ = *p_s++;
				}
				user_memory_free((void*)temp_addr);
			}
			else
			{
				//NAND_LOCK;
				ret = (INT32S)NandBootReadSector(((INT32U)data_offset_addr>>9), 2, (INT32U)buffer_addr);
				//NAND_UNLOCK;
			}
		#else
			ret = 0;
		#endif
			break;
		case 3://from file system
			ret = (INT32S)lseek(fd, (data_offset_addr&0xfffffe00), SEEK_SET);
			ret = (INT32S)fs_read(fd, (INT32U)buffer_addr, 512*2);
			break;
		case 4://nv_read
			//ret = (INT32S)nv_lseek(fd, data_offset_addr, SEEK_SET);
			//ret = (INT32S)nv_read(fd, (INT32U)buffer_addr, 512*2);
			break;
		case 5://nvp_read
			//ret = (INT32S)nvp_lseek(fd, data_offset_addr);
			//ret = (INT32S)nvp_fread(fd, (INT32U)buffer_addr, 512*2);
			break;
		case 6://from sdram without fs
			//p_s = (INT8U*)(data_offset_addr&0xfffffe00);
                        p_s = (INT8U*)(data_offset_addr);
			p_t = (INT8U*)buffer_addr;
			for(i=0; i<(512*2); i++)
				*p_t++ = *p_s++;

			ret = i;
			break;
		default:break;
	}
	return ret;
#endif
}

INT32S SPU_read_anterior_mididata_to_ringbuffer(void)//fill ring buffer anterior area
{
	INT32U offset_addr;
	INT32S ret;
	INT32U temp_addr;
	INT32S i;

	remain_midi_length = static_midi_length;
	offset_addr = midi_start_addr + static_midi_offset;//midi数据在idi中的地址 
	static_midi_ring_buffer_ri = offset_addr & 0x000001ff;//表示从midi_ring_buffer的该位置开始播 

	remain_midi_length += static_midi_ring_buffer_ri;
	offset_addr &= 0xfffffe00;//指向当前sector的开始 
	if((midi_ring_buffer_wi != 0) && (midi_ring_buffer_wi != MIDI_RING_BUFFER_SIZE/2) && (midi_ring_buffer_wi != (MIDI_RING_BUFFER_SIZE/2 + MIDI_RING_BUFFER_SIZE/4)))
	{
		SPU_err();
		return -1;
	}
	temp_addr = midi_ring_buffer_addr + midi_ring_buffer_wi;
	switch(midi_ring_buffer_wi)
	{
		case 0:
			i = MIDI_RING_BUFFER_SIZE/2;//需要读到buffer中的字节数 
			break;
		default:
			i = MIDI_RING_BUFFER_SIZE/4;
			break;
	}
	while(remain_midi_length)
	{
		ret = read_a_sector(static_fd_idi, (offset_addr & 0xfffffe00), temp_addr, SPU_load_data_mode);//081031
		if(ret < 0)
		{
			SPU_err();
			return ret;
		}
		if(remain_midi_length >= 512)
			remain_midi_length -= 512;//表示可读的剩余midi长度 
		else
			remain_midi_length = 0;
		offset_addr += 512;
		temp_addr += 512;
		midi_ring_buffer_wi += 512;
		i -= 512;
		if(i <= 0)
		{
			break;
		}
	}
	if(midi_ring_buffer_wi <= MIDI_RING_BUFFER_SIZE/2)
	{
		midi_ring_buffer_wi = MIDI_RING_BUFFER_SIZE/2;
	}
	else
		if(midi_ring_buffer_wi <= (MIDI_RING_BUFFER_SIZE/2 + MIDI_RING_BUFFER_SIZE/4))
		{
			midi_ring_buffer_wi = (MIDI_RING_BUFFER_SIZE/2 + MIDI_RING_BUFFER_SIZE/4);
			static_midi_ring_buffer_ri += MIDI_RING_BUFFER_SIZE/2;
		}
		else
		{
			midi_ring_buffer_wi = 0;
			static_midi_ring_buffer_ri += MIDI_RING_BUFFER_SIZE/2 + MIDI_RING_BUFFER_SIZE/4;
		}
	currt_midi_data_offset = offset_addr;
	//midi_ring_buffer_wi = 0;//表示已将midi_ring_buffer的B区域写满，如为MIDI_RING_BUFFER_SIZE/2则表示已将A区域填满 
	return 0;
}

INT32S SPU_load_tonecolor(INT16S fd_idi, INT32U MIDI_Index,INT8U mode)//load音色 
{//以byte为单位进行访问 
	INT32U inst_tab_start_addr;
	INT32U offset_addr,offset_addr1;
	INT32U total_midi;
	INT32U temp_buffer_addr,temp_sector_start;//081029
	INT32U temp_read_start_addr;//目前读到temp buffer中的数据在文件或nand flash中的偏移量 
	INT32U inst_temp_buffer_addr, drum_temp_buffer_addr;
	INT8U  *U8_ptr,*U8_ptr1;//081029
	INT32S ret;
	INT32U temp,temp1,temp2;
	INT32S dwSize;
	INT32S i,j;
	INT32U temp_addr,temp_size;
	INT8U char_temp3,char_temp2,char_temp1,char_temp0;//后面定义的局部变量地址越来越小//081029
	//-----------------------------------------------------------------------//get every single file 's offset
	SPU_Free_Midi();//081115
	SPU_load_data_mode = mode;
	static_fd_idi = fd_idi;//080912
#if USER_DEFINE_MALLOC_EN == 1
    if(pfunc_user_memory_malloc)
    {
        temp_buffer_addr = (INT32U)pfunc_user_memory_malloc(512*2);//512 byte per sector;//081029
    }
    else
    {
		SPU_err();
		return -1;
    }
#else
	temp_buffer_addr = (INT32U)user_memory_malloc(512*2);//512 byte per sector;//081029
#endif
	if((temp_buffer_addr&0x80000000) || (temp_buffer_addr==0))
	{
		SPU_err();
		return -1;
	}
	offset_addr = idi_offset_addr;//数据在文件中的偏移地址//081029
	temp_read_start_addr = offset_addr & 0xfffffe00;//081228
	ret = read_two_sector(static_fd_idi, (offset_addr & 0xfffffe00), temp_buffer_addr, SPU_load_data_mode);
	if(ret < 0)
	{
		SPU_err();
		return ret;
	}
	U8_ptr = (INT8U*)(temp_buffer_addr + ((INT32U)offset_addr - temp_read_start_addr) );//081031
	char_temp0 = *(U8_ptr++);
	char_temp1 = *(U8_ptr++);
	char_temp2 = *(U8_ptr++);
	char_temp3 = *(U8_ptr++);
	inst_start_addr = (INT32U)(((char_temp3<<24) | (char_temp2<<16) | (char_temp1<<8) | char_temp0)/* + idi_offset_addr*/);
	if(inst_start_addr == 0x444d5047)//"GPMD", gmd file
	{
		U8_ptr += 2;
		char_temp0 = *U8_ptr;
		if(char_temp0 == 0)//single midi file
		{
			MIDI_Index = 0;
		}
		U8_ptr += 6;
		char_temp0 = *(U8_ptr++);
		char_temp1 = *(U8_ptr++);
		char_temp2 = *(U8_ptr++);
		char_temp3 = *(U8_ptr++);
		inst_start_addr = (INT32U)(((char_temp3<<24) | (char_temp2<<16) | (char_temp1<<8) | char_temp0)/* + idi_offset_addr*/);
	}
	inst_start_addr += idi_offset_addr;
	char_temp0 = *(U8_ptr++);
	char_temp1 = *(U8_ptr++);
	char_temp2 = *(U8_ptr++);
	char_temp3 = *(U8_ptr++);
	lib_start_addr = (INT32U)(((char_temp3<<24) | (char_temp2<<16) | (char_temp1<<8) | char_temp0) + idi_offset_addr);
	char_temp0 = *(U8_ptr++);
	char_temp1 = *(U8_ptr++);
	char_temp2 = *(U8_ptr++);
	char_temp3 = *(U8_ptr++);
	midi_start_addr = (INT32U)(((char_temp3<<24) | (char_temp2<<16) | (char_temp1<<8) | char_temp0) + idi_offset_addr);
	//----------------------------------------------------------------------------------
	offset_addr = inst_start_addr;
	temp_read_start_addr = offset_addr & 0xfffffe00;//081228
	ret = read_two_sector(static_fd_idi, (offset_addr & 0xfffffe00), temp_buffer_addr, SPU_load_data_mode);
	if(ret < 0)
	{
		SPU_err();
		return ret;
	}
	U8_ptr = (INT8U*)(temp_buffer_addr + ((INT32U)offset_addr - temp_read_start_addr) );//081031
	//----------------------------------------------------------------------------------
	char_temp0 = *(U8_ptr++);
	char_temp1 = *(U8_ptr++);
	char_temp2 = *(U8_ptr++);
	char_temp3 = *(U8_ptr++);
	total_midi = (INT32U)((char_temp3<<24) | (char_temp2<<16) | (char_temp1<<8) | char_temp0);
	if(MIDI_Index >= total_midi)
	{
		SPU_err();
		return ret;
	}
	offset_addr += 4;
	offset_addr = offset_addr + MIDI_Index*4;
	temp_read_start_addr = offset_addr & 0xfffffe00;//081228
	ret = read_two_sector(static_fd_idi, (offset_addr & 0xfffffe00), temp_buffer_addr, SPU_load_data_mode);
	if(ret < 0)
	{
		SPU_err();
		return ret;
	}
	U8_ptr = (INT8U*)(temp_buffer_addr + ((INT32U)offset_addr - temp_read_start_addr) );//081031
	char_temp0 = *(U8_ptr++);
	char_temp1 = *(U8_ptr++);
	char_temp2 = *(U8_ptr++);
	char_temp3 = *(U8_ptr++);
	temp = (INT32U)((char_temp3<<24) | (char_temp2<<16) | (char_temp1<<8) | char_temp0);
	inst_tab_start_addr = inst_start_addr + temp;//081031
	offset_addr = inst_tab_start_addr;
	temp_sector_start = offset_addr;//081030 记录当前sector的首地址 
	temp_read_start_addr = offset_addr & 0xfffffe00;//081228
	ret = read_two_sector(static_fd_idi, (offset_addr & 0xfffffe00), temp_buffer_addr, SPU_load_data_mode);
	if(ret < 0)
	{
		SPU_err();
		return ret;
	}
	U8_ptr = (INT8U*)(temp_buffer_addr + ((INT32U)offset_addr - temp_read_start_addr) );//081031
	char_temp0 = *(U8_ptr++);
	char_temp1 = *(U8_ptr++);
	temp = (INT32U)((char_temp1<<8) | char_temp0);
	//已让 U16_ptr 指向inst_tab的开始 
	dwSize = (INT32S)temp;
	for(i=0; i<dwSize; i++)
	{
		offset_addr1 = offset_addr + 2;
		if((offset_addr1 - temp_sector_start) >= 512)//081030
		{
			temp_read_start_addr = offset_addr1 & 0xfffffe00;//081228
			ret = read_two_sector(static_fd_idi, (offset_addr1 & 0xfffffe00), temp_buffer_addr, SPU_load_data_mode);
			if(ret < 0)
			{
				SPU_err();
				return ret;
			}
			offset_addr = offset_addr1;
			temp_sector_start = offset_addr;//081030 记录当前sector的首地址 
			U8_ptr = (INT8U*)(temp_buffer_addr + ((INT32U)offset_addr - temp_read_start_addr) );//081031
		}
		else
		{
			offset_addr += 2;
		}
		char_temp0 = *(U8_ptr++);
		char_temp1 = *(U8_ptr++);
		temp = (INT32U)((char_temp1<<8) | char_temp0);
		T_InstrumentStartSection[i] = (INT32U)temp;
	}
	//已将 T_InstrumentStartSection 载入 
	offset_addr += 2;
	temp_sector_start = offset_addr;//081030 记录当前sector的首地址 
	temp_read_start_addr = offset_addr & 0xfffffe00;//081228
	ret = read_two_sector(static_fd_idi, (offset_addr & 0xfffffe00), temp_buffer_addr, SPU_load_data_mode);
	if(ret < 0)
	{
		SPU_err();
		return ret;
	}
	U8_ptr = (INT8U*)(temp_buffer_addr + ((INT32U)offset_addr - temp_read_start_addr) );
	char_temp0 = *(U8_ptr++);
	char_temp1 = *(U8_ptr++);
	temp = (INT32U)((char_temp1<<8) | char_temp0);
	//已让 U16_ptr 指向inst_tab_pitchTable的开始 
	dwSize = (INT32S)temp;
	for(i=0; i<dwSize; i++)
	{
		offset_addr1 = offset_addr + 2;
		if((offset_addr1 - temp_sector_start) >= 512)//081030
		{
			temp_read_start_addr = offset_addr1 & 0xfffffe00;//081228
			ret = read_two_sector(static_fd_idi, (offset_addr1 & 0xfffffe00), temp_buffer_addr, SPU_load_data_mode);
			if(ret < 0)
			{
				SPU_err();
				return ret;
			}
			offset_addr = offset_addr1;
			temp_sector_start = offset_addr;//081030 记录当前sector的首地址 
			U8_ptr = (INT8U*)(temp_buffer_addr + ((INT32U)offset_addr - temp_read_start_addr) );//081031
		}
		else
		{
			offset_addr += 2;
		}
		char_temp0 = *(U8_ptr++);
		char_temp1 = *(U8_ptr++);
		temp = (INT32U)((char_temp1<<8) | char_temp0);
		T_InstrumentPitchTable[i] = (INT32U)temp;//081031
	}
	//已将 T_InstrumentPitchTable 载入-----------------------------------------------------OK
	offset_addr += 2;
	temp_sector_start = offset_addr;//081030 记录当前sector的首地址 
	temp_read_start_addr = offset_addr & 0xfffffe00;//081228
	ret = read_two_sector(static_fd_idi, (offset_addr & 0xfffffe00), temp_buffer_addr, SPU_load_data_mode);
	if(ret < 0)
	{
		SPU_err();
		return ret;
	}
	U8_ptr = (INT8U*)(temp_buffer_addr + ((INT32U)offset_addr - temp_read_start_addr) );
	char_temp0 = *(U8_ptr++);
	char_temp1 = *(U8_ptr++);
	temp2 = (INT32U)((char_temp1<<8) | char_temp0);
	temp = temp2;//low word of size
	offset_addr += 2;//081030
	U8_ptr = (INT8U*)(temp_buffer_addr + (offset_addr - temp_read_start_addr) );//081228
	char_temp0 = *(U8_ptr++);
	char_temp1 = *(U8_ptr++);
	temp2 = (INT32U)((char_temp1<<8) | char_temp0);
	temp1 = temp2;//high word of size
	dwSize = (INT32S)((temp1<<16) | temp);//size of instrument sectors
	total_inst = dwSize;
#if USER_DEFINE_MALLOC_EN == 1
    if(pfunc_user_memory_malloc)
    {
        inst_temp_buffer_addr = (INT32U)pfunc_user_memory_malloc(dwSize*8);//512 byte per sector;//081029
    }
    else
    {
		SPU_err();
		return -1;
    }
#else
	inst_temp_buffer_addr = (INT32U)user_memory_malloc(dwSize*8);
#endif
	if((inst_temp_buffer_addr&0x80000000) || (inst_temp_buffer_addr==0))
	{
		SPU_err();
		return -1;
	}
	U8_ptr1 = (INT8U*)inst_temp_buffer_addr;
//	offset_addr += 1;//081031 由于刚读了2byte的数据，而在接下来的循环中只增加1，所以要预先加1 

	for(i=0; i<dwSize*4; i++)//081031
	{
		offset_addr1 = offset_addr + 2;//081031
		if((offset_addr1 - temp_sector_start) >= 512)//081030
		{
			temp_read_start_addr = offset_addr1 & 0xfffffe00;//081228
			ret = read_two_sector(static_fd_idi, (offset_addr1 & 0xfffffe00), temp_buffer_addr, SPU_load_data_mode);
			if(ret < 0)
			{
				SPU_err();
				return ret;
			}
			offset_addr = offset_addr1;
			temp_sector_start = offset_addr;//081030 记录当前sector的首地址 
			U8_ptr = (INT8U*)(temp_buffer_addr + ((INT32U)offset_addr - temp_read_start_addr) );//081031
		}
		else
		{
			offset_addr += 2;
		}
		*U8_ptr1++ = (INT8U)*(U8_ptr++);//081031
		*U8_ptr1++ = (INT8U)*(U8_ptr++);//081031
	}
	//已将tab中的instrument section的地址和长度的那一段完全拷贝到inst_temp_buffer_addr这个buffer中； 
	offset_addr += 2;
	temp_sector_start = offset_addr;//081030 记录当前sector的首地址 
	temp_read_start_addr = offset_addr & 0xfffffe00;//081228
	ret = read_two_sector(static_fd_idi, (offset_addr & 0xfffffe00), temp_buffer_addr, SPU_load_data_mode);
	if(ret < 0)
	{
		SPU_err();
		return ret;
	}
	U8_ptr = (INT8U*)(temp_buffer_addr + ((INT32U)offset_addr - temp_read_start_addr) );
	char_temp0 = *(U8_ptr++);
	char_temp1 = *(U8_ptr++);
	temp2 = (INT32U)((char_temp1<<8) | char_temp0);
	temp = temp2;//low word of size
	offset_addr += 2;//081030

	char_temp0 = *(U8_ptr++);
	char_temp1 = *(U8_ptr++);
	temp2 = (INT32U)((char_temp1<<8) | char_temp0);
	temp1 = temp2;//high word of size
	dwSize = (INT32S)((temp1<<16) | temp);//size of drum sectors
	total_drum = dwSize;
#if USER_DEFINE_MALLOC_EN == 1
    if(pfunc_user_memory_malloc)
    {
        drum_temp_buffer_addr = (INT32U)pfunc_user_memory_malloc(dwSize*8);//512 byte per sector;//081029
    }
    else
    {
		SPU_err();
		return -1;
    }
#else
	drum_temp_buffer_addr = (INT32U)user_memory_malloc(dwSize*8);
#endif
	if( dwSize && ((drum_temp_buffer_addr&0x80000000) || (drum_temp_buffer_addr==0)))
	{
		SPU_err();
		return -1;
	}
	U8_ptr1 = (INT8U*)drum_temp_buffer_addr;
//	offset_addr += 1;//081031 由于刚读了2byte的数据，而在接下来的循环中只增加1，所以要预先加1 
	for(i=0; i<dwSize*4; i++)
	{
		offset_addr1 = offset_addr + 2;//081031
		if((offset_addr1 - temp_sector_start) >= 512)//081030
		{
			temp_read_start_addr = offset_addr1 & 0xfffffe00;//081228
			ret = read_two_sector(static_fd_idi, (offset_addr1 & 0xfffffe00), temp_buffer_addr, SPU_load_data_mode);
			if(ret < 0)
			{
				SPU_err();
				return ret;
			}
			offset_addr = offset_addr1;
			temp_sector_start = offset_addr;//081030 记录当前sector的首地址 
			U8_ptr = (INT8U*)(temp_buffer_addr + ((INT32U)offset_addr - temp_read_start_addr) );//081031
		}
		else
		{
			offset_addr += 2;//081031
		}
		*U8_ptr1++ = (INT8U)*(U8_ptr++);//low word of addr//081031
		*U8_ptr1++ = (INT8U)*(U8_ptr++);//low word of addr//081031
	}
	//已将tab中的drum section的地址和长度的那一段完全拷贝到drum_temp_buffer_addr这个buffer中---------OK
	for(i=0; i<total_inst; i++)
	{
		U8_ptr = (INT8U*)(inst_temp_buffer_addr + i*8);
		char_temp0 = *(U8_ptr++);//081031
		char_temp1 = *(U8_ptr++);
		char_temp2 = *(U8_ptr++);
		char_temp3 = *(U8_ptr++);
		temp2 = (INT32U)((char_temp3<<24) | (char_temp2<<16) | (char_temp1<<8) | char_temp0);//081031
		temp_addr = temp2;
		char_temp0 = *(U8_ptr++);//081031
		char_temp1 = *(U8_ptr++);
		char_temp2 = *(U8_ptr++);
		char_temp3 = *(U8_ptr++);
		temp2 = (INT32U)((char_temp3<<24) | (char_temp2<<16) | (char_temp1<<8) | char_temp0);//081031
		temp_size = temp2;
#if USER_DEFINE_MALLOC_EN == 1
    if(pfunc_user_memory_malloc)
    {
        temp = (INT32U)pfunc_user_memory_malloc(temp_size);//512 byte per sector;//081029
    }
    else
    {
		SPU_err();
		return -1;
    }
#else
		temp = (INT32U)user_memory_malloc(temp_size);
#endif
		if((temp&0x80000000) || (temp == 0))
		{
			SPU_err();
			return -1;
		}
		offset_addr = lib_start_addr + temp_addr;
		j = temp_size;//j表示需要读取的字节数 
		temp_read_start_addr = offset_addr & 0xfffffe00;//081228
		ret = read_two_sector(static_fd_idi, (offset_addr & 0xfffffe00), temp_buffer_addr, SPU_load_data_mode);//081030
		if(ret < 0)
		{
			SPU_err();
			return ret;
		}
		U8_ptr = (INT8U*)(temp_buffer_addr + ((INT32U)offset_addr - temp_read_start_addr) );//081030
		U8_ptr1 = (INT8U*)temp;//081030
		offset_addr1 = (offset_addr & 0xfffffe00) + 0x00000200;//下一个sector的起始地址 
		while((offset_addr < offset_addr1) && (j>0))
		{
			*U8_ptr1++ = *U8_ptr++;//081030
			offset_addr += 1;//081030
			j -= 1;//081030
		}
		for(;j>=512;)//一次读一个sector
		{
			ret = read_a_sector(static_fd_idi, (offset_addr & 0xfffffe00), (INT32U)U8_ptr1, SPU_load_data_mode);//081030
			if(ret < 0)
			{
				SPU_err();
				return ret;
			}
			j -= 512;
			offset_addr += 512;//512 byte
			U8_ptr1 += 512;//256 word//081030
			if(offset_addr & 0x000001ff)
			{
				SPU_err();
				return ret;
			}
		}
		if(j>0)
		{
			temp_read_start_addr = offset_addr & 0xfffffe00;//081228
			ret = read_two_sector(static_fd_idi, (offset_addr & 0xfffffe00), temp_buffer_addr, SPU_load_data_mode);//081030
			if(ret < 0)
			{
				SPU_err();
				return ret;
			}
			U8_ptr = (INT8U*)(temp_buffer_addr + ((INT32U)offset_addr - temp_read_start_addr) );//081030
			while(j>0)
			{
				*U8_ptr1++ = *U8_ptr++;//081030
				offset_addr += 1;//081030
				j -= 1;//081030
			}
		}
		T_InstrumentSectionAddr[i] = (INT32U*)temp;
	}
#if USER_DEFINE_MALLOC_EN == 1
    if(pfunc_user_memory_free)
    {
        pfunc_user_memory_free((void *)inst_temp_buffer_addr);
    }
    else
    {
		SPU_err();
		return -1;
    }
#else
	user_memory_free((void *)inst_temp_buffer_addr);
#endif
	//已将instrument sections全部拷贝到申请的RAM区域中，并将申请的ram区域的首地址写入T_InstrumentSectionAddr[]中 
	for(i=0; i<total_drum; i++)
	{
		U8_ptr = (INT8U*)(drum_temp_buffer_addr + i*8);
		char_temp0 = *(U8_ptr++);//081031
		char_temp1 = *(U8_ptr++);
		char_temp2 = *(U8_ptr++);
		char_temp3 = *(U8_ptr++);
		temp2 = (INT32U)((char_temp3<<24) | (char_temp2<<16) | (char_temp1<<8) | char_temp0);//081031
		temp_addr = temp2;
		char_temp0 = *(U8_ptr++);//081031
		char_temp1 = *(U8_ptr++);
		char_temp2 = *(U8_ptr++);
		char_temp3 = *(U8_ptr++);
		temp2 = (INT32U)((char_temp3<<24) | (char_temp2<<16) | (char_temp1<<8) | char_temp0);//081031
		temp_size = temp2;
#if USER_DEFINE_MALLOC_EN == 1
        if(pfunc_user_memory_malloc)
        {
            temp = (INT32U)pfunc_user_memory_malloc(temp_size);//512 byte per sector;//081029
        }
        else
        {
            SPU_err();
            return -1;
        }
#else
		temp = (INT32U)user_memory_malloc(temp_size);
#endif
		if((temp&0x80000000) || (temp == 0))
		{
			SPU_err();
			return -1;
		}
		offset_addr = lib_start_addr + temp_addr;
		j = temp_size;//j表示需要读取的字节数 
		temp_read_start_addr = offset_addr & 0xfffffe00;//081228
		ret = read_two_sector(static_fd_idi, (offset_addr & 0xfffffe00), temp_buffer_addr, SPU_load_data_mode);//081030
		if(ret < 0)
		{
			SPU_err();
			return ret;
		}
		U8_ptr = (INT8U*)(temp_buffer_addr + ((INT32U)offset_addr - temp_read_start_addr) );//081030
		U8_ptr1 = (INT8U*)temp;//081030
		offset_addr1 = (offset_addr & 0xfffffe00) + 0x00000200;//下一个sector的起始地址 
		while((offset_addr < offset_addr1) && (j>0))
		{
			*U8_ptr1++ = *U8_ptr++;//081030
			offset_addr += 1;//081030
			j -= 1;//081030
		}
		for(;j>=512;)//一次读一个sector
		{
			ret = read_a_sector(static_fd_idi, (offset_addr & 0xfffffe00), (INT32U)U8_ptr1, SPU_load_data_mode);//081030
			if(ret < 0)
			{
				SPU_err();
				return ret;
			}
			j -= 512;
			offset_addr += 512;//512 byte
			U8_ptr1 += 512;//256 word//081030
			if(offset_addr & 0x000001ff)
			{
				SPU_err();
				return ret;
			}
		}
		if(j>0)
		{
			temp_read_start_addr = offset_addr & 0xfffffe00;//081228
			ret = read_two_sector(static_fd_idi, (offset_addr & 0xfffffe00), temp_buffer_addr, SPU_load_data_mode);//081030
			if(ret < 0)
			{
				SPU_err();
				return ret;
			}
			U8_ptr = (INT8U*)(temp_buffer_addr + ((INT32U)offset_addr - temp_read_start_addr));//081030
			while(j>0)
			{
				*U8_ptr1++ = *U8_ptr++;//081030
				offset_addr += 1;//081030
				j -= 1;//081030
			}
		}
		T_DrumAddr[i] = (INT32U*)temp;
	}
#if USER_DEFINE_MALLOC_EN == 1
    if(pfunc_user_memory_free)
    {
        pfunc_user_memory_free((void *)drum_temp_buffer_addr);
    }
    else
    {
		SPU_err();
		return -1;
    }
#else
	user_memory_free((void *)drum_temp_buffer_addr);
#endif
	//已将drum sections全部拷贝到申请的RAM区域中，并将申请的ram区域的首地址写入T_DrumAddr[]中，并释放 
	offset_addr = midi_start_addr + 4 + MIDI_Index*8;
	temp_sector_start = offset_addr;//081031 记录当前sector的首地址 
	temp_read_start_addr = offset_addr & 0xfffffe00;//081228
	ret = read_two_sector(static_fd_idi, (offset_addr & 0xfffffe00), temp_buffer_addr, SPU_load_data_mode);//081031
	if(ret < 0)
	{
		SPU_err();
		return ret;
	}
	U8_ptr = (INT8U*)(temp_buffer_addr + ((INT32U)offset_addr - temp_read_start_addr) );
	char_temp0 = *(U8_ptr++);
	char_temp1 = *(U8_ptr++);
	temp2 = (INT32U)((char_temp1<<8) | char_temp0);
	temp = temp2;//low word of addr
	offset_addr += 2;//081031
	U8_ptr = (INT8U*)(temp_buffer_addr + ((INT32U)offset_addr - temp_read_start_addr) );
	char_temp0 = *(U8_ptr++);
	char_temp1 = *(U8_ptr++);
	temp2 = (INT32U)((char_temp1<<8) | char_temp0);
	temp1 = temp2;//high word of addr
	temp_addr = (INT32U)((temp1<<16) | temp);//midi offset
	static_midi_offset = temp_addr;
	//已得到midi偏移地址 
	offset_addr += 2;//081031
	U8_ptr = (INT8U*)(temp_buffer_addr + ((INT32U)offset_addr - temp_read_start_addr) );
	char_temp0 = *(U8_ptr++);
	char_temp1 = *(U8_ptr++);
	temp2 = (INT32U)((char_temp1<<8) | char_temp0);
	temp = temp2;//low word of size
	offset_addr += 2;//081031
	U8_ptr = (INT8U*)(temp_buffer_addr + ((INT32U)offset_addr - temp_read_start_addr) );
	char_temp0 = *(U8_ptr++);
	char_temp1 = *(U8_ptr++);
	temp2 = (INT32U)((char_temp1<<8) | char_temp0);
	temp1 = temp2;//high word of size
	static_midi_length = (temp1<<16) | temp;
	//得到midi的长度（字节数） 
	midi_ring_buffer_wi = 0;
	SPU_read_anterior_mididata_to_ringbuffer();
	midi_ring_buffer_ri = static_midi_ring_buffer_ri;
#if USER_DEFINE_MALLOC_EN == 1
    if(pfunc_user_memory_free)
    {
        pfunc_user_memory_free((void *)temp_buffer_addr);
    }
    else
    {
		SPU_err();
		return -1;
    }
#else
	user_memory_free((void *)temp_buffer_addr);
#endif
	return total_midi;
}

//---------------------------------------------------------------------------------切换音色和播音符用 
INT32S SPU_load_tonecolor_1(INT16S fd_idi, INT32U MIDI_Index,INT8U mode)//load备用音色 
{//以byte为单位进行访问 
	INT32U inst_start_addr1,lib_start_addr1,midi_start_addr1;
	INT32U inst_tab_start_addr1;
	INT32U offset_addr,offset_addr1;
	INT32U total_midi;
	INT32U total_inst_1;
	INT32U temp_buffer_addr,temp_sector_start;//081029
	INT32U temp_read_start_addr;
	INT32U inst_temp_buffer_addr;
	INT8U  *U8_ptr,*U8_ptr1;//081029
	INT32S ret;
	INT32U temp,temp1,temp2;
	INT32S dwSize;
	INT32S i,j;
	INT32U temp_addr,temp_size;
	INT8U char_temp3,char_temp2,char_temp1,char_temp0;//后面定义的局部变量地址越来越小//081029
	//-----------------------------------------------------------------------//get every single file 's offset
	SPU_Free_ToneColor_1();
#if USER_DEFINE_MALLOC_EN == 1
    if(pfunc_user_memory_malloc)
    {
        temp_buffer_addr = (INT32U)pfunc_user_memory_malloc(512*2);//512 byte per sector;//081029
    }
    else
    {
		SPU_err();
		return -1;
    }
#else
	temp_buffer_addr = (INT32U)user_memory_malloc(512*2);//512 byte per sector;
#endif
	if((temp_buffer_addr&0x80000000) || (temp_buffer_addr==0))
	{
		SPU_err();
		return -1;
	}
	offset_addr = idi_offset_addr;//数据在文件中的偏移地址//081029
	temp_read_start_addr = offset_addr & 0xfffffe00;//081228
	ret = read_two_sector(fd_idi, (offset_addr & 0xfffffe00), temp_buffer_addr, mode);
	if(ret < 0)
	{
		SPU_err();
		return ret;
	}
	U8_ptr = (INT8U*)(temp_buffer_addr + ((INT32U)offset_addr - temp_read_start_addr) );//081031
	char_temp0 = *(U8_ptr++);
	char_temp1 = *(U8_ptr++);
	char_temp2 = *(U8_ptr++);
	char_temp3 = *(U8_ptr++);
	inst_start_addr1 = (INT32U)(((char_temp3<<24) | (char_temp2<<16) | (char_temp1<<8) | char_temp0) + idi_offset_addr);
	char_temp0 = *(U8_ptr++);
	char_temp1 = *(U8_ptr++);
	char_temp2 = *(U8_ptr++);
	char_temp3 = *(U8_ptr++);
	lib_start_addr1 = (INT32U)(((char_temp3<<24) | (char_temp2<<16) | (char_temp1<<8) | char_temp0) + idi_offset_addr);
	char_temp0 = *(U8_ptr++);
	char_temp1 = *(U8_ptr++);
	char_temp2 = *(U8_ptr++);
	char_temp3 = *(U8_ptr++);
	midi_start_addr1 = (INT32U)(((char_temp3<<24) | (char_temp2<<16) | (char_temp1<<8) | char_temp0) + idi_offset_addr);
	//-----------------------------------------------------------------------
	char_temp0 = *(U8_ptr++);
	char_temp1 = *(U8_ptr++);
	char_temp2 = *(U8_ptr++);
	char_temp3 = *(U8_ptr++);
	total_midi = (INT32U)((char_temp3<<24) | (char_temp2<<16) | (char_temp1<<8) | char_temp0);
	if(MIDI_Index >= total_midi)
	{
		SPU_err();
		return ret;
	}
	offset_addr += 16;
	offset_addr = offset_addr + MIDI_Index*4;
	temp_read_start_addr = offset_addr & 0xfffffe00;//081228
	ret = read_two_sector(fd_idi, (offset_addr & 0xfffffe00), temp_buffer_addr, mode);
	if(ret < 0)
	{
		SPU_err();
		return ret;
	}
	U8_ptr = (INT8U*)(temp_buffer_addr + ((INT32U)offset_addr - temp_read_start_addr) );//081031
	char_temp0 = *(U8_ptr++);
	char_temp1 = *(U8_ptr++);
	char_temp2 = *(U8_ptr++);
	char_temp3 = *(U8_ptr++);
	temp = (INT32U)((char_temp3<<24) | (char_temp2<<16) | (char_temp1<<8) | char_temp0);
	inst_tab_start_addr1 = inst_start_addr1 + temp;//081031
	offset_addr = inst_tab_start_addr1;
	temp_sector_start = offset_addr;//081030 记录当前sector的首地址 
	temp_read_start_addr = offset_addr & 0xfffffe00;//081228
	ret = read_two_sector(fd_idi, (offset_addr & 0xfffffe00), temp_buffer_addr, mode);
	if(ret < 0)
	{
		SPU_err();
		return ret;
	}
	U8_ptr = (INT8U*)(temp_buffer_addr + ((INT32U)offset_addr - temp_read_start_addr) );//081031
	char_temp0 = *(U8_ptr++);
	char_temp1 = *(U8_ptr++);
	temp = (INT32U)((char_temp1<<8) | char_temp0);
	//已让 U16_ptr 指向inst_tab的开始 
	dwSize = (INT32S)temp;
	//将全音色MIDI的T_ InstrumentStartSection载到T_InstrumentStartSection_1[]中， 
	for(i=0; i<dwSize; i++)
	{
		offset_addr1 = offset_addr + 2;
		if((offset_addr1 - temp_sector_start) >= 512)//081030
		{
			temp_read_start_addr = offset_addr1 & 0xfffffe00;//081228
			ret = read_two_sector(fd_idi, (offset_addr1 & 0xfffffe00), temp_buffer_addr, mode);
			if(ret < 0)
			{
				SPU_err();
				return ret;
			}
			offset_addr = offset_addr1;
			temp_sector_start = offset_addr;//081030 记录当前sector的首地址 
			U8_ptr = (INT8U*)(temp_buffer_addr + ((INT32U)offset_addr - temp_read_start_addr) );//081031
		}
		else
		{
			offset_addr += 2;
		}
		char_temp0 = *(U8_ptr++);
		char_temp1 = *(U8_ptr++);
		temp = (INT32U)((char_temp1<<8) | char_temp0);
		T_InstrumentStartSection_1[i] |= (INT32U)temp;//保留b31
	}
	//已将 T_InstrumentStartSection 载入 
	offset_addr += 2;
	temp_sector_start = offset_addr;//081030 记录当前sector的首地址 
	temp_read_start_addr = offset_addr & 0xfffffe00;//081228
	ret = read_two_sector(fd_idi, (offset_addr & 0xfffffe00), temp_buffer_addr, mode);
	if(ret < 0)
	{
		SPU_err();
		return ret;
	}
	U8_ptr = (INT8U*)(temp_buffer_addr + ((INT32U)offset_addr - temp_read_start_addr) );
	char_temp0 = *(U8_ptr++);
	char_temp1 = *(U8_ptr++);
	temp = (INT32U)((char_temp1<<8) | char_temp0);
	//已让 U16_ptr 指向inst_tab_pitchTable的开始 
	dwSize = (INT32S)temp;
	for(i=0;i<500;i++)
	{
		T_InstrumentPitchTable_1[i] = 0x00000000;
	}
	for(i=0; i<dwSize; i++)
	{
		offset_addr1 = offset_addr + 2;
		if((offset_addr1 - temp_sector_start) >= 512)//081030
		{
			temp_read_start_addr = offset_addr1 & 0xfffffe00;//081228
			ret = read_two_sector(fd_idi, (offset_addr1 & 0xfffffe00), temp_buffer_addr, mode);
			if(ret < 0)
			{
				SPU_err();
				return ret;
			}
			offset_addr = offset_addr1;
			temp_sector_start = offset_addr;//081030 记录当前sector的首地址 
			U8_ptr = (INT8U*)(temp_buffer_addr + ((INT32U)offset_addr - temp_read_start_addr) );//081031
		}
		else
		{
			offset_addr += 2;
		}
		char_temp0 = *(U8_ptr++);
		char_temp1 = *(U8_ptr++);
		temp = (INT32U)((char_temp1<<8) | char_temp0);
		T_InstrumentPitchTable_1[i] = (INT32U)temp;//081031
	}
	//已将 T_InstrumentPitchTable 载入到T_InstrumentPitchTable_1[]----------------------OK
	for(i=0;i<128;i++)
	{
		if((T_InstrumentStartSection_1[i] & 0x80000000) == 0x00000000)//b31为0，则该乐器不需要载入 
		{
			for(j=(T_InstrumentStartSection_1[i]&0x0000ffff); j<(T_InstrumentStartSection_1[i+1]&0x0000ffff); j++)
			{
				T_InstrumentPitchTable_1[j] = 0x00000000;//将pitchtable清0，表示该section不需要载入 
			}
		}
	}
	//已将T_InstrumentPitchTable_1[]中不需要载入的section对应的单元清0 
	offset_addr += 2;
	temp_sector_start = offset_addr;//081030 记录当前sector的首地址 
	temp_read_start_addr = offset_addr & 0xfffffe00;//081228
	ret = read_two_sector(fd_idi, (offset_addr & 0xfffffe00), temp_buffer_addr, mode);
	if(ret < 0)
	{
		SPU_err();
		return ret;
	}
	U8_ptr = (INT8U*)(temp_buffer_addr + ((INT32U)offset_addr - temp_read_start_addr) );
	char_temp0 = *(U8_ptr++);
	char_temp1 = *(U8_ptr++);
	temp2 = (INT32U)((char_temp1<<8) | char_temp0);
	temp = temp2;//low word of size
	offset_addr += 2;//081030
	U8_ptr = (INT8U*)(temp_buffer_addr + ((INT32U)offset_addr - temp_read_start_addr) );
	char_temp0 = *(U8_ptr++);
	char_temp1 = *(U8_ptr++);
	temp2 = (INT32U)((char_temp1<<8) | char_temp0);
	temp1 = temp2;//high word of size
	dwSize = (INT32S)((temp1<<16) | temp);//size of instrument sectors
	total_inst_1 = dwSize;
#if USER_DEFINE_MALLOC_EN == 1
    if(pfunc_user_memory_malloc)
    {
        inst_temp_buffer_addr = (INT32U)pfunc_user_memory_malloc(dwSize*8);
    }
    else
    {
		SPU_err();
		return -1;
    }
#else
	inst_temp_buffer_addr = (INT32U)user_memory_malloc(dwSize*8);
#endif
	if((inst_temp_buffer_addr&0x80000000) || (inst_temp_buffer_addr==0))
	{
		SPU_err();
		return -1;
	}
	U8_ptr1 = (INT8U*)inst_temp_buffer_addr;
	for(i=0; i<dwSize*4; i++)
	{
		offset_addr1 = offset_addr + 2;
		if((offset_addr1 - temp_sector_start) >= 512)//081030
		{
			temp_read_start_addr = offset_addr1 & 0xfffffe00;//081228
			ret = read_two_sector(fd_idi, (offset_addr1 & 0xfffffe00), temp_buffer_addr, mode);
			if(ret < 0)
			{
				SPU_err();
				return ret;
			}
			offset_addr = offset_addr1;
			temp_sector_start = offset_addr;//081030 记录当前sector的首地址 
			U8_ptr = (INT8U*)(temp_buffer_addr + ((INT32U)offset_addr - temp_read_start_addr) );//081031
		}
		else
		{
			offset_addr += 2;
		}
		*U8_ptr1++ = (INT8U)*(U8_ptr++);//081031
		*U8_ptr1++ = (INT8U)*(U8_ptr++);//081031
	}
	//已将tab中的instrument section的地址和长度的那一段完全拷贝到inst_temp_buffer_addr这个buffer中； 
	for(i=0;i<500;i++)
	{
		T_InstrumentSectionAddr_1[i] = 0x00000000;
	}
	for(i=0; i<total_inst_1; i++)
	{
		U8_ptr = (INT8U*)(inst_temp_buffer_addr + i*8);
		char_temp0 = *(U8_ptr++);//081031
		char_temp1 = *(U8_ptr++);
		char_temp2 = *(U8_ptr++);
		char_temp3 = *(U8_ptr++);
		temp2 = (INT32U)((char_temp3<<24) | (char_temp2<<16) | (char_temp1<<8) | char_temp0);//081031
		temp_addr = temp2;
		char_temp0 = *(U8_ptr++);//081031
		char_temp1 = *(U8_ptr++);
		char_temp2 = *(U8_ptr++);
		char_temp3 = *(U8_ptr++);
		temp2 = (INT32U)((char_temp3<<24) | (char_temp2<<16) | (char_temp1<<8) | char_temp0);//081031
		temp_size = temp2;
		if(T_InstrumentPitchTable_1[i])
		{
#if USER_DEFINE_MALLOC_EN == 1
            if(pfunc_user_memory_malloc)
            {
                temp = (INT32U)pfunc_user_memory_malloc(temp_size);
            }
            else
            {
                SPU_err();
                return -1;
            }
#else
			temp = (INT32U)user_memory_malloc(temp_size);
#endif
			if((temp&0x80000000) || (temp==0))
			{
				SPU_err();
				return -1;
			}
			offset_addr = lib_start_addr1 + temp_addr;
			j = temp_size;//j表示需要读取的字节数 
			temp_read_start_addr = offset_addr & 0xfffffe00;//081228
			ret = read_two_sector(fd_idi, (offset_addr & 0xfffffe00), temp_buffer_addr, mode);
			if(ret < 0)
			{
				SPU_err();
				return ret;
			}
			U8_ptr = (INT8U*)(temp_buffer_addr + ((INT32U)offset_addr - temp_read_start_addr) );//081030
			U8_ptr1 = (INT8U*)temp;//081030
			offset_addr1 = (offset_addr & 0xfffffe00) + 0x00000200;//下一个sector的起始地址 
			while((offset_addr < offset_addr1) && (j>0))
			{
				*U8_ptr1++ = *U8_ptr++;//081030
				offset_addr += 1;//081030
				j -= 1;//081030
			}
			for(;j>=512;)//一次读一个sector
			{
				ret = read_a_sector(fd_idi, (offset_addr & 0xfffffe00), (INT32U)U8_ptr1, mode);
				if(ret < 0)
				{
					SPU_err();
					return ret;
				}
				j -= 512;
				offset_addr += 512;//512 byte
				U8_ptr1 += 512;//256 word//081030
				if(offset_addr & 0x000001ff)
				{
					SPU_err();
					return ret;
				}
			}
			if(j>0)
			{
				temp_read_start_addr = offset_addr & 0xfffffe00;//081228
				ret = read_two_sector(fd_idi, (offset_addr & 0xfffffe00), temp_buffer_addr, mode);
				if(ret < 0)
				{
					SPU_err();
					return ret;
				}
				U8_ptr = (INT8U*)(temp_buffer_addr + ((INT32U)offset_addr - temp_read_start_addr) );//081030
				while(j>0)
				{
					*U8_ptr1++ = *U8_ptr++;//081030
					offset_addr += 1;//081030
					j -= 1;//081030
				}
			}
			T_InstrumentSectionAddr_1[i] = (INT32U*)temp;
		}
	}
#if USER_DEFINE_MALLOC_EN == 1
    if(pfunc_user_memory_free)
    {
        pfunc_user_memory_free((void *)inst_temp_buffer_addr);
        pfunc_user_memory_free((void *)temp_buffer_addr);
    }
    else
    {
		SPU_err();
		return -1;
    }
#else
	user_memory_free((void *)inst_temp_buffer_addr);
	//已将需要载入的instrument sections拷贝到申请的RAM区域中，并将申请的ram区域的首地址写入T_InstrumentSectionAddr_1[]中 
	user_memory_free((void *)temp_buffer_addr);
#endif

	return total_midi;
}

INT32S SPU_load_comb_adpcm_to_RAM(INT16S fd_adpcm_comb, INT32U adpcm_index, INT8U mode)//mode 0: from nand flash, others:from FS
{//以byte为单位进行访问 
	INT32U offset_addr,offset_addr1;
	INT32U total_adpcm;
	INT32U temp_buffer_addr;//081029
	INT32U temp_read_start_addr;
	INT8U  *U8_ptr,*U8_ptr1;//081029
	INT32S ret;
	INT32S S_temp;
	INT32S j;
	INT32U temp_addr,temp_size;
	INT8U char_temp3,char_temp2,char_temp1,char_temp0;//后面定义的局部变量地址越来越小//081029
	//-----------------------------------------------------------------------//get every single file 's offset
	SPU_Free_adpcm_data_temp_buffer();//释放用于存放ADPCM数据的内存 
	S_temp = (INT32S)user_memory_malloc(512*2);//512 byte per sector;
    //S_temp = (INT32S)&spu_test_ram_buffer[0];
    //S_temp = (INT32S)&spu_512_byte_per_sector_buffer[0];
	if(S_temp <= 0)
	{
		SPU_err();
		return S_temp;
	}
	temp_buffer_addr = (INT32U)S_temp;
	offset_addr = adpcm_comb_offset_addr;//数据在文件中的偏移地址//081029
	temp_read_start_addr = offset_addr & 0xfffffe00;//081228
	ret = read_two_sector(fd_adpcm_comb, (offset_addr & 0xfffffe00), temp_buffer_addr, mode);
	if(ret < 0)
	{
		SPU_err();
		return ret;
	}
	U8_ptr = (INT8U*)(temp_buffer_addr + ((INT32U)offset_addr - temp_read_start_addr) );
	char_temp0 = *(U8_ptr++);
	char_temp1 = *(U8_ptr++);
	char_temp2 = *(U8_ptr++);
	char_temp3 = *(U8_ptr++);
	total_adpcm = (INT32U)((char_temp3<<24) | (char_temp2<<16) | (char_temp1<<8) | char_temp0);
	if(adpcm_index >= total_adpcm)
	{
		SPU_err();
		return ret;
	}
	offset_addr += 4;//offset += 4 byte
	offset_addr += adpcm_index*8;
	temp_read_start_addr = offset_addr & 0xfffffe00;//081228
	ret = read_two_sector(fd_adpcm_comb, (offset_addr & 0xfffffe00), temp_buffer_addr, mode);
	if(ret < 0)
	{
		SPU_err();
		return ret;
	}
	U8_ptr = (INT8U*)(temp_buffer_addr + ((INT32U)offset_addr - temp_read_start_addr) );
	char_temp0 = *(U8_ptr++);
	char_temp1 = *(U8_ptr++);
	char_temp2 = *(U8_ptr++);
	char_temp3 = *(U8_ptr++);
	temp_addr = (INT32U)((char_temp3<<24) | (char_temp2<<16) | (char_temp1<<8) | char_temp0);
	char_temp0 = *(U8_ptr++);
	char_temp1 = *(U8_ptr++);
	char_temp2 = *(U8_ptr++);
	char_temp3 = *(U8_ptr++);
	temp_size = (INT32U)((char_temp3<<24) | (char_temp2<<16) | (char_temp1<<8) | char_temp0);
	//得到adpcm数据的相对地址和长度 
	if(flag_malloc_adpcm_ram_buffer)//1:需要申请RAM
	{
		S_temp = (INT32S)user_memory_malloc(temp_size);
        //S_temp = (INT32S)&spu_test_ram_buffer[0];
	}
	else
	{
		S_temp = (INT32S)adpcm_ram_buffer_addr;
	}
	if(S_temp <= 0)
	{
		SPU_err();
		return S_temp;
	}
	adpcm_data_temp_buffer_addr = (INT32U)S_temp;
	offset_addr = adpcm_comb_offset_addr + temp_addr;
	j = temp_size;//j表示需要读取的字节数 
	temp_read_start_addr = offset_addr & 0xfffffe00;//081228
	ret = read_two_sector(fd_adpcm_comb, (offset_addr & 0xfffffe00), temp_buffer_addr, mode);
	if(ret < 0)
	{
		SPU_err();
		return ret;
	}
	U8_ptr = (INT8U*)(temp_buffer_addr + ((INT32U)offset_addr - temp_read_start_addr) );
	U8_ptr1 = (INT8U*)adpcm_data_temp_buffer_addr;
	offset_addr1 = (offset_addr & 0xfffffe00) + 0x00000200;//下一个sector的起始地址 
	while((offset_addr < offset_addr1) && (j>0))
	{
		*U8_ptr1++ = *U8_ptr++;//081030
		offset_addr += 1;//081030
		j -= 1;//081030
	}
	for(;j>=512;)//一次读一个sector
	{
		ret = read_a_sector(fd_adpcm_comb, (offset_addr & 0xfffffe00), (INT32U)U8_ptr1, mode);
		if(ret < 0)
		{
			SPU_err();
			return ret;
		}
		j -= 512;
		offset_addr += 512;//512 byte
		U8_ptr1 += 512;//256 word//081030
		if(offset_addr & 0x000001ff)
		{
			SPU_err();
			return ret;
		}
	}
	if(j>0)
	{
		temp_read_start_addr = offset_addr & 0xfffffe00;//081228
		ret = read_two_sector(fd_adpcm_comb, (offset_addr & 0xfffffe00), temp_buffer_addr, mode);
		if(ret < 0)
		{
			SPU_err();
			return ret;
		}
		U8_ptr = (INT8U*)(temp_buffer_addr + ((INT32U)offset_addr - temp_read_start_addr) );//081030
		while(j>0)
		{
			*U8_ptr1++ = *U8_ptr++;//081030
			offset_addr += 1;//081030
			j -= 1;//081030
		}
	}
	//已将ADPCM数据load到申请的RAM中，并将RAM地址赋给adpcm_data_temp_buffer_addr
	user_memory_free((void *)temp_buffer_addr);
	return total_adpcm;
}

INT32S SPU_play_comb_adpcm_FixCH(INT8U uiPan, INT8U uiVelocity, INT8U uiSPUChannel)
{
	if(adpcm_data_temp_buffer_addr)
	{
		SPU_PlayPCM_NoEnv_FixCH((INT32U*)adpcm_data_temp_buffer_addr, uiPan, uiVelocity, uiSPUChannel);
		return 0;
	}
	else
		return -1;
}

INT32S SPU_check_fill_midi_ring_buffer(void)
{
	INT32S ret = 0;

	midi_ring_buffer_ri = SPU_get_midi_ring_buffer_ri();
	switch(midi_ring_buffer_wi)
	{
		case 0://起始处 
			if(midi_ring_buffer_ri >= (MIDI_RING_BUFFER_SIZE/2) )
			{
				if(MIDI_PlayFlag & STS_MIDI_REPEAT_ON)
					if(remain_midi_length == 0)//重新读取midi数据的开始部分 
					{
						SPU_read_anterior_mididata_to_ringbuffer();
						return 0;
					}
				while(remain_midi_length)
				{

					ret = read_a_sector(static_fd_idi, (currt_midi_data_offset & 0xfffffe00), (midi_ring_buffer_addr + midi_ring_buffer_wi),SPU_load_data_mode);
					if(ret < 0)
					{
						SPU_err();
						return ret;
					}
					currt_midi_data_offset += 512;
					midi_ring_buffer_wi += 512;
					if(remain_midi_length >= 512)
						remain_midi_length -= 512;//表示可读的剩余midi长度 
					else
						remain_midi_length = 0;
					if(remain_midi_length == 0)
					{
						midi_ring_buffer_wi = MIDI_RING_BUFFER_SIZE/2;
					}
					if(midi_ring_buffer_wi >= (MIDI_RING_BUFFER_SIZE/2) )
					{
						midi_ring_buffer_wi = MIDI_RING_BUFFER_SIZE/2;
						break;
					}
				}
			}
			break;

		case (MIDI_RING_BUFFER_SIZE/2)://1/2处
			if((midi_ring_buffer_ri < MIDI_RING_BUFFER_SIZE/2) || (midi_ring_buffer_ri >= (MIDI_RING_BUFFER_SIZE/2 + MIDI_RING_BUFFER_SIZE/4)) )
			{
				if(MIDI_PlayFlag & STS_MIDI_REPEAT_ON)
					if(remain_midi_length == 0)//重新读取midi数据的开始部分
					{
						SPU_read_anterior_mididata_to_ringbuffer();
						return 0;
					}
				while(remain_midi_length)
				{
					ret = read_a_sector(static_fd_idi, (currt_midi_data_offset & 0xfffffe00), (midi_ring_buffer_addr + midi_ring_buffer_wi),SPU_load_data_mode);
					if(ret < 0)
					{
						SPU_err();
						return ret;
					}
					currt_midi_data_offset += 512;
					midi_ring_buffer_wi += 512;
					if(remain_midi_length >= 512)
						remain_midi_length -= 512;//表示可读的剩余midi长度
					else
						remain_midi_length = 0;
					if(remain_midi_length == 0)
					{
						midi_ring_buffer_wi = MIDI_RING_BUFFER_SIZE/2 + MIDI_RING_BUFFER_SIZE/4;
					}
					if(midi_ring_buffer_wi >= (MIDI_RING_BUFFER_SIZE/2 + MIDI_RING_BUFFER_SIZE/4))
					{
						midi_ring_buffer_wi = MIDI_RING_BUFFER_SIZE/2 + MIDI_RING_BUFFER_SIZE/4;
						break;
					}
				}
			}
			break;

		case (MIDI_RING_BUFFER_SIZE/2 + MIDI_RING_BUFFER_SIZE/4)://3/4处
			if(midi_ring_buffer_ri < (MIDI_RING_BUFFER_SIZE/2 + MIDI_RING_BUFFER_SIZE/4) )
			{
				if(MIDI_PlayFlag & STS_MIDI_REPEAT_ON)
					if(remain_midi_length == 0)//重新读取midi数据的开始部分
					{
						SPU_read_anterior_mididata_to_ringbuffer();
						return 0;
					}
				while(remain_midi_length)
				{

					ret = read_a_sector(static_fd_idi, (currt_midi_data_offset & 0xfffffe00), (midi_ring_buffer_addr + midi_ring_buffer_wi),SPU_load_data_mode);
					if(ret < 0)
					{
						SPU_err();
						return ret;
					}
					currt_midi_data_offset += 512;
					midi_ring_buffer_wi += 512;
					if(remain_midi_length >= 512)
						remain_midi_length -= 512;//表示可读的剩余midi长度
					else
						remain_midi_length = 0;
					if(remain_midi_length == 0)
					{
						midi_ring_buffer_wi = 0;
					}
					if(midi_ring_buffer_wi >= MIDI_RING_BUFFER_SIZE)
					{
						midi_ring_buffer_wi = 0;
						break;
					}
				}
			}
			break;

		default:
				return 0;
				break;
	}
	return ret;
}

INT32S spu_user_malloc_set(INT32S (*mallc_func)(INT32U size), INT32S (*free_func)(void *ptr))
{
	if(mallc_func && free_func) {
		pfunc_user_memory_malloc = mallc_func;
		pfunc_user_memory_free = free_func;

        return 0;
	}
	else
        return -1;
}
//-----------------------------------------------------------------
