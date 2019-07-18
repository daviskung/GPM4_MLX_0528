#define TH32x32_REG_ID	    	0x1A //0x1A << 1(need shfit 1 bit !!)
#define TH32x32_EEPROM_ID	    0x50 //0x50 << 1(need shfit 1 bit !!)
#define MLX90640_SLAVE_ADDR	    0x33 // SA = 0X33

#define C_ISR_TEST_PIN				IO_D9  //0x08			// IOA10


typedef struct {      
		INT8U		DeviceAdd;
		INT8U		RegAdd;				  
		INT8U		trimValue;	
} sensor32x32_cmdCode;

#define	WRITE_CMD				0x00
#define	READ_CMD				0x00


#define	TH32x32_CONFIG_REG			0x01
#define	TH32x32_STATUS_REG			0x02
#define	TH32x32_MBIT_TRIM			0x03
#define	TH32x32_BIAS_TRIM_TOP		0x04
#define	TH32x32_BIAS_TRIM_BOT		0x05
#define	TH32x32_CLK_TRIM			0x06
#define	TH32x32_BPA_TRIM_TOP		0x07
#define	TH32x32_BPA_TRIM_BOT		0x08
#define	TH32x32_PU_SDA_TRIM			0x09
#define	TH32x32_READ_DATA_TOP		0x0a
#define	TH32x32_READ_DATA_BOM		0x0b

#define CONFIG_REG_WAKEUP			0x01	// wake
#define CONFIG_REG_START_B0_WAKEUP	0x09	// start / wake 
#define CONFIG_REG_START_B1_WAKEUP	0x19	// start / wake 
#define CONFIG_REG_START_B2_WAKEUP	0x29	// start / wake 
#define CONFIG_REG_START_B3_WAKEUP	0x39	// start / wake 

#define ReadElecOffset_VDD_MEAS		0x0f	// start / VDD_MEAS / blink /wake
#define READ_ELEC_OFFSET			0x0b	// start / blink /wake
#define MBIT_TRIM_12BIT_DEFAULT_REF_CAL 		0x2c	//  (m+4) bit as ADC resolution 
#define MBIT_TRIM_12BIT_DEFAULT		0x0c	//  (m+4) bit as ADC resolution 
#define MBIT_TRIM_10BIT_DEFAULT		0x1a	
#define MBIT_TRIM_11BIT_DEFAULT		0x1b

#define BIAS_TRIM_TOP_SAMPLE_VAL	0x0c		
#define BIAS_TRIM_BOM_SAMPLE_VAL	0x0c		
#define CLK_TRIM_SAMPLE_VAL			0x14
#define BPA_TRIM_TOP_SAMPLE_VAL		0x0c
#define BPA_TRIM_BOM_SAMPLE_VAL		0x0c
#define PU_SDA_TRIM_SAMPLE_VAL		0x88	//  internal pull up resistor 100 kOhm
//#define PU_SDA_TRIM_SAMPLE_VAL		0x11	//  internal pull up resistor 1 kOhm
//#define PU_SDA_TRIM_SAMPLE_VAL		0x21	//  internal pull up resistor 1 kOhm(scl)/ 10 kOhm(sda)
#define CLKTRIMDefault 				0x15 
#define CLKTRIM_8MHz  				0x1A	
#define CLKTRIM_10MHz  				0x20	//0x20 to let it run with 10 Hz
#define CLKTRIM_13MHz  				0x3F	//0x20 to let it run with 10 Hz

#define CONVERT_WAIT_TIME		15

//mode[in]:i2c restart without stop or not: 
// 1:I2C_RESTART_WITHOUT_STOP 0:I2C_RESTART_WITH_STOP

#define TH32x32_I2C_RESTART_MODE	    1
#define ReadBlock0				0x09
#define Read_VDD_MEAS_Block0	0x0D

#define COLOR_TABLE_OVR_RoomTemp 		20 // 大於 室溫 3 度
#define COLOR_TABLE_UNDER_RoomTemp 		60 // 低於 室溫 6 度
#define NOISE_OVR_RoomTemp 				20 // 大於 室溫 3 度


#define AdrPixCMin 0x00
#define AdrPixCMax 0x04
#define AdrGradScale 0x08
#define AdrTableNumber 0x0B 		//changed to 0x0B with Rev 0.14 and adpated TN readout
#define AdrEpsilon 0x0D

#define AdrMBITPixC 0x1A
#define AdrBIASPixC 0x1B
#define AdrCLKPixC 0x1C
#define AdrBPAPixC 0x1D
#define AdrPUPixC 0x1E

//#define AdrVddMeasTh1 0x26
#define AdrVddCalib 0x26
#define AdrPTATTh1 0x3C
#define AdrVddTh1 0x46

#define AdrPTATGrad 0x34

#define AdrVddScaling 0x4E
#define AdrVddScalingOff 0x4F

#define AdrMBITUser 0x60
#define AdrBIASUser 0x61
#define AdrCLKUser 0x62
#define AdrBPAUser 0x63
#define AdrPUUser 0x64

#define AdrDevID 0x74
#define AdrNrOfDefPix 0x7F
#define AdrDeadPix 		0x80	//currently reserved space for 24 Pixel
#define AdrDeadPixMask 	0xB0	//currently reserved space for 24 Pixel


/*
#define MLX90640_AdrDevID 		0x2407
#define MLX90640_AdrStatus		0x8000
#define MLX90640_AdrRegister1 	0x800D
#define MLX90640_AdrConfig 		0x800F

#define MLX90640_EEAddrRegister1 		0x240C
#define MLX90640_EEAddrRegister2		0x240D
#define MLX90640_EEAddrConfig 			0x240E
#define MLX90640_EEAddrInternal_I2C		0x240F

#define MLX90640_EEAddrstart 		0x2400
*/

#define AdrGlobalOffset 0x54
#define AdrGlobalGain 0x55

//#define AdrVddCompValues2 0x340
//#define AdrVddCompValues 0x540
#define AdrVddCompGrad 0x340
#define AdrVddCompOff 0x540
#define AdrTh1 0x740
#define AdrTh2 0xF40
#define AdrPixC 0x1740

#define BIAScurrentDefault 0x05
#define CLKTRIMDefault 0x15 //0x20 to let it run with 10 Hz
#define BPATRIMDefault 0x0C
//#define MBITTRIMDefault 0x0C
#define PUTRIMDefault	0x88

#define MAXNROFDEFECTS	5


//pixelcount etc. for 32x32d
#define Pixel 1024				//=32x32
#define PixelEighth 128
#define PixelOfBlock 128
#define LINE 32
#define rowNumEnd_32 32
#define COLUMN 32
#define DATALength 1292//1098					//length of first packet
#define DATALength2 1288//1096					//lenght of second/last packet
#define DataLengthHalf 646
#define PTATamount 8
#define ELOFFSET 1024			//start address of el. Offset
#define ELAMOUNT 256
#define ELAMOUNTHALF 128
#define StackSize 16			//must be choosen by the user!
#define PTATSTARTADSRESS 1282
#define VDDADDRESS 1280

#define GetElEveryFrameX 10		//amount of normal frames to capture after which the el. Offset is fetched
#define STACKSIZEPTAT 30		//should be an even number
#define STACKSIZEVDD 50			//should be an even number
#define VddStackAmount 30

#define ReadToFromTable
#ifdef ReadToFromTable
		//#define HTPA32x32dL2_1HiSiF5_0
		//#define HTPA32x32dL2_1HiSiF5_0_withSiFilter
		//#define HTPA32x32dL3_6HiSi
		//#define HTPA32x32dL3_6HiSi_Rev1
		//#define HTPA32x32dL7_0HiSi
		//#define HTPA32x32dL5_0HiGeF7_7
		//#define HTPA32x32dL1_6HiGe
		//#define HTPA32x32dR1L1_6HiGe_Gain3k3
		//#define HTPA32x32dR1L2_1SiF5_0_N2
		//#define HTPA32x32dL2_1HiSiF5_0_Gain3k3			// FOV=90
		#define HTPA32x32dR1L1k8_0k7HiGe_Bodo				// FOV=105
		//#define HTPA32x32dR1L2_1HiSiF5_0_Precise	
		//#define HTPA32x32dR1L2_1HiSiF5_0_Gain3k3_Extended
		//#define HTPA32x32dR1L2_85Hi_Gain3k3		
		//#define HTPA32x32dR1L3_6HiSi_Rev1_Gain3k3
		//#define HTPA32x32dR1L5_0HiGeF7_7_Gain3k3			// FOV=33
		//#define HTPA32x32dR1L5_0HiGeF7_7_Gain3k3_TaExtended
		//#define HTPA32x32dR1L7_0HiSi_Gain3k3

    #ifdef HTPA32x32dL5_0HiGe
	 	#define TABLENUMBER		79
		#define PCSCALEVAL		100000000 //327000000000		//PixelConst scale value for table... lower 'L' for (long)
		#define NROFTAELEMENTS 	7
		#define NROFADELEMENTS 	471	//130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
		#define TAEQUIDISTANCE	100		//dK
		#define ADEQUIDISTANCE	64		//dig
		#define ADEXPBITS		6		//2^ADEXPBITS=ADEQUIDISTANCE
		#define TABLEOFFSET		512
		#define EQUIADTABLE		//if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
		#ifdef EQUIADTABLE
			#undef FLOATTABLE
		#endif   
    #endif

    #ifdef HTPA32x32dL5_0HiGeF7_7
	 	#define TABLENUMBER		92
		#define PCSCALEVAL		100000000 //327000000000		//PixelConst scale value for table... lower 'L' for (long)
		#define NROFTAELEMENTS 	7
		#define NROFADELEMENTS 	471	//130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
		#define TAEQUIDISTANCE	100		//dK
		#define ADEQUIDISTANCE	64		//dig
		#define ADEXPBITS		6		//2^ADEXPBITS=ADEQUIDISTANCE
		#define TABLEOFFSET		256
		#define EQUIADTABLE		//if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
		#ifdef EQUIADTABLE
			#undef FLOATTABLE
		#endif   
    #endif
	
    #ifdef HTPA32x32dR1L5_0HiGeF7_7_Gain3k3
	 	#define TABLENUMBER		113
		#define PCSCALEVAL		100000000 //327000000000		//PixelConst scale value for table... lower 'L' for (long)
		#define NROFTAELEMENTS 	7
		//#define NROFADELEMENTS 	1595	//130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
		#define NROFADELEMENTS 	130	
		#define TAEQUIDISTANCE	100		//dK
		#define ADEQUIDISTANCE	64		//dig
		#define ADEXPBITS		6		//2^ADEXPBITS=ADEQUIDISTANCE
		#define TABLEOFFSET		1024
		#define EQUIADTABLE		//if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
		#ifdef EQUIADTABLE
			#undef FLOATTABLE
		#endif   
		#define MBITTRIMDefault 0x2C
		#define SensRv 1
    #endif	
	
    #ifdef HTPA32x32dR1L5_0HiGeF7_7_Gain3k3_TaExtended
	 	#define TABLENUMBER		113
		#define PCSCALEVAL		100000000 //327000000000		//PixelConst scale value for table... lower 'L' for (long)
		#define NROFTAELEMENTS 	12
		#define NROFADELEMENTS 	1595	//130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
		#define TAEQUIDISTANCE	100		//dK
		#define ADEQUIDISTANCE	64		//dig
		#define ADEXPBITS		6		//2^ADEXPBITS=ADEQUIDISTANCE
		#define TABLEOFFSET		1024
		#define EQUIADTABLE		//if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
		#ifdef EQUIADTABLE
			#undef FLOATTABLE
		#endif   
		#define MBITTRIMDefault 0x2C
		#define SensRv 1
    #endif		

    #ifdef HTPA32x32dL1_6HiGe
	 	#define TABLENUMBER		101	
		#define PCSCALEVAL		100000000		//PixelConst scale value for table... lower 'L' for (long)
		#define NROFTAELEMENTS 	7
		#define NROFADELEMENTS 	471		//possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
		#define TAEQUIDISTANCE	100		//dK
		#define ADEQUIDISTANCE	64		//dig
		#define ADEXPBITS		6		//2^ADEXPBITS=ADEQUIDISTANCE
		#define TABLEOFFSET		256
		#define EQUIADTABLE		//if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
		#ifdef EQUIADTABLE
			#undef FLOATTABLE
		#endif   
    #endif
	
    #ifdef HTPA32x32dR1L1_6HiGe_Gain3k3
	 	#define TABLENUMBER		119
		#define PCSCALEVAL		100000000 //327000000000		//PixelConst scale value for table... lower 'L' for (long)
		#define NROFTAELEMENTS 	7
		#define NROFADELEMENTS 	1595	//130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
		#define TAEQUIDISTANCE	100		//dK
		#define ADEQUIDISTANCE	64		//dig
		#define ADEXPBITS		6		//2^ADEXPBITS=ADEQUIDISTANCE
		#define TABLEOFFSET		1024
		#define EQUIADTABLE		//if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
		#ifdef EQUIADTABLE
			#undef FLOATTABLE
		#endif   
		#define MBITTRIMDefault 0x2C
		#define SensRv 1
    #endif	

    #ifdef HTPA32x32dL2_1HiSi
	 	#define TABLENUMBER		80
		#define PCSCALEVAL		100000000		//PixelConst scale value for table... lower 'L' for (long)
		#define NROFTAELEMENTS 	7
		#define NROFADELEMENTS 	471	//130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
		#define TAEQUIDISTANCE	100		//dK
		#define ADEQUIDISTANCE	64		//dig
		#define ADEXPBITS		6		//2^ADEXPBITS=ADEQUIDISTANCE
		#define TABLEOFFSET		256
		#define EQUIADTABLE		//if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
		#ifdef EQUIADTABLE
			#undef FLOATTABLE
		#endif
    #endif

    #ifdef HTPA32x32dL2_1HiSiF5_0
	 	#define TABLENUMBER		96
		#define PCSCALEVAL		100000000		//PixelConst scale value for table... lower 'L' for (long)
		#define NROFTAELEMENTS 	7
		#define NROFADELEMENTS 	471	//130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
		#define TAEQUIDISTANCE	100		//dK
		#define ADEQUIDISTANCE	64		//dig
		#define ADEXPBITS		6		//2^ADEXPBITS=ADEQUIDISTANCE
		#define TABLEOFFSET		256
		#define EQUIADTABLE		//if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
		#ifdef EQUIADTABLE
			#undef FLOATTABLE
		#endif
    #endif
	
    #ifdef HTPA32x32dR1L2_1SiF5_0_N2
	 	#define TABLENUMBER		130
		#define PCSCALEVAL		100000000 //327000000000		//PixelConst scale value for table... lower 'L' for (long)
		#define NROFTAELEMENTS 	7
		#define NROFADELEMENTS 	1595	//130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
		#define TAEQUIDISTANCE	100		//dK
		#define ADEQUIDISTANCE	64		//dig
		#define ADEXPBITS		6		//2^ADEXPBITS=ADEQUIDISTANCE
		#define TABLEOFFSET		192
		#define EQUIADTABLE		//if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
		#ifdef EQUIADTABLE
			#undef FLOATTABLE
		#endif   
		#define MBITTRIMDefault 0x2C
		#define SensRv 1
    #endif	
	
    #ifdef HTPA32x32dL2_1HiSiF5_0_Gain3k3
	 	#define TABLENUMBER		114
		#define PCSCALEVAL		100000000 //327000000000		//PixelConst scale value for table... lower 'L' for (long)
		#define NROFTAELEMENTS 	7
		//#define NROFADELEMENTS 	1595	//130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
		#define NROFADELEMENTS 	130
		#define TAEQUIDISTANCE	100		//dK
		#define ADEQUIDISTANCE	64		//dig
		#define ADEXPBITS		6		//2^ADEXPBITS=ADEQUIDISTANCE
		#define TABLEOFFSET		1024
		#define EQUIADTABLE		//if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
		#ifdef EQUIADTABLE
			#undef FLOATTABLE
		#endif   
		#define MBITTRIMDefault 0x2C	//use REF_CAL=2 here. Table does not match, so GlobalGain ist set to 50 % to compensate this.
		#define SensRv 1				//Sensor Revision is set to 1 (Redesign)
    #endif	
	
    #ifdef HTPA32x32dR1L2_1HiSiF5_0_Gain3k3_Extended
	 	#define TABLENUMBER		114
		#define PCSCALEVAL		100000000 //327000000000		//PixelConst scale value for table... lower 'L' for (long)
		#define NROFTAELEMENTS 	12
		#define NROFADELEMENTS 	1595	//130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
		#define TAEQUIDISTANCE	100		//dK
		#define ADEQUIDISTANCE	64		//dig
		#define ADEXPBITS		6		//2^ADEXPBITS=ADEQUIDISTANCE
		#define TABLEOFFSET		1792
		#define EQUIADTABLE		//if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
		#ifdef EQUIADTABLE
			#undef FLOATTABLE
		#endif   
		#define MBITTRIMDefault 0x2C
		#define SensRv 1
    #endif	
 	
    #ifdef HTPA32x32dR1L2_1HiSiF5_0_Precise
	 	#define TABLENUMBER		116
		#define PCSCALEVAL		100000000 //327000000000		//PixelConst scale value for table... lower 'L' for (long)
		#define NROFTAELEMENTS 	22
		#define NROFADELEMENTS 	1000	//130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
		#define TAEQUIDISTANCE	50		//dK
		#define ADEQUIDISTANCE	32		//dig
		#define ADEXPBITS		5		//2^ADEXPBITS=ADEQUIDISTANCE
		#define TABLEOFFSET		1024
		#define EQUIADTABLE		//if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
		#ifdef EQUIADTABLE
			#undef FLOATTABLE
		#endif   
		#define MBITTRIMDefault 0x2C
		#define SensRv 1
    #endif	

    #ifdef HTPA32x32dL2_1HiSiF5_0_withSiFilter
	 	#define TABLENUMBER		97
		#define PCSCALEVAL		100000000		//PixelConst scale value for table... lower 'L' for (long)
		#define NROFTAELEMENTS 	7
		#define NROFADELEMENTS 	471	//130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
		#define TAEQUIDISTANCE	100		//dK
		#define ADEQUIDISTANCE	64		//dig
		#define ADEXPBITS		6		//2^ADEXPBITS=ADEQUIDISTANCE
		#define TABLEOFFSET		256
		#define EQUIADTABLE		//if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
		#ifdef EQUIADTABLE
			#undef FLOATTABLE
		#endif
    #endif
	
    #ifdef HTPA32x32dR1L2_85Hi_Gain3k3
	 	#define TABLENUMBER		127
		#define PCSCALEVAL		100000000 //327000000000		//PixelConst scale value for table... lower 'L' for (long)
		#define NROFTAELEMENTS 	7
		#define NROFADELEMENTS 	1595	//130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
		#define TAEQUIDISTANCE	100		//dK
		#define ADEQUIDISTANCE	64		//dig
		#define ADEXPBITS		6		//2^ADEXPBITS=ADEQUIDISTANCE
		#define TABLEOFFSET		1024
		#define EQUIADTABLE		//if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
		#ifdef EQUIADTABLE
			#undef FLOATTABLE
		#endif   
		#define MBITTRIMDefault 0x2C
		#define SensRv 1
    #endif	

    #ifdef HTPA32x32dL3_6HiSi
	 	#define TABLENUMBER		81
		#define PCSCALEVAL		100000000		//PixelConst scale value for table... lower 'L' for (long)
		#define NROFTAELEMENTS 	7
		#define NROFADELEMENTS 	471	//130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
		#define TAEQUIDISTANCE	100		//dK
		#define ADEQUIDISTANCE	64		//dig
		#define ADEXPBITS		6		//2^ADEXPBITS=ADEQUIDISTANCE
		#define TABLEOFFSET		256
		#define EQUIADTABLE		//if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
		#ifdef EQUIADTABLE
			#undef FLOATTABLE
		#endif
	#endif

    #ifdef HTPA32x32dL3_6HiSi_Rev1
	 	#define TABLENUMBER		106
		#define PCSCALEVAL		100000000		//PixelConst scale value for table... lower 'L' for (long)
		#define NROFTAELEMENTS 	7
		#define NROFADELEMENTS 	471	//130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
		#define TAEQUIDISTANCE	100		//dK
		#define ADEQUIDISTANCE	64		//dig
		#define ADEXPBITS		6		//2^ADEXPBITS=ADEQUIDISTANCE
		#define TABLEOFFSET		256
		#define EQUIADTABLE		//if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
		#ifdef EQUIADTABLE
			#undef FLOATTABLE
		#endif
	#endif
	
    #ifdef HTPA32x32dR1L3_6HiSi_Rev1_Gain3k3
	 	#define TABLENUMBER		117
		#define PCSCALEVAL		100000000		//PixelConst scale value for table... lower 'L' for (long)
		#define NROFTAELEMENTS 	7
		#define NROFADELEMENTS 	1595	//130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
		#define TAEQUIDISTANCE	100		//dK
		#define ADEQUIDISTANCE	64		//dig
		#define ADEXPBITS		6		//2^ADEXPBITS=ADEQUIDISTANCE
		#define TABLEOFFSET		1024
		#define EQUIADTABLE		//if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
		#ifdef EQUIADTABLE
			#undef FLOATTABLE
		#endif
		#define MBITTRIMDefault 0x2C
		#define SensRv 1
	#endif	

    #ifdef HTPA32x32dL7_0HiSi
	 	#define TABLENUMBER		107
		#define PCSCALEVAL		100000000		//PixelConst scale value for table... lower 'L' for (long)
		#define NROFTAELEMENTS 	7
		#define NROFADELEMENTS 	471	//130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
		#define TAEQUIDISTANCE	100		//dK
		#define ADEQUIDISTANCE	64		//dig
		#define ADEXPBITS		6		//2^ADEXPBITS=ADEQUIDISTANCE
		#define TABLEOFFSET		256
		#define EQUIADTABLE		//if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
		#ifdef EQUIADTABLE
			#undef FLOATTABLE
		#endif
	#endif
	
    #ifdef HTPA32x32dR1L7_0HiSi_Gain3k3
	 	#define TABLENUMBER		118
		#define PCSCALEVAL		100000000		//PixelConst scale value for table... lower 'L' for (long)
		#define NROFTAELEMENTS 	7
		#define NROFADELEMENTS 	1595	//130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
		#define TAEQUIDISTANCE	100		//dK
		#define ADEQUIDISTANCE	64		//dig
		#define ADEXPBITS		6		//2^ADEXPBITS=ADEQUIDISTANCE
		#define TABLEOFFSET		640
		#define EQUIADTABLE		//if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
		#ifdef EQUIADTABLE
			#undef FLOATTABLE
		#endif
		#define MBITTRIMDefault 0x2C
		#define SensRv 1
	#endif	

    #ifdef HTPA32x32dL2_1HiSiDLC
		#define TABLENUMBER		83
		#define PCSCALEVAL		100000000		//PixelConst scale value for table
		#define NROFTAELEMENTS 	7
		#define NROFADELEMENTS 	471
		#define TAEQUIDISTANCE	100		//dK
		#define ADEQUIDISTANCE	64		//dig
		#define ADEXPBITS		6		//2^ADEXPBITS=ADEQUIDISTANCE
		#define TABLEOFFSET		512
		#define EQUIADTABLE		//if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
		#ifdef EQUIADTABLE 
			#undef FLOATTABLE
		#endif
    #endif

    #ifdef HTPA32x32dL2_1Si_withSiFilter
		#define TABLENUMBER		88
		#define PCSCALEVAL		100000000		//PixelConst scale value for table
		#define NROFTAELEMENTS 	7
		#define NROFADELEMENTS 	471
		#define TAEQUIDISTANCE	100		//dK
		#define ADEQUIDISTANCE	8		//dig
		#define ADEXPBITS		3		//2^ADEXPBITS=ADEQUIDISTANCE
		#define TABLEOFFSET		64
		#define EQUIADTABLE		//if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
		#ifdef EQUIADTABLE 
			#undef FLOATTABLE
		#endif
    #endif

   #ifdef HTPA32x32dL5_0HiGeMult
	 	#define TABLENUMBER0		79
		#define PCSCALEVAL		100000000 //327000000000		//PixelConst scale value for table... lower 'L' for (long)
		#define NROFTAELEMENTS 	7
		#define NROFADELEMENTS 	251	//130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
		#define TAEQUIDISTANCE	100		//dK
		#define ADEQUIDISTANCE	128		//dig
		#define ADEXPBITS		7		//2^ADEXPBITS=ADEQUIDISTANCE
		#define TABLEOFFSET		256
		#define EQUIADTABLE		//if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
		#ifdef EQUIADTABLE
			#undef FLOATTABLE
		#endif   
    #endif
	
    #ifdef HTPA32x32dR1L1k8_0k7HiGe_Bodo
	 	#define TABLENUMBER		115	
		#define PCSCALEVAL		100000000		//PixelConst scale value for table... lower 'L' for (long)
		#define NROFTAELEMENTS 	10
		//#define NROFADELEMENTS 	471		//possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
		#define NROFADELEMENTS 	130
		#define TAEQUIDISTANCE	100		//dK
		#define ADEQUIDISTANCE	64		//dig
		#define ADEXPBITS		6		//2^ADEXPBITS=ADEQUIDISTANCE
		#define TABLEOFFSET		1024
		#define EQUIADTABLE		//if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
		#ifdef EQUIADTABLE
			#undef FLOATTABLE
		#endif   
		#define MBITTRIMDefault 0x2C
		#define SensRv 1
    #endif	
#endif
