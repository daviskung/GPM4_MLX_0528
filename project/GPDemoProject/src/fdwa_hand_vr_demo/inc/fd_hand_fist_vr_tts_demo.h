#define DEMO_MODE_SEL                   0x12345678
#define DEMO_FIND_HAND                  0x20
#define TTS_VR_COMMAND_START            0xC0
#define TTS_VR_COMMAND_END              0xCC
#define TTS_VR_FACEID_START             0xD0
#define TTS_VR_FACEID_END               0xDD
#define TTS_VR_RESULT_ERROR             0xE1
#define TTS_VR_RESULT_END               0xEE
#define	DISKUSED		 				FS_SD2//FS_SD //FS_NAND1
#define TTS_JP_EN                       0

typedef int BOOL;

typedef enum
{
    COMMAND_FACE_HAND_FIST_ZERO = 0,
    COMMAND_FIND_HAND_NO_FIST,
    COMMAND_FIND_HAND_WITH_FIST,
    COMMAND_TURN_UP,
    COMMAND_TURN_DOWN,
    COMMAND_FUNCTION_UP,
    COMMAND_FUNCTION_DOWN,
    COMMAND_MODULE_STOP,
    COMMAND_MODULE_START,
    COMMAND_MODULE_PAUSE,
    COMMAND_MODULE_RESUME,
    COMMAND_FIND_FACE_AND_TRACK,
    COMMAND_FIND_FIST_AND_TRACK,
    COMMAND_STATE_PRCESS,
    COMMAND_STATE_MAX
} module_command_state;

typedef struct {
	int	people_group;
	int	people_training_ok;
	int	people_identify_ok;
	int faceid_mode;
	int faceid_identify_cnt;
	int faceid_command_cnt;
}faceid_result_t;

extern INT32S demo_result_lock(void);
extern INT32S demo_result_unlock(void);
extern INT32S tts_vr_precss_get(void);
