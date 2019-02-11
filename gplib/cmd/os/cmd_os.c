
#include <stdio.h>
#include "gplib.h"
#include "FreeRTOSConfig.h"

#if (CMD_OS == 1)

#include "console.h"

extern void prvHeapStats( void );

#define cmdMAX_OUTPUT_SIZE 512 // increase this size if there are many task

static const char *osHelpStr =
		"\r\nUsage: os help\r\n"
    	"\r\n       os heap\r\n"
    	"\r\n       os task\r\n"
    	"\r\n       os tasktime\r\n";
		
#if 0
static signed char cOutputString[ cmdMAX_OUTPUT_SIZE ];
static char *pcOutputString = cOutputString;
#else
static char *pcOutputString = NULL; // dont use buffer, directly print out to UART
#endif

// copy from FreeRTOS demo code, thanks to original author.
static BaseType_t prvRunTimeStatsCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
BaseType_t xSpacePadding;
char *pcWriteBufferStart = pcWriteBuffer;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;

    printf("\r\n");

	if (pcWriteBufferStart != NULL)
    {
        const char *pcHeader = "  Abs Time      % Time\r\n****************************************\r\n";
        configASSERT( pcWriteBuffer );

        /* Generate a table of task stats. */
        strcpy( pcWriteBuffer, "Task" );
        pcWriteBuffer += strlen( pcWriteBuffer );

        /* Pad the string "task" with however many bytes necessary to make it the
        length of a task name.  Minus three for the null terminator and half the
        number of characters in	"Task" so the column lines up with the centre of
        the heading. */
        for( xSpacePadding = strlen( "Task" ); xSpacePadding < ( configMAX_TASK_NAME_LEN - 3 ); xSpacePadding++ )
        {
            /* Add a space to align columns after the task's name. */
            *pcWriteBuffer = ' ';
            pcWriteBuffer++;

            /* Ensure always terminated. */
            *pcWriteBuffer = 0x00;
        }

        strcpy( pcWriteBuffer, pcHeader );
        vTaskGetRunTimeStats( pcWriteBuffer + strlen( pcHeader ) );
        printf(pcWriteBufferStart);
    }
    else
    {
        const char *pcHeader = "Task\tAbs Time      % Time\r\n****************************************\r\n";
        printf(pcHeader);
        vTaskGetRunTimeStats(NULL);
    }
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

// copy from FreeRTOS demo code, thanks to original author.
static BaseType_t prvTaskStatsCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
BaseType_t xSpacePadding;
char *pcWriteBufferStart = pcWriteBuffer;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;

    printf("\r\n");

    if (pcWriteBufferStart != NULL)
    {
        const char *pcHeader = "  State\tPriority\tStack\t#\r\n************************************************\r\n";

        configASSERT( pcWriteBuffer );

        /* Generate a table of task stats. */
        strcpy( pcWriteBuffer, "Task" );
        pcWriteBuffer += strlen( pcWriteBuffer );

        /* Pad the string "task" with however many bytes necessary to make it the
        length of a task name.  Minus three for the null terminator and half the
        number of characters in	"Task" so the column lines up with the centre of
        the heading. */
        for( xSpacePadding = strlen( "Task" ); xSpacePadding < ( configMAX_TASK_NAME_LEN - 3 ); xSpacePadding++ )
        {
            /* Add a space to align columns after the task's name. */
            *pcWriteBuffer = ' ';
            pcWriteBuffer++;

            /* Ensure always terminated. */
            *pcWriteBuffer = 0x00;
        }
        strcpy( pcWriteBuffer, pcHeader );
        vTaskList( pcWriteBuffer + strlen( pcHeader ) );
        printf(pcWriteBufferStart);
    }
	else
	{
        const char *pcHeader = "Task\tState\tPriority\tStack\t#\r\n************************************************\r\n";
        printf(pcHeader);
        vTaskList(NULL);
	}

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}
/*-----------------------------------------------------------*/

static void os_cmd_help(void)
{
    printf(osHelpStr);
}

static void os_cmd_handler(int argc, char *argv[])
{
    if (STRCMP(argv[1],"heap") == 0)
	{
        prvHeapStats();
	}
    else if (STRCMP(argv[1],"task") == 0)
    {
		prvTaskStatsCommand(pcOutputString, cmdMAX_OUTPUT_SIZE, NULL);
    }
	else if (STRCMP(argv[1],"tasktime") == 0)
	{
		prvRunTimeStatsCommand(pcOutputString, cmdMAX_OUTPUT_SIZE, NULL);
	}
	else
    {
    	os_cmd_help();
    }
}

static cmd_t os_cmd_list[] =
{
    {"os",  os_cmd_handler,  NULL },
    {NULL,  NULL,   NULL}
};

void os_cmd_register(void)
{
    cmd_t *pcmd = &os_cmd_list[0];

    while (pcmd->cmd != NULL)
    {
        cmdRegister(pcmd);
        pcmd += 1;
    }
}

void OS_Cmd(void)
{
    #if defined(GPLIB_CONSOLE_EN) && (GPLIB_CONSOLE_EN == 1)
    os_cmd_register();
    #endif
}

#endif // CMD_OS == 1
