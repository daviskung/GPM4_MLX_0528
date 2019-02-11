
#include "console.h"

#define CMD_HISTORIES 4
#define CMD_BUF_SIZE  128

#if _OPERATING_SYSTEM != _OS_NONE
	#if _OPERATING_SYSTEM == _OS_UCOS2
	#elif _OPERATING_SYSTEM == _OS_FREERTOS
	extern xTaskHandle cmdTaskHandle;
	#else
	#endif
#endif


static cmd_t  *pcmds = NULL;
static unsigned char cmdBuffer[CMD_BUF_SIZE];
static unsigned int pos = 0;
static unsigned int logIn = 0;

static unsigned int histBuf[CMD_HISTORIES][CMD_BUF_SIZE];
static unsigned int histPos[CMD_HISTORIES];
static unsigned int histIns;
static unsigned int histOutput;
static unsigned int histInsWrap;
static unsigned int histOutputWrap;

static void default_cmd_handler(int argc, char *argv[]);

static cmd_t default_cmd_list[] =
{
	{"all",   default_cmd_handler,  NULL },
	{NULL,    NULL,   NULL}
};

static void default_cmd_help(void)
{
	cmd_t *curr;

	curr = pcmds->pnext;
	if (curr != NULL)
		DBG_PRINT("\r\nUsage:\r\n");
	while ( curr )
	{
		DBG_PRINT("    %s help\r\n", curr->cmd);
		curr = curr->pnext;
	}
}

static void default_cmd_handler(int argc, char *argv[])
{
	if (STRCMP(argv[1],"help") == 0)
    {
    	default_cmd_help();
    }
    else
    {
       	default_cmd_help();
    }
}

static void default_cmd_register(void)
{
	cmd_t *pcmd = &default_cmd_list[0];

	while (pcmd->cmd != NULL)
	{
		cmdRegister(pcmd);
		pcmd += 1;
	}
}

static int console_fgetchar(void)
{
	INT8U input_ch = 0;

    #if _DRV_L1_UART2 == 1
    while (drv_l1_uart2_data_get(&input_ch, 0) != STATUS_OK)
    #endif
    #if _DRV_L1_UART1 == 1
	while (drv_l1_uart1_data_get(&input_ch, 0) != STATUS_OK)
	#endif
	{
		#if _OPERATING_SYSTEM != _OS_NONE
            #if _OPERATING_SYSTEM == _OS_UCOS2
            OSTimeDly(1);
            #elif _OPERATING_SYSTEM == _OS_FREERTOS
			if (cmdTaskHandle != NULL)
				ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
			else
				osDelay(1);
            #else
            #error "which OS api for OSTimeDly ?"
            #endif
		#endif
	}
	return (int )(input_ch);
}

static int isaspace(char c)
{
	return (c == ' ' || c == '\t' || c == '\n' || c == '\12');
}

static char *gpstrdup(const char *str)
{
	size_t siz;
	char *copy;

	siz = STRLEN(str) + 1;
	if ((copy = (char *)MEMALLOC(siz)) == NULL)
		return NULL;
	MEMCPY(copy, str, siz);

	return copy;
}

static int setargs(char *args, char **argv)
{
    int count = 0;

    while (isaspace(*args)) ++args;
    while (*args)
    {
        if (argv) argv[count] = args;
        while (*args && !isaspace(*args)) ++args;
        if (argv && *args) *args++ = '\0';
        while (isaspace(*args)) ++args;
        count++;
    }
    return count;
}

static char **parsedargs(char *args, int *argc)
{
    char **argv = NULL;
    int argn = 0;

    if (args && *args)
    {
    	args = (char *)gpstrdup(args);
    	if (args)
    	{
        	argn = setargs(args,NULL);
        	if (argn)
        	{
        		argv = (char **)MEMALLOC((argn+1) * sizeof(char *));
        		if (argv)
    			{
        			*argv++ = args;
        			argn = setargs(args,argv);
    			}
    		}
    	}
	}
    if (args && !argv) FREE(args);

    *argc = argn;
    return argv;
 }

static void freeparsedargs(char **argv)
{
    if (argv)
    {
        FREE(argv[-1]);
        FREE(argv-1);
    }
}

int parse_to_argv(char *input_str, char ***out_av)
{
    char **av;
    int ac;
    char *as = NULL;

    as = input_str;

    av = parsedargs(as,&ac);

    #if 0
    int i;
    printf("== %d\r\n",ac);
    for (i = 0; i < ac; i++)
        DBG_PRINT("[%s]\r\n",av[i]);
    #endif

    *out_av = av;

    return ac;
}

static void cmdProcess(
	unsigned char *cmd,
	unsigned int repeating
)
{
	cmd_t *bc = pcmds;
	unsigned int idx = 0;
	unsigned int copy = 0;
	int argc = 0;
        char **argv = NULL;
        int found = 0;

	/*
	 * Strip the white space from the command.
	 */
	while ( cmd[idx] != '\0' )
	{
		if ( (cmd[idx] != ' ') &&
			 (cmd[idx] != '\t') &&
			 (cmd[idx] != '\r') &&
			 (cmd[idx] != '\n') )
		{
			break;
		}
		idx++;
	}

	if ( idx > 0 )
	{
		/* Reached a non-white space character, compact the string */
		while ( cmd[idx] != '\0' )
		{
			cmd[copy++] = cmd[idx++];
		}
		cmd[copy] = '\0';
	}

	/*
	 * Index points to the end of the string, move backwards.
	 */
	idx = STRLEN(cmd);

	while ( idx > 0 )
	{
		idx--;
		if ( (cmd[idx] == ' ') ||
			 (cmd[idx] == '\t') ||
			 (cmd[idx] == '\r') ||
			 (cmd[idx] == '\n') )
		{
			cmd[idx] = '\0';
		}
		else
		{
			break;
		}
	}

	/*
	 * Find the command.
	 */
	idx = 0;

	while ( cmd[idx] != '\0' )
	{
		if ( (cmd[idx] == ' ') ||
			 (cmd[idx] == '\t') ||
			 (cmd[idx] == '\r') ||
			 (cmd[idx] == '\n') )
		{
			break;
		}
		idx++;
	}

	argc = parse_to_argv((char *)cmd, &argv);

	cmd[idx] = '\0';

	if (argc > 0)
	{
		while ( bc )
		{
			// DBG_PRINT("check %s, %s\r\n", bc->cmd, (argc == 0) ? "" : argv[0]);

			if ( STRCMP(bc->cmd, argv[0]) == 0 )
			{
				(bc->phandler)(argc, argv);
                                found = 1;
				break;
			}
			bc = bc->pnext;
		}
	}

        if (!found)
          DBG_PRINT("command `%s' not found, try `all help'\r\n", (argc == 0) ? "" : argv[0]);

	if (argv != NULL)
    {
        freeparsedargs(argv);
        argv = NULL;
    }

}

static unsigned int cmdIdxIncrease(
	unsigned int *pcmdIdx
)
{
	unsigned int localIdx;
	unsigned int ret = 0;

	localIdx = *pcmdIdx;
	localIdx++;
	if ( localIdx == CMD_HISTORIES )
	{
		localIdx = 0;
		ret = 1;
	}
	*pcmdIdx = localIdx;

	return ret;
}

static unsigned int cmdFlushCopy(
	unsigned int cursorPos,
	unsigned char *pcmdBuf,
	unsigned char *pcmdSrc,
	unsigned int cmdLen
)
{
	if ( cursorPos > 0 )
	{
		for ( ; cursorPos > 0; cursorPos-- )
		{
			DBG_PRINT("\b \b");
			cmdBuffer[cursorPos] = '\0';
		}
	}
	MEMCPY(pcmdBuf, pcmdSrc, cmdLen);

	return 0;
}

void cmdMonitor(void)
{
	unsigned char c;
	unsigned int repeating;
	unsigned int histDownArw;
	static unsigned int upArrowCnt;

	if ( !logIn )
	{
		//if ( c == '\r' )
		{
			logIn = TRUE;
			DBG_PRINT("\r\n\r\ncmd>");
		}
	}
	else
	{
		c = (unsigned char )GETCH();

		switch ( c )
		{
		case '\b':
		case '\x7f':
			if ( pos > 0 )
			{
				DBG_PRINT("\b \b");
				pos--;
			}
			cmdBuffer[pos] = '\0';
			break;

		case '\r':  /* Process the command. */
			DBG_PRINT("\r\n");
			if ( pos )
			{
				/*
				 * Do not place the same last command into the history if the same.
				 */
				if ( STRCMP((unsigned char *)histBuf[histIns], cmdBuffer) )
				{
					if ( cmdIdxIncrease(&histIns) == 1 )
					{
						histInsWrap = 1;
					}
					MEMCPY(histBuf[histIns], cmdBuffer, CMD_BUF_SIZE);
					histPos[histIns] = pos;
				}
				histOutput = histIns;
				histOutputWrap = 0;
				upArrowCnt = 0;
				repeating = FALSE;
			}
			if ( pos )
			{
				cmdProcess(cmdBuffer, repeating);
				pos = 0;
				MEMSET(cmdBuffer, 0, CMD_BUF_SIZE);
				DBG_PRINT("\r\n");
			}
			DBG_PRINT("cmd>");
			break;

		case '[': /* Non ASCII characters, arrow. */
			c = GETCH();
			switch ( c )
			{
			case 'A': /* Key: up arrow */
				if ( histOutputWrap == 1 )
				{
					if ( histOutput == histIns )
					{
						break;
					}
				}
				if ( histInsWrap == 0 )
				{
					if ( histOutput == 0 )
					{
						break;
					}
				}
				upArrowCnt++;
				cmdFlushCopy(
					pos,
					cmdBuffer,
					(unsigned char *)histBuf[histOutput],
					histPos[histOutput]
				);
				pos = histPos[histOutput];
				cmdBuffer[pos + 1] = '\0';
				DBG_PRINT((CHAR *)cmdBuffer);
				if ( histInsWrap == 1 )
				{
					if ( histOutput == 0 )
					{
						histOutput = CMD_HISTORIES - 1;
						histOutputWrap = 1;
					}
					else
					{
						histOutput--;
					}
				}
				else
				{
					if ( histOutput != 0 )
					{
						/* Note that when wrap around does not occur, the least
						 * of index is 1 because it is the first one to be
						 * written.
						 */
						histOutput--;
					}
					/* Nothing to do with histOutput == 0,
					 * because there is no more commands.
					 */
				}
				break;

			case 'B': /* Key: down arrow */
				if ( upArrowCnt <= 1 )
				{
					break;
				}
				upArrowCnt--;
				cmdIdxIncrease(&histOutput);
				histDownArw = histOutput;
				cmdIdxIncrease(&histDownArw);
				cmdFlushCopy(
					pos,
					cmdBuffer,
					(unsigned char *)histBuf[histDownArw],
					histPos[histDownArw]
				);
				pos = histPos[histDownArw];
				cmdBuffer[pos + 1] = '\0';
				DBG_PRINT((CHAR *)cmdBuffer);
				break;

			case 'C': /* Key: right arrow */
				break;
			case 'D': /* Key: left arrow */
				break;
			default:
				break;
			}
			break;

		default:
			if ( (pos < (CMD_BUF_SIZE - 1)) && (c >= ' ') && (c <= 'z') )
			{
				cmdBuffer[pos++] = c;
				cmdBuffer[pos] = '\0';
				DBG_PRINT((CHAR *)cmdBuffer + pos - 1);
			}
			if ( c == '\x7e' )
			{
				cmdBuffer[pos++] = c;
				cmdBuffer[pos] = '\0';
				DBG_PRINT((CHAR *)cmdBuffer + pos - 1);
			}
			break;
		}
	} /* else of if !logged_in */
}

void cmdUartRxIsr(INT8U dev_idx, INT8U ch)
{
	#if _OPERATING_SYSTEM != _OS_NONE
		#if _OPERATING_SYSTEM == _OS_UCOS2
		#elif _OPERATING_SYSTEM == _OS_FREERTOS
		BaseType_t xHigherPriorityTaskWoken = 0;

		vTaskNotifyGiveFromISR(cmdTaskHandle, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
		#else
        #error "which OS api for vTaskNotifyGiveFromISR ?"
		#endif
	#endif
}

void Cmd_Task(void *para)
{
	#if _OPERATING_SYSTEM != _OS_NONE
		#if _OPERATING_SYSTEM == _OS_UCOS2
		#elif _OPERATING_SYSTEM == _OS_FREERTOS
		if (cmdTaskHandle != NULL)
		{
			#if _DRV_L1_UART2 == 1
			drv_l1_uart2_rx_isr_set(cmdUartRxIsr);
			#endif
			#if _DRV_L1_UART1 == 1
			drv_l1_uart1_rx_isr_set(cmdUartRxIsr);
			#endif
			}
		#else
		#endif
	#endif

	default_cmd_register();

	while ( 1 )
	{
		cmdMonitor();
	}
}


void cmdRegister(cmd_t *bc)
{
	cmd_t *prev;
	cmd_t *curr;

	bc->pnext = NULL;
	if ( pcmds == NULL )
	{
		pcmds = bc;
	}
	else
	{
		prev = NULL;
		curr = pcmds;
		while ( curr )
		{
			/* The list is sorted by alphabetic order. */
			if ( STRCMP(bc->cmd, curr->cmd) <= 0 )
			{
				bc->pnext = curr;
				if ( prev )
				{
					prev->pnext = bc;
				}
				else
				{
					pcmds = bc;
				}
				return;
			}
			prev = curr;
			curr = curr->pnext;
		}

		/* Last on the list. */

		prev->pnext = bc;
	} /* else boot_commands */
}
