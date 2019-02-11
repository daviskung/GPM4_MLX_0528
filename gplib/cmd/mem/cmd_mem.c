
#include <stdio.h>
#include "gplib.h"
#include "FreeRTOSConfig.h"

#if (CMD_MEM == 1)

#include "console.h"

#define FORMAT_BYTE 		(sizeof(INT8U ))
#define FORMAT_HALF_WORD 	(sizeof(INT16U))
#define FORMAT_WORD 		(sizeof(INT32U))
#define DUMP_CHARS_PER_LINE (16)

static const char *memHelpStr =
		"\r\nUsage: mem help\r\n"
    	"\r\n       mem dump [<b/h/w>] <saddr> [<count>]\r\n"
    	"\r\n       mem fill [<b/h/w>] <saddr> [<count>] <data>\r\n"
		"\r\n       mem search [<b/h/w>] <saddr> [<count>] <pattern>\r\n"
		"\r\n       use '0x' for hex.\r\n"
		"\r\n       <count> must have '+' to indicate number(from 1) to deal with.\r\n"
		"\r\n       b is byte, h is half word(2 byte), w is word(4 byte).\r\n";

static const char *notEnoughParamStr =
		"not enough parameters, try `mem help'\r\n";
static const char *bus_error_msg =
        "bus error (or alignment) error at 0x%08x, operation terminated\r\n";

static INT32U dumpAddr = 0;
static INT32U dumpFmt = FORMAT_WORD;
static INT32U dumpLen = 1;

static void mem_cmd_help(void)
{
    printf(memHelpStr);
}

INT32U safeByteWrite(INT32U addr, INT8U data)
{
	*((volatile INT8U *)addr) = data;

	return TRUE;
}

INT32U safeByteRead(INT32U addr, INT8U *data)
{
	*data = *((volatile INT8U *)addr);

	return TRUE;
}

INT32U safeHalfWordWrite(INT32U addr, INT16U data)
{
	if ( addr & 0x1 )
		return FALSE;

	*((volatile INT16U *)addr) = data;

	return TRUE;
}

INT32U safeHalfWordRead(INT32U addr, INT16U *data)
{
	if ( addr & 0x1 )
		return FALSE;

	*data = *((volatile INT16U *)addr);

	return TRUE;
}

INT32U safeWordWrite(INT32U addr, INT32U data)
{
	if ( addr & 0x3 )
		return FALSE;

	*((volatile INT32U *)addr) = data;

	return TRUE;
}

INT32U safeWordRead(INT32U addr, INT32U *data)
{
	if ( addr & 0x3 )
		return FALSE;

	*data = *((volatile INT32U *)addr);

	return TRUE;
}

void cmdMemDump(int argc, char *argv[])
{
	INT32U newDumpAddr = dumpAddr;
	INT32U newDumpLen = dumpLen;
	INT32U newDumpFmt = dumpFmt;
	INT32U size;
	INT32U value;
	INT32U char_value;
	INT8U  read_memory[DUMP_CHARS_PER_LINE];
	INT8U  data_8;
	INT16U data_16;
	INT32U data_32;
	INT32U bus_error = FALSE;
	INT32U end_addr;
	INT32U argv_idx = 4;

	switch ( argv[2][0] )
	{
		case 'b':
			newDumpFmt = FORMAT_BYTE;
			newDumpAddr = strtoul(argv[3], 0, 0);
			break;
		case 'h':
			newDumpFmt = FORMAT_HALF_WORD;
			newDumpAddr = strtoul(argv[3], 0, 0);
			break;
		case 'w':
			newDumpFmt = FORMAT_WORD;
			newDumpAddr = strtoul(argv[3], 0, 0);
			break;
		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
		case '8':
		case '9':
			newDumpAddr = strtoul(argv[2], 0, 0);
			argv_idx = 3;
			break;
		default:
			printf("\x7 command error with '%s'\r\n", argv[2]);
			return;
	}

	/*
	 * We have to be at the length parameter if provided.
	 */
	if ( argc > argv_idx )
	{
		/*
		 *  Check to see if the address has a plus value to indicate a length.
		 */
		if ( argv[argv_idx][0] == '+' )
		{
            end_addr = strtoul(argv[argv_idx], 0, 0);
			if (end_addr == 0)
				end_addr = 1;
			newDumpLen = end_addr * newDumpFmt;
        }
		else
		{
            end_addr = strtoul(argv[argv_idx], 0, 0);
			if ( end_addr < newDumpAddr )
			{
				printf("\x7 End address must be greater than start address.\r\n");
				return;
			}
			newDumpLen = (end_addr - newDumpAddr + 1) * newDumpFmt;
		}
	}

	/*
	 * Assign the new values. They might not have changed.
	 */

	dumpAddr = newDumpAddr;
	dumpLen = newDumpLen;
	dumpFmt = newDumpFmt;

	/*
	 * Catch a user just enter the dump command with no param
	 * for the first time.
	 */
	if ( dumpLen == 0 )
	{
		dumpLen = 32;
	}

	if ( dumpFmt == 0 )
	{
		dumpFmt = FORMAT_WORD;
	}

	size = dumpFmt;
	switch ( dumpFmt )
	{
		case FORMAT_WORD:
			if ((dumpAddr & 0x3) != 0)
			{
				dumpAddr &= (~0x3);
				dumpLen += 4;
			}
			break;
		case FORMAT_HALF_WORD:
			if ((dumpAddr & 0x1) != 0)
			{
				dumpAddr &= (~0x1);
				dumpLen += 2;
			}
			break;
		default:
			break;
	}

	printf("\r\n");
	value = 0;
	while ( TRUE )
	{
		if ( (value % DUMP_CHARS_PER_LINE) == 0 )
		{
			if ( value )
			{
				printf(" | ");

				for ( char_value = 0; char_value < DUMP_CHARS_PER_LINE; char_value++ )
				{
					if ( (read_memory[char_value] < ' ') || (read_memory[char_value] > 127) )
						printf(".");
					else
						printf("%c", read_memory[char_value]);
				}

				if ( (char_value + value) >= (dumpLen + DUMP_CHARS_PER_LINE) )
				{
					/* dump job is complete */
					printf("\r\n");
					dumpAddr += dumpLen;
					if ( bus_error )
						printf ("\x7 warning bus (or alignment) errors occurred.\r\n");
					return;
				}
				printf("\r\n");
			}
			/* Print the line offset label */
			printf("%08x", dumpAddr + value);
		}

		/* Print an 8 byte boundary */
		if ( (value & (DUMP_CHARS_PER_LINE - 1)) == (DUMP_CHARS_PER_LINE >> 1) )
			printf("-");
		else
			printf(" ");

		switch ( dumpFmt )
		{
			case FORMAT_BYTE:
				if ( safeByteRead(dumpAddr + value, &data_8) )
				{
					printf("%02x", data_8);
					read_memory[(value % DUMP_CHARS_PER_LINE) + 0] = data_8;
				}
				else
				{
					bus_error = TRUE;
					printf("**");
					read_memory[(value % DUMP_CHARS_PER_LINE) + 0] = '*';
				}
				break;

			case FORMAT_HALF_WORD:
				if ( safeHalfWordRead(dumpAddr + value, &data_16) )
				{
					printf("%04x", data_16);
					read_memory[(value % DUMP_CHARS_PER_LINE) + 0] = (INT8U) (data_16 >> 8);
					read_memory[(value % DUMP_CHARS_PER_LINE) + 1] = (INT8U) data_16;
				}
				else
				{
					bus_error = TRUE;
					printf("****");
					read_memory[(value % DUMP_CHARS_PER_LINE) + 0] = '*';
					read_memory[(value % DUMP_CHARS_PER_LINE) + 1] = '*';
				}
				break;

			case FORMAT_WORD:
			default:
				if ( safeWordRead(dumpAddr + value, &data_32) )
				{
					printf("%08x", data_32);
					read_memory[(value % DUMP_CHARS_PER_LINE) + 0] = (INT8U) (data_32 >> 24);
					read_memory[(value % DUMP_CHARS_PER_LINE) + 1] = (INT8U) (data_32 >> 16);
					read_memory[(value % DUMP_CHARS_PER_LINE) + 2] = (INT8U) (data_32 >>  8);
					read_memory[(value % DUMP_CHARS_PER_LINE) + 3] = (INT8U) data_32;
				}
				else
				{
					bus_error = TRUE;
					printf("********");
					read_memory[(value % DUMP_CHARS_PER_LINE) + 0] = '*';
					read_memory[(value % DUMP_CHARS_PER_LINE) + 1] = '*';
					read_memory[(value % DUMP_CHARS_PER_LINE) + 2] = '*';
					read_memory[(value % DUMP_CHARS_PER_LINE) + 3] = '*';
				}
				break;
		} // end switch
		value += size;
	} // end while
}

void cmdMemFill(int argc, char *argv[])
{
	INT32U saddress;
	INT32U eaddress;
	INT32U format;
	INT32U value;
	INT32U bus_error = FALSE;
	INT32U argv_idx = 3;

	if ( argc <= 3 )
	{
		printf("%s", notEnoughParamStr);
		return;
	}

	switch ( argv[2][0] )
	{
		case 'b':
			format = FORMAT_BYTE;
			break;
		case 'h':
			format = FORMAT_HALF_WORD;
			break;
		case 'w':
			format = FORMAT_WORD;
			break;
		default:
			format = FORMAT_WORD; // assume WORD format
			argv_idx = 2;
			break;
	}

	saddress = strtoul(argv[argv_idx], 0, 0);
	eaddress = saddress;

	/*
	 *  Check to see if the address has a plus value to indicate a length.
	 */
	if (argc > argv_idx+1)
	{
		if ( argv[argv_idx+1][0] == '+' )
		{
			/*
			* Need to be one less when a length to get the correct end
			* address as the equal test is for <= not less.
			*/
			eaddress = strtoul(argv[argv_idx+1], 0, 0);
			if (eaddress > 0)
				eaddress -= 1;
			eaddress = saddress + (eaddress*format);
		}
		else if (argc > argv_idx+2)
		{
			eaddress = strtoul(argv[argv_idx+1], 0, 0);
			if ( saddress > eaddress )
			{
				printf("\x7 End address must be greater than start address.\r\n");
				return;
			}
			eaddress = saddress + ((eaddress - saddress) * format);
		}
	}

	/*
	 * Check the source address is less than or equal to the end address.
	 */

	if ( saddress > eaddress )
	{
		printf("\x7 End address must be greater than start address.\r\n");
		return;
	}

	if (argc > argv_idx+2)
		value = strtoul(argv[argv_idx+2], 0, 0);
	else
		value = strtoul(argv[argv_idx+1], 0, 0);

	if ( (argv_idx == 3 && argc > 6) ||  (argv_idx == 2 && argc > 5))
	{
		printf ("extra param ignored.\r\n");
	}

	while ( saddress <= eaddress )
	{
		switch ( format )
		{
			case FORMAT_BYTE:
				bus_error = !safeByteWrite(saddress, value);
				break;
			case FORMAT_HALF_WORD:
				bus_error = !safeHalfWordWrite(saddress, value);
				break;
			case FORMAT_WORD:
				bus_error = !safeWordWrite(saddress, value);
				break;
		}
		if ( bus_error )
		{
			printf(bus_error_msg, saddress);
			return;
		}
		saddress += format;
	} // end while
}

void cmdMemSearch(int argc, char *argv[])
{
	INT32U saddress;
	INT32U eaddress;
	INT32U format;
	INT32U search_value;
	INT32U value;
	INT32U found = 0;
	INT32U bus_error = FALSE;
	INT32U argv_idx = 3;
    INT32U ever_found = 0;

	if ( argc <= 3 )
	{
		printf("%s", notEnoughParamStr);
		return;
	}

	switch ( argv[2][0] )
	{
		case 'b':
			format = FORMAT_BYTE;
			break;
		case 'h':
			format = FORMAT_HALF_WORD;
			break;
		case 'w':
			format = FORMAT_WORD;
			break;
		default:
			format = FORMAT_WORD; // assume WORD format
			argv_idx = 2;
			break;
	}

	saddress = strtoul(argv[argv_idx], 0, 0);
	eaddress = saddress;

	/*
	 *  Check to see if the address has a plus value to indicate a length.
	 */
	if (argc > argv_idx+1)
	{
		if ( argv[argv_idx+1][0] == '+' )
		{
			/*
			* Need to be one less when a length to get the correct end
			* address as the equal test is for <= not less.
			*/
			eaddress = strtoul(argv[argv_idx+1], 0, 0);
			if (eaddress > 0)
				eaddress -= 1;
			eaddress = saddress + (eaddress*format);
		}
		else if (argc > argv_idx+2)
		{
			eaddress = strtoul(argv[argv_idx+1], 0, 0);
			if ( saddress > eaddress )
			{
				printf("\x7 End address must be greater than start address.\r\n");
				return;
			}
			eaddress = saddress + ((eaddress - saddress) * format);
		}
	}

	/*
	 * Check the source address is less than or equal to the end address.
	 */
	if ( saddress > eaddress )
	{
		printf("\x7 End address must be greater than start address.\r\n");
		return;
	}

	if (argc > argv_idx+2)
		search_value = strtoul(argv[argv_idx+2], 0, 0);
	else
		search_value = strtoul(argv[argv_idx+1], 0, 0);

	if ( (argv_idx == 3 && argc > 6) ||  (argv_idx == 2 && argc > 5))
	{
		printf("extra param ignored.\r\n");
	}

	while ( saddress <= eaddress )
	{
		switch ( format )
		{
			case FORMAT_BYTE:
				bus_error = !safeByteRead(saddress, (INT8U*) &value);
				value = value & 0x000000ff;
				break;

			case FORMAT_HALF_WORD:
				bus_error = !safeHalfWordRead(saddress, (INT16U*) &value);
				value = value & 0x0000ffff;
				break;

			case FORMAT_WORD:
				bus_error = !safeWordRead(saddress, &value);
				break;
		}

		if ( bus_error )
		{
			if ( found )
				printf("\r\n");
			printf(bus_error_msg, saddress);
			return;
		}

		if ( search_value == value )
		{
            if (!ever_found)
            {
                printf("found at:\r\n");
                ever_found = 1;
            }

			if ( found > 7 )
			{
				found = 0;
				printf("\r\n");
			}
			found++;
			printf(" %08x", saddress);
		}

		saddress += format;
	} // end while

	if (!ever_found)
        printf("not found");
	printf("\r\n");
}

static void mem_cmd_handler(int argc, char *argv[])
{
    if (STRCMP(argv[1],"dump") == 0)
    {
		cmdMemDump(argc, argv);
    }
	else if (STRCMP(argv[1],"fill") == 0)
	{
		cmdMemFill(argc, argv);
	}
	else if (STRCMP(argv[1],"search") == 0)
	{
		cmdMemSearch(argc, argv);
	}
    else
    {
    	mem_cmd_help();
    }
}

static cmd_t mem_cmd_list[] =
{
    {"mem",  mem_cmd_handler,  NULL },
    {NULL,  NULL,   NULL}
};

void mem_cmd_register(void)
{
    cmd_t *pcmd = &mem_cmd_list[0];

    while (pcmd->cmd != NULL)
    {
        cmdRegister(pcmd);
        pcmd += 1;
    }
}

void Mem_Cmd(void)
{
    #if defined(GPLIB_CONSOLE_EN) && (GPLIB_CONSOLE_EN == 1)
    mem_cmd_register();
    #endif
}

#endif // CMD_MEM == 1
