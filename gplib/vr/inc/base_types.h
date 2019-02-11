/*
	Copyright (c) 2002 Cyberon Corp.  All right reserved.
	File: base_types.h
	Desc: Define all base types
	Author: Alan
	Date: 2002/7/21
	FixBug: 2002/11/18 William
	Version : 1.30.00
*/

#ifndef	__BASE_TYPES_H
#define	__BASE_TYPES_H

#if defined(_WIN32) && !defined(__SYMBIAN32__)

	#include <windows.h>
	
	typedef signed char INT8;
	typedef unsigned short USHORT;
	typedef unsigned int UINT;
	typedef unsigned long ULONG;
	typedef wchar_t WCHAR;
	typedef signed int INTEGER;

	#ifdef _WIN32
		#ifdef UNICODE
		#undef UNICODE
		#endif
		//#define UNICODE WCHAR 
		typedef WCHAR UNICODE;
	#else
		//#define UNICODE unsigned short 
		typedef unsigned short UNICODE;
	#endif

#elif defined(__SUNPLUS__)

	#include <general.h>

	typedef SINT8	CHAR; 		/*typedef signed char CHAR;*/
	typedef UINT8	BYTE;		/*typedef unsigned char BYTE;*/
	typedef SINT8	INT8;		/*typedef signed char INT8;*/
	typedef SINT16	SHORT;		/*typedef signed short SHORT;*/
	typedef UINT16	WORD;		/*typedef unsigned short WORD;*/
	typedef UINT16	USHORT;		/*typedef unsigned short USHORT;*/
	typedef SINT32	LONG;		/*typedef signed long LONG;*/
	typedef UINT32	DWORD;		/*typedef unsigned long DWORD;*/
	typedef UINT32	ULONG;		/*typedef unsigned long ULONG;*/
	typedef SINT32	INT;		/*typedef signed int INT;*/
	typedef UINT32	UINT;		/*typedef unsigned int UINT;*/
	typedef SINT32  BOOL;		/*typedef int BOOL;*/
	typedef SINT32	INTEGER;	/*typedef signed int INTEGER;*/
	typedef void	VOID;
	typedef	UINT8*	PBYTE;		/*typedef unsigned char*  PBYTE; unsigned 8 bit data */ 

	#ifndef UNICODE
	//#define UNICODE wchar_t
	typedef wchar_t UNICODE;
	#endif

#else

	#if defined(__LINUX__)
	#include <ctype.h>
	#include <wchar.h>
	#endif

//	typedef signed char CHAR;
	typedef unsigned char BYTE;
	typedef signed short SHORT;
	typedef unsigned short WORD;
	typedef signed long LONG;
	typedef unsigned int DWORD;
	typedef signed int INT;
	typedef unsigned int UINT;
	#if !defined(TARGET_OS_IPHONE) && !defined(TARGET_IPHONE_SIMULATOR)
		typedef int BOOL;
	#endif
	typedef void VOID;
//	typedef signed char INT8;
	typedef unsigned short USHORT;
	typedef unsigned long ULONG;
	typedef unsigned short WCHAR;
	typedef signed int INTEGER;

	//#define UNICODE WORD
	typedef WORD UNICODE;

#endif


#if defined(__SYMBIAN32__)
	#include <e32def.h>
	
	#ifndef HANDLE
	typedef void *HANDLE;
	#endif
	
	typedef struct tagFILETIME
	{
	    DWORD dwLowDateTime;
	    DWORD dwHighDateTime;
	}	FILETIME;
#else
	#ifndef HANDLE
	//#define HANDLE	VOID*
	typedef void *HANDLE;
	#endif
	
	#ifndef _WIN32
	typedef struct tagFILETIME
	{
	    DWORD dwLowDateTime;
	    DWORD dwHighDateTime;
	}	FILETIME;
	#endif
#endif

#include <stdlib.h>
#include <stdio.h>
//#include <string.h> // not include system string.h, but using the "bsr_string.h"

#ifdef LPVOID
#undef LPVOID
#endif
#define LPVOID	void*

#ifndef TRUE
#define TRUE  ( 1 == 1 )
#endif

#ifndef FALSE
#define FALSE ( 1 == 0 )
#endif

#ifndef NULL
#define NULL	((VOID*)0)
#endif

#define PNULL	((void*) 0)

#ifndef STATIC
#define STATIC 	static
#endif

#if defined(_WIN32) && !defined(__SYMBIAN32__)
	#define EXPAPI WINAPI
#else
	#define EXPAPI
#endif

#if defined (__SYMBIAN32__)
	#ifdef DLL_EXPORT
		#define DLLAPI EXPORT_C
	#else
		#define DLLAPI IMPORT_C
	#endif
#elif defined (_WIN32)
	#ifdef DLL_EXPORT
		#define DLLAPI __declspec(dllexport)
	#elif defined(_LIB)
		#define DLLAPI
	#else
		#define DLLAPI __declspec(dllimport)
	#endif
#endif

//#include "Portable.h"

#ifndef SAFE_DELETE
#define SAFE_DELETE(a)	if ( (a) != NULL ) {delete (a); (a) = NULL;}
#endif

#ifndef SAFE_DELETEARRAY
#define SAFE_DELETEARRAY(a)	if ( (a) != NULL ) {delete [](a); (a) = NULL;}
#endif

#ifndef SAFE_FREE
#define SAFE_FREE(a)	if ( (a) != NULL ) {free(a); (a) = NULL;}
#endif

#endif	/* #ifndef	__BASE_TYPES_H */
