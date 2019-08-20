//! This file defines the common communication types across all compilers
#ifndef TYPE_H
#define TYPE_H

	//!Definitions needed if not including StdAfx (windows) 
	#ifndef BOOL
	#define BOOL int
	#endif

	#ifndef FALSE
	#define FALSE 0
	#endif

	#ifndef TRUE
	#define TRUE 1
	#endif

	#ifndef DWORD
	#define DWORD unsigned long
	#endif


	//Now typedef all the different size variables depending on the compiler used
	#ifdef _MSC_VER

			typedef unsigned __int64 u64;
			typedef unsigned __int32 u32;
			typedef unsigned __int16 u16;
			typedef unsigned __int8 u8;
			typedef __int64 s64;
			typedef __int32 s32;
			typedef __int16 s16;
			typedef __int8 s8;
 
	#endif

	#ifdef LINUX
			typedef unsigned long	u64;
			typedef unsigned int	u32;
			typedef unsigned short	u16;
			typedef unsigned char	u8;
			typedef long		s64;
			typedef int			s32;
			typedef short		s16;
			typedef char		s8;
	#endif

#endif
