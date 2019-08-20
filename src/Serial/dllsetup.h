#ifndef DLLSETUP_H
#define DLLSETUP_H

#ifndef LINUX
	//Windows
	//#pragma warning (disable : 4231 4251)

	//#ifdef _USRDLL
		//#define DYNAMIC_LINK __declspec(dllexport)
		//#define EXTERN_LINK 
	//#else	
		//#define DYNAMIC_LINK __declspec(dllimport)
		//#define EXTERN_LINK extern
		#define DYNAMIC_LINK
	#endif
#elif LINUX 
	//Linux 
	#define DYNAMIC_LINK

//#endif 

#endif //End DLLSETUP_H
