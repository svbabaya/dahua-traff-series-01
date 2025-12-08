#ifndef __MYTYPEDEFS_H__
#define __MYTYPEDEFS_H__

typedef unsigned char uchar;
typedef unsigned int uint;

#ifdef _FixPoint_
	#include "fixPointClass.h"
	typedef fixPoint _FloatType;
#else
	typedef float _FloatType;
#endif
	
#define TEMPL_1  template <typename TVal>
#define TEMPL_IO template <typename TOut,typename TInp>

#endif
