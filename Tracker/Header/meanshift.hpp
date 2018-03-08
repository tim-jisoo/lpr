#ifndef MEANSHIFT_HPP_INCLUDED
#define MEANSHIFT_HPP_INCLUDED

#include <math.h>
#include <assert.h>
#include "../../Header/datatype.hpp"

#define MEANSHIFT_PADDING_DIRECTION	-1
#define MEANSHIFT_PADDING_SCALE_W	1.0f
#define MEANSHIFT_PADDING_SCALE_H	0.0f

int meanshift(AREA, uchar*, RECT, RECT*);

#endif // MEANSHIFT_HPP_INCLUDED
