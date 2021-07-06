//------------------------------------------------------------------------------
//
//	Copyright (C) 2017 Nexell Co. All Rights Reserved
//	Nexell Co. Proprietary & Confidential
//
//	NEXELL INFORMS THAT THIS CODE AND INFORMATION IS PROVIDED "AS IS" BASE
//  AND	WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS
//  FOR A PARTICULAR PURPOSE.
//
//	Module		:
//	File		:
//	Description	:
//	Author		:
//	Export		:
//	History		:
//
//------------------------------------------------------------------------------

#ifndef __NX_V4L2UTILS_H__
#define __NX_V4L2UTILS_H__

#include <nx_video_alloc.h>

#define V4L2_FMT_CONTINUOUS		true

int32_t NX_V4l2GetPlaneNum( uint32_t iFourcc );
int32_t NX_V4l2IsInterleavedFomrat( uint32_t iFourcc );
int32_t NX_V4l2IsContinuousPlane( uint32_t iFourcc );
int32_t NX_V4l2IsFirstCb( uint32_t iFourcc );

uint32_t NX_V4l2GetFileFormat( char *pFile, int32_t bContinuous = V4L2_FMT_CONTINUOUS );
const char* NX_V4l2GetFormatString( uint32_t iFourcc );

int32_t NX_V4l2GetImageInfo( uint32_t iFourcc, int32_t iWidth, int32_t iHeight, int32_t *iSize );
int32_t NX_V4l2LoadMemory( uint8_t *pInSrc, int32_t iWidth, int32_t iHeight, NX_VID_MEMORY_INFO *pOutMemory );
int32_t NX_V4l2DumpMemory( NX_VID_MEMORY_INFO *pInMemory, const char *pOutFile );
int32_t NX_V4l2DumpMemory( NX_VID_MEMORY_INFO *pInMemory, FILE *pFile );

int32_t NX_V4l2ClearMemory( NX_MEMORY_INFO *pMem );

#endif	// __NX_V4L2UTILS_H__
