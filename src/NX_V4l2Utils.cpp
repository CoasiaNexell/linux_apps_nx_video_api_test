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

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

#include <linux/videodev2.h>
#include <videodev2_nxp_media.h>

#include "NX_V4l2Utils.h"

//------------------------------------------------------------------------------
int32_t NX_V4l2GetPlaneNum( uint32_t iFourcc )
{
	int32_t iPlane = 0;
	switch( iFourcc )
	{
		case V4L2_PIX_FMT_YUV420:
		case V4L2_PIX_FMT_YUV420M:
		case V4L2_PIX_FMT_YVU420:
		case V4L2_PIX_FMT_YVU420M:
		case V4L2_PIX_FMT_YUV422P:
		case V4L2_PIX_FMT_YUV422M:
		case V4L2_PIX_FMT_YUV444:
		case V4L2_PIX_FMT_YUV444M:
			iPlane = 3;
			break;

		case V4L2_PIX_FMT_NV12:
		case V4L2_PIX_FMT_NV12M:
		case V4L2_PIX_FMT_NV21:
		case V4L2_PIX_FMT_NV21M:
		case V4L2_PIX_FMT_NV16:
		case V4L2_PIX_FMT_NV16M:
		case V4L2_PIX_FMT_NV61:
		case V4L2_PIX_FMT_NV61M:
		case V4L2_PIX_FMT_NV24:
		case V4L2_PIX_FMT_NV24M:
		case V4L2_PIX_FMT_NV42:
		case V4L2_PIX_FMT_NV42M:
			iPlane = 2;
			break;

		case V4L2_PIX_FMT_GREY:
			iPlane = 1;
			break;

		default:
			break;
	}

	return iPlane;
}

//------------------------------------------------------------------------------
int32_t NX_V4l2IsInterleavedFomrat( uint32_t iFourcc )
{
	int32_t bInterleaved = false;
	switch( iFourcc )
	{
		case V4L2_PIX_FMT_NV12:
		case V4L2_PIX_FMT_NV12M:
		case V4L2_PIX_FMT_NV21:
		case V4L2_PIX_FMT_NV21M:
		case V4L2_PIX_FMT_NV16:
		case V4L2_PIX_FMT_NV16M:
		case V4L2_PIX_FMT_NV61:
		case V4L2_PIX_FMT_NV61M:
		case V4L2_PIX_FMT_NV24:
		case V4L2_PIX_FMT_NV24M:
		case V4L2_PIX_FMT_NV42:
		case V4L2_PIX_FMT_NV42M:
			bInterleaved = true;
			break;

		default:
			break;
	}

	return bInterleaved;
}

//------------------------------------------------------------------------------
int32_t NX_V4l2IsContinuousPlane( uint32_t iFourcc )
{
	int32_t bContinuous = false;
	switch( iFourcc )
	{
		case V4L2_PIX_FMT_YUV420:
		case V4L2_PIX_FMT_YVU420:
		case V4L2_PIX_FMT_YUV422P:
		case V4L2_PIX_FMT_YUV444:
		case V4L2_PIX_FMT_NV12:
		case V4L2_PIX_FMT_NV21:
		case V4L2_PIX_FMT_NV16:
		case V4L2_PIX_FMT_NV61:
		case V4L2_PIX_FMT_NV24:
		case V4L2_PIX_FMT_NV42:
		case V4L2_PIX_FMT_GREY:
			bContinuous = true;
			break;

		default:
			break;
	}

	return bContinuous;
}

//------------------------------------------------------------------------------
int32_t NX_V4l2IsFirstCb( uint32_t iFourcc )
{
	int32_t bFirstCb = false;
	switch( iFourcc )
	{
		case V4L2_PIX_FMT_YUV420:
		case V4L2_PIX_FMT_YUV420M:
		case V4L2_PIX_FMT_YUV422P:
		case V4L2_PIX_FMT_YUV422M:
		case V4L2_PIX_FMT_YUV444:
		case V4L2_PIX_FMT_YUV444M:
		case V4L2_PIX_FMT_NV12:
		case V4L2_PIX_FMT_NV12M:
		case V4L2_PIX_FMT_NV16:
		case V4L2_PIX_FMT_NV16M:
		case V4L2_PIX_FMT_NV24:
		case V4L2_PIX_FMT_NV24M:
			bFirstCb = true;
			break;

		default:
			break;
	}

	return bFirstCb;
}

//------------------------------------------------------------------------------
static char* GetExtension( char *pFile )
{
	struct stat stStat;
	stat( pFile, &stStat );
	if( S_ISDIR( stStat.st_mode ) )
		return NULL;

	char *pPtr = pFile + strlen(pFile) - 1;
	while( pPtr != pFile )
	{
		if( *pPtr == '.' ) break;
		pPtr--;
	}
	return (pPtr != pFile) ? (pPtr + 1) : NULL;
}

//------------------------------------------------------------------------------
uint32_t NX_V4l2GetFileFormat( char *pFile, int32_t bContinuous )
{
	uint32_t iFourcc = 0x00000000;

	char *pExtension = GetExtension( pFile );
	if( NULL == pExtension )
		return iFourcc;

	/* Memory Format */
	if( !strcasecmp( pExtension, "yuv420") )	iFourcc = (bContinuous ? V4L2_PIX_FMT_YUV420 : V4L2_PIX_FMT_YUV420M);
	if( !strcasecmp( pExtension, "yvu420") )	iFourcc = (bContinuous ? V4L2_PIX_FMT_YVU420 : V4L2_PIX_FMT_YVU420M);
 	if( !strcasecmp( pExtension, "yuv422") )	iFourcc = (bContinuous ? V4L2_PIX_FMT_YUV422P : V4L2_PIX_FMT_YUV422M);
 	if( !strcasecmp( pExtension, "yuv444") )	iFourcc = (bContinuous ? V4L2_PIX_FMT_YUV444 : V4L2_PIX_FMT_YUV444M);
	if( !strcasecmp( pExtension, "nv12") )		iFourcc = (bContinuous ? V4L2_PIX_FMT_NV12 : V4L2_PIX_FMT_NV12M);
	if( !strcasecmp( pExtension, "nv16") )		iFourcc = (bContinuous ? V4L2_PIX_FMT_NV16 : V4L2_PIX_FMT_NV16M);
	if( !strcasecmp( pExtension, "nv24") )		iFourcc = (bContinuous ? V4L2_PIX_FMT_NV24 : V4L2_PIX_FMT_NV24M);
	if( !strcasecmp( pExtension, "grey") )		iFourcc = V4L2_PIX_FMT_GREY;
	/* Compressed Format */
	if( !strcasecmp( pExtension, "h264") )		iFourcc = V4L2_PIX_FMT_H264;
	if( !strcasecmp( pExtension, "mpeg2") )		iFourcc = V4L2_PIX_FMT_MPEG2;
	if( !strcasecmp( pExtension, "div3") )		iFourcc = V4L2_PIX_FMT_DIV3;
	if( !strcasecmp( pExtension, "xvid") )		iFourcc = V4L2_PIX_FMT_XVID;
	if( !strcasecmp( pExtension, "divx") )		iFourcc = V4L2_PIX_FMT_DIVX;
	if( !strcasecmp( pExtension, "div4") )		iFourcc = V4L2_PIX_FMT_DIV4;
	if( !strcasecmp( pExtension, "div5") )		iFourcc = V4L2_PIX_FMT_DIV5;
	if( !strcasecmp( pExtension, "div6") )		iFourcc = V4L2_PIX_FMT_DIV6;
	if( !strcasecmp( pExtension, "mpeg4") )		iFourcc = V4L2_PIX_FMT_MPEG4;
	if( !strcasecmp( pExtension, "h263") )		iFourcc = V4L2_PIX_FMT_H263;
	if( !strcasecmp( pExtension, "wmv9") )		iFourcc = V4L2_PIX_FMT_WMV9;
	if( !strcasecmp( pExtension, "wvc1") )		iFourcc = V4L2_PIX_FMT_WVC1;
	if( !strcasecmp( pExtension, "rv8") )		iFourcc = V4L2_PIX_FMT_RV8;
	if( !strcasecmp( pExtension, "rv9") )		iFourcc = V4L2_PIX_FMT_RV9;
	if( !strcasecmp( pExtension, "vp8") )		iFourcc = V4L2_PIX_FMT_VP8;
	if( !strcasecmp( pExtension, "flv1") )		iFourcc = V4L2_PIX_FMT_FLV1;
	if( !strcasecmp( pExtension, "theora") )	iFourcc = V4L2_PIX_FMT_THEORA;
	if( !strcasecmp( pExtension, "mjpeg") )		iFourcc = V4L2_PIX_FMT_MJPEG;

	return iFourcc;
}

//------------------------------------------------------------------------------
const char* NX_V4l2GetFormatString( uint32_t iFourcc )
{
	switch( iFourcc )
	{
		/* Memory Format */
		case V4L2_PIX_FMT_YUV420:	return "YUV420(I420) - YUV420 planar";
		case V4L2_PIX_FMT_YUV420M:	return "YUV420M(I420) - YUV420 non-continuous planar";
		case V4L2_PIX_FMT_YVU420:	return "YVU420(YV12) - YUV420 planar";
		case V4L2_PIX_FMT_YVU420M:	return "YVU420M(YV12) - YUV420 non-continuous planar";
		case V4L2_PIX_FMT_YUV422P:	return "YUV422 - YUV422 planar";
		case V4L2_PIX_FMT_YUV422M:	return "YUV422M - YUV422 non-continuous planar";
		case V4L2_PIX_FMT_YUV444:	return "YUV444 - YUV444 planar";
		case V4L2_PIX_FMT_YUV444M:	return "YUV444M - YUV444 non-continuous planar";
		case V4L2_PIX_FMT_NV12:		return "NV12 - YUV420 planar interleaved";
		case V4L2_PIX_FMT_NV12M:	return "NV12M - YUV420 non-continuous planar interleaved";
		case V4L2_PIX_FMT_NV21:		return "NV21 - YVU420 planar interleaved";
		case V4L2_PIX_FMT_NV21M:	return "NV21M - YVU420 non-continuous planar interleaved";
		case V4L2_PIX_FMT_NV16:		return "NV16 - YUV422 planar interleaved";
		case V4L2_PIX_FMT_NV16M:	return "NV16M - YUV422 non-continuous planar interleaved";
		case V4L2_PIX_FMT_NV61:		return "NV61 - YVU422 planar interleaved";
		case V4L2_PIX_FMT_NV61M:	return "NV61M - YVU422 non-continuous planar interleaved";
		case V4L2_PIX_FMT_NV24:		return "NV24 - YUV444 planar interleaved";
		case V4L2_PIX_FMT_NV24M:	return "NV24M - YUV444 non-continuous planar interleaved";
		case V4L2_PIX_FMT_NV42:		return "NV42 - YVU444 planar interleaved";
		case V4L2_PIX_FMT_NV42M:	return "NV42M - YVU444 non-continuous planar interleaved";
		case V4L2_PIX_FMT_GREY:		return "GREY - Y800 planar";
		/* Compressed Format */
		case V4L2_PIX_FMT_H264:		return "H264 Compressed format";
		case V4L2_PIX_FMT_MPEG2:	return "MPEG2 Compressed format";
		case V4L2_PIX_FMT_DIV3:		return "DIV3 Compressed format";
		case V4L2_PIX_FMT_XVID:		return "XVID Compressed format";
		case V4L2_PIX_FMT_DIVX:		return "DIVX Compressed format";
		case V4L2_PIX_FMT_DIV4:		return "DIV4 Compressed format";
		case V4L2_PIX_FMT_DIV5:		return "DIV5 Compressed format";
		case V4L2_PIX_FMT_DIV6:		return "DIV6 Compressed format";
		case V4L2_PIX_FMT_MPEG4:	return "MPEG4 Compressed format";
		case V4L2_PIX_FMT_H263:		return "H263 Compressed format";
		case V4L2_PIX_FMT_WMV9:		return "WMV9 Compressed format";
		case V4L2_PIX_FMT_WVC1:		return "WVC1 Compressed format";
		case V4L2_PIX_FMT_RV8:		return "RV8 Compressed format";
		case V4L2_PIX_FMT_RV9:		return "RV9 Compressed format";
		case V4L2_PIX_FMT_VP8:		return "VP8 Compressed format";
		case V4L2_PIX_FMT_FLV1:		return "FLV1 Compressed format";
		case V4L2_PIX_FMT_THEORA:	return "THEORA Compressed format";
		case V4L2_PIX_FMT_MJPEG:	return "MJPEG Compressed format";
		default: break;
	}
	return "Unknown format";
}

//------------------------------------------------------------------------------
static int32_t GetChromaSize( NX_VID_MEMORY_INFO *pInMemory, int32_t *iWidth, int32_t *iHeight )
{
	if( NULL == pInMemory )
	{
		printf("Fail, Input Memory. ( %p )\n", pInMemory );
		return -1;
	}

	int32_t iChromaWidth  = 0;
	int32_t iChromaHeight = 0;

	switch( pInMemory->format )
	{
	case V4L2_PIX_FMT_GREY:
		iChromaWidth  = 0;
		iChromaHeight = 0;
		break;

	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YUV420M:
	case V4L2_PIX_FMT_YVU420:
	case V4L2_PIX_FMT_YVU420M:
		iChromaWidth  = pInMemory->width >> 1;
		iChromaHeight = pInMemory->height >> 1;
		break;

	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV12M:
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_NV21M:
		iChromaWidth  = pInMemory->width;
		iChromaHeight = pInMemory->height >> 1;
		break;

	case V4L2_PIX_FMT_YUV422P:
	case V4L2_PIX_FMT_YUV422M:
		iChromaWidth  = pInMemory->width >> 1;
		iChromaHeight = pInMemory->height;
		break;

	case V4L2_PIX_FMT_NV16:
	case V4L2_PIX_FMT_NV16M:
	case V4L2_PIX_FMT_NV61:
	case V4L2_PIX_FMT_NV61M:
		iChromaWidth  = pInMemory->width;
		iChromaHeight = pInMemory->height;
		break;

	case V4L2_PIX_FMT_YUV444:
	case V4L2_PIX_FMT_YUV444M:
		iChromaWidth  = pInMemory->width << 1;
		iChromaHeight = pInMemory->height;
		break;

	case V4L2_PIX_FMT_NV24:
	case V4L2_PIX_FMT_NV24M:
	case V4L2_PIX_FMT_NV42:
	case V4L2_PIX_FMT_NV42M:
		iChromaWidth  = pInMemory->width;
		iChromaHeight = pInMemory->height;
		break;

	default:
		break;
	}

	*iWidth  = iChromaWidth;
	*iHeight = iChromaHeight;

	return 0;
}

//------------------------------------------------------------------------------
int32_t NX_V4l2GetImageInfo( uint32_t iFourcc, int32_t iWidth, int32_t iHeight, int32_t *iSize )
{
	switch( iFourcc )
	{
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YUV420M:
	case V4L2_PIX_FMT_YVU420:
	case V4L2_PIX_FMT_YVU420M:
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV12M:
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_NV21M:
#if ENABLE_DRM_DISPLAY
	case DRM_FORMAT_YUV420:
	case DRM_FORMAT_NV12:
	case DRM_FORMAT_NV21:
#endif
		*iSize = iWidth * iHeight * 3 / 2;
		break;

	case V4L2_PIX_FMT_YUV422P:
	case V4L2_PIX_FMT_YUV422M:
	case V4L2_PIX_FMT_NV16:
	case V4L2_PIX_FMT_NV16M:
	case V4L2_PIX_FMT_NV61:
	case V4L2_PIX_FMT_NV61M:
#if ENABLE_DRM_DISPLAY
	case DRM_FORMAT_YUV422:
	case DRM_FORMAT_NV16:
	case DRM_FORMAT_NV61:
#endif
		*iSize = iWidth * iHeight * 2;
		break;

	case V4L2_PIX_FMT_YUV444:
	case V4L2_PIX_FMT_YUV444M:
	case V4L2_PIX_FMT_NV24:
	case V4L2_PIX_FMT_NV24M:
	case V4L2_PIX_FMT_NV42:
	case V4L2_PIX_FMT_NV42M:
#if ENABLE_DRM_DISPLAY
	case DRM_FORMAT_YUV444:
#endif
		*iSize = iWidth * iHeight * 3;
		break;

	case V4L2_PIX_FMT_GREY:
		*iSize = iWidth * iHeight;
		break;

	default :
		printf("Fail, Not support fourcc. ( 0x%08X - %s )\n",
			iFourcc, NX_V4l2GetFormatString( iFourcc ) );
		return -1;
	}

	return 0;
}

//------------------------------------------------------------------------------
int32_t NX_V4l2LoadMemory( uint8_t *pInSrc, int32_t iWidth, int32_t iHeight, NX_VID_MEMORY_INFO *pOutMemory )
{
	if( NULL == pOutMemory )
	{
		printf("Fail, Output Memory. ( %p )\n", pOutMemory );
		return -1;
	}

	if( pOutMemory->width != iWidth ||
		pOutMemory->height != iHeight )
	{
		printf("Fail, Memory Size. ( input: %d x %d, output: %d x %d )\n",
			iWidth, iHeight, pOutMemory->width, pOutMemory->height );
		return -1;
	}

	uint8_t *pSrc = pInSrc;
	int32_t iLumaStride, iChromaStride;
	int32_t iLumaWidth, iLumaHeight;
	int32_t iChromaWidth, iChromaHeight;

	iLumaStride = pOutMemory->stride[0];
	iChromaStride = pOutMemory->stride[1];

	iLumaWidth = iWidth;
	iLumaHeight = iHeight;
	GetChromaSize( pOutMemory, &iChromaWidth, &iChromaHeight );

	{
		uint8_t *pDst = (uint8_t*)pOutMemory->pBuffer[0];
		for( int32_t i = 0; i < iLumaHeight; i++ )
		{
			memcpy( (void*)pDst, (void*)pSrc, iLumaWidth );
			pDst += iLumaStride;
			pSrc += iLumaWidth;
		}
	}

	for( int32_t i = 1; i < pOutMemory->planes; i++ )
	{
		uint8_t *pDst = (uint8_t*)pOutMemory->pBuffer[i];
		for( int32_t j = 0; j < iChromaHeight; j++ )
		{
			memcpy( (void*)pDst, (void*)pSrc, iChromaWidth );
			pDst += iChromaStride;
			pSrc += iChromaWidth;
		}
	}

	return 0;
}

//------------------------------------------------------------------------------
int32_t NX_V4l2DumpMemory( NX_VID_MEMORY_INFO *pInMemory, const char *pOutFile )
{
	if( NULL == pInMemory )
	{
		printf("Fail, Input Memory. ( %p )\n", pInMemory );
		return -1;
	}

	if( NULL == pOutFile )
	{
		printf("Fail, Output File.\n" );
		return -1;
	}

	if( NULL == pInMemory->pBuffer[0] )
	{
		if( 0 > NX_MapVideoMemory( pInMemory ) )
		{
			printf("Fail, NX_MapVideoMemory().\n");
			return -1;
		}
	}

	FILE *pFile = fopen64( pOutFile, "wb" );
	if( NULL == pFile )
	{
		printf("Fail, fopen(). ( %s )\n", pOutFile );
		return -1;
	}

	int32_t iLumaWidth, iLumaHeight;
	int32_t iChromaWidth, iChromaHeight;

	iLumaWidth  = pInMemory->width;
	iLumaHeight = pInMemory->height;
	GetChromaSize( pInMemory, &iChromaWidth, &iChromaHeight );

	// Save Luma
	{
		uint8_t *pPtr = (uint8_t*)pInMemory->pBuffer[0];
		for( int32_t h = 0; h < iLumaHeight; h++ )
		{
			fwrite( pPtr, 1, iLumaWidth, pFile );
			pPtr += pInMemory->stride[0];
		}
	}

	// Save Chroma
	for( int32_t i = 1; i < pInMemory->planes; i++ )
	{
		uint8_t *pPtr = (uint8_t*)pInMemory->pBuffer[i];
		for( int32_t h = 0; h < iChromaHeight; h++ )
		{
			fwrite( pPtr, 1, iChromaWidth, pFile );
			pPtr += pInMemory->stride[i];
		}
	}

	if( pFile )
	{
		fclose( pFile );
	}

	return 0;
}

//------------------------------------------------------------------------------
int32_t NX_V4l2DumpMemory( NX_VID_MEMORY_INFO *pInMemory, FILE *pFile )
{
	if( pFile == NULL )
	{
		printf("Fail, Input Handle.\n");
		return -1;
	}

	if( NULL == pInMemory )
	{
		printf("Fail, Input Memory. ( %p )\n", pInMemory );
		return -1;
	}

	if( NULL == pInMemory->pBuffer[0] )
	{
		if( 0 > NX_MapVideoMemory( pInMemory ) )
		{
			printf("Fail, NX_MapVideoMemory().\n");
			return -1;
		}
	}

	int32_t iLumaWidth, iLumaHeight;
	int32_t iChromaWidth, iChromaHeight;

	iLumaWidth  = pInMemory->width;
	iLumaHeight = pInMemory->height;
	GetChromaSize( pInMemory, &iChromaWidth, &iChromaHeight );

	// Save Luma
	{
		uint8_t *pPtr = (uint8_t*)pInMemory->pBuffer[0];
		for( int32_t h = 0; h < iLumaHeight; h++ )
		{
			fwrite( pPtr, 1, iLumaWidth, pFile );
			pPtr += pInMemory->stride[0];
		}
	}

	// Save Chroma
	for( int32_t i = 1; i < pInMemory->planes; i++ )
	{
		uint8_t *pPtr = (uint8_t*)pInMemory->pBuffer[i];
		for( int32_t h = 0; h < iChromaHeight; h++ )
		{
			fwrite( pPtr, 1, iChromaWidth, pFile );
			pPtr += pInMemory->stride[i];
		}
	}

	return 0;
}

//------------------------------------------------------------------------------
int32_t NX_V4l2ClearMemory( NX_MEMORY_INFO *pMem )
{
	if( NULL == pMem->pBuffer )
	{
		if( 0 > NX_MapMemory( pMem ) )
		{
			printf("Fail, NX_MapMemory().\n");
			return -1;
		}
	}
	memset( pMem->pBuffer, 0x00, pMem->size );
	return 0;
}
