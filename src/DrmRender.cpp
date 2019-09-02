#include <stdint.h>
#include <stdlib.h>	//	malloc, free
#include <unistd.h>	//	close
#include <stdio.h>
#include <errno.h>	//	errno
#include <string.h>	//	strerror

#include <sys/types.h>	//	open
#include <sys/stat.h>
#include <fcntl.h>

#include <sys/ioctl.h>
#include <sys/mman.h>

#include <drm/drm_fourcc.h>
//#include <drm/drm.h>
#include <xf86drm.h>
#include <xf86drmMode.h>

#include <nx_video_alloc.h>

#include "DrmRender.h"

#include "Util.h"

#define ROUND_UP_16(num) (((num)+15)&~15)
#define ROUND_UP_32(num) (((num)+31)&~31)

#define	MAX_DRM_BUFFERS		64

struct DRM_DSP_INFO
{
	int			drmFd;
	uint32_t	planeID;
	uint32_t	crtcID;
	uint32_t	format;
	DRM_RECT	srcRect;		//	Source RECT
	DRM_RECT	dstRect;		//	Destination RECT

	int32_t		numBuffers;
	uint32_t	bufferIDs[MAX_DRM_BUFFERS];

	NX_VID_MEMORY_INFO vidMem[MAX_DRM_BUFFERS];
	NX_VID_MEMORY_INFO prevMem;
};

static int32_t DrmIoctl( int32_t fd, unsigned long request, void *pArg )
{
	int32_t ret;

	do {
		ret = ioctl(fd, request, pArg);
	} while (ret == -1 && (errno == EINTR || errno == EAGAIN));

	return ret;
}


static int32_t ImportGemFromFlink( int32_t fd, uint32_t flinkName )
{
	struct drm_gem_open arg;

	memset( &arg, 0x00, sizeof( drm_gem_open ) );

	arg.name = flinkName;
	if (DrmIoctl(fd, DRM_IOCTL_GEM_OPEN, &arg)) {
		return -EINVAL;
	}

	return arg.handle;
}

static int32_t FindDrmBufferIndex( DRM_DSP_HANDLE hDsp, NX_VID_MEMORY_INFO *pMem )
{
	int32_t i=0;

	for( i = 0 ; i < hDsp->numBuffers ; i ++ )
	{
		if( hDsp->vidMem[i].dmaFd[0] == pMem->dmaFd[0] )
		{
			//	already added memory
			return i;
		}
	}
	return -1;
}

#ifndef ALIGN
#define	ALIGN(X,N)	( (X+N-1) & (~(N-1)) )
#endif

static int32_t AddDrmBuffer( DRM_DSP_HANDLE hDsp, NX_VID_MEMORY_INFO *pMem )
{
	int32_t err;
	int32_t newIndex = hDsp->numBuffers;
	//	Add
	uint32_t handles[4] = {0,};
	uint32_t pitches[4] = {0,};
	uint32_t offsets[4] = {0,};
	uint32_t offset = 0;
	uint32_t strideWidth[3] = { 0, };
	uint32_t strideHeight[3] = { 0, };

	int32_t i;

	if (DRM_FORMAT_YUV420 == pMem->format)
	{
		strideWidth[0] = ROUND_UP_32(pMem->stride[0]);
		strideWidth[1] = ROUND_UP_16(strideWidth[0] >> 1);
		strideWidth[2] = strideWidth[1];
		strideHeight[0] = ROUND_UP_16(pMem->height);
		strideHeight[1] = ROUND_UP_16(pMem->height >> 1);
		strideHeight[2] = strideHeight[1];
	}
	else if (DRM_FORMAT_NV12 == pMem->format)
	{
		strideWidth[0] = ROUND_UP_32(pMem->stride[0]);
		strideWidth[1] = ROUND_UP_16(strideWidth[0]);
		strideHeight[0] = ROUND_UP_16(pMem->height);
		strideHeight[1] = ROUND_UP_16(pMem->height);
	}
	else
	{
		printf ( "Fail, not support pMem->format = %d\n", pMem->format );
		return -1;
	}

	if (DRM_FORMAT_YUV420 == pMem->format)
	{
		for( i = 0; i < 3; i++ )	//numPlane
		{
			handles[i] = (DRM_FORMAT_YUV420 == pMem->format) ?
			ImportGemFromFlink( hDsp->drmFd, pMem->flink[0] ) :
			ImportGemFromFlink( hDsp->drmFd, pMem->flink[i] );

			pitches[i] = strideWidth[i];
			offsets[i] = offset;

			offset += ( (DRM_FORMAT_YUV420 == pMem->format) ? (strideWidth[i] * strideHeight[i]) : 0 );
		}
	}
	else if (DRM_FORMAT_NV12 == pMem->format)
	{
		for( i = 0; i < 2; i++ )	//numPlane
		{
			handles[i] = (DRM_FORMAT_NV12 == pMem->format) ?
			ImportGemFromFlink( hDsp->drmFd, pMem->flink[0] ) :
			ImportGemFromFlink( hDsp->drmFd, pMem->flink[i] );

			pitches[i] = strideWidth[i];
			offsets[i] = offset;

			offset += ( (DRM_FORMAT_NV12 == pMem->format) ? (strideWidth[i] * strideHeight[i]) : 0 );
		}
	}
	else
	{
		printf ( "Fail, not support pMem->format = %d\n", pMem->format );
		return -1;
	}

	hDsp->bufferIDs[newIndex] = 0;

	err = drmModeAddFB2( hDsp->drmFd, pMem->width, pMem->height,
		pMem->format, handles, pitches, offsets, &hDsp->bufferIDs[newIndex], 0);
	if( err < 0 )
	{
		printf("drmModeAddFB2() failed !(%d)\n", err);
		return -1;
	}

	hDsp->vidMem[newIndex] = *pMem;
	hDsp->numBuffers ++;

	return newIndex;
}
//
//	Create DRM Display
//
DRM_DSP_HANDLE CreateDrmDisplay( int fd )
{
	DRM_DSP_HANDLE hDsp;

	if( 0 > drmSetClientCap( fd, DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1) )
	{
		printf("drmSetClientCap() failed !!!!\n");
		return  NULL;
	}

	hDsp = (DRM_DSP_HANDLE)calloc(sizeof(struct DRM_DSP_INFO), 1);
	hDsp->drmFd = fd;

	return hDsp;
}


//
//	Initialize DRM Display Device
//
int32_t InitDrmDisplay( DRM_DSP_HANDLE hDsp, uint32_t planeId, uint32_t crtcId, uint32_t format,
						DRM_RECT srcRect, DRM_RECT dstRect )
{
	hDsp->planeID = planeId;
	hDsp->crtcID = crtcId;
	hDsp->format = format;
	hDsp->srcRect = srcRect;
	hDsp->dstRect = dstRect;
	return 0;
}


//
//	Update Buffer
//
int32_t UpdateBuffer( DRM_DSP_HANDLE hDsp, NX_VID_MEMORY_INFO *pMem, NX_VID_MEMORY_INFO **pOldMem )
{
	int32_t err;
	int32_t index = FindDrmBufferIndex( hDsp, pMem );
	// NX_VID_MEMORY_INFO *pAddedMem;

	if( 0 > index ){
		index = AddDrmBuffer( hDsp, pMem );
	}

	// pAddedMem = &hDsp->vidMem[index];

	// printf("Index = %d, drmFd(%d), planeId(%d), crtcId(%d), bufferIDs(%d), srcRect(%d,%d,%d,%d), dstRect(%d,%d,%d,%d)\n",
	// 	index, hDsp->drmFd, hDsp->planeID, hDsp->crtcID, hDsp->bufferIDs[index],
	// 	hDsp->srcRect.x, hDsp->srcRect.y, hDsp->srcRect.width, hDsp->srcRect.height,
	// 	hDsp->dstRect.x, hDsp->dstRect.y, hDsp->dstRect.width, hDsp->dstRect.height );

	err = drmModeSetPlane( hDsp->drmFd, hDsp->planeID, hDsp->crtcID, hDsp->bufferIDs[index], 0,
			hDsp->dstRect.x, hDsp->dstRect.y, hDsp->dstRect.width, hDsp->dstRect.height,
			hDsp->srcRect.x<<16, hDsp->srcRect.y<<16, hDsp->srcRect.width<<16, hDsp->srcRect.height<<16);

	if( 0 > err )
	{
		printf("drmModeSetPlane() Failed!!(%s(%d))\n", strerror(err), err );
	}

	// if( pOldMem )
	// {
	// 	if( hDsp->pPrevMem )
	// 		*pOldMem = hDsp->pPrevMem;
	// 	else
	// 		*pOldMem = NULL;
	// }
	// hDsp->pPrevMem = pMem;

	return err;
}


void DestroyDrmDisplay( DRM_DSP_HANDLE hDsp )
{
	int32_t i;
	for( i = 0; i < hDsp->numBuffers; i++ )
	{
		if( hDsp->bufferIDs[i] )
		{
			drmModeRmFB( hDsp->drmFd, hDsp->bufferIDs[i] );
		}
	}

	free( hDsp );
}
