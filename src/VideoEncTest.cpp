//------------------------------------------------------------------------------
//
//	Copyright (C) 2016 Nexell Co. All Rights Reserved
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
#include <stdlib.h>
#include <sys/types.h>	// open
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>

#include <linux/videodev2.h>
#include <videodev2_nxp_media.h>

#include <nx_video_alloc.h>
#include <nx_video_api.h>

#if ENABLE_CAMERA
#include "NX_CV4l2Camera.h"
#endif

#include "NX_V4l2Utils.h"
#include "Util.h"

#define ENABLE_ASPECT_RATIO
#define SCREEN_WIDTH	(1080)
#define SCREEN_HEIGHT	(1920)

#if ENABLE_DRM_DISPLAY
#include <drm_fourcc.h>
#include "DrmRender.h"
#endif

#define IMAGE_BUFFER_NUM	8
#define IMG_FORMAT			V4L2_PIX_FMT_YUV420  //V4L2_PIX_FMT_NV12M
#define IMG_PLANE_NUM		NX_V4l2GetPlaneNum( IMG_FORMAT )

// #define PLANE_ID			26
// #define CRTC_ID			31
#define PLANE_ID			17
#define CRTC_ID				22

bool IsRunningLoop();
void ExitLoop(bool bExit);

//----------------------------------------------------------------------------------------------------
//
//	Signal Handler
//
static void signal_handler( int32_t sig )
{
	printf("Aborted by signal %s (%d)..\n", (char*)strsignal(sig), sig);

	switch( sig )
	{
		case SIGINT :
			printf("SIGINT..\n"); 	break;
		case SIGTERM :
			printf("SIGTERM..\n");	break;
		case SIGABRT :
			printf("SIGABRT..\n");	break;
		default :
			break;
	}

	if( IsRunningLoop() )
		ExitLoop( true );
	else{
		usleep(1000000);	//	wait 1 seconds for double Ctrl+C operation
		exit(EXIT_FAILURE);
	}
}

//------------------------------------------------------------------------------
static void register_signal( void )
{
	signal( SIGINT,  signal_handler );
	signal( SIGTERM, signal_handler );
	signal( SIGABRT, signal_handler );
}

//------------------------------------------------------------------------------
//
//	Camera Encoder Main
//
static int32_t VpuCamEncMain( CODEC_APP_DATA *pAppData )
{
	NX_V4L2ENC_HANDLE hEnc = NULL;
#if ENABLE_DRM_DISPLAY
	DRM_DSP_HANDLE hDsp = NULL;
#endif

#if ENABLE_CAMERA
	NX_VIP_INFO info;
	NX_CV4l2Camera*	pV4l2Camera = NULL;
#endif
	NX_VID_MEMORY_HANDLE hVideoMemory[IMAGE_BUFFER_NUM];

	int32_t inWidth = 0;
	int32_t inHeight = 0;

	if( (pAppData->width == 0) || (pAppData->height == 0) )
	{
		inWidth = 640;
		inHeight = 480;
	}
	else
	{
		inWidth = pAppData->width;
		inHeight = pAppData->height;
	}

	int32_t ret = 0, i;
	int32_t iPlanes = IMG_PLANE_NUM;

	FILE *fpOut = fopen(pAppData->outFileName, "wb");

	if ( fpOut == NULL)
	{
		printf("input file or output file open error!!\n");
		goto CAM_ENC_TERMINATE;
	}

	register_signal();

	//==============================================================================
	// DISPLAY INITIALIZATION
	//==============================================================================
	{
		int drmFd = open("/dev/dri/card0", O_RDWR);

#if ENABLE_DRM_DISPLAY
		hDsp = CreateDrmDisplay(drmFd);
		DRM_RECT srcRect, dstRect;

		srcRect.x = 0;
		srcRect.y = 0;
		srcRect.width = inWidth;
		srcRect.height = inHeight;
		dstRect.x = 0;
		dstRect.y = 0;
		dstRect.width = inWidth;
		dstRect.height = inHeight;

#ifdef ENABLE_ASPECT_RATIO
		double xRatio = (double)SCREEN_WIDTH/(double)inWidth;
		double yRatio = (double)SCREEN_HEIGHT/(double)inHeight;

		if (xRatio > yRatio)
		{
			dstRect.width = inWidth * yRatio;
			dstRect.height = SCREEN_HEIGHT;
			dstRect.x = abs(SCREEN_WIDTH - dstRect.width)/2;
		}
		else
		{
			dstRect.width = SCREEN_WIDTH;
			dstRect.height = inHeight * xRatio;
			dstRect.y = abs(SCREEN_HEIGHT - dstRect.height) / 2;
		}
#endif

		InitDrmDisplay(hDsp, PLANE_ID, CRTC_ID, DRM_FORMAT_YUV420, srcRect, dstRect );
#endif	//	ENABLE_DRM_DISPLAY
	}

	//==============================================================================
	// CAMERA INITIALIZATION
	//==============================================================================
#if ENABLE_CAMERA
	{
		memset( &info, 0x00, sizeof(info) );

		info.iModule		= 0;
		info.iSensorId		= nx_sensor_subdev;
		info.bUseMipi		= true;

		info.iWidth			= inWidth;
		info.iHeight		= inHeight;
		info.iFpsNum		= 30;
		info.iFpsDen		= 1;

		info.iNumPlane		= 1;

		info.iCropX			= 0;
		info.iCropY			= 0;
		info.iCropWidth		= inWidth;
		info.iCropHeight	= inHeight;

		info.iOutWidth		= inWidth;
		info.iOutHeight		= inHeight;
		pV4l2Camera = new NX_CV4l2Camera();
		if( 0 > pV4l2Camera->Init( &info ) )
		{
			delete pV4l2Camera;
			pV4l2Camera = NULL;

			printf( "Fail, V4l2Camera Init().\n");
			goto CAM_ENC_TERMINATE;
		}

		for( i = 0; i < IMAGE_BUFFER_NUM; i++ )
		{
			hVideoMemory[i] = (NX_VID_MEMORY_INFO*)malloc( sizeof(NX_VID_MEMORY_INFO) );
			memset( hVideoMemory[i], 0, sizeof(NX_VID_MEMORY_INFO) );
			pV4l2Camera->SetVideoMemory( hVideoMemory[i] );
		}
	}
#endif

	//==============================================================================
	// ENCODER INITIALIZATION
	//==============================================================================
	{
		NX_V4L2ENC_PARA encPara;
		uint8_t *pSeqData;
		int32_t seqSize;

		// Initialize Encoder
		if (pAppData->codec == 0)
		{
			pAppData->codec = V4L2_PIX_FMT_H264;
		}
		else if (pAppData->codec == 1)
		{
			pAppData->codec = V4L2_PIX_FMT_MPEG4;
		}
		else if (pAppData->codec == 2)
		{
			pAppData->codec = V4L2_PIX_FMT_H263;
			encPara.profile = V4L2_MPEG_VIDEO_H263_PROFILE_P3;
		}
		else if (pAppData->codec == 3)
		{
			pAppData->codec = V4L2_PIX_FMT_MJPEG;
		}
		else
		{
			printf("Fail, not supported codec!!!\n");
			goto CAM_ENC_TERMINATE;
		}

		hEnc = NX_V4l2EncOpen(pAppData->codec);
		if (hEnc == NULL)
		{
			printf("video encoder open is failed!!!");
			ret = -1;
			goto CAM_ENC_TERMINATE;
		}

		memset(&encPara, 0, sizeof(encPara));
		encPara.width = inWidth;
		encPara.height = inHeight;
		encPara.fpsNum = (pAppData->fpsNum) ? (pAppData->fpsNum) : (30);
		encPara.fpsDen = (pAppData->fpsDen) ? (pAppData->fpsDen) : (1);
		encPara.keyFrmInterval = (pAppData->gop) ? (pAppData->gop) : (encPara.fpsNum / encPara.fpsDen);
		encPara.bitrate = pAppData->kbitrate * 1024;
		encPara.maximumQp = (pAppData->maxQp) ? (pAppData->maxQp) : (0);
		encPara.rcVbvSize = (pAppData->vbv) ? (pAppData->vbv) : (0);
		encPara.disableSkip = 0;
		encPara.RCDelay = 0;
		encPara.gammaFactor = 0;
		encPara.initialQp = pAppData->qp;
		encPara.numIntraRefreshMbs = 0;
		encPara.searchRange = 0;
		encPara.enableAUDelimiter = 0;
		encPara.imgFormat = IMG_FORMAT;
		encPara.imgBufferNum = IMAGE_BUFFER_NUM;
		encPara.imgPlaneNum = iPlanes;
		encPara.pImage = hVideoMemory[0];

		if (pAppData->codec == V4L2_PIX_FMT_MJPEG)
			encPara.jpgQuality = (pAppData->qp == 0) ? (90) : (pAppData->qp);

		ret = NX_V4l2EncInit(hEnc, &encPara);
		if (ret < 0)
		{
			printf("video encoder initialization is failed!!!");
			goto CAM_ENC_TERMINATE;
		}

		ret = NX_V4l2EncGetSeqInfo(hEnc, &pSeqData, &seqSize);
		if (ret < 0)
		{
			printf("Getting Sequence header is failed!!!");
			goto CAM_ENC_TERMINATE;
		}

		// Write Sequence Data
		if (seqSize > 0)
			fwrite((void *)pSeqData, 1, seqSize, fpOut);
	}
	//==============================================================================
	// PROCESS UNIT
	//==============================================================================
	{
		int32_t frmCnt = 0;
		uint64_t startTime, endTime;

		while (IsRunningLoop())
		{
			NX_V4L2ENC_IN encIn;
			NX_V4L2ENC_OUT encOut;
			NX_VID_MEMORY_INFO *pBuf = NULL;
			int32_t BufferIndex = 0;

#if ENABLE_CAMERA
			//Camera
			if( 0 >  pV4l2Camera->DequeueBuffer( &BufferIndex, &pBuf ) )
			{
				printf( "Fail, DequeueBuffer().\n" );
				break;
			}
#endif


#if ENABLE_DRM_DISPLAY
			UpdateBuffer(hDsp, pBuf, NULL);
#endif
			memset(&encIn, 0, sizeof(NX_V4L2ENC_IN));
			memset(&encOut, 0, sizeof(NX_V4L2ENC_OUT));

			encIn.pImage = pBuf;
			encIn.imgIndex = BufferIndex;
			encIn.forcedIFrame = 0;
			encIn.forcedSkipFrame = 0;
			encIn.quantParam = pAppData->qp;

			startTime = NX_GetTickCount();
			ret = NX_V4l2EncEncodeFrame(hEnc, &encIn, &encOut);
			endTime = NX_GetTickCount();

			if (ret < 0)
			{
				printf("[%d] Frame]NX_V4l2EncEncodeFrame() is failed!!\n", frmCnt);
				break;
			}
#if ENABLE_CAMERA
			if( 0 >  pV4l2Camera->QueueBuffer( pBuf ) )
			{
				printf( "Fail, DequeueBuffer().\n" );
				break;
			}
#endif

			printf("[%04d Frm]Size = %5d, Type = %1d, Time = %6llu\n", frmCnt, encOut.strmSize, encOut.frameType, (endTime - startTime));

			if (fpOut && encOut.strmSize > 0)
				fwrite((void *)encOut.strmBuf, 1, encOut.strmSize, fpOut);

			frmCnt++;
		}//while end
	}

	//==============================================================================
	// TERMINATION
	//==============================================================================
CAM_ENC_TERMINATE:
	if (hEnc)
		NX_V4l2EncClose(hEnc);

#if ENABLE_CAMERA
	if( pV4l2Camera )
	{
		pV4l2Camera->Deinit();
		delete pV4l2Camera;
		pV4l2Camera = NULL;
	}
#endif

	for( i = 0; i < IMAGE_BUFFER_NUM; i++ )
	{
		if( hVideoMemory[i] )
		{
			free( hVideoMemory[i] );
			hVideoMemory[i] = NULL;
		}
	}

	if (fpOut)
		fclose(fpOut);

	printf("Cam Encode End!!\n" );

	return ret;
}

//------------------------------------------------------------------------------
static int32_t VpuEncPerfMain(CODEC_APP_DATA *pAppData)
{
	NX_V4L2ENC_HANDLE hEnc = NULL;
#if ENABLE_DRM_DISPLAY
	DRM_DSP_HANDLE hDsp = NULL;
#endif
	NX_VID_MEMORY_HANDLE hImage[IMAGE_BUFFER_NUM];

	int32_t inWidth = pAppData->width;
	int32_t inHeight = pAppData->height;
	int32_t ret = 0, i;
	int32_t iPlaneNum = 0;
	int32_t iFormat = 0;

	FILE *fpIn = fopen(pAppData->inFileName, "rb");
	FILE *fpOut = fopen(pAppData->outFileName, "wb");

	if (fpIn == NULL || fpOut == NULL)
	{
		printf("input file or output file open error!!\n");
		goto ENC_TERMINATE;
	}

	register_signal();

	//==============================================================================
	// DISPLAY INITIALIZATION
	//==============================================================================
	{
		int drmFd = open("/dev/dri/card0", O_RDWR);

#if ENABLE_DRM_DISPLAY
		hDsp = CreateDrmDisplay(drmFd);
		DRM_RECT srcRect, dstRect;

		srcRect.x = 0;
		srcRect.y = 0;
		srcRect.width = inWidth;
		srcRect.height = inHeight;
		dstRect.x = 0;
		dstRect.y = 0;
		dstRect.width = inWidth;
		dstRect.height = inHeight;

#ifdef ENABLE_ASPECT_RATIO
		double xRatio = (double)SCREEN_WIDTH/(double)inWidth;
		double yRatio = (double)SCREEN_HEIGHT/(double)inHeight;

		if (xRatio > yRatio)
		{
			dstRect.width = inWidth * yRatio;
			dstRect.height = SCREEN_HEIGHT;
			dstRect.x = abs(SCREEN_WIDTH - dstRect.width)/2;
		}
		else
		{
			dstRect.width = SCREEN_WIDTH;
			dstRect.height = inHeight * xRatio;
			dstRect.y = abs(SCREEN_HEIGHT - dstRect.height) / 2;
		}
#endif

		InitDrmDisplay(hDsp, PLANE_ID, CRTC_ID, DRM_FORMAT_YUV420, srcRect, dstRect );
#endif	//	ENABLE_DRM_DISPLAY
	}

	//==============================================================================
	// ENCODER INITIALIZATION
	//==============================================================================
	{
		NX_V4L2ENC_PARA encPara;
		uint8_t *pSeqData;
		int32_t seqSize;

		// Initialize Encoder
		if (pAppData->codec == 0)
		{
			pAppData->codec = V4L2_PIX_FMT_H264;
		}
		else if (pAppData->codec == 1)
		{
			pAppData->codec = V4L2_PIX_FMT_MPEG4;
		}
		else if (pAppData->codec == 2)
		{
			pAppData->codec = V4L2_PIX_FMT_H263;
			encPara.profile = V4L2_MPEG_VIDEO_H263_PROFILE_P3;
		}
		else if (pAppData->codec == 3)
		{
			pAppData->codec = V4L2_PIX_FMT_MJPEG;
		}
		else
		{
			printf("Fail, not supported codec!!!\n");
			goto ENC_TERMINATE;
		}

		iFormat = NX_V4l2GetFileFormat( pAppData->inFileName );
		if( iFormat )
		{
			iPlaneNum = NX_V4l2GetPlaneNum( iFormat );
		}
		else
		{
			iFormat   = IMG_FORMAT;
			iPlaneNum = IMG_PLANE_NUM;
		}

		printf("-. inFile    = %s\n", pAppData->inFileName );
		printf("-. iFormat   = %s( 0x%08X )\n", NX_V4l2GetFormatString(iFormat), iFormat );
		printf("-. iPlaneNum = %d\n", iPlaneNum );

		//
		// Allocate Input Buffer
		//
		for (i = 0; i < IMAGE_BUFFER_NUM; i++)
		{
			hImage[i] = NX_AllocateVideoMemory(inWidth, inHeight, iPlaneNum, iFormat, 4096);

			if (hImage[i] == NULL)
			{
				printf("Failed to allocate %d image buffer\n", i);
				goto ENC_TERMINATE;
			}

			if (NX_MapVideoMemory(hImage[i]) != 0)
			{
				printf("Video Memory Mapping Failed\n");
				goto ENC_TERMINATE;
			}
		}

		hEnc = NX_V4l2EncOpen(pAppData->codec);
		if (hEnc == NULL)
		{
			printf("video encoder open is failed!!!");
			ret = -1;
			goto ENC_TERMINATE;
		}

		memset(&encPara, 0, sizeof(encPara));
		encPara.width = inWidth;
		encPara.height = inHeight;
		encPara.fpsNum = (pAppData->fpsNum) ? (pAppData->fpsNum) : (30);
		encPara.fpsDen = (pAppData->fpsDen) ? (pAppData->fpsDen) : (1);
		encPara.keyFrmInterval = (pAppData->gop) ? (pAppData->gop) : (encPara.fpsNum / encPara.fpsDen);
		encPara.bitrate = pAppData->kbitrate * 1024;
		encPara.maximumQp = (pAppData->maxQp) ? (pAppData->maxQp) : (0);
		encPara.rcVbvSize = (pAppData->vbv) ? (pAppData->vbv) : (0);
		encPara.disableSkip = 0;
		encPara.RCDelay = 0;
		encPara.gammaFactor = 0;
		encPara.initialQp = pAppData->qp;
		encPara.numIntraRefreshMbs = 0;
		encPara.searchRange = 0;
		encPara.enableAUDelimiter = 0;
		encPara.imgFormat = iFormat;
		encPara.imgBufferNum = IMAGE_BUFFER_NUM;
		encPara.imgPlaneNum = iPlaneNum;
		encPara.pImage = hImage[0];

		if (pAppData->codec == V4L2_PIX_FMT_MJPEG)
			encPara.jpgQuality = (pAppData->qp == 0) ? (90) : (pAppData->qp);

		ret = NX_V4l2EncInit(hEnc, &encPara);
		if (ret < 0)
		{
			printf("video encoder initialization is failed!!!\n");
			goto ENC_TERMINATE;
		}

		ret = NX_V4l2EncGetSeqInfo(hEnc, &pSeqData, &seqSize);
		if (ret < 0)
		{
			printf("Getting Sequence header is failed!!!\n");
			goto ENC_TERMINATE;
		}

		// Write Sequence Data
		if (seqSize > 0)
			fwrite((void *)pSeqData, 1, seqSize, fpOut);
	}

	//==============================================================================
	// PROCESS UNIT
	//==============================================================================
	{
		int32_t frmCnt = 0, readSize;
		uint64_t startTime, endTime;
		int32_t imgSize;
		uint8_t *pSrcBuf;

		NX_V4l2GetImageInfo( iFormat, inWidth, inHeight, &imgSize);

		pSrcBuf = (uint8_t *)malloc(imgSize);
		while (IsRunningLoop())
		{
			NX_V4L2ENC_IN encIn;
			NX_V4L2ENC_OUT encOut;
			int32_t index = frmCnt % IMAGE_BUFFER_NUM;

			if (fpIn)
			{
				readSize = fread(pSrcBuf, 1, imgSize, fpIn);
				if (readSize != imgSize)
				{
					printf("End of Image\n");
					break;
				}

				NX_V4l2LoadMemory(pSrcBuf, inWidth, inHeight, hImage[index]);
			}

#if ENABLE_DRM_DISPLAY
			UpdateBuffer(hDsp, hImage[index], NULL);
#endif

			memset(&encIn, 0, sizeof(NX_V4L2ENC_IN));
			memset(&encOut, 0, sizeof(NX_V4L2ENC_OUT));

			encIn.pImage = hImage[index];
			encIn.imgIndex = index;
			encIn.forcedIFrame = 0;
			encIn.forcedSkipFrame = 0;
			encIn.quantParam = pAppData->qp;

			startTime = NX_GetTickCount();
			ret = NX_V4l2EncEncodeFrame(hEnc, &encIn, &encOut);
			endTime = NX_GetTickCount();

			if (ret < 0)
			{
				printf("[%d] Frame]NX_V4l2EncEncodeFrame() is failed!!\n", frmCnt);
				break;
			}

			printf("[%04d Frm]Size = %5d, Type = %1d, Time = %6llu\n", frmCnt, encOut.strmSize, encOut.frameType, (endTime - startTime));

			if (fpOut && encOut.strmSize > 0)
				fwrite((void *)encOut.strmBuf, 1, encOut.strmSize, fpOut);

			frmCnt++;
		}

		if (pSrcBuf)
			free(pSrcBuf);
	}

	//==============================================================================
	// TERMINATION
	//==============================================================================
ENC_TERMINATE:
	if (hEnc)
		NX_V4l2EncClose(hEnc);

	for (i = 0; i < IMAGE_BUFFER_NUM; i++)
			NX_FreeVideoMemory(hImage[i]);

	if (fpIn)
		fclose(fpIn);

	if (fpOut)
		fclose(fpOut);

	printf("Encode End!!\n" );

	return ret;
}

int32_t VpuEncMain(CODEC_APP_DATA *pAppData)
{
	if (pAppData->inFileName)
		return VpuEncPerfMain(pAppData);
	else
		return VpuCamEncMain(pAppData);
}
