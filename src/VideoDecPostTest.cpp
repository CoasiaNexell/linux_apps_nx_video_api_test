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
#define _FILE_OFFSET_BITS	64

#include <stdio.h>

#include <sys/types.h>	//	open
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>

#include <unistd.h>

#include <linux/videodev2.h>

#include <nx_video_alloc.h>
#include <nx_video_api.h>

#include "MediaExtractor.h"
#include "CodecInfo.h"
#include "Util.h"
#include "NX_V4l2Utils.h"

#include <videodev2_nxp_media.h>

#define ENABLE_DRM_DISPLAY			(1)
#define SCREEN_WIDTH				(1024)
#define SCREEN_HEIGHT				(600)

#include <drm/drm_fourcc.h>
#if ENABLE_DRM_DISPLAY
#include "DrmRender.h"
#endif

//#define NX_IMAGE_FORMAT					V4L2_PIX_FMT_YUV420	// V4L2_PIX_FMT_YVU420
#define NX_ADDITIONAL_BUFFER			3

#define NX_ENABLE_FRAME_INFO			false

#define NX_TEST_FLUSH_API				true	// use flush api

// #define PLANE_ID		33
// #define CRTC_ID			32

#include <nx_gl_tools.h>
enum {
	POST_PROC_DEINT,
	POST_PROC_COPY
};

enum {
	DEINT_MODE_ABLEND,			//	Advanced Blending Mode for 30 fsp
	DEINT_MODE_MOTION,			//	Motion Adaptation Mode for 60fps
};

#define	POST_OUT_BUF_CNT		(4)

typedef struct _POST_PROC_DATA {
	void				*handle;
	uint32_t			mode;			//	0 : Deinterlace
	uint32_t			deintMode;		//	0 : Advanced Blending, 1 : Motion Adaptation
	uint32_t			srcWidth;
	uint32_t			srcHeight;
	uint32_t			dstWidth;
	uint32_t			dstHeight;
	NX_MEMORY_HANDLE 	hMotionMem[2];
} POST_PROC_DATA, *POSTPROC_HANDLE;

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

//
//	mode : POST Processing Mode
//			POST_PROC_DEINT
//
POSTPROC_HANDLE InitPostProcessing( uint32_t mode, uint32_t deintMode, uint32_t srcWidth, uint32_t srcHeight,
						 uint32_t dstWidth, uint32_t dstHeight, 
						 int (*pDstDmaFd)[3], int srcImageFormat, int32_t dstOutBufNum, float coeff)
{
	POSTPROC_HANDLE hPost = (POSTPROC_HANDLE)malloc(sizeof(POST_PROC_DATA));
	memset( hPost, 0, sizeof(POST_PROC_DATA) );
	if( mode == POST_PROC_DEINT )		//	Deinterlace
	{
		int motionFds[2] = {0, };
		if( deintMode == DEINT_MODE_MOTION )
		{
			hPost->hMotionMem[0] = NX_AllocateMemory( srcWidth*srcHeight, 16 );
			hPost->hMotionMem[1] = NX_AllocateMemory( srcWidth*srcHeight, 16 );
			if( NULL == hPost->hMotionMem[0] || NULL == hPost->hMotionMem[1] )
			{
				if( hPost->hMotionMem[0] )
					NX_FreeMemory(hPost->hMotionMem[0]);
				free(hPost);
				return NULL;
			}

			motionFds[0] = hPost->hMotionMem[0]->dmaFd;
			motionFds[1] = hPost->hMotionMem[1]->dmaFd;

			NX_V4l2ClearMemory(hPost->hMotionMem[0]);
			NX_V4l2ClearMemory(hPost->hMotionMem[1]);

			hPost->handle = NX_GlDeinterlaceInit(srcWidth, srcHeight, dstWidth, dstHeight, pDstDmaFd, srcImageFormat, dstOutBufNum, DEINT_MODE_ADAPTIVE, motionFds, coeff);
		}
		else if( deintMode == DEINT_MODE_ABLEND )
		{
			hPost->handle = NX_GlDeinterlaceInit(srcWidth, srcHeight, dstWidth, dstHeight, pDstDmaFd, srcImageFormat, dstOutBufNum, DEINT_MODE_MIXING, NULL, coeff);
		}
		else
		{
			printf("Unknown deintMode = %d\n", deintMode);
		}

		if( !hPost->handle )
		{
			hPost->mode = POST_PROC_DEINT;
			free(hPost);
			return NULL;
		}
		hPost->mode = POST_PROC_DEINT;
		hPost->srcWidth = srcWidth;
		hPost->srcHeight = srcHeight;
		hPost->dstWidth = dstWidth;
		hPost->dstHeight = dstHeight;
		hPost->deintMode = deintMode;
	}
	else
	{
		return NULL;
	}
	return hPost;
}

//	only deinterlace
int PostProcessingMotion(POSTPROC_HANDLE hPost, int *pSrcDmaFds, int *pSrcNDmaFds)
{
	return NX_GlDeinterlaceMotion(hPost->handle, pSrcDmaFds, pSrcNDmaFds );
}

int PostProcessing(POSTPROC_HANDLE hPost, int *pSrcDmaFds, int *pSrcNDmaFds, int *pDstDmaFds)
{
	if( hPost->mode == POST_PROC_DEINT )
	{
		return NX_GlDeinterlaceRun(hPost->handle, pSrcDmaFds, pSrcNDmaFds, pDstDmaFds);
	}
	return -1;
}

void DestroyPostProcessing(POSTPROC_HANDLE hPost)
{
	if( hPost )
	{
		if( hPost->handle )
		{
			if( hPost->mode == POST_PROC_DEINT )
			{
				NX_GlDeinterlaceDeInit(hPost->handle);
				if( hPost->hMotionMem[0] )
					NX_FreeMemory(hPost->hMotionMem[0]);
				if( hPost->hMotionMem[1] )
					NX_FreeMemory(hPost->hMotionMem[1]);
			}
		}
		free( hPost );
	}
}


void UpdateMinMaxTime( uint64_t &minTime, uint64_t &maxTime, uint64_t time )
{
	//	Update Min
	if( time < minTime )
		minTime = time;
	if( time > maxTime )
		maxTime = time;
}

//------------------------------------------------------------------------------
//	Video Decoder Postprocessing Main
//
//	: Decoding -> PostPorcessing (Linux Only)
//
int32_t VpuDecPostMain ( CODEC_APP_DATA *pAppData )
{
	NX_V4L2DEC_HANDLE hDec = NULL;
	int32_t nxImageFormat =	V4L2_PIX_FMT_YUV420;
	int32_t nxDrmFormat = DRM_FORMAT_YUV420;
#if ENABLE_DRM_DISPLAY
	DRM_DSP_HANDLE hDsp = NULL;
#endif
	uint8_t streamBuffer[4*1024*1024];
	int32_t ret;
	int32_t imgWidth = -1, imgHeight = -1;
	int drmFd = 0;

	//	Post processing Output Memory
	POSTPROC_HANDLE hPost = NULL;
	NX_VID_MEMORY_HANDLE hPostOutVidMem[POST_OUT_BUF_CNT] = { NULL, };
	uint32_t postOutBufIdx = 0;
	int outDmaFd[POST_OUT_BUF_CNT][3];

	uint32_t deintMode = pAppData->deintMode;

	if( !IsRunningLoop() )
		return -1;

	CMediaReader *pMediaReader = new CMediaReader();
	if (!pMediaReader->OpenFile( pAppData->inFileName))
	{
		printf("Cannot open media file(%s)\n", pAppData->inFileName);
		exit(-1);
	}
	pMediaReader->GetVideoResolution(&imgWidth, &imgHeight);

	register_signal();

	//==============================================================================
	// DISPLAY INITIALIZATION
	//==============================================================================
	if( NX_IsCpuNXP322X() )
	{
		nxImageFormat =	V4L2_PIX_FMT_NV12;
		nxDrmFormat = DRM_FORMAT_NV12;
	}

	{
#if ENABLE_DRM_DISPLAY
		drmFd = open("/dev/dri/card0", O_RDWR);

		hDsp = CreateDrmDisplay(drmFd);
		DRM_RECT srcRect, dstRect;

		srcRect.x = 0;
		srcRect.y = 0;
		srcRect.width = imgWidth;
		srcRect.height = imgHeight;
		dstRect.x = 0;
		dstRect.y = 0;
		dstRect.width = imgWidth;
		dstRect.height = imgHeight;

		//Drm info
		MP_DRM_PLANE_INFO drmDisplayInfo;
		int32_t crtcIdx;
		int32_t layerIdx;
		int32_t findRgb;
		drmDisplayInfo.iConnectorID = -1;
		drmDisplayInfo.iCrtcId		= -1;
		drmDisplayInfo.iPlaneId		= -1;

		crtcIdx  = 0;
		findRgb  = 0;		
		layerIdx = 1;
		if( 0 > NX_FindPlaneForDisplay(crtcIdx, findRgb, layerIdx, &drmDisplayInfo) )
		{
			printf( "cannot found video format for %dth crtc\n", crtcIdx );
		}

		if( pAppData->iDisplayMode == 0 )	//	aspect ratio
		{
			double xRatio = (double)SCREEN_WIDTH/(double)imgWidth;
			double yRatio = (double)SCREEN_HEIGHT/(double)imgHeight;

			if (xRatio > yRatio)
			{
				dstRect.width = imgWidth * yRatio;
				dstRect.height = SCREEN_HEIGHT;
				dstRect.x = abs(SCREEN_WIDTH - dstRect.width)/2;
			}
			else
			{
				dstRect.width = SCREEN_WIDTH;
				dstRect.height = imgHeight * xRatio;
				dstRect.y = abs(SCREEN_HEIGHT - dstRect.height)/2;
			}
		}
		else if( pAppData->iDisplayMode == 1 )	//	no scaling
		{
			dstRect.x = 0;
			dstRect.y = 0;
			dstRect.width = imgWidth;
			dstRect.height = imgHeight;
		}
		else if( pAppData->iDisplayMode == 2 )	//	scale to screen size
		{
			dstRect.x = 0;
			dstRect.y = 0;
			dstRect.width = SCREEN_WIDTH;
			dstRect.height = SCREEN_HEIGHT;
		}

		//	Init Display
		InitDrmDisplay(hDsp, drmDisplayInfo.iPlaneId, drmDisplayInfo.iCrtcId, nxDrmFormat, srcRect, dstRect, 1);

		//	Set Video Layer to 0
		if( pAppData->iDisplayPriority != -1 )
		{
			DspVideoSetPriority( hDsp, pAppData->iDisplayPriority );
		}
#endif	//	ENABLE_DRM_DISPLAY
	}

	uint32_t v4l2CodecType;
	int32_t fourcc = -1, codecId = -1;

	pMediaReader->GetCodecTagId(AVMEDIA_TYPE_VIDEO, &fourcc, &codecId);
	v4l2CodecType = CodecIdToV4l2Type(codecId, fourcc);

	hDec = NX_V4l2DecOpen(v4l2CodecType);
	if (hDec == NULL)
	{
		printf("Fail, NX_V4l2DecOpen().\n");
		return -1;
	}

	printf(">>> File Info: %s ( format: 0x%08X, %s )\n",
		pAppData->inFileName, v4l2CodecType, NX_V4l2GetFormatString(v4l2CodecType));

	//==============================================================================
	// PROCESS UNIT
	//==============================================================================
	{
		int32_t bInit = false, bSeek = false;

		int32_t size = 0;
		uint32_t frmCnt = 0, outFrmCnt = 0;
		uint64_t startTime, endTime, totalTime = 0;
		int64_t timeStamp = -1;

		uint64_t postSTime, postETime, postTTime = 0;
		uint64_t postMinTime=0, postMaxTime=0;
		uint32_t postProcCnt = 0;

		FILE *fpOut = NULL;
		int32_t prvIndex = -1, curIndex = -1;
		NX_VID_MEMORY_HANDLE hCurImg = NULL;

		int32_t additionSize = 0;
		int32_t inFlushFrameCount = 0;

		NX_V4L2DEC_SEQ_IN seqIn;
		NX_V4L2DEC_SEQ_OUT seqOut;

		NX_V4L2DEC_IN decIn;
		NX_V4L2DEC_OUT decOut;

		int prevFds[4] = {0, };		//	Previous' Frame's FDS

		if (pAppData->outFileName)
		{
			fpOut = fopen64(pAppData->outFileName, "wb");
			if (fpOut == NULL)
			{
				printf("output file open error!!\n");
				ret = -1;
				goto DEC_TERMINATE;
			}
		}

		while(IsRunningLoop())
		{
			int32_t key = 0;

			if( !bInit )
			{
				additionSize = pMediaReader->GetVideoSeqInfo(streamBuffer);
			}

			if (pMediaReader->ReadStream(CMediaReader::MEDIA_TYPE_VIDEO, streamBuffer + additionSize, &size, &key, &timeStamp ) != 0)
			{
				size = 0;
				break;
			}

			if( !bInit && !key )
				continue;

			if( bSeek && !key && (inFlushFrameCount != 1) )
				continue;

			if( !bInit )
			{
				memset(&seqIn, 0, sizeof(seqIn));
				seqIn.width     = imgWidth;
				seqIn.height    = imgHeight;
				seqIn.seqSize   = size + additionSize;
				seqIn.seqBuf    = streamBuffer;
				seqIn.timeStamp = timeStamp;

				if (v4l2CodecType == V4L2_PIX_FMT_MJPEG)
					seqIn.thumbnailMode = 0;

				ret = NX_V4l2DecParseVideoCfg(hDec, &seqIn, &seqOut);
				if (ret < 0)
				{
					printf("Fail, NX_V4l2DecParseVideoCfg()\n");
					goto DEC_TERMINATE;
				}

				seqIn.width       = seqOut.width;
				seqIn.height      = seqOut.height;
				seqIn.imgPlaneNum = NX_V4l2GetPlaneNum(nxImageFormat);
				seqIn.imgFormat   = nxImageFormat;
				seqIn.numBuffers  = seqOut.minBuffers + NX_ADDITIONAL_BUFFER;

				printf("[Sequence Data] width( %d ), height( %d ), plane( %d ), format( 0x%08x ), reqiured buffer( %d ), current buffer( %d )\n",
					seqIn.width, seqIn.height, seqIn.imgPlaneNum, seqIn.imgFormat, seqIn.numBuffers - NX_ADDITIONAL_BUFFER, seqIn.numBuffers );

				ret = NX_V4l2DecInit(hDec, &seqIn);
				if (ret < 0)
				{
					printf("Fail, NX_V4l2DecInit().\n");
					goto DEC_TERMINATE;
				}

				bInit = true;
				additionSize = 0;

				//==============================================================================
				// Deinterlace Initialization
				//==============================================================================
				for(int32_t i = 0; i < POST_OUT_BUF_CNT; i++)
				{
					hPostOutVidMem[i] = NX_AllocateVideoMemory(seqOut.width, seqOut.height, 3, nxImageFormat, 4096);
					if(hPostOutVidMem[i] == NULL)
					{
						printf( "Deinterlace:Failed to allocate Outimage buffer\n");
						goto DEC_TERMINATE;
					}
					outDmaFd[i][0] = hPostOutVidMem[i]->dmaFd[0];
					outDmaFd[i][1] = hPostOutVidMem[i]->dmaFd[1];
					outDmaFd[i][2] = hPostOutVidMem[i]->dmaFd[2];
				}
				hPost = InitPostProcessing(0, deintMode, seqOut.width, seqOut.height, seqOut.width, seqOut.height, outDmaFd, nxImageFormat, POST_OUT_BUF_CNT, pAppData->coeff);
				if(hPost == NULL)
				{
					printf( "InitPostProcessing(): Fail pGlHandle is NULL !!\n");
					goto DEC_TERMINATE;
				}

				continue;
			}

			/*
			Skip Bitstream
			*/
			if(	(V4L2_PIX_FMT_XVID == v4l2CodecType && 0 < size &&   7 >= size) ||
				(V4L2_PIX_FMT_DIV5 == v4l2CodecType && 0 < size &&   8 >= size) )
			{
				// NX_DumpData( streamBuffer, (size < 16) ? size : 16, "[     Skip] " );
				continue;
			}

			if( bSeek )
			{
				inFlushFrameCount++;
				additionSize += size;				
				if(inFlushFrameCount == 2)
				{
					bSeek = false;
					inFlushFrameCount = 0;
				}
				continue;
			}

			do
			{
				memset(&decIn, 0, sizeof(NX_V4L2DEC_IN));
				decIn.strmBuf   = (size > 0) ? streamBuffer : NULL;
				decIn.strmSize  = (size > 0) ? size + additionSize : 0;
				decIn.timeStamp = (size > 0) ? timeStamp : 0;
				decIn.eos       = (size > 0) ? 0 : 1;

				startTime       = NX_GetTickCount();
				ret             = NX_V4l2DecDecodeFrame(hDec, &decIn, &decOut);
				endTime         = NX_GetTickCount();
				totalTime       += (endTime - startTime);

				additionSize = 0;

				if (ret < 0)
				{
					printf("Fail, NX_V4l2DecDecodeFrame().\n");
					break;
				}

#if NX_ENABLE_FRAME_INFO
				printf("[%5d Frm] Key=%d, Size=%6d, DecIdx=%2d, DispIdx=%2d, InTimeStamp=%7lld, outTimeStamp=%7lld, Time=%6llu, interlace=%1d %1d, Reliable=%3d, %3d, type =%3d, %3d, UsedByte=%6d, RemainByte=%6d\n",
					frmCnt, key, decIn.strmSize, decOut.decIdx, decOut.dispIdx, timeStamp, decOut.timeStamp[DISPLAY_FRAME], (endTime - startTime), decOut.interlace[DECODED_FRAME], decOut.interlace[DISPLAY_FRAME],
					decOut.outFrmReliable_0_100[DECODED_FRAME], decOut.outFrmReliable_0_100[DISPLAY_FRAME], decOut.picType[DECODED_FRAME], decOut.picType[DISPLAY_FRAME], decOut.usedByte, decOut.remainByte);
				/*printf("%2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x\n",
					streamBuffer[0], streamBuffer[1], streamBuffer[2], streamBuffer[3], streamBuffer[4], streamBuffer[5], streamBuffer[6], streamBuffer[7],
					streamBuffer[8], streamBuffer[9], streamBuffer[10], streamBuffer[11], streamBuffer[12], streamBuffer[13], streamBuffer[14], streamBuffer[15]);*/

				// NX_DumpData( streamBuffer, (size < 16) ? size : 16, "[%5d Frm] ", frmCnt );
#endif

				curIndex = decOut.dispIdx;
				hCurImg  = &decOut.hImg;

				if (curIndex >= 0)
				{
					if( prvIndex >= 0 )
					{
						//
						//	Deinterlace Motion Adaptive Mode
						//
						if( deintMode == DEINT_MODE_ADAPTIVE )
						{
							PostProcessingMotion( hPost, prevFds, decOut.hImg.dmaFd );

							//	Deinterlace : Previous Even + Previous Odd
							postSTime = NX_GetTickCount();
							ret = PostProcessing( hPost, prevFds, NULL, hPostOutVidMem[postOutBufIdx]->dmaFd );
							postETime = NX_GetTickCount();
							postTTime += (postETime - postSTime);
							if( postProcCnt == 0 )
							{
								postMinTime = postMaxTime = postETime - postSTime;
							}
							UpdateMinMaxTime( postMinTime, postMaxTime, postETime-postSTime );

							postProcCnt ++;
							if (ret < 0)
							{
								printf("Fail, PostProcessing(). ret = %d\n", ret );
								break;
							}

							if (fpOut)
							{
								NX_V4l2DumpMemory(hPostOutVidMem[postOutBufIdx], fpOut);
							}

#if ENABLE_DRM_DISPLAY
							UpdateBuffer(hDsp, hPostOutVidMem[postOutBufIdx++], NULL);
							postOutBufIdx %= POST_OUT_BUF_CNT;
#endif	//	ENABLE_DRM_DISPLAY

							//	Deinterlace : Previous Odd + Current Even
							postSTime = NX_GetTickCount();
							ret = PostProcessing( hPost, prevFds, decOut.hImg.dmaFd, hPostOutVidMem[postOutBufIdx]->dmaFd );
							postETime = NX_GetTickCount();
							postTTime += (postETime - postSTime);
							UpdateMinMaxTime( postMinTime, postMaxTime, postETime-postSTime );
							postProcCnt ++;
							if (ret < 0)
							{
								printf("Fail, PostProcessing(). ret = %d\n", ret );
								break;
							}
							if (fpOut)
							{
								NX_V4l2DumpMemory(hPostOutVidMem[postOutBufIdx], fpOut);
							}
#if ENABLE_DRM_DISPLAY
							UpdateBuffer(hDsp, hPostOutVidMem[postOutBufIdx++], NULL);
							postOutBufIdx %= POST_OUT_BUF_CNT;
#endif	//	ENABLE_DRM_DISPLAY
						}
						//
						//	Deinterlace Advanced Blending Mode
						//
						else
						{
							//	Deinterlace
							postSTime = NX_GetTickCount();
							ret = PostProcessing( hPost, decOut.hImg.dmaFd, NULL, hPostOutVidMem[postOutBufIdx]->dmaFd );
							postETime = NX_GetTickCount();
							postTTime += (postETime - postSTime);
							UpdateMinMaxTime( postMinTime, postMaxTime, postETime-postSTime );
							postProcCnt ++;
							if (ret < 0)
							{
								printf("Fail, PostProcessing(). ret = %d\n", ret );
								break;
							}
							if (fpOut)
							{
								NX_V4l2DumpMemory(hPostOutVidMem[postOutBufIdx], fpOut);
							}
#if ENABLE_DRM_DISPLAY
							UpdateBuffer(hDsp, hPostOutVidMem[postOutBufIdx++], NULL);
							postOutBufIdx %= POST_OUT_BUF_CNT;
#endif	//	ENABLE_DRM_DISPLAY
						}

					}	//	prvIndex >= 0

					if( pAppData->dumpFileName && outFrmCnt==pAppData->dumpFrameNumber )
					{
						printf("Dump Frame. ( frm: %d, name: %s )\n", outFrmCnt, pAppData->dumpFileName);
						NX_V4l2DumpMemory(hCurImg, (const char*)pAppData->dumpFileName );
					}

					if( prvIndex >= 0 )
					{
						ret = NX_V4l2DecClrDspFlag(hDec, NULL, prvIndex);
						if (0 > ret)
						{
							printf("Fail, NX_V4l2DecClrDspFlag().\n");
							break;
						}
					}

					prvIndex = curIndex;
					for( int ii=0 ; ii<4 ; ii++ )
						prevFds[ii] = decOut.hImg.dmaFd[ii];
					outFrmCnt++;
				}	//	curIndex >= 0

				frmCnt++;
				additionSize = 0;

				if (pAppData->iMaxLimitFrame != 0 && pAppData->iMaxLimitFrame <= frmCnt)
				{
					printf("Force Break by User.\n");
					ret = -1;
					break;
				}
			} while( size == 0 && decOut.dispIdx >= 0 && !ret );

			if( 0 > ret ) break;
		}

		if( prvIndex >= 0 )
		{
			NX_V4l2DecClrDspFlag(hDec, NULL, prvIndex);
			prvIndex = -1;
		}

		if (fpOut)
			fclose(fpOut);

		printf(" Post Porcessing Result (deintMode = %s) :\n", (deintMode==1)?"Motion Adaptive Mode":"Advanced Blending Mode");
		printf("     outFrameCnt  = %d\n", outFrmCnt);
		printf("     postProcCnt  = %d\n", postProcCnt);
		printf("     Total Time   = %lld\n", postTTime);
		printf("     Min Time     = %lld\n", postMinTime);
		printf("     Max Time     = %lld\n", postMaxTime);
		double avgTime = (double)postTTime/(double)postProcCnt;
		printf("     Average Time = %.2f(%.2ffps)\n", avgTime, 1000./avgTime);

	}

	//==============================================================================
	// TERMINATION
	//==============================================================================
DEC_TERMINATE:
	if (hDec)
		ret = NX_V4l2DecClose(hDec);

#if ENABLE_DRM_DISPLAY
	if(hDsp != NULL)
	{
		DestroyDrmDisplay(hDsp);
		close(drmFd);
	}
#endif // ENABLE_DRM_DISPLAY
	if (pMediaReader != NULL)
		delete pMediaReader;

	for(int32_t i = 0; i < POST_OUT_BUF_CNT; i++)		
	{
		if(hPostOutVidMem[i] != NULL)
		{
			NX_FreeVideoMemory(hPostOutVidMem[i]);
			hPostOutVidMem[i] = NULL;
		}
	}

	if (hPost)
	{
		DestroyPostProcessing(hPost);
	}

	printf("Decode End!!(ret = %d)\n", ret);
	return ret;
}
