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

#include <stdio.h>		// printf
#include <unistd.h>		// getopt & optarg
#include <stdlib.h>		// atoi
#include <string.h>		// strdup
#include <dirent.h>
#include <pthread.h>
#include <sys/stat.h>

#include <Util.h>

//------------------------------------------------------------------------------
extern int32_t VpuDecMain(CODEC_APP_DATA *pAppData);
extern int32_t VpuEncMain(CODEC_APP_DATA *pAppData);

#ifndef ANDROID
//	Linux Only
extern int32_t VpuDecPostMain ( CODEC_APP_DATA *pAppData );
#endif

#define MAX_FILE_LIST		256
#define MAX_PATH_SIZE		1024

bool bExitLoop = false;

enum {
	MODE_NONE,
	DECODER_MODE,
	ENCODER_MODE,
	DECODER_POST_MODE,
	MODE_MAX
};

//------------------------------------------------------------------------------
void print_usage(const char *appName)
{
	printf(
		"Usage : %s [options] -i [input file], [M] = mandatory, [O] = Optional \n"
		"  common options :\n"
		"     -m [mode]                  [O]  : 1:decoder mode, 2:encoder mode, 3:decoder post mode (def:decoder mode)\n"
		"     -i [input file name]       [M]  : input media file name (When is camera encoder, the value set NULL\n"
		"     -o [output file name]      [O]  : output file name\n"
		"     -r [repeat count]          [O]  : application repeat num. ( 0: unlimited repeat )\n"
		"     -a [display mode]          [O]  : 0:aspect ratio, 1:org display, 2:dsp device (def:aspect ratio)\n"
		"     -p [priority]              [O]  : 0/1/2(primary only) (def:none)\n"
		"     -h : help\n"
		" -------------------------------------------------------------------------------------------------------------------\n"
		"  only decoder options :\n"
		"     -j [seek frame],[position] [O]  : seek start frame, seek potition(sec)\n"
		"     -d [frame num],[file name] [O]  : dump frame number, dump file name\n"
		"     -l [frame num]             [O]  : max limitation frame\n"
		" -------------------------------------------------------------------------------------------------------------------\n"
		"  only encoder options :\n"
		"     -c [codec]                 [O]  : 0:H.264, 1:Mp4v, 2:H.263, 3:JPEG (def:H.264)\n"
		"     -s [width],[height]        [M]  : input image's size\n"
		"     -f [fps Num],[fps Den]     [O]  : input image's framerate(def:30/1) \n"
		"     -b [Kbitrate]              [M]  : target Kilo bitrate (0:VBR mode, other:CBR mode)\n"
		"     -g [gop size]              [O]  : gop size (def:framerate) \n"
		"     -q [quality or QP]         [O]  : Jpeg Quality or Other codec Quantization Parameter(When is VBR, it is valid) \n"
		"     -v [VBV]                   [O]  : VBV Size (def:2Sec)\n"
		"     -x [Max Qp]                [O]  : Maximum Qp \n"
		" ===================================================================================================================\n\n"
		,appName);
	printf(
		"Examples\n");
	printf(
		" Decoder Mode :\n"
		"     #> %s -i [input filename]\n", appName);
	printf(
		" Decoder Mode & Capture :\n"
		"     #> %s -i [input filename] -d [num],[dump filename] \n", appName);
	printf(
		" Encoder Camera Mode :\n"
		"     #> %s -m 2 -o [output filename]\n", appName);
	printf(
		" Encoder File Mode :(H.264, 1920x1080, 10Mbps, 30fps, 30 gop)\n"
		"     #> %s -m 2 -i [input filename] -o [output filename] -s 1920,1080 -f 30,1 -b 10000 -g 30 \n", appName);
	printf(
		" Decoder Post Processing :\n"
		"     #> %s -m 3 -i [input filename]\n", appName);
}

//------------------------------------------------------------------------------
static int32_t IsRegularFile( char *pFile )
{
	struct stat statinfo;
	if( 0 > stat( pFile, &statinfo) )
		return 0;

	return S_ISREG( statinfo.st_mode );
}

//------------------------------------------------------------------------------
static int32_t IsDirectory( char *pFile )
{
	struct stat statinfo;
	if( 0 > stat( pFile, &statinfo) )
		return 0;

	return S_ISDIR( statinfo.st_mode );
}

//------------------------------------------------------------------------------
static int32_t IsVideo( char *pFile )
{
	const char *pVidExtension[] = {
		"avi",	"wmv",	"asf",	"mpg",	"mpeg",	"mpv2",	"mp2v",	"ts",	"tp",	"vob",
		"ogg",	"ogm",	"ogv",	"mp4",	"m4v",	"m4p",	"3gp",	"3gpp",	"mkv",	"rm",
		"rmvb",	"flv",	"m2ts",	"m2t",	"divx",	"webm",
	};

	char *pExtension = pFile + strlen(pFile) - 1;
	while( pFile != pExtension )
	{
		if( *pExtension == '.' ) {
			pExtension++;
			break;
		}
		pExtension--;
	}

	if( pFile != pExtension )
	{
		for( int32_t i = 0; i < (int32_t)(sizeof(pVidExtension) / sizeof(pVidExtension[0])); i++ )
		{
			if( !strcasecmp( pExtension, pVidExtension[i] ) )
				return true;
		}
	}

	return false;
}

//------------------------------------------------------------------------------
static int32_t MakeFileList( char *pDirectory, char *pFileList[MAX_FILE_LIST], int32_t *iFileNum )
{
	DIR *pDir = NULL;
	struct dirent *pDirent;

	if( NULL == (pDir = opendir( pDirectory )) )
	{
		return -1;
	}

	while( NULL != (pDirent = readdir(pDir)) )
	{
		if( !strcmp( pDirent->d_name, "." ) ||
			!strcmp( pDirent->d_name, ".." ) )
			continue;

		char szFile[MAX_PATH_SIZE];
		sprintf( szFile, "%s/%s", pDirectory, pDirent->d_name );

		if( IsDirectory(szFile) )
		{
			MakeFileList( szFile, pFileList, iFileNum );
		}
		else if( IsRegularFile(szFile) )
		{
			if (MAX_FILE_LIST <= *iFileNum)
			{
				printf("Warn, List Array Limitation. ( %s )\n", szFile);
				continue;
			}
			else
			{
				if( IsVideo(szFile) )
				{
					pFileList[*iFileNum] = strdup(szFile);
					*iFileNum = *iFileNum + 1;
				}
			}
		}
	}

	closedir( pDir );
	return 0;
}

//------------------------------------------------------------------------------
static void FreeFileList( char *pFileList[MAX_FILE_LIST], int32_t iFileNum )
{
	for( int32_t i = 0; i < iFileNum; i++ )
	{
		if (pFileList[i]) free( pFileList[i] );
	}
}

//------------------------------------------------------------------------------
int32_t main(int32_t argc, char *argv[])
{
	int32_t iRet = 0;
	int32_t opt;
	int32_t mode = DECODER_MODE;
	uint32_t iRepeat = 1, iCount = 0;
	// uint32_t iInstance = 1;
	char szTemp[1024];

	CODEC_APP_DATA appData;
	memset(&appData, 0, sizeof(CODEC_APP_DATA));
	appData.iDisplayPriority = -1;
	appData.coeff = 2.5;

	while (-1 != (opt = getopt(argc, argv, "m:i:o:hc:d:s:f:b:g:q:v:x:j:l:r:t:a:p:k:")))
	{
		switch (opt)
		{
		case 'm':
			mode = atoi(optarg);
#ifdef ANDROID
			if ((mode != DECODER_MODE) && (mode != ENCODER_MODE))
#else
			if ((mode != DECODER_MODE) && (mode != ENCODER_MODE) && (mode != DECODER_POST_MODE))
#endif
			{
				printf("Error : invalid mode ( %d:decoder mode, %d:encoder mode )!!!\n", DECODER_MODE, ENCODER_MODE);
				return -1;
			}
			break;
		case 'i':
			appData.inFileName  = ( IsRegularFile(optarg) && !IsDirectory(optarg)) ? strdup(optarg)         : NULL;
			appData.inDirectory = (!IsRegularFile(optarg) &&  IsDirectory(optarg)) ? realpath(optarg, NULL) : NULL;
			break;
		case 'o':	appData.outFileName = strdup(optarg);  break;
		case 'h':	print_usage(argv[0]);  return 0;
		case 'c':	appData.codec = atoi(optarg);  break;
		case 'd':	sscanf(optarg, "%d,%s", &appData.dumpFrameNumber, szTemp); appData.dumpFileName = strdup(szTemp); break;
		case 's':	sscanf(optarg, "%d,%d", &appData.width, &appData.height); break;
		case 'f':	sscanf( optarg, "%d,%d", &appData.fpsNum, &appData.fpsDen );  break;
		case 'b':	appData.kbitrate = atoi(optarg);  break;
		case 'g':	appData.gop = atoi(optarg);  break;
		case 'q':	appData.qp = atoi(optarg);  break;		/* JPEG Quality or Quantization Parameter */
		case 'v':	appData.vbv = atoi(optarg);  break;
		case 'x':	appData.maxQp = atoi(optarg);  break;
		case 'j':	sscanf(optarg, "%d,%d", &appData.iSeekStartFrame, &appData.iSeekPos);  break;
		case 'l':	appData.iMaxLimitFrame = atoi(optarg); printf(">>>>>>>>>>> Limit = %d\n", appData.iMaxLimitFrame); break;
		case 'r':	sscanf(optarg, "%u", &iRepeat);
		case 'a':	appData.iDisplayMode = atoi(optarg); break;
		case 'p':	appData.iDisplayPriority = atoi(optarg); break;
		case 'k':	appData.coeff = atof(optarg);  break;
		default:	break;
		}
	}
	do {
		switch (mode)
		{
		case DECODER_MODE:
			if( appData.inFileName )
			{
				iRet = VpuDecMain(&appData);
			}
			else if( appData.inDirectory )
			{
				char *pFileList[MAX_FILE_LIST];
				int32_t iFileNum = 0;
				MakeFileList( appData.inDirectory, pFileList, &iFileNum );
				for( int32_t i = 0; i < iFileNum; i++ )
				{
					appData.inFileName = pFileList[i];
					VpuDecMain(&appData);
				}
				FreeFileList( pFileList, iFileNum );
				appData.inFileName = NULL;
			}
			break;
		case DECODER_POST_MODE:
			iRet = VpuDecPostMain(&appData);
			break;
		case ENCODER_MODE:
#ifndef ENABLE_3220
			iRet = VpuEncMain(&appData);
#endif
			break;
		}
	} while(++iCount != iRepeat);

	if( appData.inFileName )	free( appData.inFileName );
	if( appData.inDirectory )	free( appData.inDirectory );
	if( appData.outFileName )	free( appData.outFileName );

	return iRet;
}
