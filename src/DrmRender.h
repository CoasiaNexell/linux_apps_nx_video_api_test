#ifndef __DRMRENDER_H__
#define __DRMRENDER_H__

typedef struct DRM_DSP_INFO *DRM_DSP_HANDLE;

typedef struct _DRM_RECT{
	int32_t x;
	int32_t y;
	int32_t width;
	int32_t height;
} DRM_RECT;


DRM_DSP_HANDLE CreateDrmDisplay( int fd );
int32_t InitDrmDisplay( DRM_DSP_HANDLE hDsp, uint32_t planeId, uint32_t crtcId, uint32_t format,
						DRM_RECT srcRect, DRM_RECT dstRect, uint32_t nonBlock = 0 );
int32_t UpdateBuffer( DRM_DSP_HANDLE hDsp, NX_VID_MEMORY_INFO *pMem , NX_VID_MEMORY_INFO **pOldMem );
int32_t DspVideoSetPriority(DRM_DSP_HANDLE hDsp, int32_t priority);
void DestroyDrmDisplay( DRM_DSP_HANDLE hDsp );


#endif	//  __DRMRENDER_H__
