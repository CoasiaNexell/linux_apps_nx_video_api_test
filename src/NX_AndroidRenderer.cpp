#include <gui/Surface.h>
#include <gui/SurfaceComposerClient.h>
#include <cutils/log.h>

#include <gralloc_priv.h>	//	private_handle_t

#include "NX_AndroidRenderer.h"

CNX_AndroidRenderer::CNX_AndroidRenderer( int32_t iWndWidth, int32_t iWndHeight )
	: m_WndWidth(0), m_WndHeight(0)
	, m_ImgWidth(0), m_ImgHeight(0)
	, m_bFirst(false)
{
	int32_t err;
	sp<SurfaceComposerClient> surfComClient = new SurfaceComposerClient();
	sp<SurfaceControl> yuvSurfaceControl =
		surfComClient->createSurface(String8("YUV Surface"), iWndWidth, iWndHeight, HAL_PIXEL_FORMAT_YV12, ISurfaceComposerClient::eFXSurfaceNormal);
	if (yuvSurfaceControl == NULL) {
		printf("failed to create yuv surface!!!");
		exit(-1);
	}
	SurfaceComposerClient::openGlobalTransaction();
	yuvSurfaceControl->show();
	yuvSurfaceControl->setLayer(99999);
	SurfaceComposerClient::closeGlobalTransaction();

	m_NumBuffers = 0;
	for( int32_t i=0 ; i<MAX_NUMBER_MEMORY ; i++ )
	{
		m_pYuvANBuffer[i] = NULL;
		m_MemoryHandles[i] = NULL;
	}

	m_SurfComClient = surfComClient;
	m_YuvSurfaceControl = yuvSurfaceControl;
}

CNX_AndroidRenderer::~CNX_AndroidRenderer()
{
	for( int32_t i=0 ; i<MAX_NUMBER_MEMORY ; i++ )
	{
		if( m_MemoryHandles[i] )
			free(m_MemoryHandles[i]);
	}
}

int32_t CNX_AndroidRenderer::GetBuffers( int32_t iNumBuf, int32_t iImgWidth, int32_t iImgHeight, NX_VID_MEMORY_HANDLE **pMemHandle )
{
	int32_t err;
	sp<ANativeWindow> yuvWindow(m_YuvSurfaceControl->getSurface());

	err = native_window_api_connect(yuvWindow.get(), NATIVE_WINDOW_API_CPU);
	if (err) {
		printf("failed to yuv native_window_api_connect!!!\n");
		return -1;
	}

	err = native_window_set_buffer_count(yuvWindow.get(), iNumBuf);
	if (err) {
		printf("failed to yuv native_window_set_buffer_count!!!\n");
		return -1;
	}
	err = native_window_set_scaling_mode(yuvWindow.get(), NATIVE_WINDOW_SCALING_MODE_SCALE_TO_WINDOW);
	if (err) {
		printf("failed to yuv native_window_set_scaling_mode!!!\n");
		return -1;
	}
	err = native_window_set_usage(yuvWindow.get(), GRALLOC_USAGE_HW_CAMERA_WRITE);
	if (err) {
		printf("failed to yuv native_window_set_usage!!!\n");
		return -1;
	}
#if 0
	//
	//	deprecated fucntion in Android Nougat.
	//
	err = native_window_set_buffers_geometry(yuvWindow.get(), iImgWidth, iImgHeight, HAL_PIXEL_FORMAT_YV12);
	if (err) {
		printf("failed to yuv native_window_set_buffers_geometry!!!\n");
		return -1;
	}
#else
	err = native_window_set_buffers_dimensions(yuvWindow.get(), iImgWidth, iImgHeight);
	if (err) {
		printf("failed to yuv native_window_set_buffers_size!!!\n");
		return -1;
	}

	err = native_window_set_buffers_format(yuvWindow.get(), HAL_PIXEL_FORMAT_YV12);
	if (err) {
		printf("failed to yuv native_window_set_buffers_format!!!\n");
		return -1;
	}
#endif
	for (int i = 0; i < iNumBuf; i++)
	{
		ANativeWindow *pAnw = NULL;
		pAnw = yuvWindow.get();
		int fenceFd = -1;
		err = native_window_dequeue_buffer_and_wait(yuvWindow.get(), &m_pYuvANBuffer[i]);
		if (err) {
			printf("failed to yuv dequeue buffer..\n");
			return -1;
		}

		private_handle_t const *yuvHandle = reinterpret_cast<private_handle_t const *>(m_pYuvANBuffer[i]->handle);
		m_MemoryHandles[i] = (NX_VID_MEMORY_HANDLE)calloc(1, sizeof(NX_VID_MEMORY_INFO));
		NX_PrivateHandleToVideoMemory( yuvHandle, m_MemoryHandles[i] );
	}
	*pMemHandle = &m_MemoryHandles[0];
	m_YuvWindow = yuvWindow;
	return 0;
}

int32_t CNX_AndroidRenderer::DspQueueBuffer( NX_VID_MEMORY_HANDLE hHandle, int32_t index )
{
	int ret = 0;
	(void) hHandle;
	ret = m_YuvWindow->queueBuffer(m_YuvWindow.get(), m_pYuvANBuffer[index], -1);
	if(ret)
	{
		printf("failed to yuv DspQueueBuffer buffer..\n");
	}
	return 0;
}

int32_t CNX_AndroidRenderer::DspDequeueBuffer( NX_VID_MEMORY_HANDLE *hHandle, int32_t *index )
{
	int ret = 0;
    ANativeWindowBuffer *yuvTempBuffer;
	(void) hHandle;
	(void) index;
	native_window_dequeue_buffer_and_wait(m_YuvWindow.get(), &yuvTempBuffer);
	if(ret)
	{
		printf("failed to yuv DspDequeueBuffer buffer..\n");
	}

	return 0;
}
