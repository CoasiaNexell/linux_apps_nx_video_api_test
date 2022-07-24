AM_CFLAGS =

AM_CPPFLAGS = \
	$(WARN_CFLAGS)	\
	-I.				\
	-I${includedir}	\
	-I${includedir}/libdrm	\
	-I${top_builddir}/src/include

video_api_test_LDADD = \
	-lavcodec			\
	-lavformat			\
	-lavutil			\
	-lswresample		\
	-ldrm				\
	-lnx_video_api		\
	-lnx_drm_allocator	\
	-lnx_v4l2

AM_CFLAGS += -DTEST_POST_PROC
AM_CPPFLAGS += -DTEST_POST_PROC
video_api_test_LDADD += -lnx_gl_tools

video_api_test_SOURCES = \
	CodecInfo.cpp		\
	DrmRender.cpp		\
	MediaExtractor.cpp	\
	NX_CV4l2Camera.cpp	\
	NX_Queue.cpp		\
	NX_Semaphore.cpp	\
	Util.cpp			\
	NX_V4l2Utils.cpp	\
	VideoDecTest.cpp	\
	VideoEncTest.cpp	\
	main.cpp

video_api_test_SOURCES += VideoDecPostTest.cpp

bin_PROGRAMS = video_api_test
