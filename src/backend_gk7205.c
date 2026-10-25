#include "backend.h"
#include "venc_au.h"

#include "buffer.h"
#include "common.h"
#include "hi_comm_isp.h"
#include "hi_comm_sys.h"
#include "hi_comm_vb.h"
#include "hi_comm_venc.h"
#include "hi_comm_vi.h"
#include "hi_comm_vpss.h"
#include "hi_common.h"
#include "hi_defines.h"
#include "hi_mipi.h"
#include "mpi_ae.h"
#include "mpi_awb.h"
#include "mpi_isp.h"
#include "mpi_sys.h"
#include "mpi_vb.h"
#include "mpi_venc.h"
#include "mpi_vi.h"
#include "mpi_vpss.h"
#include "sns_ctrl.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define GK7205_VI_DEV 0
#define GK7205_VI_PIPE 0
#define GK7205_VI_CHN 0
#define GK7205_VPSS_GRP 0
#define GK7205_VPSS_CHN 0
#define GK7205_VENC_CHN 0
#define GK7205_RING_SLOTS 8u
#define GK7205_RING_BYTES (512u * 1024u)

extern ISP_SNS_OBJ_S stSnsImx307_2l_Obj;
extern VI_DEV_ATTR_S DEV_ATTR_IMX307_2M_BASE;
extern VI_PIPE_ATTR_S PIPE_ATTR_RAW10_420_3DNR_RFR;
extern VI_CHN_ATTR_S CHN_ATTR_420_SDR8_LINEAR;
extern combo_dev_attr_t MIPI_4lane_CHN0_SENSOR_IMX307_12BIT_2M_NOWDR_ATTR;
extern ISP_PUB_ATTR_S ISP_PROFILE_IMX307_MIPI_2M_30FPS;

extern int mipi_set_hs_mode(int device, lane_divide_mode_t mode);
extern int mipi_enable_clock(int device, uint32_t device_id, int enable);
extern int mipi_set_reset(int device, uint32_t device_id, int enable);
extern int mipi_enable_sensor_clock(int device, sns_clk_source_t sensor_id,
	int enable);
extern int mipi_set_sensor_reset(int device, sns_clk_source_t sensor_id,
	int enable);
extern int mipi_configure(int device, combo_dev_attr_t *config);

uint16_t goke_version = 200;
uint32_t sensor_framerate;

typedef struct {
	VencConfig cfg;
	VencOutputUri output_uri;
	venc_frame_ring_t *ring;
	ISP_SNS_COMMBUS_U sensor_bus;
	ALG_LIB_S ae_lib;
	ALG_LIB_S awb_lib;
	int mipi_fd;
	int venc_fd;
	pthread_t isp_thread;
	struct sigaction old_sigint;
	struct sigaction old_sigterm;
	unsigned int acquired;
	int isp_thread_started;
} Gk7205BackendContext;

enum {
	GK_HAVE_VB = 1u << 0,
	GK_HAVE_SYS = 1u << 1,
	GK_MIPI_CLOCK = 1u << 2,
	GK_SENSOR_CLOCK = 1u << 3,
	GK_VI_DEV = 1u << 4,
	GK_VI_PIPE = 1u << 5,
	GK_VI_PIPE_STARTED = 1u << 6,
	GK_VI_CHN = 1u << 7,
	GK_SENSOR_REGISTERED = 1u << 8,
	GK_AE_REGISTERED = 1u << 9,
	GK_AWB_REGISTERED = 1u << 10,
	GK_ISP = 1u << 11,
	GK_VPSS_GRP = 1u << 12,
	GK_VPSS_CHN = 1u << 13,
	GK_VPSS_STARTED = 1u << 14,
	GK_VI_BOUND = 1u << 15,
	GK_VENC_CHN_CREATED = 1u << 16,
	GK_VENC_RECV_STARTED = 1u << 17,
	GK_VENC_BOUND = 1u << 18,
	GK_VENC_FD = 1u << 19,
	GK_SIGNAL_HANDLERS = 1u << 20
};

static volatile sig_atomic_t gk7205_stop_requested;

static void gk7205_signal_handler(int signal_number)
{
	(void)signal_number;
	gk7205_stop_requested = 1;
}

static void *gk7205_isp_thread(void *opaque)
{
	VI_PIPE pipe = (VI_PIPE)(uintptr_t)opaque;
	HI_S32 result = HI_MPI_ISP_Run(pipe);
	return (void *)(intptr_t)result;
}

const char *venc_api_validate_loaded_config(const VencConfig *cfg)
{
	if (!cfg)
		return "invalid config state";
	if (cfg->video0.fps > 60)
		return "GK7205 IMX307 supports at most 60 fps";
	if ((cfg->video0.width != 0 || cfg->video0.height != 0) &&
	    (cfg->video0.width != 1280 || cfg->video0.height != 720))
		return "GK7205 IMX307 supports 1280x720";
	if (cfg->outgoing.server[0] &&
	    strncmp(cfg->outgoing.server, "frame-shm://", 12) != 0)
		return "GK7205 output must use frame-shm://";
	return NULL;
}

static int gk7205_check(const char *api, HI_S32 result)
{
	if (result == HI_SUCCESS)
		return 0;
	fprintf(stderr, "[gk7205] %s failed: 0x%x\n", api, (unsigned int)result);
	return -EIO;
}

static int gk7205_prepare_signals(Gk7205BackendContext *ctx)
{
	struct sigaction action;

	memset(&action, 0, sizeof(action));
	action.sa_handler = gk7205_signal_handler;
	sigemptyset(&action.sa_mask);
	if (sigaction(SIGINT, &action, &ctx->old_sigint) != 0)
		return -errno;
	if (sigaction(SIGTERM, &action, &ctx->old_sigterm) != 0) {
		int saved_errno = errno;
		(void)sigaction(SIGINT, &ctx->old_sigint, NULL);
		return -saved_errno;
	}
	ctx->acquired |= GK_SIGNAL_HANDLERS;
	gk7205_stop_requested = 0;
	return 0;
}

static int gk7205_start_mipi(Gk7205BackendContext *ctx,
	uint32_t width, uint32_t height)
{
	combo_dev_attr_t mipi_attr =
		MIPI_4lane_CHN0_SENSOR_IMX307_12BIT_2M_NOWDR_ATTR;
	int ret;

	ctx->mipi_fd = open(DEV_MIPI, O_RDWR);
	if (ctx->mipi_fd < 0)
		return -errno;
	ret = mipi_set_hs_mode(ctx->mipi_fd, LANE_DIVIDE_MODE_0);
	if (ret != 0)
		goto mipi_error;
	ret = mipi_enable_clock(ctx->mipi_fd, 0, 1);
	if (ret != 0)
		goto mipi_error;
	ctx->acquired |= GK_MIPI_CLOCK;
	ret = mipi_set_reset(ctx->mipi_fd, 0, 1);
	if (ret != 0)
		goto mipi_error;
	ret = mipi_enable_sensor_clock(ctx->mipi_fd, 0, 1);
	if (ret != 0)
		goto mipi_error;
	ctx->acquired |= GK_SENSOR_CLOCK;
	ret = mipi_set_sensor_reset(ctx->mipi_fd, 0, 1);
	if (ret != 0)
		goto mipi_error;
	mipi_attr.devno = 0;
	mipi_attr.img_rect.width = width;
	mipi_attr.img_rect.height = height;
	mipi_attr.mipi_attr.input_data_type = DATA_TYPE_RAW_10BIT;
	ret = mipi_configure(ctx->mipi_fd, &mipi_attr);
	if (ret != 0)
		goto mipi_error;
	ret = mipi_set_reset(ctx->mipi_fd, 0, 0);
	if (ret != 0)
		goto mipi_error;
	ret = mipi_set_sensor_reset(ctx->mipi_fd, 0, 0);
	if (ret != 0)
		goto mipi_error;
	if (close(ctx->mipi_fd) != 0) {
		ctx->mipi_fd = -1;
		return -errno;
	}
	ctx->mipi_fd = -1;
	return 0;

mipi_error:
	ret = ret < 0 ? ret : -EIO;
	fprintf(stderr, "[gk7205] IMX307 MIPI setup failed: %d\n", ret);
	return ret;
}

static int gk7205_create_venc(Gk7205BackendContext *ctx)
{
	VENC_CHN_ATTR_S attr;
	VENC_RECV_PIC_PARAM_S recv_param;
	uint32_t width = ctx->cfg.video0.width;
	uint32_t height = ctx->cfg.video0.height;
	uint32_t fps = ctx->cfg.video0.fps;
	uint32_t gop = (uint32_t)(ctx->cfg.video0.gop_size * fps + 0.5);
	uint32_t bitrate = ctx->cfg.video0.bitrate;
	HI_S32 result;

	if (gop == 0)
		gop = 1;
	if (bitrate < VENC_BITRATE_MIN_KBPS)
		bitrate = VENC_BITRATE_MIN_KBPS;
	if (bitrate > VENC_BITRATE_MAX_KBPS)
		bitrate = VENC_BITRATE_MAX_KBPS;
	memset(&attr, 0, sizeof(attr));
	attr.stVencAttr.enType = PT_H265;
	attr.stVencAttr.u32MaxPicWidth = width;
	attr.stVencAttr.u32MaxPicHeight = height;
	attr.stVencAttr.u32PicWidth = width;
	attr.stVencAttr.u32PicHeight = height;
	attr.stVencAttr.u32BufSize = (width * height * 3u / 4u + 63u) & ~63u;
	attr.stVencAttr.bByFrame = HI_TRUE;
	attr.stVencAttr.stAttrH265e.bRcnRefShareBuf = HI_TRUE;
	attr.stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
	attr.stRcAttr.stH265Cbr.u32SrcFrameRate = fps;
	attr.stRcAttr.stH265Cbr.fr32DstFrameRate = fps;
	attr.stRcAttr.stH265Cbr.u32StatTime = 1;
	attr.stRcAttr.stH265Cbr.u32Gop = gop;
	attr.stRcAttr.stH265Cbr.u32BitRate = bitrate;
	attr.stGopAttr.enGopMode = VENC_GOPMODE_NORMALP;
	attr.stGopAttr.stNormalP.s32IPQpDelta = 4;
	result = HI_MPI_VENC_CreateChn(GK7205_VENC_CHN, &attr);
	if (gk7205_check("HI_MPI_VENC_CreateChn", result) != 0)
		return -EIO;
	ctx->acquired |= GK_VENC_CHN_CREATED;

	ctx->venc_fd = HI_MPI_VENC_GetFd(GK7205_VENC_CHN);
	if (ctx->venc_fd < 0) {
		fprintf(stderr, "[gk7205] HI_MPI_VENC_GetFd failed: %d\n", ctx->venc_fd);
		return -EIO;
	}
	ctx->acquired |= GK_VENC_FD;

	recv_param.s32RecvPicNum = -1;
	result = HI_MPI_VENC_StartRecvFrame(GK7205_VENC_CHN, &recv_param);
	if (gk7205_check("HI_MPI_VENC_StartRecvFrame", result) != 0)
		return -EIO;
	ctx->acquired |= GK_VENC_RECV_STARTED;
	return 0;
}

static int gk7205_init_media(Gk7205BackendContext *ctx)
{
	VI_DEV_ATTR_S dev_attr = DEV_ATTR_IMX307_2M_BASE;
	VI_PIPE_ATTR_S pipe_attr = PIPE_ATTR_RAW10_420_3DNR_RFR;
	VI_CHN_ATTR_S vi_chn_attr = CHN_ATTR_420_SDR8_LINEAR;
	ISP_PUB_ATTR_S isp_attr = ISP_PROFILE_IMX307_MIPI_2M_30FPS;
	VI_DEV_BIND_PIPE_S bind_pipe;
	VI_VPSS_MODE_S vi_vpss_mode;
	VB_CONFIG_S vb_config;
	VPSS_GRP_ATTR_S vpss_group_attr;
	VPSS_CHN_ATTR_S vpss_chn_attr;
	MPP_CHN_S vi_source;
	MPP_CHN_S vpss_destination;
	MPP_CHN_S vpss_source;
	MPP_CHN_S venc_destination;
	uint32_t width = ctx->cfg.video0.width;
	uint32_t height = ctx->cfg.video0.height;
	uint32_t fps = ctx->cfg.video0.fps;
	HI_S32 result;

	memset(&vb_config, 0, sizeof(vb_config));
	vb_config.u32MaxPoolCnt = 2;
	vb_config.astCommPool[0].u32BlkCnt = 0;
	vb_config.astCommPool[0].u64BlkSize = VI_GetRawBufferSize(width, height,
		PIXEL_FORMAT_RGB_BAYER_10BPP, COMPRESS_MODE_NONE, DEFAULT_ALIGN);
	vb_config.astCommPool[1].u32BlkCnt = 2;
	vb_config.astCommPool[1].u64BlkSize = COMMON_GetPicBufferSize(width, height,
		PIXEL_FORMAT_YVU_SEMIPLANAR_420, DATA_BITWIDTH_8, COMPRESS_MODE_NONE,
		DEFAULT_ALIGN);
	result = HI_MPI_VB_SetConfig(&vb_config);
	if (gk7205_check("HI_MPI_VB_SetConfig", result) != 0)
		return -EIO;
	result = HI_MPI_VB_Init();
	if (gk7205_check("HI_MPI_VB_Init", result) != 0)
		return -EIO;
	ctx->acquired |= GK_HAVE_VB;
	result = HI_MPI_SYS_Init();
	if (gk7205_check("HI_MPI_SYS_Init", result) != 0)
		return -EIO;
	ctx->acquired |= GK_HAVE_SYS;
	result = HI_MPI_SYS_GetVIVPSSMode(&vi_vpss_mode);
	if (gk7205_check("HI_MPI_SYS_GetVIVPSSMode", result) != 0)
		return -EIO;
	vi_vpss_mode.aenMode[GK7205_VI_DEV] = VI_ONLINE_VPSS_ONLINE;
	result = HI_MPI_SYS_SetVIVPSSMode(&vi_vpss_mode);
	if (gk7205_check("HI_MPI_SYS_SetVIVPSSMode", result) != 0)
		return -EIO;
	if (gk7205_start_mipi(ctx, width, height) != 0)
		return -EIO;

	dev_attr.stSize.u32Width = width;
	dev_attr.stSize.u32Height = height;
	dev_attr.stBasAttr.stSacleAttr.stBasSize.u32Width = width;
	dev_attr.stBasAttr.stSacleAttr.stBasSize.u32Height = height;
	dev_attr.stSynCfg.stTimingBlank.u32HsyncAct = width;
	dev_attr.stSynCfg.stTimingBlank.u32VsyncVact = height;
	dev_attr.stWDRAttr.u32CacheLine = height;
	result = HI_MPI_VI_SetDevAttr(GK7205_VI_DEV, &dev_attr);
	if (gk7205_check("HI_MPI_VI_SetDevAttr", result) != 0)
		return -EIO;
	result = HI_MPI_VI_EnableDev(GK7205_VI_DEV);
	if (gk7205_check("HI_MPI_VI_EnableDev", result) != 0)
		return -EIO;
	ctx->acquired |= GK_VI_DEV;
	memset(&bind_pipe, 0, sizeof(bind_pipe));
	bind_pipe.u32Num = 1;
	bind_pipe.PipeId[0] = GK7205_VI_PIPE;
	result = HI_MPI_VI_SetDevBindPipe(GK7205_VI_DEV, &bind_pipe);
	if (gk7205_check("HI_MPI_VI_SetDevBindPipe", result) != 0)
		return -EIO;
	pipe_attr.bSharpenEn = HI_TRUE;
	pipe_attr.u32MaxW = width;
	pipe_attr.u32MaxH = height;
	result = HI_MPI_VI_CreatePipe(GK7205_VI_PIPE, &pipe_attr);
	if (gk7205_check("HI_MPI_VI_CreatePipe", result) != 0)
		return -EIO;
	ctx->acquired |= GK_VI_PIPE;
	result = HI_MPI_VI_StartPipe(GK7205_VI_PIPE);
	if (gk7205_check("HI_MPI_VI_StartPipe", result) != 0)
		return -EIO;
	ctx->acquired |= GK_VI_PIPE_STARTED;
	vi_chn_attr.stSize.u32Width = width;
	vi_chn_attr.stSize.u32Height = height;
	vi_chn_attr.stFrameRate.s32SrcFrameRate = fps;
	vi_chn_attr.stFrameRate.s32DstFrameRate = fps;
	result = HI_MPI_VI_SetChnAttr(GK7205_VI_PIPE, GK7205_VI_CHN,
		&vi_chn_attr);
	if (gk7205_check("HI_MPI_VI_SetChnAttr", result) != 0)
		return -EIO;
	result = HI_MPI_VI_EnableChn(GK7205_VI_PIPE, GK7205_VI_CHN);
	if (gk7205_check("HI_MPI_VI_EnableChn", result) != 0)
		return -EIO;
	ctx->acquired |= GK_VI_CHN;

	ctx->sensor_bus.s8I2cDev = 0;
	if (stSnsImx307_2l_Obj.pfnSetBusInfo(GK7205_VI_PIPE,
		ctx->sensor_bus) != HI_SUCCESS)
		return -EIO;
	memset(&ctx->ae_lib, 0, sizeof(ctx->ae_lib));
	memset(&ctx->awb_lib, 0, sizeof(ctx->awb_lib));
	ctx->ae_lib.s32Id = 0;
	ctx->awb_lib.s32Id = 0;
	(void)snprintf(ctx->ae_lib.acLibName, sizeof(ctx->ae_lib.acLibName),
		"%s", HI_AE_LIB_NAME);
	(void)snprintf(ctx->awb_lib.acLibName, sizeof(ctx->awb_lib.acLibName),
		"%s", HI_AWB_LIB_NAME);
	result = stSnsImx307_2l_Obj.pfnRegisterCallback(GK7205_VI_PIPE,
		&ctx->ae_lib, &ctx->awb_lib);
	if (gk7205_check("IMX307 register callback", result) != 0)
		return -EIO;
	ctx->acquired |= GK_SENSOR_REGISTERED;
	result = HI_MPI_AE_Register(GK7205_VI_PIPE, &ctx->ae_lib);
	if (gk7205_check("HI_MPI_AE_Register", result) != 0)
		return -EIO;
	ctx->acquired |= GK_AE_REGISTERED;
	result = HI_MPI_AWB_Register(GK7205_VI_PIPE, &ctx->awb_lib);
	if (gk7205_check("HI_MPI_AWB_Register", result) != 0)
		return -EIO;
	ctx->acquired |= GK_AWB_REGISTERED;
	result = HI_MPI_ISP_MemInit(GK7205_VI_PIPE);
	if (gk7205_check("HI_MPI_ISP_MemInit", result) != 0)
		return -EIO;
	isp_attr.f32FrameRate = fps;
	isp_attr.stSnsSize.u32Width = width;
	isp_attr.stSnsSize.u32Height = height;
	isp_attr.stWndRect.u32Width = width;
	isp_attr.stWndRect.u32Height = height;
	result = HI_MPI_ISP_SetPubAttr(GK7205_VI_PIPE, &isp_attr);
	if (gk7205_check("HI_MPI_ISP_SetPubAttr", result) != 0)
		return -EIO;
	result = HI_MPI_ISP_Init(GK7205_VI_PIPE);
	if (gk7205_check("HI_MPI_ISP_Init", result) != 0)
		return -EIO;
	ctx->acquired |= GK_ISP;
	if (pthread_create(&ctx->isp_thread, NULL,
		gk7205_isp_thread,
		(void *)(uintptr_t)GK7205_VI_PIPE) != 0)
		return -EIO;
	ctx->isp_thread_started = 1;

	memset(&vpss_group_attr, 0, sizeof(vpss_group_attr));
	vpss_group_attr.enDynamicRange = DYNAMIC_RANGE_SDR8;
	vpss_group_attr.enPixelFormat = PIXEL_FORMAT_YVU_SEMIPLANAR_420;
	vpss_group_attr.u32MaxW = width;
	vpss_group_attr.u32MaxH = height;
	vpss_group_attr.bNrEn = HI_TRUE;
	vpss_group_attr.stNrAttr.enNrType = VPSS_NR_TYPE_VIDEO;
	vpss_group_attr.stNrAttr.enNrMotionMode = NR_MOTION_MODE_NORMAL;
	vpss_group_attr.stNrAttr.enCompressMode = COMPRESS_MODE_NONE;
	vpss_group_attr.stFrameRate.s32SrcFrameRate = fps;
	vpss_group_attr.stFrameRate.s32DstFrameRate = fps;
	result = HI_MPI_VPSS_CreateGrp(GK7205_VPSS_GRP, &vpss_group_attr);
	if (gk7205_check("HI_MPI_VPSS_CreateGrp", result) != 0)
		return -EIO;
	ctx->acquired |= GK_VPSS_GRP;
	memset(&vpss_chn_attr, 0, sizeof(vpss_chn_attr));
	vpss_chn_attr.enChnMode = VPSS_CHN_MODE_USER;
	vpss_chn_attr.u32Width = width;
	vpss_chn_attr.u32Height = height;
	vpss_chn_attr.enVideoFormat = VIDEO_FORMAT_LINEAR;
	vpss_chn_attr.enPixelFormat = PIXEL_FORMAT_YVU_SEMIPLANAR_420;
	vpss_chn_attr.enDynamicRange = DYNAMIC_RANGE_SDR8;
	vpss_chn_attr.enCompressMode = COMPRESS_MODE_NONE;
	vpss_chn_attr.stFrameRate.s32SrcFrameRate = fps;
	vpss_chn_attr.stFrameRate.s32DstFrameRate = fps;
	vpss_chn_attr.u32Depth = 0;
	vpss_chn_attr.stAspectRatio.enMode = ASPECT_RATIO_NONE;
	result = HI_MPI_VPSS_SetChnAttr(GK7205_VPSS_GRP, GK7205_VPSS_CHN,
		&vpss_chn_attr);
	if (gk7205_check("HI_MPI_VPSS_SetChnAttr", result) != 0)
		return -EIO;
	result = HI_MPI_VPSS_EnableChn(GK7205_VPSS_GRP, GK7205_VPSS_CHN);
	if (gk7205_check("HI_MPI_VPSS_EnableChn", result) != 0)
		return -EIO;
	ctx->acquired |= GK_VPSS_CHN;
	result = HI_MPI_VPSS_StartGrp(GK7205_VPSS_GRP);
	if (gk7205_check("HI_MPI_VPSS_StartGrp", result) != 0)
		return -EIO;
	ctx->acquired |= GK_VPSS_STARTED;

	memset(&vi_source, 0, sizeof(vi_source));
	vi_source.enModId = HI_ID_VI;
	vi_source.s32DevId = GK7205_VI_DEV;
	vi_source.s32ChnId = GK7205_VI_CHN;
	memset(&vpss_destination, 0, sizeof(vpss_destination));
	vpss_destination.enModId = HI_ID_VPSS;
	vpss_destination.s32DevId = GK7205_VPSS_GRP;
	vpss_destination.s32ChnId = GK7205_VPSS_CHN;
	result = HI_MPI_SYS_Bind(&vi_source, &vpss_destination);
	if (gk7205_check("HI_MPI_SYS_Bind(VI,VPSS)", result) != 0)
		return -EIO;
	ctx->acquired |= GK_VI_BOUND;
	if (gk7205_create_venc(ctx) != 0)
		return -EIO;
	memset(&vpss_source, 0, sizeof(vpss_source));
	vpss_source.enModId = HI_ID_VPSS;
	vpss_source.s32DevId = GK7205_VPSS_GRP;
	vpss_source.s32ChnId = GK7205_VPSS_CHN;
	memset(&venc_destination, 0, sizeof(venc_destination));
	venc_destination.enModId = HI_ID_VENC;
	venc_destination.s32DevId = 0;
	venc_destination.s32ChnId = GK7205_VENC_CHN;
	result = HI_MPI_SYS_Bind(&vpss_source, &venc_destination);
	if (gk7205_check("HI_MPI_SYS_Bind(VPSS,VENC)", result) != 0)
		return -EIO;
	ctx->acquired |= GK_VENC_BOUND;
	return 0;
}

static int gk7205_publish_stream(Gk7205BackendContext *ctx)
{
	VENC_CHN_STATUS_S status;
	VENC_PACK_S vendor_packs[VENC_AU_MAX_PACKS];
	VencAuPackView views[VENC_AU_MAX_PACKS];
	VENC_STREAM_S stream;
	VencFrameMeta meta;
	struct timespec now;
	HI_S32 result;

	result = HI_MPI_VENC_QueryStatus(GK7205_VENC_CHN, &status);
	if (result != HI_SUCCESS)
		return gk7205_check("HI_MPI_VENC_QueryStatus", result);
	if (status.u32CurPacks == 0)
		return 0;
	if (status.u32CurPacks > VENC_AU_MAX_PACKS) {
		venc_frame_ring_note_other_drop(ctx->ring);
		fprintf(stderr, "[gk7205] VENC pack count exceeds bounded AU capacity\n");
		return -EOVERFLOW;
	}
	memset(&stream, 0, sizeof(stream));
	stream.pstPack = vendor_packs;
	stream.u32PackCount = status.u32CurPacks;
	result = HI_MPI_VENC_GetStream(GK7205_VENC_CHN, &stream, 0);
	if (result != HI_SUCCESS)
		return gk7205_check("HI_MPI_VENC_GetStream", result);
	if (stream.u32PackCount == 0 ||
	    stream.u32PackCount > VENC_AU_MAX_PACKS) {
		venc_frame_ring_note_other_drop(ctx->ring);
		(void)HI_MPI_VENC_ReleaseStream(GK7205_VENC_CHN, &stream);
		return -EOVERFLOW;
	}
	memset(&meta, 0, sizeof(meta));
	meta.codec = VENC_FRAME_CODEC_H265;
	if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
		(void)HI_MPI_VENC_ReleaseStream(GK7205_VENC_CHN, &stream);
		return -errno;
	}
	meta.pts = (uint32_t)((uint64_t)now.tv_sec * 1000000u +
		(uint64_t)now.tv_nsec / 1000u);
	for (uint32_t i = 0; i < stream.u32PackCount; ++i) {
		VENC_PACK_S *pack = &stream.pstPack[i];
		if (!pack->pu8Addr || pack->u32Offset >= pack->u32Len) {
			venc_frame_ring_note_other_drop(ctx->ring);
			(void)HI_MPI_VENC_ReleaseStream(GK7205_VENC_CHN, &stream);
			return -EINVAL;
		}
		views[i].data = pack->pu8Addr;
		views[i].data_size = pack->u32Len;
		views[i].offset = pack->u32Offset;
		views[i].length = pack->u32Len - pack->u32Offset;
		if (pack->DataType.enH265EType == H265E_NALU_IDRSLICE)
			meta.flags |= VENC_FRAME_FLAG_IDR;
	}
	if (!stream.pstPack[stream.u32PackCount - 1].bFrameEnd) {
		venc_frame_ring_note_other_drop(ctx->ring);
		(void)HI_MPI_VENC_ReleaseStream(GK7205_VENC_CHN, &stream);
		return 0;
	}
	int publish_result = venc_au_publish(ctx->ring, &meta, views,
		stream.u32PackCount);
	HI_S32 release_result = HI_MPI_VENC_ReleaseStream(GK7205_VENC_CHN,
		&stream);
	if (gk7205_check("HI_MPI_VENC_ReleaseStream", release_result) != 0)
		return -EIO;
	return publish_result == 0 ? 0 : -ENOBUFS;
}

static VencConfig *gk7205_config(void *opaque)
{
	Gk7205BackendContext *ctx = opaque;
	return ctx ? &ctx->cfg : NULL;
}

static int gk7205_prepare(void *opaque)
{
	Gk7205BackendContext *ctx = opaque;
	if (!ctx)
		return -EINVAL;
	if (!ctx->cfg.outgoing.server[0])
		(void)snprintf(ctx->cfg.outgoing.server,
			sizeof(ctx->cfg.outgoing.server), "frame-shm://venc0");
	if (strncmp(ctx->cfg.outgoing.server, "frame-shm://", 12) != 0) {
		fprintf(stderr, "[gk7205] output must use frame-shm:// for vehicle video\n");
		return -EINVAL;
	}
	ctx->cfg.outgoing.enabled = true;
	if (ctx->cfg.video0.width == 0)
		ctx->cfg.video0.width = 1280;
	if (ctx->cfg.video0.height == 0)
		ctx->cfg.video0.height = 720;
	if (ctx->cfg.video0.fps == 0)
		ctx->cfg.video0.fps = 30;
	sensor_framerate = ctx->cfg.video0.fps;
	if (ctx->cfg.video0.width != 1280 || ctx->cfg.video0.height != 720 ||
	    ctx->cfg.video0.fps > 60) {
		fprintf(stderr, "[gk7205] IMX307 mode supports 1280x720 up to 60 fps\n");
		return -EINVAL;
	}
	if (venc_config_parse_output_uri(ctx->cfg.outgoing.server,
		&ctx->output_uri) != 0 ||
	    ctx->output_uri.type != VENC_OUTPUT_URI_FRAME_SHM) {
		fprintf(stderr, "[gk7205] invalid VFRM output URI: %s\n",
			ctx->cfg.outgoing.server);
		return -EINVAL;
	}
	return 0;
}

static int gk7205_init(void *opaque)
{
	Gk7205BackendContext *ctx = opaque;
	int ret;
	if (!ctx)
		return -EINVAL;
	ctx->mipi_fd = -1;
	ctx->venc_fd = -1;
	ret = gk7205_prepare_signals(ctx);
	if (ret != 0)
		return ret;
	ctx->ring = venc_frame_ring_create(ctx->output_uri.endpoint,
		GK7205_RING_SLOTS, GK7205_RING_BYTES);
	if (!ctx->ring)
		return -errno;
	ret = gk7205_init_media(ctx);
	if (ret != 0)
		return ret;
	fprintf(stderr, "[gk7205] IMX307 2-lane 1280x720@%u H.265 VFRM ready\n",
		ctx->cfg.video0.fps);
	return 0;
}

static int gk7205_run(void *opaque)
{
	Gk7205BackendContext *ctx = opaque;
	struct pollfd pfd;
	if (!ctx || !(ctx->acquired & GK_VENC_FD))
		return -EINVAL;
	pfd.fd = ctx->venc_fd;
	pfd.events = POLLIN;
	pfd.revents = 0;
	while (!gk7205_stop_requested) {
		int ready = poll(&pfd, 1, 250);
		if (ready < 0) {
			if (errno == EINTR)
				continue;
			return -errno;
		}
		if (ready == 0)
			continue;
		if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))
			return -EIO;
		if ((pfd.revents & POLLIN) && gk7205_publish_stream(ctx) != 0)
			return -EIO;
		pfd.revents = 0;
	}
	return 0;
}

static void gk7205_teardown(void *opaque)
{
	Gk7205BackendContext *ctx = opaque;
	MPP_CHN_S vi_source;
	MPP_CHN_S vpss_destination;
	MPP_CHN_S vpss_source;
	MPP_CHN_S venc_destination;
	if (!ctx)
		return;
	if (ctx->acquired & GK_VENC_RECV_STARTED) {
		(void)HI_MPI_VENC_StopRecvFrame(GK7205_VENC_CHN);
		ctx->acquired &= ~GK_VENC_RECV_STARTED;
	}
	if (ctx->acquired & GK_VENC_BOUND) {
		memset(&vpss_source, 0, sizeof(vpss_source));
		vpss_source.enModId = HI_ID_VPSS;
		vpss_source.s32DevId = GK7205_VPSS_GRP;
		vpss_source.s32ChnId = GK7205_VPSS_CHN;
		memset(&venc_destination, 0, sizeof(venc_destination));
		venc_destination.enModId = HI_ID_VENC;
		venc_destination.s32DevId = 0;
		venc_destination.s32ChnId = GK7205_VENC_CHN;
		(void)HI_MPI_SYS_UnBind(&vpss_source, &venc_destination);
		ctx->acquired &= ~GK_VENC_BOUND;
	}
	if (ctx->acquired & GK_VENC_FD) {
		(void)HI_MPI_VENC_CloseFd(GK7205_VENC_CHN);
		ctx->acquired &= ~GK_VENC_FD;
	}
	if (ctx->acquired & GK_VENC_CHN_CREATED) {
		(void)HI_MPI_VENC_DestroyChn(GK7205_VENC_CHN);
		ctx->acquired &= ~GK_VENC_CHN_CREATED;
	}
	if (ctx->acquired & GK_VI_BOUND) {
		memset(&vi_source, 0, sizeof(vi_source));
		vi_source.enModId = HI_ID_VI;
		vi_source.s32DevId = GK7205_VI_DEV;
		vi_source.s32ChnId = GK7205_VI_CHN;
		memset(&vpss_destination, 0, sizeof(vpss_destination));
		vpss_destination.enModId = HI_ID_VPSS;
		vpss_destination.s32DevId = GK7205_VPSS_GRP;
		vpss_destination.s32ChnId = GK7205_VPSS_CHN;
		(void)HI_MPI_SYS_UnBind(&vi_source, &vpss_destination);
		ctx->acquired &= ~GK_VI_BOUND;
	}
	if (ctx->acquired & GK_VPSS_STARTED) {
		(void)HI_MPI_VPSS_StopGrp(GK7205_VPSS_GRP);
		ctx->acquired &= ~GK_VPSS_STARTED;
	}
	if (ctx->acquired & GK_VPSS_CHN) {
		(void)HI_MPI_VPSS_DisableChn(GK7205_VPSS_GRP, GK7205_VPSS_CHN);
		ctx->acquired &= ~GK_VPSS_CHN;
	}
	if (ctx->acquired & GK_VPSS_GRP) {
		(void)HI_MPI_VPSS_DestroyGrp(GK7205_VPSS_GRP);
		ctx->acquired &= ~GK_VPSS_GRP;
	}
	if (ctx->acquired & GK_ISP) {
		(void)HI_MPI_ISP_Exit(GK7205_VI_PIPE);
		ctx->acquired &= ~GK_ISP;
	}
	if (ctx->isp_thread_started) {
		(void)pthread_join(ctx->isp_thread, NULL);
		ctx->isp_thread_started = 0;
	}
	if (ctx->acquired & GK_AWB_REGISTERED) {
		(void)HI_MPI_AWB_UnRegister(GK7205_VI_PIPE, &ctx->awb_lib);
		ctx->acquired &= ~GK_AWB_REGISTERED;
	}
	if (ctx->acquired & GK_AE_REGISTERED) {
		(void)HI_MPI_AE_UnRegister(GK7205_VI_PIPE, &ctx->ae_lib);
		ctx->acquired &= ~GK_AE_REGISTERED;
	}
	if (ctx->acquired & GK_SENSOR_REGISTERED) {
		(void)stSnsImx307_2l_Obj.pfnUnRegisterCallback(GK7205_VI_PIPE,
			&ctx->ae_lib, &ctx->awb_lib);
		ctx->acquired &= ~GK_SENSOR_REGISTERED;
	}
	if (ctx->acquired & GK_VI_CHN) {
		(void)HI_MPI_VI_DisableChn(GK7205_VI_PIPE, GK7205_VI_CHN);
		ctx->acquired &= ~GK_VI_CHN;
	}
	if (ctx->acquired & GK_VI_PIPE_STARTED) {
		(void)HI_MPI_VI_StopPipe(GK7205_VI_PIPE);
		ctx->acquired &= ~GK_VI_PIPE_STARTED;
	}
	if (ctx->acquired & GK_VI_PIPE) {
		(void)HI_MPI_VI_DestroyPipe(GK7205_VI_PIPE);
		ctx->acquired &= ~GK_VI_PIPE;
	}
	if (ctx->acquired & GK_VI_DEV) {
		(void)HI_MPI_VI_DisableDev(GK7205_VI_DEV);
		ctx->acquired &= ~GK_VI_DEV;
	}
	if (ctx->acquired & GK_HAVE_SYS) {
		(void)HI_MPI_SYS_Exit();
		ctx->acquired &= ~GK_HAVE_SYS;
	}
	if (ctx->acquired & GK_HAVE_VB) {
		(void)HI_MPI_VB_Exit();
		ctx->acquired &= ~GK_HAVE_VB;
	}
	if (ctx->mipi_fd >= 0) {
		(void)close(ctx->mipi_fd);
		ctx->mipi_fd = -1;
	}
	if (ctx->acquired & GK_SENSOR_CLOCK) {
		int fd = open(DEV_MIPI, O_RDWR);
		if (fd >= 0) {
			(void)mipi_set_sensor_reset(fd, 0, 1);
			(void)mipi_enable_sensor_clock(fd, 0, 0);
			(void)close(fd);
		}
		ctx->acquired &= ~GK_SENSOR_CLOCK;
	}
	if (ctx->acquired & GK_MIPI_CLOCK) {
		int fd = open(DEV_MIPI, O_RDWR);
		if (fd >= 0) {
			(void)mipi_set_reset(fd, 0, 1);
			(void)mipi_enable_clock(fd, 0, 0);
			(void)close(fd);
		}
		ctx->acquired &= ~GK_MIPI_CLOCK;
	}
	if (ctx->ring) {
		venc_frame_ring_destroy(ctx->ring);
		ctx->ring = NULL;
	}
	if (ctx->acquired & GK_SIGNAL_HANDLERS) {
		(void)sigaction(SIGINT, &ctx->old_sigint, NULL);
		(void)sigaction(SIGTERM, &ctx->old_sigterm, NULL);
		ctx->acquired &= ~GK_SIGNAL_HANDLERS;
	}
}

const BackendOps *backend_get_selected(void)
{
	static const BackendOps backend = {
		.name = "GK7205V200",
		.context_size = sizeof(Gk7205BackendContext),
		.config = gk7205_config,
		.prepare = gk7205_prepare,
		.init = gk7205_init,
		.run = gk7205_run,
		.teardown = gk7205_teardown,
	};
	return &backend;
}