/*
 * @video_stage.c
 * @author marc-olivier.dzeukou@parrot.com
 * @date 2007/07/27
 *
 * ihm vision thread implementation
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>

#include <sys/time.h>
#include <time.h>

#include <VP_Api/vp_api.h>
#include <VP_Api/vp_api_error.h>
#include <VP_Api/vp_api_stage.h>
#include <VP_Api/vp_api_picture.h>
#include <VP_Stages/vp_stages_io_file.h>
#include <VP_Stages/vp_stages_i_camif.h>

#include <config.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_delay.h>
#include <VP_Stages/vp_stages_yuv2rgb.h>
#include <VP_Stages/vp_stages_buffer_to_picture.h>
#include <VLIB/Stages/vlib_stage_decode.h>

#include <ardrone_tool/ardrone_tool.h>
#include <ardrone_tool/Com/config_com.h>

#ifndef RECORD_VIDEO
#define RECORD_VIDEO
#endif
#ifdef RECORD_VIDEO
#    include <ardrone_tool/Video/video_stage_recorder.h>
#endif

#include <ardrone_tool/Video/video_com_stage.h>

#include "Video/vision_stage.h"
#include "Video/video_processing.h"

#include <SDL/SDL.h>

#define NB_STAGES 10

PIPELINE_HANDLE pipeline_handle;

static uint8_t*  pixbuf_data       = NULL;
static vp_os_mutex_t  video_update_lock = PTHREAD_MUTEX_INITIALIZER;

static void getPicSizeFromBufferSize (uint32_t bufSize, uint32_t *width, uint32_t *height)
{
    if (NULL == width || NULL == height)
    {
        return;
    }

    switch (bufSize)
    {
    case 50688: //QCIF > 176*144 *2bpp
        *width = 176;
        *height = 144;
        break;
    case 153600: //QVGA > 320*240 *2bpp
        *width = 320;
        *height = 240;
        break;
    case 460800: //360p > 640*360 *2bpp
        *width = 640;
        *height = 360;
        break;
    case 1843200: //720p > 1280*720 *2bpp
        *width = 1280;
        *height = 720;
        break;
    default:
        *width = 0;
        *height = 0;
        break;
    }
}

C_RESULT vision_stage_open( void *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
    printf("\nvision_stage_open\n");
    return (SUCCESS);
}

IplImage *ipl_image_from_data(uint8_t* data, int reduced_image, int width, int height)
{
    IplImage *currframe;
    IplImage *dst;

    currframe = cvCreateImage(cvSize(width,height), IPL_DEPTH_8U, 3);
    dst = cvCreateImage(cvSize(width,height), IPL_DEPTH_8U, 3);

    currframe->imageData = data;
    cvCvtColor(currframe, dst, CV_BGR2RGB);
    cvReleaseImage(&currframe);

    return dst;
}

C_RESULT vision_stage_transform( void *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
    IplImage *cv_out;
    vp_os_mutex_lock(&video_update_lock);

    uint32_t width = 0, height = 0;
    getPicSizeFromBufferSize (in->size, &width, &height);

    IplImage *img = ipl_image_from_data((uint8_t*)in->buffers[0], 1, 640, 360);

    vp_os_mutex_unlock(&video_update_lock);

    cv_out = process(img);

    cvShowImage("Video", img);
    cvShowImage("Detect", cv_out);

    cvWaitKey(1);

    cvReleaseImage(&img);
    cvReleaseImage(&cv_out);

    return (SUCCESS);
}

C_RESULT vision_stage_close( void *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
    return (SUCCESS);
}


const vp_api_stage_funcs_t vision_stage_funcs =
{
    NULL,
    (vp_api_stage_open_t)vision_stage_open,
    (vp_api_stage_transform_t)vision_stage_transform,
    (vp_api_stage_close_t)vision_stage_close
};

/*DEFINE_THREAD_ROUTINE(video_stage, data)
  {
  C_RESULT res;

  vp_api_io_pipeline_t    pipeline;
  vp_api_io_data_t        out;
  vp_api_io_stage_t       stages[NB_STAGES];

  vp_api_picture_t picture;

  video_com_config_t              icc;
  vlib_stage_decoding_config_t    vec;
  vp_stages_yuv2rgb_config_t      yuv2rgbconf;
#ifdef RECORD_VIDEO
video_stage_recorder_config_t   vrc;
#endif
/// Picture configuration
picture.format        = PIX_FMT_YUV420P;

picture.width         = QVGA_WIDTH;
picture.height        = QVGA_HEIGHT;
picture.framerate     = 30;

picture.y_buf   = vp_os_malloc( QVGA_WIDTH * QVGA_HEIGHT     );
picture.cr_buf  = vp_os_malloc( QVGA_WIDTH * QVGA_HEIGHT / 4 );
picture.cb_buf  = vp_os_malloc( QVGA_WIDTH * QVGA_HEIGHT / 4 );

picture.y_line_size   = QVGA_WIDTH;
picture.cb_line_size  = QVGA_WIDTH / 2;
picture.cr_line_size  = QVGA_WIDTH / 2;

vp_os_memset(&icc,          0, sizeof( icc ));
vp_os_memset(&vec,          0, sizeof( vec ));
vp_os_memset(&yuv2rgbconf,  0, sizeof( yuv2rgbconf ));

icc.com                 = COM_VIDEO();
icc.buffer_size         = 100000;
icc.protocol            = VP_COM_UDP;
COM_CONFIG_SOCKET_VIDEO(&icc.socket, VP_COM_CLIENT, VIDEO_PORT, wifi_ardrone_ip);

vec.width               = QVGA_WIDTH;
vec.height              = QVGA_HEIGHT;
vec.picture             = &picture;
vec.block_mode_enable   = TRUE;
vec.luma_only           = FALSE;

yuv2rgbconf.rgb_format = VP_STAGES_RGB_FORMAT_RGB24;
#ifdef RECORD_VIDEO
vrc.fp = NULL;
#endif

pipeline.nb_stages = 0;

stages[pipeline.nb_stages].type    = VP_API_INPUT_SOCKET;
stages[pipeline.nb_stages].cfg     = (void *)&icc;
stages[pipeline.nb_stages].funcs   = video_com_funcs;

pipeline.nb_stages++;

#ifdef RECORD_VIDEO
stages[pipeline.nb_stages].type    = VP_API_FILTER_DECODER;
stages[pipeline.nb_stages].cfg     = (void*)&vrc;
stages[pipeline.nb_stages].funcs   = video_recorder_funcs;

pipeline.nb_stages++;
#endif // RECORD_VIDEO
stages[pipeline.nb_stages].type    = VP_API_FILTER_DECODER;
stages[pipeline.nb_stages].cfg     = (void*)&vec;
stages[pipeline.nb_stages].funcs   = vlib_decoding_funcs;

pipeline.nb_stages++;

stages[pipeline.nb_stages].type    = VP_API_FILTER_YUV2RGB;
stages[pipeline.nb_stages].cfg     = (void*)&yuv2rgbconf;
stages[pipeline.nb_stages].funcs   = vp_stages_yuv2rgb_funcs;

pipeline.nb_stages++;

stages[pipeline.nb_stages].type    = VP_API_OUTPUT_SDL;
stages[pipeline.nb_stages].cfg     = NULL;
stages[pipeline.nb_stages].funcs   = vp_stages_vision_funcs;

pipeline.nb_stages++;

pipeline.stages = &stages[0];*/

/* Processing of a pipeline */
/*    if( !ardrone_tool_exit() )
      {
      PRINT("\n   Video stage thread initialisation\n\n");

      res = vp_api_open(&pipeline, &pipeline_handle);

      if( SUCCEED(res) )
      {
      int loop = SUCCESS;
      out.status = VP_API_STATUS_PROCESSING;

      while( !ardrone_tool_exit() && (loop == SUCCESS) )
      {
      if( SUCCEED(vp_api_run(&pipeline, &out)) ) {
      if( (out.status == VP_API_STATUS_PROCESSING || out.status == VP_API_STATUS_STILL_RUNNING) ) {
      loop = SUCCESS;
      }
      }
      else loop = -1; // Finish this thread
      }

      vp_api_close(&pipeline, &pipeline_handle);
      }
      }

      PRINT("   Video stage thread ended\n\n");

      return (THREAD_RET)0;
      }*/

