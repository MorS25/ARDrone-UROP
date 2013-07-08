#ifndef _IHM_STAGES_O_GTK_H
#define _IHM_STAGES_O_GTK_H

#include <config.h>
#include <VP_Api/vp_api_thread_helper.h>

extern const vp_api_stage_funcs_t vision_stage_funcs;

C_RESULT vision_stage_open( void *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out);
C_RESULT vision_stage_transform( void *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out);
C_RESULT vision_stage_close( void *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out);

#endif // _IHM_STAGES_O_GTK_H
