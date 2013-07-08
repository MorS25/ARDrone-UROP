/**
 * @file main.c
 * @author sylvain.gaeremynck@parrot.com
 * @date 2009/07/01
 */
#include <main.h>
#include <SDL/SDL.h>

//ARDroneLib
#include <utils/ardrone_time.h>
#include <ardrone_tool/Navdata/ardrone_navdata_client.h>
#include <ardrone_tool/Control/ardrone_control.h>
#include <ardrone_tool/UI/ardrone_input.h>

//Common
#include <config.h>
#include <ardrone_api.h>

//VP_SDK
#include <ATcodec/ATcodec_api.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Api/vp_api_thread_helper.h>
#include <VP_Os/vp_os_signal.h>

#include <ardrone_tool/Video/video_stage.h>

//Local project
#include <Video/vision_stage.h>
#include <Control/joypad_control.h>

// Video Processing
#include <opencv2/core/core_c.h>
#include <opencv2/highgui/highgui_c.h>

static int32_t exit_ihm_program = 1;
SDL_Surface* screen;
input_device_t gamepad;// = malloc(sizeof(input_device_t));

/* Implementing Custom methods for the main function of an ARDrone application */
int main(int argc, char** argv)
{
    if (SDL_Init( SDL_INIT_JOYSTICK | SDL_INIT_VIDEO ) < 0)
    {
        fprintf(stderr, "Couldn't initialize SDL: %s\n", SDL_GetError());
        exit(1);
    }

    //Set up the screen
    screen = SDL_SetVideoMode( 640, 480, 32, SDL_SWSURFACE );

    //If there was an error in setting up the screen
    if( screen == NULL )
    {
        return 0; 
    }

    return ardrone_tool_main(argc, argv);
}

C_RESULT start_video_thread(void)
{
    specific_parameters_t *params = (specific_parameters_t *)vp_os_calloc (1, sizeof (specific_parameters_t));
    specific_stages_t *example_pre_stages = (specific_stages_t *)vp_os_calloc (1, sizeof (specific_stages_t));
    specific_stages_t *example_post_stages = (specific_stages_t *)vp_os_calloc (1, sizeof (specific_stages_t));
    vp_api_picture_t *in_picture = (vp_api_picture_t *)vp_os_calloc (1, sizeof (vp_api_picture_t));
    vp_api_picture_t *out_picture = (vp_api_picture_t *)vp_os_calloc (1, sizeof (vp_api_picture_t));
    uint8_t stages_index;
    uint8_t visionCfg;

    /**
     * Fill the vp_api_pictures used for video decoding
     */
    in_picture->width = 640; // Drone 1 only : Must be greater than the drone 1 picture size (320)
    in_picture->height = 360; // Drone 1 only : Must be greater that the drone 1 picture size (240)
    out_picture->format = PIX_FMT_RGB24; // MANDATORY ! Only RGB24, RGB565 are supported
    out_picture->width = in_picture->width;
    out_picture->height = in_picture->height;

    // Alloc Y, CB, CR bufs according to target format
    uint32_t bpp = 0;
    switch (out_picture->format)
    {
        case PIX_FMT_RGB24:
            // One buffer, three bytes per pixel
            bpp = 3;
            out_picture->y_buf = vp_os_malloc ( out_picture->width * out_picture->height * bpp );
            out_picture->cr_buf = NULL;
            out_picture->cb_buf = NULL;
            out_picture->y_line_size = out_picture->width * bpp;
            out_picture->cb_line_size = 0;
            out_picture->cr_line_size = 0;
            break;
        case PIX_FMT_RGB565:
            // One buffer, two bytes per pixel
            bpp = 2;
            out_picture->y_buf = vp_os_malloc ( out_picture->width * out_picture->height * bpp );
            out_picture->cr_buf = NULL;
            out_picture->cb_buf = NULL;
            out_picture->y_line_size = out_picture->width * bpp;
            out_picture->cb_line_size = 0;
            out_picture->cr_line_size = 0;
            break;
        default:
            fprintf (stderr, "Wrong video format, must be either PIX_FMT_RGB565 or PIX_FMT_RGB24\n");
            exit (-1);
            break;
    }

    /**
     * Allocate the stage lists
     *
     * - "pre" stages are called before video decoding is done
     *  -> A pre stage get the encoded video frame (including PaVE header for AR.Drone 2 frames) as input
     *  -> A pre stage MUST NOT modify these data, and MUST pass it to the next stage
     * - Typical "pre" stage : Encoded video recording for AR.Drone 1 (recording for AR.Drone 2 is handled differently)
     *
     * - "post" stages are called after video decoding
     *  -> The first post stage will get the decoded video frame as its input
     *   --> Video frame format depend on out_picture->format value (RGB24 / RGB565)
     *  -> A post stage CAN modify the data, as ardrone_tool won't process it afterwards
     *  -> All following post stages will use the output of the previous stage as their inputs
     * - Typical "post" stage : Display the decoded frame
     */
    example_pre_stages->stages_list = (vp_api_io_stage_t *)vp_os_calloc (0, sizeof (vp_api_io_stage_t));
    example_post_stages->stages_list = (vp_api_io_stage_t *)vp_os_calloc (1, sizeof (vp_api_io_stage_t));

    example_pre_stages->length = 0;

    /**
     * Fill the POST stage list
     * - name and type are debug infos only
     * - cfg is the pointer passed as "cfg" in all the stages calls
     * - funcs is the pointer to the stage functions
     */
    stages_index = 0;

    vp_os_memset (&visionCfg, 0, 0);
    /*    visionCfg.bpp = bpp;
          visionCfg.decoder_info = in_picture;*/

    example_post_stages->stages_list[stages_index].name = "Vision processing"; // Debug info
    example_post_stages->stages_list[stages_index].type = VP_API_OUTPUT_SDL; // Debug info
    example_post_stages->stages_list[stages_index].cfg  = &visionCfg;
    example_post_stages->stages_list[stages_index++].funcs  = vision_stage_funcs;

    example_post_stages->length = stages_index;

    /**
     * Fill thread params for the ardrone_tool video thread
     *  - in_pic / out_pic are reference to our in_picture / out_picture
     *  - pre/post stages lists are references to our stages lists
     *  - needSetPriority and priority are used to control the video thread priority
     *   -> if needSetPriority is set to 1, the thread will try to set its priority to "priority"
     *   -> if needSetPriority is set to 0, the thread will keep its default priority (best on PC)
     */
    params->in_pic = in_picture;
    params->out_pic = out_picture;
    params->pre_processing_stages_list  = example_pre_stages;
    params->post_processing_stages_list = example_post_stages;
    params->needSetPriority = 0;
    params->priority = 0;

    START_THREAD(video_stage, params);
    video_stage_init();

    video_stage_resume_thread ();

    return C_OK;
}

/* The delegate object calls this method during initialization of an ARDrone application */
C_RESULT ardrone_tool_init_custom(void)
{
    strcpy(gamepad.name, "XBox controller");
    gamepad.init = &joypad_init;
    gamepad.update = &joypad_update;
    gamepad.shutdown = &joypad_shutdown;

    /* Registering for a new device of game controller */
    ardrone_tool_input_add( &gamepad );

    /* Opencv Window */
    cvNamedWindow("Video", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("Detect", CV_WINDOW_AUTOSIZE);

    /* Start all threads of your application */
    start_video_thread();

    //  ardrone_tool_set_ui_pad_start(0);

    return C_OK;
}

/* The delegate object calls this method when the event loop exit */
C_RESULT ardrone_tool_shutdown_custom(void)
{
    /* Relinquish all threads of your application */
    //JOIN_THREAD( video_stage );

    /* Destroy opencv window */
    cvDestroyAllWindows();

    /* Unregistering for the current device */
    // ardrone_tool_input_remove( &gamepad );

    return C_OK;
}

/* The event loop calls this method for the exit condition */
bool_t ardrone_tool_exit()
{
    return exit_ihm_program == 0;
}

C_RESULT signal_exit()
{
    exit_ihm_program = 0;

    return C_OK;
}

/* Implementing thread table in which you add routines of your application and those provided by the SDK */
    BEGIN_THREAD_TABLE
    THREAD_TABLE_ENTRY( ardrone_control, 20 )
    THREAD_TABLE_ENTRY( navdata_update, 20 )
THREAD_TABLE_ENTRY( video_stage, 20 )
    END_THREAD_TABLE

