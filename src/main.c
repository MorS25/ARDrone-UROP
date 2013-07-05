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

//Local project
#include <Video/video_stage.h>
#include <Control/joypad_control.h>

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

/* The delegate object calls this method during initialization of an ARDrone application */
C_RESULT ardrone_tool_init_custom(void)
{
    strcpy(gamepad.name, "XBox controller");
    gamepad.init = &joypad_init;
    gamepad.update = &joypad_update;
    gamepad.shutdown = &joypad_shutdown;

    /* Registering for a new device of game controller */
    ardrone_tool_input_add( &gamepad );

    /* Start all threads of your application */
    START_THREAD( video_stage, NULL );

    return C_OK;
}

/* The delegate object calls this method when the event loop exit */
C_RESULT ardrone_tool_shutdown_custom(void)
{
    /* Relinquish all threads of your application */
    JOIN_THREAD( video_stage );

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

