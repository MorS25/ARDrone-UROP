#include <Control/automated_control.h>
#include <Control/joypad_control.h>

#include <ardrone_tool/UI/ardrone_input.h>
#include <ardrone_api.h>
#include <SDL/SDL.h>

static uint8_t canFly = 0;
static uint8_t takeOffFlag = 0;
static uint8_t emergencyFlag = 1;

C_RESULT automated_init(void)
{
    return joypad_init();
    return C_OK;
}

static float horizontalStrafe = 0;
static float ahead = 0;
static float horizontalTurn = 0;
static float vertical = 0;
static uint8_t can_see_ball = 0;

void set_target_location(int32_t x, int32_t y, uint32_t maxX, uint32_t maxY, uint8_t sees_ball)
{
    if(!(can_see_ball = sees_ball))
        ardrone_tool_set_ui_pad_start(takeOffFlag = 0);

    float newHorizontalStrafe = (float)x / maxX;
    if(fabs(newHorizontalStrafe) > 0.15)
        horizontalStrafe = newHorizontalStrafe;
    else
        horizontalStrafe = 0;
    float newVertical = (float)y / maxY;
    if(fabs(newVertical) > 0.15)
        vertical = newVertical;
    else
        vertical = 0;
}

C_RESULT automated_update(void)
{
    if(!can_see_ball)
    {
        ardrone_tool_set_ui_pad_start(takeOffFlag = 0);
        return C_OK;
    }
    if(vertical > 0 && !takeOffFlag)
        ardrone_tool_set_ui_pad_start(takeOffFlag = !takeOffFlag);

    printf("horizontalStrafe: %f\nvertical: %f\n\n", horizontalStrafe, vertical);

//    if(vertical > 0)
//    {
//    if(1 || canFly)
//    {
        ardrone_tool_set_progressive_cmd( 1,
            /*roll*/horizontalStrafe,
            /*pitch*/ahead,
            /*gaz*/vertical,
            /*yaw*/horizontalTurn,
            /*psi*/0.0,
            /*psi_accuracy*/0.0);
//    }
//    printf("roll: %f, pitch: %f, gaz: %f, yaw: %f\n",
//            /*roll*/(float)(leftStickX)/32768.0f,
//            /*pitch*/(float)(leftStickY)/32768.0f,
//            /*gaz*/-(float)(rightStickY)/32768.0f,
//            /*yaw*/(float)(rightStickX)/32768.0f);*/
//    }
//    else
//    {
//        ardrone_tool_set_ui_pad_start(takeOffFlag = 0);
//    }

    joypad_update();

    return C_OK;
}

C_RESULT automated_shutdown(void)
{
    printf("Shutdown\n");
    ardrone_tool_set_ui_pad_start(takeOffFlag = 0);
    return C_OK;
}
