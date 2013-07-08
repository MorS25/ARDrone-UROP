#include <Control/joypad_control.h>

#include <ardrone_tool/UI/ardrone_input.h>
#include <ardrone_api.h>
#include <SDL/SDL.h>

static SDL_Joystick *joystick;
static uint8_t canFly = 0;
static uint8_t takeOffFlag = 0;
static uint8_t emergencyFlag = 1;

C_RESULT joypad_init(void)
{
    SDL_JoystickEventState(SDL_ENABLE);
    joystick = SDL_JoystickOpen(0);

    if(joystick)
    {
        printf("Number of joysticks: %d\n", SDL_NumJoysticks());
        printf("Name: %s\n", SDL_JoystickName(0));
        printf("Number of Axes: %d\n", SDL_JoystickNumAxes(joystick));
        printf("Number of Buttons: %d\n", SDL_JoystickNumButtons(joystick));
        printf("Number of Balls: %d\n", SDL_JoystickNumBalls(joystick));
    }
    else
        printf("Couldn't open Joystick 0\n");

    return C_OK;
}

static int16_t leftStickX = 0;
static int16_t leftStickY = 0;
static int16_t rightStickX = 0;
static int16_t rightStickY = 0;

C_RESULT joypad_update(void)
{
    SDL_Event event;

    while(SDL_PollEvent(&event))
    {  
        switch(event.type)
        {
            case SDL_JOYAXISMOTION:  /* Handle Joystick Motion */
                if ( ( event.jaxis.value < -32768 / 5 ) || (event.jaxis.value > 32768 / 5 ) ) 
                {
                    switch(event.jaxis.axis)
                    {
                        case 0:
                            printf("left X\n");
                            leftStickX = event.jaxis.value;
                            break;
                        case 1:
                            printf("left Y\n");
                            leftStickY = event.jaxis.value;
                            break;
                        case 2:
                            printf("right X\n");
                            rightStickX= event.jaxis.value;
                            break;
                        case 3:
                            printf("right Y\n");
                            rightStickY = event.jaxis.value;
                            break;
                        case 4:
                            printf("right T\n");
                            break;
                        case 5:
                            printf("left T\n");
                            break;
                    }
                }
                else
                {
                    switch(event.jaxis.axis)
                    {
                        case 0:
                            leftStickX = 0;
                            break;
                        case 1:
                            leftStickY = 0;
                            break;
                        case 2:
                            rightStickX= 0;
                            break;
                        case 3:
                            rightStickY = 0;
                            break;
                        case 4:
                            printf("right T\n");
                            break;
                        case 5:
                            printf("left T\n");
                            break;
                    }
                }
                break;
            case SDL_JOYBUTTONDOWN:
                switch(event.jbutton.button)
                {
                    case 0:
                        printf("A pressed\n");
                        takeOffFlag = !takeOffFlag;
                        ardrone_tool_set_ui_pad_start(takeOffFlag);
                        break;
                    case 1:
                        break;
                    case 2:
                        break;
                    case 3:
                        break;
                    case 4:
                        break;
                    case 5:
                        break;
                    case 6:
                        break;
                    case 7:
                        break;
                    case 8:
                        emergencyFlag = !emergencyFlag;
                        ardrone_tool_set_ui_pad_select(emergencyFlag);
                        break;
                    case 9:
                        break;
                    case 10:
                        canFly = 1;
                        break;
                }
            case SDL_JOYBUTTONUP:
                switch(event.jbutton.button)
                {
                    case 0:
                        break;
                    case 1:
                        break;
                    case 2:
                        break;
                    case 3:
                        break;
                    case 4:
                        break;
                    case 5:
                        break;
                    case 6:
                        break;
                    case 7:
                        break;
                    case 8:
                        break;
                    case 9:
                        break;
                    case 10:
                        canFly = 0;
                        break;
                }
                // 0: A
                // 1: B
                // 2: X
                // 3: Y
                // 4: LB
                // 5: RB
                // 6: Back
                // 7: Start
                // 8: Guide
                // 9: Left Stick Down
                // 10: Right Stick Down
                break;
            case SDL_QUIT:
                /* Set whatever flags are necessary to */
                /* end the main game loop here */
                break;
        }
    }

//    if(1 || canFly)
//    {
//        ardrone_tool_set_progressive_cmd( 1,
//            /*roll*/(float)(leftStickX)/32768.0f,
//            /*pitch*/(float)(leftStickY)/32768.0f,
//            /*gaz*/-(float)(rightStickY)/32768.0f,
//            /*yaw*/(float)(rightStickX)/32768.0f,
//            /*psi*/0.0,
//            /*psi_accuracy*/0.0);
//    }
//    printf("roll: %f, pitch: %f, gaz: %f, yaw: %f\n",
//            /*roll*/(float)(leftStickX)/32768.0f,
//            /*pitch*/(float)(leftStickY)/32768.0f,
//            /*gaz*/-(float)(rightStickY)/32768.0f,
//            /*yaw*/(float)(rightStickX)/32768.0f);*/

    return C_OK;
}

C_RESULT joypad_shutdown(void)
{
    return C_OK;
}
