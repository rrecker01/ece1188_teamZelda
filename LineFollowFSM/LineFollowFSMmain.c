#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/LaunchPad.h"
#include "../inc/TExaS.h"
#include "Reflectance.h"
#include "../inc/MotorSimple.h"

/*(Left,Right) Motors, call LaunchPad_Output (positive logic)
3   1,1     both motors, yellow means go straight
2   1,0     left motor,  green  means turns right
1   0,1     right motor, red    means turn left
0   0,0     both off,    dark   means stop
(Left,Right) Sensors, call LaunchPad_Input (positive logic)
3   1,1     both buttons pushed means on line,
2   1,0     SW2 pushed          means off to right
1   0,1     SW1 pushed          means off to left
0   0,0     neither button      means lost
 */

// Linked data structure
struct State {
  uint32_t out;                // 2-bit output
  uint32_t delay;              // time to delay in 1ms
  const struct State *next[4]; // Next if 2-bit input is 0-3
};
typedef const struct State State_t;

#define Center &fsm[0]
#define Left   &fsm[1]
#define Right  &fsm[2]
State_t fsm[3]={
  {0x03, 500, { Right, Left,   Right,  Center }},  // Center
  {0x02, 500, { Left,  Center, Right,  Center }},  // Left
  {0x01, 500, { Right, Left,   Center, Center }}   // Right
};
State_t *Spt;  // pointer to the current state
uint32_t Input;
uint32_t Output;
/*Run FSM continuously
1) Output depends on State (LaunchPad LED)
2) Wait depends on State
3) Input (LaunchPad buttons)
4) Next depends on (Input,State)
 */
int main(void){ uint32_t heart=0;
  Clock_Init48MHz();
  LaunchPad_Init();
  Reflectance_Init();
  Motor_InitSimple();
  //TExaS_Init(LOGICANALYZER);  // optional
  Spt = Center;
  while(1){
    Output = Spt->out;            // set output from FSM
    LaunchPad_Output(Output);     // do output to two motors
    if(Output == 0x03)
        Motor_ForwardSimple(1000, 500);
    else if(Output == 0x02)
        Motor_LeftSimple(1000, 500);
    else if(Output == 0x01)
        Motor_RightSimple(1000, 500);
    //TExaS_Set(Input<<2|Output);   // optional, send data to logic analyzer
    Clock_Delay1ms(Spt->delay);   // wait
    //Input = LaunchPad_Input();
    Input = Reflectance_Read(1000)&0x18;    // read sensors
    Spt = Spt->next[Input>>3];       // next depends on input and state
    heart = heart^1;
    LaunchPad_LED(heart);         // optional, debugging heartbeat
  }
}

// Color    LED(s) Port2
// dark     ---    0
// red      R--    0x01
// blue     --B    0x04
// green    -G-    0x02
// yellow   RG-    0x03
// sky blue -GB    0x06
// white    RGB    0x07
// pink     R-B    0x05
