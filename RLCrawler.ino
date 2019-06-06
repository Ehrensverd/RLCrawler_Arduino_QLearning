#define DEBUG //Uncomment for debugging mode

#ifdef DEBUG
#define DEBUG_PRINT(x)    hc05.print(x)
#define DEBUG_PRINTLN(x)  hc05.println(x)
#define DEBUG_DELAY(x)    delay(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_DELAY(x) 
#endif

#include <Encoder.h>
#include <SoftwareSerial.h> // import the serial library
#include <Servo.h>
Servo S1, S2;

SoftwareSerial hc05(10, 11); // RX, TX
// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder rot_encoder(2, 3);
//   avoid using pins with LEDs attached
int oldPosition  = 0;
int newPosition = 0;
int btdata= 0;
int i,j=0;

float delta_q = 0;
float max_q = 0;
int max_q_index = 0;
int reward;
int action; //up=0; down=1; backwards=2; forwards=3;
int state[2] = {0, 0}; //Servo1, Servo2
int newstate[2] = {0, 0}; //Servo1, Servo2
int n_states[2] = {5, 5};
int servopos[5][2] = {{20, 30}, //{Servo1, Servo2}
                      {50, 60},
                      {80, 90},
                      {110, 120},
                      {140, 150}};
int stateindex[5][5] = {{1, 2, 3, 4, 5},
                        {6, 7, 8, 9, 10},
                        {11, 12, 13, 14, 15},
                        {16, 17, 18, 19, 20},
                        {21, 22, 23, 24, 25}};
float Q[25][4] = {{0, 0, 0, 0},{0, 0, 0, 0},{0, 0, 0, 0},{0, 0, 0, 0},
                  {0, 0, 0, 0},{0, 0, 0, 0},{0, 0, 0, 0},{0, 0, 0, 0},
                  {0, 0, 0, 0},{0, 0, 0, 0},{0, 0, 0, 0},{0, 0, 0, 0},
                  {0, 0, 0, 0},{0, 0, 0, 0},{0, 0, 0, 0},{0, 0, 0, 0},
                  {0, 0, 0, 0},{0, 0, 0, 0},{0, 0, 0, 0},{0, 0, 0, 0},
                  {0, 0, 0, 0},{0, 0, 0, 0},{0, 0, 0, 0},{0, 0, 0, 0},
                  {0, 0, 0, 0}};
float e_greed = 50;//93; // factor for  e-greedy action selection in range from 0-99; balance between exploration & exploitation
float e_greed_final = 93; //98
float e_greed_step = 0.075; //value with which the e_greed factor is increased in each iteration. This way exploring at the beginning and exploitation at the end can be achieved.
float alpha = 1.0; //0.85 // Learn rate: Low-pass filter for changes of the entries of the q-matrix in presence of q-value update
float alpha_final = 0.1;
float alpha_step = 0.0005;
float gamma = 0.7; // 0.7 //discount factor of Q-value of next state

void setup() {  
  S1.attach(6); //range 20 to 180
  S2.attach(5); //range 10 to 150; 90 is 45degree up-forward; 150 is forward-down; 10 is backwards-up 
  hc05.begin(115200);
  hc05.println("Start-up");
  randomSeed(55);
  state[0] = random(0, 5);
  state[1] = random(0, 5); //random initial state
  S1.write(servopos[state[0]][0]);
  S2.write(servopos[state[1]][1]);
  delay(2000);
}


void loop() {
  
  DEBUG_PRINTLN(j);
  //Pick an action by e-greedy policy
  action = e_greedy(e_greed, e_greed_final, e_greed_step, Q, stateindex[state[0]][state[1]]);
  DEBUG_PRINT("Action in main: ");
  DEBUG_PRINTLN(action);
  DEBUG_PRINT("STATE: ");
  DEBUG_PRINT(state[0]); DEBUG_PRINT(" "); DEBUG_PRINTLN(state[1]);
  //compute the new state reached by the picked action    
    for (i=0; i < 2; i++)
    {newstate[i] = state[i];}
    
  if ((state[0] == 0 && action == 0)||(state[0] == n_states[0]-1 && action == 1)) //actions up=1; down=2;
    {for (i=0; i < 2; i++)
    {newstate[i] = state[i];}DEBUG_PRINT("1111111");} 
  else if ((state[1] == 0 && action == 2)||(state[1] == n_states[1]-1 && action == 3)) //actions backwards=3; forwards=4;
    {for (i=0; i < 2; i++)
    {newstate[i] = state[i];}DEBUG_PRINT("2222222");} 
  else if (action == 0)
    {newstate[0] = state[0] - 1;DEBUG_PRINT("3333333");} 
  else if (action == 1)
    {newstate[0] = state[0] + 1;DEBUG_PRINT("4444444");} 
  else if (action == 2)
    {newstate[1] = state[1] - 1;DEBUG_PRINT("5555555");} 
  else if (action == 3)
    {newstate[1] = state[1] + 1;DEBUG_PRINT("6666666");} 
  //else {for (i=0; i < 2; i++)
    //{newstate[i] = state[i];}}
    
  //action2state(newstate[0], newstate[1], n_states, action);
  
  DEBUG_PRINT("NEWSTATE: ");
  DEBUG_PRINT(newstate[0]); DEBUG_PRINT(" "); DEBUG_PRINTLN(newstate[1]);
  DEBUG_PRINTLN(servopos[newstate[0]][0]);
  DEBUG_PRINTLN(servopos[newstate[1]][1]);
  //update the real world system to the new state -> moving the servos
  S1.write(servopos[newstate[0]][0]);
  S2.write(servopos[newstate[1]][1]);
  delay(700);
  
  newPosition = rot_encoder.read();
  reward = newPosition - oldPosition;
    //hc05.print("Current Position: ");
    //hc05.println(newPosition);
    //if(hc05.available())
    //{
    //btdata = hc05.read();
    // } 
  oldPosition = newPosition;
  
  //compute Q update
  
  max_q = Q[stateindex[state[0]][state[1]]][0];
  for (i = 1; i < 4; i++) { //compute possible max q value of new state
    if (Q[stateindex[state[0]][state[1]]][i] > max_q) {
      max_q = Q[stateindex[state[0]][state[1]]][i];
    }
  }
  if (alpha >= alpha_final + alpha_step) //Increase e_greedy value ofer time to exploit more after some learning
    {
      alpha = alpha - alpha_step;
    }
  else {alpha = alpha_final;} 

  delta_q = reward + (gamma * max_q) - Q[stateindex[state[0]][state[1]]][action];
  Q[stateindex[state[0]][state[1]]][action] = Q[stateindex[state[0]][state[1]]][action] + alpha * delta_q;


  j++;
}
