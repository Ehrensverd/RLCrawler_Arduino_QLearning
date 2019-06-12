
#include <MemoryFree.h>
#include <pgmStrToRAM.h>

#define DEBUG //Uncomment for debugging mode

#ifdef DEBUG
#define DEBUG_PRINT(x)    Serial1.print(x)
#define DEBUG_PRINTLN(x)  Serial1.println(x)
#define DEBUG_DELAY(x)    delay(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_DELAY(x) 
#endif


//#define ENCODER_USE_INTERRUPTS
#include <Encoder.h>
#include <Servo.h>
Servo S1, S2;



Encoder rot_encoder(2, 3);
//   avoid using pins with LEDs attached
int h,i,j,k=0;
long oldPosition  = 0;
volatile long newPosition;
long newPosition_hold = 0;
long ms, ms_hold =0;
int vel = 0;
int vel_dt = 10; // dt in milliseconds for differentiate the wheel velocity from the rot_encoder position 
int waitforservos = 70; // wait until the servos moved to new postion in milliseconds

float delta_q = 0;
float max_q = 0;
int max_q_index = 0;
float reward =0;
int action; //up=0; down=1; backwards=2; forwards=3;


int gwS1 = 0; //gridworld state of Servo 1
int gwS2 = 0; //gridworld state of Servo 2
int newgwS1 = 0;
int newgwS2 = 0;
float Q_currentState[4] = {0, 0, 0, 0};    
int movedir;

bool go_backwards = true;


#define TWENTYFIVE_STATES
#ifdef TWENTYFIVE_STATES
#define N_STATES     int n_states[2] = {5, 5}; //{Servo1, Servo2}
//#define SERVOPOS     int servopos[5][2] = {{20, 30}, \
//                                           {50, 60}, \
//                                           {80, 90}, \
//                                           {110, 120}, \
//                                           {140, 150}}; 
#define SERVOPOS     int servopos[5][2] = {{40, 30}, \
                                            {65, 60}, \
                                            {90, 90}, \
                                            {115, 120}, \
                                            {140, 150}}; 
                                          
#define GW2STATE     int gw2state[5][5] = {{0, 1, 2, 3, 4}, \ 
                                            {5, 6, 7, 8, 9}, \
                                            {10, 11, 12, 13, 14}, \
                                            {15, 16, 17, 18, 19}, \
                                            {20, 21, 22, 23, 24}}; //gridworld indices to state number 
#define QMATRIX     float Q[25][4] = {{0, 0, 0, 0},{0, 0, 0, 0},{0, 0, 0, 0},{0, 0, 0, 0}, \
                                      {0, 0, 0, 0},{0, 0, 0, 0},{0, 0, 0, 0},{0, 0, 0, 0}, \
                                      {0, 0, 0, 0},{0, 0, 0, 0},{0, 0, 0, 0},{0, 0, 0, 0}, \
                                      {0, 0, 0, 0},{0, 0, 0, 0},{0, 0, 0, 0},{0, 0, 0, 0}, \
                                      {0, 0, 0, 0},{0, 0, 0, 0},{0, 0, 0, 0},{0, 0, 0, 0}, \
                                      {0, 0, 0, 0},{0, 0, 0, 0},{0, 0, 0, 0},{0, 0, 0, 0}, \
                                      {0, 0, 0, 0}};
#else
#define N_STATES     int n_states[2] = {3, 3};//{Servo1, Servo2} 
#define SERVOPOS     int servopos[3][2] = {{85, 30}, \
                                           {105, 90}, \
                                           {120, 150}};
#define GW2STATE     int gw2state[3][3] = {{0, 1, 2}, \
                                            {3, 4, 5}, \
                                            {6, 7, 8}};
#define QMATRIX     float Q[9][4] = {{0, 0, 0, 0},{0, 0, 0, 0},{0, 0, 0, 0}, \
                                     {0, 0, 0, 0},{0, 0, 0, 0},{0, 0, 0, 0}, \
                                     {0, 0, 0, 0},{0, 0, 0, 0},{0, 0, 0, 0}};                   
#endif
N_STATES
SERVOPOS
GW2STATE
QMATRIX
float e_greed = 50;//93; // factor for  e-greedy action selection in range from 0-99; balance between exploration & exploitation
float e_greed_final = 90; //98
float e_greed_step = 0.015; //value with which the e_greed factor is increased in each iteration. This way exploring at the beginning and exploitation at the end can be achieved.
float alpha = 1.0; //0.85 // Learn rate: Low-pass filter for changes of the entries of the q-matrix in presence of q-value update
float alpha_final = 0.5;
float alpha_step = 0.0005;
float gamma = 0.9; // 0.5 at 9states worked 0.7 //discount factor of Q-value of next state

void setup() {  
  S1.attach(6); //range 20 to 180
  S2.attach(5); //range 10 to 150; 90 is 45degree up-forward; 150 is forward-down; 10 is backwards-up 
  
  Serial1.begin(115200);
  Serial1.println(F("*******************Start-up****************************"));

  
  if(go_backwards)
  {movedir = -1;}
  else
  {movedir = 1;}
  
  randomSeed(25); //35 good for 9 states
  gwS1 = random(0, n_states[0]);
  gwS2 = random(0, n_states[1]); //random initial state
  Serial1.print("Random initial state: ");
  Serial1.println(gw2state[gwS1][gwS2]);
  S1.write(servopos[gwS1][0]);
  S2.write(servopos[gwS2][1]);
  delay(2000);
      Serial1.print("g1: "); Serial1.print(gwS1);
    Serial1.print("g2: "); Serial1.print(gwS2);
    newPosition_hold = rot_encoder.read();
   
}


void loop() {
  
  DEBUG_PRINTLN("****************");
  DEBUG_PRINT("OLD STATE: "); DEBUG_PRINTLN(gw2state[gwS1][gwS2]);
  
//Pick an action by e-greedy policy
  for(i=0;i<4;i++){Q_currentState[i] = Q[gw2state[gwS1][gwS2]][i];}
  action = e_greedy(e_greed, e_greed_final, e_greed_step, Q_currentState);
 
//compute the new state reached by the picked action  
   action2state(gwS1, gwS2, newgwS1, newgwS2, n_states, action); 
      DEBUG_PRINT("NEW STATE: ");DEBUG_PRINTLN(gw2state[newgwS1][newgwS2]);
    //DEBUG_PRINT("ng1: ");DEBUG_PRINTLN(newgwS1);
    //DEBUG_PRINT("ng2: ");DEBUG_PRINTLN(newgwS2);    
  
//update the real world system to the new state -> moving the servos
  S1.write(servopos[newgwS1][0]);
  S2.write(servopos[newgwS2][1]);
  delay(waitforservos);

//Measure change in Position with the rotary encoder
   k=0;
   ms_hold = millis();
  while(true) // wait and measure as long as the velocity is above "vel"
  {
    ms = millis();
      if(ms >= ms_hold + vel_dt) 
      {
       ms_hold = ms;
       newPosition = rot_encoder.read(); 
       vel = ((newPosition - newPosition_hold) * 1000) / vel_dt;
        //DEBUG_PRINT("Velocity: ");Serial1.println(vel);
        //DEBUG_PRINTLN(newPosition);
       
          if(abs(vel) < 300) 
          {
           newPosition_hold = newPosition;
           Serial1.println("BREAK");
           break;
          }
          
          newPosition_hold = newPosition;
       }
      k++;
  }

//Observe Reward
  reward = newPosition - oldPosition;
  reward = reward * movedir; // Negate reward when moving backwards is selected

  DEBUG_PRINT("OLD POS: "); DEBUG_PRINTLN(oldPosition);
  DEBUG_PRINT("NEW POS: "); DEBUG_PRINTLN(newPosition);
  DEBUG_PRINT("REWARD: "); DEBUG_PRINTLN(reward);

  oldPosition = newPosition;  // Store current position for next iteration

//decrease alpha over time to decrease the effect of Q value updates 
  if (alpha >= alpha_final + alpha_step) //Increase e_greedy value ofer time to exploit more after some learning
    {
      alpha = alpha - alpha_step;
    }
  else {alpha = alpha_final;} 
  
//Q-Learning
  max_q = Q[gw2state[newgwS1][newgwS2]][0];
   for (i = 1; i < 4; i++) { //compute possible max q value of new state
    if (Q[gw2state[newgwS1][newgwS2]][i] > max_q) {
      max_q = Q[gw2state[newgwS1][newgwS2]][i];
      }
   }
  delta_q = reward + (gamma * max_q) - Q[gw2state[gwS1][gwS2]][action];
  Q[gw2state[gwS1][gwS2]][action] = Q[gw2state[gwS1][gwS2]][action] + alpha * delta_q;
  
  gwS2 = newgwS2; //Store new state for next iteration
  gwS1 = newgwS1;
  
  for (h=0; h < (n_states[0]*n_states[1]); h++){
    for(i=0;i<4;i++){
      DEBUG_PRINT(Q[h][i]);
      DEBUG_PRINT(", ");
    }
    DEBUG_PRINTLN();
  }

DEBUG_PRINTLN(j); 
j++; 
//delay(300);
}
