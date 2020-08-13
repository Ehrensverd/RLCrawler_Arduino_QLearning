#include <Encoder.h>
#include <Servo.h>
#include <math.h>
Servo S1, S2, S3;
Encoder rot_encoder(2, 3); //avoid using pins with LEDs attached

int h,i,j,k,n=0;
long oldPosition  = 0;
volatile long newPosition;
long newPosition_hold = 0;
long ms, ms_hold =0;
long rnd_seed;
int vel = 0;
int vel_dt = 10; // dt in milliseconds for differentiate the wheel velocity from the rot_encoder position 

float delta_q = 0;
float max_q = 0;
float vertical = 160;
float reward =0;

int gwS1, gwS2, gw_phase, newgwS1, newgwS2, newgw_phase = gwS1 = gwS2 = gw_phase = newgwS1 = newgwS2 = newgw_phase = 0;

int movedir;
float del;

float e_greed = 90;//93; // factor for  e-greedy action selection in range from 0-99; balance between exploration & exploitation
float e_greed_final = 90; //98
float e_greed_step = 0.015; //value with which the e_greed factor is increased in each iteration. This way exploring at the beginning and exploitation at the end can be achieved.
float alpha = 0.3; //0.85 // Learn rate: Low-pass filter for changes of the entries of the q-matrix in presence of q-value update
//float alpha_final = 0.5;
//float alpha_step = 0.0005;
float gamma = 0.95; // 0.5 at 9states worked 0.7 //discount factor of Q-value of next state
bool go_backwards = false;

// utility identifiers for readablity
const int phase_shift = 0;
const int servo_1 = 1;
const int servo_2 = 2; // state_space_high[servo_1] and angle_delta[servo_1] vs state_space_high[1] and angle_delta[0]
 int old_state = 0;
 int new_state = 0; // only for readability
    bool phased = false;
    bool sine2_amped = false;
    bool plotting = false;

// State variables
const int state_size = 5;  // amount of states per of a state type
int n_states[3] = {state_size, state_size, state_size}; //{Servo2_phaseshift, Servo1_amplitude, Servo2_amplitude, } %

const int state_types = 3;
const int state_space = pow(state_size, state_types);
double state_space_high[2]; // highest sinus amplitude state
float sine_states[state_size][state_types];

// servo and angles
  
int servo_delay = 8;
double next_angle, next_angle_2;
const int min_max_angles[2][2] = {{50, 150}, {30, 150}};
const int SERVO_STEPS = 210; // amount of servo steps per iteration. less = larger angular intervals
int angle_delta[2]; // distance between min and max servo angles, used for partitioning sinus amplitudes
double s2_old_angle = 90;
double radii[SERVO_STEPS];

// All possible states. 0 to  state_space -1.
int gw2state[state_size][state_size][state_size]; // [phase_shift][servo_1][servo_2]

// actions                                              // 0 = do nothing
const int actions = 7;                                  // 1 = phase_shift--
int action;                                             // 2 = phase_shift++
float Q_currentState[actions] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, };                    // 3 = s1--
bool is_invalid[actions] = {false, false, false, false, false, false, false, };                               // 4 = s1++
                                                        // 5 = s2--
                                                        // 6 = s2++

// Q table. Q ( state , action )
float Q[state_space][actions];

void setup() {  
   // while (!Serial){};   // wait for serial port to connect. Needed for Native USB only
  Serial1.begin(115200);
   
      
     // calculate max amplitude and angle delta for sinus states.
    for (i = 0 ; i < 2 ; i ++ ){//servo min    + servo max
        angle_delta[i] = (min_max_angles[i][0] + min_max_angles[i][1])/2.0;
        state_space_high[i] = (min_max_angles[i][1] - angle_delta[i]); //

        
      
    }
    // min max sine_amplitude *2 , phase_shift
    float low_high[2][3] = {{0, 20,  20}, {2, state_space_high[0],  state_space_high[1]}};

    // state step distance = (high-low)/state_size - 1. //default: Servo1: 0.1, 0.275, 0.45, 0.625, 0.8 |  Servo2: 0.1, 0.25, 0.4, 0.55, 0.7 |   phase_shift: 0, 0.4, 0.8, 1.2, 1.6.   | phase 2 and 0 are same phase
  
    for(i = 0 ; i < state_size; i++){
        for (j = 0 ; j < state_types; j++ ){   //      distance
                                       // base + ( max ampl        -  min ampl ) /                   *   index)
            if(j==0){
               sine_states[i][j] = low_high[0][j] + (((low_high[1][j] - low_high[0][j]) / (state_size)) * i); // phase shift resyncs at 2, so we dont include it.
              } else {
            sine_states[i][j] = low_high[0][j] + (((low_high[1][j] - low_high[0][j]) / (state_size-1)) * i);
            }
           
        }
 
    }
    //gridworld indices to state populating. from 0 to state_space-1 
    for (i = 0, n = 0 ; i < state_size ; i++){
        for (j = 0 ; j < state_size ; j++ ){
            for(k = 0 ; k < state_size ; k++, n++){
                gw2state[i][j][k] = n;
            }
        }
    }
    // Q_table init. 
    init_Q_table();
    
    // Sinus wave partitioner
    double delta = (double)(2*M_PI / SERVO_STEPS);
    for (i = 0 ; i < SERVO_STEPS; i++){
        radii[i] = i * delta;
    }

    
    S1.attach(6); //range 20 to 180
    S2.attach(5); //range 10 to 150; 90 is 45degree up-forward; 150 is forward-down; 10 is backwards-up
    S3.attach(4);
    
     Serial1.println("*******************Start-up****************************");

      // sets which direction to reward
      movedir = go_backwards ? -1 : 1;
      
    delay(1000);
      //randomSeed(25); //35 good for 9 states

      for (int p = A0; p <= A7; p++)  {
          if (random(2)) {
              rnd_seed += analogRead(p);
        } else  {
              rnd_seed *= analogRead(p);
        }
        randomSeed(abs(rnd_seed));
      }

       //random initial state
      gwS1 = random(state_size);
      if(gwS1<0 || gwS1 >= state_size){
          gwS1 = 0;
      }
      gwS2 = random(state_size);
      if(gwS2<0 || gwS2 >= state_size){
          gwS2 = 0;
      }
      gw_phase = random(state_size);
      if(gw_phase<0 || gw_phase >= state_size){
          gw_phase = 0;
      }

      S1.write(angle_delta[0]);
      S2.write(angle_delta[1]);
      S3.write(90);
      s2_old_angle = angle_delta[1] +10;
      delay(1000);

      newPosition_hold = rot_encoder.read();

      print_info();
}

void loop() {

    old_state = gw2state[gw_phase][gwS1][gwS2];
    phased = false;

   
    // get current state Q table actions
    for(i=0;i<7;i++){Q_currentState[i] = Q[old_state][i];}
 
    
    // check fo invalid actions. 0, 1 and 2 allways valid
    is_invalid[3] = (gwS1 == 0);
    is_invalid[4] = (gwS1 == state_size-1);
    is_invalid[5] = (gwS2 == 0);
    is_invalid[6] = (gwS2 == state_size-1);

    //Pick an action by e-greedy policy
    action = e_greedy(e_greed, e_greed_final, e_greed_step, Q_currentState, is_invalid, plotting);

    // Update state changed by action.
    // action 0 does no changes. ie. repeat last state.
    switch (action) {
    case 1:  // phase shift has no min max, its circular
        newgw_phase = (gw_phase == 0)   ? state_size-1 : newgw_phase -1;
        phased = true;
        break;
    case 2:
        newgw_phase = (gw_phase == state_size - 1) ? 0 : newgw_phase + 1;
        phased = true;
        break;
    case 3:  // servo 1 
        newgwS1--;
        break;
    case 4:
        newgwS1++;
        break;
     case 5:  // servo 2 
        newgwS2--;
        sine2_amped = true;
        break;
    case 6:
        newgwS2++;
        sine2_amped = true;
        break;    
    }
    new_state = gw2state[newgw_phase][newgwS1][newgwS2]; // only for readability


      if(!plotting){
        Serial1.println("s1:" + String(newgwS1) + "|s2:" + String(newgwS2) + "|ps:" + String(newgw_phase) );
     }
    // motor iteration

    
    
    for (i = 0 ; i < SERVO_STEPS ; i++){
        // S1
        next_angle = ((sin(radii[i]) * sine_states[newgwS1][servo_1]))+angle_delta[0];
        S1.write(next_angle);
        delay(servo_delay);

        // S2
        next_angle_2 = ((sin(radii[i]+ (M_PI*sine_states[newgw_phase][phase_shift])) * sine_states[newgwS2][servo_2] ))+angle_delta[1];
           
        if(sine2_amped){ 
            next_angle_2 = ((sin(radii[i]+ (M_PI*sine_states[newgw_phase][phase_shift])) * sine_states[gwS2][servo_2] ))+angle_delta[1];
            if(abs(next_angle_2 - angle_delta[1])<1){
                sine2_amped = false;
              }
          } 
        else if(phased){
           // if phase_shifted wait till old S2 angle matches new
            if(abs(s2_old_angle - next_angle_2) < 1){
                phased = false;
              } else{
          next_angle_2 = s2_old_angle;
          }
        }
      
        
           //Serial1.print("E" +String(int(next_angle))+","+String(int(next_angle_2))+"\n");
            S2.write(next_angle_2);
      
       
        delay(servo_delay);
    if(plotting && i%5 == 0){

            Serial1.print("Graph:");
            Serial1.print(next_angle);
            Serial1.print("|");
            Serial1.print(next_angle_2);
            Serial1.print("$");
            Serial1.println();
          
          }
    }
    phased = false;
    sine2_amped = false;
    s2_old_angle = next_angle_2;

     // flush buffer
     if(plotting  && Serial1.available()){
        if(int(Serial1.read()) == 1){
          plotting = false;
          Serial1.println("Stopping graph plotting");
          } 

      
     while(Serial1.read() >= 0);

     }
    //Measure change in Position with the rotary encoder

    ms_hold = millis();
    // wait and measure as long as the velocity is above "vel"
    while(true) {
        ms = millis();
        if(ms >= ms_hold + vel_dt){
            ms_hold = ms;
            newPosition = rot_encoder.read();
            vel = ((newPosition - newPosition_hold) * 1000) / vel_dt;
       
            if(abs(vel) < 300){
                newPosition_hold = newPosition;
           //  Serial1.println("BREAK");
                break;
          }
            newPosition_hold = newPosition;
       }
  }

    //Observe Reward
    reward = newPosition - oldPosition;
    reward = reward * movedir; // Negate reward when moving backwards is selected
    reward = (reward/100); //scale rewards a bit
  
    reward = pow(reward, 3);     //emphasize larger rewards
    reward = reward - 100; //give a penalty of -20 for each step to keep the bot from ideling
    if(!plotting){
    Serial1.println("Reward:" + String(reward));
    Serial1.println();
    }
    // Store current position for next iteration
    oldPosition = newPosition;

    //decrease alpha over time to decrease the effect of Q value updates
    // if (alpha >= alpha_final + alpha_step) //Increase e_greedy value ofer time to exploit more after some learning
    //   {
    //     alpha = alpha - alpha_step;
    //   }
    // else {alpha = alpha_final;}

    //Q-Learning
    max_q = Q[new_state][0];
    for (i = 1; i < actions; i++) { //compute possible max q value of new state
        if (Q[new_state][i] > max_q) {
            max_q = Q[new_state][i];
        }
    }

    delta_q = reward + (gamma * max_q) - Q[old_state][action];
    Q[old_state][action] = Q[old_state][action] + alpha * delta_q;

    //Store new state for next iteration
    gwS2 = newgwS2;
    gwS1 = newgwS1;
    gw_phase = newgw_phase;


    if (!plotting && Serial1.available()){
      int BT_out =  Serial1.read();
         switch (BT_out) {
    case 0:  // change direction
          movedir = movedir *-1;
          movedir == -1 ? Serial1.println("Going backwards") :Serial1.println("Going forwards");
          break;
    case 1:  //  plotting on off
          plotting = true;
          Serial1.println("Starting graph plotting"); 
          break;
     
    case 2:
          for (i= 90; i >=65; i -= 1) {
              S3.write(i);          
              delay(15);                   
          }
        break;
    case 3:  // servo 1 
          for (i= 65; i <= 90; i += 1) {
              S3.write(i);          
              delay(15);                   
        }
        break;
 
    case 4:  // phase shift has no min max, its circular
            if(servo_delay >= 4){
                servo_delay--;
                Serial1.println("Faster speed. Current servo delay:" + String(servo_delay));
            }
         break;
    case 5:  // phase shift has no min max, its circular
           servo_delay++;
           Serial1.println("Slower speed. Current servo delay:" + String(servo_delay));
        break;
    case 7:   // start stop
            Serial1.println("Stopped");
            while(int(Serial1.read())!= 7){
                if(int(Serial1.read()) == 9){
                    print_Q();
                } else if(int(Serial1.read()) == 10){
                    print_info();
                }
                while(Serial1.read() >= 0);
             }
            Serial1.println("Starting");
    
        break;
    case 8:    // reset QTable
            Serial1.println("Resetting QTable");
            init_Q_table();
            break;
    case 9:   // print Qtable
        print_Q();
        break; 
    
     case 10:   // info
        print_info();
        break; 
    }
          while(Serial1.read() >= 0); //flush buffer
        }

      


}
    




void print_info(){
    Serial1.println("INFO:");
    Serial1.println("state space:" + String(state_space));
    Serial1.println("Angle deltas");

     // calculate max amplitude and angle delta for sinus states.
    for (i = 0 ; i < 2 ; i ++ ){//servo min    + servo max
        Serial1.println("delta:" + String(angle_delta[i]) + "  S high:" + String(state_space_high[i]));
    }


    Serial1.println("Sine state table");
    for(i = 0 ; i < state_size; i++){
        for (j = 0 ; j < state_types; j++ ){   //      distance
            Serial1.print(sine_states[i][j]);
            Serial1.print(" | ");
        }
        Serial1.println();
    }

      Serial1.print("Random Seed: ");
      Serial1.println(abs(rnd_seed));
      Serial1.print("Random initial state: " );
      Serial1.println(gw2state[gw_phase][gwS1][gwS2]);
      Serial1.println("gw_phase: " + String(gw_phase) +" | gwS1:" + String(gwS1) + " | gwS2:"+ String(gwS2) );
      Serial1.println(gw2state[gw_phase][gwS1][gwS2]);


        Serial1.println("INFO END");
        while(Serial1.read() >= 0);
  }


  
void init_Q_table() {
  
 for (i = 0 ; i < state_space ; i++){
            for (j = 0 ; j < actions ; j++ ){
                Q[i][j] = 0;
            }
         }
}



void print_Q() {
  
    for (i = 0 ; i < state_space ; i++){
        for (j = 0 ; j < actions ; j++ ){
            Serial1.println("Q["+String(i)+"]["+String(j)+"] = " +   String(Q[i][j]));
        }
    }
}
