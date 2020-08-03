
int e_greedy(float &e_greed, float e_greed_final, float e_greed_step, float Q[7], bool is_invalid[7]){
    
    int action = 0;
    int similar_max[7] = {0, 0, 0, 0, 0, 0 ,0};
    float max_q = 0;
    
    //Pick action: e-Greedy:
    e_greed = (e_greed <= (e_greed_final - e_greed_step)) ? e_greed + e_greed_step : e_greed_final; //Increase e_greedy value too offer time to exploit more after some learning
 
    //Exploration
    if (random(0, 100) > e_greed) {
       do { action = random(7);
       } while(is_invalid[action]); // chose random valid action.
    } else { 
    //Exploitation 
        max_q = Q[0];     // find highest q value in current state, action 0 always valid
        for (int i = 1 ; i < 7 ; i++){
            if(!invalid_actions[i] && Q[i] > max_q){ 
                max_q = Q[i];
                action = i;
            }        
        } /*DEBUG_PRINTLN("e-Greedy: Highest Q-Choice"); DEBUG_PRINT("MAX Q: "); DEBUG_PRINTLN(max_q);*/
        
        // check and handle similar max_q values
        for (int i = 0, n = 0; i < 7 ; i++) {
            if(!invalid_actions[i] && Q[i] == max_q){
                similar_max[n] = i; 
                n++;
            }
        }
        if(n >= 2){ 
            action = similar_max[random(0, n)];
          } 
      }
      
    return action;
}
      
