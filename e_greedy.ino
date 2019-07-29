
int e_greedy(float &e_greed, float e_greed_final, float e_greed_step, float Q[4])
{
   int action = 0;
   int n, i = 0;
   int similar_max[4] = {0, 0, 0, 0};
   float max_q = 0;

   
    //Pick action: e-Greedy:
      if (e_greed <= (e_greed_final - e_greed_step)) //Increase e_greedy value ofer time to exploit more after some learning
      {
      e_greed = e_greed + e_greed_step;
      }
      else {e_greed = e_greed_final;}
      DEBUG_PRINT("e-Greed: "); DEBUG_PRINTLN(e_greed);
          
      //Exploration
      if (random(0, 100) > e_greed)
      { 
      action = random(0, 4);
      DEBUG_PRINTLN("e-Greedy: Random Choice");
      }
      else //Exploitation
      { //find highest q value in current state
      max_q = Q[0];     //Initially guess that the first action (index 0) is has the highest Q-val
      for (i=1; i<4; i++) 
        {
        if (Q[i] > max_q) //TODO handle decision between two actions with same Q values
          {
          max_q = Q[i];
          action = i;
          }        
        } DEBUG_PRINTLN("e-Greedy: Highest Q-Choice"); DEBUG_PRINT("MAX Q: "); DEBUG_PRINTLN(max_q);
      n=0;
      for (i=0; i<4; i++)
        {
        if(Q[i] == max_q)
          {
          DEBUG_PRINT(i); DEBUG_PRINT(", "); 
          similar_max[n] = i; 
          n++;
          }
        }  
        
      if(n >= 2)
        { 
          DEBUG_PRINT("<- have equal high Q-values"); 
          action = similar_max[random(0, n)];
          DEBUG_PRINTLN("e-Greedy: Random action choice ot of multiple equal highest Q-Values");
        } //if all q values in current state are zero, pick a random action
      }
    DEBUG_PRINT("Action chosen: ");
    DEBUG_PRINTLN(action);
    

return action;
}
      
