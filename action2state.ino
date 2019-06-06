int action2state(int &state1, int &state2, int n_states[2], int action) //n_states is the number of states for each servo
{
  int state[2] = {state1, state2};
  int newstate[2] = {0, 0};
  byte i = 0;
  
  for (i=0; i < 2; i++)
    {newstate[i] = state[i];}
    
  if ((state[0] == 1 && action == 0)||(state[0] == n_states[0] && action == 1)) //actions up=1; down=2;
    {for (i=0; i < 2; i++)
    {newstate[i] = state[i];}}
  else if ((state[1] == 1 && action == 2)||(state[1] == n_states[1] && action == 3)) //actions backwards=3; forwards=4;
    {for (i=0; i < 2; i++)
    {newstate[i] = state[i];}}
  else if (action == 0)
    {newstate[0] = state[0] + 1;}
  else if (action == 1)
    {newstate[0] = state[0] - 1;}
  else if (action == 2)
    {newstate[1] = state[1] - 1;}
  else if (action == 3)
    {newstate[1] = state[1] + 1;}
  else {for (i=0; i < 2; i++)
    {newstate[i] = state[i];}}

  //return newstate;
} 
