void action2state(int gwS1, int gwS2, int& newgwS1, int& newgwS2, int n_states[2], int action) //n_states is the number of states for each servo
{
   
if (action == 0)
    {
      //Serial1.print("A0 ");
    if ((gwS1 <= 0) && (action == 0))
      {
      newgwS1 = gwS1;
      newgwS2 = gwS2;
      }
    else 
      {
      newgwS1 = gwS1 - 1;
      newgwS2 = gwS2;
      }
    }
    
   if (action == 1)
    { //Serial1.print("A1 ");
    if ((gwS1 >= n_states[0]-1) && (action == 1))
      {
      newgwS1 = gwS1;
      newgwS2 = gwS2;
      }
    else 
      {
      newgwS1 = gwS1 + 1;
      newgwS2 = gwS2;
      }
    }

   if (action == 2)
    { //Serial1.print("A2 ");
    if ((gwS2 <= 0) && (action == 2))
      {
      newgwS1 = gwS1;
      newgwS2 = gwS2;
      }
    else 
      {
      newgwS1 = gwS1;
      newgwS2 = gwS2 - 1;
      }
    }

   if (action == 3)
    {// Serial1.print("A3 ");
    if ((gwS2 >= n_states[1]-1)) //&& (action == 3))
      {
      newgwS1 = gwS1;
      newgwS2 = gwS2;
      }
    else
      { 
      newgwS1 = gwS1;  
      newgwS2 = gwS2 + 1;
      }
    }
} 
