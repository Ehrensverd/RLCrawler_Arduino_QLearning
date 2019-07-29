//Epsilon greedy
int e_greedy(float &e_greed, float e_greed_final, float e_greed_step, float Q[4]);

//Delta: Mapping from actions to states
void action2state(int gwS1, int gwS2, int& newgwS1, int& newgwS2, int n_states[2], int action);

#ifdef DEBUG
#define DEBUG_PRINT(x)    Serial1.print(x)
#define DEBUG_PRINTLN(x)  Serial1.println(x)
#define DEBUG_DELAY(x)    delay(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_DELAY(x) 
#endif