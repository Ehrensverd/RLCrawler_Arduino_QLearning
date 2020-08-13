# q-learning-sine-crawler

q-learning-sine-crawler was created during an internship at Førde Dynamics during the summer of 2020. 

This project is an alternative implementation of [RLCrawler_Arduino_QLearning](https://github.com/frdedynamics/RLCrawler_Arduino_QLearning). 


## How it works
Initially the crawler knows nothing of its environment nor how to move. 
It will learn how to move through experience from its behaviour and environment.

This version replaces the discrete servo position states of the original implementation with sine amplitude and phase shift states. The servo joints oscillate continuously about their midpoints with their extent of travel controlled individually by amplitude state variables. While the joints share a fixed oscillation frequency, a final state variable controls their relative phase shift, thus allowing a diverse range of movement patterns to be achieved.
### Bluetoith Terminal
TODO insert gif
### Bluetooth plotter
TODO insert gif4
## Acknowledgments
Consept by Daniel Schäle.


