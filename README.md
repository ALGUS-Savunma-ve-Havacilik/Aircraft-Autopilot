#A Simple Aircraft Autopilot implemented by a Particle Filter using Octave/Matlab.
To run the code simply pass the "map.txt" to runpAutopilot function:
```matlab
runAutopilot('./Maps/map1.txt',[0 0 0 0 0]' ,100 ,1) ;
```
As for creating Maps, have a look on the "map1.txt" to get the format of the file. Basically it has the coordinates of the landmarks (position and altitude).