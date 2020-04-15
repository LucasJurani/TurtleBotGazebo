# TurtleBotGazebo
A Star Algorithm linked to PurePursuit and Monte Carlo Localization
First of all, We used a A* algorithm and linked it to the PurePursuit algorithm from the MatLab. None of them we fully implemented.
A important thing is that the A* algorithm has output called OptimalPath. The Optimal path return the X and Y that the Turtlebot must follow.
But, on the A* output, the X and Y axes are inverted. Because of that, we created a function called RetornadorXY(). We took the output of the A*, and pasted it on the Google SpreadSheet, and splitted the X and Y axis and inverted it to the right format, and copy and paste to the function called RetornardorXY().
Still Yet, we had to make some changes on the coordinates of the A* algorithm. Since I noticed that our Gazebo map was'nt on the same dimensions of my OccupancyGrid map.
I hope this code helps you somehow.
Thanks.
