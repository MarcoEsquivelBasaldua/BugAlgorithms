# BugAlgorithms (in progress)

In the problem of Navigation, we are interested in finding a collision-free motion of the robot to move from one state to another. The Bug1 and Bug2 algorithms are sensor-based planners which assume the robot as a point operating in the plane provided with a contact sensor to detect obstacles. Tangt Bug is an improvement to the Bug2 algorithm in that it determines the shortest path to the goal using a range sensor with a 360 degree infinite orientation resolution.

In this implementation, the robot is not treated as a point but as a disk with constant radius. For the Tangent Bug algorithm, the range sensor has not infinite orientation resolution. And, in general, I am considering limitations based on pixel sampling on the screen. It is worth mentioning that the robot simulates only the contact sensor (for Bug1 and Bug2) and range sesnsor (for Tangent Bug) and no prior information regarding obstacle positons are provided. However, in all cases, the robot knows the exact goal position.

Simulations are performed using pygame 2.6.1.

## Bug 1

Bug1 algorithm switches between two motion behaviors: move to the goal in a straight line (motion-to-goal), and obstacle circumnavigating (boundary-following). During motion-to-goal, the robot approaches the goal until goal is reached or an obstacle is encountered. If the sensor detects an object, the boundary-following motion takes place circumnavigating the whole obstacle up to the same spot it first touched the obstacle. At every moment, during this motion, the robot measures and remembers the distance between every point in the obstacle boundary and the goal. Once the obstacle is completly sensed, the robot goes to the point closest to the goal and retakes its motion-to-goal behavior.

In this example we can see Bug1 algorithm reaching the goal.
![Bug1 goal reached](./images/bug1_goalReached.gif)

Bug1 is called a complete algorithm since it is able to find a path to the goal or tell if no path exists at all. In this case, the robot circumnavigates the obstacle it is enclosed into, but since reaching the leaving point, it can not get closer to the goal, it reports that it is impossible to get there.

![Bug1 goal not reached](./images/bug1_goalNotReached.gif)

## Bug 2

Just like Bug1 algorithm, Bug2 toggles between motion-to-goal and boundary-following but in a different way. When moving to the goal, the robot computes what is known as the \textit{m-line} which is the slope that allows arriving to the goal in a straight line. Once an obstacle is sensed, the robot circumnavigates it until the slope between its position and the goal is again the \textit{m-line}. An extra condition is required to switch again to the motion-to-goal behavior: the distance to the goal shall be shorter than when the obstacle was first encountered.
![Bug2 goal reached](./images/Bug2_goalReached.mp4.gif)

Bug2 algorithm is also able to tell whether a path to the goal is possible or not. If the robot circumnavigates completly an obstacle, it can conclude that there is not a way to get to the goal.
![Bug2 goal not reached](./images/Bug2_goalNotReached.mp4.gif)

## Tangent Bug

A robot executing the Tanget Bug algorithm is not only provided with a contact sensor but with a range sensor too. Using this range sensor, the robot is able to detect discontinuities ($O_i$) up to a distance \textit{R} from the current robot's position. Just like the other algorithms, Tangent bug iterates between motion-to-goal and boundary-following behaviors. First, the robot moves in a straight line to the goal until it senses an obstacle \textit{R} units away between it and the goal. The robot then moves to the $O_i$ that maximally decreases a heuristic distance to the goal, for instance $d(x, O_i) + d(O_i, q_{goal})$. At every step, the robot keeps updating all $O_i$ and moving to the right one. If the heuristic distance can no longer be decresed, the robot follows now the obstacle boundary.

![Tangent bug goal reached](./images/tangentBug_GoalReached.gif)

As for the Bug2 Algorithm, if an abstacle is entirely encircled, tangent Bug concludes that no path to the goal can be found.

![Tangent bug goal not reached](./images/tangentBug_GoalNotReached.gif)
