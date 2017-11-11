# eYRC2016_LM438
This repository contains our codes used for the eYantra Robotics Competition, <br>
Theme: Launch a Module | Team ID: eYRC-LM#438 | First prize winner at National Level e-Yantra Robotics Competition 2016-17 conducted by e-Yantra, CSE Department, IIT Bombay, sponsored by MHRD, Government of India.

The goal was to deposit the correct objects(foam cubes) to their respective matches(stickers place on the left having identical colour, shape, size) using a Firebird V robot while avoiding obstacles (large red squares). The only feedback that the robot has, is through the motor encoders and commands received from the computer. 

The computer is running a python program which constantly processes frames from an overhead camera to first find the optimal solution and then sends commands to the robot using XBee modules (like pickup, move, turn, etc). As the robot moves, it makes movement errors which are detected using the pink strips on the robot and commands are sent to make tiny corrections and keep it from straying away(and boy did it stray!).

We have also designed a modified system which could carry 3 objects at a time, but because of some software issues found right before the finals, we ended up not using it for the final round. 

Huge thanks to e-yantra for conducting this competition every year and providing all the resources free of cost! [http://e-yantra.org/](http://e-yantra.org/)

## Video Demonstration
[Pre Finals Demonstration Video](https://www.youtube.com/watch?v=5FSh1IrSGKg)

[Three block setup Video](https://www.youtube.com/watch?v=Qtwek7J2ddI)

## Flowchart
![](https://s19.postimg.org/ld52b9nnn/flowchart.png)

## Images

![](https://s19.postimg.org/3odbj5br7/raw.png)

![](https://s19.postimg.org/o8i5hmc2r/final1.png)

![](https://s19.postimg.org/we07frslv/pos8.jpg)

![](https://s18.postimg.org/wcna80fg9/bonus_thumb.jpg)

![](https://s18.postimg.org/b2znx6u0p/final_thumb.jpg)
