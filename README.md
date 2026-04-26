# Air Hockey Robot

Control for movement of a robotic air hockey table.


## About the Project

All movements of the mechanical system designed for a robotic air hockey table is controlled through this program. This is accomplished through multiple threads handling different processes. The two main sections consist of laptop and microcontroller code. The laptop code handles complex processing including vision processing to track the motion of the puck and routines which handle calculations to both predict where the puck will be moving and how to respond. The microcontroller code focuses on the setup and communication with each device used to assist with the mechanical systems movement.
## Requirements

* A simulator is included to test routines without needing an entire mechanical system built.
* To fully run the program a camera and mechanical system with similar or matching devices is required.
## How to Run

* Download main.exe
* Plug in a compatible camera and microcontroller to the laptop
* Identify comm port of the microcontroller
* Run main.exe
* Input tracking mode, select desired difficulty/routine, and comm port 
    * Tracking modes:
        * yellow - Track for yellow objects
        * green - Track for green objects
        * phys - Only run simulation
        * none - No operation
    * Difficulties:
        * 0 - Basic motion test
        * 1 - Avoid puck's location
        * 2 - Basic defense
        * 3 - Basic offense 
        * 4 - Advanced defense
        * 5 - Advanced offense
        * manual - Keyboard input to move robot
        * none - No operation

## Authors

- [Andrew Piunno, @ToxicogenicBees](https://www.github.com/ToxicogenicBees)
- [Kaden Carpenter, @KadenCarp64](https://www.github.com/KadenCarp64)
- [William Forcey, @wawesomeNOGUI](https://www.github.com/wawesomeNOGUI)
- [Xander Zavatchen, @maxxrz31](https://www.github.com/maxxrz31)
