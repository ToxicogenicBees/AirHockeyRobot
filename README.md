# Air Hockey Robot

Control for the movement of a robotic air hockey table.


## About the Project

All movements of the mechanical system designed for a robotic air hockey table are controlled through this program. This is accomplished through multiple threads handling different processes. The two main sections consist of laptop and microcontroller code. The laptop code handles complex processing, including vision processing to track the motion of the puck and routines that handle calculations to both predict where the puck will be moving and how to respond. The microcontroller code focuses on the setup and communication with each device used to assist with the mechanical system's movement.
## Requirements

* A simulator is included to test routines without needing an entire mechanical system built.
* To fully run the program, a camera and mechanical system with similar or matching devices is required.
## How to Run

* Download main.exe
* Plug in a compatible camera and microcontroller to the laptop
* Identify the COM port of the microcontroller
* Run main.exe
* Input tracking mode and COM port (0-12)
    * Tracking modes:
        * yellow - Track for yellow objects
        * green - Track for green objects
        * phys - Only run simulation
        * none - No operation
## Authors

- [Kaden Carpenter, @KadenCarp64](https://www.github.com/KadenCarp64)
- [William Forcey, @wawesomeNOGUI](https://www.github.com/wawesomeNOGUI)
- [Andrew Piunno, @ToxicogenicBees](https://www.github.com/ToxicogenicBees)
- [Xander Zavatchen, @maxxrz31](https://www.github.com/maxxrz31)
