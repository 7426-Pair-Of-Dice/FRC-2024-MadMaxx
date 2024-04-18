<div align="center">
    <img src="https://i.imgur.com/A0nEszi.png">
</div>

<h1 align="center">MAD MAXX 2024</h1>

<div align="center">
    <a href="https://www.firstinspires.org/robotics/frc/game-and-season"><img src="https://img.shields.io/badge/2024-CRESCENDO-00b96b?logo=first" alt="2024 - CRESCENDO"></a>
    <a href="https://www.thebluealliance.com/event/2024utwv"><img src="https://img.shields.io/badge/UTR-14--3--0-3f51b5" alt="UTR - 14-3-0"></a>
    <a href="https://www.thebluealliance.com/event/2024casf"><img src="https://img.shields.io/badge/SFR-11--5--0-c01ced" alt="SFR - 11-5-0"></a>
    <a href="https://www.thebluealliance.com/event/2024nvlv"><img src="https://img.shields.io/badge/LVR-7--4--0-ed1c24" alt="LVR - 7-4-0"></a>
    <a href="https://www.thebluealliance.com/event/2024utwv"><img src="https://img.shields.io/badge/Curie-TODO-e69d00" alt="Curie - TODO"></a>
    <a href="https://www.statbotics.io/team/7426/2024"><img width="20" height="20" src="https://www.statbotics.io/circ_favicon.ico"></a>
</div>
<br>
<div align="center">
    <img src="https://i.imgur.com/Fm0PJIk.png" width="400">
</div>

## 2024 Competition Repository

This repo contains 7426's final competition code for the 2024 FRC game Crescendo. 

MAD MAXX has been through 3 total revisions and began life as a chunky bot named "Bot Marley". This revision was not competition ready and the design had a few flaws, such as a live-axle arm, a queuing motor, and limelights mounted on our manipulator. 

These were later corrected before our first competition in Utah on our Rev 2, publically named "Mad Maxx" after Maxx Crosby. In this revision, we added a beam brake to our intake for note detection and LEDs around our frame for communicating to our drive team and flare. 

This codebase reflects the changes only on our final revision for the championship. This version adds an additional beam brake for improved intaking, a limelight 3G for better tag detection, more LEDs and the switch from L1 to L2 gearing on our swerve modules.

Additional changes will be made to clean up the code to serve as a resource for 


## Subsystem Specifications
| Subsystem 	| Role       	| Type  	| Controller 	| Additional Sensor        	| CAN ID 	|
|-----------	|------------	|--------	|------------	|--------------------------	|--------	|
| Robot     	| PDH          	| Power  	| N/A       	|                          	| 1      	|
| Swerve    	| Gyro       	| Pigeon 	| N/A       	|                           | 2      	|
| Swerve    	| FL Drive   	| Kraken 	| TalonFX    	|                          	| 3      	|
| Swerve    	| FR Drive   	| Kraken 	| TalonFX    	|                          	| 4      	|
| Swerve    	| BL Drive   	| Kraken 	| TalonFX    	|                          	| 5      	|
| Swerve    	| BR Drive   	| Kraken 	| TalonFX    	|                          	| 6      	|
| Swerve    	| FL Angle   	| Kraken 	| TalonFX    	| Cancoder (11)             | 7      	|
| Swerve    	| FR Angle   	| Kraken 	| TalonFX    	| Cancoder (12)            	| 8      	|
| Swerve    	| BL Angle   	| Kraken 	| TalonFX    	| Cancoder (13)             | 9      	|
| Swerve    	| BR Angle   	| Kraken 	| TalonFX    	| Cancoder (14)             | 10      	|
| Shooter   	| Top Shooter   | Falcon 	| TalonFX    	|                          	| 15       	|
| Shooter   	| Bottom Shooter| Falcon 	| TalonFX    	|                          	| 16       	|
| Arm       	| Top Arm      	| Falcon   	| TalonFX   	| Cancoder (21)             | 17       	|
| Arm       	| Bottom Arm   	| Falcon   	| TalonFX   	|                          	| 18       	|
| Elevator  	| Elevator A 	| Falcon   	| TalonFX   	| Cancoder (22)         	| 19       	|
| Elevator  	| Elevator B 	| Falcon   	| TalonFX   	|                        	| 20       	|
| Intake      	| Top Intake  	| Neo 550   | Sparkmax   	|                        	| 23       	|
| Intake      	| Bottom Intake	| Neo 550   | Sparkmax   	|                          	| 24       	|

