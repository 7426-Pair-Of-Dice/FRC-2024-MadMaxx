﻿<div align="center">
    <img src="https://i.imgur.com/A0nEszi.png">
</div>

<h1 align="center">MAD MAXX 2024</h1>

<div align="center">
    <a href="https://firstfrc.blob.core.windows.net/frc2024/Manual/2024GameManual.pdf" target="_blank"><img src="https://img.shields.io/badge/2024-CRESCENDO-00b96b?logo=first" alt="2024 - CRESCENDO"></a>
    <a href="https://www.thebluealliance.com/event/2024utwv" target="_blank"><img src="https://img.shields.io/badge/UTR-14--3--0-3f51b5" alt="UTR - 14-3-0"></a>
    <a href="https://www.thebluealliance.com/event/2024casf" target="_blank"><img src="https://img.shields.io/badge/SFR-11--5--0-c01ced" alt="SFR - 11-5-0"></a>
    <a href="https://www.thebluealliance.com/event/2024nvlv" target="_blank"><img src="https://img.shields.io/badge/LVR-7--4--0-ed1c24" alt="LVR - 7-4-0"></a>
    <a href="https://www.thebluealliance.com/event/2024cur" target="_blank"><img src="https://img.shields.io/badge/Curie-5--5--0-e69d00" alt="Curie - 5-5-0"></a>
    <a href="https://www.statbotics.io/team/7426/2024" target="_blank"><img width="20" height="20" src="https://www.statbotics.io/circ_favicon.ico"></a>
</div>
<br>
<div align="center">
    <img src="https://i.imgur.com/Fm0PJIk.png" width="400">
</div>

## 2024 Competition Repository

This repo contains 7426's final competition code for the 2024 FRC game Crescendo. 

### Overview

- ⚡&nbsp;Command-Based Framework
- 🟢&nbsp;CTRE Swerve Template Base
- ♾️&nbsp;Pathplanner Autonomous
- 👁️&nbsp;Limelight Vision
- ✨&nbsp;TalonFX and CANSparkMax Contollers

### Subsystems

**Swerve** - Uses four Falcons for angle motors, and four Krakens for drive motors. Utilizes the [CTRE Swerve Project Generator](https://pro.docs.ctr-electronics.com/en/latest/docs/tuner/tuner-swerve/index.html) as a template.

**Arm** - Runs off of two Kraken motors and uses a CANCoder for position, used to articulate the manipulator 90+ degrees.

**Elevator** - Runs off of two Falcon motors and uses a CANCoder for position, used to extend the manipulator.

**Intake** - Runs off of two NEO 550 motors and two beam brakes, used to bring a note into our robot and queue it to a predefined position.

**Shooter** - Runs off of two Falcon motors, used to shoot a note at a set velocity.

**Climber** - Runs off of one Kraken motor, used to climb onto the chain during endgame.


## About MAD MAXX

MAD MAXX has been through 3 total revisions and began life as a chunky bot named "Bot Marley". This revision was not competition ready and the design had a few flaws, such as a live-axle arm, a queuing motor, and limelights mounted on our manipulator. 

These were later corrected before our first competition in Utah on our Rev 2, publically named "Mad Maxx" after Maxx Crosby. In this revision, we added a beam brake to our intake for note detection, and LEDs on our frame to communicate with our drivers.

Our latest and final revision once again rebuilt the entire robot from scratch, featuring an additional beam brake for improved intaking, a Limelight 3G for auto-aim and auto-centering, more LEDs for better visiblilty and flare, and the switch to L2 gearing on our swerve modules for faster speeds.

## Credits

This years code wouldn't have been possible without the combined efforts of our programmers, mentors, and other external help. Special thanks to [Mechanical Advantage](https://github.com/Mechanical-Advantage) and others teams on [The Open Alliance](https://www.theopenalliance.com/teams/2024/) for making their code public as a resource.

<div align="center">

| [<img src="https://avatars.githubusercontent.com/u/32149826?v=4" width="75px;"/><br /><sub>Ohlin Arellano</sub>](https://github.com/murphy28)<br /> <sub>Lead Programmer</sub>|  [<img src="https://avatars.githubusercontent.com/u/149628585?v=4" width="75px;"/><br /><sub>West Porter</sub>](https://github.com/westernGui)<br /> <sub>Apprentice</sub>|  [<img src="https://avatars.githubusercontent.com/u/95384605?v=4" width="75px;"/><br /><sub>Nathan Arciniega</sub>](https://github.com/devtriangulumketchup)<br /> <sub>Apprentice</sub>|
| :---: | :---: | :---: |

</div>
