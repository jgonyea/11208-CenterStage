# Kardia CenterStage Robot Config
This file contains the configuration from our robot's control hub. In case it is accidentally deleted, we can manually restore it from here.

### Expansion Hub 3

**Motors**

| Port | Attached | Name |
| :-: | :- | :- |
| 0 | GoBILDA 5202/3/4 series | `driveRR` |
| 1 | GoBILDA 5202/3/4 series | `liftL` |
| 2 | GoBILDA 5202/3/4 series | `liftR` |

**Servos**

| Port | Attached | Name |
| :-: | :- | :- |
| 0 | Servo | `launcher` |
| 1 | Servo | `hand` |
| 2 | Servo | `wrist` |
| 3 | Servo | `pincerL` |
| 4 | Servo | `pincerR` |

### Control Hub

**Motors**

| Port | Attached | Name |
| :-: | :- | :- |
| 0 | GoBILDA 5202/3/4 series | `driveFL` |
| 1 | GoBILDA 5202/3/4 series | `driveFR` |
| 2 | GoBILDA 5202/3/4 series | `driveRL` |

**Servos**

| Port | Attached | Name |
| :-: | :- | :- |
| 0 | Servo | `armL` |
| 1 | Servo | `armR` |
| 4 | Servo | `frontpL` |
| 5 | Servo | `frontpR` |

**Digital Devices**

| Port | Attached | Name |
| :-: | :- | :- |
| 4 | Digital Device | `parkSwitchI` |
| 5 | Digital Device | `parkSwitchII` |
| 6 | Digital Device | `switch2` |
| 7 | Digital Device | `switch3` |

**I2C Bus 0**

| Port | Attached | Name |
| :-: | :- | :- |
| 0 | REV internal IMU (BHI260AP) | `imu` |

**I2C Bus 1**

| Port | Attached | Name |
| :-: | :- | :- |
| 0 | REV 2M Distance Sensor | `distL` |

**I2C Bus 2**

| Port | Attached | Name |
| :-: | :- | :- |
| 0 | REV 2M Distance Sensor | `distR` |
