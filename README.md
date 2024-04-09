# 2024-Bot-Marley

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