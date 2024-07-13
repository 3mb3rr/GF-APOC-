## Welcome!
This is the Jayveer Pathing path following program developed by FTC team 19013 G-Force with his kids 
in the 2023-2024 Centerstage season.

## Installation
The quickest way to get started is with the quickstart [here](https://github.com/3mb3rr/GF-APOC-).

Otherwise, take the `jayveerPathing` folder and put it under the `teamcode` folder in your project.
You can do this from either downloading the project from the above quickstart link or the 19013
CENTERSTAGE repository [here](https://github.com/3mb3rr/GF-APOC-).

For this version of Jayveer Pathing, the localizer used is the Road Runner localizer. To install its
dependencies:
1. Find `build.dependencies.gradle` in the main folder of your project.
2. Add the following code to the end of the `repositories` block:
```
maven { url = 'https://maven.brott.dev/' }
```
3. Then, add the following code to the end of your `dependencies` block:
```
implementation 'com.acmerobotics.dashboard:dashboard:0.4.5'
``` 
4. Find the `build.gradle` file under the `teamcode` folder.
5. In this gradle file, add the following dependency:
```
implementation 'com.fasterxml/jackson.core:jacson-databind:2.12.7'
implementation 'org.jetbrains.kotlin:kotlin-stdlib:1.4.21'
```