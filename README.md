[![Build](https://github.com/cLarson-asbHum/ftc-summer-25/actions/workflows/gradle-build.yaml/badge.svg?branch=test-double)](https://github.com/cLarson-asbHum/ftc-summer-25/actions/workflows/gradle-build.yaml)
[![Unit Tests](https://github.com/cLarson-asbHum/ftc-summer-25/actions/workflows/gradle-test.yaml/badge.svg?branch=test-double)](https://github.com/cLarson-asbHum/ftc-summer-25/actions/workflows/gradle-test.yaml)

# C. Larson Summer 2025 Testing Repo

Hi! This repo probably *isn't* what you're looking for, as this is a repo 
practicing *my* RoadRunner and JUnit 5 abilities. Unless you find that 
interesting or useful, please, search elsewhere.

## `test-double`

This branch pertains to the creation and testing of test doubles of common classes in `com.qualcomm.robotcore.hardware`. The currently implemented classes can be seen below in the "HardwareFaker Subproject" section.

## `HardwareFaker` Subproject

This subproject contains several test doubles of `com.qualcomm.robotcore` devices common to FTC projects. Currently implemented Test doubles include the following:

 * DcMotorEx (and so DcMotor)

Along with these, the following hardware devices are planned to be implemented:

 * ColorRangeSensor
 * CRServo
 * DistanceSensor
 * IMU
 * Servo
 * TouchSensor

### Usage

1. Clone this repository **or** manually copy the HardwareFaker directory into your project

    i. If you are copying the HardwareFaker directory manually, add the following line to the top-level (i.e. not inside TeamCode or FtcRobotController) `settings.gradle`:

```gradle
include ":HardwareFaker" // Or whatever you rename it to.
```

2. Include the following code in your project's `build.gradle` (most commonly in the `:TeamCode` subproject):

```gradle
dependencies {
    // ... other statements

    implementation project(':HardwareFaker')

    // ...
}
```

3. Import the desired test double. All fakes are located in the `clarson.ftc.faker` package and have the class name formula of `${HardwareDevice}Fake`. For example, to import the test double for `DcMotorEx`, place the following import statement at thte beginning of your code:

```java
import clarson.ftc.faker.DcMotorExFake;
```

4. Construct the fake. `DcMotorExFake` is constructed with an RPM and number of ticks per revolution, in that order. Specifics for other constructors varies.

5. Use the fake wherever you would use the ordinary object! All fakes implement the corresponding interfaces; `DcMotorExFake` implements the `DcMotorEx` interface from `com.qualcomm.robotcore.hardware`.

```java
DcMotorEx motor = new DcMotorExFake(312, 576.6);
motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
motor.getMode(); // Gets DcMotorEx.RunMode.RUN_USING_ENCODER

motor.setPower(0.33); // The motor's current position will begin to move
motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
```

Note that if you wanted the test double of `DcMotor`, you would simply use the `DcMotorExFake` constructor but type the field/variable as `DcMotor`. In other words, line 1 in the previous example becomes

```java
DcMotor motor = new DcMotorExFake(312, 576.6);
// Previously was "DcMotorEx motor = DcMotorEx(312, 576.6)" 
```

### Known Issues
TODO: fill in with the bugs/caveats that I made.