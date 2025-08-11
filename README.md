[![Build](https://github.com/cLarson-asbHum/ftc-summer-25/actions/workflows/gradle-build.yaml/badge.svg?branch=test-double-controller)](https://github.com/cLarson-asbHum/ftc-summer-25/actions/workflows/gradle-build.yaml)
[![Unit Tests](https://github.com/cLarson-asbHum/ftc-summer-25/actions/workflows/gradle-test.yaml/badge.svg?branch=test-double-controller)](https://github.com/cLarson-asbHum/ftc-summer-25/actions/workflows/gradle-test.yaml)

# C. Larson Summer 2025 Testing Repo

Hi! This repo probably *isn't* what you're looking for, as this is a repo 
practicing *my* RoadRunner and JUnit 5 abilities. Unless you find that 
interesting or useful, please, search elsewhere.

## `test-double-controller`

This branch pertains to the creation and testing of test doubles of common 
classes in `com.qualcomm.robotcore.hardware`, but instead of directly
implementing functionality in the classes themselves, the controllers are
implemented, which handle the logic. The original RobotCore implementations of
hardware then use such controllers correctly. The currently implemented classes
can be seen below in the "HardwareFaker Subproject" section.

## `HardwareFaker` Subproject

This subproject contains several test doubles of `com.qualcomm.robotcore` 
devices common to FTC projects. Currently implemented Test doubles include 
the following:

 * DcMotorControllerEx
 * DcMotorImplEx
 * ServoControllerEx
 * CRServoImplEx
 * ServoImplEx

Along with these, the following classes are planned to be implemented:

 * ColorRangeSensor
 * DistanceSensor
 * Gamepad
 * IMU
 * Telemetry
 * TouchSensor

### Intallation

1. Clone this repository **or** manually copy the HardwareFaker directory 
   into your project

    * If you are copying the HardwareFaker directory manually, add the 
       following line to the top-level `settings.gradle`:

```gradle
include ":HardwareFaker" // Or whatever you rename the directory to.
```

2. Include the following code in your project's `build.gradle` (most commonly in 
   the `:TeamCode` subproject):

```gradle
dependencies {
    // ... other statements

    implementation project(':HardwareFaker') // Or whatever you rename the directory to.

    // ...
}
```

3. Import the desired test double. All fakes are located in the 
   `clarson.ftc.faker` package and have the class name formula of 
   `${HardwareDevice}Fake`. For example, to import the test double for 
   `DcMotorEx`, place the following import statement at the beginning of your 
   code:

```java
import clarson.ftc.faker.DcMotorExFake;
```

4. Construct the fake. `DcMotorExFake` is constructed with an RPM and number of 
   ticks per revolution, in that order. Specifics for other constructors vary.

5. Use the fake wherever you would use the ordinary object! All fakes implement 
   the corresponding interfaces; `DcMotorExFake` implements the `DcMotorEx` 
   interface from `com.qualcomm.robotcore.hardware`, and so the following code 
   is completely valid:

```java
DcMotorImplEx motor = new DcMotorImplExFake(312, 576.6);
motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
motor.getMode(); // Returns DcMotorEx.RunMode.RUN_USING_ENCODER

motor.setPower(0.33); // The motor's current position will begin to move
motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); // Resets getCurrentPosition()
```

**NOTE:** If you wanted the test double of `DcMotor`, you would simply use 
the `DcMotorExFake` constructor as normal but use the field/variable as 
`DcMotor`. In other words, line 1 in the previous example becomes

```java
DcMotor motor = new DcMotorImplExFake(312, 576.6);
// Previously was "DcMotorEx motor = new DcMotorExFake(312, 576.6)" 
```

This also applies for other super-interfaces, such as `Servo` and `CRServo`; use the fake with "Ex" in its name. For `Servo`, use `ServoImplExFake`, and with `CRServo` use `CRServoImplExFake`.  

### Known Issues

 * Calls to Lynx-module issuing methods such as `setPower()` do not simulate the
   delay of such operation. This, in turn, means opmode devices are not being
   updated

 * `ServoControllerExFake.setServoType()` is provided for compatability and not
    does not affect how the servo interacts

 * `DcMotorControllerExFake.setMotorType()` only respects the `orientation` field
   on the given configuration, but changing this from CW to CCW is untested and 
   expected to cause unintended behavior. 

 * All actuators assume acceleration and friction are negligible. This is most
   notable with `DcMotorExFake.setVelocity()`, which instantly changes the
   velocity.

 * All PID(F) coefficient-setting methods in `DcMotorExFake` do nothing as neither
  `RUN_TO_POSITION` nor `RUN_USING_ENCODER` are implemented with PID(F)
   controllers.
 
 * Framing length in `PwmController.PwmRange` objects is ignored for all servo
   fakes, continuous or positional.
