# DASH-Hardware-Interface

<h3>Build instruction</h3>

```
mkdir build
cd build
cmake ..
make
```
<h3> Running closed loop position control example</h3>

The closed-loop position control is based on the article [Controlling ODrive from an Arduino via CAN](https://docs.odriverobotics.com/v/latest/guides/arduino-can-guide.html).

A few things need to be in order:
1. Set static IP for UPXtreme and Teensy
* UPXtreme IP: 10.176.32.14
* Teensy IP: 10.176.32.33

2. Follow the [Configuring the ODrive](https://docs.odriverobotics.com/v/latest/guides/arduino-can-guide.html#configuring-the-odrive) steps.
3. Make sure the components are wired correctly
* Ensure that the CAN bus is grounded to the common ground as advised [here](https://docs.odriverobotics.com/v/latest/guides/can-guide.html#hardware-setup)

```
cd build
./closed_loop_test
```
