# locator_ros_bridge

This repository contains the [bosch_locator_bridge](bosch_locator_bridge) package, which provides a [ROS] interface to the [Bosch Rexroth Locator Software].
It translates ROS messages to the Locator API (as described in the Locator API documentation) and vice versa.
It also allows to control the Locator software via ROS service calls.

There are versions for ROS 1 and ROS 2:
* [ROS 1 Noetic](../../tree/noetic)
* ROS 2 Foxy (this repository)

## Installation

### Building from Source

#### Dependencies

- [ROS]
- [Poco] C++ library (Should be installed automatically with rosdep, otherwise try: ```sudo apt install libpoco-dev```)

#### Building

To build from source, make sure your colcon workspace is set up correctly. Then clone the latest version from this repository into your colcon workspace and compile the package using

    cd colcon_ws
    rosdep install --from-paths . --ignore-src
    colcon build --symlink-install

## How to Get Started

To get started, take a look at the [README.md](bosch_locator_bridge/README.md) of the bosch_locator_bridge package.

## License

locator_ros_bridge is open-sourced under the Apache-2.0 license. See the [LICENSE](LICENSE) file for details.


[ROS]: https://www.ros.org/
[Poco]: https://pocoproject.org/
[Bosch Rexroth Locator Software]: https://www.boschrexroth.com/en/xc/products/product-groups/components-for-mobile-robotics/index
