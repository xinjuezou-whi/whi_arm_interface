# whi_arm_interface

arm hardware interface

## Supported arms
| Brand   | Seriels | Test     |
|---------|---------|----------|
| AR      | ar3     | not yet  |
| Chin    | All     | not yet  |
| JAKA    | All     | Passed   |
| Fairino | All     | not yet  |

## Dependency

```
git clone https://github.com/xinjuezou-whi/whi_interfaces.git
```

And other third parties:
* sockpp, refer to [its repository](https://github.com/fpagliughi/sockpp)
* CRC++, refer to [its repository](https://github.com/d-bahr/CRCpp)
* jsoncpp, refer to [its repository](https://github.com/open-source-parsers/jsoncpp)

## Advertise services
**arm_io**(whi_interfaces::WhiSrvIo)
Offers the IO setting service, here is an example of a client's request:
```
rosservice call /whi_arm_interface/arm_io "{addr: 1, operation: 1, level: 1}"
```

**arm_ready**(std_srvs::Trigger)
Offers the query of whether the arm is on standby for motion execution, here is an example of a client's request:
```
rosservice call /whi_arm_interface/arm_ready
```

## Publish topics
**arm_motion_state**(whi_interfaces::WhiMotionState)
Publishes the state of arm periodically

## Usage
First, it requires the moveit_config packages that WHI refactors:

moveit_config packages for [AR series](https://github.com/xinjuezou-whi/ar_arm.git), [Chin series](https://github.com/xinjuezou-whi/chin_arm), [JAKA series](https://github.com/xinjuezou-whi/jaka_robot), and [FAIR series](https://github.com/xinjuezou-whi/frcobot_ros)

Then, launch the node with the specified arm brand and its model:
| Brand   | Seriels |
|---------|---------|
| ar      | ar3     |
| chin    | crb7    |
| jaka    | zu5, a5 |
| fr      | 5v6     |

Take the JAKA a5 as an example:
```
ros2 launch whi_arm_interface launch.py arm:=jaka arm_model:=a5
```

## Limited
1. For FAIR series, collision check and recovery have not been implemented yet due to absence of protocol
2. xxx

## Permission for /dev/ttyama0

For the AR arm which takes the serial port to communicate, the following command can be used to grant the serial privilege:
```
sudo usermod -a -G dialout <user name>
```
Then reboot
