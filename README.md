# whi_arm_interface

arm hardware interface

## Supported arms
| Brand   | Seriels | Test     |
|---------|---------|----------|
| AR      | ar3     | Passed   |
| Chin    | All     | Passed   |
| JAKA    | All     | Passed   |
| Fairino | All     | on going |

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

## Permission for /dev/ttyama0

For the AR arm which takes the serial port to communicate, the following command can be used to grant the serial privilege:
```
sudo usermod -a -G dialout <user name>
```
Then reboot
