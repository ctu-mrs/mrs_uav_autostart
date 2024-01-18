# MRS UAV Autostart

## General description
The ROS package for automating takeoff routine for UAVs. The **Autostart** checks the availability of essential parts of the system and validity of received data. Once all modules and data are available, it waits for the UAV to be armed and switched to offboard mode. Then, after a __safety timeout__, it initiates the takeoff procedure. During the __safety timeout__ period, takeoff can be aborted by switching the mode back to manual or by disarming the UAV.

The **Autostart** performs following checks:

* availability of [UAV Manager](https://github.com/ctu-mrs/mrs_uav_managers#uavmanager),
* availability of [Control Manager](https://github.com/ctu-mrs/mrs_uav_managers#controlmanager),
* availability of [Estimation Manager](https://github.com/ctu-mrs/mrs_uav_managers#estimationmanager),
* connection to [HW Api](https://github.com/ctu-mrs/mrs_uav_hw_api#mrs-uav-hw-api-docs),
* validity of current position of the UAV (takeoff outside safety area is not allowed),
* limit on current maximum estimated speed (UAV should be static before takeoff),
* the UAV height (if available),
* availability of data on additional user-specified topics.

## Configuration and use

### Custom topics check

The **Autostart** node allows the user to specify additional topics that have to be available before initiating takeoff procedure. If this check is enabled and the latest message received on one of the specified topics is older than __timeout__, the takeoff of the UAV will not be allowed.  

```yaml
preflight_check:
  topic_check:
    enabled: true
    timeout: 5.0 # [s]
    topics: [
      "estimation_manager/uav_state",
      "/mission_controller/diagnostics"
    ]
```

## Dependencies

* [mrs_lib](https://github.com/ctu-mrs/mrs_lib)
* [mrs_msgs](https://github.com/ctu-mrs/mrs_msgs)
