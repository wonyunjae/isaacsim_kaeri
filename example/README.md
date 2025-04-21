# [Sensor]
### d455 
```
./run_isaac.sh kaeri/isaac/sensors/d455/main.py 
```

### lidar (RangeSensorCreateLidar)
```
./run_isaac.sh kaeri/isaac/sensors/lidar/main.py 
```

### lidar (IsaacSensorCreateRtxLidar)

```
./run_isaac.sh kaeri/isaac/sensors/lidar_rtx/main.py
```

# [Ghost]
### lidar (IsaacSensorCreateRtxLidar) only
```
./run_isaac.sh kaeri/isaac/ghost/lidar_rtx/main.py 
```

### lidar + multiple d455s
```
(TBW)
```

# [Manipulator]
### 
```
./run_isaac_with_orbit.sh -p kaeri/isaac/quadruped/go1/main.py 
```

# [Quadruped]
### go1
```
./run_isaac_with_orbit.sh -p kaeri/isaac/quadruped/go1_with_sensors/main.py
```

# [Multiple Robots]
### armstrong + go1
```
./run_isaac_with_orbit.sh -p kaeri/isaac/integ_go1_armstrong/main.py
```