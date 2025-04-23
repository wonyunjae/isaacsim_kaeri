import omni.graph.core as og

def ros2_clock_publisher_go1():
    keys = og.Controller.Keys
    (clock_graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": "/ROS/ROS2_clock_publisher",
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("readSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("publishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "publishClock.inputs:execIn"),
                ("readSimTime.outputs:simulationTime", "publishClock.inputs:timeStamp"),
            ]
        },
    )

    return clock_graph

def ros2_joint_state_publisher_go1():
    keys = og.Controller.Keys
    (joint_graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": "/ROS/ROS2_joint_publisher",
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("readSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("publishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "publishJointState.inputs:execIn"),
                ("readSimTime.outputs:simulationTime", "publishJointState.inputs:timeStamp"),
            ],
            keys.SET_VALUES: [
                ("publishJointState.inputs:targetPrim", "/World/Robot/Go1"),
                ("publishJointState.inputs:nodeNamespace", "/go1"),
                ("publishJointState.inputs:topicName",  "joint_state"),
            ]
        }
    )

    return joint_graph

def ros2_joints_tf_publisher_go1(bodies):
    keys = og.Controller.Keys
    (graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": "/ROS/ROS2_joints_tf_publisher",
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("readSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("publishTransform", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "publishTransform.inputs:execIn"),
                ("readSimTime.outputs:simulationTime", "publishTransform.inputs:timeStamp"),
            ],
            keys.SET_VALUES: [
                ("publishTransform.inputs:parentPrim", bodies),
                ("publishTransform.inputs:targetPrims",  bodies),
                ("publishTransform.inputs:nodeNamespace", "/go1"),
            ]
        }
    )

    return graph

def ros2_sensors_tf_publisher_go1(): 
    keys = og.Controller.Keys 
    (graph, _, _, _) = og.Controller.edit( 
        { 
            "graph_path": "/ROS/ROS2_sensors_tf_publisher", 
            "evaluator_name": "push", 
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND, 
        }, 
        { 
            keys.CREATE_NODES: [ 
                ("OnTick", "omni.graph.action.OnTick"), 
                ("readSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"), 
                ("publishTransform", "isaacsim.ros2.bridge.ROS2PublishTransformTree"), 
            ], 
            keys.CONNECT: [ 
                ("OnTick.outputs:tick", "publishTransform.inputs:execIn"), 
                ("readSimTime.outputs:simulationTime", "publishTransform.inputs:timeStamp"), 
            ], 
            keys.SET_VALUES: [
                ("publishTransform.inputs:parentPrim", "/World/Robot/Go1/base"), 
                # ("publishTransform.inputs:targetPrims",  ["/World/Robot/Go1/base/lidar",
                #                                           "/World/Robot/Go1/base/Realsense_D455_1", 
                #                                           "/World/Robot/Go1/base/Realsense_D455_2", 
                #                                           "/World/Robot/Go1/base/Realsense_D455_3", 
                #                                           "/World/Robot/Go1/base/Realsense_D455_4"]),
            ]
        }
    )

    return graph

def ros2_foot_contact_publisher_go1():
    keys = og.Controller.Keys
    (graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": "/ROS/ROS2_foot_contact_publisher",
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("readSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),

                ("readContactFL", "omni.isaac.sensor.IsaacReadContactSensor"),
                ("readContactFR", "omni.isaac.sensor.IsaacReadContactSensor"),
                ("readContactRL", "omni.isaac.sensor.IsaacReadContactSensor"),
                ("readContactRR", "omni.isaac.sensor.IsaacReadContactSensor"),

                # ROS2 Bool 메시지 퍼블리셔 사용 - 컨택트 센서용
                ("publishContactFL", "isaacsim.ros2.bridge.ROS2PublishBool"),
                ("publishContactFR", "isaacsim.ros2.bridge.ROS2PublishBool"),
                ("publishContactRL", "isaacsim.ros2.bridge.ROS2PublishBool"),
                ("publishContactRR", "isaacsim.ros2.bridge.ROS2PublishBool"),
            ],
            keys.CONNECT: [
                # 기존 연결 유지
                ("OnTick.outputs:tick", "readContactFL.inputs:execIn"),
                ("OnTick.outputs:tick", "readContactFR.inputs:execIn"),
                ("OnTick.outputs:tick", "readContactRL.inputs:execIn"),
                ("OnTick.outputs:tick", "readContactRR.inputs:execIn"),
                
                ("OnTick.outputs:tick", "publishContactFL.inputs:execIn"),
                ("OnTick.outputs:tick", "publishContactFR.inputs:execIn"),
                ("OnTick.outputs:tick", "publishContactRL.inputs:execIn"),
                ("OnTick.outputs:tick", "publishContactRR.inputs:execIn"),

                ("readSimTime.outputs:simulationTime", "publishContactFL.inputs:timeStamp"),
                ("readSimTime.outputs:simulationTime", "publishContactFR.inputs:timeStamp"),
                ("readSimTime.outputs:simulationTime", "publishContactRL.inputs:timeStamp"),
                ("readSimTime.outputs:simulationTime", "publishContactRR.inputs:timeStamp"),

                # ROS2 Bool 메시지에는 inContact만 연결
                ("readContactFL.outputs:inContact", "publishContactFL.inputs:data"),
                ("readContactFR.outputs:inContact", "publishContactFR.inputs:data"),
                ("readContactRL.outputs:inContact", "publishContactRL.inputs:data"),
                ("readContactRR.outputs:inContact", "publishContactRR.inputs:data"),
            ],
            keys.SET_VALUES: [
                # 기존 설정 유지
                ("readContactFL.inputs:csPrim", "/World/Robot/Go1/FL_foot/sensor"),
                ("readContactFR.inputs:csPrim", "/World/Robot/Go1/FR_foot/sensor"),
                ("readContactRL.inputs:csPrim", "/World/Robot/Go1/RL_foot/sensor"),
                ("readContactRR.inputs:csPrim", "/World/Robot/Go1/RR_foot/sensor"),

                # 네임스페이스 설정
                ("publishContactFL.inputs:nodeNamespace", "/go1"),
                ("publishContactFR.inputs:nodeNamespace", "/go1"),
                ("publishContactRL.inputs:nodeNamespace", "/go1"),
                ("publishContactRR.inputs:nodeNamespace", "/go1"),

                # 토픽명 설정
                ("publishContactFL.inputs:topicName", "/FL_foot"),
                ("publishContactFR.inputs:topicName", "/FR_foot"),
                ("publishContactRL.inputs:topicName", "/RL_foot"),
                ("publishContactRR.inputs:topicName", "/RR_foot"),
            ]
        }
    )

    return graph

def ros2_odom_publisher_go1():
    keys = og.Controller.Keys
    (odom_graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": "/ROS/ROS2_odom_publisher",
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("readSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("readOdom", "omni.isaac.core_nodes.IsaacComputeOdometry"),
                ("publishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "readOdom.inputs:execIn"),
                ("OnTick.outputs:tick", "publishOdom.inputs:execIn"),
                
                ("readSimTime.outputs:simulationTime", "publishOdom.inputs:timeStamp"),

                ("readOdom.outputs:angularVelocity", "publishOdom.inputs:angularVelocity"),
                ("readOdom.outputs:linearVelocity", "publishOdom.inputs:linearVelocity"),
                ("readOdom.outputs:position", "publishOdom.inputs:position"),
                ("readOdom.outputs:orientation", "publishOdom.inputs:orientation"),
            ],
            keys.SET_VALUES: [
                ("readOdom.inputs:chassisPrim", "/World/Robot/Go1/trunk"), 

                ("publishOdom.inputs:odomFrameId",  "odom"),
                ("publishOdom.inputs:chassisFrameId",  "trunk"),
                ("publishOdom.inputs:nodeNamespace", "/go1"),
                ("publishOdom.inputs:topicName",  "odom_sim"),
            ]
        }
    )

    return odom_graph

def ros2_imu_publisher_go1():
    keys = og.Controller.Keys
    (imu_graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": "/ROS/ROS2_imu_publisher",
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("readSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("readImu", "omni.isaac.sensor.IsaacReadIMU"),
                ("publishImu", "isaacsim.ros2.bridge.ROS2PublishImu"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "publishImu.inputs:execIn"),
                ("OnTick.outputs:tick", "readImu.inputs:execIn"),

                ("readSimTime.outputs:simulationTime", "publishImu.inputs:timeStamp"),

                ("readImu.outputs:angVel", "publishImu.inputs:angularVelocity"),
                ("readImu.outputs:linAcc", "publishImu.inputs:linearAcceleration"),
                ("readImu.outputs:orientation", "publishImu.inputs:orientation")
            ],
            keys.SET_VALUES: [
                ("readImu.inputs:imuPrim", "/World/Robot/Go1/imu_link/imu_sensor"),
                ("publishImu.inputs:nodeNamespace", "/go1"),
                ("publishImu.inputs:topicName",  "imu"),
                ("publishImu.inputs:frameId",  "/imu_link"),
            ]
        }
    )

    return imu_graph

def ros2_lidar_publisher_go1(frame_id, topic_name, lidar_prim):
    keys = og.Controller.Keys
    (lidar_graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": "/ROS/ROS2_lidar_publisher",
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("readSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("readLidar", "omni.isaac.range_sensor.IsaacReadLidarPointCloud"),
                ("publishLidar", "isaacsim.ros2.bridge.ROS2PublishPointCloud"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "readLidar.inputs:execIn"),

                ("readSimTime.outputs:simulationTime", "publishLidar.inputs:timeStamp"),
                
                ("readLidar.outputs:execOut", "publishLidar.inputs:execIn"),
                ("readLidar.outputs:data", "publishLidar.inputs:data"),
            ],
            keys.SET_VALUES: [
                ("readLidar.inputs:lidarPrim", lidar_prim),

                ("publishLidar.inputs:frameId",  frame_id),
                ("publishLidar.inputs:nodeNamespace", "/go1"),
                ("publishLidar.inputs:topicName",  topic_name),
            ]
        }
    )

    return lidar_graph

def ros2_d455_publisher_go1(frame_id, namespace, topic_name, depth_prim, rgb_prim):
    keys = og.Controller.Keys
    
    (d455_graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": "/ROS/ROS2_" + frame_id + "_publisher",
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),

                ("createRenderDepth", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                ("createRenderRGB", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),

                ("publishD455_camera_info_depth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("publishD455_camera_info_rgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),

                ("publishD455_depth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("publishD455_depth_pcl", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("publishD455_rgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "createRenderDepth.inputs:execIn"),
                ("OnTick.outputs:tick", "createRenderRGB.inputs:execIn"),

                ("createRenderDepth.outputs:renderProductPath", "publishD455_camera_info_depth.inputs:renderProductPath"),
                ("createRenderDepth.outputs:renderProductPath", "publishD455_depth.inputs:renderProductPath"),
                ("createRenderDepth.outputs:renderProductPath", "publishD455_depth_pcl.inputs:renderProductPath"),

                ("createRenderRGB.outputs:renderProductPath", "publishD455_camera_info_rgb.inputs:renderProductPath"),
                ("createRenderRGB.outputs:renderProductPath", "publishD455_rgb.inputs:renderProductPath"),

                ("createRenderDepth.outputs:execOut", "publishD455_camera_info_depth.inputs:execIn"),
                ("createRenderDepth.outputs:execOut", "publishD455_depth.inputs:execIn"),
                ("createRenderDepth.outputs:execOut", "publishD455_depth_pcl.inputs:execIn"),

                ("createRenderRGB.outputs:execOut", "publishD455_camera_info_rgb.inputs:execIn"),
                ("createRenderRGB.outputs:execOut", "publishD455_rgb.inputs:execIn"),
            ],
            keys.SET_VALUES: [
                ("createRenderDepth.inputs:cameraPrim", depth_prim),
                ("createRenderDepth.inputs:height", 45),
                ("createRenderDepth.inputs:width", 80),

                ("createRenderRGB.inputs:cameraPrim", rgb_prim),
                ("createRenderRGB.inputs:height", 180),
                ("createRenderRGB.inputs:width", 320),


                ("publishD455_camera_info_depth.inputs:frameId",  frame_id),
                ("publishD455_camera_info_rgb.inputs:frameId",  frame_id),
                ("publishD455_depth.inputs:frameId",  frame_id),
                ("publishD455_depth_pcl.inputs:frameId",  frame_id),
                ("publishD455_rgb.inputs:frameId",  frame_id),

                ("publishD455_camera_info_depth.inputs:nodeNamespace",  "/go1"),
                ("publishD455_camera_info_rgb.inputs:nodeNamespace",  "/go1"),
                ("publishD455_depth.inputs:nodeNamespace",  "/go1"),
                ("publishD455_depth_pcl.inputs:nodeNamespace",  "/go1"),
                ("publishD455_rgb.inputs:nodeNamespace",  "/go1"),

                ("publishD455_camera_info_depth.inputs:topicName",  namespace + "/" + topic_name["camera_info"]),
                ("publishD455_camera_info_rgb.inputs:topicName",  namespace + "/" + topic_name["camera_info"]),
                ("publishD455_depth.inputs:topicName",  namespace + "/" + topic_name["depth"]),
                ("publishD455_depth_pcl.inputs:topicName",  namespace + "/" + topic_name["depth_pcl"]),
                ("publishD455_rgb.inputs:topicName",  namespace + "/" + topic_name["rgb"]),

                ("publishD455_camera_info_depth.inputs:type",  "camera_info"),
                ("publishD455_camera_info_rgb.inputs:type",  "camera_info"),
                ("publishD455_depth.inputs:type",  "depth"),
                ("publishD455_depth_pcl.inputs:type",  "depth_pcl"),
                ("publishD455_rgb.inputs:type",  "rgb"),
            ]
        }
    )

    return d455_graph