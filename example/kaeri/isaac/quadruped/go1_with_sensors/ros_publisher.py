import omni.graph.core as og

def ros_clock_publisher():
    keys = og.Controller.Keys
    (clock_graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": "/ROS/ROS_clock_publisher",
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("readSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("publishClock", "isaacsim.ros1.bridge.ROS1PublishClock"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "publishClock.inputs:execIn"),
                ("readSimTime.outputs:simulationTime", "publishClock.inputs:timeStamp"),
            ]
        },
    )

    return clock_graph

def ros_joint_state_publisher():
    keys = og.Controller.Keys
    (joint_graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": "/ROS/ROS_joint_publisher",
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("readSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("publishJointState", "isaacsim.ros1.bridge.ROS1PublishJointState"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "publishJointState.inputs:execIn"),
                ("readSimTime.outputs:simulationTime", "publishJointState.inputs:timeStamp"),
            ],
            keys.SET_VALUES: [
                ("publishJointState.inputs:targetPrim", "/World/Robot/Go1"),
                ("publishJointState.inputs:topicName",  "joint_state"),
            ]
        }
    )

    return joint_graph

def ros_joints_tf_publisher(bodies):
    keys = og.Controller.Keys
    (graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": "/ROS/ROS_joints_tf_publisher",
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("readSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("publishTransform", "isaacsim.ros1.bridge.ROS1PublishTransformTree"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "publishTransform.inputs:execIn"),
                ("readSimTime.outputs:simulationTime", "publishTransform.inputs:timeStamp"),
            ],
            keys.SET_VALUES: [
                ("publishTransform.inputs:parentPrim", bodies),
                ("publishTransform.inputs:targetPrims",  bodies),
            ]
        }
    )

    return graph

def ros_sensors_tf_publisher(): 
    keys = og.Controller.Keys 
    (graph, _, _, _) = og.Controller.edit( 
        { 
            "graph_path": "/ROS/ROS_sensors_tf_publisher", 
            "evaluator_name": "push", 
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND, 
        }, 
        { 
            keys.CREATE_NODES: [ 
                ("OnTick", "omni.graph.action.OnTick"), 
                ("readSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"), 
                ("publishTransform", "isaacsim.ros1.bridge.ROS1PublishTransformTree"), 
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

def ros_foot_contact_publisher():
    keys = og.Controller.Keys
    (graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": "/ROS/ROS_foot_contact_publisher",
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

                ("publishContactFL", "omni.kaeri.ros_bridge.ROS1PublishContactSensor"),
                ("publishContactFR", "omni.kaeri.ros_bridge.ROS1PublishContactSensor"),
                ("publishContactRL", "omni.kaeri.ros_bridge.ROS1PublishContactSensor"),
                ("publishContactRR", "omni.kaeri.ros_bridge.ROS1PublishContactSensor"),
            ],
            keys.CONNECT: [
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

                ("readContactFL.outputs:inContact", "publishContactFL.inputs:inContact"),
                ("readContactFR.outputs:inContact", "publishContactFR.inputs:inContact"),
                ("readContactRL.outputs:inContact", "publishContactRL.inputs:inContact"),
                ("readContactRR.outputs:inContact", "publishContactRR.inputs:inContact"),

                ("readContactFL.outputs:value", "publishContactFL.inputs:value"),
                ("readContactFR.outputs:value", "publishContactFR.inputs:value"),
                ("readContactRL.outputs:value", "publishContactRL.inputs:value"),
                ("readContactRR.outputs:value", "publishContactRR.inputs:value"),
            ],
            keys.SET_VALUES: [
                ("readContactFL.inputs:csPrim", "/World/Robot/Go1/FL_foot/sensor"),
                ("readContactFR.inputs:csPrim", "/World/Robot/Go1/FR_foot/sensor"),
                ("readContactRL.inputs:csPrim", "/World/Robot/Go1/RL_foot/sensor"),
                ("readContactRR.inputs:csPrim", "/World/Robot/Go1/RR_foot/sensor"),

                # set nodeNamespace
                ("publishContactFL.inputs:nodeNamespace", "foot_incontact"),
                ("publishContactFR.inputs:nodeNamespace", "foot_incontact"),
                ("publishContactRL.inputs:nodeNamespace", "foot_incontact"),
                ("publishContactRR.inputs:nodeNamespace", "foot_incontact"),

                # set topicName
                ("publishContactFL.inputs:topicName", "FL_foot"),
                ("publishContactFR.inputs:topicName", "FR_foot"),
                ("publishContactRL.inputs:topicName", "RL_foot"),
                ("publishContactRR.inputs:topicName", "RR_foot"),
            ]
        }
    )

    return graph

def ros_odom_publisher():
    keys = og.Controller.Keys
    (odom_graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": "/ROS/ROS_odom_publisher",
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("readSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("readOdom", "omni.isaac.core_nodes.IsaacComputeOdometry"),
                ("publishOdom", "isaacsim.ros1.bridge.ROS1PublishOdometry"),
                # ("publishOdom_tf", "isaacsim.ros1.bridge.ROS1PublishRawTransformTree"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "readOdom.inputs:execIn"),
                ("OnTick.outputs:tick", "publishOdom.inputs:execIn"),
                # ("OnTick.outputs:tick", "publishOdom_tf.inputs:execIn"),
                
                ("readSimTime.outputs:simulationTime", "publishOdom.inputs:timeStamp"),
                # ("readSimTime.outputs:simulationTime", "publishOdom_tf.inputs:timeStamp"),

                ("readOdom.outputs:angularVelocity", "publishOdom.inputs:angularVelocity"),
                ("readOdom.outputs:linearVelocity", "publishOdom.inputs:linearVelocity"),
                ("readOdom.outputs:position", "publishOdom.inputs:position"),
                ("readOdom.outputs:orientation", "publishOdom.inputs:orientation"),
                # ("readOdom.outputs:position", "publishOdom_tf.inputs:translation"),
                # ("readOdom.outputs:orientation", "publishOdom_tf.inputs:rotation"),
            ],
            keys.SET_VALUES: [
                ("readOdom.inputs:chassisPrim", "/World/Robot/Go1/trunk"), 

                ("publishOdom.inputs:odomFrameId",  "odom"),
                ("publishOdom.inputs:chassisFrameId",  "trunk"),
                ("publishOdom.inputs:topicName",  "odom_sim"),

                # ("publishOdom_tf.inputs:parentFrameId", "odom"),
                # ("publishOdom_tf.inputs:childFrameId", "trunk"),
            ]
        }
    )

    return odom_graph


def ros_imu_publisher():
    keys = og.Controller.Keys
    (imu_graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": "/ROS/ROS_imu_publisher",
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("readSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("readImu", "omni.isaac.sensor.IsaacReadIMU"),
                ("publishImu", "isaacsim.ros1.bridge.ROS1PublishImu"),
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
                ("publishImu.inputs:topicName",  "imu"),
                ("publishImu.inputs:frameId",  "imu_link"),
            ]
        }
    )

    return imu_graph

def ros_lidar_publisher(frame_id, topic_name, lidar_prim):
    keys = og.Controller.Keys
    (lidar_graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": "/ROS/ROS_lidar_publisher",
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("readSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("readLidar", "omni.isaac.range_sensor.IsaacReadLidarPointCloud"),
                ("publishLidar", "isaacsim.ros1.bridge.ROS1PublishPointCloud"),
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
                ("publishLidar.inputs:topicName",  topic_name),
            ]
        }
    )

    return lidar_graph

def ros_d455_publisher(frame_id, namespace, topic_name, depth_prim, rgb_prim):
    keys = og.Controller.Keys
    
    (d455_graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": "/ROS/ROS_" + frame_id + "_publisher",
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),

                ("createRenderDepth", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                ("createRenderRGB", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),

                ("publishD455_camera_info_depth", "isaacsim.ros1.bridge.ROS1CameraHelper"),
                ("publishD455_camera_info_rgb", "isaacsim.ros1.bridge.ROS1CameraHelper"),

                ("publishD455_depth", "isaacsim.ros1.bridge.ROS1CameraHelper"),
                ("publishD455_depth_pcl", "isaacsim.ros1.bridge.ROS1CameraHelper"),
                ("publishD455_rgb", "isaacsim.ros1.bridge.ROS1CameraHelper"),
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
                # ("createRenderRGB.inputs:height", 45),
                # ("createRenderRGB.inputs:width", 80),
                ("createRenderRGB.inputs:height", 180),
                ("createRenderRGB.inputs:width", 320),


                ("publishD455_camera_info_depth.inputs:frameId",  frame_id),
                ("publishD455_camera_info_rgb.inputs:frameId",  frame_id),
                ("publishD455_depth.inputs:frameId",  frame_id),
                ("publishD455_depth_pcl.inputs:frameId",  frame_id),
                ("publishD455_rgb.inputs:frameId",  frame_id),

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