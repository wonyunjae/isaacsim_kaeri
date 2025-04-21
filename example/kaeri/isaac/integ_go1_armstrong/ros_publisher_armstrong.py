import omni.graph.core as og

def ros_clock_publisher():
    keys = og.Controller.Keys
    (clock_graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": "/ROS_armstrong/ROS_clock_publisher",
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
            "graph_path": "/ROS_armstrong/ROS_joint_publisher",
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
                ("publishJointState.inputs:targetPrim", "/full_test"),
                ("publishJointState.inputs:topicName",  "joint_state"),
            ]
        }
    )

    return joint_graph

def ros_joint_tf_publisher_armstrong(bodies):
    keys = og.Controller.Keys
    (graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": "/ROS_armstrong/ROS_joints_tf_publisher",
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
                ("publishTransform.inputs:nodeNamespace", "/armstrong"),
            ]
        }
    )

    return graph

def ros_odom_publisher():
    keys = og.Controller.Keys
    (odom_graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": "/ROS_armstrong/ROS_odom_publisher",
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("readSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("readOdom", "omni.isaac.core_nodes.IsaacComputeOdometry"),
                ("publishOdom", "isaacsim.ros1.bridge.ROS1PublishOdometry"),
                ("publishOdom_tf", "isaacsim.ros1.bridge.ROS1PublishRawTransformTree"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "readOdom.inputs:execIn"),
                ("OnTick.outputs:tick", "publishOdom.inputs:execIn"),
                ("OnTick.outputs:tick", "publishOdom_tf.inputs:execIn"),
                
                ("readSimTime.outputs:simulationTime", "publishOdom.inputs:timeStamp"),
                ("readSimTime.outputs:simulationTime", "publishOdom_tf.inputs:timeStamp"),

                ("readOdom.outputs:angularVelocity", "publishOdom.inputs:angularVelocity"),
                ("readOdom.outputs:linearVelocity", "publishOdom.inputs:linearVelocity"),
                ("readOdom.outputs:position", "publishOdom.inputs:position"),
                ("readOdom.outputs:orientation", "publishOdom.inputs:orientation"),
                ("readOdom.outputs:position", "publishOdom_tf.inputs:translation"),
                ("readOdom.outputs:orientation", "publishOdom_tf.inputs:rotation"),
            ],
            keys.SET_VALUES: [
                ("readOdom.inputs:chassisPrim", "/full_test/base_track"),

                ("publishOdom.inputs:odomFrameId",  "odom"),
                ("publishOdom.inputs:chassisFrameId",  "base_link"),
                ("publishOdom.inputs:nodeNamespace", "/go1"),
                ("publishOdom_tf.inputs:childFrameId", "base_link"),
            ]
        }
    )

    return odom_graph

def ros_d455_publisher_armstrong(frame_id, namespace, topic_name, depth_prim, rgb_prim):
    keys = og.Controller.Keys
    
    (d455_graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": "/ROS_armstrong/ROS_" + frame_id + "_publisher",
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
                ("createRenderRGB.inputs:height", 180),
                ("createRenderRGB.inputs:width", 320),

                ("publishD455_camera_info_depth.inputs:frameId",  frame_id),
                ("publishD455_camera_info_rgb.inputs:frameId",  frame_id),
                ("publishD455_depth.inputs:frameId",  frame_id),
                ("publishD455_depth_pcl.inputs:frameId",  frame_id),
                ("publishD455_rgb.inputs:frameId",  frame_id),

                ("publishD455_camera_info_depth.inputs:nodeNamespace",  "/armstrong"),
                ("publishD455_camera_info_rgb.inputs:nodeNamespace",  "/armstrong"),
                ("publishD455_depth.inputs:nodeNamespace", "/armstrong"),
                ("publishD455_depth_pcl.inputs:nodeNamespace",  "/armstrong"),
                ("publishD455_rgb.inputs:nodeNamespace",  "/armstrong"),

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