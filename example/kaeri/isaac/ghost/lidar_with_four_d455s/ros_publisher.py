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


def ros_lidar_publisher(frame_id, topic_name, lidar_prim):
    keys = og.Controller.Keys
    (lidar_graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": "/ROS/ROS_rtxlidar_publisher",
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"), 
                ("renderProduct", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                ("publishLidar", "isaacsim.ros1.bridge.ROS1RtxLidarHelper"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "renderProduct.inputs:execIn"),
                ("renderProduct.outputs:execOut", "publishLidar.inputs:execIn"),
                ("renderProduct.outputs:renderProductPath", "publishLidar.inputs:renderProductPath"),
            ],
            keys.SET_VALUES: [
                ("renderProduct.inputs:cameraPrim", lidar_prim),
                ("publishLidar.inputs:type", "point_cloud"),
                ("publishLidar.inputs:frameId", frame_id),
                ("publishLidar.inputs:topicName", topic_name),
            ]
        }
    )

    return lidar_graph