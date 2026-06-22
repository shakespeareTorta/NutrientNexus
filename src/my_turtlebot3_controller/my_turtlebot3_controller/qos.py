from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

STATE_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
