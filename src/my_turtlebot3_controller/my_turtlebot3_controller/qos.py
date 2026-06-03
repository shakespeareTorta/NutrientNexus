from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

STATE_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
