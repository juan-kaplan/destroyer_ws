import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import os
import yaml


class MapSaver(Node):
    def __init__(self):
        super().__init__("map_saver")

        self.declare_parameter("map_name", "my_map")
        self.map_name = (
            self.get_parameter("map_name").get_parameter_value().string_value
        )

        self.subscription = self.create_subscription(
            OccupancyGrid, "/map", self.save_map_callback, 10
        )
        self.get_logger().info("Waiting for /map topic to save...")

    def save_map_callback(self, msg):
        self.get_logger().info("Received map! Saving...")

        # 1. Save Image (.pgm)
        self.save_pgm(msg)

        # 2. Save Metadata (.yaml)
        self.save_yaml(msg)

        self.get_logger().info(
            f"Map saved successfully to {self.map_name}.pgm and .yaml"
        )

        # 3. Shutdown self after saving once
        rclpy.shutdown()

    def save_pgm(self, msg):
        width = msg.info.width
        height = msg.info.height

        # Convert ROS Data (-1, 0..100) to PGM Data (0..255)
        # ROS: -1 = Unknown, 0 = Free, 100 = Occupied
        # PGM: 205= Unknown, 254= Free, 0   = Occupied (Black)

        data = np.array(msg.data).reshape((height, width))

        # Flip Y (ROS origin is bottom-left, Image origin is top-left)
        data = np.flipud(data)

        # Create image array
        pgm_data = np.zeros((height, width), dtype=np.uint8)

        # Map values
        pgm_data.fill(205)  # Default Unknown (Gray)
        pgm_data[data == 0] = 254  # Free (White-ish)
        pgm_data[data == 100] = 0  # Occupied (Black)

        header = f"P5\n{width} {height}\n255\n"

        with open(f"{self.map_name}.pgm", "wb") as f:
            f.write(header.encode("ascii"))
            f.write(pgm_data.tobytes())

    def save_yaml(self, msg):
        yaml_data = {
            "image": f"{self.map_name}.pgm",
            "resolution": msg.info.resolution,
            "origin": [
                msg.info.origin.position.x,
                msg.info.origin.position.y,
                0.0,
            ],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.196,
        }

        with open(f"{self.map_name}.yaml", "w") as f:
            yaml.dump(yaml_data, f)


def main(args=None):
    rclpy.init(args=args)
    node = MapSaver()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()
