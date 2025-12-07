import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import os
import yaml
import cv2


class MapSaver(Node):
    def __init__(self):
        super().__init__("map_saver")

        self.declare_parameter("map_name", "my_map")
        self.map_name = (
            self.get_parameter("map_name").get_parameter_value().string_value
        )

        self.subscription = self.create_subscription(
            OccupancyGrid, "/best_map", self.save_map_callback, 10
        )
        self.get_logger().info("Waiting for /map topic to save...")

    def save_map_callback(self, msg):
        self.get_logger().info("Received map! Saving...")

        self.save_png(msg)
        self.save_yaml(msg)
        self.get_logger().info(
            f"Map saved successfully to {self.map_name}.png and .yaml"
        )
        rclpy.shutdown()

    def save_png(self, msg):
        width = msg.info.width
        height = msg.info.height

        data = np.array(msg.data).reshape((height, width))
        data = np.flipud(data)
        img = np.full((height, width), 205, dtype=np.uint8)

        known_mask = data >= 0
        img[known_mask] = 255 - (data[known_mask] * 255 / 100)

        img = img.astype(np.uint8)

        kernel_size = 2
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        img = cv2.erode(img, kernel, iterations=1)

        cv2.imwrite(f"src/state_machine/maps/{self.map_name}.png", img)

    def save_yaml(self, msg):
        yaml_data = {
            "image": f"{self.map_name}.png",
            "mode": "scale",
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
