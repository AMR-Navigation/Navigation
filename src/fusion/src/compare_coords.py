#!/usr/bin/env python

import rospy
from messages.msg import coordinates
from geometry_msgs.msg import Point

class CoordinateTracker:
    def __init__(self, filename="coordinates_record.txt"):
        rospy.init_node("coordinate_tracker", anonymous=False)
        self.sub = rospy.Subscriber("coordinates_topic", coordinates, self.callback)
        self.filename = filename
        self.matched_objects = []
        self.unmatched_objects = []
        self.load_existing_coordinates()

    def load_existing_coordinates(self):
        try:
            with open(self.filename, "r") as file:
                section = None
                for line in file:
                    line = line.strip()
                    if line == "Matched Coordinates:":
                        section = "matched"
                    elif line == "Unmatched Coordinates:":
                        section = "unmatched"
                    elif line and section:
                        x, y, z = map(float, line.split(","))
                        point = Point(x=x, y=y, z=z)
                        if section == "matched":
                            self.matched_objects.append(point)
                        elif section == "unmatched":
                            self.unmatched_objects.append(point)
        except FileNotFoundError:
            pass  # No existing file, will create a new one

    def callback(self, data):
        self.update_coordinates(data.matched_objects, self.matched_objects)
        self.update_coordinates(data.unmatched_objects, self.unmatched_objects)
        self.save_coordinates()

    def update_coordinates(self, new_coords, existing_coords):
        for new_point in new_coords:
            if not any(self.is_close(new_point, existing_point) for existing_point in existing_coords):
                existing_coords.append(new_point)

    def is_close(self, point1, point2, threshold=0.3):
        return abs(point1.x - point2.x) < threshold and abs(point1.y - point2.y) < threshold

    def save_coordinates(self):
        with open(self.filename, "w") as file:
            file.write("Matched Coordinates:\n")
            for point in self.matched_objects:
                file.write(f"{point.x}, {point.y}, {point.z}\n")
            file.write("Unmatched Coordinates:\n")
            for point in self.unmatched_objects:
                file.write(f"{point.x}, {point.y}, {point.z}\n")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    tracker = CoordinateTracker()
    tracker.run()
