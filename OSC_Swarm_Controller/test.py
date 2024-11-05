# from PlaceHomogeneousPointsInZone import generate_relaxed_points

# data_string = "[[373.1137, 114.1604], [371.8446, 109.2067], [365.2107, 111.1211], [358.9317, 113.6646], [349.6094, 116.1712], [337.3386, 114.4723], [321.368, 111.348], [311.154, 108.8157], [301.6672, 104.1241], [294.3586, 101.7708], [284.0513, 102.7553], [269.9465, 106.9397], [255.9055, 110.7102], [241.1849, 114.9598], [229.188, 116.5114], [215.9669, 116.8528], [202.7015, 125.4388], [194.1635, 137.4292], [188.4225, 154.1989], [182.8929, 169.9486], [180.3201, 186.0556], [174.3452, 201.0879], [166.1181, 216.2103], [155.8573, 230.9045], [147.907, 246.29], [140.8328, 262.0089], [136.477, 279.8167], [136.4165, 297.2284], [134.1736, 315.7289], [131.8721, 332.7748], [134.4662, 344.7558], [134.3878, 356.0133], [131.1751, 369.5604], [133.9034, 380.6879], [142.8212, 393.7831], [159.0959, 406.1145], [173.306, 419.035], [190.4665, 431.5323], [203.4956, 443.083], [215.8334, 454.3948], [228.913, 461.5452], [241.5366, 467.3407], [254.8916, 471.9225], [268.6401, 475.0311], [285.4601, 478.6144], [304.5397, 480.2299], [323.0768, 483.2185], [340.5895, 486.4331], [357.4611, 488.2809], [372.4318, 487.9362], [386.9002, 485.3282], [400.1117, 480.9183], [410.5583, 469.9876], [423.3351, 460.0084], [438.1455, 446.9809], [451.0481, 434.028], [461.1772, 423.9272], [467.4336, 415.4895], [472.5055, 406.7267], [478.1209, 396.4081], [483.8834, 377.9023], [490.2021, 362.7736], [497.2954, 350.0859], [504.4456, 342.1916], [514.77, 334.6975], [527.3573, 325.4929], [536.9893, 313.1041], [540.5679, 299.3723], [537.7455, 285.1886], [533.356, 274.5914], [531.7812, 263.9793], [533.1267, 254.7114], [536.5532, 245.1473], [536.5417, 238.4453], [531.2181, 230.5258], [522.2779, 224.6041], [511.7994, 221.0675], [506.341, 217.725], [504.0556, 215.4425], [501.8379, 215.209], [496.2083, 214.7189], [491.7082, 214.8229], [490.6353, 216.3463], [493.8458, 219.3714], [497.5232, 217.9637], [494.0149, 219.7406], [488.0893, 218.7645]]"
# num_points = 10
# iterations = 1000

# points = generate_relaxed_points(data_string, num_points, iterations)

# print(points)


import sys
import ast
from PyQt5 import QtWidgets, QtGui, QtCore
from PlaceHomogeneousPointsInZone import generate_relaxed_points

class PolygonDisplay(QtWidgets.QWidget):
    def __init__(self, data_string, points):
        super().__init__()
        self.setWindowTitle("Polygon and Relaxed Points")
        self.resize(800, 600)

        # Parse the data_string to get polygon_coords
        try:
            polygon_coords = ast.literal_eval(data_string)
            if not isinstance(polygon_coords, list):
                raise ValueError("Parsed data is not a list.")
            self.polygon_points = [QtCore.QPointF(coords[0], coords[1]) for coords in polygon_coords]
        except (SyntaxError, ValueError) as e:
            QtWidgets.QMessageBox.critical(self, "Error", f"Invalid coordinate data: {e}")
            self.polygon_points = []
            return

        # Store the relaxed points
        self.relaxed_points = [QtCore.QPointF(x, y) for x, y in points]

        # Adjust the window size based on the polygon bounds
        self.adjust_window_size()

    def adjust_window_size(self):
        # Calculate the bounding rectangle of the polygon
        xs = [point.x() for point in self.polygon_points]
        ys = [point.y() for point in self.polygon_points]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        margin = 50  # Add some margin around the polygon

        width = max_x - min_x + 2 * margin
        height = max_y - min_y + 2 * margin

        self.resize(int(width), int(height))

        # Offset for centering the polygon in the window
        self.x_offset = -min_x + margin
        self.y_offset = -min_y + margin

        # Apply offset to polygon points
        self.polygon_points = [QtCore.QPointF(p.x() + self.x_offset, p.y() + self.y_offset) for p in self.polygon_points]
        # Apply offset to relaxed points
        self.relaxed_points = [QtCore.QPointF(p.x() + self.x_offset, p.y() + self.y_offset) for p in self.relaxed_points]

    def paintEvent(self, event):
        qp = QtGui.QPainter(self)
        qp.setRenderHint(QtGui.QPainter.Antialiasing)
        # Draw the polygon
        if self.polygon_points:
            pen = QtGui.QPen(QtCore.Qt.black, 2)
            qp.setPen(pen)
            qp.drawPolygon(QtGui.QPolygonF(self.polygon_points))

        # Draw the relaxed points
        if self.relaxed_points:
            pen = QtGui.QPen(QtCore.Qt.blue)
            brush = QtGui.QBrush(QtCore.Qt.blue)
            qp.setPen(pen)
            qp.setBrush(brush)
            for point in self.relaxed_points:
                qp.drawEllipse(point, 3, 3)

if __name__ == "__main__":
    from PlaceHomogeneousPointsInZone import generate_relaxed_points

    data_string = "[[373.1137, 114.1604], [371.8446, 109.2067], [365.2107, 111.1211], [358.9317, 113.6646], [349.6094, 116.1712], [337.3386, 114.4723], [321.368, 111.348], [311.154, 108.8157], [301.6672, 104.1241], [294.3586, 101.7708], [284.0513, 102.7553], [269.9465, 106.9397], [255.9055, 110.7102], [241.1849, 114.9598], [229.188, 116.5114], [215.9669, 116.8528], [202.7015, 125.4388], [194.1635, 137.4292], [188.4225, 154.1989], [182.8929, 169.9486], [180.3201, 186.0556], [174.3452, 201.0879], [166.1181, 216.2103], [155.8573, 230.9045], [147.907, 246.29], [140.8328, 262.0089], [136.477, 279.8167], [136.4165, 297.2284], [134.1736, 315.7289], [131.8721, 332.7748], [134.4662, 344.7558], [134.3878, 356.0133], [131.1751, 369.5604], [133.9034, 380.6879], [142.8212, 393.7831], [159.0959, 406.1145], [173.306, 419.035], [190.4665, 431.5323], [203.4956, 443.083], [215.8334, 454.3948], [228.913, 461.5452], [241.5366, 467.3407], [254.8916, 471.9225], [268.6401, 475.0311], [285.4601, 478.6144], [304.5397, 480.2299], [323.0768, 483.2185], [340.5895, 486.4331], [357.4611, 488.2809], [372.4318, 487.9362], [386.9002, 485.3282], [400.1117, 480.9183], [410.5583, 469.9876], [423.3351, 460.0084], [438.1455, 446.9809], [451.0481, 434.028], [461.1772, 423.9272], [467.4336, 415.4895], [472.5055, 406.7267], [478.1209, 396.4081], [483.8834, 377.9023], [490.2021, 362.7736], [497.2954, 350.0859], [504.4456, 342.1916], [514.77, 334.6975], [527.3573, 325.4929], [536.9893, 313.1041], [540.5679, 299.3723], [537.7455, 285.1886], [533.356, 274.5914], [531.7812, 263.9793], [533.1267, 254.7114], [536.5532, 245.1473], [536.5417, 238.4453], [531.2181, 230.5258], [522.2779, 224.6041], [511.7994, 221.0675], [506.341, 217.725], [504.0556, 215.4425], [501.8379, 215.209], [496.2083, 214.7189], [491.7082, 214.8229], [490.6353, 216.3463], [493.8458, 219.3714], [497.5232, 217.9637], [494.0149, 219.7406], [488.0893, 218.7645]]"

    num_points = 10
    iterations = 1000

    # Generate the relaxed points
    points = generate_relaxed_points(data_string, num_points, iterations)

    # Create the application and display the window
    app = QtWidgets.QApplication(sys.argv)
    window = PolygonDisplay(data_string, points)
    window.show()
    sys.exit(app.exec_())
