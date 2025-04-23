import sys
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QGroupBox, QFormLayout, QPushButton, QTableWidget,
    QTableWidgetItem, QHeaderView, QSpinBox, QGridLayout, QComboBox
)
from PyQt5.QtCore import Qt
from datetime import datetime

class WorkerGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Roskatsu Worker Dashboard")
        self.setGeometry(100, 100, 1200, 800)
        self.setStyleSheet("""
            QMainWindow {
                background-color: #f4f2ec;
            }
            QLabel#titleLabel {
                font-size: 20px;
                font-weight: bold;
                color: #3e3e3e;
            }
            QGroupBox {
                background-color: #fffdf6;
                border: 1px solid #c2bca2;
                border-radius: 6px;
                margin-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
                font-weight: bold;
                color: #5b5a4e;
            }
            QPushButton {
                background-color: #708238;
                color: white;
                border-radius: 5px;
                padding: 6px 12px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #5e6f2a;
            }
        """)
        self._init_ui()

    def _init_ui(self):
        self.task_counter = 1
        central = QWidget(self)
        main_layout = QVBoxLayout()

        title = QLabel("Roskatsu Worker Panel")
        title.setObjectName("titleLabel")
        title.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title)

        # Robot Overview
        robot_group = QGroupBox("Robot Overview")
        robot_layout = QVBoxLayout()
        robot_layout.addWidget(QLabel("Pinky-1 | Battery: 75% | Status: Idle"))
        robot_layout.addWidget(QLabel("Pinky-2 | Battery: 50% | Status: Moving"))
        robot_group.setLayout(robot_layout)

        # Task Request Form
        request_group = QGroupBox("Create New Task")
        form_layout = QFormLayout()
        origin_grid = QGridLayout()
        origin_grid.setHorizontalSpacing(20)
        origin_grid.setVerticalSpacing(10)
        origin_grid.setContentsMargins(0, 0, 0, 0)

        self.origin_inputs = {}
        zones = ["A", "B", "C", "D", "E", "F", "G", "H"]
        for i, zone in enumerate(zones):
            row = i // 4
            col = i % 4
            label = QLabel(zone)
            spinbox = QSpinBox()
            spinbox.setRange(0, 100)
            spinbox.setFixedWidth(50)
            self.origin_inputs[zone] = spinbox
            container = QWidget()
            hbox = QHBoxLayout()
            hbox.setContentsMargins(0, 0, 0, 0)
            hbox.setSpacing(5)
            hbox.addWidget(label)
            hbox.addWidget(spinbox)
            container.setLayout(hbox)
            origin_grid.addWidget(container, row, col)

        form_layout.addRow(QLabel("Select Origin Zones and Quantity:"), origin_grid)

        # Robot Selector
        self.robot_selector = QComboBox()
        self.robot_selector.addItem("Auto (Optimal)")
        self.robot_selector.addItem("Pinky-1")
        self.robot_selector.addItem("Pinky-2")
        form_layout.addRow("Assign Robot:", self.robot_selector)

        submit_btn = QPushButton("Submit Task")
        submit_btn.clicked.connect(self.add_task_request)
        form_layout.addRow(submit_btn)
        request_group.setLayout(form_layout)

        # Horizontally place Robot Overview and Task Form
        top_section = QHBoxLayout()
        top_section.addWidget(robot_group)
        top_section.addWidget(request_group)
        main_layout.addLayout(top_section)

        # Request Table
        table_group = QGroupBox("My Task Requests")
        table_layout = QVBoxLayout()
        self.table = QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels(["Robot ID", "Task ID", "Origin(s)", "Status", "Time"])
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        table_layout.addWidget(self.table)
        table_group.setLayout(table_layout)
        main_layout.addWidget(table_group)

        central.setLayout(main_layout)
        self.setCentralWidget(central)

    def add_task_request(self):
        origin_desc = []
        for zone, spinbox in self.origin_inputs.items():
            if spinbox.value() > 0:
                origin_desc.append(f"{zone}:{spinbox.value()}")

        if not origin_desc:
            return  # no valid selection

        time_str = datetime.now().strftime("%H:%M")
        task_id = f"T{self.task_counter:03}"
        self.task_counter += 1

        robot_id = self.robot_selector.currentText()
        if robot_id == "Auto (Optimal)":
            robot_id = "Auto"

        row_pos = self.table.rowCount()
        self.table.insertRow(0)
        self.table.setItem(0, 0, QTableWidgetItem(robot_id))
        self.table.setItem(0, 1, QTableWidgetItem(task_id))
        self.table.setItem(0, 2, QTableWidgetItem(", ".join(origin_desc)))

        status_item = QTableWidgetItem("Pending")
        status_item.setForeground(Qt.gray)
        self.table.setItem(0, 3, status_item)
        self.table.setItem(0, 4, QTableWidgetItem(time_str))

        # Reset spinboxes
        for spinbox in self.origin_inputs.values():
            spinbox.setValue(0)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = WorkerGUI()
    window.show()
    sys.exit(app.exec_())
