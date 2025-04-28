# dashboard_panel.py
from PyQt5.QtWidgets import (
    QWidget, QGroupBox, QVBoxLayout, QHBoxLayout, QLabel, QFormLayout,
    QGridLayout, QSpinBox, QComboBox, QPushButton, QSpacerItem, QSizePolicy
)
from PyQt5.QtCore import Qt, pyqtSignal
from datetime import datetime
from .theme import apply_staff_theme
from server.main_server.db_access import insert_task


class StaffDashboard(QWidget):
    task_submitted = pyqtSignal(dict)  # signal: emits task info when submitted

    def __init__(self):
        super().__init__()
        self.task_counter = 1
        apply_staff_theme(self)
        self._init_ui()

    def _init_ui(self):
        layout = QHBoxLayout()

        # Robot Overview
        robot_group = QGroupBox("Robot Overview")
        robot_layout = QVBoxLayout()
        robot_layout.addWidget(QLabel("Pinky-1 | Battery: 75% | Status: Idle"))
        robot_layout.addWidget(QLabel("Pinky-2 | Battery: 50% | Status: Moving"))
        robot_group.setLayout(robot_layout)

        # Task Form
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

        # Robot selector
        self.robot_selector = QComboBox()
        self.robot_selector.addItem("Auto (Optimal)")
        self.robot_selector.addItem("Pinky-1")
        self.robot_selector.addItem("Pinky-2")
        form_layout.addRow("Assign Robot:", self.robot_selector)

        submit_btn = QPushButton("Submit Task")
        submit_btn.clicked.connect(self._emit_task_request)
        form_layout.addRow(submit_btn)
        request_group.setLayout(form_layout)

        layout.addWidget(robot_group)
        layout.addWidget(request_group)
        self.setLayout(layout)

    def _emit_task_request(self):
        origin_list = []
        quantity_list = []

        for zone, spinbox in self.origin_inputs.items():
            count = spinbox.value()
            if count > 0:
                origin_list.append(zone)
                quantity_list.append(str(count))

        if not origin_list:
            return

        origin_str = ",".join(origin_list)
        quantity_str = ",".join(quantity_list)
        now = datetime.now()  # Use full datetime object
        task_id = f"T{self.task_counter:03}"
        self.task_counter += 1

        robot_id = self.robot_selector.currentText()
        if robot_id == "Auto (Optimal)":
            robot_id = "Auto"

        task_info = {
            "robot": robot_id,
            "task_id": task_id,
            "origin": origin_str,
            "quantity": quantity_str,
            "status": "Pending",
            "time": now  # Store full datetime
        }

        insert_task(
            robot=task_info["robot"],
            task_code=task_info["task_id"],
            origin=task_info["origin"],
            quantity=task_info["quantity"],
            status=task_info["status"],
            time=task_info["time"]
        )

        self.task_submitted.emit(task_info)

        for spinbox in self.origin_inputs.values():
            spinbox.setValue(0)
