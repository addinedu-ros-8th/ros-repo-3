# ROScars(로스카스 🍺)
파이널 프로젝트 3조 저장소. 협업형 스마트 물류 어시스턴트 시스템

```plaintext
project_root/
├── docs
│   ├── file_structure.md
│   ├── interface_specifications.md
│   └── system_architecture.md
├── mobile_controller
│   ├── actuator.cpp
│   ├── camera_streamer.cpp
│   ├── emergency_monitor.cpp
│   ├── image_encoder.rs
│   ├── ros_nodes
│   │   ├── controller_node.py
│   │   └── launch.py
│   └── sensor.cpp
├── README.md
├── requirements.txt
├── roscars.sql
├── server
│   ├── ai_server
│   │   ├── detection
│   │   │   ├── object_detector.py
│   │   │   └── tracking_module.py
│   │   ├── docker-compose.yml
│   │   ├── filesystem
│   │   │   ├── file_cleanup.py
│   │   │   └── file_writer.py
│   │   ├── model_loader.py
│   │   └── tcp_server.py
│   └── main_server
│       ├── db_access.py
│       ├── logger.py
│       ├── main.py
│       ├── network
│       │   ├── emergency_handler.py
│       │   ├── message_router.py
│       │   ├── __pycache__
│       │   │   ├── emergency_handler.cpython-312.pyc
│       │   │   └── tcp_handler.cpython-312.pyc
│       │   └── tcp_handler.py
│       ├── parameters_config.yaml
│       ├── __pycache__
│       │   ├── db_access.cpython-312.pyc
│       │   └── logger.cpython-312.pyc
│       ├── ros_nodes
│       │   ├── launch.py
│       │   ├── __pycache__
│       │   │   ├── service_node.cpython-312.pyc
│       │   │   └── task_manager.cpython-312.pyc
│       │   ├── service_node.py
│       │   └── task_manager.py
│       ├── server_log.txt
│       └── tests
│           ├── test_db_access.py
│           ├── test_emergency_handler.py
│           ├── test_service_node.py
│           ├── test_task_manager.py
│           └── test_tcp_handler.py
├── shared_interfaces
│   ├── msg
│   │   ├── BatteryStatus.msg
│   │   ├── Emergency.msg
│   │   └── TaskResult.msg
│   └── srv
│       └── ManualOverride.srv
├── test.py
└── viewer
    ├── manager
    │   ├── dashboard_panel.py
    │   ├── dialogs.py
    │   ├── main.py
    │   ├── __pycache__
    │   │   ├── dashboard_panel.cpython-312.pyc
    │   │   ├── dialogs.cpython-312.pyc
    │   │   └── main.cpython-312.pyc
    │   └── ros_bridge.py
    ├── mode_select.py
    ├── __pycache__
    │   ├── mode_select.cpython-312.pyc
    │   └── theme.cpython-312.pyc
    ├── staff
    │   ├── cache_manager.py
    │   ├── camera_panel.py
    │   ├── cart_panel.py
    │   ├── main.py
    │   ├── product_info_panel.py
    │   ├── __pycache__
    │   │   ├── cache_manager.cpython-312.pyc
    │   │   ├── camera_panel.cpython-312.pyc
    │   │   ├── cart_panel.cpython-312.pyc
    │   │   ├── dashboard_panel.cpython-312.pyc
    │   │   ├── main.cpython-312.pyc
    │   │   ├── product_info_panel.cpython-312.pyc
    │   │   ├── qr_reader.cpython-312.pyc
    │   │   ├── request_table.cpython-312.pyc
    │   │   └── request_wait_panel.cpython-312.pyc
    │   ├── qr_reader.py
    │   └── request_wait_panel.py
    └── theme.py
```

