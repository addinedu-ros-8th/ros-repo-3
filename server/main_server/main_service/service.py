import os
import json
import tempfile
import subprocess
from datetime import datetime

from server.main_server.databases.logger import RoscarsLogWriter
from server.main_server.databases.query import MainServiceQuery
from server.main_server.databases.models.roscars_models import Task, RosCars, Delivery

class MainService:
    def __init__(self, db):
        self.logger_session = db.get_session("roscars_log")
        self.logger = RoscarsLogWriter(self.logger_session)
        self.roscars_session = db.get_session("roscars")
        self.query = MainServiceQuery(
            roscars_session=self.roscars_session,
            roscars_log_session=self.logger_session
        )
        self.enable_shutdown_after_ai_result = False

    def _launch_start_delivery_client(self, roscar: RosCars, delivery: Delivery):
        task_list = self.roscars_session.query(Task).filter_by(delivery_id=delivery.delivery_id).all()
        tasks = [{"task_id": t.task_id, "shoes_model_id": t.shoes_model_id, "location_id": t.location_id} for t in task_list]
        with tempfile.NamedTemporaryFile(mode="w", delete=False, suffix=".json") as tf:
            json.dump({"delivery_id": delivery.delivery_id, "tasks": tasks}, tf)
            json_path = tf.name

        namespace = roscar.roscar_namespace
        domain_id = str(roscar.roscar_domain_id or "214")
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = domain_id

        ros_env_command = (
            "source ~/.bashrc && "
            "source .roscars_venv/bin/activate && "
            "source install/setup.bash && "
            f"ROS_DOMAIN_ID={domain_id} python3 server/main_server/main_service/ros_interface/action/start_delivery_client.py {namespace} {domain_id} {json_path}"
        )
        cmd = ["bash", "-c", ros_env_command]
        print(f"[ROS2 Client] subprocess 실행 → {cmd} with ROS_DOMAIN_ID={domain_id}")
        subprocess.Popen(cmd, env=env)

    def register_shutdown_flag(self, shutdown_flag):
        self.shutdown_flag = shutdown_flag