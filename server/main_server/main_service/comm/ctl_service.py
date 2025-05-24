import os
import json
import tempfile
import subprocess

from server.main_server.databases.logger import RoscarsLogWriter
from server.main_server.databases.query import MainServiceQuery
from server.main_server.databases.query_function import QueryFunction
from server.main_server.databases.models.roscars_models import Task, RosCars, Delivery
from server.main_server.databases.models.roscars_models import RosCars

class MainControlService:
    def __init__(self, db):
        self.db = db 
        self.query_function = QueryFunction
        self.enable_shutdown_after_ai_result = False
        self.shutdown_flag = None

    def run_logger(self, method_name: str, *args, **kwargs):
        logger = RoscarsLogWriter(self.db)
        method = getattr(logger, method_name)
        return method(*args, **kwargs)

    def run_query(self, method_name: str, *args, **kwargs):
        query = MainServiceQuery(self.db, self.query_function)
        method = getattr(query, method_name)
        return method(*args, **kwargs)

    def _launch_start_delivery_client(self, roscar: RosCars, delivery: Delivery):
        with self.db.get_session("roscars") as session:
            task_list = session.query(Task).filter_by(delivery_id=delivery.delivery_id).all()
            tasks = [
                {"task_id": t.task_id, "shoes_model_id": t.shoes_model_id, "location_id": t.location_id}
                for t in task_list
            ]

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
            f"ROS_DOMAIN_ID={domain_id} python3 server/main_server/main_service/main_service/action/start_delivery_client.py {namespace} {domain_id} {json_path}"
        )
        cmd = ["bash", "-c", ros_env_command]
        print(f"[ROS2 Client] subprocess 실행 → {cmd} with ROS_DOMAIN_ID={domain_id}")
        subprocess.Popen(cmd, env=env)


    def register_shutdown_flag(self, shutdown_flag):
        self.shutdown_flag = shutdown_flag
