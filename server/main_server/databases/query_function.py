from server.main_server.databases.models.roscars_models import (
    RackLocation, Delivery, RosCars, Task, RackLocation)
from server.main_server.databases.models.roscars_log_models import (
    TaskEventLog, DeliveryEventLog, RosCarEventLog,
    PrecisionStopLog, RoscarTrajectoryLog, RosCarDrivingEventLog,
    RoscarSensorFusionRawLog, ControlCommandLog, FileSystemLog,
    RackSensorLog
)

class QueryFunction:
    @staticmethod
    def get_location_id_by_name(sess, rack_name: str) -> int | None:
        loc = sess.query(RackLocation).filter_by(name=rack_name).first()
        return loc.location_id if loc else None

    @staticmethod
    def get_task_event_log_by_task_id(roscars_sess, log_sess, task_id: int):
        logs = log_sess.query(TaskEventLog)\
            .filter(TaskEventLog.task_id == task_id)\
            .order_by(TaskEventLog.changed_at).all()

        task = roscars_sess.query(Task).filter_by(task_id=task_id).first()
        shoes_model = task.shoes_model if task else None
        location = task.location if task else None

        return [{
            "event_id": log.event_id,
            "task_id": log.task_id,
            "previous_event": log.previous_event.name,
            "current_event": log.current_event.name,
            "changed_at": log.changed_at,
            "shoes_name": shoes_model.name if shoes_model else None,
            "location_name": location.name if location else None
        } for log in logs]

    @staticmethod
    def get_delivery_event_log_by_delivery_id(roscars_sess, log_sess, delivery_id: int):
        logs = log_sess.query(DeliveryEventLog)\
            .filter(DeliveryEventLog.delivery_id == delivery_id)\
            .order_by(DeliveryEventLog.timestamp).all()

        delivery = roscars_sess.query(Delivery).filter_by(delivery_id=delivery_id).first()
        user = delivery.user if delivery else None
        roscar = delivery.roscar if delivery else None

        return [{
            "event_id": log.event_id,
            "delivery_id": log.delivery_id,
            "timestamp": log.timestamp,
            "event_type": log.new_event.name,
            "user_name": user.user_name if user else None,
            "roscar_namespace": roscar.roscar_namespace if roscar else None,
        } for log in logs]

    @staticmethod
    def get_all_delivery_logs(roscars_sess, log_sess):
        logs = log_sess.query(DeliveryEventLog).order_by(DeliveryEventLog.timestamp).all()

        result = []
        for log in logs:
            delivery = roscars_sess.query(Delivery).filter_by(delivery_id=log.delivery_id).first()
            user = delivery.user if delivery else None
            roscar = delivery.roscar if delivery else None

            result.append({
                "event_id": log.event_id,
                "delivery_id": log.delivery_id,
                "timestamp": log.timestamp,
                "event_type": log.new_event.name,
                "user_name": user.user_name if user else None,
                "roscar_namespace": roscar.roscar_namespace if roscar else None,
            })
        return result

    @staticmethod
    def get_roscar_event_detail(roscars_sess, log_sess):
        logs = log_sess.query(RosCarEventLog).all()
        roscars = roscars_sess.query(RosCars).all()
        roscar_map = {r.roscar_id: r for r in roscars}

        return [{
            "event_id": log.event_id,
            "roscar_id": log.roscar_id,
            "roscar_namespace": roscar_map.get(log.roscar_id).roscar_namespace if log.roscar_id in roscar_map else None,
            "operational_status": roscar_map.get(log.roscar_id).operational_status if log.roscar_id in roscar_map else None,
            "roscar_ip_v4": roscar_map.get(log.roscar_id).roscar_ip_v4 if log.roscar_id in roscar_map else None,
            "cart_id": roscar_map.get(log.roscar_id).cart_id if log.roscar_id in roscar_map else None,
            "cart_ip_v4": roscar_map.get(log.roscar_id).cart_ip_v4 if log.roscar_id in roscar_map else None,
            "event_type": log.event_type.name,
            "event_timestamp": log.event_timestamp
        } for log in logs]

    @staticmethod
    def get_task_event_history(roscars_sess, log_sess):
        logs = log_sess.query(TaskEventLog).order_by(TaskEventLog.changed_at).all()

        task_ids = [log.task_id for log in logs]

        tasks = roscars_sess.query(Task)\
            .filter(Task.task_id.in_(task_ids)).all()

        task_map = {task.task_id: task for task in tasks}
        shoes_model_map = {
            task.task_id: task.shoes_model for task in tasks if task.shoes_model is not None
        }
        location_map = {
            task.task_id: task.location for task in tasks if task.location is not None
        }

        return [{
            "event_id": log.event_id,
            "task_id": log.task_id,
            "status": task_map[log.task_id].status if log.task_id in task_map else None,
            "shoes_name": shoes_model_map.get(log.task_id).name if log.task_id in shoes_model_map else None,
            "location_name": location_map.get(log.task_id).name if log.task_id in location_map else None,
            "previous_event": log.previous_event.name,
            "current_event": log.current_event.name,
            "changed_at": log.changed_at
        } for log in logs]

    @staticmethod
    def get_delivery_log(roscars_sess, log_sess):
        logs = log_sess.query(DeliveryEventLog)\
            .order_by(DeliveryEventLog.timestamp).all()

        delivery_ids = [log.delivery_id for log in logs]

        deliveries = roscars_sess.query(Delivery)\
            .filter(Delivery.delivery_id.in_(delivery_ids)).all()

        delivery_map = {d.delivery_id: d for d in deliveries}
        user_map = {d.delivery_id: d.user for d in deliveries if d.user is not None}
        roscar_map = {d.delivery_id: d.roscar for d in deliveries if d.roscar is not None}

        return [{
            "event_id": log.event_id,
            "delivery_id": log.delivery_id,
            "timestamp": log.timestamp,
            "event_type": log.new_event.name,
            "user_name": user_map.get(log.delivery_id).user_name if log.delivery_id in user_map else None,
            "roscar_namespace": roscar_map.get(log.delivery_id).roscar_namespace if log.delivery_id in roscar_map else None,
            "delivery_status": delivery_map.get(log.delivery_id).delivery_status if log.delivery_id in delivery_map else None,
        } for log in logs]


    @staticmethod
    def get_precision_stop_result(roscars_sess, log_sess):
        logs = log_sess.query(PrecisionStopLog)\
            .order_by(PrecisionStopLog.timestamp).all()

        roscar_ids = list({log.roscar_id for log in logs if log.roscar_id is not None})
        roscars = roscars_sess.query(RosCars)\
            .filter(RosCars.roscar_id.in_(roscar_ids)).all()
        roscar_map = {r.roscar_id: r for r in roscars}

        return [{
            "log_id": log.log_id,
            "roscar_id": log.roscar_id,
            "roscar_namespace": roscar_map.get(log.roscar_id).roscar_namespace if log.roscar_id in roscar_map else None,
            "task_id": log.task_id,
            "is_success": log.is_success,
            "deviation_cm": log.deviation_cm,
            "timestamp": log.timestamp
        } for log in logs]

    @staticmethod
    def get_roscar_trajectory(roscars_sess, log_sess):
        logs = log_sess.query(RoscarTrajectoryLog)\
            .order_by(RoscarTrajectoryLog.timestamp).all()

        roscar_ids = list({log.roscar_id for log in logs if log.roscar_id is not None})
        roscars = roscars_sess.query(RosCars)\
            .filter(RosCars.roscar_id.in_(roscar_ids)).all()
        roscar_map = {r.roscar_id: r for r in roscars}

        return [{
            "trajectory_id": log.trajectory_id,
            "roscar_id": log.roscar_id,
            "roscar_namespace": roscar_map.get(log.roscar_id).roscar_namespace if log.roscar_id in roscar_map else None,
            "task_id": log.task_id,
            "timestamp": log.timestamp,
            "position_x": log.position_x,
            "position_y": log.position_y,
            "velocity": log.velocity,
            "heading_angle": log.heading_angle,
        } for log in logs]

    @staticmethod
    def get_roscar_driving_event_log(roscars_sess, log_sess):
        logs = log_sess.query(RosCarDrivingEventLog)\
            .order_by(RosCarDrivingEventLog.timestamp).all()

        roscar_ids = list({log.roscar_id for log in logs if log.roscar_id is not None})
        roscars = roscars_sess.query(RosCars)\
            .filter(RosCars.roscar_id.in_(roscar_ids)).all()
        roscar_map = {r.roscar_id: r for r in roscars}

        return [{
            "event_id": log.event_id,
            "roscar_id": log.roscar_id,
            "roscar_namespace": roscar_map.get(log.roscar_id).roscar_namespace if log.roscar_id in roscar_map else None,
            "driving_event": log.driving_event.name,
            "timestamp": log.timestamp,
        } for log in logs]

    @staticmethod
    def get_sensor_for_training(roscars_sess, log_sess):
        logs = log_sess.query(RoscarSensorFusionRawLog)\
            .order_by(RoscarSensorFusionRawLog.timestamp).all()

        roscar_ids = list({log.roscar_id for log in logs if log.roscar_id is not None})
        roscars = roscars_sess.query(RosCars)\
            .filter(RosCars.roscar_id.in_(roscar_ids)).all()
        roscar_map = {r.roscar_id: r for r in roscars}

        return [{
            "sensor_log_id": log.sensor_log_id,
            "roscar_id": log.roscar_id,
            "roscar_namespace": roscar_map.get(log.roscar_id).roscar_namespace if log.roscar_id in roscar_map else None,
            "timestamp": log.timestamp,
            "lidar_raw": log.lidar_raw,
            "imu_data": log.imu_data,
            "ultrasonic_data": log.ultrasonic_data,
            "camera_frame_id": log.camera_frame_id,
        } for log in logs]

    @staticmethod
    def get_control_command_log(roscars_sess, log_sess):
        logs = log_sess.query(ControlCommandLog)\
            .order_by(ControlCommandLog.timestamp).all()

        roscar_ids = list({log.roscar_id for log in logs if log.roscar_id is not None})
        roscars = roscars_sess.query(RosCars)\
            .filter(RosCars.roscar_id.in_(roscar_ids)).all()
        roscar_map = {r.roscar_id: r for r in roscars}

        return [{
            "command_id": log.command_id,
            "roscar_id": log.roscar_id,
            "roscar_namespace": roscar_map.get(log.roscar_id).roscar_namespace if log.roscar_id in roscar_map else None,
            "timestamp": log.timestamp,
            "linear_velocity": log.linear_velocity,
            "angular_velocity": log.angular_velocity,
            "control_source": log.control_source,
        } for log in logs]

    @staticmethod
    def get_filesystem_log(roscars_sess, log_sess):
        logs = log_sess.query(FileSystemLog).order_by(FileSystemLog.timestamp).all()

        roscar_ids = list({log.roscar_id for log in logs if log.roscar_id is not None})
        roscars = roscars_sess.query(RosCars).filter(RosCars.roscar_id.in_(roscar_ids)).all()
        roscar_map = {r.roscar_id: r for r in roscars}

        return [{
            "log_id": log.log_id,
            "roscar_id": log.roscar_id,
            "roscar_namespace": roscar_map.get(log.roscar_id).roscar_namespace if log.roscar_id in roscar_map else None,
            "file_path": log.file_path,
            "timestamp": log.timestamp,
        } for log in logs]

    @staticmethod
    def get_rack_sensor_log(roscars_sess, log_sess):
        logs = log_sess.query(RackSensorLog).order_by(RackSensorLog.timestamp).all()

        roscar_ids = list({log.roscar_id for log in logs if log.roscar_id is not None})
        roscars = roscars_sess.query(RosCars).filter(RosCars.roscar_id.in_(roscar_ids)).all()
        roscar_map = {r.roscar_id: r for r in roscars}

        return [{
            "sensor_log_id": log.sensor_log_id,
            "roscar_id": log.roscar_id,
            "roscar_namespace": roscar_map[log.roscar_id].roscar_namespace if log.roscar_id in roscar_map else None,
            "rack_id": log.rack_id,
            "rack_status": log.rack_status,
            "rack_position_x": log.rack_position_x,
            "rack_position_y": log.rack_position_y,
            "rack_position_z": log.rack_position_z,
            "timestamp": log.timestamp,
        } for log in logs]
