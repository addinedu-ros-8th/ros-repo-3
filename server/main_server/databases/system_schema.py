from server.main_server.databases.schema.system_schema_function import SystemSchemaFunction

class SystemSchema:
    def __init__(self, db):
        self.db = db
        self.system_function = SystemSchemaFunction

    def update_roscar_status(self, namespace: str, battery: float, ip: str | None = None):
        return self.system_function.upsert_roscar_status(self.db, namespace, battery, ip)

    def save_sensor_data_log(self, namespace: str, timestamp, parsed: dict) -> tuple[bool, int]:
        return self.system_function.save_sensor_fusion_data(self.db, namespace, timestamp, parsed)
