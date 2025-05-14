from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from dotenv import load_dotenv
import os

class DatabaseManager:
    def __init__(self):
        load_dotenv()
        self.engines = {}
        self.sessions = {}
        self._init_connections()

    def _init_connections(self):
        DB_HOST = os.getenv("DB_HOST")
        DB_USER = os.getenv("DB_USER")
        DB_PSWD = os.getenv("DB_PSWD")

        DB_URI = f"mysql+pymysql://{DB_USER}:{DB_PSWD}@{DB_HOST}"

        self.engines["roscars"] = create_engine(
            f"{DB_URI}/roscars", echo=False, pool_pre_ping=True
        )
        self.engines["roscars_log"] = create_engine(
            f"{DB_URI}/roscars_log", echo=False, pool_pre_ping=True
        )

        self.sessions["roscars"] = sessionmaker(bind=self.engines["roscars"])
        self.sessions["roscars_log"] = sessionmaker(bind=self.engines["roscars_log"])

    def get_engine(self, name: str):
        return self.engines[name]

    def get_session(self, name: str):
        return self.sessions[name]()

    def close_engine(self, name: str):
        self.engines[name].dispose()
