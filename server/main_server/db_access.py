import os
from datetime import datetime
import mysql.connector
from dotenv import load_dotenv
from server.main_server.logger import log_info, log_error

load_dotenv()

DB_HOST = os.getenv("DB_HOST")
DB_USER = os.getenv("DB_USER")
DB_PASSWORD = os.getenv("DB_PASSWORD")
DB_NAME = os.getenv("DB_NAME")

class DatabaseAccessor:
    def __init__(self):
        self.connection = None

    def connect(self):
        try:
            self.connection = mysql.connector.connect(
                host=DB_HOST,
                user=DB_USER,
                password=DB_PASSWORD,
                database=DB_NAME
            )
        except mysql.connector.Error as err:
            log_error(f"[DatabaseAccessor] Database connection error: {err}")
            raise

    def close(self):
        if self.connection:
            self.connection.close()

    def insert_task(self, task_info):
        """작업(Task)을 데이터베이스에 삽입한다."""
        try:
            self.connect()
            cursor = self.connection.cursor()
            query = """
                INSERT INTO RequestTask (roscar_id, task_id, origin, quantity, status, task_start_time)
                VALUES (%s, %s, %s, %s, %s, %s)
            """
            now_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            values = (
                task_info.get("roscar_id", None),
                task_info.get("task_id", None),
                task_info.get("origin", None),
                task_info.get("quantity", ""),
                task_info.get("status", "Pending"),
                now_time
            )
            cursor.execute(query, values)
            self.connection.commit()
            return cursor.lastrowid
        except Exception as e:
            log_error(f"[DatabaseAccessor] insert_task Error: {str(e)}")
            raise
        finally:
            self.close()

    def fetch_all_tasks(self):
        """모든 Task를 가져온다."""
        try:
            self.connect()
            cursor = self.connection.cursor(dictionary=True)
            query = "SELECT * FROM Task ORDER BY task_start_time DESC"
            cursor.execute(query)
            tasks = cursor.fetchall()
            for task in tasks:
                if 'task_start_time' in task and task['task_start_time']:
                    try:
                        if isinstance(task['task_start_time'], datetime):
                            task['task_start_time'] = task['task_start_time'].strftime('%Y-%m-%d %H:%M:%S')
                        else:
                            task['task_start_time'] = datetime.strptime(str(task['task_start_time']), '%Y-%m-%d %H:%M:%S').strftime('%Y-%m-%d %H:%M:%S')
                    except Exception as e:
                        log_error(f"[DatabaseAccessor] fetch_all_tasks Time Parse Error: {e}")
            return tasks
        except Exception as e:
            log_error(f"[DatabaseAccessor] fetch_all_tasks Error: {str(e)}")
            raise
        finally:
            self.close()

    # 로봇 추가를 위한 메서드
    def add_roscar(self, roscar_name, roscar_ip):
        """새로운 로봇을 데이터베이스에 추가한다."""
        try:
            self.connect()
            cursor = self.connection.cursor()
            query = """
                INSERT INTO Robots (name, ip, status)
                VALUES (%s, %s, 'IDLE')
            """
            values = (roscar_name, roscar_ip)
            cursor.execute(query, values)
            self.connection.commit()
            log_info(f"[DatabaseAccessor] Added roscar: {roscar_name} with IP: {roscar_ip}")
            return cursor.lastrowid  # 새로 삽입된 로봇의 ID 반환
        except Exception as e:
            log_error(f"[DatabaseAccessor] add_roscar Error: {str(e)}")
            raise
        finally:
            self.close()

    def search_user_by_name(self, name):
        """사용자 이름으로 검색"""
        try:
            self.connect()
            cursor = self.connection.cursor(dictionary=True)
            query = "SELECT * FROM Users WHERE name = %s"
            cursor.execute(query, (name,))
            user = cursor.fetchone()
            return user
        except Exception as e:
            log_error(f"[DatabaseAccessor] search_user_by_name Error: {str(e)}")
            raise
        finally:
            self.close()

    def get_user_gui_data(self, user_id):
        """사용자 GUI 데이터를 가져온다."""
        try:
            self.connect()
            cursor = self.connection.cursor(dictionary=True)
            query = "SELECT * FROM UserGuiData WHERE user_id = %s"
            cursor.execute(query, (user_id,))
            gui_data = cursor.fetchall()
            return gui_data
        except Exception as e:
            log_error(f"[DatabaseAccessor] get_user_gui_data Error: {str(e)}")
            raise
        finally:
            self.close()

    def search_qr_code(self, qr_code):
        """QR 코드로 검색"""
        try:
            self.connect()
            cursor = self.connection.cursor(dictionary=True)
            query = "SELECT * FROM QRCode WHERE qr_code = %s"
            cursor.execute(query, (qr_code,))
            qr_info = cursor.fetchone()
            return qr_info
        except Exception as e:
            log_error(f"[DatabaseAccessor] search_qr_code Error: {str(e)}")
            raise
        finally:
            self.close()

    def search_shoes_data(self, shoes_id):
        """신발 ID로 데이터 검색"""
        try:
            self.connect()
            cursor = self.connection.cursor(dictionary=True)
            query = "SELECT * FROM Shoes WHERE id = %s"
            cursor.execute(query, (shoes_id,))
            shoes_info = cursor.fetchone()
            return shoes_info
        except Exception as e:
            log_error(f"[DatabaseAccessor] search_shoes_data Error: {str(e)}")
            raise
        finally:
            self.close()

    def query_idle_roscars(self):
        """Idle 상태 로봇 목록 가져오기"""
        try:
            self.connect()
            cursor = self.connection.cursor(dictionary=True)
            query = "SELECT * FROM Robots WHERE status = 'IDLE'"
            cursor.execute(query)
            roscars = cursor.fetchall()
            return roscars
        except Exception as e:
            log_error(f"[DatabaseAccessor] query_idle_roscars Error: {str(e)}")
            raise
        finally:
            self.close()

    def search_roscar_status(self):
        """로봇 상태 전체 조회"""
        try:
            self.connect()
            cursor = self.connection.cursor(dictionary=True)
            query = "SELECT roscar_id, status FROM Robots"
            cursor.execute(query)
            roscar_status = cursor.fetchall()
            return roscar_status
        except Exception as e:
            log_error(f"[DatabaseAccessor] search_roscar_status Error: {str(e)}")
            raise
        finally:
            self.close()

# Wrapping functions for external imports
db_accessor = DatabaseAccessor()

def insert_task(roscar, task_code, origin, quantity, status, time):
    """viewer에서 import 해서 바로 쓰는 insert_task 함수 (DatabaseAccessor 래핑)"""
    task_info = {
        "roscar_id": roscar,
        "task_id": task_code,
        "origin": origin,
        "quantity": quantity,
        "status": status,
        "task_start_time": time
    }
    db_accessor.insert_task(task_info)

def fetch_all_tasks():
    """viewer에서 import 해서 바로 쓰는 fetch_all_tasks 함수 (DatabaseAccessor 래핑)"""
    return db_accessor.fetch_all_tasks()
