from datetime import datetime
import mysql.connector
import os
from dotenv import load_dotenv

load_dotenv()

DB_HOST = os.getenv("DB_HOST")
DB_USER = os.getenv("DB_USER")
DB_PASSWORD = os.getenv("DB_PASSWORD")
DB_NAME = os.getenv("DB_NAME")

def get_connection():
    return mysql.connector.connect(
        host=DB_HOST,
        user=DB_USER,
        password=DB_PASSWORD,
        database=DB_NAME,
    )

def insert_task(robot, task_code, origin, quantity="", status="Pending", time=None):
    if time is None:
        time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')  # ✅ 시:분:초까지 포함된 현재 시간

    conn = get_connection()
    cursor = conn.cursor()
    query = """
        INSERT INTO RequestTask (robot_id, task_id, origin, quantity, status, task_start_time)
        VALUES (%s, %s, %s, %s, %s, %s)
    """
    cursor.execute(query, (robot, task_code, origin, quantity, status, time))
    conn.commit()
    conn.close()

def fetch_all_tasks():
    conn = get_connection()
    cursor = conn.cursor(dictionary=True)
    cursor.execute("SELECT * FROM Task ORDER BY task_start_time DESC")
    tasks = cursor.fetchall()
    conn.close()

    # ✅ time 필드를 문자열로 변환 및 포맷
    for task in tasks:
        if 'task_start_time' in task and task['time']:
            try:
                # MySQL에서 받아온 datetime 객체 혹은 str을 처리
                if isinstance(task['time'], datetime):
                    task['time'] = task['time'].strftime('%Y-%m-%d %H:%M:%S')
                else:
                    task['time'] = datetime.strptime(str(task['time']), '%Y-%m-%d %H:%M:%S').strftime('%Y-%m-%d %H:%M:%S')
            except Exception as e:
                print(f"[시간 파싱 오류] {task['time']}: {e}")
    return tasks
