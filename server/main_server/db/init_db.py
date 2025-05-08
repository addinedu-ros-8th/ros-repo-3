from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker

from roscars_models import RoscarsBase
from roscars_log_models import RoscarsLogBase

from dotenv import load_dotenv

import query_service
import os

load_dotenv()

DB_HOST = os.getenv("DB_HOST")
DB_USER = os.getenv("DB_USER")
DB_PSWD = os.getenv("DB_PSWD")

DB_URI = f"mysql+pymysql://{DB_USER}:{DB_PSWD}@{DB_HOST}"

ROSCARS_URL = f"{DB_URI}/roscars"
ROSCARS_LOG_URL = f"{DB_URI}/roscars_log"

# 엔진 생성
roscars_engine = create_engine(ROSCARS_URL)
roscars_log_engine = create_engine(ROSCARS_LOG_URL)
# 각각의 SessionLocal
RoscarsSession = sessionmaker(bind=roscars_engine)
RoscarsLogSession = sessionmaker(bind=roscars_log_engine)

# 테이블 생성
def create_all_tables():
    RoscarsBase.metadata.create_all(bind=roscars_engine)
    RoscarsLogBase.metadata.create_all(bind=roscars_log_engine)

# TODO: seed 데이터 삽입
def insert_seed_data():
    session = RoscarsSession()
    try:
        # TODO: 사용자 추가
        
        # TODO: 로봇 추가
        
        session.commit()
    except Exception as e:
        session.rollback()
        print("Seed 데이터 삽입 중 오류:", e)
    finally:
        session.close()

# query_service 호출 예시
def run_sample_queries():
    log_session = RoscarsLogSession()
    main_session = RoscarsSession()
    try:
        # 로봇 이름 포함 이벤트 로그 확인
        result = query_service.get_roscar_event_detail(log_session)
        print("로봇 이벤트 로그:")
        for row in result:
            print(row)

        # 작업 이벤트 로그 출력
        result = query_service.get_task_event_history(log_session)
        print("작업 이벤트 로그:")
        for row in result:
            print(row)

    finally:
        log_session.close()
        main_session.close()

# 전체 실행
if __name__ == "__main__":
    create_all_tables()
    # insert_seed_data()
    run_sample_queries()
