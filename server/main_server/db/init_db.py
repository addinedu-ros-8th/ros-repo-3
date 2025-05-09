from connect_db import (
    roscars_engine, roscars_log_engine,
    RoscarsSession, RoscarsLogSession
)

from connect_db import check_db_init
from roscars_models import RoscarsBase
from roscars_log_models import RoscarsLogBase
import query_service

# 테이블 생성
def create_all_tables():
    print("Creating tables for roscars...")
    RoscarsBase.metadata.create_all(bind=roscars_engine)
    print("Creating tables for roscars_log...")
    RoscarsLogBase.metadata.create_all(bind=roscars_log_engine)

# Seed 데이터 삽입
def insert_seed_data():
    main_session = RoscarsSession()
    log_session = RoscarsLogSession()

    try:
        if hasattr(query_service, "insert_default_users"):
            query_service.insert_default_users(main_session)

        if hasattr(query_service, "insert_sample_logs"):
            query_service.insert_sample_logs(log_session)

        main_session.commit()
        log_session.commit()
        print("Seed 데이터 삽입 완료")
    except Exception as e:
        main_session.rollback()
        log_session.rollback()
        print("Seed 데이터 삽입 중 오류:", e)
    finally:
        main_session.close()
        log_session.close()

# Query 테스트
def run_sample_queries():
    log_session = RoscarsLogSession()
    main_session = RoscarsSession()

    try:
        result = query_service.get_roscar_event_detail(log_session)
        print("로봇 이벤트 로그:")
        for row in result:
            print(row)

        result = query_service.get_task_event_history(log_session)
        print("작업 이벤트 로그:")
        for row in result:
            print(row)

    finally:
        log_session.close()
        main_session.close()

# 전체 실행
if __name__ == "__main__":
    check_db_init()
    # insert_seed_data()
    # run_sample_queries()
