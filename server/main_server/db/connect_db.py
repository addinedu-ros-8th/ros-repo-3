import os
from dotenv import load_dotenv

from sqlalchemy import create_engine, inspect, text
from sqlalchemy.orm import sessionmaker

from db.roscars_models import RoscarsBase
from db.roscars_log_models import RoscarsLogBase

load_dotenv()

DB_HOST = os.getenv("DB_HOST")
DB_USER = os.getenv("DB_USER")
DB_PSWD = os.getenv("DB_PSWD")

DB_URI = f"mysql+pymysql://{DB_USER}:{DB_PSWD}@{DB_HOST}"

ROSCARS_URL = f"{DB_URI}/roscars"
ROSCARS_LOG_URL = f"{DB_URI}/roscars_log"

roscars_engine = create_engine(ROSCARS_URL, echo=False, pool_pre_ping=True, connect_args={"ssl_disabled": "True"})
roscars_log_engine = create_engine(ROSCARS_LOG_URL, echo=False, pool_pre_ping=True, connect_args={"ssl_disabled": "True"})

RoscarsSession = sessionmaker(bind=roscars_engine, autocommit=False, autoflush=False)
RoscarsLogSession = sessionmaker(bind=roscars_log_engine, autocommit=False, autoflush=False)

def get_db_url(db_name):
    return f"{DB_URI}/{db_name}"

def get_engine(db_name):
    if db_name == "roscars":
        return roscars_engine
    elif db_name == "roscars_log":
        return roscars_log_engine
    else:
        raise ValueError("Invalid database name")

def get_session(engine):
    Session = sessionmaker(bind=engine, autocommit=False, autoflush=False)
    return Session()

def get_roscars_session():
    return get_session(roscars_engine)

def get_roscars_log_session():
    return get_session(roscars_log_engine)

from sqlalchemy import text, inspect
from sqlalchemy.engine import Engine

# 외래 키 제약을 제거하고 테이블을 삭제하는 함수
def drop_all_tables(engine: Engine):
    inspector = inspect(engine)
    table_names = inspector.get_table_names()

    with engine.connect() as conn:
        trans = conn.begin()
        try:
            # 외래 키 제약을 비활성화
            conn.execute(text("SET FOREIGN_KEY_CHECKS = 0;"))
            
            for table in table_names:
                conn.execute(text(f"DROP TABLE IF EXISTS `{table}`"))
            
            # 외래 키 제약을 활성화
            conn.execute(text("SET FOREIGN_KEY_CHECKS = 1;"))
            
            trans.commit()
            print(f"{engine.url.database}의 모든 테이블 삭제 완료")
        except Exception as e:
            trans.rollback()
            print(f"테이블 삭제 중 오류 발생: {e}")

# 테이블을 삭제 후 재생성하는 함수
def recreate_all_tables():
    print("테이블 구조 불일치: 기존 테이블 삭제 후 재생성 시작...")

    # Drop all tables
    drop_all_tables(roscars_engine)
    drop_all_tables(roscars_log_engine)

    # Create all tables
    RoscarsBase.metadata.create_all(bind=roscars_engine)
    RoscarsLogBase.metadata.create_all(bind=roscars_log_engine)

    print("테이블 재생성 완료")


def check_db_init():
    try:
        insp_main = inspect(roscars_engine)
        insp_log = inspect(roscars_log_engine)

        existing_main_tables = set(insp_main.get_table_names())
        existing_log_tables = set(insp_log.get_table_names())

        expected_main_tables = set(RoscarsBase.metadata.tables.keys())
        expected_log_tables = set(RoscarsLogBase.metadata.tables.keys())

        if existing_main_tables == expected_main_tables and existing_log_tables == expected_log_tables:
            print("DB 연결 성공 및 구조 일치")
            return True
        else:
            print("DB 구조 불일치: 재초기화 수행")
            try:
                recreate_all_tables()
                print("DB 재초기화 완료")
                return True
            except Exception as e:
                print(f"DB 재초기화 중 오류 발생: {e}")
                return False
    except Exception as e:
        print(f"DB 초기화 검사 실패: {e}")
        return False

