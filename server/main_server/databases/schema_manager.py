from sqlalchemy import inspect, text, Engine
from server.main_server.databases.database_manager import DatabaseManager
from server.main_server.databases.models.roscars_models import RoscarsBase
from server.main_server.databases.models.roscars_log_models import RoscarsLogBase
from server.main_server.databases.seed_data_loader import SeedDataLoader

class SchemaManager:
    def __init__(self, db_manager: DatabaseManager):
        self.db = db_manager
        self.roscars_engine = self.db.get_engine("roscars")
        self.roscars_log_engine = self.db.get_engine("roscars_log")
    
    def load_seed_data(self):
        session = self.db.get_session("roscars")
        loader = SeedDataLoader(session)
        loader.load_all()

    def drop_all_tables(self, engine: Engine):
        inspector = inspect(engine)
        table_names = inspector.get_table_names()

        with engine.connect() as conn:
            trans = conn.begin()
            try:
                conn.execute(text("SET FOREIGN_KEY_CHECKS = 0;"))
                for table in table_names:
                    conn.execute(text(f"DROP TABLE IF EXISTS `{table}`"))
                conn.execute(text("SET FOREIGN_KEY_CHECKS = 1;"))
                trans.commit()
                print(f"{engine.url.database}의 모든 테이블 삭제 완료")
            except Exception as e:
                trans.rollback()
                print(f"테이블 삭제 중 오류 발생: {e}")

    def recreate_all_tables(self):
        print("테이블 구조 불일치: 기존 테이블 삭제 후 재생성 시작...")
        
        self.drop_all_tables(self.roscars_engine)
        self.drop_all_tables(self.roscars_log_engine)
        RoscarsBase.metadata.create_all(bind=self.roscars_engine)
        RoscarsLogBase.metadata.create_all(bind=self.roscars_log_engine)
        
        print("테이블 재생성 완료")

    def check_db_init(self) -> bool:
        try:
            main_engine = self.roscars_engine
            log_engine = self.roscars_log_engine

            insp_main = inspect(main_engine)
            insp_log = inspect(log_engine)

            existing_main = set(insp_main.get_table_names())
            expected_main = set(RoscarsBase.metadata.tables.keys())

            existing_log = set(insp_log.get_table_names())
            expected_log = set(RoscarsLogBase.metadata.tables.keys())

            if existing_main == expected_main and existing_log == expected_log:
                print("DB 구조 일치")
                return True
            else:
                print("DB 구조 불일치: 재초기화 수행")
                self.recreate_all_tables()
                return True
        except Exception as e:
            print(f"DB 초기화 확인 중 오류: {e}")
            return False
