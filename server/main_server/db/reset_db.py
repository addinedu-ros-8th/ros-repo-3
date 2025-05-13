# reset_db.py
import os
import sys

current_dir = os.path.abspath(os.path.dirname(__file__))
project_root = os.path.abspath(os.path.join(current_dir, '..'))

for path in [current_dir, project_root]:
    if path not in sys.path:
        sys.path.insert(0, path)

from db.connect_db import drop_all_tables, recreate_all_tables, get_engine

if __name__ == '__main__':
    print("⚠️ 모든 테이블을 삭제하고 재생성합니다...")
    drop_all_tables(get_engine("roscars"))
    drop_all_tables(get_engine("roscars_log"))
    recreate_all_tables()
    print("✅ 모든 테이블 재생성 완료")
