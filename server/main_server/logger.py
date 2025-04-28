# logger.py

import logging
import os

# 로그 파일 저장 위치
LOG_FILE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "server_log.txt")

# 로거 설정
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler(LOG_FILE_PATH, encoding="utf-8"),
        logging.StreamHandler()
    ]
)

def log_info(message):
    logging.info(message)

def log_warning(message):
    logging.warning(message)

def log_error(message):
    logging.error(message)
