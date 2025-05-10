import os
import sys
import subprocess
import signal
import time

def validate_yaml_file(yaml_path):
    """
    YAML 경로가 존재하고 .yaml 확장자인지 확인
    """
    if not os.path.isfile(yaml_path):
        print(f"[오류] 파일이 존재하지 않습니다: {yaml_path}")
        return False

    if not yaml_path.endswith('.yaml') and not yaml_path.endswith('.yml'):
        print(f"[오류] 올바른 YAML 파일이 아닙니다: {yaml_path}")
        return False

    return True

def launch_domain_bridge(yaml_path):
    """
    주어진 YAML 설정 파일을 기반으로 도메인 브리지 실행
    """
    if not validate_yaml_file(yaml_path):
        return

    try:
        process = subprocess.Popen([
            'ros2', 'run', 'domain_bridge', 'domain_bridge_main',
            '--config', yaml_path
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        print(f"[성공] 도메인 브리지 실행됨: {yaml_path}")
        print(f"[PID] {process.pid}")

        # 추후 관리 위해 process 반환 가능
        return process

    except FileNotFoundError:
        print("[실패] 'ros2' 명령어를 찾을 수 없습니다. ROS 2 환경을 확인하세요.")
    except Exception as e:
        print(f"[실패] 도메인 브리지 실행 중 예외 발생: {e}")

def main():
    if len(sys.argv) != 2:
        print("사용법: python3 launch_domain_bridge.py <yaml_config_path>")
        sys.exit(1)

    yaml_config_path = sys.argv[1]
    launch_domain_bridge(yaml_config_path)

if __name__ == "__main__":
    main()
