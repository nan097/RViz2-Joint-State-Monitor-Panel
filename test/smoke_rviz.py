#!/usr/bin/env python3
"""启动已安装的 RViz、示例配置和模拟发布器，检查进程启停。
需要可用桌面和 OpenGL，不改写用户保存的配置。
界面数值和宿主数据接收的断言由 rviz_host_smoke 完成。
"""
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    env = dict(os.environ, QT_QPA_PLATFORM='xcb')
    env.setdefault('ROS_LOG_DIR', '/tmp/joint-panel-smoke-logs')
    prefix = subprocess.check_output(['ros2', 'pkg', 'prefix', 'joint_state_rviz_panel'], text=True).strip()
    config = str(Path(prefix) / 'share/joint_state_rviz_panel/rviz/joint_monitor.rviz')
    log_path = '/tmp/joint-panel-rviz-smoke.log'
    with open(log_path, 'w') as log:
        processes = []
        try:
            pub = subprocess.Popen(['ros2', 'run', 'joint_state_rviz_panel', 'mock_joint_states_publisher.py', '--count', '14', '--hz', '100', '--shuffle'], env=env, stdout=log, stderr=log, start_new_session=True)
            processes.append(pub)
            rviz = subprocess.Popen(['rviz2', '-d', config, '--ros-args', '-r', '__node:=joint_panel_rviz_smoke'], env=env, stdout=log, stderr=log, start_new_session=True)
            processes.append(rviz)
            time.sleep(8)
            assert rviz.poll() is None and pub.poll() is None, f'Exited early; see {log_path}'
            os.killpg(rviz.pid, signal.SIGINT)
            assert rviz.wait(timeout=10) == 0, f'RViz shutdown failed; see {log_path}'
            print(f'PASS: installed RViz started with panel config + 14-joint mock, stayed alive and exited cleanly. Log: {log_path}')
        finally:
            for process in reversed(processes):
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()


if __name__ == '__main__':
    main()
