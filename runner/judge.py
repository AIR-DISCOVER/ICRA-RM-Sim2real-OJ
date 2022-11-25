import logging
import os
import re
import time
import uuid

import docker

from .timeout import TimeoutException, time_limit
from .upload_log import upload_log

class Runner:
    def __init__(self, server_image, client_image, display:str=None) -> None:
        self.server_image = server_image
        self.client_image = client_image
        self.id = uuid.uuid1()
        self.logger = logging.getLogger(client_image)
        self.DISPLAY = display if display is not None else os.environ['DISPLAY']
        
        self.docker_exe = docker.from_env()

        try:
            self.docker_exe.images.pull(server_image)
        except:
            self.logger.error("Server image is not found.")
        try:
            self.docker_exe.images.pull(client_image)
        except:
            self.logger.error("Client image is not found.")

    def create(self, vis=False, wait_sec=15, debug=False):
        self.network = self.docker_exe.networks.create(f"net-{self.id}")
        self.ros_master = self.docker_exe.containers.run(
            'ros:noetic-ros-core-focal',
            'roscore',
            tty=True,
            stdin_open=True,
            remove=True,
            detach=True,
            name=f"rosmaster-{self.id}",
            network=f"net-{self.id}",
        )
        self.logger.info("Core started")
        self.server = self.docker_exe.containers.run(
            self.server_image,
            tty=True,
            stdin_open=True,
            remove=True, # FIXME
            detach=True,
            runtime='nvidia',
            name=f"server-{self.id}",
            network=f"net-{self.id}",
            environment=[
                f"ROS_MASTER_URI=http://rosmaster-{self.id}:11311",
                f"DISPLAY={self.DISPLAY}",
                "QT_X11_NO_MITSHM=1",
                "NO_AT_BRIDGE=1",
                "LIBGL_ALWAYS_SOFTWARE=1",
                "NVIDIA_VISIBLE_DEVICES=all",
                "NVIDIA_DRIVER_CAPABILITIES=all",
            ],
            volumes=[
                "/tmp/.X11-unix:/tmp/.X11-unix",
            ],
        )
        self.logger.info("Server started")
        if vis:
            self.server.exec_run(
                '''/opt/ros/noetic/env.sh rosrun image_view image_view image:=/third_rgb''',
                stdin=True,
                tty=True,
                detach=True,
            )
            self.logger.info("Visualization started")
        if wait_sec is not None:
            time.sleep(wait_sec)
        self.client = self.docker_exe.containers.run(
            self.client_image,
            command=None if not debug else "bash",
            cpuset_cpus="5",
            mem_limit="8192m",
            tty=True,
            stdin_open=True,
            remove=True,
            detach=True,
            name=f"client-{self.id}",
            network=f"net-{self.id}",
            environment=[
                f"ROS_MASTER_URI=http://rosmaster-{self.id}:11311",
                f"DISPLAY={self.DISPLAY}",
                "QT_X11_NO_MITSHM=1",
                "NO_AT_BRIDGE=1",
                "LIBGL_ALWAYS_SOFTWARE=1",
            ],
            volumes=[
                "/tmp/.X11-unix:/tmp/.X11-unix",
            ],
        )
        self.logger.info("Client started")

    def remove(self):
        def try_kill(container):
            try:
                container.kill()
            except Exception as e:
                self.logger.warning(e)
        try_kill(self.client)
        try_kill(self.server)
        try_kill(self.ros_master)
        try:
            self.network.remove()
        except Exception as e:
            self.logger.warning(e)

    def run(self):
        result = [float('inf') for _ in range(3)]
        self.client.exec_run(
            '''/opt/ros/noetic/env.sh /opt/workspace/devel_isolated/env.sh /opt/ep_ws/devel/env.sh rostopic pub -1 /reset geometry_msgs/Point "x: 0.0
y: 0.0
z: 0.0"''',
            stdin=True,
            tty=True,
        ).output.decode('utf-8')
        for i in range(300):
            result = self.ros_master.exec_run(
                '/opt/ros/noetic/env.sh rostopic echo -n 1 /judgement/markers_time',
                stdin=True,
                tty=True,
            ).output.decode('utf-8')
            self.logger.info(str(i) + result)
            
            matched_list = re.findall(r"data\:\s\"(.*?),\s(.*?),\s(.*?)\"", result)
            if not len(matched_list) == 1:
                self.logger.warning("Data published by /judgement/markers_time has invalid form")
            matched_result = matched_list[0]
            if matched_result[0] == "None":
                result = "Illegal"
                break
            else:
                result = [float(i) for i in matched_result]
                if min(result) > 1e-5:
                    result = max(result)
                    break
            time.sleep(1)
            result = [float('inf') for i in result if i < 1e-5]
        server_log = self.server.logs().decode('utf-8')
        client_log = self.client.logs().decode('utf-8')
        return result, server_log, client_log

def run(client_image: str, display: str = None, vis=False, run_id=None  ):
    try:
        server_image = "docker.discover-lab.com:55555/rm-sim2real/server:latest"
        runner = Runner(server_image, client_image, display=display)
        runner.create(vis=vis, wait_sec=15)
        # with time_limit(360):
        result, server_log, client_log = runner.run()
        upload_log(run_id, server_log, client_log)
    except TimeoutException:
        result = "timeout"
    except Exception as e:
        print(type(e))
        print(e)
        result = 'error'
    finally:
        try:
            runner.remove()
        except:
            ...
            
    return result


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    client_image = "docker.discover-lab.com:55555/rm-sim2real/client:tagtest"
    print(run(client_image, 1))
