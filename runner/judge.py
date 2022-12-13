import logging
import os
import re
import time
import uuid

import docker

from .timeout import TimeoutException, time_limit
from .upload_log import upload_log

class Runner:
    def __init__(self, server_image, client_image, display:str=None, run_type=1) -> None:
        self.server_image = server_image
        self.client_image = client_image
        self.id = uuid.uuid1()
        self.logger = logging.getLogger(client_image)
        self.type = run_type
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
        os.makedirs(f'/tmp/save_video/{self.id}', exist_ok=True)
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
                f"/tmp/save_video/{self.id}:/tmp/save_video"
            ],
        )
        self.logger.info(f"Server started on DISPLAY {self.DISPLAY}")
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
        print("Run task type: ", self.type)
        if self.type != 1:
            result = self.server.exec_run(
                r'''/opt/ros/noetic/env.sh /opt/workspace/devel/env.sh rostopic pub -1 /reset geometry_msgs/Point 0.0 0.0 0.0''',
                stdin=True,
                tty=True,
                detach=True,
            )
            print(result)
            time.sleep(5)

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
        self.logger.info(f"Client started on DISPLAY {self.DISPLAY}")

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
        self.server.exec_run(
            fr'''/opt/ros/noetic/env.sh /opt/workspace/devel/env.sh rosbag record /third_rgb -O /tmp/save_video/out.bag __name:=record''',
            stdin=True,
            tty=True,
            detach=True,
        )
        self.logger.info("Start recording")
        for i in range(180):
            file = self.ros_master.exec_run(
                '/opt/ros/noetic/env.sh rostopic echo -n 1 /judgement/markers_time',
                stdin=True,
                tty=True,
                socket=True
            ).output
            result = ''
            for _ in range(20):
                if not result:
                    result = file.readline()
                else:
                    break
            if not result:
                assert False
            result = result.decode('utf-8')
            self.logger.info("Counter #" + str(i) + str(result))
            
            matched_list = re.findall(r"data\:\s\"(.*?),\s(.*?),\s(.*?)\"", result)
            if not len(matched_list) == 1:
                self.logger.warning("Data published by /judgement/markers_time has invalid form")
            matched_result = matched_list[0]
            if matched_result[0] == "None" or matched_result[1] == "None" or matched_result[2] == "None":
                result = matched_result
                break
            else:
                result = [float(i) for i in matched_result]
                if min(result) > 1e-5:
                    break
            time.sleep(1)
            result = [float('inf') if i < 1e-5 else i for i in result] # rosnode kill /my_bag
        self.server.exec_run(
            fr'''/opt/ros/noetic/env.sh /opt/workspace/devel/env.sh rosnode kill /record''',
            stdin=True,
            tty=True,
            # detach=True,
        )
        # self.server.exec_run(
        #     fr'''/opt/ros/noetic/env.sh /opt/workspace/devel/env.sh rosbag reindex /tmp/save_video/out_f.bag.active''',
        #     stdin=True,
        #     tty=True,
        #     # detach=True,
        # )
        # self.server.exec_run(
        #     fr'''/opt/ros/noetic/env.sh /opt/workspace/devel/env.sh rosbag fix /tmp/save_video/out_f.bag.active /tmp/save_video/out_f.bag''',
        #     stdin=True,
        #     tty=True,
        #     # detach=True,
        # )
        out = self.server.exec_run(
            fr'''/opt/ros/noetic/env.sh /opt/workspace/devel/env.sh python3 /opt/record.py  -o /tmp/save_video/final.mp4 /tmp/save_video/out.bag''',
            stdin=True,
            tty=True,
            # detach=True,
        )
        time.sleep(5)
        print(out.output.decode('utf-8'))
        
        server_log = self.server.logs().decode('utf-8')
        client_log = self.client.logs().decode('utf-8')
        
        return result, server_log, client_log
    
    def clean(self):
        import shutil
        shutil.rmtree(f'/tmp/save_video/{self.id}')

def run(client_image: str, display: str = None, vis=False, run_id=None, run_type=1, online=True):
    try:
        # server_image = "docker.discover-lab.com:55555/rm-sim2real/server:latest"
        server_image = 'server-final:latest'
        runner = Runner(server_image, client_image, display=display, run_type=run_type)
        runner.create(vis=vis, wait_sec=15)
        result, server_log, client_log = runner.run()
        if online:
            print(upload_log(run_id, server_log=server_log, client_log=client_log))
            print(upload_log(run_id, video=f'/tmp/save_video/{runner.id}/final.mp4'))
        # runner.clean()
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
    client_image = "client-test:latest"
    # client_image = "docker.discover-lab.com:55555/test/client:v3.0.0"
    print(run(client_image, run_type=2, run_id=172))
