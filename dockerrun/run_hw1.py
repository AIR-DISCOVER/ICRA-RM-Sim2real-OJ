import re
import time
import uuid

import docker
from IPython import embed


def run(student_id: str, vis=False):
    client = docker.from_env()
    id = uuid.uuid1()
    # id = 'test'
    # print(id)
    if student_id == 'Ground Truth':
        client_image = f"docker.discover-lab.com:55555/rmus-2022-fall-answer/client:hw1-dev"
    else:
        client_image = f"docker.discover-lab.com:55555/{student_id}/client:hw1"
    server_image = f"docker.discover-lab.com:55555/rmus-2022-fall/sim-headless:hw1-batch"
    client_assist_image = f"docker.discover-lab.com:55555/rmus-2022-fall-answer/client-assist:hw1"
    
    try:
        client.images.pull(client_image)
    except:
        return None
    client.images.pull(server_image)
    client.images.pull(client_assist_image)
    
    states = []

    try:
        network = client.networks.create(f"net-{id}")
        ros_master = client.containers.run(
            'ros:noetic-ros-core-focal',
            'roscore',
            tty=True,
            stdin_open=True,
            remove=True,
            detach=True,
            name=f"rosmaster-{id}",
            network=f"net-{id}",
        )
        server = client.containers.run(
            server_image,
            tty=True,
            stdin_open=True,
            remove=True,
            detach=True,
            name=f"sim-{id}",
            network=f"net-{id}",
            environment=[
                f"ROS_MASTER_URI=http://rosmaster-{id}:11311",
                "NVIDIA_VISIBLE_DEVICES=all",
            ],
            runtime='nvidia',
        )
        hw_assist = client.containers.run(
            client_assist_image,
            tty=True,
            stdin_open=True,
            remove=True,
            detach=True,
            name=f"cli-assist-{id}",
            network=f"net-{id}",
            environment=[
                f"ROS_MASTER_URI=http://rosmaster-{id}:11311",
            ],
        )
        hw = client.containers.run(
            client_image,
            tty=True,
            stdin_open=True,
            remove=True,
            detach=True,
            name=f"cli-{id}",
            network=f"net-{id}",
            environment=[
                f"ROS_MASTER_URI=http://rosmaster-{id}:11311",
            ],
        )
        if vis:
            vis = client.containers.run(
                'docker.discover-lab.com:55555/rmus-2022-fall/ros-gui',
                '/opt/ros/noetic/env.sh rosrun image_view image_view image:=/third_rgb',
                tty=True,
                stdin_open=True,
                remove=True,
                detach=True,
                name=f"vis-{id}",
                network=f"net-{id}",
                environment=[
                    f"ROS_MASTER_URI=http://rosmaster-{id}:11311",
                    "QT_X11_NO_MITSHM=1",
                    "DISPLAY=:1",
                ],
                volumes=['/tmp/.X11-unix:/tmp/.X11-unix']
            )
        
        def remove():    
            try:
                vis.kill()
            except Exception as e:
                print(e)
            try:
                hw.kill()
            except Exception as e:
                print(e)
            try:
                hw_assist.kill()
            except Exception as e:
                print(e)
            try:
                server.kill()
            except Exception as e:
                print(e)
            try:
                ros_master.kill()
            except Exception as e:
                print(e)
            try:
                network.remove()
            except Exception as e:
                print(e)
        def run():
            # while True:
            #     ...
            time.sleep(5)
            ros_master.exec_run(
                '''/opt/ros/noetic/env.sh rostopic pub -1 /reset geometry_msgs/Point "x: 0.0
y: 0.0
z: 0.0"''',
                stdin=True,
                tty=True,
                detach=True,
            )
            time.sleep(5)
            for i in range(10):
                ros_master.exec_run(
                    '''/opt/ros/noetic/env.sh rostopic pub -1 /reset geometry_msgs/Point "x: 0.0
y: 0.0
z: 0.0"''',
                    stdin=True,
                    tty=True,
                    detach=True,
                )
                time.sleep(3)
                ros_master.exec_run(
                    '''/opt/ros/noetic/env.sh rostopic pub -1 /arm_gripper geometry_msgs/Point "x: 1.0
y: 0.0
z: 0.0"''',
                    stdin=True,
                    tty=True,
                    detach=True,
                )
                try:
                    # print(f"{i}-start")
                    a = ros_master.exec_run(
                        '/opt/ros/noetic/env.sh rostopic echo -n 1 /gripper_state',
                        stdin=True,
                        tty=True,
                    )
                    # print(f"{i}-end")
                    a = a.output.decode('utf-8')
                    s = re.findall(r"x:\s(\d)\.0", a)
                    result = int(s[0])
                    if result == 1:
                        states.append(True)
                        print('Pass', end='\t')
                    else:
                        states.append(False)
                        print('Fail', end='\t')
                except:
                    states.append(False)
                    print('Fail', end='\t')

        run()
        remove()
    except:
        remove()
        raise

    return states


if __name__ == "__main__":
    id_list = [
'2020011633',
'2019011017',
'2020013359',
'2021010101',
'2021011028',
'2021011030',
'2021011040',
'2021013034',
'2021013036',
'2021013432',
'2021013445',
'2021013446',
'2021013467',
'2021013484',
'2021013492',
'2021013495',
'2021013498',
'2020013366',
'2020013434',
'2021011039',
'2021013021',
'2021013039',
'2021013474',
    ]
    for id in id_list:
        print(id, end='\t')
        result = run(id, vis=True)
        if result is None:
            print("Not Submitted")
        else:
            print(f"{sum(result)}")
