import logging
from pydoc import cli
import uuid
from .judge import run
import zerorpc
import threading

from .upload_log import upload_log

class TaskManager(object):
    def __init__(self) -> None:
        self.task_list = {}
        self.finished_task = {}
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def add_task(self, client_image, testrun_id):
        if not type(client_image) == str:
            return 'invalid'
        id = str(uuid.uuid1())
        current_waiting = [(k, v) for k, v in self.task_list.items()]
        for old_id, old_task in current_waiting:
            if client_image == old_task["image"]:
                if old_task['status'] == "waiting":
                    self.task_list.pop(old_id)
                    self.task_list[id] = {"image": client_image, "status": "waiting", "trigger_run_id": testrun_id}
                    return id
                elif old_task['status'] == 'running':
                    return 'invalid'
                elif old_task['status'] == 'finished':
                    self.task_list[id] = {"image": client_image, "status": "waiting", "trigger_run_id": testrun_id}
                    return id
                else:
                    return 'invalid'
        self.task_list[id] = {"image": client_image, "status": "waiting", "trigger_run_id": testrun_id}
        return id
    
    def get_task_list(self):
        return self.task_list
    
    def clear(self):
        self.task_list = {k: v for k, v in self.task_list.items() if v['status'] != "finished"}
    
    def query_task(self, id):
        if not type(id) == str:
            return None, None
        if id not in self.task_list.keys():
            return "not found", None
        return self.task_list[id]["status"], self.task_list[id].get("result", None)
    
    def run(self):
        while True:
            try:
                todo = [k for k, v in self.task_list.items() if v['status'] == 'waiting']
                if len(todo) > 0:
                    id = todo[0]
                    self.task_list[id]["status"] = "running"
                    result = run(self.task_list[id]["image"], run_id=self.task_list[id]["trigger_run_id"])
                    print(result)
                    self.task_list[id]["status"] = "finished"
                    self.task_list[id]["result"] = result
            except Exception as e:
                print(type(e))
                print(e)


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    s = zerorpc.Server(TaskManager())
    s.bind("tcp://127.0.0.1:12034")
    try:
        s.run()
    except KeyboardInterrupt as e:
        ...