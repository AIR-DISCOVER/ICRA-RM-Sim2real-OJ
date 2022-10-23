import logging
import uuid
from .judge import run
import zerorpc
import threading

class TaskManager(object):
    def __init__(self) -> None:
        self.task_list = {}
        self.finished_task = {}
        thread = threading.Thread(target=self.run)
        thread.start()

    def add_task(self, client_image):
        if not type(client_image) == str:
            return None
        id = uuid.uuid1()
        self.task_list[id] = {"image": client_image, "status": "waiting"}
        return id
    
    def get_task_list(self):
        return self.task_list
    
    def clear(self):
        self.task_list = {k: v for k, v in self.task_list.items() if v['status'] != "finished"}
    
    def query_task(self, id):
        if not type(id) == str:
            return None
        if id not in self.task_list.keys():
            return None
        return self.task_list[id]["status"]
    
    def run(self):
        while True:
            if len(self.task_list) > 0:
                id = list(self.task_list.keys())[0]
                self.task_list[id]["status"] = "running"
                run(self.task_list[id]["image"])
                self.task_list[id]["status"] = "finished"


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    s = zerorpc.Server(TaskManager())
    s.bind("tcp://127.0.0.1:12034")
    s.run()