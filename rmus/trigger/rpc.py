import zerorpc
import logging
from .secret import *

def submit(cli_image):
    while True:
        try:
            c = zerorpc.Client(timeout=10)
            c.connect(RPC_URL)
            id = c.add_task(cli_image)
            break
        except zerorpc.exceptions.LostRemote as e:
            logging.warning('No connection to runner')
    return id

def query(id):
    try:
        c = zerorpc.Client(timeout=10)
        c.connect(RPC_URL)
        status, result = c.query_task(id)
    except zerorpc.exceptions.LostRemote as e:
        status, result = 'no connection', None
        logging.warning('No connection to runner')
    return status, result