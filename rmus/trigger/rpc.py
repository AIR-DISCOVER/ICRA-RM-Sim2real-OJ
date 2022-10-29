import zerorpc
import logging
from .secret import *


def submit(cli_image):
    try:
        c = zerorpc.Client(timeout=10)
        c.connect(RPC_URL)
        id = c.add_task(cli_image)
    except zerorpc.exceptions.LostRemote as e:
        id = None
        logging.warning('No connection to runner')
    except zerorpc.exceptions.TimeoutExpired as e:
        id = None
        logging.warning('No connection to runner')
    except Exception as e:
        logging.warning('Other exceptions')
    return id


def query(id):
    try:
        c = zerorpc.Client(timeout=10)
        c.connect(RPC_URL)
        status, result = c.query_task(id)
    except zerorpc.exceptions.LostRemote as e:
        status, result = 'no connection', None
        logging.warning('No connection to runner')
    except zerorpc.exceptions.TimeoutExpired as e:
        status, result = 'no connection', None
        logging.warning('No connection to runner')
    return status, result