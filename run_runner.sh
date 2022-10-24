#!/bin/bash
source secret.sh
frp/frpc -c frp/frpc_runner.ini &
python -m runner.rpc
