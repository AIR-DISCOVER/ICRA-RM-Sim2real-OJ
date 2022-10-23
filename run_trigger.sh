#!/bin/bash
source runner/secret.sh
frp/frpc -c frp/frpc_trigger.ini &
python trigger/app.py
