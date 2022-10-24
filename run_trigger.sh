#!/bin/bash
source secret.sh
frp/frpc -c frp/frpc_trigger.ini &
python trigger/app.py
