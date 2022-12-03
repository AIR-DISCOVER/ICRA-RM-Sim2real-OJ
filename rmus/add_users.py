import requests
import json


def send_request(url, payload):
    url = f"https://docker.discover-lab.com:55555/api/v2.0{url}"

    headers = {
        'Authorization': 'Basic YWRtaW46VEJXemgwNjE3',
        'Content-Type': 'application/json'
    }
    response = requests.request("POST", url, headers=headers, data=payload)
    return response.status_codes


def create_user(name):
    payload = lambda user: json.dumps({
        "email": f"{user}@123.com",
        "realname": f"{user}",
        "comment": f"{user}",
        "password": f"{user}ABCdef123",
        "username": f"{user}"
    })
    return send_request('/users', payload(name))


def create_project(name):
    payload = lambda project: json.dumps({
        "project_name": f"{project}",
        "public": False,
        "metadata": {
            "public": "false",
        },
        "storage_limit": 50
    })
    return send_request('/projects', payload(name))


def add_user_to_project(project, user):
    payload = lambda project, name: json.dumps({
        "role_id": 0,
        "member_user": {
            "username": name
        },
        "member_group": {
            "id": 0,
            "group_name": "string",
            "group_type": 0,
            "ldap_group_dn": "string"
        }
    })
    return send_request(f'/projects/{project}/members')
