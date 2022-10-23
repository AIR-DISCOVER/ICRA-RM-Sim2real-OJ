import logging
import flask
import json

app = flask.Flask(__name__)

@app.route('/', methods=['GET', 'POST'])
def login():
    try:
        data = json.loads(flask.request.get_data())
    except json.JSONDecodeError:
        logging.error("Cannot parse body in JSON format")
        return flask.Response(status=500)

    print(json.dumps(data, indent=4))
    return flask.Response(status=200)

app.run(host="0.0.0.0")