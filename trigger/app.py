import logging
import flask
import json
import zerorpc

app = flask.Flask(__name__)

@app.route('/', methods=['GET', 'POST'])
def trigger():
    try:
        data = json.loads(flask.request.get_data())
    except json.JSONDecodeError:
        logging.error("Cannot parse body in JSON format")
        return flask.Response(status=500)

    if data['type'] == "PUSH_ARTIFACT":
        pushed_image = data["event_data"]["resources"][0]["resource_url"]
        c = zerorpc.Client()
        c.connect("tcp://127.0.0.1:12034")
        print(c.test(pushed_image))
    return flask.Response(status=200)

app.run(host="0.0.0.0", port=22022)