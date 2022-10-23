from .judge import run
import zerorpc


class Server(object):

    def echo(self, str):
        return str

    def test(self, client_image):
        return run(client_image)


if __name__ == '__main__':
    s = zerorpc.Server(Server())
    s.bind("tcp://127.0.0.1:12033")
    s.run()