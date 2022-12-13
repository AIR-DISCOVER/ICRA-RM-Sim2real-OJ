import requests
import tempfile

def upload_log(testrun_id, server_log, client_log, video=None):
    url = f'https://sim2real.discover-lab.com:11011/apis/{testrun_id}/upload_log/'
    f1 = tempfile.NamedTemporaryFile('r+', suffix='.txt')
    print('==============Server=================', file=f1)
    print(server_log, file=f1)
    f1.seek(0)
    f2 = tempfile.NamedTemporaryFile('r+', suffix='.txt')
    print('==============Client=================', file=f2)
    print(client_log, file=f2)
    f2.seek(0)
    files = [('log_file', ('server_log.txt', f1, 'application/octet-stream')), ('another_log_file', ('client_log.txt', f2, 'application/octet-stream')), ]
    if video is not None:
        f3 = open(video, 'rb')
        files.append(('video', ('video.mp4', f3, 'application/octet-stream')))
    headers = {
        'Authorization': 'Bearer 8c74c34a1cf4944cdf8ce6634e33e61529fccec32d008dfd'
    }
    response = requests.request('POST', url, headers=headers, data={}, files=files)
    f1.close()
    f2.close()
    f3.close()
    return response

if __name__ == '__main__':
    print(upload_log(154, 'heihei', 'haha'))