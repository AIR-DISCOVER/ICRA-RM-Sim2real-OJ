import requests
import tempfile

def upload_log(testrun_id, server_log, client_log):
    f = tempfile.NamedTemporaryFile('r+', suffix='.txt')
    url = f'https://sim2real.discover-lab.com:11011/apis/{testrun_id}/upload_log/'
    print('==============Server=================', file=f)
    print(server_log, file=f)
    print('==============Client=================', file=f)
    print(client_log, file=f)
    f.seek(0)
    files = [('log_file', ('log.txt', f, 'application/octet-stream'))]
    headers = {
        'Authorization': 'Bearer 8c74c34a1cf4944cdf8ce6634e33e61529fccec32d008dfd'
    }
    response = requests.request('POST', url, headers=headers, data={}, files=files)
    f.close()
    return response

if __name__ == '__main__':
    upload_log(2, 'heihei', 'haha')