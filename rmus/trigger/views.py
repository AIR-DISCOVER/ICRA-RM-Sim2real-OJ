import json
from django.shortcuts import render
from django.http import JsonResponse, HttpResponseServerError, Http404
from django.views.decorators.csrf import csrf_exempt
import logging
from .models import TestRun


# Create your views here.
def index(request):
    testrun_list = TestRun.objects.all()
    return render(request, 'trigger/index.html', {
        'testrun_list': testrun_list,
    })


def status(request, id):
    testrun = TestRun.objects.filter(id=id)
    if testrun.count() > 0:
        testrun = testrun.get(id=id)
        return render(request, 'trigger/status.html', {
            'id': testrun.id,
            'status': testrun.STATS_DICT.get(testrun.status, 'Unknown'),
            'submit_id': testrun.submitter_id,
            'submit_name': testrun.submitter_name,
            'time': testrun.submit_time,
            'result': testrun.result,
        })
    else:
        raise Http404()

# {
#     'type': 'PUSH_ARTIFACT',
#     'occur_at': 1667012162,
#     'operator': 'admin',
#     'event_data': {
#         'resources': [{
#             'digest':
#             'sha256:b8e334e578a285fef66adbb134cae2f594c889eed85569d22830652283760b9e',
#             'tag':
#             'latest',
#             'resource_url':
#             'docker.discover-lab.com: 55555/rm-sim2real/client:latest'
#         }],
#         'repository': {
#             'date_created': 1666503042,
#             'name': 'client',
#             'namespace': 'rm-sim2real',
#             'repo_full_name': 'rm-sim2real/client',
#             'repo_type': 'public'
#         }
#     }
# }

@csrf_exempt
def create_testrun(request):
    try:
        assert request.method == 'POST'
        if request.META.has_key('HTTP_X_FORWARDED_FOR'):
            ip =  request.META['HTTP_X_FORWARDED_FOR']
        else:
            ip = request.META['REMOTE_ADDR']
        assert ip == '127.0.0.1'
        jstring = request.body.decode('UTF-8')
        info = json.loads(jstring)
        assert 'submitter_name' in info.keys()
        assert 'submitter_id' in info.keys()
        assert info['authkey'] == '08D10441-1F98-4656-97D8-AB31BAF216EA'
        run = TestRun(
            submitter_name=info['submitter_name'],
            submitter_id=info['submitter_id'],
        )
        run.save()
        return JsonResponse({'testrun_id': run.id})
    except Exception as e:
        logging.error(e)
        return HttpResponseServerError()


def update_testrun(request):
    ...