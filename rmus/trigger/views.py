import json
from django.shortcuts import render
from django.http import JsonResponse, HttpResponseServerError, Http404
from django.views.decorators.csrf import csrf_exempt
import logging
from .models import TestRun
from .secret import AUTH_HEADER
from django_q.tasks import async_task, schedule
from django_q.models import Schedule
from django.utils import timezone
from datetime import timedelta


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
        return render(
            request, 'trigger/status.html',
            {''.join(k.split()): v
             for k, v in testrun.status_dict().items()})
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
#             'docker.discover-lab.com:55555/rm-sim2real/client:latest'
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
        assert request.META["HTTP_AUTHORIZATION"] == AUTH_HEADER
        assert request.method == 'POST'
        jstring = request.body.decode('UTF-8')
        info = json.loads(jstring)
        logging.info(info)
        submitter = info['operator']
        timestamp = int(info['occur_at'])
        repo = info['event_data']['repository']['repo_full_name']
        tag = info['event_data']['resources'][0]['resource_url'].split(':')[-1]
        digest = info['event_data']['resources'][0]['digest']
        print(repo, tag, digest)
        run = TestRun(
            submit_time=timestamp,
            submitter=submitter,
            image_name=repo,
            image_tag=tag,
            image_digest=digest,
        )
        run.save()

        # async_handle(run)
        async_task("trigger.tasks.handle", run.id)
        print(f"Scheduled at " + str(timezone.now() + timedelta(seconds=10)))
        return JsonResponse({'testrun_id': run.id})
    except Exception as e:
        logging.error(type(e))
        logging.error(e)
        return HttpResponseServerError()


def update_testrun(request):
    ...