import json
from django.shortcuts import render
from django.http import JsonResponse, HttpResponseServerError, Http404
from django.views.decorators.csrf import csrf_exempt
import logging
from .models import TestRun
from .secret import AUTH_HEADER, USER_AUTH_HEADER
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
        log_file_url = testrun.log_file.url if testrun.log_file and hasattr(testrun.log_file, 'url') else None
        # from IPython import embed
        # embed()
        return render(
            request, 'trigger/status.html',
            {'object': testrun, 'log_file_url': log_file_url})
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
def create_testrun(request, run_type=1):
    try:
        assert request.META["HTTP_AUTHORIZATION"] == USER_AUTH_HEADER
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
            run_type=run_type
        )
        run.save()

        # async_handle(run)
        async_task("trigger.tasks.handle", run.id, run.run_type)
        logging.info(f"Scheduled at " + str(timezone.now() + timedelta(seconds=10)))
        return JsonResponse({'testrun_id': run.id})
    except Exception as e:
        logging.error(type(e))
        logging.error(e)
        return HttpResponseServerError()


# {
#     "runner": "None",
#     "status": "SUBMITTED",
#     "run_result": "time of evalutaion",
#     "submit_time": 1667012162,
#     "submitter": 'Name',
#     "image_name": 'docker.discover-lab.com:55555/repo/image',
#     "image_tag": 'latest',
#     "image_digest": 'sha256:abcdef...xx',
#     "video_link": 'https://...',
#     "log": "logabcdef",
# }

@csrf_exempt
def create_real_testrun(request):
    try:
        # for k in sorted(request.META):
        #     print(k, request.META[k])
        assert request.META["HTTP_AUTHORIZATION"] == AUTH_HEADER
        print(request.META["HTTP_AUTHORIZATION"])
        print(request.method)
        assert request.method == 'POST'
        jstring = request.body.decode('UTF-8')
        info = json.loads(jstring)
        logging.info(info)

        run = TestRun(
            submit_time=info['submit_time'],
            submitter=info['submitter'],
            image_name=info['image_name'],
            image_tag=info['image_tag'],
            image_digest=info['image_digest'],
            video_link=info['video_link'],
            testrun_type='Real',
        )
        run.save()
        return JsonResponse({'testrun_id': run.id})

    except Exception as e:
        logging.error(type(e))
        logging.error(e)
        return HttpResponseServerError()


# {
#     "id": "1112",
#     "runner": "None",
#     "status": "FINISHED",
#     "run_result": "104s",
#     "video_link": 'https://...',
#     "log": "log text",
# }

@csrf_exempt
def update_real_testrun(request):
    try:
        assert request.META["HTTP_AUTHORIZATION"] == AUTH_HEADER
        assert request.method == 'POST'
        jstring = request.body.decode('UTF-8')
        info = json.loads(jstring)
        logging.info(info)

        run = TestRun.objects.filter(id=int(info['id']))
        if run.count() > 0:
            run = run.get(id=int(info['id']))
        else:
            return JsonResponse({'testrun_id': None})

        if 'runner' in info:
            run.runner_id = info['runner']
        if 'status' in info:
            run.status = info['status']
        if 'run_result' in info:
            run.result = info['run_result']
        if 'video_link' in info:
            run.video_link = info['video_link']
        if 'log' in info:
            run.log = info['log']
        run.save()
        return JsonResponse({'testrun_id': run.id})

    except Exception as e:
        logging.error(type(e))
        logging.error(e)
        return HttpResponseServerError()

@csrf_exempt
def upload_log(request, id):
    try:
        assert request.META["HTTP_AUTHORIZATION"] == AUTH_HEADER
        assert request.method == 'POST'
        run = TestRun.objects.filter(id=id)
        if run.count() > 0:
            run = run.get(id=id)
        else:
            return JsonResponse({'testrun_id': None})
        if 'log_file' in request.FILES:
            run.log_file = request.FILES['log_file']
        if 'another_log_file' in request.FILES:
            run.another_log_file = request.FILES['another_log_file']
        
        run.save()
        return JsonResponse({'testrun_id': run.id})

    except Exception as e:
        logging.error(type(e))
        logging.error(e)
        return HttpResponseServerError()
