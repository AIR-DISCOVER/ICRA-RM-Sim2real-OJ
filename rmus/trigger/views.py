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
from datetime import timedelta, datetime
from hashlib import sha256
import json

with open('group.json') as f:
    GROUP_MAPPING = json.load(f)

# Create your views here.
def index(request, group_digest=''):
    if "HTTP_AUTHORIZATION" in request.META and request.META["HTTP_AUTHORIZATION"] == AUTH_HEADER:
        testrun_list = TestRun.objects.all()
    else:
        testrun_list = TestRun.objects.filter(group=GROUP_MAPPING.get(group_digest, ''))
    return render(request, 'trigger/index.html', {
        'testrun_list': testrun_list,
    })


def status(request, id):
    testrun = TestRun.objects.filter(digest=id)
    if testrun.count() > 0:
        testrun = testrun.get(digest=id)
        log_file_url = testrun.log_file.url if testrun.log_file and hasattr(testrun.log_file, 'url') else None
        log_file_2_url = testrun.another_log_file.url if testrun.another_log_file and hasattr(testrun.another_log_file, 'url') else None
        if testrun.video and hasattr(testrun.video, 'url'):
            video_url = testrun.video.url
        elif testrun.video_link is not None:
            video_url = testrun.video_link
        else:
            video_url = None

        return render(
            request, 'trigger/status.html',
            {'object': testrun, 'log_file_url': log_file_url, 'log_file_2_url': log_file_2_url, 'video_url': video_url})
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
def create_testrun(request, run_type=2):
    try:
        assert request.META["HTTP_AUTHORIZATION"] == USER_AUTH_HEADER
        assert request.method == 'POST'
        jstring = request.body.decode('UTF-8')
        info = json.loads(jstring)
        logging.info(info)
        submitter = info['operator']
        timestamp = int(info['occur_at'])
        repo = info['event_data']['repository']['repo_full_name']
        group = info['event_data']['repository']['namespace']
        tag = info['event_data']['resources'][0]['resource_url'].split(':')[-1]
        if len(tag) > 40:
            return HttpResponseServerError()
        digest = info['event_data']['resources'][0]['digest']

        shasum = sha256(('salt'+str(timestamp)+ 'repo' + 'tag').encode()).hexdigest()
        print(repo, tag, digest, shasum)
        run = TestRun(
            submit_time=timestamp,
            submitter=submitter,
            image_name=repo,
            image_tag=tag,
            image_digest=digest,
            run_type=run_type,
            digest=shasum,
            group=group,
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
#     "submit_time": 1667012162,
#     "submitter": 'Name',
#     "image_name": 'docker.discover-lab.com:55555/repo/image',
#     "image_tag": 'latest',
#     "image_digest": 'sha256:abcdef...xx',
#     "video_link": 'https://...',
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
        group = info['image_name'].split('/')[1]
        shasum = sha256(('salt' + str(int(datetime.now().timestamp()))).encode()).hexdigest()
        run = TestRun(
            submit_time=info['submit_time'],
            submitter=info['submitter'],
            image_name=info['image_name'],
            image_tag=info['image_tag'],
            image_digest=info['image_digest'],
            video_link=info['video_link'],
            testrun_type='Real',
            run_type=2,
            digest=shasum,
            group=group
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
#     "video_link": 'https://...'
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
        if 'video' in request.FILES:
            run.video = request.FILES['video']
        
        run.save()
        return JsonResponse({'testrun_id': run.id})

    except Exception as e:
        logging.error(type(e))
        logging.error(e)
        return HttpResponseServerError()
