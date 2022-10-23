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

@csrf_exempt
def create_testrun(request):
    try:
        assert request.method == 'POST'
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