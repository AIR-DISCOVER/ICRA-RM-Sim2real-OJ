from django_q.tasks import async_task, schedule
from django_q.models import Schedule
from django.utils import timezone
from datetime import timedelta
from .models import TestRun
from .rpc import query, submit


def handle(testrun_id: str):
    testrun = TestRun.objects.filter(id=testrun_id)
    if testrun.count() > 0:
        testrun = testrun.get(id=testrun_id)
    else:
        assert False, "unable to get testrun"
    if testrun.status == TestRun.SUBMITTED:
        id = submit(f"docker.discover-lab.com:55555/{testrun.image_name}@{testrun.image_digest}")
        if id is None:
            schedule("trigger.tasks.handle",
                     testrun.id,
                     schedule_type=Schedule.ONCE,
                     next_run=timezone.now() + timedelta(seconds=10))
        elif id == 'invalid':
            testrun.status = TestRun.ERROR
            testrun.save()
        else:
            testrun.status = TestRun.WAITING
            testrun.runner_id = id
            testrun.save()
            schedule("trigger.tasks.handle",
                     testrun.id,
                     schedule_type=Schedule.ONCE,
                     next_run=timezone.now() + timedelta(seconds=10))
    else:
        status, result = query(testrun.runner_id)
        if status != 'finished':
            schedule("trigger.tasks.handle",
                     testrun.id,
                     schedule_type=Schedule.ONCE,
                     next_run=timezone.now() + timedelta(seconds=10))
        if testrun.status == TestRun.WAITING:
            if status == 'waiting':
                pass
            elif status == 'running':
                testrun.status = TestRun.RUNNING
            elif status == 'finished':
                testrun.status = TestRun.FINISHED
                testrun.result = result
            elif status == 'not found':
                # Rerun
                testrun.status = TestRun.SUBMITTED
            elif status == 'no connection':
                testrun.status = TestRun.UNKNOWN
            else:
                assert False, f"Unreachable, {testrun.status} and {status}"
            testrun.save()
        elif testrun.status == TestRun.RUNNING:
            if status == 'waiting':
                assert False, f"Unreachable, {testrun.status} and {status}"
            elif status == 'running':
                pass
            elif status == 'finished':
                testrun.status = TestRun.FINISHED
                testrun.result = result
            elif status == 'not found':
                testrun.status = TestRun.SUBMITTED
            elif status == 'no connection':
                testrun.status = TestRun.UNKNOWN
            else:
                assert False, f"Unreachable, {testrun.status} and {status}"
            testrun.save()
        elif testrun.status == TestRun.UNKNOWN:
            if status == 'waiting':
                testrun.status = TestRun.WAITING
            elif status == 'running':
                testrun.status = TestRun.RUNNING
            elif status == 'finished':
                testrun.status = TestRun.FINISHED
                testrun.result = result
            elif status == 'not found':
                testrun.status = TestRun.SUBMITTED
            elif status == 'no connection':
                pass
            else:
                assert False, f"Unreachable, {testrun.status} and {status}"
            testrun.save()
        elif testrun.status == TestRun.FINISHED:
            assert False, f"Unreachable, handle finished"
        elif testrun.status == TestRun.ERROR:
            assert False, f"Unreachable, handle error"