from django_q.tasks import async_task, schedule
from django_q.models import Schedule
from django.utils import timezone
from datetime import timedelta
from .models import TestRun
from .rpc import query, submit


def async_handle(testrun: TestRun):
    async_task(handle, testrun)


def handle(testrun: TestRun):
    if testrun.status == TestRun.SUBMITTED:
        # May take long time
        id = submit(testrun)
        if id is None:
            testrun.status = TestRun.ERROR
            testrun.save()
        else:
            testrun.status = TestRun.WAITING
            testrun.runner_id = id
            testrun.save()
            schedule(handle,
                     testrun,
                     schedule_type=Schedule.ONCE,
                     next_run=timezone.now() + timedelta(seconds=10))
    else:
        status, result = query(testrun.runner_id)
        if status != 'finished':
            schedule(handle,
                     testrun,
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