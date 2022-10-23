from django.db import models


# Create your models here.
class TestRun(models.Model):
    SUBMITTED = "SUB"
    WAITING = "WAIT"
    RUNNING = "RUN"
    FINISHED = "FIN"
    ERROR = "ERR"
    UNKNOWN = "UNK"
    STATS = [
        (SUBMITTED, "Submitted"),
        (WAITING, 'Waiting'),
        (RUNNING, 'Running'),
        (FINISHED, 'Finished'),
        (ERROR, "Error occurred"),
        (UNKNOWN, "Unknown status"),
    ]
    STATS_DICT = {i[0]: i[1] for i in STATS}

    submit_time = models.DateTimeField("Submit Time", auto_now_add=True)
    status = models.CharField("Status", max_length=4, choices=STATS, default=SUBMITTED)
    result = models.TextField("Run Result", max_length=1000)
    submitter_name = models.CharField("Name of Submitter", max_length=20)
    submitter_id = models.CharField("ID of submitter", max_length=12)

    def status_lines(self) -> str:
        status = [
            f"Run ID: {self.id}",
            f"Status: {self.STATS_DICT[self.status]}",
            f"Submit Time: {self.submit_time}",
            f"Result: {self.result}",
        ]
        return status

    __str__ = lambda self: ' '.join(self.status_lines())
    format = lambda self: '<br />'.join(self.status_lines())