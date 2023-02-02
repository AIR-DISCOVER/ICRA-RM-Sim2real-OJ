from email.policy import default
from django.db import models


# Create your models here.
class TestRun(models.Model):
    SUBMITTED = "SUBMITTED"
    WAITING = "WAITING"
    RUNNING = "RUNNING"
    FINISHED = "FINISHED"
    ERROR = "ERROR"
    RETRY = "RETRYING"
    UNKNOWN = "UNKKNOWN"
    CRASH = "CRASH"
    CRASHED = "CRASHED"
    TIMEOUT = "TIMEOUT"
    STATS = [
        (SUBMITTED, 'Submitted'),
        (WAITING, 'Waiting'),
        (RUNNING, 'Running'),
        (FINISHED, 'Finished'),
        (ERROR, "Error occurred"),
        (UNKNOWN, "Unknown status"),
        (RETRY, "Retrying"),
        (CRASH, "Crash"),
        (CRASHED, "Crashed"),
        (TIMEOUT, "TIMEOUT")
    ]
    STATS_DICT = {i[0]: i[1] for i in STATS}

    submit_time = models.DateTimeField("Submit Time", auto_now_add=True)
    submitter = models.CharField("Name of Submitter", max_length=100)
    image_name = models.CharField("Image name", max_length=200)
    image_tag = models.CharField("Image tag", max_length=100)
    image_digest = models.CharField("Image digest", max_length=200)

    runner_id = models.CharField("Runner ID", default="None", max_length=100)
    video_link = models.CharField("Video Link", default="None", max_length=400)
    testrun_type = models.CharField("Type", default="Sim", max_length=10)

    status = models.CharField("Status",
                              max_length=10,
                              choices=STATS,
                              default=SUBMITTED)
    result = models.TextField("Run Result", max_length=1000)
    run_type = models.IntegerField("Run Type")
    digest = models.CharField("SHA256", max_length=200)
    group = models.CharField("Group Name", max_length=100)

    log_file = models.FileField()
    another_log_file = models.FileField()
    video = models.FileField(null=True, blank=True, default=None)
    def status_dict(self) -> dict:
        status = {
            "Run ID": self.id,
            "Runner ID": self.runner_id,
            "Status": self.STATS_DICT[self.status],
            "Run Result": self.result,
            "Submit Time": self.submit_time,
            "Name of Submitter": self.submitter,
            "Image Name": self.image_name,
            "Image Tag": self.image_tag,
            "Image Digest": self.image_digest,
            "Video Link": self.video_link,
            "Type": self.testrun_type,
        }
        return status

    format = lambda self: '<br />'.join(
        [f"{k}:\t{v}" for k, v in self.status_dict().items()])

    __str__ = lambda self: '-'.join([str(i) for i in self.status_dict().values()])