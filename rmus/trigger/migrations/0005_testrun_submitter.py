# Generated by Django 4.1.2 on 2022-10-29 04:11

from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ('trigger', '0004_remove_testrun_submitter_id_and_more'),
    ]

    operations = [
        migrations.AddField(
            model_name='testrun',
            name='submitter',
            field=models.CharField(default='', max_length=100, verbose_name='Name of Submitter'),
            preserve_default=False,
        ),
    ]
