# Generated by Django 4.1.2 on 2022-10-29 04:09

from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ('trigger', '0003_testrun_submitter_id_testrun_submitter_name_and_more'),
    ]

    operations = [
        migrations.RemoveField(
            model_name='testrun',
            name='submitter_id',
        ),
        migrations.RemoveField(
            model_name='testrun',
            name='submitter_name',
        ),
        migrations.AddField(
            model_name='testrun',
            name='image_digest',
            field=models.CharField(default='', max_length=200, verbose_name='Image digest'),
            preserve_default=False,
        ),
        migrations.AddField(
            model_name='testrun',
            name='image_name',
            field=models.CharField(default='', max_length=200, verbose_name='Name of Repository'),
            preserve_default=False,
        ),
        migrations.AddField(
            model_name='testrun',
            name='image_tag',
            field=models.CharField(default='', max_length=100, verbose_name='Image tag'),
            preserve_default=False,
        ),
    ]
