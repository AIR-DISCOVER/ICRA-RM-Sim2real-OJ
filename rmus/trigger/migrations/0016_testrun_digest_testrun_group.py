# Generated by Django 4.1.2 on 2022-12-30 09:58

from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ('trigger', '0015_testrun_video_alter_testrun_status'),
    ]

    operations = [
        migrations.AddField(
            model_name='testrun',
            name='digest',
            field=models.CharField(default='123', max_length=200, verbose_name='SHA256'),
            preserve_default=False,
        ),
        migrations.AddField(
            model_name='testrun',
            name='group',
            field=models.CharField(default='a', max_length=100, verbose_name='Group Name'),
            preserve_default=False,
        ),
    ]
