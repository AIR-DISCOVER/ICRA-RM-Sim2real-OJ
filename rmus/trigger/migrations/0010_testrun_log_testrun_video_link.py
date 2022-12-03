# Generated by Django 4.1.2 on 2022-11-24 09:31

from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ('trigger', '0009_alter_testrun_status'),
    ]

    operations = [
        migrations.AddField(
            model_name='testrun',
            name='log',
            field=models.TextField(default='', max_length=300000, verbose_name='Log'),
            preserve_default=False,
        ),
        migrations.AddField(
            model_name='testrun',
            name='video_link',
            field=models.CharField(default='None', max_length=400, verbose_name='Video Link'),
        ),
    ]