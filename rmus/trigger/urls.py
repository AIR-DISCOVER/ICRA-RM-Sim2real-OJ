from django.urls import path

from . import views

app_name = 'trigger'
urlpatterns = [
    path('', views.index, name='index'),
    path('status/', views.index, name='index'),
    path('status/<int:id>/', views.status, name='status'),
    path('apis/create/', views.create_testrun, name='create_testrun'),
    path('apis/create/<int:type>/', views.create_testrun, name='create_testrun'),
    path('apis/create_real/', views.create_real_testrun, name='create_testrun'),
    path('apis/update_real/', views.update_real_testrun, name='update_testrun'),
    path('apis/<int:id>/upload_log/', views.upload_log, name='update_testrun'),
]