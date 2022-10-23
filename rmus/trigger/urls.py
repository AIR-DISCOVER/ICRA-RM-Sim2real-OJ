from django.urls import path

from . import views

app_name = 'trigger'
urlpatterns = [
    path('status/', views.index, name='index'),
    path('status/<int:id>/', views.status, name='status'),
    path('apis/create/', views.create_testrun, name='create_testrun'),
    path('apis/update/', views.update_testrun, name='update_testrun'),
]