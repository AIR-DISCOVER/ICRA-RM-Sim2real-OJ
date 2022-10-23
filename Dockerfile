FROM python:3.9-alpine

RUN pip install Flask -i https://pypi.tuna.tsinghua.edu.cn/simple

WORKDIR /opt

ADD app.py .

CMD ["python", "app.py"]