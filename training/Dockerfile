FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=:0

RUN apt-get update && \
    apt-get install -y \
    python3 \
    python3-pip \
    git \
    build-essential \
    autoconf \
    libtool && \
    apt-get clean

RUN apt update && apt install python-opengl -y

WORKDIR /app

COPY . .

RUN pip install protobuf==3.20
RUN pip install ./rSoccer

WORKDIR /app

CMD ["python3", "teste.py"]