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

RUN git clone https://github.com/robocin/rSoccer.git
RUN sed -E -i "s/install_requires.+/install_requires=['gym==0.21.0', 'rc-robosim==1.2.0', 'pyglet==1.5.21', 'protobuf==3.20']/" ./rSoccer/setup.py
RUN pip install ./rSoccer

WORKDIR /app

CMD ["python3", "main.py"]