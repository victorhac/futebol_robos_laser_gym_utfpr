unzip -o rSoccer.zip -d .
sed -E -i "s/install_requires.+/install_requires=['gym==0.21.0', 'rc-robosim==1.2.0', 'pyglet==1.5.21', 'protobuf==3.20']/" ./rSoccer/setup.py
RUN pip install ./rSoccer