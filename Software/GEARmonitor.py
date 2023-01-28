import json
from paho.mqtt import client as mqtt_client
import time
from math import ceil, floor
import _thread
import pyqtgraph as pg
import array
from PyQt5 import QtWidgets
import pandas as pd

# 公共变量
broker = '192.168.0.120' #MQTT服务器地址
topic = "MAV3_TX" #监控的无人机的话题
topic_remote = "Remote" #监控的遥控器的话题
port = 1883
client_id = f'Monitor'

type_rev = 1
add_flag = 0

time_stamp = array.array('d') #可动态改变数组的大小,double型数组
value1_cache = array.array('d') #可动态改变数组的大小,double型数组
value2_cache = array.array('d') #可动态改变数组的大小,double型数组
value3_cache = array.array('d') #可动态改变数组的大小,double型数组

t = time.time()
system_time_ms = (int(round(t * 1000)))    #毫秒级时间戳
delay_time_ms = 0
heart = 0

## GUI初始化
app = QtWidgets.QApplication([])
pg.setConfigOptions(leftButtonPan=True, antialias=True)
pg.setConfigOption('background', 'w')
pg.setConfigOption('foreground', 'k')

win = QtWidgets.QWidget()
win.resize(1000, 700)
win.setWindowTitle(u'GEARmonitor自定监看变量')
## 控件初始化
loadfile = QtWidgets.QPushButton('Load files (L)')
loadfile.setShortcut('l')
clearscreen = QtWidgets.QPushButton('Clear Screen (C)')
clearscreen.setShortcut('c')
listw = QtWidgets.QListWidget()
p = pg.PlotWidget()
p.addLegend()
remotestate = QtWidgets.QLabel("遥控器延迟：%ums"% delay_time_ms)
serverstate = QtWidgets.QLabel("正在连接服务器……")

curve = p.plot(pen='r',markersize = 4)#绘制一个图形
curve2 = p.plot(pen='b',markersize = 4)#绘制一个图形
curve3 = p.plot(pen='m',markersize = 4)#绘制一个图形

## 创建GUI布局
layout = QtWidgets.QGridLayout()
win.setLayout(layout)
layout.setColumnStretch(1, 10)

## 创建控件布局
layout.addWidget(clearscreen, 0, 0)   
layout.addWidget(loadfile, 1, 0)  
layout.addWidget(listw, 2, 0)  
layout.addWidget(remotestate, 3, 0)
layout.addWidget(serverstate, 4, 0)
layout.addWidget(p, 0, 1, 5, 1) 
historyLength = 50#横坐标长度


"""mqtt subscribe topic"""
def parse_mqttmsg(msg):
    global value1_cache,value2_cache,value3_cache,time_stamp,curve,curve2,curve3,add_flag
    global type_rev,system_time_ms,delay_time_ms,heart
    """解析mqt头数据   MAC samplerate sampletime battery acc"""
    if msg.topic == topic:
        content = json.loads(msg.payload.decode())
        type_rev = content['type']
        time_ms = content['time']
        if type_rev == 1:
            
            value1_rev = content['value1']
            value2_rev = content['value2']
            value3_rev = content['value3']

            try:
                time_stamp.append(time_ms)
                value1_cache.append(value1_rev)
                value2_cache.append(value2_rev)
                value3_cache.append(value3_rev)
                add_flag = 1 #允许读取数据（线程同步）

            except:
                pass
        if len(time_stamp) > 300:
            p.setRange(xRange=[time_stamp[-300],time_stamp[-1]], padding=0)
    elif msg.topic == topic_remote:
        heart = (int(round(time.time() * 1000)))    #毫秒级时间戳
        
def connect_mqtt():
    global serverstate
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
            serverstate.setText("服务器已连接")
        else:
            print("Failed to connect, return code %d\n", rc)
            serverstate.setText("服务器连接失败！")
            pass

    client = mqtt_client.Client(client_id)
    client.username_pw_set('admin', 'public')  # 链接mqtt所需的用户名和密码，没有可不写
    client.on_connect = on_connect
    client.connect(broker, port)
    return client


def subscribe(client: mqtt_client):
    def on_message(client, userdata, msg):
        parse_mqttmsg(msg)

    client.subscribe(topic)
    client.subscribe(topic_remote)
    client.on_message = on_message


def run():
    client = connect_mqtt()
    subscribe(client)
    client.loop_forever()

""" draw figures """
def draw_figure():
    global value1_cache,value2_cache,value3_cache,time_stamp,curve,curve2,curve3,add_flag
    global type_rev,system_time_ms,delay_time_ms,heart

    delay_time_ms = (int(round(time.time() * 1000))) - heart

    p.showGrid(x=True, y=True)#把X和Y的表格打开
    p.setLabel(axis='left', text='Y')#靠左
    p.setLabel(axis='bottom', text ='X(ms)')
    p.setTitle('GEARmonitor 齿轮监控')#表格的名字
    try:
        if type_rev == 1 and add_flag == 1:
            curve.setData(time_stamp,value1_cache)
            curve2.setData(time_stamp,value2_cache)
            curve3.setData(time_stamp,value3_cache)
            add_flag = 0 #已取走数据cc

        if delay_time_ms < 500:
            remotestate.setText(u'遥控器延迟：%ums'% delay_time_ms)
        else:
            remotestate.setText(u'遥控器：LOSS')
    except:
        pass

def clf_screen():
    global value1_cache,value2_cache,value3_cache,time_stamp,curve,curve2,curve3
    p.clear()
    del value1_cache[:]
    del value2_cache[:]
    del value3_cache[:]
    del time_stamp[:]
    curve = p.plot(pen='r',markersize = 4)#绘制一个图形
    curve2 = p.plot(pen='b',markersize = 4)#绘制一个图形
    curve3 = p.plot(pen='m',markersize = 4)#绘制一个图形

def load_file():

    global value1_cache,value2_cache,value3_cache,time_stamp,curve,curve2,curve3,add_flag
    file_path, _ = QtWidgets.QFileDialog.getOpenFileName(win, "选择文件","./","csv(*csv)")
    listw.addItem(file_path)
    data=pd.read_csv(file_path,error_bad_lines=False)
    #必须添加header=None，否则默认把第一行数据处理成列名导致缺失
    del value1_cache[:]
    del value2_cache[:]
    del value3_cache[:]
    del time_stamp[:]
    if data[r"x0000"].values.tolist()[0] > 6000:
        time_stamp = data[r"x0000"].values.tolist()
        value1_cache = data[r"y0000"].values.tolist()
        value2_cache = data[r"y0001"].values.tolist()
        value3_cache = data[r"y0002"].values.tolist()
    else:
        time_stamp = data[r"y0000"].values.tolist()
        value1_cache = data[r"x0000"].values.tolist()
        value2_cache = data[r"x0001"].values.tolist()
        value3_cache = data[r"x0002"].values.tolist()
    p.setAutoVisible(x=True)
    p.setAutoVisible(y=True)
    add_flag = 1

if __name__ == '__main__':

    clearscreen.clicked.connect(clf_screen)
    loadfile.clicked.connect(load_file)
    win.setWindowTitle(u'GEARmonitor 齿轮监控')
    win.show()

    # 多线程
    _thread.start_new_thread(run, ())
    timer = pg.QtCore.QTimer()
    timer.timeout.connect(draw_figure)#定时调用plotData函数
    timer.start(10)#多少ms调用一次
    app.exec_()
    