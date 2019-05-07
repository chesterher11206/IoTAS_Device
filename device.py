import os, sys, time, platform, socket
import serial
import threading
import requests
import paho.mqtt.client as mqtt
from urllib.request import urlopen


def get_uuid(os_name):
    if os_name == 'Darwin':
        cmd = "/usr/sbin/system_profiler SPHardwareDataType | fgrep 'UUID' | awk '{print $NF}'"
        output = os.popen(cmd).readlines()
        uuid_list = [line.strip() for line in output]
        partuuid_list = []
    elif os_name == 'Linux':
        # cmd = "dmidecode -t system | grep -i {}"
        uuid_cmd = "blkid -s UUID | fgrep 'UUID' | awk '{print $NF}'"
        partuuid_cmd = "blkid -s PARTUUID | fgrep 'PARTUUID' | awk '{print $NF}'"
        uuid_output = os.popen(uuid_cmd).readlines()
        partuuid_output = os.popen(partuuid_cmd).readlines()
        uuid_list = [line.split('\"')[-2].strip() for line in uuid_output]
        partuuid_list = [line.split('\"')[-2].strip() for line in partuuid_output]
    uuid_f = ""
    if uuid_list:
        uuid_f = uuid_list[0]
    for uuid in uuid_list:
        uuid_f = uuid
        if len(uuid_f) == 36:
            break
    if partuuid_list:
        uuid_f = uuid_f + ":" + partuuid_list[0]
    return uuid_f

def get_private_ip(os_name):
    if os_name == 'Darwin':
        cmd = "ifconfig en0 | fgrep -w 'inet' | awk '{print $NF}'"
        output = os.popen(cmd)
        ip = output.read().strip()
    elif os_name == 'Linux':
        get_s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        get_s.connect(('8.8.8.8', 0))
        ip = get_s.getsockname()[0]
    return ip

def get_public_ip():
    ip = requests.get('https://api.ipify.org').text
    return str(ip)

def get_hostname():
    myname = socket.getfqdn(socket.gethostname())
    #print(type(myname))
    return myname


class Device(object):
    def __init__(self, *args, **kwargs):
        self.init_status()
        self.init_config()
        self.set_device_info()
        self.set_mqtt()
        sensor_thread = threading.Thread(target=self.set_sensor)
        sensor_thread.daemon = True
        sensor_thread.start()
        receive_thread = threading.Thread(target=self.receive_server)
        receive_thread.daemon = True
        receive_thread.start()

    def init_status(self):
        self.is_connect = False
        self.status = False
        self.prev_DHT = int(time.time())
        self.prev_light = int(time.time())
        self.dht_comp = ""
        self.light_comp = ""

    def init_config(self):
        self.freq_DHT = 2
        self.freq_light = 2
        self.conti = False
        self.sensor_on = []

    def set_device_info(self):
        self.device_info = dict()
        os_name = platform.system()
        self.device_info['os_name'] = os_name
        self.device_info['uuid'] = get_uuid(os_name)
        self.device_info['private_ip'] = get_private_ip(os_name)
        self.device_info['public_ip'] = get_public_ip()
        self.device_info['host'] = get_hostname()

    def set_mqtt(self):
        BROKER_IP = "140.112.18.2"
        PORT = 11053
        LAG_TIME = 600
        TOPIC = "server"

        #Note:return command result to broker
        self.response_client = mqtt.Client()
        self.response_client.on_connect = self.on_connect
        self.response_client.on_publish = self.on_publish
        self.inform_client.on_disconnect = self.on_disconnect
        self.response_client.connect(BROKER_IP, PORT, LAG_TIME)

        #Note:receive command from broker
        self.receive_client = mqtt.Client()
        self.receive_client.on_connect = self.on_connect
        self.receive_client.on_message = self.on_message
        self.inform_client.on_disconnect = self.on_disconnect
        self.receive_client.connect(BROKER_IP, PORT, LAG_TIME)
        self.receive_client.subscribe(TOPIC+'/#')

        #Note:device info and Internet connection
        self.inform_client = mqtt.Client()
        self.inform_client.on_connect = self.on_connect
        self.inform_client.on_publish = self.on_publish
        self.inform_client.on_disconnect = self.on_disconnect
        self.inform_client.connect(BROKER_IP, PORT, LAG_TIME)

    def set_sensor(self):
        self.ser = serial.Serial('/dev/ttyACM0', 9600)
        while True:
            sen_raw = self.ser.readline()
            sen = str(sen_raw)
            #print(sen)
            if 'Sensor' in sen:
                sensor_name = sen[10:-5]
                print(sensor_name)
                self.status = True
                break
        self.sensor_list = sensor_name.split("+")

        while self.ser.in_waiting:
            if not self.status and not self.is_connect:
                continue
            data = str(ser.readline())
            print("data ", data)
            if 'x' in data:
                break
            else:                
                if 'Humid' in data:
                    data_split = data.split(",")
                    #print(data_split)
                    humid_value = data_split[0][12:]
                    temp_value = data_split[1][13:-5]
                    ticks = str(int(time.time()))
                    humidtemp_dict = dict()
                    humidtemp_dict['uuid'] = self.device_info['uuid']
                    humidtemp_dict['humid'] = humid_value
                    humidtemp_dict['temperature'] = temp_value
                    # DHT_info = uuid + "," + humid_value + "," + temp_value
                    # DHT_send = uuid + "," + humid_value + "," + temp_value + "," + str(ticks)                     
                    #print(DHT_info)
                    if self.conti == True:
                        if ticks - self.prev_DHT >= self.freq_DHT:
                            humidtemp_dict['time'] = ticks
                            self.inform_client.publish("device/connect/sensor/DHT", payload=json.dumps(humidtemp_dict), qos=0)
                            self.prev_DHT = ticks
                    elif self.conti == False:
                        if json.dumps(humidtemp_dict) != self.dht_comp:
                            self.dht_comp = json.dumps(humidtemp_dict)
                            humidtemp_dict['time'] = ticks
                            self.inform_client.publish("device/connect/sensor/DHT", payload=json.dumps(humidtemp_dict), qos=0)

                elif 'Light' in data:
                    light_value = data[9:-5]
                    #value = "0"
                    ticks = str(int(time.time()))
                    light_dict = dict()
                    light_dict['uuid'] = self.device_info['uuid']
                    light_dict['light'] = light_value
                    # light_info = uuid + "," + value
                    # light_send = uuid + "," + value + "," + str(ticks)
                    #print(light_info)
                    if self.conti == True:
                        if ticks - self.prev_light >= self.freq_light:
                            light_dict['time'] = ticks
                            self.inform_client.publish("device/connect/sensor/light", payload=json.dumps(light_dict), qos=0)
                            self.prev_light = ticks
                    elif self.conti == False:
                        if json.dumps(light_dict) != self.light_comp:
                            self.light_comp = json.dumps(light_dict)
                            light_dict['time'] = ticks
                            self.inform_client.publish("device/connect/sensor/light", payload=json.dumps(light_dict), qos=0)

    def receive_server(self):
        while True:
            try:
                self.receive_client.loop_start()
            except:
                print("System closing...")
                sys.exit(0)

    def on_connect(client, userdata, flags, rc):
        print("Connected with result code " + str(rc))

    def on_publish(client, userdata, mid):
        print("A new message is published!")

    def on_disconnect(client, userdata, rc):
        if rc != 0:
            print("disconnect")

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        message = json.loads(msg.payload.decode("utf-8"))
        ticks = time.asctime(time.localtime(time.time()))
        print("-----------------------------------------------------")
        print("Time now: ", ticks)
        print(message['message'])
        uuid = self.device_info['uuid']
        device_info_json = json.dumps(self.device_info)
        if topic == "server/search/info":
            if message['message'] == "Search Device":
                self.response_client.publish("device/search/info", payload=device_info_json, qos=0)
        elif topic == "server/allow/info":
            if message['message'] == "Allow Device" and message['uuid'] == uuid:
                self.response_client.publish("device/connect/info", payload=device_info_json, qos=0)
                self.is_connect = True
        elif topic == "server/add/info":
            if message['message'] == "Add Device" and message['uuid'] == uuid:
                self.response_client.publish("device/connect/info", payload=device_info_json, qos=0)
                self.is_connect = True
        elif topic == "server/delete/info":
            if message['message'] == "Delete Device" and message['uuid'] == uuid:
                print("Disconnecting...")
                self.response_client.publish("device/disconnect/uuid", payload=device_info_json, qos=0)
                self.is_connect = False
                client_exit = True
        elif topic == "server/update/config":
            script_text = message['script_text']
            filename = "script_text.sh"
            with open(filename, 'w') as outfile:
                outfile.write(script_text)
            outfile.close()
            os.system("./{}".format(filename))
        elif topic == "server/update/info":
            if message['message'] == "Update Device" and message['uuid'] == uuid:
                # update()
                self.response_client.publish("device/update/info", payload=device_info_json, qos=0)
        elif topic == "server/reset/info":
            if message['message'] == "Reset Device" and message['uuid'] == uuid:
                self.init_config()
                self.response_client.publish("device/reset/info", payload=device_info_json, qos=0)
        elif topic == "server/config/style":
            if message['message'] == "Device style" and message['uuid'] == uuid:
                #TODO: check command type
                if message['conti'] == "True":
                    self.conti = True
                elif message['conti'] == "False":
                    self.conti = False
                self.response_client.publish("device/config/style", payload=device_info_json, qos=0)
        elif topic == "server/config/freq":
            if message['message'] == "Device freq" and message['uuid'] == uuid:
                #TODO: check command type
                self.freq_DHT = int(message['freq_DHT'])
                self.freq_light = int(message['freq_light'])
                self.response_client.publish("device/config/style", payload=device_info_json, qos=0)
