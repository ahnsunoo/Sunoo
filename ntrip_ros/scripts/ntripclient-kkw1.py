#!/usr/bin/python
#-*- coding: utf-8 -*-

from datetime import datetime
import sys
import os

from base64 import b64encode
from threading import Thread, Event

from httplib import HTTPConnection
from httplib import IncompleteRead

''' This is to fix the IncompleteRead error
    http://bobrochel.blogspot.com/2010/11/bad-servers-chunked-encoding-and.html'''
import httplib
import signal




def patch_http_response_read(func):
    def inner(*args):
        try:
            return func(*args)
        except httplib.IncompleteRead, e:
            return e.partial
    return inner

httplib.HTTPResponse.read = patch_http_response_read(httplib.HTTPResponse.read)

class ntripconnect(Thread):
    def __init__(self, ntc):
        super(ntripconnect, self).__init__()
        self.ntc = ntc
        self.stop = False
        self.fifo_name = "my_pipe"
    def run(self):

        headers = {
            'Ntrip-Version': 'Ntrip/2.0',
            'User-Agent': 'NTRIP ntrip_ros',
            'Connection': 'close',
            'Authorization': 'Basic ' + b64encode(self.ntc.ntrip_user + ':' + str(self.ntc.ntrip_pass))
        }
        connection = HTTPConnection(self.ntc.ntrip_server)
        connection.request('GET', '/'+self.ntc.ntrip_stream, self.ntc.nmea_gga, headers)
        response = connection.getresponse()
        if response.status != 200: raise Exception("blah")
        buf = ""
        #rmsg = Message()
        restart_count = 0
                
        while not self.stop:
            
            ''' This now separates individual RTCM messages and publishes each one on the same topic '''
            data = response.read(1)
            
            if len(data) != 0:
                if ord(data[0]) == 211:
                    buf += data
                    data = response.read(2)
                    buf += data
                    cnt = ord(data[0]) * 256 + ord(data[1])
                    data = response.read(2)
                    buf += data
                    typ = (ord(data[0]) * 256 + ord(data[1])) / 16
                    #print (str(datetime.now()), cnt, typ)
                    cnt = cnt + 1
                    for x in range(cnt):
                        data = response.read(1)
                        buf += data
                    print(len(buf)  ) 
                    #print(type(buf))
                    with open(self.fifo_name, "wb") as pipe:
						data = buf
						pipe.write(data)    
                    buf = ""
                    
                else: 
					print(data)
                
            else:
                ''' If zero length data, close connection and reopen it '''
                restart_count = restart_count + 1
                print("Zero length ", restart_count)
                connection.close()
                connection = HTTPConnection(self.ntc.ntrip_server)
                connection.request('GET', '/'+self.ntc.ntrip_stream, self.ntc.nmea_gga, headers)
                response = connection.getresponse()
                if response.status != 200: raise Exception("blah")
                buf = ""
        print("Connection Close!")
        connection.close()

    
class ntripclient:
	def __init__(self):

		self.ntrip_server       = 'RTS1.ngii.go.kr:2101'
		self.ntrip_user         = 'ICASjuni1'
		self.ntrip_pass         = 'ngii'
		self.ntrip_stream       = 'VRS-RTCM31'
		self.nmea_gga           = '$GPGGA,124200.186,3549.793,N,12845.307,E,1,12,1.0,0.0,M,0.0,M,,*6C'

		#self.pub = rospy.Publisher(self.rtcm_topic, Message, queue_size=10)

		self.connection = None
		self.connection = ntripconnect(self)
		self.connection.stop = False
		self.connection.start()
	
	def run(self):
		while True:
			pass

def signal_handler(signal, frame):
    print("Ctrl+C exit.")
    c.connection.stop = True
    c.connection.join()
    sys.exit(0) 
	
if __name__ == '__main__':
	
	c = ntripclient()
	signal.signal(signal.SIGINT, signal_handler)
	c.run()
	
    

