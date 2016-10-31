import sys
import requests
#from jsonrpc import JSONRPCResponseManager, dispatcher
import json
import socket
import time

keyList = [ '37', '38', '39', '40', '41', '42',     'auto', 'lock', 'pc' , 'quit' ,
            '31', '32', '33', '34', '35', '36',     'peel', 'f1'  , 'plu', 'clear',  
            '25', '26', '27', '28', '29', '30',     'f2'  , 'f3'  , 'f4' , 'func' ,
            '19', '20', '21', '22', '23', '24',     'num7', 'num8', 'num9', 'zero',
            '13', '14', '15', '16', '17', '18',     'num4', 'num5', 'num6', 'count',
            '7',  '8' , '9' , '10', '11', '12',     'num1', 'num2', 'num3', 'ok',
            '1',  '2' , '3' ,  '4', '5' , '6' ,     'num0', 'dot' , 'paper', 'print',
        ]

def PRT_DBG(msg):
    print 'dbg:', msg

class ScaleTestLib:
    url     = ""
    port    = 0
    def setURL(self, url, port):
        self.url = url
        self.port = int(port)

    def displayURL(self):
        req = (self.url, self.port)
        PRT_DBG(req)

    def testHello(self):
        # Example echo method
        if self.url == "" or self.port <= 0:
            PRT_DBG("url or port is null,please check!!!")
            return 
        addr=(self.url, self.port)
        payload = {
            "method": "sayHello",
            "params": ["hello server!"],
            "jsonrpc":"2.0",
            "id": 0,
        }
        jsonReq = json.dumps(payload)
        sock = socket.socket()
        sock.settimeout(4)
        sock.connect(addr)
        sock.send(jsonReq)
        resp = json.loads(sock.recv(1024))
        sock.close()
        PRT_DBG(resp)

    def checkeyName(self, keyName):

        if keyList.count(keyName) == 0:
            return False
        return True

    def testSleep(self, s):
        time.sleep(s)

    def pressKey(self, keyName):

        if self.url == "" or self.port <= 0:
            PRT_DBG("url or port is null,please check!!!")
            return 
        if False == self.checkeyName(keyName):
            PRT_DBG('key name is not exist')
            return False

        addr=(self.url, self.port)
        payload = {
            "method": "pressKey",
            "params": [keyName], #["hello server!"],
            "jsonrpc":"2.0",
            "id": 0,
        }
        jsonReq = json.dumps(payload)
        sock = socket.socket()
        sock.settimeout(4)
        sock.connect(addr)
        sock.send(jsonReq)
        resp = json.loads(sock.recv(1024))
        sock.close()
        PRT_DBG( resp)
        return True
        
    def prtfMessage(self, message):
        PRT_DBG (message)


#instance = ScaleTestLib();
#instance.setURL('192.168.27.1', 1234)
