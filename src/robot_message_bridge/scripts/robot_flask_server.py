import flask
import json
from gevent import pywsgi


from flask_executor import Executor
from robot_common_log import Log
from functools import partial

logger = Log(__name__, "flask").getlog()


class RobotFlaskSever:
    def __init__(self, dms_callback, nav_lock_callback,status_alarm_msg,port=3030, debug=False, host='192.168.1.146'):
        self.robot_api = flask.Flask(__name__)
        self.executor = Executor(self.robot_api)
        self.robot_port = port
        self.robot_debug = debug
        self.robot_host = host
        self.dms_callback = dms_callback
        self.nav_lock_callback = nav_lock_callback
        self.status_alarm_msg=status_alarm_msg


    def RunSever(self):

        self.robot_api.route('/', methods=['post'])(self.DmsPost)
        self.robot_api.route('/health', methods=['get'])(self.StatusAlarm)
        self.robot_api.run(port=self.robot_port,
                           debug=self.robot_debug, host=self.robot_host)

        # server = pywsgi.WSGIServer(
        #     (self.robot_host, self.robot_port), self.robot_api)
        # server.serve_forever()

    def StatusAlarm(self):
        tmp=self.status_alarm_msg()
        electricity=tmp["electricityQuantity"]
        status=tmp["status"]
        msg={}
        if "ERROR"==status and electricity<19 or electricity<15:
            msg["code"]=500
            msg["electricity"]=electricity
            msg["status"]=status
            msg["alarm"]=True
        else:
            msg["code"]=200
            msg["electricity"]=electricity
            msg["status"]=status
            msg["alarm"]=False

        return json.dumps(msg),msg["code"],[("Content-type", "application/json")]


    def DmsPost(self):
        try:
            logger.info("收到请求")
            datas = flask.request.get_data().decode('utf-8')
            datas = json.loads(datas)  # json字符串转为字典类型
            logger.info("收到指令%r", datas)

            if 'operations' in datas or 'query' in datas:
                response = json.dumps(
                    {"msg": "received successfully", "code": 200}).encode('utf-8')
                call_back = partial(self.dms_callback, datas)
                self.executor.submit(call_back)
                return response, 200, [("Content-type", "application/json")]
            elif"nav" in datas:
                self.nav_lock_callback(datas)
                response = json.dumps(
                    {"msg": "received successfully", "code": 200}).encode('utf-8')
                return response, 200, [("Content-type", "application/json")]

            else:
                response = json.dumps(
                    {"msg": "received successfully,but operate code error", "code": 200}).encode('utf-8')
                print("not normal")
                return response, 500, [("Content-type", "application/json")]

        except Exception as e:
            logger.info("请求异常")
            response = json.dumps(
                {"msg": "some error happen: " + str(e), "code": 500}).encode('utf-8')
            return response, 500, [("Content-type", "application/json")]


if __name__ == '__main__':
    data = {}

    def dms_callback():
        pass
    robot_flask_sever = RobotFlaskSever(data, dms_callback)
    robot_flask_sever.RunSever()


# def dms_callback():
#     pass
# post_data={}
# app = flask.Flask(__name__)
# executor=Executor(app)
# @app.route("/",methods=["post"])
# def hello():
#     try:
#         datas = flask.request.get_data().decode('utf-8')
#         datas = json.loads(datas)  # json字符串转为字典类型
#         # for data_key, data_value in datas.items():
#         #     self.post_data[data_key] = data_value

#         # print("received", self.post_data)
#         # logger.info("收到指令%r", self.post_data)

#         # logger.info("收到指令%r", datas)

#         if 'operations' in datas or 'query' in datas:
#             for data_key, data_value in datas.items():
#                 post_data[data_key] = data_value
#             response = json.dumps(
#                 {"msg": "received successfully", "code": 200}).encode('utf-8')
#             executor.submit(dms_callback)
#         else:
#             response = json.dumps(
#                 {"msg": "received successfully,but operate code error", "code": 200}).encode('utf-8')
#             print("not normal")

#         return response, 200, [("Content-type", "application/json")]

#     except Exception as e:
#         print(e)
#         response = json.dumps(
#             {"msg": "some error happen: " + str(e), "code": 500}).encode('utf-8')
#         return response, 200, [("Content-type", "application/json")]

# if __name__ == '__main__':
#     server = pywsgi.WSGIServer(
#             ("0.0.0.0", 3030), app)
#     server.serve_forever()
    # app.run(port=3030,debug=False)
