import roslibpy


class RosInterface:

    def __init__(self, host):
        try:
            self.client = roslibpy.Ros(host=host, port=9091)
            self.client.run()
            print('Connected')
        except Exception as e:
            print('WebSocket Error: ' + str(e))

    def publish(self, topic, msg_type, value):
        if self.isConnected():
            talker = roslibpy.Topic(self.client, topic, msg_type)
            talker.publish(roslibpy.Message({'data': value}))
            print('Sending message...')
            talker.unadvertise()
        else:
            print('The client is not connected to the server')

    def isConnected(self):
        return self.client.is_connected

    def closeConnection(self):
        if self.isConnected():
            self.client.terminate()
        else:
            print('Connection was already closed')
