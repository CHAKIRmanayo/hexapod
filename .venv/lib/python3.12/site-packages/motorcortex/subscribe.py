#!/usr/bin/python3

#
#   Developer : Alexey Zakharov (alexey.zakharov@vectioneer.com)
#   All rights reserved. Copyright (c) 2016 VECTIONEER.
#

from motorcortex.request import Request, Reply, ConnectionState
from motorcortex.subscription import Subscription
from pynng import Sub0, TLSConfig
from concurrent.futures import ThreadPoolExecutor
from queue import Queue


class Subscribe:
    """Subscribe class is used to receive continuous parameter updates from motorcortex server.

        Subscribe class simplifies creating and removing subscription groups.

        Args:
            req(Request): reference to a Request instance
            protobuf_types(MessageTypes): reference to a MessageTypes instance

    """

    def __init__(self, req, protobuf_types):
        self.__socket = None
        self.__connected_lock = None
        self.__url = None
        self.__req = req
        self.__protobuf_types = protobuf_types
        self.__subscriptions = dict()
        self.__pool = ThreadPoolExecutor()
        self.__clb = ThreadPoolExecutor(max_workers=1)
        self.__connection_state = ConnectionState.DISCONNECTED

    def connect(self, url, **kwargs):
        """Open a subscribe connection.

            Args:
                url(str): motorcortex server URL

            Returns:
                bool: True - if connected, False othesudo apt-get install python-devrwise
        """

        self.__connection_state = ConnectionState.CONNECTING
        conn_timeout_ms, recv_timeout_ms, certificate, state_update = Request.parse(**kwargs)
        if recv_timeout_ms == 0:
            recv_timeout_ms = 500

        self.__url = url
        tls_config = None
        if certificate:
            tls_config = TLSConfig(TLSConfig.MODE_CLIENT, ca_files=certificate)

        self.__socket = Sub0(recv_timeout=recv_timeout_ms, tls_config=tls_config)

        self.__connected_lock = Queue()

        def pre_connect_cb(pipe):
            self.__connected_lock.put(True)
            self.__connection_state = ConnectionState.CONNECTION_OK
            if state_update:
                self.__clb.submit(state_update, self.__req, self, self.connectionState())

        def post_remove_cb(pipe):
            if self.__connection_state == ConnectionState.DISCONNECTING:
                self.__connection_state = ConnectionState.DISCONNECTED
            elif self.__connection_state == ConnectionState.CONNECTING:
                self.__connection_state = ConnectionState.CONNECTION_FAILED
            elif self.__connection_state == ConnectionState.CONNECTION_OK:
                self.__connection_state = ConnectionState.CONNECTION_LOST
            self.__connected_lock.put(False)
            if state_update:
                self.__clb.submit(state_update, self.__req, self, self.connectionState())

        self.__socket.add_pre_pipe_connect_cb(pre_connect_cb)
        self.__socket.add_post_pipe_remove_cb(post_remove_cb)
        self.__socket.dial(url, block=False)

        self.__pool.submit(self.run, self.__socket)

        return Reply(self.__pool.submit(Request.waitForConnection, self.__connected_lock,
                                        conn_timeout_ms / 1000.0))

    def close(self):
        """Close connection to the server"""
        self.__connection_state = ConnectionState.DISCONNECTING
        if self.__connected_lock:
            self.__connected_lock.put(False)
        self.__socket.close()
        self.__pool.shutdown(wait=True)

    def run(self, socket):
        while self.__socket:
            buffer = socket.recv()
            if buffer:
                sub_id_buf = buffer[:4]
                protocol_version = sub_id_buf[3]
                sub_id = sub_id_buf[0] + (sub_id_buf[1] << 8) + (sub_id_buf[2] << 16)
                sub = self.__subscriptions.get(sub_id)
                if sub:
                    length = len(buffer)
                    if protocol_version == 1:
                        sub._updateProtocol1(buffer[4:], length - 4)
                    elif protocol_version == 0:
                        sub._updateProtocol0(buffer[4:], length - 4)
                    else:
                        print('Unknown protocol type: %d', protocol_version)

        print('Subscribe connection closed')

    def subscribe(self, param_list, group_alias, frq_divider=1):
        """Create a subscription group for a list of the parameters.

            Args:
                param_list(list(str)): list of the parameters to subscribe to
                group_alias(str): name of the group
                frq_divider(int): frequency divider is a downscaling factor for the group publish rate

            Returns:
                  Subscription: A subscription handle, which acts as a JavaScript Promise,
                  it is resolved when subscription is ready or failed. After the subscription
                  is ready the handle is used to retrieve latest data.
        """

        subscription = Subscription(group_alias, self.__protobuf_types, frq_divider)
        reply = self.__req.createGroup(param_list, group_alias, frq_divider)
        reply.then(self.__complete, subscription, self.__socket).catch(
            subscription._failed)

        return subscription

    def unsubscribe(self, subscription):
        """Unsubscribe from the group.

            Args:
                subscription(Subscription): subscription handle

            Returns:
                  Reply: Returns a Promise, which resolves when the unsubscribe
                  operation is complete, fails otherwise.

        """
        sub_id = subscription.id()
        sub_id_buf = Subscribe.__idBuf(subscription.id())

        # stop receiving sub
        try:
            self.__socket.unsubscribe(sub_id_buf)
        except:
            pass

        # find and remove subscription
        if sub_id in self.__subscriptions:
            sub = self.__subscriptions[sub_id]
            # stop sub update thread
            sub.done()
            del self.__subscriptions[sub_id]

        # send remove group request to the server
        return self.__req.removeGroup(subscription.alias())

    def connectionState(self):
        return self.__connection_state

    def resubscribe(self):
        old_sub = self.__subscriptions.copy()
        self.__subscriptions.clear()
        for i in old_sub:
            s = old_sub[i]
            try:
                # unsubscribe from the old group
                self.__socket.unsubscribe(Subscribe.__idBuf(s.id()))
            except:
                pass
            # subscribe again, update id
            msg = self.__req.createGroup(s.layout(), s.alias(), s.frqDivider()).get()
            s._updateId(msg.id)
            self.__socket.subscribe(Subscribe.__idBuf(s.id()))
            self.__subscriptions[s.id()] = s

    @staticmethod
    def __idBuf(msg_id):
        return bytes([msg_id & 0xff, (msg_id >> 8) & 0xff, (msg_id >> 16) & 0xff])

    def __complete(self, msg, subscription, socket):
        if subscription._complete(msg):
            id_buf = Subscribe.__idBuf(msg.id)
            socket.subscribe(id_buf)
            self.__subscriptions[msg.id] = subscription
