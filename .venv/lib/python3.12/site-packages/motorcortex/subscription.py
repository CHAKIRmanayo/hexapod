#!/usr/bin/python3

#
#   Developer : Alexey Zakharov (alexey.zakharov@vectioneer.com)
#   All rights reserved. Copyright (c) 2016 VECTIONEER.
#

from collections import namedtuple
from struct import unpack_from
from concurrent.futures import Future, ThreadPoolExecutor
import time

timespec = namedtuple('timespec', 'sec, nsec')
"""Timestamp of the parameter 

Args:
    sec(int): seconds
    nsec(int): nanoseconds
"""

Parameter = namedtuple('Parameter', 'timestamp, value')
"""Parameter value with a timestamp

Args:
    timestamp(timespec): Parameter timestamp
    value(any): Parameter value

"""


def compare_timespec(timestamp1, timestamp2):
    return True if (timestamp1.sec == timestamp2.sec) and (timestamp1.nsec == timestamp2.nsec) else False


"""Compare two timestamps

Args:
    timestamp1(timespec): First timestamp to compare
    timestamp2(timespec): Second timestamp to compare

"""


def timespec_to_sec(timestamp):
    return timestamp.sec + timestamp.nsec / 1000000000.0


"""Convert a timestamp to seconds

Args:
    timestamp(timespec): Timestamp to convert
    
Returns:
    Time in seconds
    
"""


def timespec_to_msec(timestamp):
    return timestamp.sec * 1000 + timestamp.nsec / 1000000


"""Convert a timestamp to milliseconds

Args:
    timestamp(timestamp): Timestamp to convert
    
Returns:
    Time in milliseconds

"""


def timespec_to_usec(timestamp):
    return timestamp.sec * 1000000 + timestamp.nsec / 1000


"""Convert a timestamp to microseconds

Args:
    timestamp(timestamp): Timestamp to convert
    
Returns:
    Time in microseconds

"""


def timespec_to_nsec(timestamp):
    return timestamp.sec * 1000000000 + timestamp.nsec


"""Convert a timestamp to nanoseconds

Args:
    timestamp(timestamp): Timestamp to convert
    
Returns:
    Time in nanoseconds

"""


class Subscription(object):
    """Subscription class represents a group of parameters.

        It returns the latest values and a timestamp of the group.

        Subscription class could be used as an observer, which
        notifies on every update or could be used as polling.
    """

    def __init__(self, group_alias, protobuf_types, frq_divider):
        self.__info = None
        self.__group_alias = group_alias
        self.__protobuf_types = protobuf_types
        self.__decoder = []
        self.__clb_list = []
        self.__future = Future()
        self.__values = None
        self.__layout = None
        self.__is_complete = False
        self.__observer_list = []
        self.__pool = None
        self.__frq_divider = frq_divider

    def id(self):
        """
            Returns:
                int: subscription identifier
        """
        return self.__info.id

    def alias(self):
        """
            Returns:
                str: group alias
        """
        return self.__group_alias

    def frqDivider(self):
        """
            Returns:
                str: frequency divider of the group
        """
        return self.__frq_divider

    def read(self):
        """Read the latest values of the parameters in the group.

            Returns:
                list(Parameter): list of parameters
        """
        return self.__values[:] if self.__is_complete else None

    def layout(self):
        """Get a layout of the group.

            Returns:
                list(str): ordered list of the parameters in the group
        """
        return self.__layout[:] if self.__is_complete else None

    def done(self):
        """
            Returns:
                bool: True if the call was successfully cancelled or finished running.

            Examples:
                >>> subscription = sub.subscribe("root/logger/logOut", "log")
                >>> while not subscription.done():
                >>>     time.sleep(0.1)
        """
        return self.__future.done()

    def get(self, timeout_sec=1.0):
        """
            Returns:
                bool: StatusMsg if the call was successfully, None if timeout happened.

            Examples:
                >>> subscription = sub.subscribe("root/logger/logOut", "log")
                >>> done = subscription.get()
        """

        return self.__future.result(timeout_sec)

    def then(self, subscribed_clb):
        """JavaScript-like promise, which is resolved when subscription is completed.

            Args:
                subscribed_clb: callback which is resolved when subscription is completed.

            Returns:
                self pointer to add 'catch' callback

            Examples:
                >>> subscription = sub.subscribe("root/logger/logOut", "log")
                >>> subscription.then(lambda val: print("got: %s"%val)).catch(lambda d: print("failed"))
        """

        self.__future.add_done_callback(
            lambda msg: subscribed_clb(msg.result()) if msg.result() else None
        )
        return self

    def catch(self, failed):
        """JavaScript-like promise, which is resolved when subscription has failed.

            Args:
                failed: callback which is resolved when subscription has failed

            Returns:
                self pointer to add 'then' callback

            Examples:
                >>> subscription = sub.subscribe("root/logger/logOut", "log")
                >>> subscription.catch(lambda d: print("failed")).then(lambda val: print("got: %s"%val))
        """

        self.__future.add_done_callback(
            lambda msg: failed() if not msg.result() else None
        )
        return self

    def notify(self, observer_list):
        """Set an observer, which is notified on every group update.

            Args:
                observer_list: a callback function (or list of callback functions)
                to notify when new values are available

            Examples:
                  >>> def update(parameters):
                  >>>   print(parameters) #list of Parameter tuples

                  >>> data_sub.notify(update)

        """
        self.__observer_list = observer_list if type(observer_list) is list else [observer_list]
        self.__pool = ThreadPoolExecutor(len(self.__observer_list))

    def _complete(self, msg):
        self.__decoder = []
        self.__values = []
        self.__layout = []
        if msg.status == 0:
            self.__info = msg
            for param in msg.params:
                data_type = param.info.data_type
                self.__decoder.append(self.__protobuf_types.getTypeByHash(data_type))
                self.__values.append(Parameter(timespec(0, 0), [0] * param.info.number_of_elements))
                self.__layout.append(param.info.path)

            self.__is_complete = True
            self.__future.set_result(msg)
            return True
        else:
            self._failed()

        return False

    def _updateId(self, new_id):
        self.__info.id = new_id

    def _updateProtocol0(self, sub_msg, length):
        if sub_msg:
            counter = 0
            n_params = len(self.__info.params)
            for param in self.__info.params:
                offset = param.offset
                size = param.size

                # the last element in the group may have a variable size
                if ((counter + 1) == n_params) and ((offset + size) > length):
                    size = length - offset

                if (offset + size) <= length:
                    timestamp = timespec._make(unpack_from('QQ', sub_msg, offset))
                    value = self.__decoder[counter].decode(sub_msg[offset + 16: offset + size],
                                                           (size - 16) / param.info.data_size)
                    self.__values[counter] = Parameter(timestamp, value)

                counter += 1

            if self.__observer_list:
                value = self.__values[:]
                for observer in self.__observer_list:
                    self.__pool.submit(observer, value)

    def _updateProtocol1(self, sub_msg, length):
        if sub_msg:
            timestamp = timespec._make(unpack_from('QQ', sub_msg, 0))
            counter = 0
            n_params = len(self.__info.params)
            for param in self.__info.params:
                offset = param.offset + 16
                size = param.size

                # the last element in the group may have a variable size
                if ((counter + 1) == n_params) and ((offset + size) > length):
                    size = length - offset

                if (offset + size) <= length:
                    value = self.__decoder[counter].decode(sub_msg[offset: offset + size],
                                                           size / param.info.data_size)
                    self.__values[counter] = Parameter(timestamp, value)

                counter += 1

            if self.__observer_list:
                value = self.__values[:]
                for observer in self.__observer_list:
                    self.__pool.submit(observer, value)

    def _failed(self):
        self.__future.set_result(None)
