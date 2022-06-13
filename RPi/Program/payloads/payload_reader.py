class PayloadReader:

    def read_payload(self, received_data):
        """
        extracts data from message received
        :param received_data: data received from zmq
        :return: the data type and the data of the payload
        """
        payload_type = received_data['payload_name']
        data = received_data['payload_data']
        keys = []
        values = []
        for k in data:
            size = len(k)
            for key, value in k.items():
                keys.append(key)
                values.append(value)
        return payload_type, keys, values
